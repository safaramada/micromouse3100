"""
mask_maze.py

Creates a binary obstacle mask for the supplied octagonal maze image.

Output convention:
- occupancy_obstacles_white.png:
    255 (white) = obstacle / wall / outside board
    0   (black) = free space

- planning_map_free_white.png:
    255 (white) = free space
    0   (black) = obstacle / wall / outside board

Designed for the maze image shown in the MTRN3100 path-planning task.
The octagonal board polygon is defined using image-relative coordinates,
so the script still works if the image is resized without changing its shape.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


# ------------------------- Parameters to tune ------------------------- #

# Pixels darker than this are initially treated as possible wall pixels.
DARK_THRESHOLD = 110

# Morphological closing joins wall pieces interrupted by the cyan tape.
CLOSE_KERNEL_SIZE = 3
CLOSE_ITERATIONS = 1

# Removes small dark squares, screws, and isolated noise.
MIN_COMPONENT_AREA = 100

# Adds a narrow obstacle boundary around the octagonal board.
BOARD_BOUNDARY_WIDTH = 7

# Approximate inner octagonal board corners, stored as fractions of
# image width and height.
BOARD_POLYGON_FRACTIONS = np.array(
    [
        (0.170, 0.045),  # top-left sloping corner
        (0.820, 0.045),  # top-right sloping corner
        (0.965, 0.190),  # upper-right side
        (0.965, 0.810),  # lower-right side
        (0.820, 0.955),  # bottom-right sloping corner
        (0.180, 0.955),  # bottom-left sloping corner
        (0.035, 0.810),  # lower-left side
        (0.035, 0.190),  # upper-left side
    ],
    dtype=np.float32,
)


def odd_positive(value: int, name: str) -> int:
    """Validate a positive odd kernel size."""
    if value <= 0 or value % 2 == 0:
        raise ValueError(f"{name} must be a positive odd integer.")
    return value


def create_board_mask(height: int, width: int) -> tuple[np.ndarray, np.ndarray]:
    """
    Return:
        board_mask: 255 inside the octagonal board and 0 outside
        polygon: integer polygon vertices
    """
    polygon = np.empty_like(BOARD_POLYGON_FRACTIONS, dtype=np.int32)
    polygon[:, 0] = np.rint(BOARD_POLYGON_FRACTIONS[:, 0] * width).astype(np.int32)
    polygon[:, 1] = np.rint(BOARD_POLYGON_FRACTIONS[:, 1] * height).astype(np.int32)

    board_mask = np.zeros((height, width), dtype=np.uint8)
    cv2.fillPoly(board_mask, [polygon], 255)

    return board_mask, polygon


def remove_small_components(binary_mask: np.ndarray, minimum_area: int) -> np.ndarray:
    """Keep only connected white regions whose area is large enough."""
    component_count, labels, statistics, _ = cv2.connectedComponentsWithStats(
        binary_mask,
        connectivity=8,
    )

    cleaned = np.zeros_like(binary_mask)

    for component_id in range(1, component_count):
        area = statistics[component_id, cv2.CC_STAT_AREA]

        if area >= minimum_area:
            cleaned[labels == component_id] = 255

    return cleaned


def create_maze_masks(
    image: np.ndarray,
    dark_threshold: int = DARK_THRESHOLD,
) -> dict[str, np.ndarray]:
    """Create the intermediate and final masks."""
    if image is None or image.size == 0:
        raise ValueError("The supplied image is empty or could not be read.")

    height, width = image.shape[:2]
    board_mask, board_polygon = create_board_mask(height, width)

    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect dark wall pixels.
    raw_dark_mask = cv2.inRange(grayscale, 0, dark_threshold)

    # Ignore the metal frame and objects outside the playable board.
    raw_dark_mask = cv2.bitwise_and(raw_dark_mask, board_mask)

    close_size = odd_positive(CLOSE_KERNEL_SIZE, "CLOSE_KERNEL_SIZE")
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size),
    )

    # Join gaps across cyan tape and small lighting discontinuities.
    closed_wall_mask = cv2.morphologyEx(
        raw_dark_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=CLOSE_ITERATIONS,
    )

    cleaned_wall_mask = remove_small_components(
        closed_wall_mask,
        MIN_COMPONENT_AREA,
    )

    # Perform one final small closing pass.
    cleaned_wall_mask = cv2.morphologyEx(
        cleaned_wall_mask,
        cv2.MORPH_CLOSE,
        np.ones((5, 5), dtype=np.uint8),
        iterations=1,
    )

    # Occupancy convention: white means blocked.
    occupancy_map = cleaned_wall_mask.copy()

    # Everything outside the octagonal board is blocked.
    occupancy_map[board_mask == 0] = 255

    # Add a clear boundary around the board edge.
    boundary_kernel = np.ones(
        (BOARD_BOUNDARY_WIDTH, BOARD_BOUNDARY_WIDTH),
        dtype=np.uint8,
    )
    board_boundary = cv2.morphologyEx(
        board_mask,
        cv2.MORPH_GRADIENT,
        boundary_kernel,
    )
    occupancy_map[board_boundary > 0] = 255

    # The opposite convention is often convenient for graph construction.
    planning_map = cv2.bitwise_not(occupancy_map)

    # Red overlay showing what has been classified as blocked.
    coloured_overlay = image.copy()
    coloured_overlay[occupancy_map == 255] = (0, 0, 255)
    overlay = cv2.addWeighted(image, 0.68, coloured_overlay, 0.32, 0)

    # Draw the detected board polygon for checking.
    polygon_preview = image.copy()
    cv2.polylines(
        polygon_preview,
        [board_polygon],
        isClosed=True,
        color=(0, 255, 255),
        thickness=2,
    )

    return {
        "01_grayscale.png": grayscale,
        "02_board_polygon.png": polygon_preview,
        "03_raw_dark_mask.png": raw_dark_mask,
        "04_closed_wall_mask.png": closed_wall_mask,
        "05_cleaned_wall_mask.png": cleaned_wall_mask,
        "06_occupancy_obstacles_white.png": occupancy_map,
        "07_planning_map_free_white.png": planning_map,
        "08_mask_overlay.png": overlay,
    }


def save_outputs(outputs: dict[str, np.ndarray], output_directory: Path) -> None:
    """Save every generated image and check for write errors."""
    output_directory.mkdir(parents=True, exist_ok=True)

    for filename, output_image in outputs.items():
        output_path = output_directory / filename

        if not cv2.imwrite(str(output_path), output_image):
            raise OSError(f"Could not save output image: {output_path}")


def show_outputs(outputs: dict[str, np.ndarray]) -> None:
    """Display the most useful masks until a key is pressed."""
    cv2.imshow("Original masking overlay", outputs["08_mask_overlay.png"])
    cv2.imshow(
        "Occupancy: white = obstacle",
        outputs["06_occupancy_obstacles_white.png"],
    )
    cv2.imshow(
        "Planning map: white = free",
        outputs["07_planning_map_free_white.png"],
    )

    print("Press any key in an image window to close the previews.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a binary obstacle mask for the octagonal maze.",
    )

    parser.add_argument(
        "image",
        type=Path,
        help="Path to the maze image.",
    )

    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help=(
            "Output folder. By default, a '<image-name>_mask_outputs' "
            "folder is created beside the input image."
        ),
    )

    parser.add_argument(
        "--threshold",
        type=int,
        default=DARK_THRESHOLD,
        help=f"Dark-pixel threshold from 0 to 255. Default: {DARK_THRESHOLD}.",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Open preview windows after saving the masks.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    if not 0 <= args.threshold <= 255:
        raise ValueError("--threshold must be between 0 and 255.")

    image_path = args.image.expanduser().resolve()

    if not image_path.is_file():
        raise FileNotFoundError(f"Image file not found: {image_path}")

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)

    if image is None:
        raise ValueError(
            f"OpenCV could not read '{image_path}'. "
            "Check that it is a valid PNG or JPG image."
        )

    if args.output is None:
        output_directory = image_path.parent / f"{image_path.stem}_mask_outputs"
    else:
        output_directory = args.output.expanduser().resolve()

    outputs = create_maze_masks(
        image=image,
        dark_threshold=args.threshold,
    )

    save_outputs(outputs, output_directory)

    print(f"Masking complete. Outputs saved to:\n{output_directory}")
    print("\nUse this file for path planning:")
    print(output_directory / "07_planning_map_free_white.png")
    print("\nPlanning-map convention:")
    print("  255 = free space")
    print("    0 = obstacle or outside the board")

    if args.show:
        show_outputs(outputs)


if __name__ == "__main__":
    main()
