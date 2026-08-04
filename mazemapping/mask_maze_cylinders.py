"""
mask_maze.py

Creates a binary obstacle mask for the supplied octagonal maze image.

Output convention:
- 06_occupancy_obstacles_white.png:
    255 (white) = obstacle / wall / outside board
    0   (black) = free space

- 07_planning_map_free_white.png:
    255 (white) = free space
    0   (black) = obstacle / wall / outside board

The preprocessing uses:
1. CLAHE local contrast enhancement
2. Gamma darkening
3. A normal dark-pixel threshold for thick walls
4. Horizontal and vertical black-hat filters for thin walls
5. Morphological closing to repair small gaps
6. Connected-component filtering that preserves long, thin walls
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


# ------------------------- Parameters to tune ------------------------- #

# Main wall threshold. Lower this if shadows are being detected as walls.
# Raise it slightly if genuine dark walls are missing.
DARK_THRESHOLD = 92

# Local contrast enhancement.
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID = (8, 8)

# Gamma > 1 darkens mid-grey/dark pixels.
GAMMA = 1.21

# Threshold applied to the black-hat response used for thin-wall recovery.
# Lower it if thin walls are missing; raise it if too many fine details appear.
THIN_LINE_THRESHOLD = 18

# Black-hat kernels for detecting horizontal and vertical dark lines.
HORIZONTAL_BLACKHAT_KERNEL = (17, 3)
VERTICAL_BLACKHAT_KERNEL = (3, 17)

# Directional opening suppresses non-line-shaped black-hat detections.
HORIZONTAL_OPEN_KERNEL = (7, 1)
VERTICAL_OPEN_KERNEL = (1, 7)

# Morphological closing joins wall pieces interrupted by cyan tape.
CLOSE_KERNEL_SIZE = 3
CLOSE_ITERATIONS = 1

# Large connected regions are kept automatically.
MIN_COMPONENT_AREA = 100

# Long, thin regions can also be kept even when their area is small.
MIN_COMPONENT_LENGTH = 15
MIN_THIN_COMPONENT_AREA = 8

# The photographed board has faint centre panel seams that are not obstacles.
# This removes thin-line-only detections on those seams while preserving any
# genuinely dark wall pixels found by the main threshold.
SUPPRESS_CENTRE_SEAMS = True
CENTRE_SEAM_HALF_WIDTH = 7

# Adds a narrow obstacle boundary around the octagonal board.
BOARD_BOUNDARY_WIDTH = 7

# ----------------------- Cylinder detection tuning ----------------------- #

# Cylinders are detected as isolated, approximately circular dark components.
# The small square floor markers are rejected mainly by the minimum area/radius.
CYLINDER_MIN_AREA = 250
CYLINDER_MAX_AREA = 6000
CYLINDER_MIN_RADIUS = 12
CYLINDER_MAX_RADIUS = 45
CYLINDER_MIN_ASPECT_RATIO = 0.65
CYLINDER_MAX_ASPECT_RATIO = 1.35
CYLINDER_MIN_CIRCULARITY = 0.42
CYLINDER_MIN_FILL_RATIO = 0.45

# Closing makes the dark circular top of each cylinder more continuous.
CYLINDER_CLOSE_KERNEL_SIZE = 7
CYLINDER_CLOSE_ITERATIONS = 1

# Extra blocked distance around each detected cylinder.
# Set this to the robot radius in pixels plus a small safety margin.
CYLINDER_BUFFER_PIXELS = 10

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


def apply_gamma(image: np.ndarray, gamma: float) -> np.ndarray:
    """Apply gamma correction. Gamma greater than 1 darkens the image."""
    if gamma <= 0:
        raise ValueError("Gamma must be greater than zero.")

    lookup_table = np.array(
        [
            np.clip(255.0 * ((pixel / 255.0) ** gamma), 0, 255)
            for pixel in range(256)
        ],
        dtype=np.uint8,
    )

    return cv2.LUT(image, lookup_table)


def remove_small_components(
    binary_mask: np.ndarray,
    minimum_area: int,
    minimum_length: int,
    minimum_thin_area: int,
) -> np.ndarray:
    """
    Keep large components and long, thin components.

    This avoids deleting a real wall merely because it appears edge-on
    and therefore has a small pixel area.
    """
    component_count, labels, statistics, _ = cv2.connectedComponentsWithStats(
        binary_mask,
        connectivity=8,
    )

    cleaned = np.zeros_like(binary_mask)

    for component_id in range(1, component_count):
        area = int(statistics[component_id, cv2.CC_STAT_AREA])
        width = int(statistics[component_id, cv2.CC_STAT_WIDTH])
        height = int(statistics[component_id, cv2.CC_STAT_HEIGHT])
        longest_dimension = max(width, height)

        is_large = area >= minimum_area
        is_long_and_thin = (
            area >= minimum_thin_area
            and longest_dimension >= minimum_length
        )

        if is_large or is_long_and_thin:
            cleaned[labels == component_id] = 255

    return cleaned


def create_thin_line_mask(
    enhanced_gray: np.ndarray,
    board_mask: np.ndarray,
    thin_line_threshold: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Detect thin horizontal and vertical dark wall sections.

    Returns:
        combined response,
        horizontal line mask,
        vertical line mask,
        combined thin-line mask
    """
    horizontal_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        HORIZONTAL_BLACKHAT_KERNEL,
    )
    vertical_kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT,
        VERTICAL_BLACKHAT_KERNEL,
    )

    horizontal_response = cv2.morphologyEx(
        enhanced_gray,
        cv2.MORPH_BLACKHAT,
        horizontal_kernel,
    )
    vertical_response = cv2.morphologyEx(
        enhanced_gray,
        cv2.MORPH_BLACKHAT,
        vertical_kernel,
    )

    _, horizontal_mask = cv2.threshold(
        horizontal_response,
        thin_line_threshold,
        255,
        cv2.THRESH_BINARY,
    )
    _, vertical_mask = cv2.threshold(
        vertical_response,
        thin_line_threshold,
        255,
        cv2.THRESH_BINARY,
    )

    # Keep detections that have a horizontal or vertical line-like shape.
    horizontal_mask = cv2.morphologyEx(
        horizontal_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, HORIZONTAL_OPEN_KERNEL),
        iterations=1,
    )
    vertical_mask = cv2.morphologyEx(
        vertical_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, VERTICAL_OPEN_KERNEL),
        iterations=1,
    )

    horizontal_mask = cv2.bitwise_and(horizontal_mask, board_mask)
    vertical_mask = cv2.bitwise_and(vertical_mask, board_mask)

    thin_line_response = cv2.max(horizontal_response, vertical_response)
    thin_line_mask = cv2.bitwise_or(horizontal_mask, vertical_mask)
    thin_line_mask = cv2.bitwise_and(thin_line_mask, board_mask)

    return (
        thin_line_response,
        horizontal_mask,
        vertical_mask,
        thin_line_mask,
    )



def create_cylinder_mask(
    grayscale: np.ndarray,
    board_mask: np.ndarray,
    dark_threshold: int,
) -> tuple[np.ndarray, list[tuple[int, int, int]]]:
    """
    Detect approximately circular dark cylinders.

    Returns:
        cylinder_mask:
            255 where a cylinder itself is detected, otherwise 0.

        detected_circles:
            List of (centre_x, centre_y, radius) values used for debugging.
    """
    cylinder_candidates = cv2.inRange(
        grayscale,
        0,
        dark_threshold,
    )
    cylinder_candidates = cv2.bitwise_and(
        cylinder_candidates,
        board_mask,
    )

    close_size = odd_positive(
        CYLINDER_CLOSE_KERNEL_SIZE,
        "CYLINDER_CLOSE_KERNEL_SIZE",
    )
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size),
    )

    cylinder_candidates = cv2.morphologyEx(
        cylinder_candidates,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=CYLINDER_CLOSE_ITERATIONS,
    )

    contour_result = cv2.findContours(
        cylinder_candidates,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    if len(contour_result) == 2:
        # OpenCV 4
        contours, hierarchy = contour_result
    else:
        # OpenCV 3
        _, contours, hierarchy = contour_result

    cylinder_mask = np.zeros_like(grayscale)
    detected_circles: list[tuple[int, int, int]] = []

    for contour in contours:
        area = float(cv2.contourArea(contour))

        if area < CYLINDER_MIN_AREA or area > CYLINDER_MAX_AREA:
            continue

        x, y, width, height = cv2.boundingRect(contour)

        if height == 0:
            continue

        aspect_ratio = width / float(height)

        if not (
            CYLINDER_MIN_ASPECT_RATIO
            <= aspect_ratio
            <= CYLINDER_MAX_ASPECT_RATIO
        ):
            continue

        perimeter = float(cv2.arcLength(contour, True))

        if perimeter <= 0:
            continue

        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        if circularity < CYLINDER_MIN_CIRCULARITY:
            continue

        (centre_x, centre_y), radius = cv2.minEnclosingCircle(contour)

        if not CYLINDER_MIN_RADIUS <= radius <= CYLINDER_MAX_RADIUS:
            continue

        enclosing_area = np.pi * radius * radius

        if enclosing_area <= 0:
            continue

        fill_ratio = area / enclosing_area

        if fill_ratio < CYLINDER_MIN_FILL_RATIO:
            continue

        centre = (
            int(round(centre_x)),
            int(round(centre_y)),
        )
        integer_radius = int(np.ceil(radius))

        # Draw a clean filled circle rather than preserving uneven shadows.
        cv2.circle(
            cylinder_mask,
            centre,
            integer_radius,
            255,
            thickness=-1,
            lineType=cv2.LINE_8,
        )

        detected_circles.append(
            (centre[0], centre[1], integer_radius)
        )

    return cylinder_mask, detected_circles


def buffer_cylinder_mask(
    cylinder_mask: np.ndarray,
    buffer_pixels: int,
) -> np.ndarray:
    """
    Expand each detected cylinder by the requested safety distance.

    The returned image includes both the physical cylinder and its buffer.
    """
    if buffer_pixels < 0:
        raise ValueError("Cylinder buffer must be zero or greater.")

    if buffer_pixels == 0:
        return cylinder_mask.copy()

    diameter = 2 * buffer_pixels + 1
    kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (diameter, diameter),
    )

    return cv2.dilate(
        cylinder_mask,
        kernel,
        iterations=1,
    )

def create_maze_masks(
    image: np.ndarray,
    dark_threshold: int = DARK_THRESHOLD,
    thin_line_threshold: int = THIN_LINE_THRESHOLD,
    gamma: float = GAMMA,
    cylinder_buffer_pixels: int = CYLINDER_BUFFER_PIXELS,
) -> dict[str, np.ndarray]:
    """Create the intermediate and final masks."""
    if image is None or image.size == 0:
        raise ValueError("The supplied image is empty or could not be read.")

    height, width = image.shape[:2]
    board_mask, board_polygon = create_board_mask(height, width)

    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Increase local contrast so thin dark walls stand out from the floor.
    clahe = cv2.createCLAHE(
        clipLimit=CLAHE_CLIP_LIMIT,
        tileGridSize=CLAHE_TILE_GRID,
    )
    clahe_gray = clahe.apply(grayscale)

    # Darken the enhanced grayscale image without changing its geometry.
    enhanced_gray = apply_gamma(clahe_gray, gamma)

    # Main detection for normal/thicker dark walls.
    #
    # Use the original grayscale here rather than the enhanced image. This
    # prevents CLAHE/gamma from turning broad grey shadows into obstacles.
    # The enhanced image is reserved for thin-line recovery below.
    main_dark_mask = cv2.inRange(
        grayscale,
        0,
        dark_threshold,
    )
    main_dark_mask = cv2.bitwise_and(main_dark_mask, board_mask)

    # Detect the isolated circular cylinders separately so that only they
    # receive an additional circular safety buffer.
    cylinder_mask, detected_circles = create_cylinder_mask(
        grayscale,
        board_mask,
        dark_threshold,
    )
    buffered_cylinder_mask = buffer_cylinder_mask(
        cylinder_mask,
        cylinder_buffer_pixels,
    )

    # Separate recovery path for narrow edge-on walls.
    (
        thin_line_response,
        horizontal_line_mask,
        vertical_line_mask,
        thin_line_mask,
    ) = create_thin_line_mask(
        enhanced_gray,
        board_mask,
        thin_line_threshold,
    )

    # Suppress the faint vertical/horizontal panel seams through the centre.
    # Only black-hat-only pixels are removed; genuine dark wall pixels from the
    # main threshold are preserved.
    if SUPPRESS_CENTRE_SEAMS:
        centre_x = width // 2
        centre_y = height // 2
        half_width = CENTRE_SEAM_HALF_WIDTH

        x_start = max(0, centre_x - half_width)
        x_end = min(width, centre_x + half_width + 1)
        y_start = max(0, centre_y - half_width)
        y_end = min(height, centre_y + half_width + 1)

        thin_line_mask[:, x_start:x_end] = cv2.bitwise_and(
            thin_line_mask[:, x_start:x_end],
            main_dark_mask[:, x_start:x_end],
        )
        thin_line_mask[y_start:y_end, :] = cv2.bitwise_and(
            thin_line_mask[y_start:y_end, :],
            main_dark_mask[y_start:y_end, :],
        )

    # Combine the thick-wall and thin-wall detections.
    raw_dark_mask = cv2.bitwise_or(
        main_dark_mask,
        thin_line_mask,
    )
    raw_dark_mask = cv2.bitwise_and(raw_dark_mask, board_mask)

    close_size = odd_positive(CLOSE_KERNEL_SIZE, "CLOSE_KERNEL_SIZE")
    close_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (close_size, close_size),
    )

    # Join small gaps across cyan tape and minor lighting discontinuities.
    closed_wall_mask = cv2.morphologyEx(
        raw_dark_mask,
        cv2.MORPH_CLOSE,
        close_kernel,
        iterations=CLOSE_ITERATIONS,
    )

    cleaned_wall_mask = remove_small_components(
        closed_wall_mask,
        MIN_COMPONENT_AREA,
        MIN_COMPONENT_LENGTH,
        MIN_THIN_COMPONENT_AREA,
    )

    # One small final closing pass; 3x3 avoids excessively thick walls.
    cleaned_wall_mask = cv2.morphologyEx(
        cleaned_wall_mask,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
        iterations=1,
    )

    # Occupancy convention: white means blocked.
    # Walls are combined with the buffered cylinder regions.
    occupancy_map = cv2.bitwise_or(
        cleaned_wall_mask,
        buffered_cylinder_mask,
    )

    # Everything outside the octagonal board is blocked.
    occupancy_map[board_mask == 0] = 255

    # Add a clear boundary around the board edge.
    boundary_width = odd_positive(
        BOARD_BOUNDARY_WIDTH,
        "BOARD_BOUNDARY_WIDTH",
    )
    boundary_kernel = np.ones(
        (boundary_width, boundary_width),
        dtype=np.uint8,
    )
    board_boundary = cv2.morphologyEx(
        board_mask,
        cv2.MORPH_GRADIENT,
        boundary_kernel,
    )
    occupancy_map[board_boundary > 0] = 255

    # Opposite convention for graph construction.
    planning_map = cv2.bitwise_not(occupancy_map)

    # Red overlay showing what has been classified as blocked.
    coloured_overlay = image.copy()
    coloured_overlay[occupancy_map == 255] = (0, 0, 255)
    overlay = cv2.addWeighted(image, 0.68, coloured_overlay, 0.32, 0)

    # Separate debugging overlay:
    # yellow = physical cylinder, orange = additional cylinder buffer.
    cylinder_overlay_coloured = image.copy()
    buffer_only = cv2.bitwise_and(
        buffered_cylinder_mask,
        cv2.bitwise_not(cylinder_mask),
    )
    cylinder_overlay_coloured[buffer_only == 255] = (0, 165, 255)
    cylinder_overlay_coloured[cylinder_mask == 255] = (0, 255, 255)
    cylinder_overlay = cv2.addWeighted(
        image,
        0.62,
        cylinder_overlay_coloured,
        0.38,
        0,
    )

    for centre_x, centre_y, radius in detected_circles:
        cv2.circle(
            cylinder_overlay,
            (centre_x, centre_y),
            radius,
            (255, 0, 255),
            thickness=2,
            lineType=cv2.LINE_AA,
        )

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
        "01a_clahe_grayscale.png": clahe_gray,
        "01b_enhanced_grayscale.png": enhanced_gray,
        "02_board_polygon.png": polygon_preview,
        "02a_thin_line_response.png": thin_line_response,
        "02b_horizontal_line_mask.png": horizontal_line_mask,
        "02c_vertical_line_mask.png": vertical_line_mask,
        "02d_thin_line_mask.png": thin_line_mask,
        "03a_main_dark_mask.png": main_dark_mask,
        "03_raw_dark_mask.png": raw_dark_mask,
        "04_closed_wall_mask.png": closed_wall_mask,
        "05_cleaned_wall_mask.png": cleaned_wall_mask,
        "05a_cylinder_mask.png": cylinder_mask,
        "05b_buffered_cylinder_mask.png": buffered_cylinder_mask,
        "05c_cylinder_buffer_overlay.png": cylinder_overlay,
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
    cv2.imshow(
        "Original masking overlay",
        outputs["08_mask_overlay.png"],
    )
    cv2.imshow(
        "Enhanced grayscale",
        outputs["01b_enhanced_grayscale.png"],
    )
    cv2.imshow(
        "Thin line response",
        outputs["02a_thin_line_response.png"],
    )
    cv2.imshow(
        "Thin line mask",
        outputs["02d_thin_line_mask.png"],
    )
    cv2.imshow(
        "Main dark mask",
        outputs["03a_main_dark_mask.png"],
    )
    cv2.imshow(
        "Detected cylinders",
        outputs["05a_cylinder_mask.png"],
    )
    cv2.imshow(
        "Buffered cylinders",
        outputs["05b_buffered_cylinder_mask.png"],
    )
    cv2.imshow(
        "Cylinder buffer overlay",
        outputs["05c_cylinder_buffer_overlay.png"],
    )
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
        help=f"Main dark-pixel threshold from 0 to 255. Default: {DARK_THRESHOLD}.",
    )

    parser.add_argument(
        "--thin-threshold",
        type=int,
        default=THIN_LINE_THRESHOLD,
        help=(
            "Thin-line black-hat threshold from 0 to 255. "
            f"Default: {THIN_LINE_THRESHOLD}."
        ),
    )

    parser.add_argument(
        "--gamma",
        type=float,
        default=GAMMA,
        help=f"Gamma value greater than zero. Default: {GAMMA}.",
    )

    parser.add_argument(
        "--cylinder-buffer",
        type=int,
        default=CYLINDER_BUFFER_PIXELS,
        help=(
            "Extra safety buffer around each detected cylinder in pixels. "
            f"Default: {CYLINDER_BUFFER_PIXELS}."
        ),
    )

    parser.add_argument(
        "--save",
        action="store_true",
        help="Save all generated mask images.",
    )

    parser.add_argument(
        "--show",
        action="store_true",
        help="Open preview windows after generating the masks.",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_arguments()

    if not 0 <= args.threshold <= 255:
        raise ValueError("--threshold must be between 0 and 255.")

    if not 0 <= args.thin_threshold <= 255:
        raise ValueError("--thin-threshold must be between 0 and 255.")

    if args.gamma <= 0:
        raise ValueError("--gamma must be greater than zero.")

    if args.cylinder_buffer < 0:
        raise ValueError("--cylinder-buffer must be zero or greater.")

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
        thin_line_threshold=args.thin_threshold,
        gamma=args.gamma,
        cylinder_buffer_pixels=args.cylinder_buffer,
    )

    if args.save:
        save_outputs(outputs, output_directory)
        print(f"Masking outputs saved to:\n{output_directory}")
        print("\nUse this file for path planning:")
        print(output_directory / "07_planning_map_free_white.png")
    else:
        print("Mask generated in memory. No images were saved.")
    print("\nPlanning-map convention:")
    print("  255 = free space")
    print("    0 = obstacle or outside the board")
    print("\nUseful tuning:")
    print("  Missing thin walls: lower --thin-threshold")
    print("  Too many thin details: raise --thin-threshold")
    print("  Shadows detected as walls: lower --threshold")
    print("  Main walls missing: raise --threshold slightly")
    print("  Cylinder buffer too small: raise --cylinder-buffer")
    print("  Cylinder buffer too large: lower --cylinder-buffer")

    if args.show:
        show_outputs(outputs)


if __name__ == "__main__":
    main()
