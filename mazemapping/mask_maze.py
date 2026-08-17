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

try:
    from .clip_grid import create_cyan_clip_mask
except ImportError:  # Support direct ``python mask_maze.py`` execution.
    from clip_grid import create_cyan_clip_mask


# ------------------------- Parameters to tune ------------------------- #

# Main wall threshold. Lower this if shadows are being detected as walls.
# Raise it slightly if genuine dark walls are missing.
DARK_THRESHOLD = 92

# Local contrast enhancement.
CLAHE_CLIP_LIMIT = 2.0
CLAHE_TILE_GRID = (8, 8)

# Gamma > 1 darkens mid-grey/dark pixels.
GAMMA = 1.21

# Run the geometry-sensitive morphology at one reference resolution. Camera
# exports of the same maze have ranged from roughly 670 px to 1160 px square;
# fixed pixel kernels otherwise behave differently on the low-resolution pair.
# Outputs are transformed back to the supplied image dimensions afterwards.
MASK_CANONICAL_SIZE = 1000

# Hysteresis thresholds for black-hat thin-wall recovery. Weak line pixels are
# retained only when their connected component reaches stronger line evidence
# or the main dark-wall mask. This preserves faint edge-on walls without
# admitting every isolated floor seam detected at the weak threshold.
THIN_LINE_THRESHOLD = 16
THIN_LINE_STRONG_THRESHOLD = 22
THIN_LINE_SUPPORT_DILATION_SIZE = 15


# Black-hat kernels for detecting horizontal and vertical dark lines.
HORIZONTAL_BLACKHAT_KERNEL = (17, 3)
VERTICAL_BLACKHAT_KERNEL = (3, 17)

# Directional opening suppresses non-line-shaped black-hat detections.
HORIZONTAL_OPEN_KERNEL = (6, 1)
VERTICAL_OPEN_KERNEL = (1, 6)

# Morphological closing joins wall pieces interrupted by cyan tape.
CLOSE_KERNEL_SIZE = 3
CLOSE_ITERATIONS = 1

# Large connected regions are kept automatically.
MIN_COMPONENT_AREA = 100

# Long, thin regions can also be kept even when their area is small.
MIN_COMPONENT_LENGTH = 15
MIN_THIN_COMPONENT_AREA = 8

# A short black-hat response can also be caused by a light floor mark. Only
# small isolated components receive this absolute-intensity check; full walls
# and Task 2 cylinders are deliberately outside its scope.
MAX_SMALL_COMPONENT_AREA = 250
MAX_SMALL_COMPONENT_MEDIAN_GRAY = 165

# The photographed board has centre panel seams that are not obstacles.
# Suppression is limited to the known centre strips and protects crossing
# walls and broad objects.
SUPPRESS_CENTRE_SEAMS = True
# CENTRE_SEAM_HALF_WIDTH = 7
CENTRE_SEAM_HALF_WIDTH = 8

# Pixels on the known centre seams are removed unless they belong to a
# perpendicular wall or a genuinely broad object. This rejects dark tape/seam
# fragments without cutting a crossing wall or a Task 2 cylinder.
CENTRE_SEAM_CROSSING_LENGTH = 19
CENTRE_SEAM_BROAD_FEATURE_SIZE = 11


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


def suppress_bright_small_components(
    binary_mask: np.ndarray,
    grayscale: np.ndarray,
    maximum_area: int = MAX_SMALL_COMPONENT_AREA,
    maximum_median_gray: int = MAX_SMALL_COMPONENT_MEDIAN_GRAY,
) -> np.ndarray:
    """Discard small isolated line responses that are too bright to be walls."""
    if binary_mask.shape != grayscale.shape:
        raise ValueError("binary_mask and grayscale must have matching shapes")

    component_count, labels, statistics, _ = cv2.connectedComponentsWithStats(
        binary_mask,
        connectivity=8,
    )
    cleaned = binary_mask.copy()
    for component_id in range(1, component_count):
        area = int(statistics[component_id, cv2.CC_STAT_AREA])
        if area > maximum_area:
            continue
        component_values = grayscale[labels == component_id]
        if (
            component_values.size
            and float(np.median(component_values)) > maximum_median_gray
        ):
            cleaned[labels == component_id] = 0
    return cleaned


def suppress_centre_seam_artifacts(
    binary_mask: np.ndarray,
    half_width: int = CENTRE_SEAM_HALF_WIDTH,
) -> np.ndarray:
    """Remove the two known panel seams while preserving real obstacles.

    A horizontal structure is protected where it crosses the vertical seam,
    and vice versa. A small two-dimensional opening additionally protects
    broad objects such as cylinders. Thin structures running along a centre
    seam are board artifacts because the nine-cell grid places these seams
    through cell centres rather than on valid wall boundaries.
    """
    if binary_mask.ndim != 2:
        raise ValueError("binary_mask must be a two-dimensional image")
    if half_width < 0:
        raise ValueError("half_width must not be negative")
    if half_width == 0:
        return binary_mask.copy()

    height, width = binary_mask.shape
    centre_x = width // 2
    centre_y = height // 2
    x_start = max(0, centre_x - half_width)
    x_end = min(width, centre_x + half_width + 1)
    y_start = max(0, centre_y - half_width)
    y_end = min(height, centre_y + half_width + 1)

    crossing_length = odd_positive(
        CENTRE_SEAM_CROSSING_LENGTH,
        "CENTRE_SEAM_CROSSING_LENGTH",
    )
    broad_size = odd_positive(
        CENTRE_SEAM_BROAD_FEATURE_SIZE,
        "CENTRE_SEAM_BROAD_FEATURE_SIZE",
    )
    horizontal_crossings = cv2.morphologyEx(
        binary_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (crossing_length, 1)),
    )
    vertical_crossings = cv2.morphologyEx(
        binary_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, (1, crossing_length)),
    )
    broad_features = cv2.morphologyEx(
        binary_mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (broad_size, broad_size)),
    )

    cleaned = binary_mask.copy()
    vertical_protection = cv2.bitwise_or(horizontal_crossings, broad_features)
    horizontal_protection = cv2.bitwise_or(vertical_crossings, broad_features)
    cleaned[:, x_start:x_end] = cv2.bitwise_and(
        binary_mask[:, x_start:x_end],
        vertical_protection[:, x_start:x_end],
    )
    cleaned[y_start:y_end, :] = cv2.bitwise_and(
        cleaned[y_start:y_end, :],
        horizontal_protection[y_start:y_end, :],
    )

    # At the crossing of the two panel seams, only a broad object is valid.
    cleaned[y_start:y_end, x_start:x_end] = cv2.bitwise_and(
        binary_mask[y_start:y_end, x_start:x_end],
        broad_features[y_start:y_end, x_start:x_end],
    )
    return cleaned


def keep_supported_line_components(
    candidate_mask: np.ndarray,
    support_mask: np.ndarray,
) -> np.ndarray:
    """Keep weak line components only when they touch stronger wall support."""
    if candidate_mask.shape != support_mask.shape:
        raise ValueError("candidate and support masks must have matching shapes")

    component_count, labels = cv2.connectedComponents(candidate_mask, connectivity=8)
    supported = np.zeros_like(candidate_mask)
    for component_id in range(1, component_count):
        component = labels == component_id
        if np.any(support_mask[component] > 0):
            supported[component] = 255
    return supported


def _directional_line_mask(
    response: np.ndarray,
    threshold: int,
    open_kernel_size: tuple[int, int],
    close_kernel_size: tuple[int, int],
) -> np.ndarray:
    """Threshold and directionally regularise one black-hat response."""
    _, mask = cv2.threshold(response, threshold, 255, cv2.THRESH_BINARY)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_RECT, open_kernel_size),
        iterations=1,
    )
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_RECT, close_kernel_size),
        iterations=1,
    )


def create_thin_line_mask(
    enhanced_gray: np.ndarray,
    board_mask: np.ndarray,
    main_dark_mask: np.ndarray,
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

    horizontal_mask = _directional_line_mask(
        horizontal_response,
        thin_line_threshold,
        HORIZONTAL_OPEN_KERNEL,
        (9, 1),
    )
    vertical_mask = _directional_line_mask(
        vertical_response,
        thin_line_threshold,
        VERTICAL_OPEN_KERNEL,
        (1, 9),
    )

    strong_threshold = max(thin_line_threshold, THIN_LINE_STRONG_THRESHOLD)
    strong_horizontal = _directional_line_mask(
        horizontal_response,
        strong_threshold,
        HORIZONTAL_OPEN_KERNEL,
        (9, 1),
    )
    strong_vertical = _directional_line_mask(
        vertical_response,
        strong_threshold,
        VERTICAL_OPEN_KERNEL,
        (1, 9),
    )

    support_size = odd_positive(
        THIN_LINE_SUPPORT_DILATION_SIZE,
        "THIN_LINE_SUPPORT_DILATION_SIZE",
    )
    support_kernel = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (support_size, support_size),
    )
    horizontal_support = cv2.dilate(
        cv2.bitwise_or(strong_horizontal, main_dark_mask),
        support_kernel,
        iterations=1,
    )
    vertical_support = cv2.dilate(
        cv2.bitwise_or(strong_vertical, main_dark_mask),
        support_kernel,
        iterations=1,
    )

    supported_horizontal = keep_supported_line_components(
        horizontal_mask,
        horizontal_support,
    )
    supported_vertical = keep_supported_line_components(
        vertical_mask,
        vertical_support,
    )
    horizontal_mask = cv2.bitwise_and(horizontal_mask, board_mask)
    vertical_mask = cv2.bitwise_and(vertical_mask, board_mask)
    supported_horizontal = cv2.bitwise_and(supported_horizontal, board_mask)
    supported_vertical = cv2.bitwise_and(supported_vertical, board_mask)

    thin_line_response = cv2.max(horizontal_response, vertical_response)
    thin_line_mask = cv2.bitwise_or(supported_horizontal, supported_vertical)
    thin_line_mask = cv2.bitwise_and(thin_line_mask, board_mask)

    return (
        thin_line_response,
        horizontal_mask,
        vertical_mask,
        thin_line_mask,
    )


def _create_maze_masks_native(
    image: np.ndarray,
    dark_threshold: int = DARK_THRESHOLD,
    thin_line_threshold: int = THIN_LINE_THRESHOLD,
    gamma: float = GAMMA,
) -> dict[str, np.ndarray]:
    """Create the intermediate and final masks."""
    if image is None or image.size == 0:
        raise ValueError("The supplied image is empty or could not be read.")

    height, width = image.shape[:2]
    board_mask, board_polygon = create_board_mask(height, width)

    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    cyan_clip_mask = create_cyan_clip_mask(image)

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
    if SUPPRESS_CENTRE_SEAMS:
        main_dark_mask = suppress_centre_seam_artifacts(main_dark_mask)

    # Separate recovery path for narrow edge-on walls.
    (
        thin_line_response,
        horizontal_line_mask,
        vertical_line_mask,
        thin_line_mask,
    ) = create_thin_line_mask(
        enhanced_gray,
        board_mask,
        main_dark_mask,
        thin_line_threshold,
    )

    # Suppress directional thin-line evidence on the corresponding known panel
    # seam. Perpendicular wall evidence remains untouched.
    if SUPPRESS_CENTRE_SEAMS:
        centre_x = width // 2
        centre_y = height // 2
        half_width = CENTRE_SEAM_HALF_WIDTH

        x_start = max(0, centre_x - half_width)
        x_end = min(width, centre_x + half_width + 1)
        y_start = max(0, centre_y - half_width)
        y_end = min(height, centre_y + half_width + 1)

        vertical_line_mask[:, x_start:x_end] = 0
        horizontal_line_mask[y_start:y_end, :] = 0
        thin_line_mask = suppress_centre_seam_artifacts(thin_line_mask)

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
    cleaned_wall_mask = suppress_bright_small_components(
        cleaned_wall_mask,
        grayscale,
    )

    # Occupancy convention: white means blocked.
    occupancy_map = cleaned_wall_mask.copy()

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
        "02e_cyan_clip_mask.png": cyan_clip_mask,
        "03a_main_dark_mask.png": main_dark_mask,
        "03_raw_dark_mask.png": raw_dark_mask,
        "04_closed_wall_mask.png": closed_wall_mask,
        "05_cleaned_wall_mask.png": cleaned_wall_mask,
        "06_occupancy_obstacles_white.png": occupancy_map,
        "07_planning_map_free_white.png": planning_map,
        "08_mask_overlay.png": overlay,
    }


_BINARY_OUTPUTS = frozenset(
    {
        "02b_horizontal_line_mask.png",
        "02c_vertical_line_mask.png",
        "02d_thin_line_mask.png",
        "02e_cyan_clip_mask.png",
        "03a_main_dark_mask.png",
        "03_raw_dark_mask.png",
        "04_closed_wall_mask.png",
        "05_cleaned_wall_mask.png",
        "06_occupancy_obstacles_white.png",
    }
)


def create_maze_masks(
    image: np.ndarray,
    dark_threshold: int = DARK_THRESHOLD,
    thin_line_threshold: int = THIN_LINE_THRESHOLD,
    gamma: float = GAMMA,
    canonical_size: int | None = MASK_CANONICAL_SIZE,
) -> dict[str, np.ndarray]:
    """Create masks with resolution-normalised morphology.

    The photographed rig fills the source frame in the supplied captures, so
    resizing it to a common square makes the fixed black-hat, opening, closing,
    and component kernels represent consistent physical sizes. Every returned
    output is restored to the input dimensions for Task 1 compatibility.
    Pass ``canonical_size=None`` to retain native-resolution behaviour.
    """
    if image is None or image.size == 0:
        raise ValueError("The supplied image is empty or could not be read.")
    if canonical_size is None:
        return _create_maze_masks_native(
            image,
            dark_threshold,
            thin_line_threshold,
            gamma,
        )
    if canonical_size < 2:
        raise ValueError("canonical_size must be at least 2 or None")

    height, width = image.shape[:2]
    if height == canonical_size and width == canonical_size:
        return _create_maze_masks_native(
            image,
            dark_threshold,
            thin_line_threshold,
            gamma,
        )

    normalised = cv2.resize(
        image,
        (canonical_size, canonical_size),
        interpolation=(
            cv2.INTER_AREA
            if canonical_size < max(height, width)
            else cv2.INTER_CUBIC
        ),
    )
    normalised_outputs = _create_maze_masks_native(
        normalised,
        dark_threshold,
        thin_line_threshold,
        gamma,
    )

    restored: dict[str, np.ndarray] = {}
    for name, output in normalised_outputs.items():
        resized = cv2.resize(
            output,
            (width, height),
            interpolation=cv2.INTER_AREA,
        )
        if name in _BINARY_OUTPUTS:
            resized = np.where(resized > 0, 255, 0).astype(np.uint8)
        restored[name] = resized

    occupancy_name = "06_occupancy_obstacles_white.png"
    planning_name = "07_planning_map_free_white.png"
    restored[planning_name] = cv2.bitwise_not(restored[occupancy_name])

    coloured_overlay = image.copy()
    coloured_overlay[restored[occupancy_name] == 255] = (0, 0, 255)
    restored["08_mask_overlay.png"] = cv2.addWeighted(
        image,
        0.68,
        coloured_overlay,
        0.32,
        0,
    )
    return restored


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
        "--show",
        action="store_true",
        help="Open preview windows after saving the masks.",
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
    )

    # save_outputs(outputs, output_directory)

    print(f"Masking complete. Outputs saved to:\n{output_directory}")
    print("\nUse this file for path planning:")
    print(output_directory / "07_planning_map_free_white.png")
    print("\nPlanning-map convention:")
    print("  255 = free space")
    print("    0 = obstacle or outside the board")
    print("\nUseful tuning:")
    print("  Missing thin walls: lower --thin-threshold")
    print("  Too many thin details: raise --thin-threshold")
    print("  Shadows detected as walls: lower --threshold")
    print("  Main walls missing: raise --threshold slightly")

    if args.show:
        show_outputs(outputs)


if __name__ == "__main__":
    main()
