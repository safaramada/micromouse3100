"""Infer a perspective-aware maze grid from the cyan wall clips.

The clips lie on the boundaries of the maze cells.  Their detected centres
are clustered into row/column boundary indices and a homography is fitted
from logical grid coordinates to image pixels.  Node positions are then the
projected centres between adjacent boundary lines.

This module does not alter or warp the source image.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import List, NamedTuple, Sequence, Tuple

import cv2
import numpy as np


Point = Tuple[int, int]


class ClipComponent(NamedTuple):
    x: float
    y: float
    area: int


class ClipGridResult(NamedTuple):
    """Result of detecting clips and fitting the logical maze grid."""

    clip_centres: np.ndarray
    node_positions: List[Point]
    homography: np.ndarray
    inlier_count: int
    median_fit_error: float


# OpenCV HSV hue spans 0 to 179.  These defaults cover the cyan clips in the
# supplied maze photographs while excluding the orange/purple corner strips.
DEFAULT_HUE_MIN = 82
DEFAULT_HUE_MAX = 108
DEFAULT_SATURATION_MIN = 30
DEFAULT_VALUE_MIN = 0

DEFAULT_MIN_COMPONENT_AREA = 100
DEFAULT_MAX_COMPONENT_AREA = 1000
DEFAULT_MAX_COMPONENT_SIZE = 60
DEFAULT_MERGE_DISTANCE = 16.0
DEFAULT_RANSAC_THRESHOLD = 5.0


def detect_cyan_clip_components(
    image: np.ndarray,
    hue_min: int = DEFAULT_HUE_MIN,
    hue_max: int = DEFAULT_HUE_MAX,
    saturation_min: int = DEFAULT_SATURATION_MIN,
    value_min: int = DEFAULT_VALUE_MIN,
    minimum_area: int = DEFAULT_MIN_COMPONENT_AREA,
    maximum_area: int = DEFAULT_MAX_COMPONENT_AREA,
    maximum_component_size: int = DEFAULT_MAX_COMPONENT_SIZE,
) -> List[ClipComponent]:
    """Detect small cyan connected components that look like wall clips."""
    if image is None or image.size == 0:
        raise ValueError("The supplied image is empty.")

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    cyan_mask = cv2.inRange(
        hsv,
        (hue_min, saturation_min, value_min),
        (hue_max, 255, 255),
    )
    cyan_mask = cv2.morphologyEx(
        cyan_mask,
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
        iterations=1,
    )

    component_count, _, statistics, centroids = (
        cv2.connectedComponentsWithStats(cyan_mask, connectivity=8)
    )

    components: List[ClipComponent] = []
    for component_id in range(1, component_count):
        width = int(statistics[component_id, cv2.CC_STAT_WIDTH])
        height = int(statistics[component_id, cv2.CC_STAT_HEIGHT])
        area = int(statistics[component_id, cv2.CC_STAT_AREA])

        if not minimum_area <= area <= maximum_area:
            continue
        if width < 2 or height < 2:
            continue
        if width > maximum_component_size or height > maximum_component_size:
            continue

        components.append(
            ClipComponent(
                x=float(centroids[component_id, 0]),
                y=float(centroids[component_id, 1]),
                area=area,
            )
        )

    return components


def merge_nearby_components(
    components: Sequence[ClipComponent],
    merge_distance: float = DEFAULT_MERGE_DISTANCE,
) -> np.ndarray:
    """Merge fragments belonging to the same physical clip."""
    if merge_distance <= 0:
        raise ValueError("Clip merge distance must be greater than zero.")
    if not components:
        return np.empty((0, 2), dtype=np.float32)

    coordinates = np.asarray(
        [(component.x, component.y) for component in components],
        dtype=np.float32,
    )
    areas = np.asarray(
        [component.area for component in components],
        dtype=np.float32,
    )

    visited = set()
    groups: List[List[int]] = []

    for start_index in range(len(components)):
        if start_index in visited:
            continue

        stack = [start_index]
        visited.add(start_index)
        group: List[int] = []

        while stack:
            current_index = stack.pop()
            group.append(current_index)
            distances = np.linalg.norm(
                coordinates - coordinates[current_index],
                axis=1,
            )

            for neighbour_index in np.flatnonzero(
                distances < merge_distance
            ):
                neighbour = int(neighbour_index)
                if neighbour not in visited:
                    visited.add(neighbour)
                    stack.append(neighbour)

        groups.append(group)

    merged_centres = []
    for group in groups:
        group_coordinates = coordinates[group]
        group_areas = areas[group]
        weighted_centre = np.average(
            group_coordinates,
            axis=0,
            weights=group_areas,
        )
        merged_centres.append(weighted_centre)

    return np.asarray(merged_centres, dtype=np.float32)


def cluster_axis(values: np.ndarray, cluster_count: int) -> np.ndarray:
    """Assign 1-D values to ordered grid-boundary clusters."""
    if len(values) < cluster_count:
        raise RuntimeError(
            "Detected only {} clips, but {} grid-boundary levels are needed."
            .format(len(values), cluster_count)
        )

    data = values.reshape(-1, 1).astype(np.float32)
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        0.01,
    )

    cv2.setRNGSeed(3100)
    _, labels, centres = cv2.kmeans(
        data,
        cluster_count,
        None,
        criteria,
        30,
        cv2.KMEANS_PP_CENTERS,
    )

    sorted_cluster_ids = np.argsort(centres[:, 0])
    ordered_id_by_cluster = np.empty(cluster_count, dtype=np.int32)
    ordered_id_by_cluster[sorted_cluster_ids] = np.arange(cluster_count)
    return ordered_id_by_cluster[labels[:, 0]]


def combine_duplicate_grid_observations(
    logical_points: np.ndarray,
    image_points: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Average multiple fragments assigned to one logical intersection."""
    grouped = {}
    for logical_point, image_point in zip(logical_points, image_points):
        key = (int(logical_point[0]), int(logical_point[1]))
        grouped.setdefault(key, []).append(image_point)

    unique_logical = []
    averaged_image = []
    for key, observations in grouped.items():
        unique_logical.append(key)
        averaged_image.append(np.mean(observations, axis=0))

    return (
        np.asarray(unique_logical, dtype=np.float32),
        np.asarray(averaged_image, dtype=np.float32),
    )


def project_points(
    logical_points: np.ndarray,
    homography: np.ndarray,
) -> np.ndarray:
    """Project logical grid coordinates into image pixels."""
    points = np.asarray(logical_points, dtype=np.float32).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, homography)[:, 0, :]


def infer_clip_grid(
    image: np.ndarray,
    rows: int = 9,
    columns: int = 9,
    hue_min: int = DEFAULT_HUE_MIN,
    hue_max: int = DEFAULT_HUE_MAX,
    saturation_min: int = DEFAULT_SATURATION_MIN,
    value_min: int = DEFAULT_VALUE_MIN,
    merge_distance: float = DEFAULT_MERGE_DISTANCE,
    ransac_threshold: float = DEFAULT_RANSAC_THRESHOLD,
    excluded_logical_regions: Sequence[Tuple[int, int, int, int]] = (),
) -> ClipGridResult:
    """Detect cyan clips and return perspective-aware cell-centre nodes.

    ``excluded_logical_regions`` contains ``(top, left, height, width)``
    rectangles. Clip observations strictly inside those rectangles are left
    out of the homography fit, while observations on their perimeter remain
    available. This lets Task 4.2 infer its normal-maze grid from the cyan
    clips surrounding the continuous white-space region without treating that
    region as a regular maze grid.
    """
    if rows < 2 or columns < 2:
        raise ValueError("Rows and columns must both be at least two.")

    components = detect_cyan_clip_components(
        image=image,
        hue_min=hue_min,
        hue_max=hue_max,
        saturation_min=saturation_min,
        value_min=value_min,
    )
    clip_centres = merge_nearby_components(
        components,
        merge_distance=merge_distance,
    )

    minimum_observations = max(rows + 1, columns + 1) * 2
    if len(clip_centres) < minimum_observations:
        raise RuntimeError(
            "Only {} cyan clips were detected; at least {} are needed to fit "
            "the grid reliably. Adjust the HSV thresholds or use the fixed "
            "grid instead."
            .format(len(clip_centres), minimum_observations)
        )

    column_indices = cluster_axis(clip_centres[:, 0], columns + 1)
    row_indices = cluster_axis(clip_centres[:, 1], rows + 1)

    logical_observations = np.column_stack(
        (column_indices, row_indices)
    ).astype(np.float32)
    logical_observations, image_observations = (
        combine_duplicate_grid_observations(
            logical_observations,
            clip_centres,
        )
    )

    if excluded_logical_regions:
        keep = np.ones(len(logical_observations), dtype=bool)
        for top, left, region_height, region_width in excluded_logical_regions:
            if region_height <= 0 or region_width <= 0:
                raise ValueError("Excluded logical regions must have positive sizes.")
            bottom = top + region_height
            right = left + region_width
            columns_in_region = (
                (logical_observations[:, 0] > left)
                & (logical_observations[:, 0] < right)
            )
            rows_in_region = (
                (logical_observations[:, 1] > top)
                & (logical_observations[:, 1] < bottom)
            )
            keep &= ~(columns_in_region & rows_in_region)
        logical_observations = logical_observations[keep]
        image_observations = image_observations[keep]

    if len(logical_observations) < minimum_observations:
        raise RuntimeError(
            "Only {} clip observations remain outside the excluded regions; "
            "at least {} are required."
            .format(len(logical_observations), minimum_observations)
        )

    homography, inlier_mask = cv2.findHomography(
        logical_observations,
        image_observations,
        method=cv2.RANSAC,
        ransacReprojThreshold=ransac_threshold,
        maxIters=5000,
        confidence=0.999,
    )
    if homography is None or inlier_mask is None:
        raise RuntimeError("The cyan clip detections could not form a grid.")

    inliers = inlier_mask[:, 0] > 0
    inlier_count = int(np.count_nonzero(inliers))
    if inlier_count < minimum_observations:
        raise RuntimeError(
            "Only {} clip observations agree with the fitted grid; at least "
            "{} are required."
            .format(inlier_count, minimum_observations)
        )

    fitted_observations = project_points(
        logical_observations,
        homography,
    )
    fit_errors = np.linalg.norm(
        fitted_observations - image_observations,
        axis=1,
    )
    median_fit_error = float(np.median(fit_errors[inliers]))

    logical_nodes = np.asarray(
        [
            (column + 0.5, row + 0.5)
            for row in range(rows)
            for column in range(columns)
        ],
        dtype=np.float32,
    )
    projected_nodes = project_points(logical_nodes, homography)
    node_positions = [
        (int(round(float(x))), int(round(float(y))))
        for x, y in projected_nodes
    ]

    return ClipGridResult(
        clip_centres=clip_centres,
        node_positions=node_positions,
        homography=homography,
        inlier_count=inlier_count,
        median_fit_error=median_fit_error,
    )


def draw_clip_grid(
    image: np.ndarray,
    result: ClipGridResult,
    rows: int,
    columns: int,
    show_labels: bool = True,
) -> np.ndarray:
    """Draw detected clips, fitted cell boundaries, and node centres."""
    output = image.copy()

    for column in range(columns + 1):
        endpoints = project_points(
            np.asarray([(column, 0), (column, rows)], dtype=np.float32),
            result.homography,
        )
        first = tuple(np.rint(endpoints[0]).astype(int))
        second = tuple(np.rint(endpoints[1]).astype(int))
        cv2.line(output, first, second, (255, 220, 0), 2, cv2.LINE_AA)

    for row in range(rows + 1):
        endpoints = project_points(
            np.asarray([(0, row), (columns, row)], dtype=np.float32),
            result.homography,
        )
        first = tuple(np.rint(endpoints[0]).astype(int))
        second = tuple(np.rint(endpoints[1]).astype(int))
        cv2.line(output, first, second, (255, 220, 0), 2, cv2.LINE_AA)

    for x, y in result.clip_centres:
        cv2.circle(
            output,
            (int(round(float(x))), int(round(float(y)))),
            4,
            (0, 0, 255),
            -1,
            cv2.LINE_AA,
        )

    for node_id, point in enumerate(result.node_positions):
        cv2.circle(output, point, 4, (0, 255, 0), -1, cv2.LINE_AA)
        if show_labels:
            cv2.putText(
                output,
                str(node_id),
                (point[0] + 6, point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (255, 0, 255),
                1,
                cv2.LINE_AA,
            )

    return output


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Infer and preview a maze grid from its cyan wall clips.",
    )
    parser.add_argument("image", type=Path, help="Maze photograph.")
    parser.add_argument("--rows", type=int, default=9)
    parser.add_argument("--columns", type=int, default=9)
    parser.add_argument("--hue-min", type=int, default=DEFAULT_HUE_MIN)
    parser.add_argument("--hue-max", type=int, default=DEFAULT_HUE_MAX)
    parser.add_argument(
        "--saturation-min",
        type=int,
        default=DEFAULT_SATURATION_MIN,
    )
    parser.add_argument("--value-min", type=int, default=DEFAULT_VALUE_MIN)
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Preview PNG; default is '<image>_clip_grid.png'.",
    )
    parser.add_argument(
        "--json-output",
        type=Path,
        default=None,
        help="Optional JSON file containing the fitted nodes and homography.",
    )
    parser.add_argument("--no-labels", action="store_true")
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_arguments()
    image_path = args.image.expanduser().resolve()
    if not image_path.is_file():
        raise FileNotFoundError("Maze image not found: {}".format(image_path))

    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("OpenCV could not read: {}".format(image_path))

    result = infer_clip_grid(
        image=image,
        rows=args.rows,
        columns=args.columns,
        hue_min=args.hue_min,
        hue_max=args.hue_max,
        saturation_min=args.saturation_min,
        value_min=args.value_min,
    )
    preview = draw_clip_grid(
        image=image,
        result=result,
        rows=args.rows,
        columns=args.columns,
        show_labels=not args.no_labels,
    )

    output_path = (
        args.output.expanduser().resolve()
        if args.output is not None
        else image_path.with_name(image_path.stem + "_clip_grid.png")
    )
    if not cv2.imwrite(str(output_path), preview):
        raise OSError("Could not save clip-grid preview: {}".format(output_path))

    print("Detected cyan clip centres: {}".format(len(result.clip_centres)))
    print("Grid-fit inliers: {}".format(result.inlier_count))
    print(
        "Median clip fit error: {:.2f} pixels".format(
            result.median_fit_error
        )
    )
    print("Saved clip-grid preview to: {}".format(output_path))

    if args.json_output is not None:
        json_path = args.json_output.expanduser().resolve()
        json_data = {
            "rows": args.rows,
            "columns": args.columns,
            "node_positions": [list(point) for point in result.node_positions],
            "homography": result.homography.tolist(),
            "detected_clip_count": len(result.clip_centres),
            "grid_fit_inliers": result.inlier_count,
            "median_fit_error_pixels": result.median_fit_error,
        }
        json_path.write_text(
            json.dumps(json_data, indent=2) + "\n",
            encoding="utf-8",
        )
        print("Saved clip-grid data to: {}".format(json_path))

    if args.show:
        cv2.imshow("Clip-derived maze grid", preview)
        print("Press any key in the image window to close it.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
