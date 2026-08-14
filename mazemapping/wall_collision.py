"""Shared graph-edge wall collision logic for Task 1 and Task 2."""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np


Point = Tuple[int, int]


def segment_is_collision_free(
    planning_map: np.ndarray,
    first: Point,
    second: Point,
    check_thickness: int,
) -> bool:
    """Return True only when a complete line band contains no blocked pixel.

    ``planning_map`` uses the Task 1 convention: 255 is free and 0 is blocked.
    """
    if check_thickness <= 0:
        raise ValueError("Edge-check thickness must be greater than zero.")

    test_band = np.zeros_like(planning_map)
    cv2.line(
        test_band,
        first,
        second,
        255,
        thickness=check_thickness,
        lineType=cv2.LINE_8,
    )
    return not bool(np.any((test_band > 0) & (planning_map == 0)))
