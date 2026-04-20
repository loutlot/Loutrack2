"""
Blob detection for reflective markers.

Extracted from capture_runtime.py so it can be independently imported,
tested, and potentially replaced (e.g. with a GPU-accelerated variant).
"""
from __future__ import annotations

import math

import cv2
import numpy as np


def detect_blobs(
    frame: np.ndarray,
    threshold: int,
    mask: np.ndarray | None = None,
    min_diameter_px: float | None = None,
    max_diameter_px: float | None = None,
    circularity_min: float = 0.0,
) -> tuple[list[dict[str, float]], dict[str, object]]:
    """
    Detect bright circular blobs in a grayscale (or BGR) frame.

    Algorithm:
    1. Convert to grayscale if needed.
    2. Apply static mask (zero out masked pixels).
    3. Binary threshold → find external contours.
    4. Filter by equivalent-circle diameter and circularity.
    5. Compute centroid via image moments.

    Args:
        frame:           Input image (grayscale or BGR uint8).
        threshold:       Brightness threshold [0, 255].
        mask:            Boolean mask; True pixels are suppressed.
        min_diameter_px: Minimum equivalent-circle diameter (px).
        max_diameter_px: Maximum equivalent-circle diameter (px).
        circularity_min: Minimum circularity [0, 1] (4π·area / perimeter²).

    Returns:
        (blobs, diagnostics) where each blob is {"x", "y", "area"}.
    """
    if frame.ndim == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame

    mask_bool: np.ndarray | None = None
    if mask is not None:
        if mask.shape != gray.shape:
            raise ValueError("mask dimensions must match the grayscale frame")
        mask_bool = mask.astype(bool, copy=False)

    threshold_value = max(0, min(255, int(threshold)))
    min_diameter = float(min_diameter_px) if min_diameter_px is not None else None
    max_diameter = float(max_diameter_px) if max_diameter_px is not None else None
    circularity_floor = max(0.0, min(1.0, float(circularity_min)))
    diameter_filter_enabled = min_diameter is not None or max_diameter is not None
    circularity_filter_enabled = circularity_floor > 0.0

    _retval, binary = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY)
    if mask_bool is not None:
        binary[mask_bool] = 0
    contours_info = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

    blobs: list[dict[str, float]] = []
    rejected_by_diameter = 0
    rejected_by_circularity = 0

    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area <= 0.0:
            continue

        if diameter_filter_enabled:
            diameter_px = float(math.sqrt((4.0 * area) / math.pi))
            if min_diameter is not None and diameter_px < min_diameter:
                rejected_by_diameter += 1
                continue
            if max_diameter is not None and diameter_px > max_diameter:
                rejected_by_diameter += 1
                continue

        if circularity_filter_enabled:
            perimeter = float(cv2.arcLength(contour, True))
            if perimeter <= 0.0:
                rejected_by_circularity += 1
                continue

            circularity = float((4.0 * math.pi * area) / (perimeter * perimeter))
            if circularity < circularity_floor:
                rejected_by_circularity += 1
                continue

        moments = cv2.moments(contour)
        m00 = float(moments.get("m00", 0.0))
        if m00 <= 0.0:
            continue

        blobs.append(
            {
                "x": float(moments["m10"] / m00),
                "y": float(moments["m01"] / m00),
                "area": area,
            }
        )

    if len(blobs) > 1:
        blobs.sort(key=lambda b: (b["y"], b["x"]))

    diagnostics: dict[str, object] = {
        "threshold": threshold_value,
        "min_diameter_px": min_diameter,
        "max_diameter_px": max_diameter,
        "circularity_min": circularity_floor,
        "raw_contour_count": len(contours),
        "accepted_blob_count": len(blobs),
        "rejected_by_diameter": rejected_by_diameter,
        "rejected_by_circularity": rejected_by_circularity,
        "last_blob_count": len(blobs),
    }
    return blobs, diagnostics
