from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Dict, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class WandLabelResult:
    points: np.ndarray  # shape=(4,2), order=[elbow, short, mid, long]
    collinearity_error: float
    midpoint_ratio_error: float


def _cross2d(a: np.ndarray, b: np.ndarray) -> float:
    return float((a[0] * b[1]) - (a[1] * b[0]))


def canonicalize_wand_points(
    blobs: Sequence[Dict[str, float]],
    *,
    min_blob_area: float = 0.0,
    collinearity_threshold: float = 0.035,
    midpoint_ratio_threshold: float = 0.35,
) -> Optional[WandLabelResult]:
    if len(blobs) < 4:
        return None

    usable = [blob for blob in blobs if float(blob.get("area", 0.0)) >= min_blob_area]
    if len(usable) < 4:
        return None

    top4 = sorted(usable, key=lambda blob: float(blob.get("area", 0.0)), reverse=True)[:4]
    pts = np.array([[float(blob["x"]), float(blob["y"])] for blob in top4], dtype=np.float64)

    scale = max(float(np.linalg.norm(pts.max(axis=0) - pts.min(axis=0))), 1.0)
    best_triplet = None
    best_area_norm = float("inf")
    for i, j, k in combinations(range(4), 3):
        area2 = abs(_cross2d(pts[j] - pts[i], pts[k] - pts[i]))
        area_norm = area2 / (scale * scale)
        if area_norm < best_area_norm:
            best_area_norm = area_norm
            best_triplet = (i, j, k)
    if best_triplet is None or best_area_norm > collinearity_threshold:
        return None

    line_ids = list(best_triplet)
    elbow_id = ({0, 1, 2, 3} - set(line_ids)).pop()
    elbow_pt = pts[elbow_id]

    mid_id = None
    endpoint_pair = None
    best_mid_score = float("inf")
    for candidate_mid in line_ids:
        endpoints = [idx for idx in line_ids if idx != candidate_mid]
        endpoint_a, endpoint_b = endpoints[0], endpoints[1]
        vec = pts[endpoint_b] - pts[endpoint_a]
        denom = float(np.dot(vec, vec))
        if denom <= 1e-9:
            continue
        t = float(np.dot(pts[candidate_mid] - pts[endpoint_a], vec) / denom)
        dist_line = float(abs(_cross2d(vec, pts[candidate_mid] - pts[endpoint_a])) / np.sqrt(denom))
        if not (0.10 <= t <= 0.90):
            continue
        score = abs(t - 0.5) + (dist_line / scale)
        if score < best_mid_score:
            best_mid_score = score
            mid_id = candidate_mid
            endpoint_pair = (endpoint_a, endpoint_b)
    if mid_id is None or endpoint_pair is None:
        return None

    endpoint_a, endpoint_b = endpoint_pair
    dist_a = float(np.linalg.norm(elbow_pt - pts[endpoint_a]))
    dist_b = float(np.linalg.norm(elbow_pt - pts[endpoint_b]))
    if min(dist_a, dist_b) <= 2.0 or abs(dist_a - dist_b) <= 1.0:
        return None
    short_id, long_id = ((endpoint_a, endpoint_b) if dist_a <= dist_b else (endpoint_b, endpoint_a))
    long_pt = pts[long_id]

    long_len = float(np.linalg.norm(long_pt - elbow_pt))
    mid_len = float(np.linalg.norm(pts[mid_id] - elbow_pt))
    if long_len <= 1e-9:
        return None
    ratio_err = abs((mid_len / long_len) - 0.5)
    if ratio_err > midpoint_ratio_threshold:
        return None

    return WandLabelResult(
        points=np.stack([elbow_pt, pts[short_id], pts[mid_id], long_pt], axis=0),
        collinearity_error=float(best_area_norm),
        midpoint_ratio_error=float(ratio_err),
    )
