from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np


@dataclass(frozen=True)
class LabeledFrameObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_points: np.ndarray  # shape=(4,2)
    collinearity_error: float
    midpoint_ratio_error: float
    confidence: float


@dataclass(frozen=True)
class MultiViewSample:
    sample_id: int
    timestamps: Dict[str, int]
    image_points_by_camera: Dict[str, np.ndarray]


def build_multiview_samples(
    labeled_by_camera: Dict[str, Sequence[LabeledFrameObservation]],
    reference_camera_id: str,
    pair_window_us: int,
) -> List[MultiViewSample]:
    ref_frames = list(labeled_by_camera.get(reference_camera_id, []))
    if not ref_frames:
        return []

    camera_ids = sorted(camera_id for camera_id in labeled_by_camera.keys() if camera_id != reference_camera_id)
    pointers = {camera_id: 0 for camera_id in camera_ids}
    used = {camera_id: -1 for camera_id in camera_ids}
    samples: List[MultiViewSample] = []

    for ref in ref_frames:
        timestamps = {reference_camera_id: ref.timestamp}
        points_by_camera = {reference_camera_id: ref.image_points}
        for camera_id in camera_ids:
            frames = labeled_by_camera[camera_id]
            j = max(pointers[camera_id], used[camera_id] + 1)
            while j < len(frames) and frames[j].timestamp < ref.timestamp - pair_window_us:
                j += 1
            best_idx = -1
            best_delta = pair_window_us + 1
            k = j
            while k < len(frames):
                delta = abs(frames[k].timestamp - ref.timestamp)
                if frames[k].timestamp > ref.timestamp + pair_window_us:
                    break
                if delta < best_delta:
                    best_delta = delta
                    best_idx = k
                k += 1
            pointers[camera_id] = j
            if best_idx < 0:
                continue
            used[camera_id] = best_idx
            timestamps[camera_id] = frames[best_idx].timestamp
            points_by_camera[camera_id] = frames[best_idx].image_points
            pointers[camera_id] = best_idx + 1

        if len(points_by_camera) < 2:
            continue
        samples.append(
            MultiViewSample(
                sample_id=len(samples),
                timestamps=timestamps,
                image_points_by_camera=points_by_camera,
            )
        )
    return samples
