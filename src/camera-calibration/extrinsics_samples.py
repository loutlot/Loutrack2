from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence

import numpy as np

from extrinsics_capture import PoseCaptureObservation


@dataclass(frozen=True)
class PoseCaptureSample:
    sample_id: int
    timestamps: Dict[str, int]
    image_points_by_camera: Dict[str, np.ndarray]
    quality: Dict[str, float]


def build_pose_multiview_samples(
    observations_by_camera: Dict[str, Sequence[PoseCaptureObservation]],
    reference_camera_id: str,
    pair_window_us: int,
    min_quality: float = 0.25,
) -> List[PoseCaptureSample]:
    if reference_camera_id not in observations_by_camera:
        return []

    refs = list(observations_by_camera[reference_camera_id])
    refs.sort(key=lambda item: item.timestamp)
    cursors: Dict[str, int] = {camera_id: 0 for camera_id in observations_by_camera.keys()}
    samples: List[PoseCaptureSample] = []

    for ref in refs:
        if ref.quality < min_quality:
            continue
        timestamps: Dict[str, int] = {reference_camera_id: int(ref.timestamp)}
        points: Dict[str, np.ndarray] = {reference_camera_id: ref.image_point.astype(np.float64)}
        quality_rows: Dict[str, float] = {reference_camera_id: float(ref.quality)}
        for camera_id, rows in observations_by_camera.items():
            if camera_id == reference_camera_id:
                continue
            idx = cursors[camera_id]
            while idx < len(rows) and rows[idx].timestamp < ref.timestamp - pair_window_us:
                idx += 1
            best = None
            best_idx = idx
            scan = idx
            while scan < len(rows):
                obs = rows[scan]
                delta = abs(int(obs.timestamp) - int(ref.timestamp))
                if obs.timestamp > ref.timestamp + pair_window_us:
                    break
                if obs.quality >= min_quality and (best is None or delta < best[0]):
                    best = (delta, obs)
                    best_idx = scan
                scan += 1
            if best is None:
                cursors[camera_id] = idx
                continue
            # Keep per-camera matching monotonic and avoid reusing the same observation.
            cursors[camera_id] = best_idx + 1
            obs = best[1]
            timestamps[camera_id] = int(obs.timestamp)
            points[camera_id] = obs.image_point.astype(np.float64)
            quality_rows[camera_id] = float(obs.quality)
        if len(points) < 2:
            continue
        span = max(timestamps.values()) - min(timestamps.values())
        if span > pair_window_us:
            continue
        samples.append(
            PoseCaptureSample(
                sample_id=len(samples),
                timestamps=timestamps,
                image_points_by_camera=points,
                quality={
                    "visible_camera_count": float(len(points)),
                    "span_us": float(span),
                    "mean_observation_quality": float(np.mean(list(quality_rows.values()))),
                },
            )
        )
    return samples
