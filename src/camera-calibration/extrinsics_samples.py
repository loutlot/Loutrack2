from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations, product
from typing import Any, Dict, List, Sequence

import cv2
import numpy as np

from extrinsics_capture import PoseCaptureObservation


@dataclass(frozen=True)
class PoseCaptureSample:
    sample_id: int
    timestamps: Dict[str, int]
    image_points_by_camera: Dict[str, np.ndarray]
    quality: Dict[str, float]


def _candidate_observations(
    rows: Sequence[PoseCaptureObservation],
    anchor_timestamp: int,
    pair_window_us: int,
    min_quality: float,
    limit: int,
) -> List[PoseCaptureObservation]:
    candidates: List[PoseCaptureObservation] = []
    for obs in rows:
        if abs(int(obs.timestamp) - int(anchor_timestamp)) > pair_window_us:
            continue
        if obs.quality < min_quality:
            continue
        candidates.append(obs)
    candidates.sort(
        key=lambda obs: (
            abs(int(obs.timestamp) - int(anchor_timestamp)),
            -float(obs.quality),
            -float(obs.blob_area),
        )
    )
    return candidates[: max(1, limit)]


def _normalize_point(point: np.ndarray, camera: Any | None) -> np.ndarray:
    arr = np.asarray(point, dtype=np.float64).reshape(1, 1, 2)
    if camera is None:
        return arr.reshape(2)
    return cv2.undistortPoints(
        arr,
        camera.intrinsic_matrix.astype(np.float64),
        camera.distortion_coeffs.astype(np.float64),
    ).reshape(2)


def _parallax_proxy(
    points_by_camera: Dict[str, np.ndarray],
    camera_params: Dict[str, Any] | None,
) -> float:
    normalized: Dict[str, np.ndarray] = {}
    for camera_id, point in points_by_camera.items():
        camera = None if camera_params is None else camera_params.get(camera_id)
        normalized[camera_id] = _normalize_point(point, camera)
    distances = [
        float(np.linalg.norm(normalized[a] - normalized[b]))
        for a, b in combinations(sorted(normalized.keys()), 2)
    ]
    return float(np.median(np.asarray(distances, dtype=np.float64))) if distances else 0.0


def _sample_quality(
    timestamps: Dict[str, int],
    points_by_camera: Dict[str, np.ndarray],
    observations_by_camera: Dict[str, PoseCaptureObservation],
    camera_params: Dict[str, Any] | None,
) -> Dict[str, float]:
    qualities = [float(obs.quality) for obs in observations_by_camera.values()]
    span_us = float(max(timestamps.values()) - min(timestamps.values()))
    return {
        "visible_camera_count": float(len(points_by_camera)),
        "span_us": span_us,
        "mean_observation_quality": float(np.mean(np.asarray(qualities, dtype=np.float64))) if qualities else 0.0,
        "parallax_proxy": _parallax_proxy(points_by_camera, camera_params),
    }


def build_pose_multiview_samples(
    observations_by_camera: Dict[str, Sequence[PoseCaptureObservation]],
    reference_camera_id: str,
    pair_window_us: int,
    camera_params: Dict[str, Any] | None = None,
    min_quality: float = 0.05,
    reuse_suppression_us: int | None = None,
    max_candidates_per_camera: int = 3,
) -> List[PoseCaptureSample]:
    if reference_camera_id not in observations_by_camera:
        return []

    refs = sorted(observations_by_camera[reference_camera_id], key=lambda item: item.timestamp)
    suppression_us = reuse_suppression_us if reuse_suppression_us is not None else max(1, pair_window_us // 2)
    other_camera_ids = [
        camera_id for camera_id in sorted(observations_by_camera.keys()) if camera_id != reference_camera_id
    ]
    last_selected_timestamp: Dict[str, int] = {}
    samples: List[PoseCaptureSample] = []

    for ref in refs:
        if ref.quality < min_quality:
            continue
        candidate_lists: List[List[PoseCaptureObservation | None]] = []
        for camera_id in other_camera_ids:
            candidates = _candidate_observations(
                observations_by_camera.get(camera_id, ()),
                anchor_timestamp=int(ref.timestamp),
                pair_window_us=pair_window_us,
                min_quality=min_quality,
                limit=max_candidates_per_camera,
            )
            candidate_lists.append(candidates or [None])

        best_obs_by_camera: Dict[str, PoseCaptureObservation] | None = None
        best_key: tuple[float, float, float, float] | None = None
        for combo in product(*candidate_lists) if candidate_lists else [()]:
            obs_by_camera: Dict[str, PoseCaptureObservation] = {reference_camera_id: ref}
            for camera_id, obs in zip(other_camera_ids, combo):
                if obs is not None:
                    obs_by_camera[camera_id] = obs
            if len(obs_by_camera) < 2:
                continue
            timestamps = {camera_id: int(obs.timestamp) for camera_id, obs in obs_by_camera.items()}
            span = float(max(timestamps.values()) - min(timestamps.values()))
            if span > pair_window_us:
                continue
            mean_quality = float(np.mean([float(obs.quality) for obs in obs_by_camera.values()]))
            visible_camera_count = float(len(obs_by_camera))
            suppression_penalty = float(
                sum(
                    1
                    for camera_id, obs in obs_by_camera.items()
                    if camera_id in last_selected_timestamp
                    and abs(int(obs.timestamp) - int(last_selected_timestamp[camera_id])) < suppression_us
                )
            )
            key = (span, suppression_penalty, -mean_quality, -visible_camera_count)
            if best_key is None or key < best_key:
                best_key = key
                best_obs_by_camera = obs_by_camera

        if best_obs_by_camera is None:
            continue

        timestamps = {camera_id: int(obs.timestamp) for camera_id, obs in best_obs_by_camera.items()}
        points = {
            camera_id: np.asarray(obs.image_point, dtype=np.float64)
            for camera_id, obs in best_obs_by_camera.items()
        }
        quality = _sample_quality(
            timestamps=timestamps,
            points_by_camera=points,
            observations_by_camera=best_obs_by_camera,
            camera_params=camera_params,
        )
        samples.append(
            PoseCaptureSample(
                sample_id=len(samples),
                timestamps=timestamps,
                image_points_by_camera=points,
                quality=quality,
            )
        )
        for camera_id, obs in best_obs_by_camera.items():
            last_selected_timestamp[camera_id] = int(obs.timestamp)

    return samples
