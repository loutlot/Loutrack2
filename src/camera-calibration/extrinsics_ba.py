from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from extrinsics_samples import PoseCaptureSample


TARGET_PARALLAX = 0.02


@dataclass(frozen=True)
class BAResult:
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]
    points_3d: Dict[int, np.ndarray]
    iterations: int
    cost: float
    initial_cost: float
    seed_sample_count: int
    weighted_sample_count: float
    dropped_sample_count: int
    median_seed_triangulation_angle_deg: float


def _camera_center(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    return (-(rotation.T @ translation.reshape(3, 1))).reshape(3)


def _triangulate_multiview_point(
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    observations: Sequence[Tuple[str, np.ndarray]],
) -> np.ndarray | None:
    if len(observations) < 2:
        return None
    rows: List[np.ndarray] = []
    for camera_id, image_point in observations:
        camera = camera_params[camera_id]
        rotation, translation = camera_poses[camera_id]
        projection = np.hstack([rotation, translation.reshape(3, 1)])
        normalized = cv2.undistortPoints(
            np.asarray(image_point, dtype=np.float64).reshape(1, 1, 2),
            camera.intrinsic_matrix.astype(np.float64),
            camera.distortion_coeffs.astype(np.float64),
        ).reshape(2)
        rows.append(normalized[0] * projection[2] - projection[0])
        rows.append(normalized[1] * projection[2] - projection[1])
    system = np.stack(rows, axis=0)
    _, _, vt = np.linalg.svd(system, full_matrices=False)
    point_h = vt[-1]
    if abs(float(point_h[3])) < 1e-12:
        return None
    return (point_h[:3] / point_h[3]).astype(np.float64)


def _sample_triangulation_angles(
    point: np.ndarray,
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    camera_ids: Sequence[str],
) -> np.ndarray:
    centers = {camera_id: _camera_center(*camera_poses[camera_id]) for camera_id in camera_ids}
    angles: List[float] = []
    for camera_a, camera_b in combinations(sorted(camera_ids), 2):
        ray_a = point - centers[camera_a]
        ray_b = point - centers[camera_b]
        norm_a = float(np.linalg.norm(ray_a))
        norm_b = float(np.linalg.norm(ray_b))
        if norm_a <= 1e-12 or norm_b <= 1e-12:
            continue
        cos_angle = float(np.clip(np.dot(ray_a, ray_b) / (norm_a * norm_b), -1.0, 1.0))
        angles.append(float(np.degrees(np.arccos(cos_angle))))
    return np.asarray(angles, dtype=np.float64)


def _positive_depth_ratio(
    point: np.ndarray,
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    camera_ids: Sequence[str],
) -> float:
    positives = 0
    total = 0
    for camera_id in camera_ids:
        rotation, translation = camera_poses[camera_id]
        depth = float((rotation @ point.reshape(3, 1) + translation.reshape(3, 1))[2, 0])
        positives += int(depth > 0.0)
        total += 1
    return float(positives / total) if total else 0.0


def _sample_weight(sample: PoseCaptureSample, pair_window_us: int) -> float:
    visible_camera_count = float(sample.quality.get("visible_camera_count", 2.0) or 2.0)
    mean_quality = float(sample.quality.get("mean_observation_quality", 1.0) or 1.0)
    span_us = float(sample.quality.get("span_us", 0.0) or 0.0)
    parallax_proxy = float(sample.quality.get("parallax_proxy", TARGET_PARALLAX) or TARGET_PARALLAX)
    visible_camera_count_weight = np.sqrt(max(1.0, visible_camera_count - 1.0))
    quality_weight = float(np.clip(mean_quality, 0.2, 1.0))
    span_weight = 1.0 / (1.0 + (span_us / max(float(pair_window_us), 1.0)))
    parallax_weight = float(np.clip(parallax_proxy / TARGET_PARALLAX, 0.5, 2.0))
    gate_weight = float(np.clip(sample.quality.get("gate_weight", 1.0) or 1.0, 0.25, 1.0))
    return float(visible_camera_count_weight * quality_weight * span_weight * parallax_weight * gate_weight)


def _triangulate_seed_points(
    camera_params: Dict[str, Any],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    samples: Sequence[PoseCaptureSample],
) -> tuple[Dict[int, np.ndarray], Dict[int, float], int]:
    out: Dict[int, np.ndarray] = {}
    sample_angles: Dict[int, float] = {}
    dropped = 0
    for sample in samples:
        cameras = [
            camera_id
            for camera_id in sorted(sample.image_points_by_camera.keys())
            if camera_id in camera_poses and camera_id in camera_params
        ]
        if len(cameras) < 2:
            dropped += 1
            continue
        point = _triangulate_multiview_point(
            camera_params,
            camera_poses,
            [(camera_id, sample.image_points_by_camera[camera_id]) for camera_id in cameras],
        )
        if point is None:
            dropped += 1
            continue
        if _positive_depth_ratio(point, camera_poses, cameras) < 1.0:
            dropped += 1
            continue
        angles = _sample_triangulation_angles(point, camera_poses, cameras)
        if angles.size == 0:
            dropped += 1
            continue
        out[sample.sample_id] = point
        sample_angles[sample.sample_id] = float(np.median(angles))
    return out, sample_angles, dropped


def _project(camera: Any, rotation: np.ndarray, translation: np.ndarray, point: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(rotation)
    proj, _ = cv2.projectPoints(
        point.reshape(1, 3).astype(np.float64),
        rvec.reshape(3, 1),
        translation.reshape(3, 1).astype(np.float64),
        camera.intrinsic_matrix.astype(np.float64),
        camera.distortion_coeffs.astype(np.float64),
    )
    return proj.reshape(2)


def run_pose_bundle_adjustment(
    camera_params: Dict[str, Any],
    reference_camera_id: str,
    samples: Sequence[PoseCaptureSample],
    camera_poses_init: Dict[str, Tuple[np.ndarray, np.ndarray]],
    pair_window_us: int,
    loss: str = "huber",
) -> BAResult:
    camera_ids = [camera_id for camera_id in sorted(camera_params.keys()) if camera_id != reference_camera_id]
    points_init, seed_angles, dropped_samples = _triangulate_seed_points(camera_params, camera_poses_init, samples)
    if not points_init:
        raise ValueError("No triangulatable samples for BA")

    point_ids = sorted(points_init.keys())
    point_index = {sample_id: idx for idx, sample_id in enumerate(point_ids)}

    cam_params = []
    for camera_id in camera_ids:
        rmat, tvec = camera_poses_init[camera_id]
        rvec, _ = cv2.Rodrigues(rmat)
        cam_params.append(np.concatenate([rvec.reshape(3), tvec.reshape(3)]))
    x0_cam = np.concatenate(cam_params) if cam_params else np.empty((0,), dtype=np.float64)
    x0_pts = np.concatenate([points_init[sample_id].reshape(3) for sample_id in point_ids])
    x0 = np.concatenate([x0_cam, x0_pts])

    obs_rows: List[Tuple[str, int, np.ndarray, float]] = []
    weighted_sample_count = 0.0
    for sample in samples:
        if sample.sample_id not in point_index:
            continue
        weight = _sample_weight(sample, pair_window_us)
        weighted_sample_count += weight
        sqrt_weight = float(np.sqrt(max(weight, 1e-12)))
        for camera_id, image_point in sample.image_points_by_camera.items():
            obs_rows.append((camera_id, sample.sample_id, image_point.astype(np.float64), sqrt_weight))

    def unpack(x: np.ndarray) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[int, np.ndarray]]:
        poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
            reference_camera_id: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
        }
        cursor = 0
        for camera_id in camera_ids:
            rvec = x[cursor : cursor + 3]
            tvec = x[cursor + 3 : cursor + 6]
            cursor += 6
            rmat, _ = cv2.Rodrigues(rvec.reshape(3, 1))
            poses[camera_id] = (rmat, tvec)
        points: Dict[int, np.ndarray] = {}
        for sample_id in point_ids:
            points[sample_id] = x[cursor : cursor + 3]
            cursor += 3
        return poses, points

    def residuals(x: np.ndarray) -> np.ndarray:
        poses, points = unpack(x)
        rows: List[float] = []
        for camera_id, sample_id, image_point, sqrt_weight in obs_rows:
            if sample_id not in points:
                continue
            camera = camera_params[camera_id]
            rotation, translation = poses[camera_id]
            projected = _project(camera, rotation, translation, points[sample_id])
            diff = (projected - image_point) * sqrt_weight
            rows.extend([float(diff[0]), float(diff[1])])
        return np.asarray(rows, dtype=np.float64)

    initial_residual = residuals(x0)
    result = least_squares(
        residuals,
        x0,
        method="trf",
        loss=loss,
        f_scale=2.0,
        max_nfev=200,
    )
    poses_out, points_out = unpack(result.x)
    return BAResult(
        camera_poses=poses_out,
        points_3d=points_out,
        iterations=int(result.nfev),
        cost=float(result.cost),
        initial_cost=float(np.sum(initial_residual**2) * 0.5),
        seed_sample_count=len(points_init),
        weighted_sample_count=float(weighted_sample_count),
        dropped_sample_count=int(dropped_samples),
        median_seed_triangulation_angle_deg=float(np.median(np.asarray(list(seed_angles.values()), dtype=np.float64))) if seed_angles else 0.0,
    )
