from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares

from wand_samples import MultiViewSample


@dataclass(frozen=True)
class BundleAdjustmentResult:
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]]
    wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]]
    initial_median_reproj_error_px: float
    final_median_reproj_error_px: float
    final_p90_reproj_error_px: float
    optimizer_iterations: int
    optimizer_cost: float


def _project(
    camera: Any,
    camera_pose: Tuple[np.ndarray, np.ndarray],
    wand_pose: Tuple[np.ndarray, np.ndarray],
    object_points_m: np.ndarray,
) -> np.ndarray:
    R_c, t_c = camera_pose
    R_w, t_w = wand_pose
    R_oc = R_c @ R_w
    t_oc = (R_c @ t_w.reshape(3, 1) + t_c.reshape(3, 1)).reshape(3)
    rvec, _ = cv2.Rodrigues(R_oc)
    proj, _ = cv2.projectPoints(
        object_points_m.astype(np.float64),
        rvec.reshape(3, 1),
        t_oc.reshape(3, 1),
        camera.intrinsic_matrix.astype(np.float64),
        camera.distortion_coeffs.astype(np.float64),
    )
    return proj.reshape(-1, 2)


def _flatten_pose_map(
    camera_ids: Sequence[str],
    sample_ids: Sequence[int],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]],
) -> np.ndarray:
    params: List[float] = []
    for camera_id in camera_ids:
        R, t = camera_poses[camera_id]
        rotvec, _ = cv2.Rodrigues(R)
        params.extend(rotvec.reshape(3).tolist())
        params.extend(t.reshape(3).tolist())
    for sample_id in sample_ids:
        R, t = wand_poses[sample_id]
        rotvec, _ = cv2.Rodrigues(R)
        params.extend(rotvec.reshape(3).tolist())
        params.extend(t.reshape(3).tolist())
    return np.array(params, dtype=np.float64)


def _unflatten_pose_map(
    params: np.ndarray,
    camera_ids: Sequence[str],
    sample_ids: Sequence[int],
    fixed_camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    reference_camera_id: str,
) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], Dict[int, Tuple[np.ndarray, np.ndarray]]]:
    out_camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        reference_camera_id: fixed_camera_poses[reference_camera_id]
    }
    out_wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    idx = 0
    for camera_id in camera_ids:
        if camera_id == reference_camera_id:
            continue
        rotvec = params[idx : idx + 3]
        trans = params[idx + 3 : idx + 6]
        idx += 6
        R, _ = cv2.Rodrigues(rotvec.reshape(3, 1))
        out_camera_poses[camera_id] = (R, trans.reshape(3))
    for sample_id in sample_ids:
        rotvec = params[idx : idx + 3]
        trans = params[idx + 3 : idx + 6]
        idx += 6
        R, _ = cv2.Rodrigues(rotvec.reshape(3, 1))
        out_wand_poses[sample_id] = (R, trans.reshape(3))
    return out_camera_poses, out_wand_poses


def _flatten_wand_pose_map(
    sample_ids: Sequence[int],
    wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]],
    scale: float,
) -> np.ndarray:
    params: List[float] = [float(np.log(max(scale, 1e-9)))]
    for sample_id in sample_ids:
        R, t = wand_poses[sample_id]
        rotvec, _ = cv2.Rodrigues(R)
        params.extend(rotvec.reshape(3).tolist())
        params.extend(t.reshape(3).tolist())
    return np.array(params, dtype=np.float64)


def _unflatten_wand_pose_map(
    params: np.ndarray,
    sample_ids: Sequence[int],
) -> Tuple[float, Dict[int, Tuple[np.ndarray, np.ndarray]]]:
    scale = float(np.exp(params[0]))
    out_wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    idx = 1
    for sample_id in sample_ids:
        rotvec = params[idx : idx + 3]
        trans = params[idx + 3 : idx + 6]
        idx += 6
        R, _ = cv2.Rodrigues(rotvec.reshape(3, 1))
        out_wand_poses[sample_id] = (R, trans.reshape(3))
    return scale, out_wand_poses


def _scale_camera_poses(
    camera_poses_similarity: Dict[str, Tuple[np.ndarray, np.ndarray]],
    scale: float,
) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    out: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for camera_id, (R, t) in camera_poses_similarity.items():
        center = (-(R.T @ t.reshape(3, 1))).reshape(3)
        center_scaled = center * float(scale)
        out[camera_id] = (R.astype(np.float64), (-(R @ center_scaled.reshape(3, 1))).reshape(3))
    return out


def _compute_residuals(
    camera_params: Dict[str, Any],
    samples: Sequence[MultiViewSample],
    camera_poses: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_poses: Dict[int, Tuple[np.ndarray, np.ndarray]],
    object_points_m: np.ndarray,
) -> np.ndarray:
    residuals: List[np.ndarray] = []
    for sample in samples:
        wand_pose = wand_poses[sample.sample_id]
        for camera_id, image_points in sample.image_points_by_camera.items():
            proj = _project(camera_params[camera_id], camera_poses[camera_id], wand_pose, object_points_m)
            residuals.append((proj - image_points).reshape(-1))
    if not residuals:
        return np.zeros(0, dtype=np.float64)
    return np.concatenate(residuals, axis=0).astype(np.float64)


def run_joint_bundle_adjustment(
    camera_params: Dict[str, Any],
    reference_camera_id: str,
    samples: Sequence[MultiViewSample],
    object_points_m: np.ndarray,
    camera_poses_init: Dict[str, Tuple[np.ndarray, np.ndarray]],
    wand_poses_init: Dict[int, Tuple[np.ndarray, np.ndarray]],
    translation_max_delta_m: float = 0.05,
) -> BundleAdjustmentResult:
    sample_ids = [sample.sample_id for sample in samples]
    camera_ids = sorted(camera_params.keys())
    variable_camera_ids = [camera_id for camera_id in camera_ids if camera_id != reference_camera_id]

    init_camera_poses = {camera_id: camera_poses_init[camera_id] for camera_id in camera_ids}
    init_wand_poses = {sample_id: wand_poses_init[sample_id] for sample_id in sample_ids}

    initial_residuals = _compute_residuals(
        camera_params,
        samples,
        init_camera_poses,
        init_wand_poses,
        object_points_m,
    )
    initial_abs = np.abs(initial_residuals)
    initial_median = float(np.median(initial_abs)) if initial_abs.size else 0.0

    x0 = _flatten_pose_map(variable_camera_ids, sample_ids, init_camera_poses, init_wand_poses)
    lower = np.full_like(x0, -np.inf, dtype=np.float64)
    upper = np.full_like(x0, np.inf, dtype=np.float64)
    for i, _camera_id in enumerate(variable_camera_ids):
        base = i * 6
        lower[base + 3 : base + 6] = x0[base + 3 : base + 6] - translation_max_delta_m
        upper[base + 3 : base + 6] = x0[base + 3 : base + 6] + translation_max_delta_m

    def residual_fn(x: np.ndarray) -> np.ndarray:
        camera_poses, wand_poses = _unflatten_pose_map(
            x,
            variable_camera_ids,
            sample_ids,
            init_camera_poses,
            reference_camera_id,
        )
        return _compute_residuals(camera_params, samples, camera_poses, wand_poses, object_points_m)

    result = least_squares(
        residual_fn,
        x0,
        bounds=(lower, upper),
        method="trf",
        loss="cauchy",
        f_scale=2.0,
        max_nfev=200,
    )
    camera_poses, wand_poses = _unflatten_pose_map(
        result.x,
        variable_camera_ids,
        sample_ids,
        init_camera_poses,
        reference_camera_id,
    )
    final_residuals = _compute_residuals(camera_params, samples, camera_poses, wand_poses, object_points_m)
    final_abs = np.abs(final_residuals)
    final_median = float(np.median(final_abs)) if final_abs.size else 0.0
    final_p90 = float(np.percentile(final_abs, 90)) if final_abs.size else 0.0

    return BundleAdjustmentResult(
        camera_poses=camera_poses,
        wand_poses=wand_poses,
        initial_median_reproj_error_px=initial_median,
        final_median_reproj_error_px=final_median,
        final_p90_reproj_error_px=final_p90,
        optimizer_iterations=int(result.nfev),
        optimizer_cost=float(result.cost),
    )


def run_scale_bundle_adjustment(
    camera_params: Dict[str, Any],
    reference_camera_id: str,
    samples: Sequence[MultiViewSample],
    object_points_m: np.ndarray,
    camera_poses_similarity: Dict[str, Tuple[np.ndarray, np.ndarray]],
    scale_init: float,
    wand_poses_init: Dict[int, Tuple[np.ndarray, np.ndarray]],
) -> BundleAdjustmentResult:
    sample_ids = [sample.sample_id for sample in samples]
    init_wand_poses = {sample_id: wand_poses_init[sample_id] for sample_id in sample_ids}
    init_camera_poses = _scale_camera_poses(camera_poses_similarity, scale_init)
    initial_residuals = _compute_residuals(
        camera_params,
        samples,
        init_camera_poses,
        init_wand_poses,
        object_points_m,
    )
    initial_abs = np.abs(initial_residuals)
    initial_median = float(np.median(initial_abs)) if initial_abs.size else 0.0

    x0 = _flatten_wand_pose_map(sample_ids, init_wand_poses, scale_init)
    lower = np.full_like(x0, -np.inf, dtype=np.float64)
    upper = np.full_like(x0, np.inf, dtype=np.float64)
    lower[0] = float(np.log(max(scale_init / 4.0, 1e-9)))
    upper[0] = float(np.log(max(scale_init * 4.0, 1e-9)))

    def residual_fn(x: np.ndarray) -> np.ndarray:
        scale, wand_poses = _unflatten_wand_pose_map(x, sample_ids)
        camera_poses = _scale_camera_poses(camera_poses_similarity, scale)
        return _compute_residuals(camera_params, samples, camera_poses, wand_poses, object_points_m)

    result = least_squares(
        residual_fn,
        x0,
        bounds=(lower, upper),
        method="trf",
        loss="cauchy",
        f_scale=2.0,
        max_nfev=200,
    )
    scale_out, wand_poses = _unflatten_wand_pose_map(result.x, sample_ids)
    camera_poses = _scale_camera_poses(camera_poses_similarity, scale_out)
    final_residuals = _compute_residuals(camera_params, samples, camera_poses, wand_poses, object_points_m)
    final_abs = np.abs(final_residuals)
    final_median = float(np.median(final_abs)) if final_abs.size else 0.0
    final_p90 = float(np.percentile(final_abs, 90)) if final_abs.size else 0.0
    return BundleAdjustmentResult(
        camera_poses=camera_poses,
        wand_poses=wand_poses,
        initial_median_reproj_error_px=initial_median,
        final_median_reproj_error_px=final_median,
        final_p90_reproj_error_px=final_p90,
        optimizer_iterations=int(result.nfev),
        optimizer_cost=float(result.cost),
    )
