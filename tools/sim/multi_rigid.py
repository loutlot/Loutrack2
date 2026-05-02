"""Simulation utilities for synthetic camera projection.

This module also provides a small closed-loop simulation runner so the host
pipeline can be exercised deterministically without UDP receivers.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
from datetime import datetime
import socket
import threading
import time
from pathlib import Path
import argparse
import copy
import json
import sys
from typing import Any, Iterable

import cv2
import numpy as np

from host.geo import CameraParams, GeometryPipeline, Triangulator, create_dummy_calibration
from host.logger import FrameLogger
from host.replay import FrameReplay
from host.receiver import Frame, PairedFrames
from host.rigid import (
    CHEST_PATTERN,
    HEAD_PATTERN,
    LEFT_FOOT_PATTERN,
    MarkerPattern,
    RIGHT_FOOT_PATTERN,
    RigidBodyEstimator,
    WAIST_PATTERN,
    marker_pattern_from_points,
)
from host.visualize import TrackingVisualizer
from host.pipeline import TrackingPipeline, _variant_has_geo_hotpath_optimizations


DEFAULT_MVP_CAMERA_IDS = ("pi-cam-01", "pi-cam-02")
DEFAULT_GENERATED_CAMERA_IDS = ("pi-cam-01", "pi-cam-02", "pi-cam-03", "pi-cam-04")
DEFAULT_MVP_RIGIDS = ("waist", "wand")
DEFAULT_BODY_RIGIDS = ("head", "waist", "chest", "left_foot", "right_foot")
DESIGN_5MARKER_LAYOUT = "design_5marker_seed"
BODY_MOUNT_SCENARIOS = {
    "five_rigid_body_occlusion_v1",
    "five_rigid_body_occlusion_relaxed_v1",
}
FIVE_RIGID_BODY_SCENARIOS = {
    "five_rigid_dance_occlusion",
    "five_rigid_dance_hard_occlusion_v1",
    "five_rigid_dance_swap_red_v1",
    *BODY_MOUNT_SCENARIOS,
}


_DESIGN_5MARKER_POINTS_MM: dict[str, list[list[float]]] = {
    "head": [
        [39.5, -48.1, 18.9],
        [39.0, 26.9, 29.1],
        [-34.5, 53.8, 12.0],
        [-8.4, -63.9, 8.3],
        [64.0, 6.4, 9.2],
    ],
    "waist": [
        [18.0, -50.0, 37.5],
        [0.6, 61.8, 20.2],
        [-6.6, -13.0, 63.3],
        [22.9, -24.1, 55.9],
        [58.2, -28.6, 4.8],
    ],
    "chest": [
        [3.4, 39.5, 24.7],
        [17.6, -30.4, 54.7],
        [24.4, 58.4, 14.7],
        [63.9, -1.0, 11.8],
        [-23.3, -42.3, 32.6],
    ],
    "left_foot": [
        [-29.7, -52.0, 25.3],
        [62.1, -10.7, 16.1],
        [0.7, 47.4, 44.4],
        [-17.3, -17.8, 49.1],
        [-63.6, -3.5, 13.0],
    ],
    "right_foot": [
        [7.2, 45.2, 46.1],
        [-44.2, 34.2, 33.1],
        [24.0, -22.4, 56.1],
        [-38.0, -5.0, 52.5],
        [-59.5, 24.3, 9.8],
    ],
}


_DESIGN_5MARKER_POLICIES: dict[str, dict[str, Any]] = {
    "head": {
        "strong_4_subsets": [[0, 2, 3, 4]],
        "strong_3_subsets": [[2, 3, 4]],
    },
    "waist": {
        "strong_4_subsets": [[0, 1, 2, 3]],
        "strong_3_subsets": [[0, 2, 3]],
    },
    "chest": {
        "strong_4_subsets": [[0, 1, 2, 3]],
        "strong_3_subsets": [[0, 2, 3]],
    },
    "left_foot": {
        "strong_4_subsets": [[1, 2, 3, 4]],
        "strong_3_subsets": [[1, 2, 4]],
    },
    "right_foot": {
        "strong_4_subsets": [[0, 1, 2, 3]],
        "strong_3_subsets": [[1, 3, 4]],
    },
}


def _design_5marker_patterns() -> list[MarkerPattern]:
    patterns: list[MarkerPattern] = []
    for name in DEFAULT_BODY_RIGIDS:
        policy = {
            "boot_min_markers": 4,
            "reacquire_min_markers": 4,
            "continue_min_markers": 3,
            **_DESIGN_5MARKER_POLICIES[name],
        }
        patterns.append(
            marker_pattern_from_points(
                name,
                np.asarray(_DESIGN_5MARKER_POINTS_MM[name], dtype=np.float64) * 0.001,
                marker_diameter=0.014,
                metadata={
                    "source": "rigid_body_design_seed",
                    "marker_layout": DESIGN_5MARKER_LAYOUT,
                    "tracking_policy": policy,
                },
            )
        )
    return patterns


def _sample_gaussian_pixel_noise(
    rng: np.random.Generator,
    noise_px: float,
) -> tuple[float, float]:
    """Sample additive pixel noise for future simulation use."""
    if noise_px <= 0.0:
        return 0.0, 0.0
    dx, dy = rng.normal(0.0, noise_px, size=2)
    return float(dx), float(dy)


def verify_frame_log(path) -> dict:
    """Verify the FrameLogger log using FrameReplay utilities.

    Returns a dict with keys:
      - valid (bool)
      - camera_ids (list[str])
      - frame_count (int)
      - frame_counts_by_camera (dict[str, int])
      - error (str, optional) if invalid
    Validation rules (via FrameReplay):
      - replay.header must exist
      - replay.footer must exist
      - camera_ids derived from replay.get_cameras()
      - frame counts per camera from replay.get_frames_by_camera(cid)
      - frame_count derives from replay.frame_count
      - If footer.total_frames is present, it must equal frame_count
    """
    res: dict = {
        "valid": False,
        "camera_ids": [],
        "frame_count": 0,
        "frame_counts_by_camera": {},
    }
    try:
        replay = FrameReplay(str(path))

        # Basic structural checks
        if getattr(replay, "header", None) is None:
            res["error"] = "missing header in replay"
            return res
        if getattr(replay, "footer", None) is None:
            res["error"] = "missing footer in replay"
            return res

        # Cameras and per-camera frame counts
        camera_ids = sorted(replay.get_cameras()) if hasattr(replay, "get_cameras") else []
        res["camera_ids"] = [str(cid) for cid in camera_ids]
        frame_counts_by_camera: dict[str, int] = {}
        for cid in camera_ids:
            frames = replay.get_frames_by_camera(cid) if hasattr(replay, "get_frames_by_camera") else []
            frame_counts_by_camera[str(cid)] = int(len(frames))
        res["frame_counts_by_camera"] = frame_counts_by_camera

        # Total frame count and validation against footer
        frame_count = int(getattr(replay, "frame_count", 0))
        res["frame_count"] = frame_count
        footer_total = getattr(replay.footer, "total_frames", None) if hasattr(replay, "footer") else None
        if footer_total is not None:
            try:
                if int(footer_total) != frame_count:
                    res["error"] = (
                        f"footer total_frames {int(footer_total)} != counted {frame_count}"
                    )
                    return res
            except Exception:
                # If total_frames is not an int-able value, treat as mismatch
                res["error"] = f"invalid footer total_frames value: {footer_total}"
                return res

        res["valid"] = True
        return res
    except Exception as e:
        res["error"] = f"unexpected error: {e}"
        return res


class VirtualCamera:
    """Project world points to 2D image points using OpenCV conventions."""

    def __init__(self, camera_params: CameraParams):
        self.camera_params = camera_params
        self._rotation = np.asarray(camera_params.rotation, dtype=np.float64)
        self._translation = np.asarray(camera_params.translation, dtype=np.float64).reshape(3)
        self._rvec, _ = cv2.Rodrigues(self._rotation)

    def project_points(self, points_world: Iterable[Iterable[float]]) -> list[tuple[float, float] | None]:
        points = np.asarray(points_world, dtype=np.float64)

        if points.size == 0:
            return []
        if points.ndim == 1:
            if points.shape[0] != 3:
                raise ValueError("points_world must contain 3D points")
            points = points.reshape(1, 3)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_world must be shape (N, 3)")

        x_cam = (self._rotation @ points.T).T + self._translation
        valid_mask = x_cam[:, 2] > 0.0

        results: list[tuple[float, float] | None] = [None] * points.shape[0]
        if not np.any(valid_mask):
            return results

        projected, _ = cv2.projectPoints(
            points[valid_mask],
            self._rvec,
            self._translation,
            self.camera_params.intrinsic_matrix,
            self.camera_params.distortion_coeffs,
        )

        projected_2d = projected.reshape(-1, 2)
        valid_indices = np.flatnonzero(valid_mask)
        for idx, (x, y) in zip(valid_indices, projected_2d):
            results[int(idx)] = (float(x), float(y))

        return results

    def camera_depth(self, point_world: Iterable[float]) -> float:
        point = np.asarray(point_world, dtype=np.float64).reshape(3)
        point_camera = self._rotation @ point + self._translation
        return float(point_camera[2])


def _rotation_error_deg(r_pred: np.ndarray, r_gt: np.ndarray) -> float:
    """Rotation error in degrees between two rotation matrices."""
    r_pred = np.asarray(r_pred, dtype=np.float64).reshape(3, 3)
    r_gt = np.asarray(r_gt, dtype=np.float64).reshape(3, 3)
    cos = (np.trace(r_pred @ r_gt.T) - 1.0) / 2.0
    cos = float(np.clip(cos, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def _summary(values: list[float]) -> dict[str, float]:
    if not values:
        return {"count": 0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(np.mean(arr)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(np.max(arr)),
    }


def _max_consecutive_over(values: list[float], threshold: float) -> int:
    longest = 0
    current = 0
    for value in values:
        if float(value) > float(threshold):
            current += 1
            longest = max(longest, current)
        else:
            current = 0
    return int(longest)


def _warmup_trimmed_summary(
    values: list[float],
    *,
    warmup_samples: int,
) -> dict[str, Any]:
    warmup = min(max(0, int(warmup_samples)), len(values))
    trimmed = list(values[warmup:])
    summary = _summary(trimmed)
    summary["warmup_samples"] = int(warmup)
    return summary


def _gui_live_rigid_stabilization() -> dict[str, Any]:
    return {
        "object_conditioned_gating": True,
        "subset_ransac": False,
        "reacquire_guard_shadow_enabled": False,
        "reacquire_guard_event_logging": False,
        "reacquire_guard_enforced": True,
        "object_gating_enforced": True,
        "object_gating_pixel_max_px": 4.5,
        "pose_continuity_guard_enabled": True,
        "pose_continuity_guard_enforced": True,
        "pose_continuity_max_rotation_deg": 90.0,
        "pose_continuity_max_angular_velocity_deg_s": 2500.0,
        "pose_continuity_max_angular_accel_deg_s2": 200000.0,
        "position_continuity_guard_enabled": True,
        "position_continuity_guard_enforced": True,
        "position_continuity_max_accel_m_s2": 60.0,
        "position_continuity_max_velocity_m_s": 8.0,
    }


def _rigid_stabilization_for_profile(profile: str) -> dict[str, Any] | None:
    normalized = str(profile or "pipeline_default").strip()
    if normalized == "pipeline_default":
        return None
    if normalized == "gui_live":
        return _gui_live_rigid_stabilization()
    raise ValueError(
        "rigid_stabilization_profile must be one of: pipeline_default, gui_live"
    )


def _rigid_stabilization_for_config(
    config: "MultiRigidScenarioConfig",
) -> dict[str, Any] | None:
    base = _rigid_stabilization_for_profile(config.rigid_stabilization_profile)
    overrides = {
        str(key): value
        for key, value in (config.rigid_stabilization_overrides or {}).items()
        if value is not None
    }
    if not overrides:
        return base
    merged = dict(base or {})
    merged.update(overrides)
    return merged


def _default_rigids_for_scenario(scenario: str | None) -> tuple[str, ...]:
    if str(scenario or "") in FIVE_RIGID_BODY_SCENARIOS:
        return DEFAULT_BODY_RIGIDS
    return DEFAULT_MVP_RIGIDS


def _default_marker_layout_for_scenario(scenario: str | None, marker_layout: str) -> str:
    if (
        str(scenario or "") in FIVE_RIGID_BODY_SCENARIOS
        and str(marker_layout or "current_4marker") == "current_4marker"
    ):
        return DESIGN_5MARKER_LAYOUT
    return str(marker_layout or "current_4marker")


def _default_camera_rig_source_for_scenario(scenario: str | None, source: str | None) -> str:
    normalized = str(source or "auto").strip()
    if normalized != "auto":
        return normalized
    if str(scenario or "") in FIVE_RIGID_BODY_SCENARIOS:
        return "cube_top_2_4m_aim_center"
    return "real_2cam"


def _latest_pipeline_stage_ms(pipeline: TrackingPipeline, name: str) -> float:
    stage_lock = getattr(pipeline, "_stage_lock", None)
    stage_ms = getattr(pipeline, "_stage_ms", {})
    if stage_lock is None:
        bucket = stage_ms.get(name) if isinstance(stage_ms, dict) else None
        return float(bucket[-1]) if bucket else 0.0
    with stage_lock:
        bucket = stage_ms.get(name) if isinstance(stage_ms, dict) else None
        return float(bucket[-1]) if bucket else 0.0


def _rotz(theta_rad: float) -> np.ndarray:
    c = float(np.cos(theta_rad))
    s = float(np.sin(theta_rad))
    return np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


@dataclass(frozen=True)
class MultiRigidScenarioConfig:
    seed: int = 0
    camera_ids: tuple[str, ...] = DEFAULT_MVP_CAMERA_IDS
    frames: int = 600
    fps: float = 118.0
    rigid_names: tuple[str, ...] = DEFAULT_MVP_RIGIDS
    scenario: str = "waist_wand_static_clean"
    trajectory_name: str = "static"
    noise_px: float = 0.0
    false_blobs_per_camera: int = 0
    marker_dropout_prob: float = 0.0
    camera_dropout_prob: float = 0.0
    timestamp_jitter_us: int = 0
    occlusion_profile: str = "none"
    motion_profile: str = "linear"
    body_interaction_profile: str = "waist_wand"
    max_velocity_mps: float = 0.8
    max_angular_velocity_deg_s: float = 180.0
    body_mount_blob_merge_factor: float = 0.0
    marker_layout: str = "current_4marker"
    camera_rig_source: str = "auto"
    calibration_dir: str = "calibration"
    rigids_path: str = "calibration/tracking_rigids.json"
    pipeline_variant: str = "fast_ABCDHRF"
    subset_diagnostics_mode: str = "off"
    rigid_stabilization_profile: str = "pipeline_default"
    rigid_stabilization_overrides: dict[str, Any] | None = None


@dataclass(frozen=True)
class SyntheticBlobOwner:
    timestamp: int
    camera_id: str
    emitted_blob_index: int
    rigid_name: str
    marker_index: int | None
    synthetic_blob_id: str
    merged_owners: tuple[tuple[str, int], ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": int(self.timestamp),
            "camera_id": self.camera_id,
            "emitted_blob_index": int(self.emitted_blob_index),
            "rigid_name": self.rigid_name,
            "marker_index": None if self.marker_index is None else int(self.marker_index),
            "synthetic_blob_id": self.synthetic_blob_id,
            "merged_owners": [
                {"rigid_name": str(rigid_name), "marker_index": int(marker_index)}
                for rigid_name, marker_index in self.merged_owners
            ],
        }


@dataclass(frozen=True)
class SyntheticFrameSample:
    paired: PairedFrames
    gt_poses: dict[str, dict[str, Any]]
    ownership_ledger: dict[tuple[int, str, int], SyntheticBlobOwner]


def _pattern_to_pose_dict(
    rotation: np.ndarray,
    position: np.ndarray,
) -> dict[str, Any]:
    yaw = float(np.arctan2(rotation[1, 0], rotation[0, 0]))
    return {
        "position_m": [float(value) for value in np.asarray(position, dtype=np.float64).reshape(3)],
        "rotation": np.asarray(rotation, dtype=np.float64).reshape(3, 3).tolist(),
        "quaternion_wxyz": [
            float(np.cos(0.5 * yaw)),
            0.0,
            0.0,
            float(np.sin(0.5 * yaw)),
        ],
    }


def _load_custom_rigid_patterns(path: str | Path) -> dict[str, MarkerPattern]:
    rigid_path = Path(path)
    if not rigid_path.exists():
        return {}
    try:
        data = json.loads(rigid_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    patterns: dict[str, MarkerPattern] = {}
    for item in data.get("custom_rigids", []) if isinstance(data, dict) else []:
        if not isinstance(item, dict):
            continue
        name = str(item.get("name", "")).strip()
        points = item.get("marker_positions")
        if not name or not isinstance(points, list):
            continue
        try:
            patterns[name] = marker_pattern_from_points(
                name,
                np.asarray(points, dtype=np.float64),
                marker_diameter=float(item.get("marker_diameter_m", 0.014) or 0.014),
                metadata={"source": "tracking_rigids"},
            )
        except Exception:
            continue
    return patterns


def load_mvp_patterns(
    rigids_path: str | Path = "calibration/tracking_rigids.json",
    *,
    marker_layout: str = "current_4marker",
) -> list[MarkerPattern]:
    custom = _load_custom_rigid_patterns(rigids_path)
    if marker_layout == DESIGN_5MARKER_LAYOUT:
        patterns = _design_5marker_patterns()
    else:
        patterns = [
            HEAD_PATTERN,
            WAIST_PATTERN,
            CHEST_PATTERN,
            LEFT_FOOT_PATTERN,
            RIGHT_FOOT_PATTERN,
        ]
    if "wand" in custom:
        patterns.append(custom["wand"])
    else:
        wand_points = np.array(
            [
                [-0.090, -0.045, 0.0],
                [0.080, -0.045, 0.0],
                [0.055, 0.040, 0.0],
                [-0.035, 0.075, 0.055],
            ],
            dtype=np.float64,
        )
        patterns.append(
            MarkerPattern(
                name="wand",
                marker_positions=wand_points,
                marker_diameter=0.014,
                metadata={"source": "deterministic_fallback"},
            )
        )
    return patterns


def _copy_camera_params(camera: CameraParams, camera_id: str) -> CameraParams:
    return CameraParams(
        camera_id=camera_id,
        intrinsic_matrix=np.asarray(camera.intrinsic_matrix, dtype=np.float64).copy(),
        distortion_coeffs=np.asarray(camera.distortion_coeffs, dtype=np.float64).copy(),
        rotation=np.asarray(camera.rotation, dtype=np.float64).copy(),
        translation=np.asarray(camera.translation, dtype=np.float64).reshape(3).copy(),
        resolution=tuple(camera.resolution),
        focal_scale=float(getattr(camera, "focal_scale", 1.0)),
    )


def _load_real_camera_params(calibration_dir: str | Path) -> dict[str, CameraParams]:
    geo = GeometryPipeline()
    if geo.load_calibration(str(calibration_dir)) <= 0:
        return {}
    return dict(geo.camera_params)


def _generated_4cam_from_1_2_intrinsics(calibration_dir: str | Path) -> dict[str, CameraParams]:
    real = _load_real_camera_params(calibration_dir)
    if not all(camera_id in real for camera_id in DEFAULT_MVP_CAMERA_IDS):
        real = create_dummy_calibration(list(DEFAULT_MVP_CAMERA_IDS))
    generated: dict[str, CameraParams] = {}
    camera_centers = {
        "pi-cam-01": np.array([0.00, 0.00, 0.00], dtype=np.float64),
        "pi-cam-02": np.array([0.55, 0.00, 0.00], dtype=np.float64),
        "pi-cam-03": np.array([0.00, 0.45, 0.00], dtype=np.float64),
        "pi-cam-04": np.array([0.55, 0.45, 0.00], dtype=np.float64),
    }
    for index, camera_id in enumerate(DEFAULT_GENERATED_CAMERA_IDS):
        source_id = DEFAULT_MVP_CAMERA_IDS[index % 2]
        camera = _copy_camera_params(real[source_id], camera_id)
        camera.rotation = np.eye(3, dtype=np.float64)
        camera.translation = -camera_centers[camera_id]
        generated[camera_id] = camera
    return generated


def _look_at_world_to_camera(
    camera_center: np.ndarray,
    target: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    center = np.asarray(camera_center, dtype=np.float64).reshape(3)
    target = np.asarray(target, dtype=np.float64).reshape(3)
    forward = target - center
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm <= 1e-9:
        raise ValueError("camera center and target must differ")
    z_axis = forward / forward_norm
    world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    x_axis = np.cross(world_up, z_axis)
    if float(np.linalg.norm(x_axis)) <= 1e-9:
        world_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        x_axis = np.cross(world_up, z_axis)
    x_axis = x_axis / float(np.linalg.norm(x_axis))
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / float(np.linalg.norm(y_axis))
    rotation = np.vstack([x_axis, y_axis, z_axis])
    translation = -rotation @ center
    return rotation, translation


def _cube_top_2_4m_aim_center(calibration_dir: str | Path) -> dict[str, CameraParams]:
    real = _load_real_camera_params(calibration_dir)
    if not all(camera_id in real for camera_id in DEFAULT_MVP_CAMERA_IDS):
        real = create_dummy_calibration(list(DEFAULT_MVP_CAMERA_IDS))
    generated: dict[str, CameraParams] = {}
    half = 1.2
    height = 2.4
    target = np.array([0.0, 0.0, height * 0.5], dtype=np.float64)
    camera_centers = {
        "pi-cam-01": np.array([-half, -half, height], dtype=np.float64),
        "pi-cam-02": np.array([half, -half, height], dtype=np.float64),
        "pi-cam-03": np.array([-half, half, height], dtype=np.float64),
        "pi-cam-04": np.array([half, half, height], dtype=np.float64),
    }
    for index, camera_id in enumerate(DEFAULT_GENERATED_CAMERA_IDS):
        source_id = DEFAULT_MVP_CAMERA_IDS[index % 2]
        camera = _copy_camera_params(real[source_id], camera_id)
        camera.rotation, camera.translation = _look_at_world_to_camera(
            camera_centers[camera_id],
            target,
        )
        generated[camera_id] = camera
    return generated


def load_camera_rig(config: MultiRigidScenarioConfig) -> dict[str, CameraParams]:
    source = _default_camera_rig_source_for_scenario(
        config.scenario,
        config.camera_rig_source,
    )
    camera_ids = list(config.camera_ids)
    if source == "dummy":
        return create_dummy_calibration(camera_ids)
    if source == "generated_4cam_from_1_2_intrinsics":
        generated = _generated_4cam_from_1_2_intrinsics(config.calibration_dir)
        return {camera_id: generated[camera_id] for camera_id in camera_ids if camera_id in generated}
    if source == "cube_top_2_4m_aim_center":
        generated = _cube_top_2_4m_aim_center(config.calibration_dir)
        return {camera_id: generated[camera_id] for camera_id in camera_ids if camera_id in generated}
    real = _load_real_camera_params(config.calibration_dir)
    selected = {camera_id: real[camera_id] for camera_id in camera_ids if camera_id in real}
    if len(selected) >= 2:
        return selected
    return create_dummy_calibration(camera_ids)


def _install_camera_params_on_pipeline(
    pipeline: TrackingPipeline,
    camera_params: dict[str, CameraParams],
) -> None:
    pipeline.geometry.camera_params = camera_params
    pipeline.geometry.triangulator = Triangulator(
        camera_params,
        epipolar_threshold_px=pipeline.geometry.epipolar_threshold_px,
        fast_geometry=pipeline.pipeline_variant != "baseline",
        epipolar_pruning_enabled=pipeline.pipeline_variant == "fast_ABCDP",
        geo_hotpath_optimizations=_variant_has_geo_hotpath_optimizations(
            pipeline.pipeline_variant
        ),
        stage_callback=pipeline._record_stage,
    )
    pipeline._raw_scene_geometry.camera_params = camera_params
    pipeline._raw_scene_geometry.triangulator = Triangulator(
        camera_params,
        epipolar_threshold_px=pipeline.geometry.epipolar_threshold_px,
        fast_geometry=False,
        epipolar_pruning_enabled=False,
    )
    pipeline._calibration_loaded = True


class SimulatedRigidInstance:
    """Deterministic rigid body trajectory for multi-rigid scenarios."""

    def __init__(
        self,
        pattern: MarkerPattern,
        *,
        base_position: np.ndarray,
        yaw: float,
        fps: float,
        trajectory: str,
        phase: float = 0.0,
    ) -> None:
        self.pattern = pattern
        self.base_position = np.asarray(base_position, dtype=np.float64).reshape(3)
        self.yaw = float(yaw)
        self.fps = float(fps)
        self.trajectory = str(trajectory or "static")
        self.phase = float(phase)

    def pose_components(self, frame_index: int) -> tuple[np.ndarray, np.ndarray]:
        t_sec = float(frame_index) / self.fps
        pos = self.base_position.copy()
        yaw = self.yaw
        if self.trajectory in {"linear", "waist_wand_occlusion_reacquire"}:
            if self.pattern.name == "waist":
                pos += np.array([0.10 * t_sec, 0.015 * np.sin(t_sec + self.phase), 0.0])
                yaw += 0.04 * t_sec
            else:
                pos += np.array([0.02 * np.sin(0.7 * t_sec + self.phase), 0.0, 0.0])
        elif self.trajectory == "waist_rotate_partial_occlusion":
            if self.pattern.name == "waist":
                pos += np.array(
                    [
                        0.16 * t_sec,
                        0.035 * np.sin(1.4 * t_sec + self.phase),
                        0.012 * np.sin(0.9 * t_sec),
                    ],
                    dtype=np.float64,
                )
                yaw += 0.95 * t_sec
            else:
                pos += np.array(
                    [
                        0.025 * np.sin(0.8 * t_sec + self.phase),
                        0.010 * np.cos(0.5 * t_sec),
                        0.0,
                    ],
                    dtype=np.float64,
                )
        elif self.trajectory == "circle":
            pos += np.array([
                0.06 * np.cos(0.8 * t_sec + self.phase),
                0.06 * np.sin(0.8 * t_sec + self.phase),
                0.0,
            ])
            yaw += 0.4 * t_sec
        elif self.trajectory in FIVE_RIGID_BODY_SCENARIOS:
            # Human-scale dance stress: fast but plausible torso twist, head bob,
            # and leg crossing near the camera baseline.
            is_hard = self.trajectory in {
                "five_rigid_dance_hard_occlusion_v1",
                "five_rigid_dance_swap_red_v1",
            }
            is_red = self.trajectory == "five_rigid_dance_swap_red_v1"
            is_body_mount = self.trajectory in BODY_MOUNT_SCENARIOS
            omega = 2.0 * np.pi * (
                0.90 if is_red else 0.82 if is_hard else 0.75
            )
            travel = 1.14 if is_red else 1.06 if is_hard else 1.0
            twist = 1.18 if is_red else 1.10 if is_hard else 1.0
            phase = self.phase
            sit = max(0.0, np.sin(0.42 * omega * t_sec - 0.35)) if is_body_mount else 0.0
            if self.pattern.name == "head":
                pos += np.array(
                    [
                        travel * 0.045 * np.sin(omega * t_sec + phase),
                        travel * 0.030 * np.cos(0.7 * omega * t_sec + phase) - 0.035 * sit,
                        travel * 0.035 * np.sin(1.3 * omega * t_sec) - 0.020 * sit,
                    ],
                    dtype=np.float64,
                )
                yaw += twist * 0.70 * np.sin(omega * t_sec + phase)
            elif self.pattern.name == "chest":
                pos += np.array(
                    [
                        travel * 0.055 * np.sin(0.9 * omega * t_sec + phase),
                        travel * 0.045 * np.sin(1.1 * omega * t_sec) - 0.045 * sit,
                        travel * 0.020 * np.cos(0.8 * omega * t_sec + phase) - 0.035 * sit,
                    ],
                    dtype=np.float64,
                )
                yaw += twist * 1.05 * np.sin(0.85 * omega * t_sec)
            elif self.pattern.name == "waist":
                pos += np.array(
                    [
                        travel * 0.065 * np.sin(0.8 * omega * t_sec),
                        travel * 0.055 * np.sin(omega * t_sec + 0.4) - 0.060 * sit,
                        travel * 0.018 * np.sin(1.5 * omega * t_sec) - 0.030 * sit,
                    ],
                    dtype=np.float64,
                )
                yaw += twist * 1.25 * np.sin(0.9 * omega * t_sec)
            elif self.pattern.name == "left_foot":
                cross = np.sin(0.55 * omega * t_sec)
                pos += np.array(
                    [
                        travel * 0.20 * cross,
                        travel * 0.12 * np.sin(omega * t_sec + phase),
                        travel * 0.045 * max(0.0, np.sin(1.1 * omega * t_sec)),
                    ],
                    dtype=np.float64,
                )
                yaw += twist * 0.95 * np.sin(1.2 * omega * t_sec + phase)
            elif self.pattern.name == "right_foot":
                cross = -np.sin(0.55 * omega * t_sec)
                pos += np.array(
                    [
                        travel * 0.20 * cross,
                        -travel * 0.12 * np.sin(omega * t_sec + phase),
                        travel * 0.045 * max(0.0, np.cos(1.1 * omega * t_sec)),
                    ],
                    dtype=np.float64,
                )
                yaw += -twist * 0.95 * np.sin(1.2 * omega * t_sec + phase)
        return _rotz(yaw), pos

    def marker_positions_world(self, frame_index: int) -> np.ndarray:
        rotation, position = self.pose_components(frame_index)
        local = np.asarray(self.pattern.marker_positions, dtype=np.float64)
        return (rotation @ local.T).T + position


class MultiRigidFrameGenerator:
    """Generate deterministic PairedFrames plus GT sidecar metadata."""

    def __init__(
        self,
        config: MultiRigidScenarioConfig,
        *,
        patterns: list[MarkerPattern] | None = None,
        camera_params: dict[str, CameraParams] | None = None,
    ) -> None:
        if config.frames <= 0:
            raise ValueError("frames must be > 0")
        if config.fps <= 0.0:
            raise ValueError("fps must be > 0")
        if config.noise_px < 0.0:
            raise ValueError("noise_px must be >= 0")
        if config.false_blobs_per_camera < 0:
            raise ValueError("false_blobs_per_camera must be >= 0")
        if config.body_mount_blob_merge_factor < 0.0:
            raise ValueError("body_mount_blob_merge_factor must be >= 0")
        if not (0.0 <= config.marker_dropout_prob <= 1.0):
            raise ValueError("marker_dropout_prob must be in [0, 1]")
        if not (0.0 <= config.camera_dropout_prob <= 1.0):
            raise ValueError("camera_dropout_prob must be in [0, 1]")

        effective_marker_layout = _default_marker_layout_for_scenario(
            config.scenario,
            config.marker_layout,
        )
        if effective_marker_layout != config.marker_layout:
            config = replace(config, marker_layout=effective_marker_layout)
        effective_camera_rig_source = _default_camera_rig_source_for_scenario(
            config.scenario,
            config.camera_rig_source,
        )
        if effective_camera_rig_source != config.camera_rig_source:
            config = replace(config, camera_rig_source=effective_camera_rig_source)
        self.config = config
        self._rng = np.random.default_rng(int(config.seed))
        self._frame_index = 0
        self._dt_us = int(round(1_000_000.0 / float(config.fps)))
        self.camera_params = camera_params or load_camera_rig(config)
        self._cameras = {
            camera_id: VirtualCamera(camera)
            for camera_id, camera in self.camera_params.items()
            if camera_id in set(config.camera_ids)
        }
        pattern_list = patterns or load_mvp_patterns(
            config.rigids_path,
            marker_layout=config.marker_layout,
        )
        pattern_by_name = {pattern.name: pattern for pattern in pattern_list}
        self.patterns = [
            pattern_by_name[name]
            for name in config.rigid_names
            if name in pattern_by_name
        ]
        if not self.patterns:
            raise ValueError("no rigid patterns available for scenario")
        self._bodies = self._build_bodies(self.patterns)
        self._blob_reference_depth_by_camera = self._build_blob_reference_depths()

    def _build_bodies(self, patterns: list[MarkerPattern]) -> dict[str, SimulatedRigidInstance]:
        if self.config.scenario in FIVE_RIGID_BODY_SCENARIOS:
            if self.config.scenario == "five_rigid_dance_swap_red_v1":
                base_positions = {
                    "head": np.array([-0.18, 0.18, 2.46], dtype=np.float64),
                    "chest": np.array([0.08, 0.08, 2.34], dtype=np.float64),
                    "waist": np.array([0.05, 0.10, 2.20], dtype=np.float64),
                    "left_foot": np.array([0.00, -0.05, 2.05], dtype=np.float64),
                    "right_foot": np.array([0.10, 0.05, 2.05], dtype=np.float64),
                }
            elif self.config.scenario in BODY_MOUNT_SCENARIOS:
                base_positions = {
                    "head": np.array([-0.22, 0.18, 2.46], dtype=np.float64),
                    "chest": np.array([0.20, -0.16, 2.34], dtype=np.float64),
                    "waist": np.array([0.05, 0.10, 2.20], dtype=np.float64),
                    "left_foot": np.array([-0.28, -0.16, 2.05], dtype=np.float64),
                    "right_foot": np.array([0.38, 0.16, 2.05], dtype=np.float64),
                }
            else:
                base_positions = {
                    "head": np.array([-0.22, 0.18, 2.46], dtype=np.float64),
                    "chest": np.array([0.20, -0.16, 2.34], dtype=np.float64),
                    "waist": np.array([0.05, 0.10, 2.20], dtype=np.float64),
                    "left_foot": np.array([-0.28, -0.16, 2.05], dtype=np.float64),
                    "right_foot": np.array([0.38, 0.16, 2.05], dtype=np.float64),
                }
        else:
            base_positions = {
                "waist": np.array([0.08, 0.00, 2.20], dtype=np.float64),
                "wand": np.array([0.78, 0.18, 2.25], dtype=np.float64),
                "head": np.array([0.05, 0.00, 2.45], dtype=np.float64),
                "chest": np.array([0.05, 0.00, 2.30], dtype=np.float64),
                "left_foot": np.array([-0.10, -0.10, 2.05], dtype=np.float64),
                "right_foot": np.array([0.20, -0.10, 2.05], dtype=np.float64),
            }
        bodies = {}
        for index, pattern in enumerate(patterns):
            bodies[pattern.name] = SimulatedRigidInstance(
                pattern,
                base_position=base_positions.get(
                    pattern.name,
                    np.array([0.08 + index * 0.20, 0.0, 2.20], dtype=np.float64),
                ),
                yaw=0.08 * float(index),
                fps=self.config.fps,
                trajectory=(
                    self.config.scenario
                    if self.config.scenario
                    in {
                        "waist_wand_occlusion_reacquire",
                        "waist_rotate_partial_occlusion",
                        "five_rigid_dance_occlusion",
                        "five_rigid_dance_hard_occlusion_v1",
                        "five_rigid_dance_swap_red_v1",
                        "five_rigid_body_occlusion_v1",
                        "five_rigid_body_occlusion_relaxed_v1",
                    }
                    else self.config.trajectory_name
                ),
                phase=float(index) * 0.7,
            )
        return bodies

    def _hard_occlusion_cycle_time(self, frame_index: int) -> tuple[int, float]:
        t_sec = float(frame_index) / float(self.config.fps)
        if t_sec < 0.25:
            return 0, -1.0
        cycle_len_sec = 2.40
        cycle_pos = t_sec - 0.25
        return int(cycle_pos // cycle_len_sec), float(cycle_pos % cycle_len_sec)

    def _camera_rank(self, camera_id: str) -> int:
        camera_rank = {
            camera_id_value: index
            for index, camera_id_value in enumerate(self.config.camera_ids)
        }
        return int(camera_rank.get(camera_id, 0))

    def _real_log_like_blob_area(self, camera_id: str, *, alias: bool = False) -> float:
        rank = self._camera_rank(camera_id)
        if rank % 2 == 0:
            area = float(self._rng.normal(66.0, 16.0))
            return float(np.clip(area, 10.0, 115.0))
        area = float(self._rng.normal(11.0, 3.0))
        if alias:
            area += 2.0
        return float(np.clip(area, 4.5, 26.0))

    def _build_blob_reference_depths(self) -> dict[str, float]:
        if self.config.scenario not in {
            *BODY_MOUNT_SCENARIOS,
            "five_rigid_dance_swap_red_v1",
        }:
            return {}
        references: dict[str, float] = {}
        for camera_id, camera in self._cameras.items():
            depths: list[float] = []
            for body in self._bodies.values():
                for point in body.marker_positions_world(0):
                    depth = camera.camera_depth(point)
                    if depth > 0.0:
                        depths.append(depth)
            if depths:
                references[camera_id] = float(np.median(np.asarray(depths, dtype=np.float64)))
        return references

    def _depth_adjusted_blob_area(
        self,
        camera_id: str,
        base_area: float,
        marker_world: np.ndarray | None,
    ) -> float:
        if marker_world is None or camera_id not in self._cameras:
            return float(base_area)
        reference_depth = float(self._blob_reference_depth_by_camera.get(camera_id, 0.0))
        current_depth = float(self._cameras[camera_id].camera_depth(marker_world))
        if reference_depth <= 0.0 or current_depth <= 0.0:
            return float(base_area)

        base_diameter = float(np.sqrt(max(0.0, 4.0 * float(base_area) / np.pi)))
        depth_scale = float(np.clip((reference_depth / current_depth) ** 0.75, 0.72, 1.55))
        diameter = base_diameter * depth_scale
        if self._camera_rank(camera_id) % 2 == 0:
            diameter = float(np.clip(diameter, 3.5, 13.6))
        else:
            diameter = float(np.clip(diameter, 2.2, 6.4))
        return float(np.pi * (diameter * 0.5) ** 2)

    def _synthetic_blob_area(
        self,
        camera_id: str,
        pattern: MarkerPattern,
        marker_world: np.ndarray | None = None,
    ) -> float:
        if self.config.scenario in {
            *BODY_MOUNT_SCENARIOS,
            "five_rigid_dance_swap_red_v1",
        }:
            return self._depth_adjusted_blob_area(
                camera_id,
                self._real_log_like_blob_area(camera_id),
                marker_world,
            )
        return float(max(4.0, pattern.marker_diameter * 3600.0))

    def _merge_nearby_body_mount_blobs(
        self,
        *,
        timestamp_us: int,
        camera_id: str,
        blobs: list[dict[str, float]],
        ledger: dict[tuple[int, str, int], SyntheticBlobOwner],
    ) -> tuple[list[dict[str, float]], dict[tuple[int, str, int], SyntheticBlobOwner]]:
        merge_factor = float(self.config.body_mount_blob_merge_factor)
        if (
            self.config.scenario not in BODY_MOUNT_SCENARIOS
            or merge_factor <= 0.0
            or len(blobs) < 2
        ):
            return blobs, ledger

        parent = list(range(len(blobs)))

        def find(index: int) -> int:
            while parent[index] != index:
                parent[index] = parent[parent[index]]
                index = parent[index]
            return index

        def union(left: int, right: int) -> None:
            root_left = find(left)
            root_right = find(right)
            if root_left != root_right:
                parent[root_right] = root_left

        for left_index in range(len(blobs)):
            left_owner = ledger.get((timestamp_us, camera_id, left_index))
            if left_owner is None or left_owner.marker_index is None:
                continue
            left_blob = blobs[left_index]
            left_diameter = float(
                np.sqrt(max(0.0, 4.0 * float(left_blob.get("area", 0.0)) / np.pi))
            )
            for right_index in range(left_index + 1, len(blobs)):
                right_owner = ledger.get((timestamp_us, camera_id, right_index))
                if right_owner is None or right_owner.marker_index is None:
                    continue
                right_blob = blobs[right_index]
                right_diameter = float(
                    np.sqrt(max(0.0, 4.0 * float(right_blob.get("area", 0.0)) / np.pi))
                )
                dx = float(left_blob.get("x", 0.0)) - float(right_blob.get("x", 0.0))
                dy = float(left_blob.get("y", 0.0)) - float(right_blob.get("y", 0.0))
                distance_px = float(np.hypot(dx, dy))
                merge_distance_px = merge_factor * 0.5 * (left_diameter + right_diameter)
                if distance_px <= merge_distance_px:
                    union(left_index, right_index)

        groups: dict[int, list[int]] = {}
        for index in range(len(blobs)):
            groups.setdefault(find(index), []).append(index)

        if all(len(indices) == 1 for indices in groups.values()):
            return blobs, ledger

        merged_blobs: list[dict[str, float]] = []
        merged_ledger: dict[tuple[int, str, int], SyntheticBlobOwner] = {
            key: owner
            for key, owner in ledger.items()
            if not (key[0] == timestamp_us and key[1] == camera_id)
        }
        for indices in groups.values():
            new_index = len(merged_blobs)
            if len(indices) == 1:
                old_index = indices[0]
                blob = dict(blobs[old_index])
                owner = ledger.get((timestamp_us, camera_id, old_index))
                merged_blobs.append(blob)
                if owner is not None:
                    merged_ledger[(timestamp_us, camera_id, new_index)] = replace(
                        owner,
                        emitted_blob_index=new_index,
                    )
                continue

            area_values = np.asarray(
                [max(0.0, float(blobs[index].get("area", 0.0))) for index in indices],
                dtype=np.float64,
            )
            weight_sum = float(np.sum(area_values))
            if weight_sum <= 0.0:
                area_values = np.ones(len(indices), dtype=np.float64)
                weight_sum = float(len(indices))
            xs = np.asarray([float(blobs[index].get("x", 0.0)) for index in indices])
            ys = np.asarray([float(blobs[index].get("y", 0.0)) for index in indices])
            merged_blob = {
                "x": float(np.sum(xs * area_values) / weight_sum),
                "y": float(np.sum(ys * area_values) / weight_sum),
                "area": float(weight_sum),
            }
            merged_blobs.append(merged_blob)

            owners = [
                ledger[(timestamp_us, camera_id, index)]
                for index in indices
                if (timestamp_us, camera_id, index) in ledger
            ]
            primary = max(
                owners,
                key=lambda owner: float(blobs[owner.emitted_blob_index].get("area", 0.0)),
            )
            merged_owner_pairs = {
                (str(owner.rigid_name), int(owner.marker_index))
                for owner in owners
                if owner.marker_index is not None
            }
            for owner in owners:
                merged_owner_pairs.update(
                    (str(rigid_name), int(marker_index))
                    for rigid_name, marker_index in owner.merged_owners
                )
            merged_ledger[(timestamp_us, camera_id, new_index)] = SyntheticBlobOwner(
                timestamp=timestamp_us,
                camera_id=camera_id,
                emitted_blob_index=new_index,
                rigid_name=str(primary.rigid_name),
                marker_index=(
                    None
                    if primary.marker_index is None
                    else int(primary.marker_index)
                ),
                synthetic_blob_id=(
                    f"{timestamp_us}:{camera_id}:merged:"
                    + "+".join(str(index) for index in indices)
                ),
                merged_owners=tuple(sorted(merged_owner_pairs)),
            )

        return merged_blobs, merged_ledger

    def _hard_scenario_false_blobs_per_camera(
        self,
        frame_index: int,
        camera_id: str | None = None,
    ) -> int:
        if self.config.scenario not in {
            "five_rigid_dance_hard_occlusion_v1",
            "five_rigid_dance_swap_red_v1",
            *BODY_MOUNT_SCENARIOS,
        }:
            return int(self.config.false_blobs_per_camera)
        _, cycle_t = self._hard_occlusion_cycle_time(frame_index)
        if self.config.scenario in {
            "five_rigid_dance_swap_red_v1",
            *BODY_MOUNT_SCENARIOS,
        }:
            return 0
        else:
            extra = 6 if 0.70 <= cycle_t < 1.18 else 0
        return int(self.config.false_blobs_per_camera) + extra

    def _is_marker_occluded(
        self,
        rigid_name: str,
        marker_index: int,
        frame_index: int,
        camera_id: str,
    ) -> bool:
        if self.config.scenario in BODY_MOUNT_SCENARIOS:
            cycle_index, cycle_t = self._hard_occlusion_cycle_time(frame_index)
            if cycle_t < 0.0:
                return False
            rank = self._camera_rank(camera_id)
            even_cycle = cycle_index % 2 == 0
            relaxed = self.config.scenario == "five_rigid_body_occlusion_relaxed_v1"

            # Approximate body-mounted optics without a full body mesh: front
            # markers stay plausible, while body-side markers disappear from
            # camera groups as torso yaw, sitting, and leg crossing change.
            torso_start = 0.24 if relaxed else 0.22
            torso_end = 0.60 if relaxed else 0.64
            if rigid_name in {"chest", "waist"} and torso_start <= cycle_t < torso_end:
                offset = 0 if rigid_name == "waist" else 1
                return marker_index == ((rank + offset + cycle_index) % 5)

            waist_start = 0.92 if relaxed else 0.88
            waist_end = 1.10 if relaxed else 1.14
            if rigid_name == "waist" and waist_start <= cycle_t < waist_end:
                if rank == 0:
                    return marker_index in ({1, 4} if even_cycle else {0, 3})
                if rank == 1:
                    if relaxed:
                        return marker_index == (4 if even_cycle else 3)
                    return marker_index in ({0, 4} if even_cycle else {1, 3})
                if rank == 2 and not relaxed:
                    return marker_index == (4 if even_cycle else 0)

            foot_start = 1.10 if relaxed else 1.06
            foot_end = 1.36 if relaxed else 1.44
            if rigid_name in {"left_foot", "right_foot"} and foot_start <= cycle_t < foot_end:
                if rigid_name == "left_foot":
                    if relaxed:
                        return rank == 0 and marker_index == (1 if even_cycle else 4)
                    return rank in {0, 1} and marker_index == (1 if even_cycle else 4)
                if relaxed:
                    return rank == 3 and marker_index == (3 if even_cycle else 0)
                return rank in {2, 3} and marker_index == (3 if even_cycle else 0)

            head_start = 1.64 if relaxed else 1.60
            head_end = 1.80 if relaxed else 1.84
            if rigid_name == "head" and head_start <= cycle_t < head_end:
                if rank in {1, 3}:
                    return marker_index == (4 if even_cycle else 3)
                return marker_index == (3 if even_cycle else 2)
            return False

        if self.config.scenario == "five_rigid_dance_swap_red_v1":
            cycle_index, cycle_t = self._hard_occlusion_cycle_time(frame_index)
            if cycle_t < 0.0:
                return False
            rank = self._camera_rank(camera_id)
            even_cycle = cycle_index % 2 == 0

            if rigid_name in {"waist", "chest"} and 0.24 <= cycle_t < 0.76:
                if rigid_name == "waist" and rank in {0, 1}:
                    return marker_index in ({1, 4} if even_cycle else {0, 3})
                if rigid_name == "chest" and rank in {2, 3}:
                    return marker_index in ({0, 3} if even_cycle else {1, 4})
                return marker_index == ((rank + cycle_index) % 5)

            if rigid_name in {"left_foot", "right_foot"} and 0.66 <= cycle_t < 1.34:
                if rigid_name == "left_foot":
                    if rank in {0, 1} and marker_index in {0, 1, 4}:
                        return True
                    if rank in {2, 3} and marker_index in ({1, 4} if even_cycle else {0, 2}):
                        return True
                if rigid_name == "right_foot":
                    if rank in {2, 3} and marker_index in {2, 3, 4}:
                        return True
                    if rank in {0, 1} and marker_index in ({0, 4} if even_cycle else {1, 3}):
                        return True

            if rigid_name in {"head", "chest"} and 1.36 <= cycle_t < 1.62:
                return marker_index in ({0, 1, 2} if even_cycle else {1, 3, 4})
            return False

        if self.config.scenario == "five_rigid_dance_hard_occlusion_v1":
            cycle_index, cycle_t = self._hard_occlusion_cycle_time(frame_index)
            if cycle_t < 0.0:
                return False
            rank = self._camera_rank(camera_id)
            even_cycle = cycle_index % 2 == 0

            if rigid_name in {"waist", "chest"} and 0.00 <= cycle_t < 0.28:
                offset = 0 if rigid_name == "waist" else 2
                return marker_index == ((rank + offset + cycle_index) % 5)

            # Targeted reproducer: two cameras lose the same two markers while
            # the torso is rotating. This stresses reacquire without random loss.
            if 0.36 <= cycle_t < 0.62:
                if rigid_name == "waist" and rank in {0, 1}:
                    return marker_index in ({1, 4} if even_cycle else {0, 3})
                if rigid_name == "chest" and rank in {2, 3}:
                    return marker_index in ({0, 3} if even_cycle else {1, 4})

            if rigid_name in {"left_foot", "right_foot"} and 0.70 <= cycle_t < 1.18:
                if rigid_name == "left_foot":
                    if rank in {0, 1} and marker_index in {0, 1}:
                        return True
                    if rank in {2, 3} and marker_index == (4 if even_cycle else 3):
                        return True
                if rigid_name == "right_foot":
                    if rank in {2, 3} and marker_index in {2, 3}:
                        return True
                    if rank in {0, 1} and marker_index == (4 if even_cycle else 0):
                        return True

            # Briefly drop below reacquire minimum for one body; tracking should
            # hold or mark invalid rather than commit a weak subset pose.
            if rigid_name == ("head" if even_cycle else "chest") and 1.20 <= cycle_t < 1.40:
                return marker_index in {0, 1, 2}

            if rigid_name in {"left_foot", "right_foot"} and 1.48 <= cycle_t < 1.72:
                shared_pair = {1, 4} if even_cycle else {0, 2}
                if rank in {1, 2}:
                    return marker_index in shared_pair
            return False

        if self.config.scenario == "five_rigid_dance_occlusion":
            ratio = float(frame_index) / float(max(1, self.config.frames))
            rank = self._camera_rank(camera_id)

            # Torso self-occlusion while rotating: cameras see different
            # marker subsets, which stresses object-conditioned gating.
            if rigid_name in {"waist", "chest"}:
                if 0.18 <= ratio < 0.36:
                    return marker_index == ((rank + (0 if rigid_name == "waist" else 1)) % 4)
                if 0.58 <= ratio < 0.72:
                    return marker_index in {2, 3} and rank in {0, 2}

            # Leg-cross interval: both feet get camera-dependent marker loss,
            # including shared two-marker gaps on adjacent cameras.
            if rigid_name in {"left_foot", "right_foot"}:
                if 0.32 <= ratio < 0.56:
                    if rank in {0, 1} and marker_index in {0, 1}:
                        return rigid_name == "left_foot"
                    if rank in {2, 3} and marker_index in {2, 3}:
                        return rigid_name == "right_foot"
                if 0.56 <= ratio < 0.68:
                    return marker_index == ((rank + (1 if rigid_name == "left_foot" else 2)) % 4)

            # A brief head turn hides the raised asymmetry marker in half the
            # cameras, useful for testing pose flip resistance.
            if rigid_name == "head" and 0.42 <= ratio < 0.50:
                return marker_index == 3 and rank in {1, 3}
            return False

        if rigid_name != "waist":
            return False
        if self.config.scenario == "waist_wand_occlusion_reacquire":
            start = int(round(self.config.frames * 0.35))
            end = int(round(self.config.frames * 0.65))
            return start <= frame_index < end
        if self.config.scenario != "waist_rotate_partial_occlusion":
            return False

        ratio = float(frame_index) / float(max(1, self.config.frames))
        rank = self._camera_rank(camera_id)

        if 0.20 <= ratio < 0.34:
            return marker_index == (rank % 2)
        if 0.34 <= ratio < 0.52:
            if marker_index == 3:
                return True
            return marker_index == 0 and rank in {0, 1}
        if 0.52 <= ratio < 0.68:
            if marker_index == 1 and rank in {1, 2}:
                return True
            return marker_index == 2 and rank in {0, 3}
        if 0.68 <= ratio < 0.80:
            return marker_index == 0
        return False

    def _swap_red_alias_blobs_for_camera(
        self,
        *,
        frame_index: int,
        camera_id: str,
        projections: dict[str, dict[str, list[tuple[float, float] | None]]],
    ) -> list[tuple[float, float, str, int, str]]:
        if self.config.scenario != "five_rigid_dance_swap_red_v1":
            return []
        _, cycle_t = self._hard_occlusion_cycle_time(frame_index)
        if cycle_t < 0.0:
            return []

        rank = self._camera_rank(camera_id)
        aliases: list[tuple[float, float, str, int, str]] = []

        def add_alias(
            target_rigid: str,
            target_marker: int,
            source_rigid: str,
            source_marker: int,
            label: str,
            *,
            jitter_px: float = 0.22,
        ) -> None:
            target_projection = projections.get(camera_id, {}).get(target_rigid, [])
            if target_marker < 0 or target_marker >= len(target_projection):
                return
            projection = target_projection[target_marker]
            if projection is None:
                return
            source_projection = projections.get(camera_id, {}).get(source_rigid, [])
            if source_marker < 0 or source_marker >= len(source_projection):
                return
            if source_projection[source_marker] is None:
                return
            if self._is_marker_occluded(source_rigid, source_marker, frame_index, camera_id):
                return
            dx, dy = _sample_gaussian_pixel_noise(self._rng, jitter_px)
            aliases.append(
                (
                    float(projection[0] + dx),
                    float(projection[1] + dy),
                    source_rigid,
                    int(source_marker),
                    label,
                )
            )

        if 0.24 <= cycle_t < 0.76:
            if rank in {0, 1}:
                add_alias("waist", 1, "chest", 0, "torso_chest_as_waist")
                add_alias("waist", 4, "chest", 3, "torso_chest_as_waist")
            if rank in {2, 3}:
                add_alias("chest", 0, "waist", 1, "torso_waist_as_chest")
                add_alias("chest", 3, "waist", 4, "torso_waist_as_chest")

        if 0.66 <= cycle_t < 1.34:
            if rank in {0, 1}:
                add_alias("left_foot", 0, "right_foot", 2, "right_as_left_foot")
                add_alias("left_foot", 1, "right_foot", 3, "right_as_left_foot")
                add_alias("left_foot", 4, "right_foot", 4, "right_as_left_foot")
            if rank in {2, 3}:
                add_alias("right_foot", 2, "left_foot", 0, "left_as_right_foot")
                add_alias("right_foot", 3, "left_foot", 1, "left_as_right_foot")
                add_alias("right_foot", 4, "left_foot", 4, "left_as_right_foot")

        if 1.36 <= cycle_t < 1.62:
            if rank in {1, 2}:
                add_alias("head", 0, "chest", 0, "chest_as_head")
                add_alias("head", 1, "chest", 1, "chest_as_head")
                add_alias("head", 2, "chest", 2, "chest_as_head")
        return aliases

    def next_sample(self) -> SyntheticFrameSample | None:
        if self._frame_index >= self.config.frames:
            return None
        frame_index = int(self._frame_index)
        timestamp_us = int(frame_index * self._dt_us)
        if self.config.timestamp_jitter_us:
            timestamp_us += int(
                self._rng.integers(
                    -abs(self.config.timestamp_jitter_us),
                    abs(self.config.timestamp_jitter_us) + 1,
                )
            )

        present_camera_ids: list[str] = []
        for camera_id in self.config.camera_ids:
            if camera_id not in self._cameras:
                continue
            if (
                self.config.camera_dropout_prob > 0.0
                and float(self._rng.random()) < self.config.camera_dropout_prob
            ):
                continue
            present_camera_ids.append(camera_id)

        gt_poses: dict[str, dict[str, Any]] = {}
        marker_world_by_rigid: dict[str, np.ndarray] = {}
        for rigid_name, body in self._bodies.items():
            rotation, position = body.pose_components(frame_index)
            gt_poses[rigid_name] = _pattern_to_pose_dict(rotation, position)
            marker_world_by_rigid[rigid_name] = body.marker_positions_world(frame_index)

        projections: dict[str, dict[str, list[tuple[float, float] | None]]] = {}
        for camera_id in present_camera_ids:
            camera = self._cameras[camera_id]
            projections[camera_id] = {
                rigid_name: camera.project_points(points)
                for rigid_name, points in marker_world_by_rigid.items()
            }

        frames: dict[str, Frame] = {}
        ledger: dict[tuple[int, str, int], SyntheticBlobOwner] = {}
        for camera_id in present_camera_ids:
            blobs: list[dict[str, float]] = []
            for pattern in self.patterns:
                rigid_name = pattern.name
                for marker_index in range(pattern.num_markers):
                    if self._is_marker_occluded(rigid_name, marker_index, frame_index, camera_id):
                        continue
                    if (
                        self.config.marker_dropout_prob > 0.0
                        and float(self._rng.random()) < self.config.marker_dropout_prob
                    ):
                        continue
                    projection = projections[camera_id][rigid_name][marker_index]
                    if projection is None:
                        continue
                    dx, dy = _sample_gaussian_pixel_noise(self._rng, self.config.noise_px)
                    blob_index = len(blobs)
                    blobs.append(
                        {
                            "x": float(projection[0] + dx),
                            "y": float(projection[1] + dy),
                            "area": self._synthetic_blob_area(
                                camera_id,
                                pattern,
                                marker_world_by_rigid[rigid_name][marker_index],
                            ),
                        }
                    )
                    ledger[(timestamp_us, camera_id, blob_index)] = SyntheticBlobOwner(
                        timestamp=timestamp_us,
                        camera_id=camera_id,
                        emitted_blob_index=blob_index,
                        rigid_name=rigid_name,
                        marker_index=marker_index,
                        synthetic_blob_id=f"{timestamp_us}:{camera_id}:{rigid_name}:{marker_index}",
                    )

            camera = self.camera_params[camera_id]
            width, height = camera.resolution
            for alias_index, (x, y, source_rigid, source_marker, label) in enumerate(
                self._swap_red_alias_blobs_for_camera(
                    frame_index=frame_index,
                    camera_id=camera_id,
                    projections=projections,
                )
            ):
                if not (20.0 <= x <= float(width - 20) and 20.0 <= y <= float(height - 20)):
                    continue
                blob_index = len(blobs)
                blobs.append(
                    {
                        "x": float(x),
                        "y": float(y),
                        "area": float(self._real_log_like_blob_area(camera_id, alias=True)),
                    }
                )
                ledger[(timestamp_us, camera_id, blob_index)] = SyntheticBlobOwner(
                    timestamp=timestamp_us,
                    camera_id=camera_id,
                    emitted_blob_index=blob_index,
                    rigid_name=str(source_rigid),
                    marker_index=int(source_marker),
                    synthetic_blob_id=(
                        f"{timestamp_us}:{camera_id}:alias:{label}:{alias_index}:"
                        f"{source_rigid}:{source_marker}"
                    ),
                )
            for false_index in range(
                self._hard_scenario_false_blobs_per_camera(frame_index, camera_id)
            ):
                blob_index = len(blobs)
                blobs.append(
                    {
                        "x": float(self._rng.uniform(20.0, max(21.0, width - 20.0))),
                        "y": float(self._rng.uniform(20.0, max(21.0, height - 20.0))),
                        "area": (
                            float(self._rng.uniform(0.5, 8.0))
                            if self.config.scenario == "five_rigid_dance_swap_red_v1"
                            else 4.0
                        ),
                    }
                )
                ledger[(timestamp_us, camera_id, blob_index)] = SyntheticBlobOwner(
                    timestamp=timestamp_us,
                    camera_id=camera_id,
                    emitted_blob_index=blob_index,
                    rigid_name="__false__",
                    marker_index=None,
                    synthetic_blob_id=f"{timestamp_us}:{camera_id}:false:{false_index}",
                )

            blobs, ledger = self._merge_nearby_body_mount_blobs(
                timestamp_us=timestamp_us,
                camera_id=camera_id,
                blobs=blobs,
                ledger=ledger,
            )
            frames[camera_id] = Frame(
                camera_id=camera_id,
                timestamp=timestamp_us,
                frame_index=frame_index,
                blobs=blobs,
                received_at=0.0,
                host_received_at_us=timestamp_us,
                timestamp_source="synthetic",
                capture_to_process_ms=0.0,
                capture_to_send_ms=0.0,
            )

        self._frame_index += 1
        if len(frames) < 2:
            return SyntheticFrameSample(
                paired=PairedFrames(timestamp=timestamp_us, frames=frames, timestamp_range_us=0),
                gt_poses=gt_poses,
                ownership_ledger=ledger,
            )
        frame_timestamps = [frame.timestamp for frame in frames.values()]
        paired = PairedFrames(
            timestamp=timestamp_us,
            frames=frames,
            timestamp_range_us=int(max(frame_timestamps) - min(frame_timestamps)),
            pair_emitted_at_us=timestamp_us,
        )
        return SyntheticFrameSample(paired=paired, gt_poses=gt_poses, ownership_ledger=ledger)


def _ownership_confusion_from_snapshot(
    snapshot: dict[str, Any],
    ledger: dict[tuple[int, str, int], SyntheticBlobOwner],
) -> tuple[int, int, list[dict[str, Any]]]:
    events: list[dict[str, Any]] = []
    timestamp = int(snapshot.get("timestamp", 0) or 0)
    for point in snapshot.get("rigid_hint_triangulated_points", []) or []:
        rigid_name = point.get("rigid_name")
        marker_idx = point.get("marker_idx")
        if not rigid_name:
            continue
        for observation in point.get("observations", []) or []:
            camera_id = str(observation.get("camera_id", ""))
            blob_index = int(observation.get("blob_index", -1))
            owner = ledger.get((timestamp, camera_id, blob_index))
            if owner is None:
                continue
            merged_owners = set(owner.merged_owners or ())
            if marker_idx is not None and (str(rigid_name), int(marker_idx)) in merged_owners:
                continue
            if owner.rigid_name != rigid_name:
                events.append(
                    {
                        "type": "wrong_owner",
                        "timestamp": timestamp,
                        "camera_id": camera_id,
                        "blob_index": blob_index,
                        "expected_rigid": str(rigid_name),
                        "expected_marker": int(marker_idx) if marker_idx is not None else None,
                        "owner_rigid": str(owner.rigid_name),
                        "owner_marker": (
                            int(owner.marker_index)
                            if owner.marker_index is not None
                            else None
                        ),
                        "synthetic_blob_id": str(owner.synthetic_blob_id),
                    }
                )
            elif (
                owner.marker_index is not None
                and marker_idx is not None
                and int(owner.marker_index) != int(marker_idx)
            ):
                events.append(
                    {
                        "type": "marker_confusion",
                        "timestamp": timestamp,
                        "camera_id": camera_id,
                        "blob_index": blob_index,
                        "expected_rigid": str(rigid_name),
                        "expected_marker": int(marker_idx),
                        "owner_rigid": str(owner.rigid_name),
                        "owner_marker": int(owner.marker_index),
                        "synthetic_blob_id": str(owner.synthetic_blob_id),
                    }
                )
    wrong = sum(1 for event in events if event["type"] == "wrong_owner")
    marker_confusion = sum(1 for event in events if event["type"] == "marker_confusion")
    return wrong, marker_confusion, events


def _ownership_confusion_summary(events: list[dict[str, Any]]) -> dict[str, Any]:
    by_type = Counter(str(event.get("type", "")) for event in events)
    by_expected_rigid = Counter(str(event.get("expected_rigid", "")) for event in events)
    by_camera = Counter(str(event.get("camera_id", "")) for event in events)
    by_owner_rigid = Counter(str(event.get("owner_rigid", "")) for event in events)
    by_marker_pair = Counter(
        (
            f"{event.get('expected_rigid')}:{event.get('expected_marker')}"
            f"<-{event.get('owner_rigid')}:{event.get('owner_marker')}"
        )
        for event in events
    )
    frames = sorted({int(event.get("frame_index", -1)) for event in events})
    return {
        "count": int(len(events)),
        "by_type": dict(sorted(by_type.items())),
        "by_expected_rigid": dict(sorted(by_expected_rigid.items())),
        "by_owner_rigid": dict(sorted(by_owner_rigid.items())),
        "by_camera": dict(sorted(by_camera.items())),
        "by_marker_pair": dict(sorted(by_marker_pair.items())),
        "frames": frames[:50],
        "sample_events": events[:50],
        "sample_event_limit": 50,
    }


def _blob_merge_summary(owners: list[SyntheticBlobOwner]) -> dict[str, Any]:
    merged = [owner for owner in owners if len(owner.merged_owners or ()) > 1]
    by_camera = Counter(owner.camera_id for owner in merged)
    by_owner_count = Counter(str(len(owner.merged_owners or ())) for owner in merged)
    by_rigid_group = Counter(
        "+".join(
            sorted({str(rigid_name) for rigid_name, _ in owner.merged_owners})
        )
        for owner in merged
    )
    return {
        "count": int(len(merged)),
        "by_camera": dict(sorted(by_camera.items())),
        "by_owner_count": dict(sorted(by_owner_count.items())),
        "by_rigid_group": dict(sorted(by_rigid_group.items())),
        "sample_events": [owner.to_dict() for owner in merged[:50]],
        "sample_event_limit": 50,
    }


def run_multi_rigid_scenario(
    config: MultiRigidScenarioConfig,
    *,
    out_dir: str | None = None,
) -> dict[str, Any]:
    """Run a deterministic multi-rigid scenario through TrackingPipeline."""
    effective_marker_layout = _default_marker_layout_for_scenario(
        config.scenario,
        config.marker_layout,
    )
    if effective_marker_layout != config.marker_layout:
        config = replace(config, marker_layout=effective_marker_layout)
    effective_camera_rig_source = _default_camera_rig_source_for_scenario(
        config.scenario,
        config.camera_rig_source,
    )
    if effective_camera_rig_source != config.camera_rig_source:
        config = replace(config, camera_rig_source=effective_camera_rig_source)
    patterns = load_mvp_patterns(
        config.rigids_path,
        marker_layout=config.marker_layout,
    )
    camera_params = load_camera_rig(config)
    generator = MultiRigidFrameGenerator(config, patterns=patterns, camera_params=camera_params)

    pipeline = TrackingPipeline(
        enable_logging=False,
        patterns=[pattern for pattern in patterns if pattern.name in set(config.rigid_names)],
        pipeline_variant=config.pipeline_variant,
        subset_diagnostics_mode=config.subset_diagnostics_mode,
        rigid_stabilization=_rigid_stabilization_for_config(config),
    )
    _install_camera_params_on_pipeline(pipeline, camera_params)

    latest_poses: dict[str, Any] = {}
    errors: list[str] = []
    pipeline.set_pose_callback(lambda poses: latest_poses.update(poses))
    pipeline.set_error_callback(lambda exc: errors.append(repr(exc)))
    pipeline._running = True

    position_errors_by_rigid: dict[str, list[float]] = {name: [] for name in config.rigid_names}
    rotation_errors_by_rigid: dict[str, list[float]] = {name: [] for name in config.rigid_names}
    position_delta_errors_by_rigid: dict[str, list[float]] = {
        name: [] for name in config.rigid_names
    }
    rotation_delta_errors_by_rigid: dict[str, list[float]] = {
        name: [] for name in config.rigid_names
    }
    valid_frames_by_rigid: dict[str, int] = {name: 0 for name in config.rigid_names}
    last_valid_pose_by_rigid: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
    emitted_ledger_entries = 0
    wrong_ownership_count = 0
    marker_source_confusion_count = 0
    processed_samples = 0
    pipeline_pair_last_ms_values: list[float] = []
    rigid_last_ms_values: list[float] = []
    geometry_last_ms_values: list[float] = []
    ownership_confusion_events: list[dict[str, Any]] = []
    emitted_blob_owners: list[SyntheticBlobOwner] = []
    blob_area_values: list[float] = []
    blob_area_values_by_camera: dict[str, list[float]] = {
        camera_id: [] for camera_id in config.camera_ids
    }
    started_ns = time.perf_counter_ns()

    for _ in range(int(config.frames)):
        sample = generator.next_sample()
        if sample is None:
            break
        emitted_ledger_entries += len(sample.ownership_ledger)
        emitted_blob_owners.extend(sample.ownership_ledger.values())
        for frame in sample.paired.frames.values():
            camera_bucket = blob_area_values_by_camera.setdefault(str(frame.camera_id), [])
            for blob in frame.blobs or []:
                try:
                    area = float(blob.get("area", 0.0) or 0.0)
                except Exception:
                    continue
                if area <= 0.0:
                    continue
                blob_area_values.append(area)
                camera_bucket.append(area)
        if len(sample.paired.frames) < 2:
            continue
        latest_poses.clear()
        pipeline._on_paired_frames(sample.paired)
        processed_samples += 1
        pipeline_pair_last_ms_values.append(
            _latest_pipeline_stage_ms(pipeline, "pipeline_pair_ms")
        )
        rigid_last_ms_values.append(_latest_pipeline_stage_ms(pipeline, "rigid_ms"))
        geometry_last_ms_values.append(_latest_pipeline_stage_ms(pipeline, "triangulation_ms"))
        snapshot = pipeline.get_latest_triangulation_snapshot()
        wrong, confused, events = _ownership_confusion_from_snapshot(
            snapshot,
            sample.ownership_ledger,
        )
        frame_index = min(
            (int(frame.frame_index) for frame in sample.paired.frames.values()),
            default=int(processed_samples - 1),
        )
        for event in events:
            event["frame_index"] = int(frame_index)
        wrong_ownership_count += wrong
        marker_source_confusion_count += confused
        ownership_confusion_events.extend(events)
        for rigid_name, gt_pose in sample.gt_poses.items():
            pose = latest_poses.get(rigid_name)
            if pose is None or not getattr(pose, "valid", False):
                continue
            gt_position = np.asarray(gt_pose["position_m"], dtype=np.float64)
            gt_rotation = np.asarray(gt_pose["rotation"], dtype=np.float64)
            position_errors_by_rigid.setdefault(rigid_name, []).append(
                float(np.linalg.norm(np.asarray(pose.position, dtype=np.float64) - gt_position))
            )
            rotation_errors_by_rigid.setdefault(rigid_name, []).append(
                _rotation_error_deg(np.asarray(pose.rotation, dtype=np.float64), gt_rotation)
            )
            pose_position = np.asarray(pose.position, dtype=np.float64).reshape(3)
            pose_rotation = np.asarray(pose.rotation, dtype=np.float64).reshape(3, 3)
            previous = last_valid_pose_by_rigid.get(rigid_name)
            if previous is not None:
                prev_pose_position, prev_pose_rotation, prev_gt_position, prev_gt_rotation = previous
                estimated_delta = pose_position - prev_pose_position
                gt_delta = gt_position - prev_gt_position
                position_delta_errors_by_rigid.setdefault(rigid_name, []).append(
                    float(np.linalg.norm(estimated_delta - gt_delta))
                )
                estimated_rotation_delta = pose_rotation @ prev_pose_rotation.T
                gt_rotation_delta = gt_rotation @ prev_gt_rotation.T
                rotation_delta_errors_by_rigid.setdefault(rigid_name, []).append(
                    _rotation_error_deg(estimated_rotation_delta, gt_rotation_delta)
                )
            last_valid_pose_by_rigid[rigid_name] = (
                pose_position.copy(),
                pose_rotation.copy(),
                gt_position.copy(),
                gt_rotation.copy(),
            )
            valid_frames_by_rigid[rigid_name] = valid_frames_by_rigid.get(rigid_name, 0) + 1

    elapsed_ms = float(time.perf_counter_ns() - started_ns) / 1_000_000.0
    pipeline._running = False
    status = pipeline.get_status()
    variant_metrics = (
        pipeline.rigid_estimator.get_variant_metrics()
        if hasattr(pipeline.rigid_estimator, "get_variant_metrics")
        else {}
    )
    diagnostics = status.get("diagnostics", {})
    stage_ms = diagnostics.get("pipeline_stage_ms", {})
    pipeline_pair_ms = dict(stage_ms.get("pipeline_pair_ms", {}))
    rigid_ms = dict(stage_ms.get("rigid_ms", {}))
    geometry_ms = dict(stage_ms.get("triangulation_ms", {}))
    pipeline_pair_over_budget_threshold_ms = 8.475
    pipeline_pair_over_budget_count = sum(
        1
        for value in pipeline_pair_last_ms_values
        if float(value) > pipeline_pair_over_budget_threshold_ms
    )
    pipeline_pair_over_budget_max_run = _max_consecutive_over(
        pipeline_pair_last_ms_values,
        pipeline_pair_over_budget_threshold_ms,
    )
    warmup_trim_samples = min(30, max(0, processed_samples // 8))
    warmup_trimmed_pipeline_pair_ms = _warmup_trimmed_summary(
        pipeline_pair_last_ms_values,
        warmup_samples=warmup_trim_samples,
    )
    warmup_trimmed_rigid_ms = _warmup_trimmed_summary(
        rigid_last_ms_values,
        warmup_samples=warmup_trim_samples,
    )
    warmup_trimmed_geometry_ms = _warmup_trimmed_summary(
        geometry_last_ms_values,
        warmup_samples=warmup_trim_samples,
    )
    warmup_trimmed_pipeline_pair_over_budget_max_run = _max_consecutive_over(
        pipeline_pair_last_ms_values[warmup_trim_samples:],
        pipeline_pair_over_budget_threshold_ms,
    )
    production_go_no_go = {
        "pipeline_pair_p95_le_6ms": float(pipeline_pair_ms.get("p95", 0.0) or 0.0) <= 6.0,
        "pipeline_pair_max_le_8_475ms": (
            float(pipeline_pair_ms.get("max", 0.0) or 0.0)
            <= pipeline_pair_over_budget_threshold_ms
        ),
        "pipeline_pair_no_sustained_over_8_475ms": pipeline_pair_over_budget_max_run < 2,
        "rigid_p95_le_1_5ms": float(rigid_ms.get("p95", 0.0) or 0.0) <= 1.5,
    }
    production_go_no_go["passed"] = bool(
        production_go_no_go["pipeline_pair_p95_le_6ms"]
        and production_go_no_go["pipeline_pair_no_sustained_over_8_475ms"]
        and production_go_no_go["rigid_p95_le_1_5ms"]
    )
    valid_frame_ratio = {
        name: float(valid_frames_by_rigid.get(name, 0)) / float(max(1, processed_samples))
        for name in config.rigid_names
    }
    blob_diameter_values = [
        float(np.sqrt(4.0 * area / np.pi)) for area in blob_area_values if area > 0.0
    ]
    blob_area_summary = {
        "area_px2": _summary(blob_area_values),
        "equivalent_diameter_px": _summary(blob_diameter_values),
        "by_camera": {
            camera_id: {
                "area_px2": _summary(values),
                "equivalent_diameter_px": _summary(
                    [float(np.sqrt(4.0 * area / np.pi)) for area in values if area > 0.0]
                ),
            }
            for camera_id, values in sorted(blob_area_values_by_camera.items())
        },
    }
    max_position_delta_error_m = max(
        (
            float(_summary(values).get("max", 0.0))
            for values in position_delta_errors_by_rigid.values()
        ),
        default=0.0,
    )
    max_rotation_delta_error_deg = max(
        (
            float(_summary(values).get("max", 0.0))
            for values in rotation_delta_errors_by_rigid.values()
        ),
        default=0.0,
    )
    scenario_go_no_go = {
        "wrong_ownership_zero": int(wrong_ownership_count) == 0,
        "marker_source_confusion_zero": int(marker_source_confusion_count) == 0,
        "valid_frame_ratio_ge_0_95": all(
            float(value) >= 0.95 for value in valid_frame_ratio.values()
        ),
        "position_delta_error_max_le_2cm": max_position_delta_error_m <= 0.02,
        "rotation_delta_error_max_le_5deg": max_rotation_delta_error_deg <= 5.0,
        "production_go_no_go_passed": bool(production_go_no_go["passed"]),
    }
    scenario_go_no_go["passed"] = all(bool(value) for value in scenario_go_no_go.values())
    summary = {
        "scenario": config.scenario,
        "marker_layout": config.marker_layout,
        "camera_rig_source": config.camera_rig_source,
        "pipeline_variant": config.pipeline_variant,
        "subset_diagnostics_mode": config.subset_diagnostics_mode,
        "rigid_stabilization_profile": config.rigid_stabilization_profile,
        "rigid_stabilization_overrides": dict(config.rigid_stabilization_overrides or {}),
        "camera_ids": list(camera_params),
        "rigid_names": list(config.rigid_names),
        "body_mount_blob_merge_factor": float(config.body_mount_blob_merge_factor),
        "frames_requested": int(config.frames),
        "frames_processed": int(status.get("frames_processed", processed_samples) or 0),
        "poses_estimated": int(status.get("poses_estimated", 0) or 0),
        "elapsed_wall_ms": elapsed_ms,
        "pairs_per_second": (
            float(processed_samples / (elapsed_ms / 1000.0)) if elapsed_ms > 0.0 else 0.0
        ),
        "ownership_ledger_entries": int(emitted_ledger_entries),
        "wrong_ownership_count": int(wrong_ownership_count),
        "marker_source_confusion_count": int(marker_source_confusion_count),
        "ownership_confusion_summary": _ownership_confusion_summary(
            ownership_confusion_events
        ),
        "blob_merge_summary": _blob_merge_summary(emitted_blob_owners),
        "blob_area_summary": blob_area_summary,
        "position_error_m": {
            name: _summary(values) for name, values in position_errors_by_rigid.items()
        },
        "rotation_error_deg": {
            name: _summary(values) for name, values in rotation_errors_by_rigid.items()
        },
        "position_delta_error_m": {
            name: _summary(values)
            for name, values in position_delta_errors_by_rigid.items()
        },
        "rotation_delta_error_deg": {
            name: _summary(values)
            for name, values in rotation_delta_errors_by_rigid.items()
        },
        "valid_frame_ratio": valid_frame_ratio,
        "pipeline_pair_ms": pipeline_pair_ms,
        "rigid_ms": rigid_ms,
        "geometry_ms": geometry_ms,
        "variant_metrics": dict(variant_metrics),
        "performance_budget": {
            "pipeline_pair_over_8_475ms_count": int(pipeline_pair_over_budget_count),
            "pipeline_pair_over_8_475ms_max_consecutive": int(
                pipeline_pair_over_budget_max_run
            ),
            "pipeline_pair_over_budget_threshold_ms": float(
                pipeline_pair_over_budget_threshold_ms
            ),
            "pipeline_pair_last_ms": _summary(pipeline_pair_last_ms_values),
            "rigid_last_ms": _summary(rigid_last_ms_values),
            "geometry_last_ms": _summary(geometry_last_ms_values),
            "warmup_trim_samples": int(warmup_trim_samples),
            "warmup_trimmed": {
                "pipeline_pair_ms": warmup_trimmed_pipeline_pair_ms,
                "rigid_ms": warmup_trimmed_rigid_ms,
                "geometry_ms": warmup_trimmed_geometry_ms,
                "pipeline_pair_over_8_475ms_max_consecutive": int(
                    warmup_trimmed_pipeline_pair_over_budget_max_run
                ),
            },
        },
        "production_go_no_go": production_go_no_go,
        "scenario_go_no_go": scenario_go_no_go,
        "tracking": dict(status.get("tracking", {})),
        "error_count": len(errors),
        "errors": errors,
    }

    if out_dir is not None:
        out_path = Path(out_dir)
        out_path.mkdir(parents=True, exist_ok=True)
        (out_path / "summary.json").write_text(
            json.dumps(summary, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
    return summary


class SimulatedRigidBody:
    """Deterministic waist rigid body trajectory and marker ground truth."""

    def __init__(self, seed: int = 0, trajectory: str = "static", fps: float = 60.0):
        self.seed = int(seed)
        self.trajectory = str(trajectory)
        self.fps = float(fps)
        self._rng = np.random.default_rng(self.seed)

        if self.fps <= 0.0:
            raise ValueError("fps must be > 0")
        if self.trajectory not in {"static", "linear", "circle"}:
            raise ValueError(
                f"Unknown trajectory {self.trajectory!r}; expected one of: static, linear, circle"
            )

        # Small deterministic per-seed offsets so simulations can vary by seed.
        self._base_pos = np.array(
            [
                0.25 + float(self._rng.uniform(-0.03, 0.03)),
                float(self._rng.uniform(-0.03, 0.03)),
                2.0 + float(self._rng.uniform(-0.05, 0.05)),
            ],
            dtype=np.float64,
        )
        self._base_yaw = float(self._rng.uniform(-0.25, 0.25))

    def _pose_components(self, frame_index: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t_sec = float(frame_index) / self.fps

        if self.trajectory == "static":
            pos = self._base_pos.copy()
            yaw = self._base_yaw
        elif self.trajectory == "linear":
            vx = 0.10  # m/s
            pos = self._base_pos + np.array([vx * t_sec, 0.0, 0.0], dtype=np.float64)
            yaw = self._base_yaw + 0.05 * t_sec
        else:  # self.trajectory == "circle"
            radius = 0.08
            omega = 0.5  # rad/s
            pos = np.array(
                [
                    self._base_pos[0] + radius * np.cos(omega * t_sec),
                    self._base_pos[1] + radius * np.sin(omega * t_sec),
                    self._base_pos[2],
                ],
                dtype=np.float64,
            )
            yaw = self._base_yaw + omega * t_sec

        rot = _rotz(yaw)
        quat_wxyz = np.array([
            np.cos(0.5 * yaw),
            0.0,
            0.0,
            np.sin(0.5 * yaw),
        ], dtype=np.float64)
        return rot, pos, quat_wxyz

    def pose_world(self, frame_index: int) -> dict:
        _, pos, quat_wxyz = self._pose_components(frame_index)
        return {
            "position_m": pos.tolist(),
            "quaternion_wxyz": quat_wxyz.tolist(),
        }

    def marker_positions_world(self, frame_index: int) -> list[list[float]]:
        rot, pos, _ = self._pose_components(frame_index)
        body_pts = np.asarray(WAIST_PATTERN.marker_positions, dtype=np.float64)
        world_pts = (rot @ body_pts.T).T + pos
        return world_pts.tolist()


class SyntheticFrameGenerator:
    def __init__(
        self,
        camera_ids: list[str],
        seed: int = 0,
        fps: float = 60.0,
        noise_px: float = 0.0,
        marker_dropout: float = 0.0,
        camera_dropout: float = 0.0,
        trajectory: str = "static",
        rigid_names: tuple[str, ...] | list[str] | None = None,
        marker_layout: str = "current_4marker",
        camera_rig_source: str = "dummy",
        false_blobs_per_camera: int = 0,
        calibration_dir: str = "calibration",
        rigids_path: str = "calibration/tracking_rigids.json",
    ):
        if fps <= 0.0:
            raise ValueError("fps must be > 0")
        if noise_px < 0.0:
            raise ValueError("noise_px must be >= 0")
        if not (0.0 <= marker_dropout <= 1.0):
            raise ValueError("marker_dropout must be in [0, 1]")
        if not (0.0 <= camera_dropout <= 1.0):
            raise ValueError("camera_dropout must be in [0, 1]")

        selected_rigids = tuple(rigid_names or ("waist",))
        self._multi_generator: MultiRigidFrameGenerator | None = None
        self._last_ownership_ledger: dict[tuple[int, str, int], SyntheticBlobOwner] = {}
        if selected_rigids != ("waist",):
            config = MultiRigidScenarioConfig(
                seed=int(seed),
                camera_ids=tuple(str(camera_id) for camera_id in camera_ids),
                fps=float(fps),
                frames=1_000_000,
                rigid_names=selected_rigids,
                scenario="waist_wand_static_clean",
                trajectory_name=str(trajectory),
                noise_px=float(noise_px),
                false_blobs_per_camera=int(false_blobs_per_camera),
                marker_dropout_prob=float(marker_dropout),
                camera_dropout_prob=float(camera_dropout),
                marker_layout=str(marker_layout),
                camera_rig_source=str(camera_rig_source),
                calibration_dir=str(calibration_dir),
                rigids_path=str(rigids_path),
            )
            self._multi_generator = MultiRigidFrameGenerator(config)
            return

        self._rng = np.random.default_rng(int(seed))
        self._fps = float(fps)
        self._noise_px = float(noise_px)
        self._marker_dropout = float(marker_dropout)
        self._camera_dropout = float(camera_dropout)
        self._frame_index = 0
        self._dt_us = int(round(1_000_000.0 / self._fps))
        self._camera_ids_sorted = sorted(str(cam_id) for cam_id in camera_ids)

        calibration = create_dummy_calibration(self._camera_ids_sorted)
        self._cameras = {cid: VirtualCamera(calibration[cid]) for cid in self._camera_ids_sorted}
        self._body = SimulatedRigidBody(seed=int(seed), trajectory=trajectory, fps=self._fps)

    def ownership_ledger(self) -> dict[tuple[int, str, int], SyntheticBlobOwner]:
        return dict(self._last_ownership_ledger)

    def sidecar_ownership_ledger(self) -> dict[tuple[int, str, int], SyntheticBlobOwner]:
        return self.ownership_ledger()

    def next_paired(self) -> PairedFrames | None:
        if self._multi_generator is not None:
            sample = self._multi_generator.next_sample()
            if sample is None:
                self._last_ownership_ledger = {}
                return None
            self._last_ownership_ledger = dict(sample.ownership_ledger)
            return sample.paired

        frame_index = int(self._frame_index)
        timestamp_us = int(frame_index * self._dt_us)

        present_cam_ids: list[str] = []
        for cam_id in self._camera_ids_sorted:
            if self._camera_dropout > 0.0 and float(self._rng.random()) < self._camera_dropout:
                continue
            present_cam_ids.append(cam_id)

        if len(present_cam_ids) < 2:
            self._frame_index += 1
            return None

        marker_world = np.asarray(self._body.marker_positions_world(frame_index), dtype=np.float64)
        projections: dict[str, list[tuple[float, float] | None]] = {
            cam_id: self._cameras[cam_id].project_points(marker_world)
            for cam_id in present_cam_ids
        }

        keep_marker: list[bool] = []
        for marker_idx in range(marker_world.shape[0]):
            if self._marker_dropout > 0.0 and float(self._rng.random()) < self._marker_dropout:
                keep_marker.append(False)
                continue
            keep_marker.append(all(projections[cam_id][marker_idx] is not None for cam_id in present_cam_ids))

        frames_dict: dict[str, Frame] = {}
        for cam_id in present_cam_ids:
            blobs = []
            for marker_idx, keep in enumerate(keep_marker):
                if not keep:
                    continue
                proj = projections[cam_id][marker_idx]
                if proj is None:
                    continue
                dx, dy = _sample_gaussian_pixel_noise(self._rng, self._noise_px)
                blobs.append(
                    {
                        "x": float(proj[0] + dx),
                        "y": float(proj[1] + dy),
                        "area": 50.0,
                    }
                )
            frames_dict[cam_id] = Frame(
                camera_id=cam_id,
                timestamp=timestamp_us,
                frame_index=frame_index,
                blobs=blobs,
                received_at=0.0,
                host_received_at_us=timestamp_us,
            )

        paired = PairedFrames(timestamp=timestamp_us, frames=frames_dict, timestamp_range_us=0)
        self._frame_index += 1
        return paired


def run_closed_loop(
    *,
    frames: int,
    fps: float = 60.0,
    noise_px: float = 0.0,
    marker_dropout: float = 0.0,
    camera_dropout: float = 0.0,
    trajectory: str = "static",
    seed: int = 0,
    out_dir: str | None = None,
) -> dict:
    """Run an in-process closed-loop simulation and write artifacts.

    Returns a summary dict with keys exactly:
      mean_position_error_m, mean_rotation_error_deg, drop_rate, frame_log, pose_log, eval_json
    """

    if frames <= 0:
        raise ValueError("frames must be > 0")
    if fps <= 0:
        raise ValueError("fps must be > 0")

    rng = np.random.default_rng(int(seed))
    camera_ids = ["cam0", "cam1"]
    calibration = create_dummy_calibration(camera_ids)
    cameras = {cid: VirtualCamera(calibration[cid]) for cid in camera_ids}

    geo = GeometryPipeline()
    geo.camera_params = calibration
    geo.triangulator = Triangulator(calibration)

    # Use a larger marker_diameter for simulation to ensure waist markers cluster
    # into a single cluster in the DBSCAN-based estimator. This avoids the
    # pathological drop_rate=1.0 and NaN metrics observed in the static/no-noise
    # scenario due to overly small eps in clustering.
    rigid = RigidBodyEstimator(patterns=[WAIST_PATTERN], marker_diameter=0.2)

    if out_dir is None:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = Path("./output/sim") / f"{ts}-{seed}"
    else:
        out_path = Path(out_dir)
    out_path.mkdir(parents=True, exist_ok=True)

    frame_logger = FrameLogger(log_dir=str(out_path))
    frame_log = frame_logger.start_recording(session_name="frames")

    visualizer = TrackingVisualizer(output_dir=str(out_path), enable_console=False, enable_file=True)
    visualizer.start_session(session_name="sim")
    pose_log = str(out_path / "sim_poses.jsonl")

    body = SimulatedRigidBody(seed=seed, trajectory=trajectory, fps=fps)

    dt_us = int(round(1_000_000.0 / float(fps)))
    start_ts_us = 0

    pos_errors: list[float] = []
    rot_errors: list[float] = []
    drop_count = 0

    for i in range(int(frames)):
        timestamp_us = int(start_ts_us + i * dt_us)
        gt_r, gt_t, _ = body._pose_components(i)
        gt_markers_world = np.asarray(body.marker_positions_world(i), dtype=np.float64)

        present_cam_ids: list[str] = []
        projections: dict[str, list[tuple[float, float] | None]] = {}
        for cam_id in camera_ids:
            if camera_dropout > 0.0 and float(rng.random()) < float(camera_dropout):
                continue
            present_cam_ids.append(cam_id)
            projections[cam_id] = cameras[cam_id].project_points(gt_markers_world)

        # Shared per-frame marker drop mask to preserve blob correspondence assumptions.
        keep_marker = []
        for m in range(gt_markers_world.shape[0]):
            if marker_dropout > 0.0 and float(rng.random()) < float(marker_dropout):
                keep_marker.append(False)
                continue
            ok_all = True
            for cam_id in present_cam_ids:
                if projections[cam_id][m] is None:
                    ok_all = False
                    break
            keep_marker.append(ok_all)

        frames_dict: dict[str, Frame] = {}
        for cam_id in sorted(present_cam_ids):
            blobs = []
            for m, keep in enumerate(keep_marker):
                if not keep:
                    continue
                proj = projections[cam_id][m]
                if proj is None:
                    continue
                dx, dy = _sample_gaussian_pixel_noise(rng, float(noise_px))
                blobs.append({
                    "x": float(proj[0] + dx),
                    "y": float(proj[1] + dy),
                    "area": 50.0,
                })

            frame = Frame(
                camera_id=cam_id,
                timestamp=timestamp_us,
                frame_index=i,
                blobs=blobs,
                received_at=0.0,
                host_received_at_us=timestamp_us,
            )
            frames_dict[cam_id] = frame
            frame_logger.log_frame(frame.to_dict())

        paired = PairedFrames(timestamp=timestamp_us, frames=frames_dict, timestamp_range_us=0)
        geo_out = geo.process_paired_frames(paired)

        pts_list = geo_out.get("points_3d", [])
        if pts_list:
            pts = np.asarray(pts_list, dtype=np.float64).reshape(-1, 3)
        else:
            pts = np.zeros((0, 3), dtype=np.float64)

        poses = rigid.process_points(pts, timestamp_us)
        if pts.size == 0:
            for p in poses.values():
                p.timestamp = timestamp_us

        poses_dict = {name: pose.to_dict() for name, pose in poses.items()}
        visualizer.visualize_poses(poses_dict, timestamp_us, metrics=None)

        waist = poses.get("waist")
        if waist is None or not waist.valid:
            drop_count += 1
            continue

        pos_err = float(np.linalg.norm(np.asarray(waist.position) - gt_t))
        rot_err = _rotation_error_deg(np.asarray(waist.rotation), gt_r)
        pos_errors.append(pos_err)
        rot_errors.append(rot_err)

    frame_logger.stop_recording()
    visualizer.end_session()

    drop_rate = float(drop_count) / float(frames)
    mean_pos = float(np.mean(pos_errors)) if pos_errors else float("nan")
    mean_rot = float(np.mean(rot_errors)) if rot_errors else float("nan")

    eval_path = out_path / "eval.json"
    eval_summary = {
        "mean_position_error_m": mean_pos,
        "mean_rotation_error_deg": mean_rot,
        "max_position_error_m": float(np.max(pos_errors)) if pos_errors else float("nan"),
        "max_rotation_error_deg": float(np.max(rot_errors)) if rot_errors else float("nan"),
        "rms_position_error_m": float(np.sqrt(np.mean(np.square(pos_errors)))) if pos_errors else float("nan"),
        "rms_rotation_error_deg": float(np.sqrt(np.mean(np.square(rot_errors)))) if rot_errors else float("nan"),
        "drop_rate": drop_rate,
        "valid_frames": int(len(pos_errors)),
        "total_frames": int(frames),
        "params": {
            "frames": int(frames),
            "fps": float(fps),
            "noise_px": float(noise_px),
            "marker_dropout": float(marker_dropout),
            "camera_dropout": float(camera_dropout),
            "trajectory": str(trajectory),
            "seed": int(seed),
        },
        "artifacts": {
            "frame_log": str(frame_log),
            "pose_log": str(pose_log),
        },
    }
    eval_path.write_text(json.dumps(eval_summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return {
        "mean_position_error_m": mean_pos,
        "mean_rotation_error_deg": mean_rot,
        "drop_rate": drop_rate,
        "frame_log": str(frame_log),
        "pose_log": str(pose_log),
        "eval_json": str(eval_path),
    }


def assert_metrics(
    summary: dict,
    *,
    max_mean_position_error_m: float,
    max_mean_rotation_error_deg: float | None = None,
) -> None:
    mean_pos = float(summary.get("mean_position_error_m"))
    if not np.isfinite(mean_pos) or mean_pos > float(max_mean_position_error_m):
        raise AssertionError(
            f"mean_position_error_m={mean_pos} exceeds max_mean_position_error_m={float(max_mean_position_error_m)}"
        )

    if max_mean_rotation_error_deg is None:
        return

    mean_rot = float(summary.get("mean_rotation_error_deg"))
    if not np.isfinite(mean_rot) or mean_rot > float(max_mean_rotation_error_deg):
        raise AssertionError(
            f"mean_rotation_error_deg={mean_rot} exceeds max_mean_rotation_error_deg={float(max_mean_rotation_error_deg)}"
        )


def _parse_realtime_flag(val: str) -> bool:
    v = str(val).lower()
    return v in {"true", "1", "yes", "y"}


def _parse_bool_override(val: str | None) -> bool | None:
    if val is None:
        return None
    v = str(val).strip().lower()
    if v in {"true", "1", "yes", "y", "on"}:
        return True
    if v in {"false", "0", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true/false")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="python -m tools.sim")
    parser.add_argument("--frames", type=int, required=True, help="Number of frames (>0)")
    parser.add_argument("--fps", type=float, default=60.0, help="Frames per second (>0)")
    parser.add_argument("--scenario", type=str, default=None, help="Multi-rigid scenario name")
    parser.add_argument("--calibration", type=str, default="calibration", help="Calibration directory")
    parser.add_argument("--rigids", type=str, default="calibration/tracking_rigids.json", help="Tracking rigid pattern file")
    parser.add_argument("--marker-layout", type=str, default="current_4marker", help="Marker layout label")
    parser.add_argument(
        "--camera-rig-source",
        type=str,
        default="auto",
        choices=[
            "auto",
            "real_2cam",
            "generated_4cam_from_1_2_intrinsics",
            "cube_top_2_4m_aim_center",
            "dummy",
        ],
        help="Camera rig source for multi-rigid scenarios",
    )
    parser.add_argument("--false-blobs-per-camera", type=int, default=0)
    parser.add_argument("--pipeline-variant", type=str, default="fast_ABCDHRF")
    parser.add_argument("--subset-diagnostics-mode", type=str, default="off")
    parser.add_argument(
        "--rigid-stabilization-profile",
        type=str,
        default="pipeline_default",
        choices=["pipeline_default", "gui_live"],
        help="Rigid stabilization preset to apply before running the scenario",
    )
    parser.add_argument(
        "--object-gating-pixel-max-px",
        type=float,
        default=None,
        help="Override rigid object-gating pixel radius for sim PDCA",
    )
    parser.add_argument(
        "--object-conditioned-gating",
        type=_parse_bool_override,
        default=None,
        help="Override object-conditioned gating true/false for sim PDCA",
    )
    parser.add_argument(
        "--object-gating-enforced",
        type=_parse_bool_override,
        default=None,
        help="Override object-gating enforcement true/false for sim PDCA",
    )
    parser.add_argument(
        "--object-gating-ambiguous-blob-min-separation-px",
        type=float,
        default=None,
        help="Override object-gating ambiguous nearby-blob rejection distance for sim PDCA",
    )
    parser.add_argument(
        "--object-gating-ambiguous-blob-diameter-overlap-ratio",
        type=float,
        default=None,
        help="Override object-gating close-blob rejection as a fraction of average blob diameter",
    )
    parser.add_argument(
        "--object-gating-ambiguous-marker-assignment-min-margin-px",
        type=float,
        default=None,
        help="Override object-gating marker-vs-marker assignment margin for sim PDCA",
    )
    parser.add_argument(
        "--pose-continuity-guard",
        type=_parse_bool_override,
        default=None,
        help="Override pose continuity guard enabled/enforced true/false for sim PDCA",
    )
    parser.add_argument(
        "--position-continuity-guard",
        type=_parse_bool_override,
        default=None,
        help="Override position continuity guard enabled/enforced true/false for sim PDCA",
    )
    parser.add_argument("--noise-px", type=float, default=0.0, help="Pixel noise stddev")
    parser.add_argument(
        "--body-mount-blob-merge-factor",
        type=float,
        default=0.0,
        help="Merge close true blobs in body-mounted sim when distance <= factor * average equivalent diameter",
    )
    parser.add_argument("--marker-dropout", type=float, default=0.0, help="Marker dropout probability [0,1]")
    parser.add_argument("--camera-dropout", type=float, default=0.0, help="Camera dropout probability [0,1]")
    parser.add_argument("--mode", type=str, default="inprocess", choices=["inprocess", "udp"], help="Execution mode: inprocess or udp")
    parser.add_argument("--realtime", type=str, default="true", help=" Emit in real-time; 'true'|'false'")
    parser.add_argument(
        "--trajectory",
        type=str,
        default="static",
        help="Trajectory: static|linear|circle",
    )
    parser.add_argument("--seed", type=int, default=0, help="RNG seed")
    parser.add_argument("--out-dir", type=str, default=None, help="Output directory")
    parser.add_argument("--out", dest="out_dir", type=str, help="Output directory")

    try:
        args = parser.parse_args(argv)
    except SystemExit as e:
        try:
            return int(e.code)
        except Exception:
            return 2

    def _err(msg: str) -> int:
        print(f"error: {msg}", file=sys.stderr)
        return 2

    if args.frames <= 0:
        return _err("--frames must be > 0")
    if args.fps <= 0:
        return _err("--fps must be > 0")
    if not (0.0 <= args.marker_dropout <= 1.0):
        return _err("--marker-dropout must be in [0, 1]")
    if not (0.0 <= args.camera_dropout <= 1.0):
        return _err("--camera-dropout must be in [0, 1]")
    if args.noise_px < 0.0:
        return _err("--noise-px must be >= 0")
    if args.body_mount_blob_merge_factor < 0.0:
        return _err("--body-mount-blob-merge-factor must be >= 0")
    if args.false_blobs_per_camera < 0:
        return _err("--false-blobs-per-camera must be >= 0")
    if (
        args.object_gating_pixel_max_px is not None
        and args.object_gating_pixel_max_px <= 0.0
    ):
        return _err("--object-gating-pixel-max-px must be > 0")
    if (
        args.object_gating_ambiguous_blob_min_separation_px is not None
        and args.object_gating_ambiguous_blob_min_separation_px < 0.0
    ):
        return _err("--object-gating-ambiguous-blob-min-separation-px must be >= 0")
    if (
        args.object_gating_ambiguous_blob_diameter_overlap_ratio is not None
        and args.object_gating_ambiguous_blob_diameter_overlap_ratio < 0.0
    ):
        return _err("--object-gating-ambiguous-blob-diameter-overlap-ratio must be >= 0")
    if (
        args.object_gating_ambiguous_marker_assignment_min_margin_px is not None
        and args.object_gating_ambiguous_marker_assignment_min_margin_px < 0.0
    ):
        return _err("--object-gating-ambiguous-marker-assignment-min-margin-px must be >= 0")
    if args.trajectory not in {"static", "linear", "circle"}:
        return _err("--trajectory must be one of: static, linear, circle")

    if args.scenario:
        marker_layout = _default_marker_layout_for_scenario(
            args.scenario,
            str(args.marker_layout),
        )
        camera_rig_source = _default_camera_rig_source_for_scenario(
            args.scenario,
            str(args.camera_rig_source),
        )
        if (
            camera_rig_source
            in {"generated_4cam_from_1_2_intrinsics", "cube_top_2_4m_aim_center"}
            or args.scenario in FIVE_RIGID_BODY_SCENARIOS
        ):
            camera_ids = DEFAULT_GENERATED_CAMERA_IDS
        else:
            camera_ids = DEFAULT_MVP_CAMERA_IDS
        rigid_names = _default_rigids_for_scenario(args.scenario)
        rigid_stabilization_overrides: dict[str, Any] = {}
        if args.object_gating_pixel_max_px is not None:
            rigid_stabilization_overrides["object_gating_pixel_max_px"] = float(
                args.object_gating_pixel_max_px
            )
        if args.object_conditioned_gating is not None:
            rigid_stabilization_overrides["object_conditioned_gating"] = bool(
                args.object_conditioned_gating
            )
        if args.object_gating_enforced is not None:
            rigid_stabilization_overrides["object_gating_enforced"] = bool(
                args.object_gating_enforced
            )
        if args.object_gating_ambiguous_blob_min_separation_px is not None:
            rigid_stabilization_overrides[
                "object_gating_ambiguous_blob_min_separation_px"
            ] = float(args.object_gating_ambiguous_blob_min_separation_px)
        if args.object_gating_ambiguous_blob_diameter_overlap_ratio is not None:
            rigid_stabilization_overrides[
                "object_gating_ambiguous_blob_diameter_overlap_ratio"
            ] = float(args.object_gating_ambiguous_blob_diameter_overlap_ratio)
        if args.object_gating_ambiguous_marker_assignment_min_margin_px is not None:
            rigid_stabilization_overrides[
                "object_gating_ambiguous_marker_assignment_min_margin_px"
            ] = float(args.object_gating_ambiguous_marker_assignment_min_margin_px)
        if args.pose_continuity_guard is not None:
            rigid_stabilization_overrides["pose_continuity_guard_enabled"] = bool(
                args.pose_continuity_guard
            )
            rigid_stabilization_overrides["pose_continuity_guard_enforced"] = bool(
                args.pose_continuity_guard
            )
        if args.position_continuity_guard is not None:
            rigid_stabilization_overrides["position_continuity_guard_enabled"] = bool(
                args.position_continuity_guard
            )
            rigid_stabilization_overrides["position_continuity_guard_enforced"] = bool(
                args.position_continuity_guard
            )
        try:
            summary = run_multi_rigid_scenario(
                MultiRigidScenarioConfig(
                    seed=int(args.seed),
                    camera_ids=tuple(camera_ids),
                    frames=int(args.frames),
                    fps=float(args.fps),
                    rigid_names=rigid_names,
                    scenario=str(args.scenario),
                    trajectory_name=str(args.trajectory),
                    noise_px=float(args.noise_px),
                    body_mount_blob_merge_factor=float(
                        args.body_mount_blob_merge_factor
                    ),
                    false_blobs_per_camera=int(args.false_blobs_per_camera),
                    marker_dropout_prob=float(args.marker_dropout),
                    camera_dropout_prob=float(args.camera_dropout),
                    marker_layout=marker_layout,
                    camera_rig_source=camera_rig_source,
                    calibration_dir=str(args.calibration),
                    rigids_path=str(args.rigids),
                    pipeline_variant=str(args.pipeline_variant),
                    subset_diagnostics_mode=str(args.subset_diagnostics_mode),
                    rigid_stabilization_profile=str(args.rigid_stabilization_profile),
                    rigid_stabilization_overrides=rigid_stabilization_overrides,
                ),
                out_dir=args.out_dir,
            )
        except Exception as e:
            print(f"error: {e}", file=sys.stderr)
            return 1
        if args.out_dir:
            eval_path = Path(args.out_dir) / "eval.json"
            eval_path.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0

    # Branch on mode: inprocess vs udp
    if getattr(args, "mode", "inprocess") == "udp":
        # UDP loopback mode: emit synthetic frames over UDP for the receiver to process.
        def _udp_emit_loop(frames: int, fps: float, noise_px: float, marker_dropout: float,
                           camera_dropout: float, trajectory: str, seed: int,
                           out_dir: str | None, realtime_flag: bool) -> None:
            camera_ids = ["cam0", "cam1"]
            # Setup dummy calibration and projection utilities
            calibration = create_dummy_calibration(camera_ids)
            cameras = {cid: VirtualCamera(calibration[cid]) for cid in camera_ids}

            geo = GeometryPipeline()
            geo.camera_params = calibration
            geo.triangulator = Triangulator(calibration)

            # Use a larger marker_diameter for better clustering consistency
            rigid = RigidBodyEstimator(patterns=[WAIST_PATTERN], marker_diameter=0.2)

            # UDP socket setup
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            addr = ("127.0.0.1", 5000)

            # World model for synthetic projections
            body = SimulatedRigidBody(seed=seed, trajectory=trajectory, fps=fps)

            dt_us = int(round(1_000_000.0 / float(fps)))
            start_ts_us = 0

            rng = np.random.default_rng(int(seed))

            for i in range(int(frames)):
                timestamp_us = int(start_ts_us + i * dt_us)
                gt_r, gt_t, _ = body._pose_components(i)
                gt_markers_world = np.asarray(body.marker_positions_world(i), dtype=np.float64)

                present_cam_ids = []
                projections: dict[str, list[tuple[float, float] | None]] = {}
                for cam_id in camera_ids:
                    if camera_dropout > 0.0 and float(rng.random()) < float(camera_dropout):
                        continue
                    present_cam_ids.append(cam_id)
                    projections[cam_id] = cameras[cam_id].project_points(gt_markers_world)

                keep_marker: list[bool] = []
                for m in range(gt_markers_world.shape[0]):
                    if marker_dropout > 0.0 and float(rng.random()) < float(marker_dropout):
                        keep_marker.append(False)
                        continue
                    ok_all = True
                    for cam_id in present_cam_ids:
                        if projections[cam_id][m] is None:
                            ok_all = False
                            break
                    keep_marker.append(ok_all)

                for cam_id in sorted(present_cam_ids):
                    blobs = []
                    for m, keep in enumerate(keep_marker):
                        if not keep:
                            continue
                        proj = projections[cam_id][m]
                        if proj is None:
                            continue
                        dx, dy = _sample_gaussian_pixel_noise(rng, float(noise_px))
                        blobs.append({
                            "x": float(proj[0] + dx),
                            "y": float(proj[1] + dy),
                            "area": 50.0,
                        })

                    msg = {
                        "camera_id": cam_id,
                        "timestamp": int(timestamp_us),
                        "frame_index": int(i),
                        "blobs": blobs,
                    }
                    sock.sendto(json.dumps(msg).encode("utf-8"), addr)

                if realtime_flag:
                    time.sleep(1.0 / float(fps))

            sock.close()

        # Run UDP emitter in background once the pipeline is ready to consume frames
        import numpy as _np  # noqa: F401 (use later in closure to satisfy lints)
        realtime_flag = _parse_realtime_flag(args.realtime)
        udp_thread = threading.Thread(
            target=_udp_emit_loop,
            args=(int(args.frames), float(args.fps), float(args.noise_px), float(args.marker_dropout),
                  float(args.camera_dropout), str(args.trajectory), int(args.seed), args.out_dir, realtime_flag),
            daemon=True,
        )

        # Start the tracking pipeline before emitting UDP frames
        pipeline = TrackingPipeline(udp_port=5000, enable_logging=True, log_dir=args.out_dir)
        pipeline.use_dummy_calibration(camera_ids=["cam0", "cam1"])
        pipeline.start(session_name="frame_log")

        # Now start UDP emitter thread
        udp_thread.start()

        # Wait for UDP emission to finish, then stop the pipeline
        udp_thread.join()
        time.sleep(0.2)
        pipeline.stop()

        return 0
    try:
        summary = run_closed_loop(
            frames=int(args.frames),
            fps=float(args.fps),
            noise_px=float(args.noise_px),
            marker_dropout=float(args.marker_dropout),
            camera_dropout=float(args.camera_dropout),
            trajectory=str(args.trajectory),
            seed=int(args.seed),
            out_dir=args.out_dir,
        )
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    for k in [
        "mean_position_error_m",
        "mean_rotation_error_deg",
        "drop_rate",
        "frame_log",
        "pose_log",
        "eval_json",
    ]:
        print(f"{k}: {summary[k]}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
