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
DESIGN_5MARKER_CAD_MARKER_Z_LIFT_M = 0.007
BODY_MOUNT_SCENARIOS = {
    "five_rigid_body_mesh_lite_v1",
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
        points_m = np.asarray(_DESIGN_5MARKER_POINTS_MM[name], dtype=np.float64) * 0.001
        cad_marker_z_m = points_m[:, 2] + DESIGN_5MARKER_CAD_MARKER_Z_LIFT_M
        policy = {
            "boot_min_markers": 4,
            "reacquire_min_markers": 4,
            "continue_min_markers": 3,
            **_DESIGN_5MARKER_POLICIES[name],
        }
        patterns.append(
            marker_pattern_from_points(
                name,
                points_m,
                marker_diameter=0.014,
                metadata={
                    "source": "rigid_body_design_seed",
                    "marker_layout": DESIGN_5MARKER_LAYOUT,
                    "cad_marker_z_lift_m": DESIGN_5MARKER_CAD_MARKER_Z_LIFT_M,
                    "cad_marker_centroid_z_m": float(np.mean(cad_marker_z_m)),
                    "cad_marker_min_z_m": float(np.min(cad_marker_z_m)),
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


def _scale_pattern_marker_diameter(pattern: MarkerPattern, scale: float) -> MarkerPattern:
    if not np.isfinite(scale) or float(scale) <= 0.0 or abs(float(scale) - 1.0) < 1e-9:
        return pattern
    metadata = dict(pattern.metadata or {})
    metadata["sim_marker_diameter_scale"] = float(scale)
    return MarkerPattern(
        name=pattern.name,
        marker_positions=np.asarray(pattern.marker_positions, dtype=np.float64).copy(),
        marker_diameter=float(pattern.marker_diameter) * float(scale),
        metadata=metadata,
    )


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

    def camera_center_world(self) -> np.ndarray:
        return -self._rotation.T @ self._translation


def _rotation_error_deg(r_pred: np.ndarray, r_gt: np.ndarray) -> float:
    """Rotation error in degrees between two rotation matrices."""
    r_pred = np.asarray(r_pred, dtype=np.float64).reshape(3, 3)
    r_gt = np.asarray(r_gt, dtype=np.float64).reshape(3, 3)
    cos = (np.trace(r_pred @ r_gt.T) - 1.0) / 2.0
    cos = float(np.clip(cos, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos)))


def _summary(values: list[float]) -> dict[str, float]:
    if not values:
        return {
            "count": 0,
            "min": 0.0,
            "mean": 0.0,
            "p05": 0.0,
            "p95": 0.0,
            "max": 0.0,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "min": float(np.min(arr)),
        "mean": float(np.mean(arr)),
        "p05": float(np.percentile(arr, 5)),
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
        "anchor_guided_body_nbest": False,
        "temporal_body_nbest": True,
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


def _segment_segment_distance_with_t(
    a0: np.ndarray,
    a1: np.ndarray,
    b0: np.ndarray,
    b1: np.ndarray,
) -> tuple[float, float]:
    """Return distance between two segments and the closest parameter on a0-a1."""
    u = np.asarray(a1, dtype=np.float64) - np.asarray(a0, dtype=np.float64)
    v = np.asarray(b1, dtype=np.float64) - np.asarray(b0, dtype=np.float64)
    w = np.asarray(a0, dtype=np.float64) - np.asarray(b0, dtype=np.float64)
    a = float(np.dot(u, u))
    b = float(np.dot(u, v))
    c = float(np.dot(v, v))
    d = float(np.dot(u, w))
    e = float(np.dot(v, w))
    denom = a * c - b * b
    small = 1e-12

    if a <= small:
        sc = 0.0
        tc = float(np.clip(e / c, 0.0, 1.0)) if c > small else 0.0
    elif c <= small:
        tc = 0.0
        sc = float(np.clip(-d / a, 0.0, 1.0))
    elif denom <= small:
        sc = 0.0
        tc = float(np.clip(e / c, 0.0, 1.0))
    else:
        sc = float(np.clip((b * e - c * d) / denom, 0.0, 1.0))
        tc = float((a * e + b * sc) / c)
        if tc < 0.0:
            tc = 0.0
            sc = float(np.clip(-d / a, 0.0, 1.0))
        elif tc > 1.0:
            tc = 1.0
            sc = float(np.clip((b - d) / a, 0.0, 1.0))

    closest_a = np.asarray(a0, dtype=np.float64) + sc * u
    closest_b = np.asarray(b0, dtype=np.float64) + tc * v
    return float(np.linalg.norm(closest_a - closest_b)), float(sc)


def _segment_sphere_distance_with_t(
    a0: np.ndarray,
    a1: np.ndarray,
    center: np.ndarray,
) -> tuple[float, float]:
    segment = np.asarray(a1, dtype=np.float64) - np.asarray(a0, dtype=np.float64)
    denom = float(np.dot(segment, segment))
    if denom <= 1e-12:
        return float(np.linalg.norm(np.asarray(a0, dtype=np.float64) - center)), 0.0
    t = float(
        np.clip(
            np.dot(np.asarray(center, dtype=np.float64) - a0, segment) / denom,
            0.0,
            1.0,
        )
    )
    closest = np.asarray(a0, dtype=np.float64) + t * segment
    return float(np.linalg.norm(closest - np.asarray(center, dtype=np.float64))), t


def _unit_vector(vector: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    values = np.asarray(vector, dtype=np.float64).reshape(3)
    norm = float(np.linalg.norm(values))
    if norm > 1e-12:
        return values / norm
    fallback_values = np.asarray(fallback, dtype=np.float64).reshape(3)
    fallback_norm = float(np.linalg.norm(fallback_values))
    if fallback_norm > 1e-12:
        return fallback_values / fallback_norm
    return np.array([0.0, 0.0, 1.0], dtype=np.float64)


def _project_to_plane(
    vector: np.ndarray,
    normal: np.ndarray,
    fallback: np.ndarray,
) -> np.ndarray:
    unit_normal = _unit_vector(normal, np.array([0.0, 0.0, 1.0], dtype=np.float64))
    values = np.asarray(vector, dtype=np.float64).reshape(3)
    projected = values - unit_normal * float(np.dot(values, unit_normal))
    return _unit_vector(projected, fallback)


def _surface_mount_rotation(
    outward_normal: np.ndarray,
    preferred_x: np.ndarray,
) -> np.ndarray:
    z_axis = _unit_vector(outward_normal, np.array([0.0, -1.0, 0.0], dtype=np.float64))
    x_axis = _project_to_plane(
        preferred_x,
        z_axis,
        np.array([1.0, 0.0, 0.0], dtype=np.float64),
    )
    if float(np.linalg.norm(np.cross(x_axis, z_axis))) <= 1e-9:
        x_axis = _project_to_plane(
            np.array([0.0, 0.0, 1.0], dtype=np.float64),
            z_axis,
            np.array([1.0, 0.0, 0.0], dtype=np.float64),
        )
    y_axis = _unit_vector(np.cross(z_axis, x_axis), np.array([0.0, 1.0, 0.0]))
    x_axis = _unit_vector(np.cross(y_axis, z_axis), x_axis)
    return np.column_stack([x_axis, y_axis, z_axis])


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
    marker_detection_model: str = "iid_dropout"
    camera_dropout_prob: float = 0.0
    timestamp_jitter_us: int = 0
    occlusion_profile: str = "none"
    motion_profile: str = "linear"
    body_interaction_profile: str = "waist_wand"
    max_velocity_mps: float = 0.8
    max_angular_velocity_deg_s: float = 180.0
    body_mount_blob_merge_factor: float = 0.0
    centroid_noise_model: str = "constant"
    centroid_noise_reference_diameter_px: float = 4.0
    marker_diameter_scale: float = 1.0
    active_anchor_markers: bool = False
    mesh_lite_occlusion: str = "off"
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


@dataclass(frozen=True)
class MeshLiteMount:
    rigid_name: str
    position: np.ndarray
    rotation: np.ndarray
    normal: np.ndarray
    surface_position: np.ndarray


@dataclass(frozen=True)
class MeshLiteOccluder:
    name: str
    kind: str
    radius_m: float
    rigids: frozenset[str]
    center: np.ndarray | None = None
    a: np.ndarray | None = None
    b: np.ndarray | None = None


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
            points_array = np.asarray(points, dtype=np.float64)
            metadata: dict[str, Any] = {
                "source": "tracking_rigids",
                "tracking_policy": item.get("tracking_policy", {}),
            }
            cad_marker_z_lift_m = float(
                item.get(
                    "cad_marker_z_lift_m",
                    DESIGN_5MARKER_CAD_MARKER_Z_LIFT_M,
                )
                or 0.0
            )
            explicit_cad_centroid_z = item.get("cad_marker_centroid_z_m")
            explicit_cad_min_z = item.get("cad_marker_min_z_m")
            if explicit_cad_centroid_z is not None and explicit_cad_min_z is not None:
                metadata["cad_marker_z_lift_m"] = cad_marker_z_lift_m
                metadata["cad_marker_centroid_z_m"] = float(explicit_cad_centroid_z)
                metadata["cad_marker_min_z_m"] = float(explicit_cad_min_z)
            else:
                cad_marker_z_m = points_array[:, 2] + cad_marker_z_lift_m
                if np.all(cad_marker_z_m > 0.0):
                    metadata["cad_marker_z_lift_m"] = cad_marker_z_lift_m
                    metadata["cad_marker_centroid_z_m"] = float(np.mean(cad_marker_z_m))
                    metadata["cad_marker_min_z_m"] = float(np.min(cad_marker_z_m))
            patterns[name] = marker_pattern_from_points(
                name,
                points_array,
                marker_diameter=float(item.get("marker_diameter_m", 0.014) or 0.014),
                metadata=metadata,
            )
        except Exception:
            continue
    return patterns


def load_mvp_patterns(
    rigids_path: str | Path = "calibration/tracking_rigids.json",
    *,
    marker_layout: str = "current_4marker",
    marker_diameter_scale: float = 1.0,
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
    patterns = [
        _scale_pattern_marker_diameter(custom.get(pattern.name, pattern), marker_diameter_scale)
        for pattern in patterns
    ]
    if "wand" in custom:
        patterns.append(_scale_pattern_marker_diameter(custom["wand"], marker_diameter_scale))
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
            _scale_pattern_marker_diameter(
                MarkerPattern(
                    name="wand",
                    marker_positions=wand_points,
                    marker_diameter=0.014,
                    metadata={"source": "deterministic_fallback"},
                ),
                marker_diameter_scale,
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
        if config.centroid_noise_model not in {"constant", "diameter_scaled"}:
            raise ValueError("centroid_noise_model must be constant or diameter_scaled")
        if config.centroid_noise_reference_diameter_px <= 0.0:
            raise ValueError("centroid_noise_reference_diameter_px must be > 0")
        if config.marker_diameter_scale <= 0.0:
            raise ValueError("marker_diameter_scale must be > 0")
        if config.mesh_lite_occlusion not in {"off", "body_capsules"}:
            raise ValueError("mesh_lite_occlusion must be off or body_capsules")
        if (
            config.mesh_lite_occlusion == "body_capsules"
            and config.scenario not in BODY_MOUNT_SCENARIOS
        ):
            raise ValueError(
                "mesh_lite_occlusion=body_capsules currently requires a body-mount scenario"
            )
        if config.false_blobs_per_camera < 0:
            raise ValueError("false_blobs_per_camera must be >= 0")
        if config.body_mount_blob_merge_factor < 0.0:
            raise ValueError("body_mount_blob_merge_factor must be >= 0")
        if not (0.0 <= config.marker_dropout_prob <= 1.0):
            raise ValueError("marker_dropout_prob must be in [0, 1]")
        if config.marker_detection_model not in {"iid_dropout", "pi_snr"}:
            raise ValueError("marker_detection_model must be iid_dropout or pi_snr")
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
            marker_diameter_scale=config.marker_diameter_scale,
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
        self._marker_detection_drop_counts: Counter[tuple[str, str, int], int] = Counter()
        self._marker_detection_drop_probability_values: list[float] = []

    def _mesh_lite_body_pose_components(
        self,
        frame_index: int,
    ) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        return {
            rigid_name: body.pose_components(frame_index)
            for rigid_name, body in self._bodies.items()
        }

    def _mesh_lite_surface_model(
        self,
        body_poses: dict[str, tuple[np.ndarray, np.ndarray]],
    ) -> tuple[dict[str, MeshLiteMount], list[MeshLiteOccluder]]:
        if self.config.scenario not in BODY_MOUNT_SCENARIOS:
            return {}, []

        radii = {
            "head": 0.105,
            "torso": 0.115,
            "leg": 0.055,
        }
        front_local = np.array([0.0, -1.0, 0.0], dtype=np.float64)
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        positions = {
            name: np.asarray(position, dtype=np.float64).reshape(3)
            for name, (_rotation, position) in body_poses.items()
        }
        rotations = {
            name: np.asarray(rotation, dtype=np.float64).reshape(3, 3)
            for name, (rotation, _position) in body_poses.items()
        }
        normals: dict[str, np.ndarray] = {}
        for name in ("head", "chest", "waist"):
            rotation = rotations.get(name)
            if rotation is not None:
                normals[name] = _unit_vector(rotation @ front_local, front_local)

        waist_position = positions.get("waist")
        for name in ("left_foot", "right_foot"):
            foot_position = positions.get(name)
            rotation = rotations.get(name)
            fallback = rotation @ front_local if rotation is not None else front_local
            if waist_position is None or foot_position is None:
                normals[name] = _unit_vector(fallback, front_local)
                continue
            preferred = foot_position - waist_position
            preferred[2] = 0.0
            if float(np.linalg.norm(preferred)) <= 1e-9:
                preferred = fallback
            leg_axis = foot_position - waist_position
            normals[name] = _project_to_plane(preferred, leg_axis, fallback)

        mounts: dict[str, MeshLiteMount] = {}
        for name, position in positions.items():
            normal = normals.get(name)
            rotation = rotations.get(name)
            if normal is None or rotation is None:
                continue
            pattern = self._bodies.get(name).pattern if name in self._bodies else None
            if pattern is None:
                continue
            marker_positions = np.asarray(pattern.marker_positions, dtype=np.float64).reshape(-1, 3)
            metadata = dict(pattern.metadata or {})
            cad_centroid_z_m = metadata.get("cad_marker_centroid_z_m")
            if cad_centroid_z_m is None:
                min_local_z = float(np.min(marker_positions[:, 2])) if len(marker_positions) else 0.0
                marker_surface_clearance_m = 0.004
                mount_offset_m = marker_surface_clearance_m - min_local_z
            else:
                mount_offset_m = float(cad_centroid_z_m)
            mount_position = position + normal * mount_offset_m
            mounts[name] = MeshLiteMount(
                rigid_name=name,
                position=mount_position,
                rotation=_surface_mount_rotation(normal, rotation[:, 0]),
                normal=normal,
                surface_position=position,
            )

        primitive_centers: dict[str, np.ndarray] = {}
        for name, mount in mounts.items():
            if name == "head":
                radius = radii["head"]
            elif name in {"left_foot", "right_foot"}:
                radius = radii["leg"]
            else:
                radius = radii["torso"]
            primitive_centers[name] = mount.surface_position - mount.normal * radius

        occluders: list[MeshLiteOccluder] = []
        if self.config.mesh_lite_occlusion != "body_capsules":
            return mounts, occluders
        head_center = primitive_centers.get("head")
        if head_center is not None:
            occluders.append(
                MeshLiteOccluder(
                    name="head",
                    kind="sphere",
                    radius_m=radii["head"],
                    rigids=frozenset({"head"}),
                    center=head_center,
                )
            )
        chest_center = primitive_centers.get("chest")
        waist_center = primitive_centers.get("waist")
        if chest_center is not None and waist_center is not None:
            occluders.append(
                MeshLiteOccluder(
                    name="torso",
                    kind="capsule",
                    radius_m=radii["torso"],
                    rigids=frozenset({"chest", "waist"}),
                    a=chest_center,
                    b=waist_center,
                )
            )
        left_center = primitive_centers.get("left_foot")
        if waist_center is not None and left_center is not None:
            occluders.append(
                MeshLiteOccluder(
                    name="left_leg",
                    kind="capsule",
                    radius_m=radii["leg"],
                    rigids=frozenset({"left_foot"}),
                    a=waist_center,
                    b=left_center,
                )
            )
        right_center = primitive_centers.get("right_foot")
        if waist_center is not None and right_center is not None:
            occluders.append(
                MeshLiteOccluder(
                    name="right_leg",
                    kind="capsule",
                    radius_m=radii["leg"],
                    rigids=frozenset({"right_foot"}),
                    a=waist_center,
                    b=right_center,
                )
            )
        return mounts, occluders

    def _body_mount_pose_components(
        self,
        frame_index: int,
    ) -> tuple[dict[str, tuple[np.ndarray, np.ndarray]], list[MeshLiteOccluder]]:
        body_poses = self._mesh_lite_body_pose_components(frame_index)
        mounts, occluders = self._mesh_lite_surface_model(body_poses)
        pose_components: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for rigid_name, (rotation, position) in body_poses.items():
            mount = mounts.get(rigid_name)
            if mount is None:
                pose_components[rigid_name] = (rotation, position)
            else:
                pose_components[rigid_name] = (mount.rotation, mount.position)
        return pose_components, occluders

    def _marker_positions_world_for_frame(
        self,
        frame_index: int,
    ) -> tuple[
        dict[str, tuple[np.ndarray, np.ndarray]],
        dict[str, np.ndarray],
        list[MeshLiteOccluder],
    ]:
        pose_components, occluders = self._body_mount_pose_components(frame_index)
        marker_world_by_rigid = {
            rigid_name: (
                rotation
                @ np.asarray(self._bodies[rigid_name].pattern.marker_positions, dtype=np.float64).T
            ).T
            + position.reshape(1, 3)
            for rigid_name, (rotation, position) in pose_components.items()
            if rigid_name in self._bodies
        }
        return pose_components, marker_world_by_rigid, occluders

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
                    "head": np.array([0.00, -0.055, 1.55], dtype=np.float64),
                    "chest": np.array([0.00, -0.030, 1.15], dtype=np.float64),
                    "waist": np.array([0.00, -0.010, 0.84], dtype=np.float64),
                    "left_foot": np.array([-0.105, -0.015, 0.08], dtype=np.float64),
                    "right_foot": np.array([0.105, 0.015, 0.08], dtype=np.float64),
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
                        "five_rigid_body_mesh_lite_v1",
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
        _pose_components, marker_world_by_rigid, _occluders = (
            self._marker_positions_world_for_frame(0)
        )
        for camera_id, camera in self._cameras.items():
            depths: list[float] = []
            for points in marker_world_by_rigid.values():
                for point in points:
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
            area_px2 = self._depth_adjusted_blob_area(
                camera_id,
                self._real_log_like_blob_area(camera_id),
                marker_world,
            )
            diameter_scale = float(pattern.marker_diameter) / 0.014
            return float(area_px2 * max(0.01, diameter_scale * diameter_scale))
        return float(max(4.0, pattern.marker_diameter * 3600.0))

    def _effective_centroid_noise_px(self, area_px2: float) -> float:
        noise_px = float(self.config.noise_px)
        if noise_px <= 0.0 or self.config.centroid_noise_model == "constant":
            return noise_px
        diameter_px = float(np.sqrt(max(1e-9, 4.0 * float(area_px2) / np.pi)))
        reference_diameter_px = float(self.config.centroid_noise_reference_diameter_px)
        return float(noise_px * reference_diameter_px / max(diameter_px, 1e-6))

    def _marker_detection_drop_probability(
        self,
        *,
        area_px2: float,
        is_active_anchor: bool,
    ) -> float:
        if is_active_anchor or self.config.marker_dropout_prob <= 0.0:
            return 0.0
        dropout = float(self.config.marker_dropout_prob)
        if self.config.marker_detection_model == "iid_dropout":
            return dropout

        diameter_px = float(np.sqrt(max(1e-9, 4.0 * float(area_px2) / np.pi)))
        reference_diameter_px = float(self.config.centroid_noise_reference_diameter_px)
        snr_shortfall = max(0.0, reference_diameter_px / max(diameter_px, 1e-6) - 1.0)
        low_snr_risk = float(np.clip(snr_shortfall * snr_shortfall, 0.0, 1.0))
        return float(np.clip(dropout * low_snr_risk, 0.0, 1.0))

    def _should_drop_marker_detection(
        self,
        *,
        camera_id: str,
        rigid_name: str,
        marker_index: int,
        area_px2: float,
        is_active_anchor: bool,
    ) -> bool:
        probability = self._marker_detection_drop_probability(
            area_px2=area_px2,
            is_active_anchor=is_active_anchor,
        )
        if probability <= 0.0:
            return False
        self._marker_detection_drop_probability_values.append(float(probability))
        if float(self._rng.random()) >= probability:
            return False
        self._marker_detection_drop_counts[
            (str(camera_id), str(rigid_name), int(marker_index))
        ] += 1
        return True

    def marker_detection_summary(self) -> dict[str, Any]:
        by_rigid: Counter[str] = Counter()
        by_camera: Counter[str] = Counter()
        for (camera_id, rigid_name, _marker_index), count in (
            self._marker_detection_drop_counts.items()
        ):
            by_camera[str(camera_id)] += int(count)
            by_rigid[str(rigid_name)] += int(count)
        return {
            "model": str(self.config.marker_detection_model),
            "drop_count": int(sum(self._marker_detection_drop_counts.values())),
            "drop_count_by_camera": dict(sorted(by_camera.items())),
            "drop_count_by_rigid": dict(sorted(by_rigid.items())),
            "drop_probability": _summary(self._marker_detection_drop_probability_values),
        }

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

    def _is_marker_mesh_lite_occluded(
        self,
        rigid_name: str,
        camera_id: str,
        marker_world: np.ndarray,
        occluders: list[MeshLiteOccluder],
    ) -> bool:
        if self.config.mesh_lite_occlusion == "off":
            return False
        camera = self._cameras.get(camera_id)
        if camera is None:
            return False
        camera_center = camera.camera_center_world()
        marker = np.asarray(marker_world, dtype=np.float64).reshape(3)
        segment_length = float(np.linalg.norm(marker - camera_center))
        if segment_length <= 1e-9:
            return False

        # Keep a small endpoint clearance so a marker mounted on the surface of
        # its own simplified body volume is not hidden by the volume it belongs to.
        t_min = 0.015 / segment_length
        t_max = max(0.0, 1.0 - 0.035 / segment_length)
        for occluder in occluders:
            radius = float(occluder.radius_m)
            if radius <= 0.0:
                continue
            if occluder.kind == "sphere":
                if occluder.center is None:
                    continue
                distance, ray_t = _segment_sphere_distance_with_t(
                    camera_center,
                    marker,
                    np.asarray(occluder.center, dtype=np.float64),
                )
            else:
                if occluder.a is None or occluder.b is None:
                    continue
                distance, ray_t = _segment_segment_distance_with_t(
                    camera_center,
                    marker,
                    np.asarray(occluder.a, dtype=np.float64),
                    np.asarray(occluder.b, dtype=np.float64),
                )
            if not (t_min <= ray_t <= t_max):
                continue
            if distance > radius:
                continue
            return True
        return False

    def _is_marker_occluded(
        self,
        rigid_name: str,
        marker_index: int,
        frame_index: int,
        camera_id: str,
    ) -> bool:
        if self.config.scenario in BODY_MOUNT_SCENARIOS:
            if self.config.scenario == "five_rigid_body_mesh_lite_v1":
                return False
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
        (
            pose_components,
            marker_world_by_rigid,
            mesh_lite_occluders,
        ) = self._marker_positions_world_for_frame(frame_index)
        for rigid_name, (rotation, position) in pose_components.items():
            gt_poses[rigid_name] = _pattern_to_pose_dict(rotation, position)

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
                    marker_world = marker_world_by_rigid[rigid_name][marker_index]
                    if self._is_marker_mesh_lite_occluded(
                        rigid_name,
                        camera_id,
                        marker_world,
                        mesh_lite_occluders,
                    ):
                        continue
                    is_active_anchor = bool(
                        self.config.active_anchor_markers and marker_index == 0
                    )
                    projection = projections[camera_id][rigid_name][marker_index]
                    if projection is None:
                        continue
                    area_px2 = self._synthetic_blob_area(
                        camera_id,
                        pattern,
                        marker_world,
                    )
                    if self._should_drop_marker_detection(
                        camera_id=camera_id,
                        rigid_name=rigid_name,
                        marker_index=marker_index,
                        area_px2=area_px2,
                        is_active_anchor=is_active_anchor,
                    ):
                        continue
                    dx, dy = _sample_gaussian_pixel_noise(
                        self._rng,
                        self._effective_centroid_noise_px(area_px2),
                    )
                    blob_index = len(blobs)
                    blob_payload: dict[str, Any] = {
                        "x": float(projection[0] + dx),
                        "y": float(projection[1] + dy),
                        "area": float(area_px2),
                    }
                    if is_active_anchor:
                        blob_payload.update(
                            {
                                "active_anchor": True,
                                "active_anchor_rigid": str(rigid_name),
                                "active_anchor_marker": int(marker_index),
                            }
                        )
                    blobs.append(blob_payload)
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


def _visibility_phase(
    *,
    visible_marker_count: int,
    visible_camera_count: int,
    marker_view_count: int,
    expected_markers: int,
) -> str:
    if visible_marker_count <= 0 or marker_view_count <= 0:
        return "hidden"
    if visible_camera_count <= 1:
        return "one_camera_visible"
    if visible_marker_count >= expected_markers:
        return "full_visible"
    return "partial_visible"


def _evidence_phase(
    *,
    physical_phase: str,
    hint_marker_count: int,
    active_anchor_count: int,
    assigned_marker_views: int,
    expected_markers: int,
) -> str:
    if hint_marker_count >= expected_markers:
        return "full_3d_hint"
    if hint_marker_count >= 3:
        return "partial_3d_hint"
    if assigned_marker_views > 0 or active_anchor_count > 0:
        return "partial_2d_evidence"
    if physical_phase == "hidden":
        return "hidden_prediction_only"
    return "visible_no_solver_evidence"


def _new_phase_bucket() -> dict[str, Any]:
    return {
        "frames": 0,
        "valid": 0,
        "measurement_valid": 0,
        "prediction_hold_valid": 0,
        "invalid": 0,
        "pose_source_counts": Counter(),
        "position_error_m": [],
        "rotation_error_deg": [],
        "measurement_position_error_m": [],
        "measurement_rotation_error_deg": [],
        "prediction_hold_position_error_m": [],
        "prediction_hold_rotation_error_deg": [],
    }


def _record_phase_bucket(
    bucket: dict[str, Any],
    *,
    pose_valid: bool,
    pose_source: str,
    position_error_m: float | None,
    rotation_error_deg: float | None,
) -> None:
    bucket["frames"] = int(bucket.get("frames", 0)) + 1
    bucket["pose_source_counts"][str(pose_source or "unknown")] += 1
    hold_source = str(pose_source) in {
        "prediction_hold",
        "prediction_hold_2d_constrained",
        "pose_continuity_guard",
        "ownership_conflict_prediction_hold",
    }
    if pose_valid:
        bucket["valid"] = int(bucket.get("valid", 0)) + 1
        if hold_source:
            bucket["prediction_hold_valid"] = int(bucket.get("prediction_hold_valid", 0)) + 1
        else:
            bucket["measurement_valid"] = int(bucket.get("measurement_valid", 0)) + 1
        if position_error_m is not None:
            bucket["position_error_m"].append(float(position_error_m))
        if rotation_error_deg is not None:
            bucket["rotation_error_deg"].append(float(rotation_error_deg))
        if hold_source:
            if position_error_m is not None:
                bucket["prediction_hold_position_error_m"].append(float(position_error_m))
            if rotation_error_deg is not None:
                bucket["prediction_hold_rotation_error_deg"].append(float(rotation_error_deg))
        else:
            if position_error_m is not None:
                bucket["measurement_position_error_m"].append(float(position_error_m))
            if rotation_error_deg is not None:
                bucket["measurement_rotation_error_deg"].append(float(rotation_error_deg))
    else:
        bucket["invalid"] = int(bucket.get("invalid", 0)) + 1


def _finalize_phase_buckets(
    buckets: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for phase, bucket in sorted(buckets.items()):
        frames = max(1, int(bucket.get("frames", 0)))
        result[str(phase)] = {
            "frames": int(bucket.get("frames", 0)),
            "valid": int(bucket.get("valid", 0)),
            "invalid": int(bucket.get("invalid", 0)),
            "measurement_valid": int(bucket.get("measurement_valid", 0)),
            "prediction_hold_valid": int(bucket.get("prediction_hold_valid", 0)),
            "valid_ratio": float(int(bucket.get("valid", 0)) / frames),
            "measurement_valid_ratio": float(
                int(bucket.get("measurement_valid", 0)) / frames
            ),
            "prediction_hold_valid_ratio": float(
                int(bucket.get("prediction_hold_valid", 0)) / frames
            ),
            "pose_source_counts": dict(sorted(bucket.get("pose_source_counts", {}).items())),
            "position_error_m": _summary(bucket.get("position_error_m", [])),
            "rotation_error_deg": _summary(bucket.get("rotation_error_deg", [])),
            "measurement_position_error_m": _summary(
                bucket.get("measurement_position_error_m", [])
            ),
            "measurement_rotation_error_deg": _summary(
                bucket.get("measurement_rotation_error_deg", [])
            ),
            "prediction_hold_position_error_m": _summary(
                bucket.get("prediction_hold_position_error_m", [])
            ),
            "prediction_hold_rotation_error_deg": _summary(
                bucket.get("prediction_hold_rotation_error_deg", [])
            ),
        }
    return result


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
        marker_diameter_scale=config.marker_diameter_scale,
    )
    patterns_by_name = {pattern.name: pattern for pattern in patterns}
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
    position_delta_events_by_rigid: dict[str, list[dict[str, Any]]] = {
        name: [] for name in config.rigid_names
    }
    rotation_delta_events_by_rigid: dict[str, list[dict[str, Any]]] = {
        name: [] for name in config.rigid_names
    }
    valid_frames_by_rigid: dict[str, int] = {name: 0 for name in config.rigid_names}
    last_valid_pose_by_rigid: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
    phase_eval_by_rigid: dict[str, dict[str, dict[str, dict[str, Any]]]] = {
        rigid_name: {"physical": {}, "evidence": {}}
        for rigid_name in config.rigid_names
    }
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
    blob_area_values_by_rigid: dict[str, list[float]] = {
        rigid_name: [] for rigid_name in config.rigid_names
    }
    blob_nearest_distance_by_camera: dict[str, list[float]] = {
        camera_id: [] for camera_id in config.camera_ids
    }
    blob_nearest_distance_by_rigid: dict[str, list[float]] = {
        rigid_name: [] for rigid_name in config.rigid_names
    }
    marker_visibility_by_rigid: dict[str, dict[str, list[float]]] = {
        rigid_name: {
            "visible_marker_count": [],
            "marker_view_count": [],
            "visible_camera_count": [],
            "two_ray_marker_count": [],
        }
        for rigid_name in config.rigid_names
    }
    marker_visibility_hist_by_rigid: dict[str, dict[int, int]] = {
        rigid_name: {count: 0 for count in range(6)} for rigid_name in config.rigid_names
    }
    marker_visibility_by_frame: dict[int, dict[str, dict[str, Any]]] = {}
    rigid_hint_visibility_by_frame: dict[int, dict[str, dict[str, Any]]] = {}
    generic_3d_coverage_threshold_m = 0.02
    generic_3d_coverage_by_rigid: dict[str, dict[str, list[float]]] = {
        rigid_name: {
            "covered_marker_count": [],
            "nearest_marker_point_distance_m": [],
        }
        for rigid_name in config.rigid_names
    }
    generic_3d_coverage_by_frame: dict[int, dict[str, dict[str, Any]]] = {}
    blob_quality_by_frame: dict[int, dict[str, dict[str, Any]]] = {}
    started_ns = time.perf_counter_ns()

    for _ in range(int(config.frames)):
        sample = generator.next_sample()
        if sample is None:
            break
        emitted_ledger_entries += len(sample.ownership_ledger)
        emitted_blob_owners.extend(sample.ownership_ledger.values())
        frame_blob_distances_by_rigid: dict[str, list[float]] = {
            rigid_name: [] for rigid_name in config.rigid_names
        }
        frame_blob_distances_by_camera_rigid: dict[str, dict[str, list[float]]] = {}
        for frame in sample.paired.frames.values():
            camera_id = str(frame.camera_id)
            camera_bucket = blob_area_values_by_camera.setdefault(camera_id, [])
            camera_distance_bucket = blob_nearest_distance_by_camera.setdefault(camera_id, [])
            blobs = list(frame.blobs or [])
            blob_uvs = np.asarray(
                [
                    [float(blob.get("x", 0.0) or 0.0), float(blob.get("y", 0.0) or 0.0)]
                    for blob in blobs
                ],
                dtype=np.float64,
            ).reshape(-1, 2)
            nearest_distances: list[float] = []
            if len(blob_uvs) >= 2:
                deltas = blob_uvs[:, None, :] - blob_uvs[None, :, :]
                distances = np.sqrt(np.sum(deltas * deltas, axis=2))
                np.fill_diagonal(distances, np.inf)
                nearest_distances = [
                    float(value) for value in np.min(distances, axis=1) if np.isfinite(value)
                ]
            else:
                nearest_distances = [0.0 for _ in blobs]
            for blob_index, blob in enumerate(blobs):
                try:
                    area = float(blob.get("area", 0.0) or 0.0)
                except Exception:
                    continue
                if area <= 0.0:
                    continue
                blob_area_values.append(area)
                camera_bucket.append(area)
                if blob_index < len(nearest_distances):
                    camera_distance_bucket.append(float(nearest_distances[blob_index]))
                owner = sample.ownership_ledger.get(
                    (int(frame.timestamp), camera_id, int(blob_index))
                )
                if owner is None:
                    continue
                rigid_bucket = blob_area_values_by_rigid.setdefault(
                    str(owner.rigid_name),
                    [],
                )
                rigid_bucket.append(area)
                if blob_index < len(nearest_distances):
                    blob_nearest_distance_by_rigid.setdefault(
                        str(owner.rigid_name),
                        [],
                    ).append(float(nearest_distances[blob_index]))
                    if str(owner.rigid_name) in frame_blob_distances_by_rigid:
                        frame_blob_distances_by_rigid[str(owner.rigid_name)].append(
                            float(nearest_distances[blob_index])
                        )
                        frame_blob_distances_by_camera_rigid.setdefault(
                            camera_id,
                            {},
                        ).setdefault(str(owner.rigid_name), []).append(
                            float(nearest_distances[blob_index])
                        )
        if len(sample.paired.frames) < 2:
            continue
        frame_index = min(
            (int(frame.frame_index) for frame in sample.paired.frames.values()),
            default=int(processed_samples),
        )
        _gt_pose_components, gt_marker_world_by_rigid, _gt_mesh_lite_occluders = (
            generator._marker_positions_world_for_frame(int(frame_index))
        )
        blob_quality_by_frame[int(frame_index)] = {
            rigid_name: {
                "nearest_blob_distance_px": _summary(values),
                "by_camera": {
                    camera_id: _summary(
                        frame_blob_distances_by_camera_rigid.get(camera_id, {}).get(
                            rigid_name,
                            [],
                        )
                    )
                    for camera_id in sorted(frame_blob_distances_by_camera_rigid)
                    if rigid_name
                    in frame_blob_distances_by_camera_rigid.get(camera_id, {})
                },
            }
            for rigid_name, values in frame_blob_distances_by_rigid.items()
        }
        visible_markers: dict[str, set[int]] = {
            rigid_name: set() for rigid_name in config.rigid_names
        }
        visible_cameras: dict[str, set[str]] = {
            rigid_name: set() for rigid_name in config.rigid_names
        }
        visible_marker_cameras: dict[str, dict[int, set[str]]] = {
            rigid_name: {} for rigid_name in config.rigid_names
        }
        marker_view_counts: dict[str, int] = {rigid_name: 0 for rigid_name in config.rigid_names}
        for owner in sample.ownership_ledger.values():
            rigid_name = str(owner.rigid_name)
            if rigid_name not in visible_markers or owner.marker_index is None:
                continue
            marker_index = int(owner.marker_index)
            visible_markers[rigid_name].add(marker_index)
            visible_cameras[rigid_name].add(str(owner.camera_id))
            visible_marker_cameras[rigid_name].setdefault(marker_index, set()).add(
                str(owner.camera_id)
            )
            marker_view_counts[rigid_name] += 1
        frame_visibility: dict[str, dict[str, Any]] = {}
        for rigid_name in config.rigid_names:
            marker_count = int(len(visible_markers[rigid_name]))
            view_count = int(marker_view_counts[rigid_name])
            camera_count = int(len(visible_cameras[rigid_name]))
            two_ray_count = int(
                sum(
                    1
                    for cameras in visible_marker_cameras[rigid_name].values()
                    if len(cameras) >= 2
                )
            )
            marker_visibility_by_rigid[rigid_name]["visible_marker_count"].append(
                float(marker_count)
            )
            marker_visibility_by_rigid[rigid_name]["marker_view_count"].append(
                float(view_count)
            )
            marker_visibility_by_rigid[rigid_name]["visible_camera_count"].append(
                float(camera_count)
            )
            marker_visibility_by_rigid[rigid_name]["two_ray_marker_count"].append(
                float(two_ray_count)
            )
            marker_visibility_hist_by_rigid[rigid_name][marker_count] = (
                int(marker_visibility_hist_by_rigid[rigid_name].get(marker_count, 0)) + 1
            )
            frame_visibility[rigid_name] = {
                "visible_marker_count": marker_count,
                "marker_view_count": view_count,
                "visible_camera_count": camera_count,
                "two_ray_marker_count": two_ray_count,
                "marker_indices": [int(value) for value in sorted(visible_markers[rigid_name])],
            }
        marker_visibility_by_frame[int(frame_index)] = frame_visibility
        latest_poses.clear()
        pipeline._on_paired_frames(sample.paired)
        current_rigid_hint_visibility: dict[str, dict[str, Any]] = {}
        if hasattr(pipeline.rigid_estimator, "get_last_rigid_hint_visibility"):
            current_rigid_hint_visibility = (
                pipeline.rigid_estimator.get_last_rigid_hint_visibility()
            )
            rigid_hint_visibility_by_frame[int(frame_index)] = current_rigid_hint_visibility
        current_tracking_status = pipeline.rigid_estimator.get_tracking_status()
        processed_samples += 1
        pipeline_pair_last_ms_values.append(
            _latest_pipeline_stage_ms(pipeline, "pipeline_pair_ms")
        )
        rigid_last_ms_values.append(_latest_pipeline_stage_ms(pipeline, "rigid_ms"))
        geometry_last_ms_values.append(_latest_pipeline_stage_ms(pipeline, "triangulation_ms"))
        snapshot = pipeline.get_latest_triangulation_snapshot()
        generic_points = np.asarray(
            [
                point.get("point", [0.0, 0.0, 0.0])
                for point in snapshot.get("triangulated_points", []) or []
                if isinstance(point, dict)
            ],
            dtype=np.float64,
        ).reshape(-1, 3)
        frame_generic_coverage: dict[str, dict[str, Any]] = {}
        for rigid_name in config.rigid_names:
            gt_markers = gt_marker_world_by_rigid.get(rigid_name)
            if gt_markers is None:
                continue
            gt_markers = np.asarray(
                gt_markers,
                dtype=np.float64,
            ).reshape(-1, 3)
            marker_distances: list[float] = []
            covered_indices: list[int] = []
            for marker_index, marker_point in enumerate(gt_markers):
                if len(generic_points) == 0:
                    nearest = float("inf")
                else:
                    nearest = float(
                        np.min(
                            np.linalg.norm(
                                generic_points - marker_point.reshape(1, 3),
                                axis=1,
                            )
                        )
                    )
                marker_distances.append(nearest)
                if nearest <= generic_3d_coverage_threshold_m:
                    covered_indices.append(int(marker_index))
            finite_distances = [
                distance for distance in marker_distances if np.isfinite(distance)
            ]
            generic_3d_coverage_by_rigid[rigid_name][
                "covered_marker_count"
            ].append(float(len(covered_indices)))
            generic_3d_coverage_by_rigid[rigid_name][
                "nearest_marker_point_distance_m"
            ].extend(finite_distances)
            frame_generic_coverage[rigid_name] = {
                "covered_marker_count": int(len(covered_indices)),
                "covered_marker_indices": covered_indices,
                "nearest_marker_point_distance_m": [
                    float(distance) if np.isfinite(distance) else None
                    for distance in marker_distances
                ],
            }
        generic_3d_coverage_by_frame[int(frame_index)] = frame_generic_coverage
        wrong, confused, events = _ownership_confusion_from_snapshot(
            snapshot,
            sample.ownership_ledger,
        )
        for event in events:
            event["frame_index"] = int(frame_index)
        wrong_ownership_count += wrong
        marker_source_confusion_count += confused
        ownership_confusion_events.extend(events)
        for rigid_name, gt_pose in sample.gt_poses.items():
            pose = latest_poses.get(rigid_name)
            gt_position = np.asarray(gt_pose["position_m"], dtype=np.float64)
            gt_rotation = np.asarray(gt_pose["rotation"], dtype=np.float64)
            pose_valid = bool(pose is not None and getattr(pose, "valid", False))
            tracking_payload = (
                current_tracking_status.get(rigid_name, {})
                if isinstance(current_tracking_status, dict)
                else {}
            )
            pose_source = (
                str(tracking_payload.get("last_pose_source", "unknown"))
                if isinstance(tracking_payload, dict)
                else "unknown"
            )
            visibility_payload = frame_visibility.get(rigid_name, {})
            pattern = patterns_by_name.get(rigid_name)
            expected_markers = int(pattern.num_markers) if pattern is not None else 5
            physical_phase = _visibility_phase(
                visible_marker_count=int(
                    visibility_payload.get("visible_marker_count", 0) or 0
                ),
                visible_camera_count=int(
                    visibility_payload.get("visible_camera_count", 0) or 0
                ),
                marker_view_count=int(visibility_payload.get("marker_view_count", 0) or 0),
                expected_markers=expected_markers,
            )
            hint_payload = current_rigid_hint_visibility.get(rigid_name, {})
            object_gating = (
                tracking_payload.get("object_gating", {})
                if isinstance(tracking_payload, dict)
                else {}
            )
            evidence_phase = _evidence_phase(
                physical_phase=physical_phase,
                hint_marker_count=int(hint_payload.get("marker_count", 0) or 0),
                active_anchor_count=int(
                    object_gating.get("active_anchor_assignment_count", 0) or 0
                )
                if isinstance(object_gating, dict)
                else 0,
                assigned_marker_views=int(
                    object_gating.get("assigned_marker_views", 0) or 0
                )
                if isinstance(object_gating, dict)
                else 0,
                expected_markers=expected_markers,
            )
            position_error: float | None = None
            rotation_error: float | None = None
            if pose_valid:
                position_error = float(
                    np.linalg.norm(np.asarray(pose.position, dtype=np.float64) - gt_position)
                )
                rotation_error = _rotation_error_deg(
                    np.asarray(pose.rotation, dtype=np.float64), gt_rotation
                )
                position_errors_by_rigid.setdefault(rigid_name, []).append(position_error)
                rotation_errors_by_rigid.setdefault(rigid_name, []).append(rotation_error)
            physical_bucket = phase_eval_by_rigid[rigid_name]["physical"].setdefault(
                physical_phase,
                _new_phase_bucket(),
            )
            evidence_bucket = phase_eval_by_rigid[rigid_name]["evidence"].setdefault(
                evidence_phase,
                _new_phase_bucket(),
            )
            _record_phase_bucket(
                physical_bucket,
                pose_valid=pose_valid,
                pose_source=pose_source,
                position_error_m=position_error,
                rotation_error_deg=rotation_error,
            )
            _record_phase_bucket(
                evidence_bucket,
                pose_valid=pose_valid,
                pose_source=pose_source,
                position_error_m=position_error,
                rotation_error_deg=rotation_error,
            )
            if not pose_valid:
                continue
            pose_position = np.asarray(pose.position, dtype=np.float64).reshape(3)
            pose_rotation = np.asarray(pose.rotation, dtype=np.float64).reshape(3, 3)
            previous = last_valid_pose_by_rigid.get(rigid_name)
            if previous is not None:
                prev_pose_position, prev_pose_rotation, prev_gt_position, prev_gt_rotation = previous
                estimated_delta = pose_position - prev_pose_position
                gt_delta = gt_position - prev_gt_position
                position_delta_error = float(np.linalg.norm(estimated_delta - gt_delta))
                position_delta_errors_by_rigid.setdefault(rigid_name, []).append(
                    position_delta_error
                )
                estimated_rotation_delta = pose_rotation @ prev_pose_rotation.T
                gt_rotation_delta = gt_rotation @ prev_gt_rotation.T
                rotation_delta_error = _rotation_error_deg(
                    estimated_rotation_delta,
                    gt_rotation_delta,
                )
                rotation_delta_errors_by_rigid.setdefault(rigid_name, []).append(
                    rotation_delta_error
                )
                position_delta_events_by_rigid.setdefault(rigid_name, []).append(
                    {
                        "frame_index": int(frame_index),
                        "timestamp": int(sample.paired.timestamp),
                        "error_m": position_delta_error,
                    }
                )
                rotation_delta_events_by_rigid.setdefault(rigid_name, []).append(
                    {
                        "frame_index": int(frame_index),
                        "timestamp": int(sample.paired.timestamp),
                        "error_deg": rotation_delta_error,
                    }
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
    phase_evaluation_by_rigid = {
        rigid_name: {
            "physical": _finalize_phase_buckets(phase_payload.get("physical", {})),
            "evidence": _finalize_phase_buckets(phase_payload.get("evidence", {})),
        }
        for rigid_name, phase_payload in sorted(phase_eval_by_rigid.items())
    }
    full_3d_measurement_ratios: list[float] = []
    hold_position_max_values: list[float] = []
    hold_rotation_max_values: list[float] = []
    for phase_payload in phase_evaluation_by_rigid.values():
        full_3d = phase_payload.get("evidence", {}).get("full_3d_hint", {})
        if int(full_3d.get("frames", 0) or 0) > 0:
            full_3d_measurement_ratios.append(
                float(full_3d.get("measurement_valid_ratio", 0.0) or 0.0)
            )
        for evidence_bucket in phase_payload.get("evidence", {}).values():
            hold_position_max_values.append(
                float(
                    evidence_bucket.get("prediction_hold_position_error_m", {}).get(
                        "max",
                        0.0,
                    )
                    or 0.0
                )
            )
            hold_rotation_max_values.append(
                float(
                    evidence_bucket.get("prediction_hold_rotation_error_deg", {}).get(
                        "max",
                        0.0,
                    )
                    or 0.0
                )
            )
    phase_aware_go_no_go = {
        "full_3d_hint_measurement_valid_ratio_ge_0_95": all(
            value >= 0.95 for value in full_3d_measurement_ratios
        ),
        "prediction_hold_position_error_max_le_5cm": (
            max(hold_position_max_values, default=0.0) <= 0.05
        ),
        "prediction_hold_rotation_error_max_le_10deg": (
            max(hold_rotation_max_values, default=0.0) <= 10.0
        ),
    }
    phase_aware_go_no_go["passed"] = all(
        bool(value) for value in phase_aware_go_no_go.values()
    )
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
    blob_quality_summary_by_camera = {
        camera_id: {
            "area_px2": _summary(blob_area_values_by_camera.get(camera_id, [])),
            "equivalent_diameter_px": _summary(
                [
                    float(np.sqrt(4.0 * area / np.pi))
                    for area in blob_area_values_by_camera.get(camera_id, [])
                    if area > 0.0
                ]
            ),
            "nearest_blob_distance_px": _summary(
                blob_nearest_distance_by_camera.get(camera_id, [])
            ),
        }
        for camera_id in sorted(blob_area_values_by_camera)
    }
    blob_quality_summary_by_rigid = {
        rigid_name: {
            "area_px2": _summary(blob_area_values_by_rigid.get(rigid_name, [])),
            "equivalent_diameter_px": _summary(
                [
                    float(np.sqrt(4.0 * area / np.pi))
                    for area in blob_area_values_by_rigid.get(rigid_name, [])
                    if area > 0.0
                ]
            ),
            "nearest_blob_distance_px": _summary(
                blob_nearest_distance_by_rigid.get(rigid_name, [])
            ),
        }
        for rigid_name in sorted(blob_area_values_by_rigid)
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
    passive_stabilization_v2 = dict(
        dict(variant_metrics).get("passive_stabilization_v2", {})
    )
    marker_visibility_summary_by_rigid = {
        rigid_name: {
            "visible_marker_count": _summary(
                values.get("visible_marker_count", [])
            ),
            "marker_view_count": _summary(values.get("marker_view_count", [])),
            "visible_camera_count": _summary(
                values.get("visible_camera_count", [])
            ),
            "two_ray_marker_count": _summary(
                values.get("two_ray_marker_count", [])
            ),
            "visible_marker_count_histogram": {
                str(count): int(value)
                for count, value in sorted(
                    marker_visibility_hist_by_rigid.get(rigid_name, {}).items()
                )
            },
        }
        for rigid_name, values in sorted(marker_visibility_by_rigid.items())
    }
    shape_observability_summary_by_rigid = {}
    for rigid_name, values in sorted(marker_visibility_by_rigid.items()):
        marker_counts = [
            float(value) for value in values.get("visible_marker_count", [])
        ]
        view_counts = [float(value) for value in values.get("marker_view_count", [])]
        camera_counts = [
            float(value) for value in values.get("visible_camera_count", [])
        ]
        two_ray_counts = [
            float(value) for value in values.get("two_ray_marker_count", [])
        ]
        frame_count = float(max(1, len(marker_counts)))
        shape_observability_summary_by_rigid[rigid_name] = {
            "frames": int(len(marker_counts)),
            "visible_markers_ge_5_ratio": float(
                sum(1 for value in marker_counts if value >= 5.0) / frame_count
            ),
            "visible_markers_ge_4_ratio": float(
                sum(1 for value in marker_counts if value >= 4.0) / frame_count
            ),
            "visible_markers_ge_3_ratio": float(
                sum(1 for value in marker_counts if value >= 3.0) / frame_count
            ),
            "visible_cameras_ge_2_ratio": float(
                sum(1 for value in camera_counts if value >= 2.0) / frame_count
            ),
            "two_ray_markers_ge_4_ratio": float(
                sum(1 for value in two_ray_counts if value >= 4.0) / frame_count
            ),
            "two_ray_markers_ge_3_ratio": float(
                sum(1 for value in two_ray_counts if value >= 3.0) / frame_count
            ),
            "marker_views_ge_8_ratio": float(
                sum(1 for value in view_counts if value >= 8.0) / frame_count
            ),
            "marker_views_ge_6_ratio": float(
                sum(1 for value in view_counts if value >= 6.0) / frame_count
            ),
            "min_visible_marker_count": float(min(marker_counts, default=0.0)),
            "min_two_ray_marker_count": float(min(two_ray_counts, default=0.0)),
            "min_marker_view_count": float(min(view_counts, default=0.0)),
            "min_visible_camera_count": float(min(camera_counts, default=0.0)),
        }
    top_delta_frame_indices = {
        int(event["frame_index"])
        for events in list(position_delta_events_by_rigid.values())
        + list(rotation_delta_events_by_rigid.values())
        for event in sorted(
            events,
            key=lambda payload: max(
                float(payload.get("error_m", 0.0)),
                float(payload.get("error_deg", 0.0)),
            ),
            reverse=True,
        )[:5]
        if "frame_index" in event
    }
    marker_visibility_top_delta_frames = {
        str(frame_index): marker_visibility_by_frame.get(frame_index, {})
        for frame_index in sorted(top_delta_frame_indices)
    }
    rigid_hint_visibility_top_delta_frames = {
        str(frame_index): rigid_hint_visibility_by_frame.get(frame_index, {})
        for frame_index in sorted(top_delta_frame_indices)
    }
    generic_3d_coverage_summary_by_rigid = {
        rigid_name: {
            "covered_marker_count": _summary(
                values.get("covered_marker_count", [])
            ),
            "nearest_marker_point_distance_m": _summary(
                values.get("nearest_marker_point_distance_m", [])
            ),
            "coverage_threshold_m": float(generic_3d_coverage_threshold_m),
        }
        for rigid_name, values in sorted(generic_3d_coverage_by_rigid.items())
    }
    generic_3d_coverage_top_delta_frames = {
        str(frame_index): generic_3d_coverage_by_frame.get(frame_index, {})
        for frame_index in sorted(top_delta_frame_indices)
    }
    blob_quality_top_delta_frames = {
        str(frame_index): blob_quality_by_frame.get(frame_index, {})
        for frame_index in sorted(top_delta_frame_indices)
    }
    passive_stabilization_v2.update(
        {
            "blob_quality_summary_by_camera": blob_quality_summary_by_camera,
            "blob_quality_summary_by_rigid": blob_quality_summary_by_rigid,
            "blob_quality_top_delta_frames": blob_quality_top_delta_frames,
            "marker_visibility_summary_by_rigid": marker_visibility_summary_by_rigid,
            "shape_observability_summary_by_rigid": shape_observability_summary_by_rigid,
            "marker_visibility_top_delta_frames": marker_visibility_top_delta_frames,
            "rigid_hint_visibility_top_delta_frames": rigid_hint_visibility_top_delta_frames,
            "generic_3d_coverage_summary_by_rigid": generic_3d_coverage_summary_by_rigid,
            "generic_3d_coverage_top_delta_frames": generic_3d_coverage_top_delta_frames,
            "phase_evaluation_by_rigid": phase_evaluation_by_rigid,
            "phase_aware_go_no_go": phase_aware_go_no_go,
        }
    )
    summary = {
        "scenario": config.scenario,
        "marker_layout": config.marker_layout,
        "rigids_path": str(config.rigids_path),
        "camera_rig_source": config.camera_rig_source,
        "pipeline_variant": config.pipeline_variant,
        "subset_diagnostics_mode": config.subset_diagnostics_mode,
        "rigid_stabilization_profile": config.rigid_stabilization_profile,
        "rigid_stabilization_overrides": dict(config.rigid_stabilization_overrides or {}),
        "camera_ids": list(camera_params),
        "rigid_names": list(config.rigid_names),
        "body_mount_blob_merge_factor": float(config.body_mount_blob_merge_factor),
        "centroid_noise_model": config.centroid_noise_model,
        "centroid_noise_reference_diameter_px": float(
            config.centroid_noise_reference_diameter_px
        ),
        "marker_diameter_scale": float(config.marker_diameter_scale),
        "marker_dropout_prob": float(config.marker_dropout_prob),
        "marker_detection_model": str(config.marker_detection_model),
        "camera_dropout_prob": float(config.camera_dropout_prob),
        "active_anchor_markers": bool(config.active_anchor_markers),
        "mesh_lite_occlusion": str(config.mesh_lite_occlusion),
        "marker_detection_summary": generator.marker_detection_summary(),
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
        "passive_stabilization_v2": passive_stabilization_v2,
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
        "position_delta_top_events": {
            name: sorted(
                events,
                key=lambda event: float(event.get("error_m", 0.0)),
                reverse=True,
            )[:5]
            for name, events in position_delta_events_by_rigid.items()
        },
        "rotation_delta_top_events": {
            name: sorted(
                events,
                key=lambda event: float(event.get("error_deg", 0.0)),
                reverse=True,
            )[:5]
            for name, events in rotation_delta_events_by_rigid.items()
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
        "phase_aware_go_no_go": phase_aware_go_no_go,
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
    parser.add_argument(
        "--anchor-guided-body-nbest",
        type=_parse_bool_override,
        default=None,
        help="Enable experimental active-anchor-guided non-anchor body N-best recovery",
    )
    parser.add_argument(
        "--temporal-body-nbest",
        type=_parse_bool_override,
        default=None,
        help="Enable passive body-level N-best marker assignment with temporal rotation cost",
    )
    parser.add_argument("--noise-px", type=float, default=0.0, help="Pixel noise stddev")
    parser.add_argument(
        "--centroid-noise-model",
        type=str,
        default="constant",
        choices=["constant", "diameter_scaled"],
        help="Centroid noise model for physical-layer sim PDCA",
    )
    parser.add_argument(
        "--centroid-noise-reference-diameter-px",
        type=float,
        default=4.0,
        help="Equivalent blob diameter where --noise-px is measured for diameter_scaled noise",
    )
    parser.add_argument(
        "--marker-diameter-scale",
        type=float,
        default=1.0,
        help="Scale synthetic marker diameters for physical-layer sim PDCA",
    )
    parser.add_argument(
        "--active-anchor-markers",
        type=_parse_bool_override,
        default=False,
        help="Emit one decoded active anchor blob per rigid for physical-ID PDCA",
    )
    parser.add_argument(
        "--mesh-lite-occlusion",
        type=str,
        default="off",
        choices=["off", "body_capsules"],
        help="Add simplified body-volume occlusion for rigid shape observability comparison",
    )
    parser.add_argument(
        "--body-mount-blob-merge-factor",
        type=float,
        default=0.0,
        help="Merge close true blobs in body-mounted sim when distance <= factor * average equivalent diameter",
    )
    parser.add_argument(
        "--marker-detection-model",
        type=str,
        default="iid_dropout",
        choices=["iid_dropout", "pi_snr"],
        help="Marker detection loss model: iid_dropout keeps the legacy random loss; pi_snr scales loss by projected blob size",
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
    if args.centroid_noise_reference_diameter_px <= 0.0:
        return _err("--centroid-noise-reference-diameter-px must be > 0")
    if args.marker_diameter_scale <= 0.0:
        return _err("--marker-diameter-scale must be > 0")
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
        if args.anchor_guided_body_nbest is not None:
            rigid_stabilization_overrides["anchor_guided_body_nbest"] = bool(
                args.anchor_guided_body_nbest
            )
        if args.temporal_body_nbest is not None:
            rigid_stabilization_overrides["temporal_body_nbest"] = bool(
                args.temporal_body_nbest
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
                    centroid_noise_model=str(args.centroid_noise_model),
                    centroid_noise_reference_diameter_px=float(
                        args.centroid_noise_reference_diameter_px
                    ),
                    marker_diameter_scale=float(args.marker_diameter_scale),
                    active_anchor_markers=bool(args.active_anchor_markers),
                    mesh_lite_occlusion=str(args.mesh_lite_occlusion),
                    body_mount_blob_merge_factor=float(
                        args.body_mount_blob_merge_factor
                    ),
                    false_blobs_per_camera=int(args.false_blobs_per_camera),
                    marker_dropout_prob=float(args.marker_dropout),
                    marker_detection_model=str(args.marker_detection_model),
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
