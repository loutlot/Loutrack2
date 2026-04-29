"""Simulation utilities for synthetic camera projection.

This module also provides a small closed-loop simulation runner so the host
pipeline can be exercised deterministically without UDP receivers.
"""

from __future__ import annotations

from dataclasses import dataclass
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
from host.rigid import MarkerPattern, RigidBodyEstimator, WAIST_PATTERN, marker_pattern_from_points
from host.visualize import TrackingVisualizer
from host.pipeline import TrackingPipeline


DEFAULT_MVP_CAMERA_IDS = ("pi-cam-01", "pi-cam-02")
DEFAULT_GENERATED_CAMERA_IDS = ("pi-cam-01", "pi-cam-02", "pi-cam-03", "pi-cam-04")
DEFAULT_MVP_RIGIDS = ("waist", "wand")


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
    marker_layout: str = "current_4marker"
    camera_rig_source: str = "real_2cam"
    calibration_dir: str = "calibration"
    rigids_path: str = "calibration/tracking_rigids.json"
    pipeline_variant: str = "fast_ABCDHRF"
    subset_diagnostics_mode: str = "off"


@dataclass(frozen=True)
class SyntheticBlobOwner:
    timestamp: int
    camera_id: str
    emitted_blob_index: int
    rigid_name: str
    marker_index: int | None
    synthetic_blob_id: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp": int(self.timestamp),
            "camera_id": self.camera_id,
            "emitted_blob_index": int(self.emitted_blob_index),
            "rigid_name": self.rigid_name,
            "marker_index": None if self.marker_index is None else int(self.marker_index),
            "synthetic_blob_id": self.synthetic_blob_id,
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


def load_mvp_patterns(rigids_path: str | Path = "calibration/tracking_rigids.json") -> list[MarkerPattern]:
    custom = _load_custom_rigid_patterns(rigids_path)
    patterns = [WAIST_PATTERN]
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


def load_camera_rig(config: MultiRigidScenarioConfig) -> dict[str, CameraParams]:
    source = str(config.camera_rig_source)
    camera_ids = list(config.camera_ids)
    if source == "dummy":
        return create_dummy_calibration(camera_ids)
    if source == "generated_4cam_from_1_2_intrinsics":
        generated = _generated_4cam_from_1_2_intrinsics(config.calibration_dir)
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
        elif self.trajectory == "circle":
            pos += np.array([
                0.06 * np.cos(0.8 * t_sec + self.phase),
                0.06 * np.sin(0.8 * t_sec + self.phase),
                0.0,
            ])
            yaw += 0.4 * t_sec
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
        if not (0.0 <= config.marker_dropout_prob <= 1.0):
            raise ValueError("marker_dropout_prob must be in [0, 1]")
        if not (0.0 <= config.camera_dropout_prob <= 1.0):
            raise ValueError("camera_dropout_prob must be in [0, 1]")

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
        pattern_list = patterns or load_mvp_patterns(config.rigids_path)
        pattern_by_name = {pattern.name: pattern for pattern in pattern_list}
        self.patterns = [
            pattern_by_name[name]
            for name in config.rigid_names
            if name in pattern_by_name
        ]
        if not self.patterns:
            raise ValueError("no rigid patterns available for scenario")
        self._bodies = self._build_bodies(self.patterns)

    def _build_bodies(self, patterns: list[MarkerPattern]) -> dict[str, SimulatedRigidInstance]:
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
                    "waist_wand_occlusion_reacquire"
                    if self.config.scenario == "waist_wand_occlusion_reacquire"
                    else self.config.trajectory_name
                ),
                phase=float(index) * 0.7,
            )
        return bodies

    def _is_marker_occluded(self, rigid_name: str, marker_index: int, frame_index: int) -> bool:
        if self.config.scenario != "waist_wand_occlusion_reacquire":
            return False
        if rigid_name != "waist":
            return False
        start = int(round(self.config.frames * 0.35))
        end = int(round(self.config.frames * 0.65))
        return start <= frame_index < end

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
                    if self._is_marker_occluded(rigid_name, marker_index, frame_index):
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
                            "area": float(max(4.0, pattern.marker_diameter * 3600.0)),
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
            for false_index in range(self.config.false_blobs_per_camera):
                blob_index = len(blobs)
                blobs.append(
                    {
                        "x": float(self._rng.uniform(20.0, max(21.0, width - 20.0))),
                        "y": float(self._rng.uniform(20.0, max(21.0, height - 20.0))),
                        "area": 4.0,
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
) -> tuple[int, int]:
    wrong = 0
    marker_confusion = 0
    for point in snapshot.get("rigid_hint_triangulated_points", []) or []:
        rigid_name = point.get("rigid_name")
        marker_idx = point.get("marker_idx")
        if not rigid_name:
            continue
        for observation in point.get("observations", []) or []:
            camera_id = str(observation.get("camera_id", ""))
            blob_index = int(observation.get("blob_index", -1))
            owner = ledger.get((int(snapshot.get("timestamp", 0) or 0), camera_id, blob_index))
            if owner is None:
                continue
            if owner.rigid_name != rigid_name:
                wrong += 1
            elif owner.marker_index is not None and marker_idx is not None and int(owner.marker_index) != int(marker_idx):
                marker_confusion += 1
    return wrong, marker_confusion


def run_multi_rigid_scenario(
    config: MultiRigidScenarioConfig,
    *,
    out_dir: str | None = None,
) -> dict[str, Any]:
    """Run a deterministic multi-rigid scenario through TrackingPipeline."""
    patterns = load_mvp_patterns(config.rigids_path)
    camera_params = load_camera_rig(config)
    generator = MultiRigidFrameGenerator(config, patterns=patterns, camera_params=camera_params)

    pipeline = TrackingPipeline(
        enable_logging=False,
        patterns=[pattern for pattern in patterns if pattern.name in set(config.rigid_names)],
        pipeline_variant=config.pipeline_variant,
        subset_diagnostics_mode=config.subset_diagnostics_mode,
    )
    _install_camera_params_on_pipeline(pipeline, camera_params)

    latest_poses: dict[str, Any] = {}
    errors: list[str] = []
    pipeline.set_pose_callback(lambda poses: latest_poses.update(poses))
    pipeline.set_error_callback(lambda exc: errors.append(repr(exc)))
    pipeline._running = True

    position_errors_by_rigid: dict[str, list[float]] = {name: [] for name in config.rigid_names}
    rotation_errors_by_rigid: dict[str, list[float]] = {name: [] for name in config.rigid_names}
    valid_frames_by_rigid: dict[str, int] = {name: 0 for name in config.rigid_names}
    emitted_ledger_entries = 0
    wrong_ownership_count = 0
    marker_source_confusion_count = 0
    processed_samples = 0
    started_ns = time.perf_counter_ns()

    for _ in range(int(config.frames)):
        sample = generator.next_sample()
        if sample is None:
            break
        emitted_ledger_entries += len(sample.ownership_ledger)
        if len(sample.paired.frames) < 2:
            continue
        latest_poses.clear()
        pipeline._on_paired_frames(sample.paired)
        processed_samples += 1
        snapshot = pipeline.get_latest_triangulation_snapshot()
        wrong, confused = _ownership_confusion_from_snapshot(snapshot, sample.ownership_ledger)
        wrong_ownership_count += wrong
        marker_source_confusion_count += confused
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
            valid_frames_by_rigid[rigid_name] = valid_frames_by_rigid.get(rigid_name, 0) + 1

    elapsed_ms = float(time.perf_counter_ns() - started_ns) / 1_000_000.0
    pipeline._running = False
    status = pipeline.get_status()
    diagnostics = status.get("diagnostics", {})
    stage_ms = diagnostics.get("pipeline_stage_ms", {})
    pipeline_pair_ms = dict(stage_ms.get("pipeline_pair_ms", {}))
    rigid_ms = dict(stage_ms.get("rigid_ms", {}))
    geometry_ms = dict(stage_ms.get("triangulation_ms", {}))
    production_go_no_go = {
        "pipeline_pair_p95_le_6ms": float(pipeline_pair_ms.get("p95", 0.0) or 0.0) <= 6.0,
        "pipeline_pair_max_le_8_475ms": float(pipeline_pair_ms.get("max", 0.0) or 0.0) <= 8.475,
        "rigid_p95_le_1_5ms": float(rigid_ms.get("p95", 0.0) or 0.0) <= 1.5,
    }
    production_go_no_go["passed"] = all(production_go_no_go.values())
    summary = {
        "scenario": config.scenario,
        "marker_layout": config.marker_layout,
        "camera_rig_source": config.camera_rig_source,
        "camera_ids": list(camera_params),
        "rigid_names": list(config.rigid_names),
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
        "position_error_m": {
            name: _summary(values) for name, values in position_errors_by_rigid.items()
        },
        "rotation_error_deg": {
            name: _summary(values) for name, values in rotation_errors_by_rigid.items()
        },
        "valid_frame_ratio": {
            name: float(valid_frames_by_rigid.get(name, 0)) / float(max(1, processed_samples))
            for name in config.rigid_names
        },
        "pipeline_pair_ms": pipeline_pair_ms,
        "rigid_ms": rigid_ms,
        "geometry_ms": geometry_ms,
        "production_go_no_go": production_go_no_go,
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
        default="real_2cam",
        choices=["real_2cam", "generated_4cam_from_1_2_intrinsics", "dummy"],
        help="Camera rig source for multi-rigid scenarios",
    )
    parser.add_argument("--false-blobs-per-camera", type=int, default=0)
    parser.add_argument("--pipeline-variant", type=str, default="fast_ABCDHRF")
    parser.add_argument("--subset-diagnostics-mode", type=str, default="off")
    parser.add_argument("--noise-px", type=float, default=0.0, help="Pixel noise stddev")
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
    if args.false_blobs_per_camera < 0:
        return _err("--false-blobs-per-camera must be >= 0")
    if args.trajectory not in {"static", "linear", "circle"}:
        return _err("--trajectory must be one of: static, linear, circle")

    if args.scenario:
        if args.camera_rig_source == "generated_4cam_from_1_2_intrinsics":
            camera_ids = DEFAULT_GENERATED_CAMERA_IDS
        else:
            camera_ids = DEFAULT_MVP_CAMERA_IDS
        try:
            summary = run_multi_rigid_scenario(
                MultiRigidScenarioConfig(
                    seed=int(args.seed),
                    camera_ids=tuple(camera_ids),
                    frames=int(args.frames),
                    fps=float(args.fps),
                    rigid_names=DEFAULT_MVP_RIGIDS,
                    scenario=str(args.scenario),
                    trajectory_name=str(args.trajectory),
                    noise_px=float(args.noise_px),
                    false_blobs_per_camera=int(args.false_blobs_per_camera),
                    marker_dropout_prob=float(args.marker_dropout),
                    camera_dropout_prob=float(args.camera_dropout),
                    marker_layout=str(args.marker_layout),
                    camera_rig_source=str(args.camera_rig_source),
                    calibration_dir=str(args.calibration),
                    rigids_path=str(args.rigids),
                    pipeline_variant=str(args.pipeline_variant),
                    subset_diagnostics_mode=str(args.subset_diagnostics_mode),
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
