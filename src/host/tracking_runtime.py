"""GUI-facing runtime wrapper around TrackingPipeline."""

from __future__ import annotations

import threading
import time
from collections import defaultdict, deque
from typing import Any, Deque, Dict, List, Optional

import numpy as np

from .geo import CameraParams
from .pipeline import TrackingPipeline
from .rigid import (
    CHEST_PATTERN,
    HEAD_PATTERN,
    LEFT_FOOT_PATTERN,
    MarkerPattern,
    RIGHT_FOOT_PATTERN,
    RigidBodyPose,
    WAIST_PATTERN,
)

AVAILABLE_PATTERNS: Dict[str, MarkerPattern] = {
    WAIST_PATTERN.name: WAIST_PATTERN,
    HEAD_PATTERN.name: HEAD_PATTERN,
    CHEST_PATTERN.name: CHEST_PATTERN,
    LEFT_FOOT_PATTERN.name: LEFT_FOOT_PATTERN,
    RIGHT_FOOT_PATTERN.name: RIGHT_FOOT_PATTERN,
}


def _copy_triangulation_quality(payload: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    quality = payload or {}
    contributing = quality.get("contributing_rays", {})
    return {
        "accepted_points": int(quality.get("accepted_points", 0)),
        "contributing_rays": {
            "per_point": list(contributing.get("per_point", [])),
            "summary": dict(contributing.get("summary", {})),
        },
        "reprojection_error_px_summary": dict(
            quality.get("reprojection_error_px_summary", {})
        ),
        "epipolar_error_px_summary": dict(
            quality.get("epipolar_error_px_summary", {})
        ),
        "triangulation_angle_deg_summary": dict(
            quality.get("triangulation_angle_deg_summary", {})
        ),
        "assignment_diagnostics": dict(
            quality.get("assignment_diagnostics", {})
        ),
    }


def compute_frustum_near_corners_world(camera: CameraParams, z_near: float = 0.25) -> List[List[float]]:
    """Compute near-plane frustum corners in world coordinates."""
    width, height = camera.resolution
    corners_px = ((0.0, 0.0), (float(width), 0.0), (float(width), float(height)), (0.0, float(height)))

    center_world = -camera.rotation.T @ camera.translation
    rot_c2w = camera.rotation.T

    corners_world: List[List[float]] = []
    for u, v in corners_px:
        x = (u - camera.cx) / camera.fx * z_near
        y = (v - camera.cy) / camera.fy * z_near
        point_cam = np.array([x, y, z_near], dtype=np.float64)
        point_world = center_world + rot_c2w @ point_cam
        corners_world.append(point_world.tolist())

    return corners_world


def compute_markers_world(pose: RigidBodyPose, pattern: MarkerPattern) -> List[List[float]]:
    """Transform rigid body local markers into world coordinates."""
    transformed = (pose.rotation @ pattern.marker_positions.T).T + pose.position
    return transformed.tolist()


class TrackingRuntime:
    """Small runtime helper for GUI tracking status and scene snapshots."""

    def __init__(self, udp_port: int = 5000, trail_length: int = 120, z_near: float = 0.25):
        self.udp_port = udp_port
        self.trail_length = trail_length
        self.z_near = z_near

        self._pipeline: Optional[TrackingPipeline] = None
        self._patterns_by_name: Dict[str, MarkerPattern] = {}
        self._trail_by_name: Dict[str, Deque[List[float]]] = defaultdict(
            lambda: deque(maxlen=self.trail_length)
        )
        self._active_calibration_path: Optional[str] = None
        self._selected_pattern_names: List[str] = []
        self._last_stop_summary: Dict[str, Any] = {}
        self._latest_scene: Dict[str, Any] = self._empty_scene()
        self._scene_sequence = 0
        self._latest_scene_snapshot: Dict[str, Any] = self._build_scene_snapshot(self._latest_scene)
        self._camera_scene: List[Dict[str, Any]] = []
        self._status_cache: Dict[str, Any] = {}
        self._status_cache_monotonic = 0.0
        self._status_cache_interval_s = 0.25
        self._lock = threading.Lock()
        self._scene_condition = threading.Condition(self._lock)

    def start(self, calibration_path: str, patterns: Optional[List[str]] = None) -> Dict[str, Any]:
        """Start tracking runtime and attach pose callback."""
        self.stop()

        selected_names = patterns or [WAIST_PATTERN.name]
        missing = [name for name in selected_names if name not in AVAILABLE_PATTERNS]
        if missing:
            raise ValueError(f"unknown pattern names: {missing}")

        selected_patterns = [AVAILABLE_PATTERNS[name] for name in selected_names]

        pipeline = TrackingPipeline(
            udp_port=self.udp_port,
            calibration_path=calibration_path,
            patterns=selected_patterns,
        )
        pipeline.set_pose_callback(self._on_pose)
        pipeline.start(session_name="tracking_gui")
        camera_scene = self._build_camera_scene(pipeline)

        with self._lock:
            self._pipeline = pipeline
            self._active_calibration_path = calibration_path
            self._selected_pattern_names = [pattern.name for pattern in selected_patterns]
            self._patterns_by_name = {pattern.name: pattern for pattern in selected_patterns}
            self._trail_by_name = defaultdict(lambda: deque(maxlen=self.trail_length))
            self._last_stop_summary = {}
            self._camera_scene = camera_scene
            self._status_cache = {}
            self._status_cache_monotonic = 0.0
            self._set_latest_scene_locked(
                self._empty_scene(
                    cameras=camera_scene,
                    coordinate_frame=getattr(pipeline.geometry, "coordinate_frame", "camera_similarity"),
                    coordinate_origin=getattr(pipeline.geometry, "coordinate_origin", "reference_camera"),
                    coordinate_origin_source=getattr(
                        pipeline.geometry,
                        "coordinate_origin_source",
                        "extrinsics_pose_reference",
                    ),
                ),
            )
        status = self._refresh_status_cache(force=True)
        with self._lock:
            scene = dict(self._latest_scene)
            scene["tracking"] = self._scene_tracking_from_status(status)
            self._set_latest_scene_locked(scene)
        return status

    def stop(self) -> Dict[str, Any]:
        """Stop tracking runtime."""
        with self._lock:
            pipeline = self._pipeline
            self._pipeline = None

        if pipeline is None:
            return {}

        summary = pipeline.stop()
        with self._lock:
            self._patterns_by_name = {}
            self._trail_by_name = defaultdict(lambda: deque(maxlen=self.trail_length))
            self._last_stop_summary = dict(summary)
            self._status_cache = self._stopped_status_payload_locked()
            self._status_cache_monotonic = time.monotonic()
            self._set_latest_scene_locked(
                self._empty_scene(
                    cameras=[dict(camera) for camera in self._camera_scene],
                    tracking=self._scene_tracking_from_status(self._status_cache),
                    coordinate_frame=getattr(pipeline.geometry, "coordinate_frame", "camera_similarity"),
                    coordinate_origin=getattr(pipeline.geometry, "coordinate_origin", "reference_camera"),
                    coordinate_origin_source=getattr(
                        pipeline.geometry,
                        "coordinate_origin_source",
                        "extrinsics_pose_reference",
                    ),
                ),
            )
        return summary

    def status(self) -> Dict[str, Any]:
        """Return current runtime status."""
        with self._lock:
            pipeline = self._pipeline
            calibration_path = self._active_calibration_path
            pattern_names = list(self._selected_pattern_names)
            last_stop_summary = dict(self._last_stop_summary)
            cached_status = dict(self._status_cache)
            cached_at = self._status_cache_monotonic

        if pipeline is None:
            _ = (cached_status, cached_at)
            return self._with_scene_diagnostics(self._stopped_status_payload())

        _ = (calibration_path, pattern_names, last_stop_summary)
        return self._with_scene_diagnostics(self._refresh_status_cache(force=False))

    def scene_snapshot(self) -> Dict[str, Any]:
        """Return latest scene snapshot for GUI polling."""
        with self._lock:
            return self._scene_snapshot_locked()

    def wait_for_scene_update(self, last_sequence: Optional[int], timeout: float = 15.0) -> Dict[str, Any]:
        """Wait for a newer scene snapshot, or return the latest snapshot on timeout."""
        with self._scene_condition:
            if last_sequence is not None and self._scene_sequence <= last_sequence:
                self._scene_condition.wait_for(
                    lambda: self._scene_sequence > last_sequence,
                    timeout=timeout,
                )
            return self._scene_snapshot_locked()

    def _on_pose(self, poses: Dict[str, RigidBodyPose]) -> None:
        pipeline = None
        with self._lock:
            pipeline = self._pipeline

        if pipeline is None:
            return

        triangulation = pipeline.get_latest_triangulation_snapshot()
        status = self._refresh_status_cache(force=False)

        with self._lock:
            if self._pipeline is not pipeline:
                return

            camera_scene = self._camera_scene
            rigid_bodies = []
            for name, pose in poses.items():
                pattern = self._patterns_by_name.get(name)
                markers_world: List[List[float]] = []
                if pattern is not None and pose.valid:
                    markers_world = compute_markers_world(pose, pattern)

                trail = self._trail_by_name[name]
                if pose.valid:
                    trail.append(pose.position.tolist())

                rigid_bodies.append(
                    {
                        "name": name,
                        "valid": pose.valid,
                        "position": pose.position.tolist(),
                        "quaternion": pose.quaternion.tolist(),
                        "rms_error": pose.rms_error,
                        "observed_markers": pose.observed_markers,
                        "markers_world": markers_world,
                        "trail": list(trail),
                    }
                )

            self._set_latest_scene_locked(
                {
                    "tracking": self._scene_tracking_from_status(status, pipeline),
                    "cameras": [dict(camera) for camera in camera_scene],
                    "rigid_bodies": rigid_bodies,
                    "raw_points": list(triangulation.get("points_3d", [])),
                    "triangulation_quality": _copy_triangulation_quality(
                        triangulation.get("triangulation_quality", {})
                    ),
                    "coordinate_frame": getattr(pipeline.geometry, "coordinate_frame", "camera_similarity"),
                    "coordinate_origin": getattr(pipeline.geometry, "coordinate_origin", "reference_camera"),
                    "coordinate_origin_source": getattr(
                        pipeline.geometry,
                        "coordinate_origin_source",
                        "extrinsics_pose_reference",
                    ),
                    "timestamp_us": int(triangulation.get("timestamp", 0)),
                }
            )

    def _refresh_status_cache(self, *, force: bool) -> Dict[str, Any]:
        with self._lock:
            pipeline = self._pipeline
            if pipeline is None:
                return self._stopped_status_payload_locked()
            cached_status = self._status_cache
            cached_at = self._status_cache_monotonic
            calibration_path = self._active_calibration_path
            pattern_names = list(self._selected_pattern_names)
            last_stop_summary = dict(self._last_stop_summary)

        now = time.monotonic()
        if not force and cached_status and (now - cached_at) <= self._status_cache_interval_s:
            return self._copy_status_payload(cached_status)

        status = pipeline.get_status()
        status["calibration_path"] = calibration_path
        status["active_calibration_path"] = calibration_path
        status["patterns"] = pattern_names
        status["last_stop_summary"] = last_stop_summary
        with self._lock:
            if self._pipeline is pipeline:
                self._status_cache = self._copy_status_payload(status)
                self._status_cache_monotonic = now
                return self._copy_status_payload(self._status_cache)
        return self._stopped_status_payload()

    def _stopped_status_payload_locked(self) -> Dict[str, Any]:
        calibration_path = self._active_calibration_path
        pattern_names = list(self._selected_pattern_names)
        last_stop_summary = dict(self._last_stop_summary)
        return {
            "running": False,
            "calibration_loaded": calibration_path is not None,
            "active_calibration_path": calibration_path,
            "calibration_path": calibration_path,
            "patterns": pattern_names,
            "frames_processed": 0,
            "poses_estimated": 0,
            "receiver": {"frames_received": 0, "cameras_discovered": 0},
            "metrics": {},
            "tracking": {},
            "sync": {},
            "triangulation_quality": {},
            "uptime_seconds": 0.0,
            "last_stop_summary": last_stop_summary,
        }

    def _stopped_status_payload(self) -> Dict[str, Any]:
        with self._lock:
            return self._stopped_status_payload_locked()

    @staticmethod
    def _copy_status_payload(status: Dict[str, Any]) -> Dict[str, Any]:
        return {
            **status,
            "receiver": dict(status.get("receiver", {})),
            "metrics": dict(status.get("metrics", {})),
            "tracking": dict(status.get("tracking", {})),
            "sync": dict(status.get("sync", {})),
            "triangulation_quality": _copy_triangulation_quality(
                status.get("triangulation_quality", {})
            ),
            "patterns": list(status.get("patterns", [])),
            "last_stop_summary": dict(status.get("last_stop_summary", {})),
        }

    @staticmethod
    def _scene_tracking_from_status(
        status: Dict[str, Any],
        pipeline: Optional[TrackingPipeline] = None,
    ) -> Dict[str, Any]:
        pipeline_frames = int(getattr(pipeline, "frames_processed", 0)) if pipeline is not None else 0
        pipeline_poses = int(getattr(pipeline, "poses_estimated", 0)) if pipeline is not None else 0
        return {
            "running": bool(status.get("running", False) or getattr(pipeline, "is_running", False)),
            "frames_processed": max(int(status.get("frames_processed", 0)), pipeline_frames),
            "poses_estimated": max(int(status.get("poses_estimated", 0)), pipeline_poses),
        }

    def _set_latest_scene_locked(self, scene: Dict[str, Any]) -> None:
        next_timestamp = int(scene.get("timestamp_us", 0))
        current_timestamp = int(self._latest_scene.get("timestamp_us", -1))
        if next_timestamp > 0 and next_timestamp == current_timestamp:
            return
        self._scene_sequence += 1
        scene["host_scene_updated_realtime_us"] = int(time.time() * 1_000_000)
        scene["host_scene_updated_monotonic_ms"] = time.monotonic() * 1000.0
        scene["scene_update_count"] = self._scene_sequence
        scene["sequence"] = self._scene_sequence
        self._latest_scene = scene
        self._latest_scene_snapshot = self._build_scene_snapshot(scene)
        self._scene_condition.notify_all()

    def _scene_snapshot_locked(self) -> Dict[str, Any]:
        return dict(self._latest_scene_snapshot)

    def _build_scene_snapshot(self, scene: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "tracking": dict(scene["tracking"]),
            "cameras": list(scene["cameras"]),
            "rigid_bodies": list(scene["rigid_bodies"]),
            "raw_points": list(scene["raw_points"]),
            "triangulation_quality": _copy_triangulation_quality(
                scene.get("triangulation_quality", {})
            ),
            "coordinate_frame": scene.get("coordinate_frame", "camera_similarity"),
            "coordinate_origin": scene.get("coordinate_origin", "reference_camera"),
            "coordinate_origin_source": scene.get("coordinate_origin_source", "extrinsics_pose_reference"),
            "timestamp_us": scene["timestamp_us"],
            "sequence": int(scene.get("sequence", self._scene_sequence)),
            "host_scene_updated_realtime_us": int(scene.get("host_scene_updated_realtime_us", 0)),
            "host_scene_updated_monotonic_ms": float(scene.get("host_scene_updated_monotonic_ms", 0.0)),
            "scene_update_count": int(scene.get("scene_update_count", self._scene_sequence)),
        }

    def _scene_diagnostics_locked(self) -> Dict[str, Any]:
        snapshot = self._latest_scene_snapshot
        updated_realtime_us = int(snapshot.get("host_scene_updated_realtime_us", 0) or 0)
        age_ms = 0.0
        if updated_realtime_us > 0:
            age_ms = max(0.0, float(int(time.time() * 1_000_000) - updated_realtime_us) / 1_000.0)
        return {
            "scene_update_count": int(snapshot.get("scene_update_count", self._scene_sequence)),
            "host_scene_updated_realtime_us": updated_realtime_us,
            "host_scene_updated_monotonic_ms": float(snapshot.get("host_scene_updated_monotonic_ms", 0.0)),
            "scene_update_age_ms": age_ms,
        }

    def _with_scene_diagnostics(self, status: Dict[str, Any]) -> Dict[str, Any]:
        with self._lock:
            diagnostics = self._scene_diagnostics_locked()
        result = dict(status)
        result["scene"] = diagnostics
        return result

    def _build_camera_scene(self, pipeline: TrackingPipeline) -> List[Dict[str, Any]]:
        cameras: List[Dict[str, Any]] = []
        for camera_id, camera in pipeline.geometry.camera_params.items():
            camera_position = (-camera.rotation.T @ camera.translation).tolist()
            cameras.append(
                {
                    "camera_id": camera_id,
                    "position": camera_position,
                    "rotation_matrix": camera.rotation.tolist(),
                    "frustum_near_corners": compute_frustum_near_corners_world(
                        camera,
                        z_near=self.z_near,
                    ),
                }
            )
        return cameras

    def _empty_scene(
        self,
        cameras: Optional[List[Dict[str, Any]]] = None,
        tracking: Optional[Dict[str, Any]] = None,
        coordinate_frame: str = "camera_similarity",
        coordinate_origin: str = "reference_camera",
        coordinate_origin_source: str = "extrinsics_pose_reference",
    ) -> Dict[str, Any]:
        return {
            "tracking": tracking or {"running": False, "frames_processed": 0, "poses_estimated": 0},
            "cameras": cameras or [],
            "rigid_bodies": [],
            "raw_points": [],
            "triangulation_quality": {},
            "coordinate_frame": coordinate_frame,
            "coordinate_origin": coordinate_origin,
            "coordinate_origin_source": coordinate_origin_source,
            "timestamp_us": 0,
        }
