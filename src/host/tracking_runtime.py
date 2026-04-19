"""GUI-facing runtime wrapper around TrackingPipeline."""

from __future__ import annotations

import threading
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

        with self._lock:
            self._pipeline = pipeline
            self._active_calibration_path = calibration_path
            self._selected_pattern_names = [pattern.name for pattern in selected_patterns]
            self._patterns_by_name = {pattern.name: pattern for pattern in selected_patterns}
            self._trail_by_name = defaultdict(lambda: deque(maxlen=self.trail_length))
            self._last_stop_summary = {}
            self._set_latest_scene_locked(
                self._empty_scene(
                    cameras=self._build_camera_scene(pipeline),
                    coordinate_frame=getattr(pipeline.geometry, "coordinate_frame", "camera_similarity"),
                    coordinate_origin=getattr(pipeline.geometry, "coordinate_origin", "reference_camera"),
                    coordinate_origin_source=getattr(
                        pipeline.geometry,
                        "coordinate_origin_source",
                        "extrinsics_pose_reference",
                    ),
                ),
            )

        return self.status()

    def stop(self) -> Dict[str, Any]:
        """Stop tracking runtime."""
        with self._lock:
            pipeline = self._pipeline
            self._pipeline = None

        if pipeline is None:
            return {}

        summary = pipeline.stop()
        cameras = self._build_camera_scene(pipeline)
        with self._lock:
            self._patterns_by_name = {}
            self._trail_by_name = defaultdict(lambda: deque(maxlen=self.trail_length))
            self._last_stop_summary = dict(summary)
            self._set_latest_scene_locked(
                self._empty_scene(
                    cameras=cameras,
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

        if pipeline is None:
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
                "uptime_seconds": 0.0,
                "last_stop_summary": last_stop_summary,
            }

        status = pipeline.get_status()
        status["calibration_path"] = calibration_path
        status["active_calibration_path"] = calibration_path
        status["patterns"] = pattern_names
        status["last_stop_summary"] = last_stop_summary
        return status

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
        status = pipeline.get_status()

        with self._lock:
            if self._pipeline is not pipeline:
                return

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
                    "tracking": {
                        "running": status.get("running", False),
                        "frames_processed": status.get("frames_processed", 0),
                        "poses_estimated": status.get("poses_estimated", 0),
                    },
                    "cameras": self._build_camera_scene(pipeline),
                    "rigid_bodies": rigid_bodies,
                    "raw_points": list(triangulation.get("points_3d", [])),
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

    def _set_latest_scene_locked(self, scene: Dict[str, Any]) -> None:
        self._scene_sequence += 1
        scene["sequence"] = self._scene_sequence
        self._latest_scene = scene
        self._scene_condition.notify_all()

    def _scene_snapshot_locked(self) -> Dict[str, Any]:
        return {
            "tracking": dict(self._latest_scene["tracking"]),
            "cameras": [dict(camera) for camera in self._latest_scene["cameras"]],
            "rigid_bodies": [dict(body) for body in self._latest_scene["rigid_bodies"]],
            "raw_points": [list(point) for point in self._latest_scene["raw_points"]],
            "coordinate_frame": self._latest_scene.get("coordinate_frame", "camera_similarity"),
            "coordinate_origin": self._latest_scene.get("coordinate_origin", "reference_camera"),
            "coordinate_origin_source": self._latest_scene.get("coordinate_origin_source", "extrinsics_pose_reference"),
            "timestamp_us": self._latest_scene["timestamp_us"],
            "sequence": int(self._latest_scene.get("sequence", self._scene_sequence)),
        }

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
        coordinate_frame: str = "camera_similarity",
        coordinate_origin: str = "reference_camera",
        coordinate_origin_source: str = "extrinsics_pose_reference",
    ) -> Dict[str, Any]:
        return {
            "tracking": {"running": False, "frames_processed": 0, "poses_estimated": 0},
            "cameras": cameras or [],
            "rigid_bodies": [],
            "raw_points": [],
            "coordinate_frame": coordinate_frame,
            "coordinate_origin": coordinate_origin,
            "coordinate_origin_source": coordinate_origin_source,
            "timestamp_us": 0,
        }
