"""
Tracking pipeline that integrates all components.

Provides the complete processing chain:
- UDP reception → Frame pairing → Triangulation → Rigid body estimation → Output
"""

import time
import threading
import numpy as np
from typing import Optional, Dict, Any, List, Callable, Deque
from collections import deque
from datetime import datetime

from .receiver import FrameProcessor, PairedFrames
from .geo import (
    GeometryPipeline,
    create_dummy_calibration,
    normalize_epipolar_threshold_px,
)
from .rigid import (
    RigidBodyEstimator, RigidBodyPose, 
    MarkerPattern, WAIST_PATTERN, ReacquireGuardConfig
)
from .metrics import MetricsCollector
from .logger import FrameLogger
from .wand_session import FIXED_PAIR_WINDOW_US


def _stage_summary(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"last": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    sorted_values = sorted(float(v) for v in values)
    p95_index = min(len(sorted_values) - 1, int(round((len(sorted_values) - 1) * 0.95)))
    return {
        "last": float(values[-1]),
        "mean": sum(sorted_values) / len(sorted_values),
        "p95": sorted_values[p95_index],
        "max": sorted_values[-1],
    }


def _empty_triangulation_quality() -> Dict[str, Any]:
    empty_summary = {
        "count": 0,
        "mean": 0.0,
        "median": 0.0,
        "p90": 0.0,
        "p95": 0.0,
        "min": 0.0,
        "max": 0.0,
    }
    return {
        "accepted_points": 0,
        "contributing_rays": {"per_point": [], "summary": dict(empty_summary)},
        "reprojection_error_px_summary": dict(empty_summary),
        "epipolar_error_px_summary": dict(empty_summary),
        "triangulation_angle_deg_summary": dict(empty_summary),
        "assignment_diagnostics": {},
    }


def _copy_triangulation_quality(payload: Dict[str, Any]) -> Dict[str, Any]:
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


class TrackingPipeline:
    """
    Complete tracking pipeline from UDP frames to rigid body poses.
    
    Usage:
        pipeline = TrackingPipeline(config)
        pipeline.set_pose_callback(on_pose)
        pipeline.start()
        # ... running ...
        pipeline.stop()
    """
    
    def __init__(
        self,
        udp_port: int = 5000,
        calibration_path: Optional[str] = None,
        patterns: Optional[List[MarkerPattern]] = None,
        enable_logging: bool = True,
        log_dir: str = "./logs",
        timestamp_tolerance_us: int = FIXED_PAIR_WINDOW_US,
        epipolar_threshold_px: Optional[float] = None,
        reacquire_guard_config: Optional[ReacquireGuardConfig] = None,
        reacquire_guard_event_logging: bool = False,
    ):
        """
        Initialize tracking pipeline.
        
        Args:
            udp_port: UDP port for frame reception
            calibration_path: Path to calibration files
            patterns: List of rigid body patterns to track
            enable_logging: Whether to log frames
            log_dir: Directory for log files
        """
        self.udp_port = udp_port
        self.enable_logging = enable_logging
        self.log_dir = log_dir
        self.configured_pair_window_us = int(timestamp_tolerance_us)
        self._epipolar_threshold_px_override = (
            normalize_epipolar_threshold_px(epipolar_threshold_px)
            if epipolar_threshold_px is not None
            else None
        )
        self.reacquire_guard_event_logging = bool(reacquire_guard_event_logging)
        
        # Initialize components
        self.frame_processor = FrameProcessor(
            udp_port=udp_port,
            timestamp_tolerance_us=timestamp_tolerance_us,
            frame_index_fallback=False,
        )
        self.geometry = GeometryPipeline()
        self.rigid_estimator = RigidBodyEstimator(
            patterns=patterns or [WAIST_PATTERN],
            reacquire_guard_config=reacquire_guard_config or ReacquireGuardConfig(),
        )
        self.metrics = MetricsCollector()
        
        # Load calibration
        self._calibration_loaded = False
        if calibration_path:
            self._calibration_loaded = self.geometry.load_calibration(calibration_path) > 0
        self._apply_epipolar_threshold_override()
        
        # Logger
        self.logger: Optional[FrameLogger] = None
        if enable_logging:
            self.logger = FrameLogger(log_dir=log_dir)
        
        # Callbacks
        self._pose_callback: Optional[Callable[[Dict[str, RigidBodyPose]], None]] = None
        self._error_callback: Optional[Callable[[Exception], None]] = None
        
        # State
        self._running = False
        self._last_timestamp = 0
        self._triangulation_lock = threading.Lock()
        self._latest_triangulation_snapshot: Dict[str, Any] = {
            "timestamp": 0,
            "points_3d": [],
            "observations_by_camera": {},
            "triangulated_points": [],
            "reprojection_errors": [],
            "triangulation_quality": _empty_triangulation_quality(),
            "contributing_rays": {"per_point": [], "summary": {}},
            "reprojection_error_px_summary": {},
            "epipolar_error_px_summary": {},
            "triangulation_angle_deg_summary": {},
            "assignment_diagnostics": {},
            "pair_timestamp_range_us": 0,
        }
        
        # Statistics
        self.frames_processed = 0
        self.poses_estimated = 0
        self.start_time: Optional[float] = None
        self._stage_lock = threading.Lock()
        self._stage_ms: Dict[str, Deque[float]] = {
            "log_enqueue_ms": deque(maxlen=240),
            "triangulation_ms": deque(maxlen=240),
            "rigid_ms": deque(maxlen=240),
            "metrics_update_ms": deque(maxlen=240),
            "pose_callback_ms": deque(maxlen=240),
            "pipeline_pair_ms": deque(maxlen=240),
        }
        self._last_diagnostics_event_monotonic = 0.0
        self._diagnostics_event_interval_s = 1.0
        self._reacquire_guard_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
    
    def set_pose_callback(self, callback: Callable[[Dict[str, RigidBodyPose]], None]) -> None:
        """Set callback for estimated poses."""
        self._pose_callback = callback

    def set_patterns(self, patterns: List[MarkerPattern]) -> None:
        """Update tracked rigid-body patterns without restarting the pipeline."""
        next_patterns = list(patterns or [WAIST_PATTERN])
        self.rigid_estimator.set_patterns(next_patterns)
    
    def set_error_callback(self, callback: Callable[[Exception], None]) -> None:
        """Set callback for errors."""
        self._error_callback = callback
    
    def load_calibration(self, filepath: str) -> int:
        """
        Load camera calibration.
        
        Args:
            filepath: Path to calibration file or directory
            
        Returns:
            Number of cameras loaded
        """
        count = self.geometry.load_calibration(filepath)
        self._calibration_loaded = count > 0
        self._apply_epipolar_threshold_override()
        return count

    def _apply_epipolar_threshold_override(self) -> None:
        if self._epipolar_threshold_px_override is None:
            return
        self.geometry.epipolar_threshold_px = float(self._epipolar_threshold_px_override)
        if self.geometry.triangulator is not None:
            self.geometry.triangulator.epipolar_threshold_px = float(
                self._epipolar_threshold_px_override
            )
    
    def use_dummy_calibration(self, camera_ids: List[str]) -> None:
        """Use dummy calibration for testing."""
        params = create_dummy_calibration(camera_ids)
        self.geometry.camera_params = params
        from .geo import Triangulator
        self.geometry.triangulator = Triangulator(
            params,
            epipolar_threshold_px=self.geometry.epipolar_threshold_px,
        )
        self._calibration_loaded = True
    
    def start(self, session_name: Optional[str] = None) -> None:
        """Start the tracking pipeline."""
        if self._running:
            return
        
        self._running = True
        self.start_time = time.time()
        self.frame_processor.pairer.timestamp_tolerance_us = int(self.configured_pair_window_us)
        
        # Start logging
        if self.logger:
            self.logger.start_recording(session_name=session_name)
        
        # Set up frame callback
        self.frame_processor.set_paired_callback(self._on_paired_frames)
        
        # Start frame processor
        self.frame_processor.start()
    
    def stop(self) -> Dict[str, Any]:
        """Stop the tracking pipeline."""
        self._running = False
        
        # Stop frame processor
        self.frame_processor.stop()
        
        # Stop logging
        log_metadata = {}
        if self.logger:
            log_metadata = self.logger.stop_recording()
        
        return {
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "duration_seconds": time.time() - self.start_time if self.start_time else 0,
            "log_metadata": log_metadata
        }
    
    def _on_paired_frames(self, paired_frames: PairedFrames) -> None:
        """Process paired frames."""
        if not self._running:
            return
        
        pipeline_started_ns = time.perf_counter_ns()
        try:
            timestamp = paired_frames.timestamp
            
            # Log frames
            stage_started_ns = time.perf_counter_ns()
            if self.logger:
                for cam_id, frame in paired_frames.frames.items():
                    self.logger.log_frame(frame.to_dict())
            self._record_stage("log_enqueue_ms", self._elapsed_ms(stage_started_ns))
            
            # Triangulate
            result: Dict[str, Any] = {"reprojection_errors": []}
            stage_started_ns = time.perf_counter_ns()
            if self._calibration_loaded:
                result = self.geometry.process_paired_frames(
                    paired_frames,
                    min_inlier_views=2,
                )
                points_3d = result.get("points_3d", [])
            else:
                points_3d = []
            self._record_stage("triangulation_ms", self._elapsed_ms(stage_started_ns))

            points_3d_list = list(points_3d) if points_3d is not None else []
            point_count = len(points_3d_list)
            triangulation_quality = _copy_triangulation_quality(
                result.get("triangulation_quality", _empty_triangulation_quality())
            )

            with self._triangulation_lock:
                self._latest_triangulation_snapshot = {
                    "timestamp": timestamp,
                    "points_3d": [
                        point.tolist() if hasattr(point, "tolist") else list(point)
                        for point in points_3d_list
                    ],
                    "observations_by_camera": dict(result.get("observations_by_camera", {})),
                    "triangulated_points": list(result.get("triangulated_points", [])),
                    "reprojection_errors": list(result.get("reprojection_errors", [])),
                    "triangulation_quality": triangulation_quality,
                    "contributing_rays": dict(triangulation_quality.get("contributing_rays", {})),
                    "reprojection_error_px_summary": dict(
                        triangulation_quality.get("reprojection_error_px_summary", {})
                    ),
                    "epipolar_error_px_summary": dict(
                        triangulation_quality.get("epipolar_error_px_summary", {})
                    ),
                    "triangulation_angle_deg_summary": dict(
                        triangulation_quality.get("triangulation_angle_deg_summary", {})
                    ),
                    "assignment_diagnostics": dict(result.get("assignment_diagnostics", {})),
                    "pair_timestamp_range_us": paired_frames.timestamp_range_us,
                }
            
            # Estimate rigid body poses
            points_array = (
                np.asarray(points_3d_list, dtype=np.float64)
                if point_count > 0
                else np.empty((0, 3), dtype=np.float64)
            )
            
            stage_started_ns = time.perf_counter_ns()
            if hasattr(self.rigid_estimator, "process_context"):
                poses = self.rigid_estimator.process_context(
                    points_array,
                    timestamp,
                    camera_params=self.geometry.camera_params,
                    observations_by_camera=result.get("observations_by_camera", {}),
                    coordinate_space="raw_pixel",
                )
            else:
                poses = self.rigid_estimator.process_points(points_array, timestamp)
            self._record_stage("rigid_ms", self._elapsed_ms(stage_started_ns))
            self._record_reacquire_guard_events(timestamp)
            
            # Update metrics
            stage_started_ns = time.perf_counter_ns()
            for cam_id, frame in paired_frames.frames.items():
                self.metrics.record_frame(
                    camera_id=cam_id,
                    timestamp=frame.timestamp,
                    blob_count=len(frame.blobs),
                    frame_index=frame.frame_index,
                    received_at=frame.received_at,
                    timestamp_source=frame.timestamp_source,
                    capture_to_process_ms=frame.capture_to_process_ms,
                    capture_to_send_ms=frame.capture_to_send_ms,
                )
            
            if point_count > 0:
                self.metrics.record_triangulation(
                    point_count,
                    result.get("reprojection_errors", [])
                )
            self._record_stage("metrics_update_ms", self._elapsed_ms(stage_started_ns))
            
            # Callback
            self.frames_processed += 1
            valid_pose_count = sum(1 for pose in poses.values() if pose.valid)
            self.poses_estimated += valid_pose_count

            stage_started_ns = time.perf_counter_ns()
            if self._pose_callback:
                self._pose_callback(poses)
            self._record_stage("pose_callback_ms", self._elapsed_ms(stage_started_ns))
            self._record_stage("pipeline_pair_ms", self._elapsed_ms(pipeline_started_ns))
            self._maybe_log_diagnostics_event()
            
        except Exception as e:
            if self._error_callback:
                self._error_callback(e)

    @staticmethod
    def _elapsed_ms(started_ns: int) -> float:
        return float(time.perf_counter_ns() - started_ns) / 1_000_000.0

    def _record_stage(self, name: str, value_ms: float) -> None:
        with self._stage_lock:
            bucket = self._stage_ms.setdefault(name, deque(maxlen=240))
            bucket.append(float(value_ms))

    def _stage_diagnostics(self) -> Dict[str, Dict[str, float]]:
        with self._stage_lock:
            return {
                name: _stage_summary(list(values))
                for name, values in self._stage_ms.items()
            }

    def _logger_diagnostics(self) -> Dict[str, Any]:
        if self.logger is None:
            return {"recording": False}
        get_stats = getattr(self.logger, "get_stats", None)
        if callable(get_stats):
            return get_stats()
        return {"recording": bool(getattr(self.logger, "is_recording", False))}

    def _geometry_diagnostics(self) -> Dict[str, Any]:
        get_diagnostics = getattr(self.geometry, "get_diagnostics", None)
        if callable(get_diagnostics):
            return get_diagnostics()
        return {}

    def _diagnostics_snapshot(self) -> Dict[str, Any]:
        return {
            "receiver": self.frame_processor.get_stats(),
            "geometry": self._geometry_diagnostics(),
            "tracking": self.rigid_estimator.get_tracking_status(),
            "reacquire_guard_events": self.get_reacquire_guard_events(limit=20),
            "pipeline_stage_ms": self._stage_diagnostics(),
            "logger": self._logger_diagnostics(),
            "metrics": self.metrics.get_summary(),
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
        }

    def _record_reacquire_guard_events(self, timestamp: int) -> None:
        tracking = self.rigid_estimator.get_tracking_status()
        for rigid_name, status in tracking.items():
            if not isinstance(status, dict):
                continue
            guard = status.get("reacquire_guard")
            if not isinstance(guard, dict):
                continue
            if not (guard.get("evaluated") or guard.get("would_reject")):
                continue
            event = {
                "timestamp": int(timestamp),
                "rigid_name": str(rigid_name),
                "mode": str(status.get("mode", "")),
                "valid": bool(status.get("valid", False)),
                "would_reject": bool(guard.get("would_reject", False)),
                "passed": bool(guard.get("passed", True)),
                "reason": str(guard.get("reason", "")),
                "reprojection_score": dict(guard.get("score", {})),
                "position_innovation_m": float(guard.get("position_innovation_m", 0.0)),
                "rotation_innovation_deg": float(guard.get("rotation_innovation_deg", 0.0)),
                "enforced": bool(guard.get("enforced", False)),
                "rejected_count": int(guard.get("rejected_count", 0)),
            }
            self._reacquire_guard_events.append(event)
            if (
                self.reacquire_guard_event_logging
                and self.logger is not None
                and self.logger.is_recording
            ):
                self.logger.log_event("reacquire_guard", event)

    def get_reacquire_guard_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._reacquire_guard_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def _maybe_log_diagnostics_event(self) -> None:
        if self.logger is None or not self.logger.is_recording:
            return
        now = time.monotonic()
        if now - self._last_diagnostics_event_monotonic < self._diagnostics_event_interval_s:
            return
        self._last_diagnostics_event_monotonic = now
        self.logger.log_event("tracking_diagnostics", self._diagnostics_snapshot())
    
    def get_status(self) -> Dict[str, Any]:
        """Get current pipeline status."""
        triangulation = self.get_latest_triangulation_snapshot()
        return {
            "running": self._running,
            "calibration_loaded": self._calibration_loaded,
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "uptime_seconds": time.time() - self.start_time if self.start_time else 0,
            "receiver": self.frame_processor.get_stats(),
            "metrics": self.metrics.get_summary(),
            "tracking": self.rigid_estimator.get_tracking_status(),
            "triangulation_quality": _copy_triangulation_quality(
                triangulation.get("triangulation_quality", _empty_triangulation_quality())
            ),
            "diagnostics": {
                "geometry": self._geometry_diagnostics(),
                "tracking": self.rigid_estimator.get_tracking_status(),
                "reacquire_guard_events": self.get_reacquire_guard_events(limit=20),
                "pipeline_stage_ms": self._stage_diagnostics(),
                "logger": self._logger_diagnostics(),
            },
        }
    
    @property
    def is_running(self) -> bool:
        return self._running

    def get_latest_triangulation_snapshot(self) -> Dict[str, Any]:
        """Get latest triangulation output for GUI/runtime consumers."""
        with self._triangulation_lock:
            return {
                "timestamp": self._latest_triangulation_snapshot["timestamp"],
                "points_3d": [list(point) for point in self._latest_triangulation_snapshot["points_3d"]],
                "observations_by_camera": {
                    camera_id: [dict(observation) for observation in observations]
                    for camera_id, observations in self._latest_triangulation_snapshot.get(
                        "observations_by_camera",
                        {},
                    ).items()
                },
                "triangulated_points": [
                    dict(point)
                    for point in self._latest_triangulation_snapshot.get(
                        "triangulated_points",
                        [],
                    )
                ],
                "reprojection_errors": list(self._latest_triangulation_snapshot["reprojection_errors"]),
                "triangulation_quality": _copy_triangulation_quality(
                    self._latest_triangulation_snapshot.get(
                        "triangulation_quality",
                        _empty_triangulation_quality(),
                    )
                ),
                "contributing_rays": {
                    "per_point": list(
                        self._latest_triangulation_snapshot.get(
                            "triangulation_quality",
                            {},
                        ).get("contributing_rays", {}).get("per_point", [])
                    ),
                    "summary": dict(
                        self._latest_triangulation_snapshot.get(
                            "triangulation_quality",
                            {},
                        ).get("contributing_rays", {}).get("summary", {})
                    ),
                },
                "reprojection_error_px_summary": dict(
                    self._latest_triangulation_snapshot.get(
                        "reprojection_error_px_summary",
                        {},
                    )
                ),
                "epipolar_error_px_summary": dict(
                    self._latest_triangulation_snapshot.get(
                        "epipolar_error_px_summary",
                        {},
                    )
                ),
                "triangulation_angle_deg_summary": dict(
                    self._latest_triangulation_snapshot.get(
                        "triangulation_angle_deg_summary",
                        {},
                    )
                ),
                "assignment_diagnostics": dict(self._latest_triangulation_snapshot.get("assignment_diagnostics", {})),
                "pair_timestamp_range_us": self._latest_triangulation_snapshot["pair_timestamp_range_us"],
            }


class TrackingSession:
    """
    Legacy high-level session wrapper around TrackingPipeline.

    The host GUI now uses TrackingRuntime as the primary orchestration layer.
    TrackingSession remains available for older call sites, tests, and scripts
    that still want a history-owning session facade.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize tracking session.
        
        Args:
            config: Configuration dictionary
        """
        self.config = config or {}
        
        self.pipeline = TrackingPipeline(
            udp_port=self.config.get("udp_port", 5000),
            calibration_path=self.config.get("calibration_path"),
            patterns=self.config.get("patterns"),
            enable_logging=self.config.get("enable_logging", True),
            log_dir=self.config.get("log_dir", "./logs")
        )
        
        self._max_history: int = self.config.get("max_history", 3600)  # ~1 min at 60fps
        self._pose_history: Deque[Dict[str, Any]] = deque(maxlen=self._max_history)
        self._history_lock = threading.Lock()
    
    def start(self, session_name: Optional[str] = None) -> None:
        """Start tracking session."""
        def on_pose(poses: Dict[str, RigidBodyPose]):
            with self._history_lock:
                # deque(maxlen=) auto-discards oldest entries — no manual trimming needed
                self._pose_history.append({
                    "timestamp": max(p.timestamp for p in poses.values()),
                    "poses": {name: pose.to_dict() for name, pose in poses.items()}
                })
        
        self.pipeline.set_pose_callback(on_pose)
        self.pipeline.start(session_name=session_name)
    
    def stop(self) -> Dict[str, Any]:
        """Stop tracking session."""
        result = self.pipeline.stop()
        result["pose_history_count"] = len(self._pose_history)
        return result
    
    def get_pose_history(
        self,
        body_name: Optional[str] = None,
        since_timestamp: Optional[int] = None
    ) -> List[Dict[str, Any]]:
        """
        Get pose history.
        
        Args:
            body_name: Filter by body name (None = all)
            since_timestamp: Get poses after this timestamp (None = all)
            
        Returns:
            List of pose entries
        """
        with self._history_lock:
            history = list(self._pose_history)
        
        if since_timestamp is not None:
            history = [h for h in history if h["timestamp"] > since_timestamp]
        
        if body_name is not None:
            history = [
                {"timestamp": h["timestamp"], "pose": h["poses"].get(body_name)}
                for h in history
                if body_name in h["poses"]
            ]
        
        return history
    
    def get_latest_poses(self) -> Dict[str, Dict[str, Any]]:
        """Get most recent poses for all tracked bodies."""
        with self._history_lock:
            if not self._pose_history:
                return {}
            return self._pose_history[-1]["poses"]
    
    def get_status(self) -> Dict[str, Any]:
        """Get session status."""
        status = self.pipeline.get_status()
        status["history_count"] = len(self._pose_history)
        return status
    
    def export_poses(self, filepath: str, format: str = "json") -> None:
        """
        Export pose history to file.
        
        Args:
            filepath: Output file path
            format: Output format ("json", "csv")
        """
        import json
        
        with self._history_lock:
            history = list(self._pose_history)
        
        if format == "json":
            with open(filepath, 'w') as f:
                json.dump(history, f, indent=2)
        
        elif format == "csv":
            import csv
            with open(filepath, 'w', newline='') as f:
                writer = csv.writer(f)
                # Header
                writer.writerow([
                    "timestamp", "body_name", 
                    "pos_x", "pos_y", "pos_z",
                    "quat_w", "quat_x", "quat_y", "quat_z",
                    "rms_error", "valid"
                ])
                # Data
                for entry in history:
                    ts = entry["timestamp"]
                    for name, pose in entry["poses"].items():
                        writer.writerow([
                            ts, name,
                            pose["position"][0], pose["position"][1], pose["position"][2],
                            pose["quaternion"][0], pose["quaternion"][1],
                            pose["quaternion"][2], pose["quaternion"][3],
                            pose["rms_error"], pose["valid"]
                        ])
