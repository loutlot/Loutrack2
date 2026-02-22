"""
Tracking pipeline that integrates all components.

Provides the complete processing chain:
- UDP reception → Frame pairing → Triangulation → Rigid body estimation → Output
"""

import time
import threading
from typing import Optional, Dict, Any, List, Callable
from datetime import datetime

from .receiver import FrameProcessor, PairedFrames
from .geo import GeometryPipeline, create_dummy_calibration
from .rigid import (
    RigidBodyEstimator, RigidBodyPose, 
    MarkerPattern, WAIST_PATTERN
)
from .metrics import MetricsCollector
from .logger import FrameLogger
from .sync_eval import SyncEvaluator


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
        log_dir: str = "./logs"
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
        
        # Initialize components
        self.frame_processor = FrameProcessor(udp_port=udp_port)
        self.geometry = GeometryPipeline()
        self.rigid_estimator = RigidBodyEstimator(patterns=patterns or [WAIST_PATTERN])
        self.metrics = MetricsCollector()
        self.sync_evaluator = SyncEvaluator(
            tolerance_windows_us=(1000, 2000, 5000, 10000),
            target_range_us=(2000, 5000),
            coverage_target=0.95,
        )
        
        # Load calibration
        self._calibration_loaded = False
        if calibration_path:
            self._calibration_loaded = self.geometry.load_calibration(calibration_path) > 0
        
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
        
        # Statistics
        self.frames_processed = 0
        self.poses_estimated = 0
        self.start_time: Optional[float] = None
    
    def set_pose_callback(self, callback: Callable[[Dict[str, RigidBodyPose]], None]) -> None:
        """Set callback for estimated poses."""
        self._pose_callback = callback
    
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
        return count
    
    def use_dummy_calibration(self, camera_ids: List[str]) -> None:
        """Use dummy calibration for testing."""
        params = create_dummy_calibration(camera_ids)
        self.geometry.camera_params = params
        from .geo import Triangulator
        self.geometry.triangulator = Triangulator(params)
        self._calibration_loaded = True
    
    def start(self, session_name: Optional[str] = None) -> None:
        """Start the tracking pipeline."""
        if self._running:
            return
        
        self._running = True
        self.start_time = time.time()
        
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
        
        try:
            timestamp = paired_frames.timestamp
            self.sync_evaluator.evaluate_pair(paired_frames)
            
            # Log frames
            if self.logger:
                for cam_id, frame in paired_frames.frames.items():
                    self.logger.log_frame(frame.to_dict())
            
            # Triangulate
            result: Dict[str, Any] = {"reprojection_errors": []}
            if self._calibration_loaded:
                result = self.geometry.process_paired_frames(paired_frames)
                points_3d = result.get("points_3d", [])
            else:
                points_3d = []
            
            # Estimate rigid body poses
            points_array = points_3d if points_3d else []
            
            poses = self.rigid_estimator.process_points(points_array, timestamp)
            
            # Update metrics
            for cam_id, frame in paired_frames.frames.items():
                self.metrics.record_frame(
                    camera_id=cam_id,
                    timestamp=frame.timestamp,
                    blob_count=len(frame.blobs),
                    frame_index=frame.frame_index
                )
            
            if points_3d:
                self.metrics.record_triangulation(
                    len(points_3d),
                    result.get("reprojection_errors", [])
                )
            
            # Callback
            if self._pose_callback and poses:
                self._pose_callback(poses)
            
            self.frames_processed += 1
            self.poses_estimated += len([p for p in poses.values() if p.valid])
            
        except Exception as e:
            if self._error_callback:
                self._error_callback(e)
    
    def get_status(self) -> Dict[str, Any]:
        """Get current pipeline status."""
        return {
            "running": self._running,
            "calibration_loaded": self._calibration_loaded,
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "uptime_seconds": time.time() - self.start_time if self.start_time else 0,
            "receiver": self.frame_processor.get_stats(),
            "metrics": self.metrics.get_summary(),
            "tracking": self.rigid_estimator.get_tracking_status(),
            "sync": self.sync_evaluator.get_status(),
        }
    
    @property
    def is_running(self) -> bool:
        return self._running


class TrackingSession:
    """
    High-level session manager for tracking.
    
    Handles session lifecycle, output files, and statistics.
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
        
        self._pose_history: List[Dict[str, Any]] = []
        self._history_lock = threading.Lock()
        self._max_history = self.config.get("max_history", 3600)  # ~1 min at 60fps
    
    def start(self, session_name: Optional[str] = None) -> None:
        """Start tracking session."""
        def on_pose(poses: Dict[str, RigidBodyPose]):
            with self._history_lock:
                self._pose_history.append({
                    "timestamp": max(p.timestamp for p in poses.values()),
                    "poses": {name: pose.to_dict() for name, pose in poses.items()}
                })
                # Trim history
                if len(self._pose_history) > self._max_history:
                    self._pose_history = self._pose_history[-self._max_history:]
        
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
            history = self._pose_history.copy()
        
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
            history = self._pose_history.copy()
        
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
