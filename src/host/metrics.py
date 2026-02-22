"""
Metrics module for monitoring and visualizing system performance.

Provides functionality to:
- Track FPS (frames per second) per camera and overall
- Measure end-to-end latency
- Calculate reprojection error for triangulation quality
- Export metrics for visualization (JSON, Prometheus format)
"""

import time
from datetime import datetime
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
from collections import deque
import threading
import json


@dataclass
class FrameStats:
    """Statistics for a single frame."""
    camera_id: str
    timestamp: int  # Frame timestamp (us)
    received_at: float  # System time when received (seconds)
    blob_count: int
    frame_index: int


@dataclass
class CameraMetrics:
    """Per-camera metrics."""
    camera_id: str
    frame_count: int = 0
    last_frame_index: int = 0
    last_timestamp: int = 0
    fps: float = 0.0
    latency_ms: float = 0.0
    missing_frames: int = 0
    blob_count_avg: float = 0.0
    
    # Rolling windows for FPS calculation
    _timestamps: deque = field(default_factory=lambda: deque(maxlen=60))
    _blob_counts: deque = field(default_factory=lambda: deque(maxlen=60))


@dataclass 
class TriangulationMetrics:
    """Metrics for triangulation quality."""
    total_points: int = 0
    reprojection_errors: List[float] = field(default_factory=list)
    mean_error: float = 0.0
    max_error: float = 0.0
    rms_error: float = 0.0


class MetricsCollector:
    """
    Thread-safe metrics collector for the tracking system.
    
    Usage:
        metrics = MetricsCollector()
        
        # Record frame receipt
        metrics.record_frame("pi-01", 1234567890000000, 4, frame_index=42)
        
        # Record triangulation result
        metrics.record_triangulation(12, [0.5, 0.3, 0.8])
        
        # Get current metrics
        summary = metrics.get_summary()
    """
    
    def __init__(self, history_size: int = 60):
        """
        Initialize metrics collector.
        
        Args:
            history_size: Number of frames to keep for rolling averages
        """
        self.history_size = history_size
        self._cameras: Dict[str, CameraMetrics] = {}
        self._triangulation = TriangulationMetrics()
        self._lock = threading.Lock()
        self._start_time = time.time()
        self._global_frame_count = 0
        
        # Rolling window for global FPS
        self._frame_times: deque = deque(maxlen=history_size)
    
    def record_frame(
        self,
        camera_id: str,
        timestamp: int,
        blob_count: int,
        frame_index: int
    ) -> Dict[str, Any]:
        """
        Record a received frame and update metrics.
        
        Args:
            camera_id: Camera identifier
            timestamp: Frame timestamp in microseconds
            blob_count: Number of blobs detected in frame
            frame_index: Frame sequence number
            
        Returns:
            Updated metrics for this camera
        """
        with self._lock:
            received_at = time.time()
            
            # Get or create camera metrics
            if camera_id not in self._cameras:
                self._cameras[camera_id] = CameraMetrics(
                    camera_id=camera_id,
                    _timestamps=deque(maxlen=self.history_size),
                    _blob_counts=deque(maxlen=self.history_size)
                )
            
            cam = self._cameras[camera_id]
            
            # Check for missing frames (gap in frame_index)
            if cam.frame_count > 0 and frame_index > cam.last_frame_index + 1:
                gap = frame_index - cam.last_frame_index - 1
                cam.missing_frames += gap
            
            # Update frame counts
            cam.frame_count += 1
            cam.last_frame_index = frame_index
            cam.last_timestamp = timestamp
            
            # Calculate latency (if timestamp is PTP-synchronized)
            # Convert timestamp from us to seconds
            frame_time_seconds = timestamp / 1_000_000.0
            current_time_seconds = time.time()
            # Note: This assumes PTP sync; without PTP, this is relative
            cam.latency_ms = (current_time_seconds - frame_time_seconds) * 1000
            
            # Record timestamp for FPS calculation
            cam._timestamps.append(received_at)
            cam._blob_counts.append(blob_count)
            
            # Calculate FPS (frames in last window / time span)
            if len(cam._timestamps) >= 2:
                time_span = cam._timestamps[-1] - cam._timestamps[0]
                if time_span > 0:
                    cam.fps = (len(cam._timestamps) - 1) / time_span
            
            # Calculate average blob count
            if cam._blob_counts:
                cam.blob_count_avg = sum(cam._blob_counts) / len(cam._blob_counts)
            
            # Update global metrics
            self._global_frame_count += 1
            self._frame_times.append(received_at)
            
            return {
                "camera_id": camera_id,
                "fps": round(cam.fps, 2),
                "latency_ms": round(cam.latency_ms, 2),
                "blob_count": blob_count,
                "frame_index": frame_index
            }
    
    def record_triangulation(
        self,
        point_count: int,
        reprojection_errors: List[float]
    ) -> Dict[str, float]:
        """
        Record triangulation results.
        
        Args:
            point_count: Number of 3D points triangulated
            reprojection_errors: List of reprojection errors in pixels
            
        Returns:
            Triangulation metrics summary
        """
        with self._lock:
            self._triangulation.total_points += point_count
            self._triangulation.reprojection_errors.extend(reprojection_errors)
            
            # Keep only last 1000 errors for memory
            if len(self._triangulation.reprojection_errors) > 1000:
                self._triangulation.reprojection_errors = \
                    self._triangulation.reprojection_errors[-1000:]
            
            # Calculate error statistics
            if self._triangulation.reprojection_errors:
                errors = self._triangulation.reprojection_errors
                self._triangulation.mean_error = sum(errors) / len(errors)
                self._triangulation.max_error = max(errors)
                
                # RMS error
                squared_sum = sum(e * e for e in errors)
                self._triangulation.rms_error = (
                    squared_sum / len(errors)
                ) ** 0.5
            
            return {
                "total_points": self._triangulation.total_points,
                "mean_error": round(self._triangulation.mean_error, 4),
                "max_error": round(self._triangulation.max_error, 4),
                "rms_error": round(self._triangulation.rms_error, 4)
            }
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Get a complete metrics summary.
        
        Returns:
            Dictionary containing all metrics
        """
        with self._lock:
            # Calculate global FPS
            global_fps = 0.0
            if len(self._frame_times) >= 2:
                time_span = self._frame_times[-1] - self._frame_times[0]
                if time_span > 0:
                    global_fps = (len(self._frame_times) - 1) / time_span
            
            # Summarize per-camera metrics
            cameras = {}
            for cam_id, cam in self._cameras.items():
                cameras[cam_id] = {
                    "fps": round(cam.fps, 2),
                    "latency_ms": round(cam.latency_ms, 2),
                    "frame_count": cam.frame_count,
                    "missing_frames": cam.missing_frames,
                    "blob_count_avg": round(cam.blob_count_avg, 2),
                    "last_frame_index": cam.last_frame_index
                }
            
            return {
                "timestamp": datetime.now().isoformat(),
                "uptime_seconds": round(time.time() - self._start_time, 2),
                "global": {
                    "fps": round(global_fps, 2),
                    "total_frames": self._global_frame_count,
                    "camera_count": len(self._cameras)
                },
                "cameras": cameras,
                "triangulation": {
                    "total_points": self._triangulation.total_points,
                    "mean_error": round(self._triangulation.mean_error, 4),
                    "max_error": round(self._triangulation.max_error, 4),
                    "rms_error": round(self._triangulation.rms_error, 4)
                }
            }
    
    def get_camera_metrics(self, camera_id: str) -> Optional[Dict[str, Any]]:
        """Get metrics for a specific camera."""
        with self._lock:
            if camera_id not in self._cameras:
                return None
            
            cam = self._cameras[camera_id]
            return {
                "camera_id": camera_id,
                "fps": round(cam.fps, 2),
                "latency_ms": round(cam.latency_ms, 2),
                "frame_count": cam.frame_count,
                "missing_frames": cam.missing_frames,
                "blob_count_avg": round(cam.blob_count_avg, 2)
            }
    
    def export_prometheus(self) -> str:
        """
        Export metrics in Prometheus text format.
        
        Returns:
            Prometheus-formatted metrics string
        """
        summary = self.get_summary()
        
        lines = [
            "# HELP loutrack_frames_total Total frames received",
            "# TYPE loutrack_frames_total counter",
            f"loutrack_frames_total {summary['global']['total_frames']}",
            "",
            "# HELP loutrack_fps Current frames per second",
            "# TYPE loutrack_fps gauge",
            f"loutrack_fps {summary['global']['fps']}",
            "",
            "# HELP loutrack_camera_fps Per-camera FPS",
            "# TYPE loutrack_camera_fps gauge",
        ]
        
        for cam_id, cam_data in summary['cameras'].items():
            lines.append(f'loutrack_camera_fps{{camera="{cam_id}"}} {cam_data["fps"]}')
        
        lines.extend([
            "",
            "# HELP loutrack_camera_latency_ms Per-camera latency in milliseconds",
            "# TYPE loutrack_camera_latency_ms gauge",
        ])
        
        for cam_id, cam_data in summary['cameras'].items():
            lines.append(
                f'loutrack_camera_latency_ms{{camera="{cam_id}"}} {cam_data["latency_ms"]}'
            )
        
        lines.extend([
            "",
            "# HELP loutrack_reprojection_error_rms RMS reprojection error in pixels",
            "# TYPE loutrack_reprojection_error_rms gauge",
            f"loutrack_reprojection_error_rms {summary['triangulation']['rms_error']}",
        ])
        
        return "\n".join(lines)
    
    def reset(self) -> None:
        """Reset all metrics."""
        with self._lock:
            self._cameras.clear()
            self._triangulation = TriangulationMetrics()
            self._frame_times.clear()
            self._global_frame_count = 0
            self._start_time = time.time()


class MetricsExporter:
    """
    Export metrics to file or external systems.
    """
    
    @staticmethod
    def to_json(metrics: Dict[str, Any], filepath: str) -> None:
        """Write metrics to JSON file."""
        with open(filepath, 'w') as f:
            json.dump(metrics, f, indent=2)
    
    @staticmethod
    def to_jsonl(metrics: Dict[str, Any], filepath: str) -> None:
        """Append metrics as JSONL line."""
        with open(filepath, 'a') as f:
            f.write(json.dumps(metrics) + '\n')
    
    @staticmethod
    def to_prometheus_file(metrics: MetricsCollector, filepath: str) -> None:
        """Write Prometheus-format metrics to file."""
        content = metrics.export_prometheus()
        with open(filepath, 'w') as f:
            f.write(content)
