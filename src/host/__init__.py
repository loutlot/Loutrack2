"""
Host-side modules for loutrack2 motion capture system.

Modules:
- logger: Frame logging and recording
- replay: Log file playback
- metrics: Performance monitoring and metrics collection
- receiver: UDP frame reception and pairing
- geo: Geometry (triangulation, reprojection error)
- rigid: Rigid body estimation (Kabsch, DBSCAN, tracking)
- pipeline: Complete tracking pipeline
- visualize: Simple visualization utilities
"""

from .logger import FrameLogger, list_log_files
from .replay import FrameReplay, validate_log_integrity, FrameEntry, EventEntry
from .metrics import MetricsCollector, MetricsExporter
from .receiver import (
    Frame, PairedFrames, FrameBuffer, FramePairer,
    UDPReceiver, FrameProcessor
)
from .geo import (
    CameraParams, CalibrationLoader, Triangulator,
    GeometryPipeline, create_dummy_calibration
)
from .rigid import (
    MarkerPattern, RigidBodyPose, RigidBodyTracker,
    RigidBodyEstimator, KabschEstimator, PointClusterer,
    WAIST_PATTERN, HEAD_PATTERN
)
from .pipeline import TrackingPipeline, TrackingSession
from .visualize import TrackingVisualizer, SimpleGraph
from .sync_eval import SyncEvaluator

__all__ = [
    # Logger
    "FrameLogger",
    "list_log_files",
    # Replay
    "FrameReplay", 
    "validate_log_integrity",
    "FrameEntry",
    "EventEntry",
    # Metrics
    "MetricsCollector",
    "MetricsExporter",
    # Receiver
    "Frame",
    "PairedFrames",
    "FrameBuffer",
    "FramePairer",
    "UDPReceiver",
    "FrameProcessor",
    # Geometry
    "CameraParams",
    "CalibrationLoader",
    "Triangulator",
    "GeometryPipeline",
    "create_dummy_calibration",
    # Rigid body
    "MarkerPattern",
    "RigidBodyPose",
    "RigidBodyTracker",
    "RigidBodyEstimator",
    "KabschEstimator",
    "PointClusterer",
    "WAIST_PATTERN",
    "HEAD_PATTERN",
    # Pipeline
    "TrackingPipeline",
    "TrackingSession",
    "SyncEvaluator",
    # Visualize
    "TrackingVisualizer",
    "SimpleGraph",
]
