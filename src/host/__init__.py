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
    WAIST_PATTERN, HEAD_PATTERN, CHEST_PATTERN,
    LEFT_FOOT_PATTERN, RIGHT_FOOT_PATTERN
)
from .pipeline import TrackingPipeline, TrackingSession
from .visualize import TrackingVisualizer, SimpleGraph
from .sync_eval import SyncEvaluator
from .wand_session import (
    WAND_NAME,
    WAND_MARKER_DIAMETER_MM,
    WAND_OUTER_SHORT_MM,
    WAND_OUTER_LONG_MM,
    WAND_POINTS_MM,
    CameraTarget,
    SessionConfig,
    WandSession,
)

from .control import (
    send_request,
    ping,
    start,
    stop,
    set_exposure,
    set_gain,
    set_fps,
    set_focus,
    set_threshold,
    set_blob_diameter,
    set_circularity_min,
)

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
    "CHEST_PATTERN",
    "LEFT_FOOT_PATTERN",
    "RIGHT_FOOT_PATTERN",
    # Pipeline
    "TrackingPipeline",
    "TrackingSession",
    "SyncEvaluator",
    "send_request",
    "ping",
    "start",
    "stop",
    "set_exposure",
    "set_gain",
    "set_fps",
    "set_focus",
    "set_threshold",
    "set_blob_diameter",
    "set_circularity_min",
    "WAND_NAME",
    "WAND_MARKER_DIAMETER_MM",
    "WAND_OUTER_SHORT_MM",
    "WAND_OUTER_LONG_MM",
    "WAND_POINTS_MM",
    "CameraTarget",
    "SessionConfig",
    "WandSession",
    # Visualize
    "TrackingVisualizer",
    "SimpleGraph",
]
