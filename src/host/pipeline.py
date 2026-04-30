"""
Tracking pipeline that integrates all components.

Provides the complete processing chain:
- UDP reception → Frame pairing → Triangulation → Rigid body estimation → Output
"""

import time
import threading
import numpy as np
from typing import Optional, Dict, Any, List, Callable, Deque
from collections import Counter, deque
from datetime import datetime

from .receiver import FrameProcessor, PairedFrames
from .geo import (
    GeometryPipeline,
    create_dummy_calibration,
    normalize_epipolar_threshold_px,
)
from .rigid import (
    RigidBodyEstimator, RigidBodyPose, 
    MarkerPattern, WAIST_PATTERN, ReacquireGuardConfig, ObjectGatingConfig,
    PoseContinuityGuardConfig, PositionContinuityGuardConfig, SubsetSolveConfig
)
from .metrics import MetricsCollector
from .logger import FrameLogger
from .wand_session import FIXED_PAIR_WINDOW_US


PIPELINE_VARIANTS = (
    "baseline",
    "fast_A",
    "fast_AB",
    "fast_ABC",
    "fast_ABCD",
    "fast_ABCDP",
    "fast_ABCDS",
    "fast_ABCDG",
    "fast_ABCDF",
    "fast_ABCDH",
    "fast_ABCDHF",
    "fast_ABCDHR",
    "fast_ABCDHRF",
    "fast_ABCDR",
    "fast_ABCDX",
    "fast_ABCDE",
)
DEFAULT_PIPELINE_VARIANT = "fast_ABCD"
_PIPELINE_VARIANT_RANK = {name: index for index, name in enumerate(PIPELINE_VARIANTS)}
_STAGE_DETAIL_NAMES = {
    "undistort_ms",
    "epipolar_match_ms",
    "generic_triangulation_ms",
    "object_gating_ms",
    "rigid_hint_triangulation_ms",
    "rigid_pose_ms",
    "subset_ms",
    "fallback_ms",
}


def normalize_pipeline_variant(value: Optional[str]) -> str:
    variant = str(value or DEFAULT_PIPELINE_VARIANT)
    if variant not in _PIPELINE_VARIANT_RANK:
        raise ValueError(f"unknown pipeline_variant: {variant}")
    return variant


def _variant_at_least(variant: str, minimum: str) -> bool:
    return _PIPELINE_VARIANT_RANK[normalize_pipeline_variant(variant)] >= _PIPELINE_VARIANT_RANK[minimum]


def default_subset_diagnostics_mode_for_variant(variant: str) -> str:
    return "sampled" if _variant_at_least(variant, "fast_ABC") else "full"


def _variant_has_subset_budget(variant: str) -> bool:
    return normalize_pipeline_variant(variant) in {
        "fast_ABCDS",
        "fast_ABCDG",
        "fast_ABCDF",
        "fast_ABCDH",
        "fast_ABCDHF",
        "fast_ABCDHR",
        "fast_ABCDHRF",
        "fast_ABCDR",
        "fast_ABCDX",
    }


def _variant_has_relaxed_object_gating(variant: str) -> bool:
    return normalize_pipeline_variant(variant) in {
        "fast_ABCDG",
        "fast_ABCDF",
        "fast_ABCDH",
        "fast_ABCDHF",
        "fast_ABCDHR",
        "fast_ABCDHRF",
        "fast_ABCDR",
        "fast_ABCDX",
    }


def _variant_has_rigid_candidate_separation(variant: str) -> bool:
    return normalize_pipeline_variant(variant) in {
        "fast_ABCDHR",
        "fast_ABCDHRF",
        "fast_ABCDR",
        "fast_ABCDX",
    }


def _variant_uses_hint_only_geometry(variant: str) -> bool:
    return normalize_pipeline_variant(variant) in {
        "fast_ABCDH",
        "fast_ABCDHF",
        "fast_ABCDHR",
        "fast_ABCDHRF",
    }


def _variant_has_geo_hotpath_optimizations(variant: str) -> bool:
    return normalize_pipeline_variant(variant) in {
        "fast_ABCDF",
        "fast_ABCDHF",
        "fast_ABCDHRF",
    }


def subset_time_budget_ms_for_variant(variant: str) -> Optional[float]:
    return 6.0 if _variant_has_subset_budget(variant) else None


def subset_max_hypotheses_for_variant(variant: str) -> Optional[int]:
    return 512 if _variant_has_subset_budget(variant) else None


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


def _rigid_stabilization_configs(settings: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    payload = settings if isinstance(settings, dict) else {}
    return {
        "reacquire_guard_config": ReacquireGuardConfig(
            shadow_enabled=bool(payload.get("reacquire_guard_shadow_enabled", True)),
            enforced=bool(payload.get("reacquire_guard_enforced", False)),
            post_reacquire_continue_frames=int(
                payload.get("reacquire_guard_post_reacquire_frames", 0)
            ),
            max_rotation_innovation_deg=float(
                payload.get("reacquire_guard_max_rotation_deg", 136.0)
            ),
        ),
        "object_gating_config": ObjectGatingConfig(
            enabled=bool(payload.get("object_conditioned_gating", True)),
            enforce=bool(payload.get("object_gating_enforced", False)),
            activation_mode=str(payload.get("object_gating_activation_mode", "always")),
        ),
        "pose_continuity_guard_config": PoseContinuityGuardConfig(
            enabled=bool(payload.get("pose_continuity_guard_enabled", False)),
            enforced=bool(payload.get("pose_continuity_guard_enforced", False)),
            max_position_innovation_m=float(
                payload.get("pose_continuity_max_position_m", 0.08)
            ),
            max_rotation_innovation_deg=float(
                payload.get("pose_continuity_max_rotation_deg", 90.0)
            ),
            max_angular_velocity_deg_s=float(
                payload.get("pose_continuity_max_angular_velocity_deg_s", 2500.0)
            ),
            max_angular_accel_deg_s2=float(
                payload.get("pose_continuity_max_angular_accel_deg_s2", 200000.0)
            ),
        ),
        "position_continuity_guard_config": PositionContinuityGuardConfig(
            enabled=bool(payload.get("position_continuity_guard_enabled", False)),
            enforced=bool(payload.get("position_continuity_guard_enforced", False)),
            max_accel_m_s2=float(
                payload.get("position_continuity_max_accel_m_s2", 60.0)
            ),
            max_velocity_m_s=float(
                payload.get("position_continuity_max_velocity_m_s", 8.0)
            ),
        ),
        "subset_solve_config": SubsetSolveConfig(
            enabled=bool(payload.get("subset_ransac", True)),
            diagnostics_only=True,
        ),
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
        object_gating_config: Optional[ObjectGatingConfig] = None,
        pose_continuity_guard_config: Optional[PoseContinuityGuardConfig] = None,
        position_continuity_guard_config: Optional[PositionContinuityGuardConfig] = None,
        subset_solve_config: Optional[SubsetSolveConfig] = None,
        rigid_stabilization: Optional[Dict[str, Any]] = None,
        pipeline_variant: str = DEFAULT_PIPELINE_VARIANT,
        subset_diagnostics_mode: Optional[str] = None,
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
        self.pipeline_variant = normalize_pipeline_variant(pipeline_variant)
        self.subset_diagnostics_mode = str(
            subset_diagnostics_mode
            or default_subset_diagnostics_mode_for_variant(self.pipeline_variant)
        )
        stabilization_configs = _rigid_stabilization_configs(rigid_stabilization)
        if rigid_stabilization and "reacquire_guard_event_logging" in rigid_stabilization:
            self.reacquire_guard_event_logging = bool(
                rigid_stabilization.get("reacquire_guard_event_logging")
            )
        
        # Initialize components
        self.frame_processor = FrameProcessor(
            udp_port=udp_port,
            timestamp_tolerance_us=timestamp_tolerance_us,
            frame_index_fallback=False,
        )
        self.geometry = GeometryPipeline(
            pipeline_variant=self.pipeline_variant,
            stage_callback=self._record_stage,
        )
        self._raw_scene_geometry = GeometryPipeline(pipeline_variant="baseline")
        self._raw_scene_condition = threading.Condition()
        self._raw_scene_worker: Optional[threading.Thread] = None
        self._raw_scene_worker_stop = False
        self._raw_scene_pending_pair: Optional[PairedFrames] = None
        self._raw_scene_latest_points: List[List[float]] = []
        self._raw_scene_latest_timestamp = 0
        self._raw_scene_latest_quality: Dict[str, Any] = _empty_triangulation_quality()
        self.rigid_estimator = RigidBodyEstimator(
            patterns=patterns or [WAIST_PATTERN],
            reacquire_guard_config=reacquire_guard_config
            or stabilization_configs["reacquire_guard_config"],
            object_gating_config=object_gating_config
            or stabilization_configs["object_gating_config"],
            pose_continuity_guard_config=pose_continuity_guard_config
            or stabilization_configs["pose_continuity_guard_config"],
            position_continuity_guard_config=position_continuity_guard_config
            or stabilization_configs["position_continuity_guard_config"],
            subset_solve_config=subset_solve_config
            or stabilization_configs["subset_solve_config"],
            subset_diagnostics_mode=self.subset_diagnostics_mode,
            subset_time_budget_ms=subset_time_budget_ms_for_variant(self.pipeline_variant),
            subset_max_hypotheses=subset_max_hypotheses_for_variant(self.pipeline_variant),
            rigid_candidate_separation_enabled=_variant_has_rigid_candidate_separation(
                self.pipeline_variant
            ),
            stage_callback=self._record_stage,
        )
        self.metrics = MetricsCollector()
        
        # Load calibration
        self._calibration_loaded = False
        if calibration_path:
            self._calibration_loaded = self.load_calibration(calibration_path) > 0
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
            "raw_scene_points_3d": [],
            "raw_scene_timestamp": 0,
            "raw_scene_triangulation_quality": _empty_triangulation_quality(),
            "rigid_hint_points_3d": [],
            "observations_by_camera": {},
            "triangulated_points": [],
            "rigid_hint_triangulated_points": [],
            "reprojection_errors": [],
            "rigid_hint_reprojection_errors": [],
            "triangulation_quality": _empty_triangulation_quality(),
            "rigid_hint_quality": {},
            "contributing_rays": {"per_point": [], "summary": {}},
            "reprojection_error_px_summary": {},
            "epipolar_error_px_summary": {},
            "triangulation_angle_deg_summary": {},
            "assignment_diagnostics": {},
            "object_gating": {},
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
        for detail_name in _STAGE_DETAIL_NAMES:
            self._stage_ms.setdefault(detail_name, deque(maxlen=240))
        self._variant_metric_lock = threading.Lock()
        self._variant_counts: Counter[str] = Counter()
        self._fallback_reason_counts: Counter[str] = Counter()
        self._object_gating_filter_reason_counts: Counter[str] = Counter()
        self._active_pair_stage_ms: Optional[Dict[str, float]] = None
        self._active_pair_decision = ""
        self._slow_pair_events: Deque[Dict[str, Any]] = deque(maxlen=400)
        self._slow_pair_lock = threading.Lock()
        self._last_diagnostics_event_monotonic = 0.0
        self._diagnostics_event_interval_s = 1.0
        self._reacquire_guard_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._object_gating_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._pose_continuity_guard_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._position_continuity_guard_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._rigid_hint_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._rigid_hint_pose_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
        self._subset_hypothesis_events: Deque[Dict[str, Any]] = deque(maxlen=2000)
    
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
        if count > 0:
            self._raw_scene_geometry.load_calibration(filepath)
        self._calibration_loaded = count > 0
        self._apply_epipolar_threshold_override()
        return count

    def _apply_epipolar_threshold_override(self) -> None:
        if self._epipolar_threshold_px_override is None:
            return
        self._apply_epipolar_threshold_override_to(self.geometry)
        self._apply_epipolar_threshold_override_to(self._raw_scene_geometry)

    def _apply_epipolar_threshold_override_to(self, geometry: GeometryPipeline) -> None:
        geometry.epipolar_threshold_px = float(self._epipolar_threshold_px_override)
        if geometry.triangulator is not None:
            geometry.triangulator.epipolar_threshold_px = float(
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
            fast_geometry=self.pipeline_variant != "baseline",
            epipolar_pruning_enabled=self.pipeline_variant == "fast_ABCDP",
            geo_hotpath_optimizations=_variant_has_geo_hotpath_optimizations(
                self.pipeline_variant
            ),
            stage_callback=self._record_stage,
        )
        self._raw_scene_geometry.camera_params = params
        self._raw_scene_geometry.epipolar_threshold_px = self.geometry.epipolar_threshold_px
        self._raw_scene_geometry.triangulator = Triangulator(
            params,
            epipolar_threshold_px=self._raw_scene_geometry.epipolar_threshold_px,
            fast_geometry=False,
            epipolar_pruning_enabled=False,
            geo_hotpath_optimizations=False,
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
        self._start_raw_scene_worker()
        
        # Start frame processor
        self.frame_processor.start()
    
    def stop(self) -> Dict[str, Any]:
        """Stop the tracking pipeline."""
        self._running = False
        
        # Stop frame processor
        self.frame_processor.stop()
        self._stop_raw_scene_worker()
        
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

    def _frame_blob_count(self, paired_frames: PairedFrames) -> int:
        return int(
            sum(len(getattr(frame, "blobs", []) or []) for frame in paired_frames.frames.values())
        )

    def _record_variant_metric(self, name: str, amount: int = 1) -> None:
        with self._variant_metric_lock:
            self._variant_counts[str(name)] += int(amount)

    def _record_fallback_reason(self, reason: str) -> None:
        with self._variant_metric_lock:
            self._fallback_reason_counts[str(reason or "unknown")] += 1

    def _record_object_gating_filter_reasons(self, reasons: Counter[str]) -> None:
        if not reasons:
            return
        with self._variant_metric_lock:
            self._object_gating_filter_reason_counts.update(reasons)

    def _record_blob_reduction(self, *, full_blob_count: int, filtered_blob_count: int) -> None:
        with self._variant_metric_lock:
            self._variant_counts["full_blob_count"] += int(full_blob_count)
            self._variant_counts["filtered_blob_count"] += int(filtered_blob_count)

    @staticmethod
    def _serialize_points(points: Any) -> List[List[float]]:
        serialized: List[List[float]] = []
        for point in list(points or []):
            values = point.tolist() if hasattr(point, "tolist") else list(point)
            if len(values) >= 3:
                serialized.append([float(values[0]), float(values[1]), float(values[2])])
        return serialized

    def _raw_scene_snapshot(self) -> tuple[List[List[float]], int, Dict[str, Any]]:
        with self._triangulation_lock:
            return (
                [list(point) for point in self._raw_scene_latest_points],
                int(self._raw_scene_latest_timestamp),
                _copy_triangulation_quality(self._raw_scene_latest_quality),
            )

    def _start_raw_scene_worker(self) -> None:
        if self._raw_scene_geometry.triangulator is None:
            return
        with self._raw_scene_condition:
            if self._raw_scene_worker is not None and self._raw_scene_worker.is_alive():
                return
            self._raw_scene_worker_stop = False
            self._raw_scene_pending_pair = None
            self._raw_scene_worker = threading.Thread(
                target=self._raw_scene_worker_loop,
                name="loutrack-raw-scene",
                daemon=True,
            )
            self._raw_scene_worker.start()

    def _stop_raw_scene_worker(self) -> None:
        with self._raw_scene_condition:
            self._raw_scene_worker_stop = True
            self._raw_scene_pending_pair = None
            self._raw_scene_condition.notify_all()
            worker = self._raw_scene_worker
        if worker is not None:
            worker.join(timeout=1.0)
        with self._raw_scene_condition:
            if self._raw_scene_worker is worker:
                self._raw_scene_worker = None

    def _submit_raw_scene_pair(self, paired_frames: PairedFrames) -> None:
        with self._raw_scene_condition:
            if self._raw_scene_worker is None or not self._raw_scene_worker.is_alive():
                return
            if self._raw_scene_pending_pair is not None:
                self._record_variant_metric("raw_scene_coalesced_count")
            self._raw_scene_pending_pair = paired_frames
            self._raw_scene_condition.notify()

    def _raw_scene_worker_loop(self) -> None:
        while True:
            with self._raw_scene_condition:
                self._raw_scene_condition.wait_for(
                    lambda: self._raw_scene_worker_stop or self._raw_scene_pending_pair is not None
                )
                if self._raw_scene_worker_stop:
                    return
                paired_frames = self._raw_scene_pending_pair
                self._raw_scene_pending_pair = None
            if paired_frames is None:
                continue
            try:
                result = self._raw_scene_geometry.process_paired_frames(
                    paired_frames,
                    min_inlier_views=2,
                    object_gating=None,
                )
                points = self._serialize_points(result.get("points_3d", []))
                quality = _copy_triangulation_quality(
                    result.get("triangulation_quality", _empty_triangulation_quality())
                )
                timestamp = int(result.get("timestamp", paired_frames.timestamp) or 0)
                with self._triangulation_lock:
                    if timestamp >= self._raw_scene_latest_timestamp:
                        self._raw_scene_latest_points = points
                        self._raw_scene_latest_timestamp = timestamp
                        self._raw_scene_latest_quality = quality
                        latest_timestamp = int(
                            self._latest_triangulation_snapshot.get("timestamp", 0) or 0
                        )
                        if timestamp >= latest_timestamp:
                            self._latest_triangulation_snapshot["raw_scene_points_3d"] = [
                                list(point) for point in points
                            ]
                            self._latest_triangulation_snapshot["raw_scene_timestamp"] = timestamp
                            self._latest_triangulation_snapshot[
                                "raw_scene_triangulation_quality"
                            ] = quality
                self._record_variant_metric("raw_scene_update_count")
            except Exception:
                self._record_variant_metric("raw_scene_error_count")

    def _variant_metrics_snapshot(self) -> Dict[str, Any]:
        with self._variant_metric_lock:
            counts = dict(self._variant_counts)
            fallback_reasons = dict(self._fallback_reason_counts)
            object_gating_filter_reasons = dict(self._object_gating_filter_reason_counts)
        rigid_metrics = {}
        get_variant_metrics = getattr(self.rigid_estimator, "get_variant_metrics", None)
        if callable(get_variant_metrics):
            rigid_metrics = dict(get_variant_metrics())
        return {
            "pipeline_variant": self.pipeline_variant,
            "fast_path_attempt_count": int(counts.get("fast_path_attempt_count", 0)),
            "fast_path_used_count": int(counts.get("fast_path_used_count", 0)),
            "fallback_count": int(counts.get("fallback_count", 0)),
            "fallback_reason_counts": fallback_reasons,
            "full_blob_count": int(counts.get("full_blob_count", 0)),
            "filtered_blob_count": int(counts.get("filtered_blob_count", 0)),
            "raw_scene_update_count": int(counts.get("raw_scene_update_count", 0)),
            "raw_scene_coalesced_count": int(counts.get("raw_scene_coalesced_count", 0)),
            "raw_scene_error_count": int(counts.get("raw_scene_error_count", 0)),
            "subset_sampled_count": int(rigid_metrics.get("subset_sampled_count", 0)),
            "subset_skipped_count": int(rigid_metrics.get("subset_skipped_count", 0)),
            "subset_budget_exceeded_count": int(
                rigid_metrics.get("subset_budget_exceeded_count", 0)
            ),
            "object_gating_filter_reason_counts": object_gating_filter_reasons,
            "rigid_candidate_separated_count": int(
                rigid_metrics.get("rigid_candidate_separated_count", 0)
            ),
            "rigid_candidate_fallback_count": int(
                rigid_metrics.get("rigid_candidate_fallback_count", 0)
            ),
            "rigid_candidate_fallback_reason_counts": dict(
                rigid_metrics.get("rigid_candidate_fallback_reason_counts", {})
            ),
        }

    def _fallback_summary(self) -> Dict[str, Any]:
        metrics = self._variant_metrics_snapshot()
        return {
            "fast_path_used_count": int(metrics.get("fast_path_used_count", 0)),
            "fast_path_attempt_count": int(metrics.get("fast_path_attempt_count", 0)),
            "fallback_count": int(metrics.get("fallback_count", 0)),
            "fallback_reason_counts": dict(metrics.get("fallback_reason_counts", {})),
            "object_gating_filter_reason_counts": dict(
                metrics.get("object_gating_filter_reason_counts", {})
            ),
            "filtered_blob_count": int(metrics.get("filtered_blob_count", 0)),
            "full_blob_count": int(metrics.get("full_blob_count", 0)),
        }

    def _stage_detail_diagnostics(self) -> Dict[str, Dict[str, float]]:
        stages = self._stage_diagnostics()
        return {name: stages.get(name, _stage_summary([])) for name in sorted(_STAGE_DETAIL_NAMES)}

    def _fast_generic_filter(
        self,
        object_gating: Dict[str, Any],
    ) -> tuple[Optional[Dict[str, set[int]]], str]:
        if not object_gating:
            return None, "no_object_gating"
        min_markers = max(3, int(getattr(self.rigid_estimator.object_gating_config, "min_enforced_markers", 3)))
        relaxed = _variant_has_relaxed_object_gating(self.pipeline_variant)
        min_assigned_views = min_markers * 2
        min_two_ray_markers = min_markers
        if relaxed:
            min_assigned_views = max(min_markers + 1, min_assigned_views - 1)
            min_two_ray_markers = max(2, min_markers - 1)
        filtered: Dict[str, set[int]] = {}
        reject_reasons: Counter[str] = Counter()
        usable_rigids = 0
        partial_filter_rejected = False
        for rigid_name, gating in object_gating.items():
            if not isinstance(gating, dict):
                reject_reasons["invalid_gating_payload"] += 1
                partial_filter_rejected = True
                continue
            if not gating.get("evaluated"):
                reject_reasons["gating_not_evaluated"] += 1
                partial_filter_rejected = True
                continue
            reason = str(gating.get("reason") or "unknown")
            if reason != "ok":
                reject_reasons[f"gating_reason_{reason}"] += 1
                partial_filter_rejected = True
                continue
            assigned_views = int(gating.get("assigned_marker_views", 0))
            if assigned_views < min_assigned_views:
                reject_reasons["assigned_marker_views_below_min"] += 1
                partial_filter_rejected = True
                continue
            two_ray_markers = int(gating.get("markers_with_two_or_more_rays", 0))
            single_ray_candidates = int(gating.get("single_ray_candidates", 0))
            if two_ray_markers < min_two_ray_markers:
                reject_reasons["two_ray_markers_below_min"] += 1
                partial_filter_rejected = True
                continue
            if two_ray_markers < min_markers and single_ray_candidates <= 0:
                reject_reasons["single_ray_not_available_for_relaxed_gate"] += 1
                partial_filter_rejected = True
                continue
            per_camera = gating.get("per_camera", {})
            if not isinstance(per_camera, dict):
                reject_reasons["missing_per_camera_assignments"] += 1
                partial_filter_rejected = True
                continue
            rigid_added = 0
            for camera_id, camera_payload in per_camera.items():
                if not isinstance(camera_payload, dict):
                    continue
                for assignment in camera_payload.get("assignments", []) or []:
                    if not isinstance(assignment, dict):
                        continue
                    try:
                        blob_index = int(assignment["blob_index"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    if blob_index < 0:
                        continue
                    filtered.setdefault(str(camera_id), set()).add(blob_index)
                    rigid_added += 1
            if rigid_added > 0:
                usable_rigids += 1
            else:
                reject_reasons["no_filter_indices"] += 1
                partial_filter_rejected = True
        self._record_object_gating_filter_reasons(reject_reasons)
        if usable_rigids > 0 and partial_filter_rejected:
            reason = reject_reasons.most_common(1)[0][0] if reject_reasons else "unknown"
            full_recovery_reasons = {
                "gating_not_evaluated",
                "invalid_gating_payload",
                "missing_per_camera_assignments",
                "no_filter_indices",
            }
            needs_full_recovery = any(
                item in full_recovery_reasons or item.startswith("gating_reason_")
                for item in reject_reasons
            )
            if relaxed and not needs_full_recovery:
                return filtered, f"partial_object_gating_filtered:{reason}"
            return None, f"partial_object_gating:{reason}"
        if usable_rigids <= 0 or not any(filtered.values()):
            if reject_reasons:
                reason = reject_reasons.most_common(1)[0][0]
                return None, f"insufficient_object_gating:{reason}"
            return None, "insufficient_object_gating"
        return filtered, "ok"

    def _filtered_result_usable(self, result: Dict[str, Any]) -> tuple[bool, str]:
        if not isinstance(result, dict) or result.get("error"):
            return False, "filtered_geometry_error"
        quality = result.get("rigid_hint_quality", {})
        if not isinstance(quality, dict):
            return False, "no_rigid_hint_quality"
        by_rigid = quality.get("by_rigid", {})
        if not isinstance(by_rigid, dict) or not by_rigid:
            return False, "no_rigid_hint_candidates"
        min_markers = max(3, int(getattr(self.rigid_estimator.object_gating_config, "min_enforced_markers", 3)))
        reprojection_summary = quality.get("reprojection_error_px_summary", {})
        p95_error = (
            float(reprojection_summary.get("p95", 0.0))
            if isinstance(reprojection_summary, dict)
            else 0.0
        )
        max_p95_error = float(getattr(self.rigid_estimator, "reprojection_match_gate_px", 12.0))
        for summary in by_rigid.values():
            if not isinstance(summary, dict):
                continue
            if int(summary.get("invalid_assignments", 0)) > 0:
                continue
            if int(summary.get("accepted_points", 0)) < min_markers:
                continue
            if int(summary.get("markers_with_two_or_more_rays", 0)) < min_markers:
                continue
            if p95_error > max_p95_error:
                continue
            return True, "ok"
        return False, "filtered_hint_quality_gate"

    def _process_geometry_with_variant(
        self,
        paired_frames: PairedFrames,
        object_gating: Dict[str, Any],
    ) -> Dict[str, Any]:
        full_blob_count = self._frame_blob_count(paired_frames)
        if not _variant_at_least(self.pipeline_variant, "fast_ABCD"):
            self._record_blob_reduction(
                full_blob_count=full_blob_count,
                filtered_blob_count=full_blob_count,
            )
            return self.geometry.process_paired_frames(
                paired_frames,
                min_inlier_views=2,
                object_gating=object_gating,
            )

        generic_filter, reason = self._fast_generic_filter(object_gating)
        if generic_filter is None:
            self._record_variant_metric("fallback_count")
            self._record_fallback_reason(reason)
            self._active_pair_decision = f"fallback:{reason}"
            self._record_blob_reduction(
                full_blob_count=full_blob_count,
                filtered_blob_count=full_blob_count,
            )
            return self.geometry.process_paired_frames(
                paired_frames,
                min_inlier_views=2,
                object_gating=object_gating,
            )

        filtered_blob_count = int(sum(len(indices) for indices in generic_filter.values()))
        self._record_variant_metric("fast_path_attempt_count")
        self._record_blob_reduction(
            full_blob_count=full_blob_count,
            filtered_blob_count=filtered_blob_count,
        )
        filtered_kwargs: Dict[str, Any] = {
            "min_inlier_views": 2,
            "object_gating": object_gating,
            "generic_blob_indices_by_camera": generic_filter,
        }
        if _variant_uses_hint_only_geometry(self.pipeline_variant):
            filtered_kwargs["use_rigid_hint_as_generic"] = True
        filtered_result = self.geometry.process_paired_frames(
            paired_frames,
            **filtered_kwargs,
        )
        usable, fallback_reason = self._filtered_result_usable(filtered_result)
        if usable:
            self._record_variant_metric("fast_path_used_count")
            self._active_pair_decision = "fast_path"
            return filtered_result

        self._record_variant_metric("fallback_count")
        self._record_fallback_reason(fallback_reason)
        self._active_pair_decision = f"fallback:{fallback_reason}"
        fallback_started_ns = time.perf_counter_ns()
        fallback_result = self.geometry.process_paired_frames(
            paired_frames,
            min_inlier_views=2,
            object_gating=object_gating,
        )
        self._record_stage("fallback_ms", self._elapsed_ms(fallback_started_ns))
        return fallback_result
    
    def _on_paired_frames(self, paired_frames: PairedFrames) -> None:
        """Process paired frames."""
        if not self._running:
            return
        
        pipeline_started_ns = time.perf_counter_ns()
        self._active_pair_stage_ms = {}
        self._active_pair_decision = "full_path"
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
            object_gating = {}
            if self._calibration_loaded and hasattr(self.rigid_estimator, "evaluate_object_conditioned_gating"):
                object_stage_started_ns = time.perf_counter_ns()
                object_gating = self.rigid_estimator.evaluate_object_conditioned_gating(
                    timestamp=timestamp,
                    camera_params=self.geometry.camera_params,
                    frames_by_camera=paired_frames.frames,
                    coordinate_space="raw_pixel",
                )
                self._record_stage("object_gating_ms", self._elapsed_ms(object_stage_started_ns))
                self._record_object_gating_events(timestamp, object_gating)
            stage_started_ns = time.perf_counter_ns()
            if self._calibration_loaded:
                result = self._process_geometry_with_variant(
                    paired_frames,
                    object_gating,
                )
                points_3d = result.get("points_3d", [])
                self._record_rigid_hint_events(timestamp, result.get("rigid_hint_quality", {}))
            else:
                points_3d = []
            self._record_stage("triangulation_ms", self._elapsed_ms(stage_started_ns))
            self._submit_raw_scene_pair(paired_frames)

            points_3d_list = list(points_3d) if points_3d is not None else []
            point_count = len(points_3d_list)
            triangulation_quality = _copy_triangulation_quality(
                result.get("triangulation_quality", _empty_triangulation_quality())
            )
            points_3d_serialized = self._serialize_points(points_3d_list)
            raw_scene_points, raw_scene_timestamp, raw_scene_quality = self._raw_scene_snapshot()
            if not raw_scene_points:
                raw_scene_points = [list(point) for point in points_3d_serialized]
                raw_scene_timestamp = int(timestamp)
                raw_scene_quality = triangulation_quality
            rigid_hint_points = list(result.get("rigid_hint_points_3d", []) or [])

            with self._triangulation_lock:
                self._latest_triangulation_snapshot = {
                    "timestamp": timestamp,
                    "points_3d": points_3d_serialized,
                    "raw_scene_points_3d": raw_scene_points,
                    "raw_scene_timestamp": raw_scene_timestamp,
                    "raw_scene_triangulation_quality": raw_scene_quality,
                    "rigid_hint_points_3d": [
                        point.tolist() if hasattr(point, "tolist") else list(point)
                        for point in rigid_hint_points
                    ],
                    "observations_by_camera": dict(result.get("observations_by_camera", {})),
                    "triangulated_points": list(result.get("triangulated_points", [])),
                    "rigid_hint_triangulated_points": list(
                        result.get("rigid_hint_triangulated_points", [])
                    ),
                    "reprojection_errors": list(result.get("reprojection_errors", [])),
                    "rigid_hint_reprojection_errors": list(
                        result.get("rigid_hint_reprojection_errors", [])
                    ),
                    "triangulation_quality": triangulation_quality,
                    "rigid_hint_quality": dict(result.get("rigid_hint_quality", {})),
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
                    "object_gating": dict(object_gating),
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
                    rigid_hint_triangulated_points=result.get(
                        "rigid_hint_triangulated_points",
                        [],
                    ),
                    coordinate_space="raw_pixel",
                )
            else:
                poses = self.rigid_estimator.process_points(points_array, timestamp)
            self._record_stage("rigid_ms", self._elapsed_ms(stage_started_ns))
            self._record_stage("rigid_pose_ms", self._elapsed_ms(stage_started_ns))
            if hasattr(self.rigid_estimator, "get_tracking_event_status"):
                tracking_status = self.rigid_estimator.get_tracking_event_status()
            else:
                tracking_status = self.rigid_estimator.get_tracking_status()
            self._record_reacquire_guard_events(timestamp, tracking_status)
            self._record_position_continuity_guard_events(timestamp, tracking_status)
            self._record_pose_continuity_guard_events(timestamp, tracking_status)
            self._record_rigid_hint_pose_events(timestamp, tracking_status)
            self._record_subset_hypothesis_events(timestamp, tracking_status)
            
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
            if self._active_pair_stage_ms is not None:
                self._record_slow_pair_event(
                    paired_frames,
                    dict(self._active_pair_stage_ms),
                )
            self._maybe_log_diagnostics_event()
            
        except Exception as e:
            if self._error_callback:
                self._error_callback(e)
        finally:
            self._active_pair_stage_ms = None
            self._active_pair_decision = ""

    @staticmethod
    def _elapsed_ms(started_ns: int) -> float:
        return float(time.perf_counter_ns() - started_ns) / 1_000_000.0

    def _record_stage(self, name: str, value_ms: float) -> None:
        with self._stage_lock:
            bucket = self._stage_ms.setdefault(name, deque(maxlen=240))
            value = float(value_ms)
            bucket.append(value)
            if self._active_pair_stage_ms is not None:
                self._active_pair_stage_ms[name] = (
                    float(self._active_pair_stage_ms.get(name, 0.0)) + value
                )

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

    @staticmethod
    def _object_gating_from_tracking(tracking_status: Dict[str, Any]) -> Dict[str, Any]:
        return {
            name: dict(status.get("object_gating", {}))
            for name, status in tracking_status.items()
            if isinstance(status, dict) and isinstance(status.get("object_gating"), dict)
        }

    def _diagnostics_snapshot(self) -> Dict[str, Any]:
        tracking_status = self.rigid_estimator.get_tracking_status()
        return {
            "receiver": self.frame_processor.get_stats(),
            "geometry": self._geometry_diagnostics(),
            "tracking": tracking_status,
            "object_gating": self._object_gating_from_tracking(tracking_status),
            "object_gating_events": self.get_object_gating_events(limit=20),
            "position_continuity_guard_events": (
                self.get_position_continuity_guard_events(limit=20)
            ),
            "pose_continuity_guard_events": self.get_pose_continuity_guard_events(limit=20),
            "rigid_hint_events": self.get_rigid_hint_events(limit=20),
            "rigid_hint_pose_events": self.get_rigid_hint_pose_events(limit=20),
            "subset_hypothesis_events": self.get_subset_hypothesis_events(limit=20),
            "reacquire_guard_events": self.get_reacquire_guard_events(limit=20),
            "pipeline_stage_ms": self._stage_diagnostics(),
            "stage_ms_detail": self._stage_detail_diagnostics(),
            "variant_metrics": self._variant_metrics_snapshot(),
            "fallback_summary": self._fallback_summary(),
            "backpressure_summary": {},
            "slow_pair_events": self.get_slow_pair_events(limit=20),
            "logger": self._logger_diagnostics(),
            "metrics": self.metrics.get_summary(),
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
        }

    def _record_reacquire_guard_events(
        self,
        timestamp: int,
        tracking: Optional[Dict[str, Any]] = None,
    ) -> None:
        if tracking is None:
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

    def _record_object_gating_events(
        self,
        timestamp: int,
        object_gating: Dict[str, Dict[str, Any]],
    ) -> None:
        for rigid_name, gating in object_gating.items():
            if not isinstance(gating, dict) or not gating.get("evaluated"):
                continue
            event = {
                "timestamp": int(timestamp),
                "rigid_name": str(rigid_name),
                "mode": str(gating.get("mode", "")),
                "enforced": bool(gating.get("enforced", False)),
                "diagnostics_only": bool(gating.get("diagnostics_only", True)),
                "reason": str(gating.get("reason", "")),
                "confidence": float(gating.get("confidence", 0.0)),
                "pixel_gate_px": float(gating.get("pixel_gate_px", 0.0)),
                "assigned_marker_views": int(gating.get("assigned_marker_views", 0)),
                "candidate_window_count": int(gating.get("candidate_window_count", 0)),
                "markers_with_two_or_more_rays": int(gating.get("markers_with_two_or_more_rays", 0)),
                "single_ray_candidates": int(gating.get("single_ray_candidates", 0)),
                "generic_fallback_blob_count": int(gating.get("generic_fallback_blob_count", 0)),
            }
            self._object_gating_events.append(event)

    def _record_position_continuity_guard_events(
        self,
        timestamp: int,
        tracking: Optional[Dict[str, Any]] = None,
    ) -> None:
        if tracking is None:
            tracking = self.rigid_estimator.get_tracking_status()
        for rigid_name, status in tracking.items():
            if not isinstance(status, dict):
                continue
            guard = status.get("position_continuity_guard")
            if not isinstance(guard, dict):
                continue
            if not (
                guard.get("evaluated")
                or guard.get("would_reject")
                or guard.get("clamped_position")
            ):
                continue
            self._position_continuity_guard_events.append(
                {
                    "timestamp": int(timestamp),
                    "rigid_name": str(rigid_name),
                    "mode": str(status.get("mode", "")),
                    "valid": bool(status.get("valid", False)),
                    "enforced": bool(guard.get("enforced", False)),
                    "passed": bool(guard.get("passed", True)),
                    "would_reject": bool(guard.get("would_reject", False)),
                    "clamped_position": bool(guard.get("clamped_position", False)),
                    "reason": str(guard.get("reason", "")),
                    "occluded": bool(guard.get("occluded", False)),
                    "observed_markers": int(guard.get("observed_markers", 0)),
                    "expected_markers": int(guard.get("expected_markers", 0)),
                    "missing_marker_views": int(guard.get("missing_marker_views", 0)),
                    "position_innovation_m": float(guard.get("position_innovation_m", 0.0)),
                    "position_velocity_m_s": float(guard.get("position_velocity_m_s", 0.0)),
                    "previous_velocity_m_s": float(guard.get("previous_velocity_m_s", 0.0)),
                    "position_accel_m_s2": float(guard.get("position_accel_m_s2", 0.0)),
                    "limited_velocity_m_s": float(guard.get("limited_velocity_m_s", 0.0)),
                    "clamped_count": int(guard.get("clamped_count", 0)),
                }
            )

    def _record_pose_continuity_guard_events(
        self,
        timestamp: int,
        tracking: Optional[Dict[str, Any]] = None,
    ) -> None:
        if tracking is None:
            tracking = self.rigid_estimator.get_tracking_status()
        for rigid_name, status in tracking.items():
            if not isinstance(status, dict):
                continue
            guard = status.get("pose_continuity_guard")
            if not isinstance(guard, dict):
                continue
            if not (
                guard.get("evaluated")
                or guard.get("would_reject")
                or guard.get("held_prediction")
                or guard.get("held_rotation")
            ):
                continue
            self._pose_continuity_guard_events.append(
                {
                    "timestamp": int(timestamp),
                    "rigid_name": str(rigid_name),
                    "mode": str(status.get("mode", "")),
                    "valid": bool(status.get("valid", False)),
                    "enforced": bool(guard.get("enforced", False)),
                    "passed": bool(guard.get("passed", True)),
                    "would_reject": bool(guard.get("would_reject", False)),
                    "held_prediction": bool(guard.get("held_prediction", False)),
                    "held_rotation": bool(guard.get("held_rotation", False)),
                    "reason": str(guard.get("reason", "")),
                    "occluded": bool(guard.get("occluded", False)),
                    "observed_markers": int(guard.get("observed_markers", 0)),
                    "expected_markers": int(guard.get("expected_markers", 0)),
                    "missing_marker_views": int(guard.get("missing_marker_views", 0)),
                    "position_innovation_m": float(guard.get("position_innovation_m", 0.0)),
                    "rotation_innovation_deg": float(guard.get("rotation_innovation_deg", 0.0)),
                    "angular_velocity_deg_s": float(guard.get("angular_velocity_deg_s", 0.0)),
                    "angular_accel_deg_s2": float(guard.get("angular_accel_deg_s2", 0.0)),
                    "held_count": int(guard.get("held_count", 0)),
                }
            )

    def _record_rigid_hint_events(
        self,
        timestamp: int,
        rigid_hint_quality: Dict[str, Any],
    ) -> None:
        if not isinstance(rigid_hint_quality, dict):
            return
        by_rigid = rigid_hint_quality.get("by_rigid", {})
        if not isinstance(by_rigid, dict) or not by_rigid:
            return
        reprojection_summary = rigid_hint_quality.get("reprojection_error_px_summary", {})
        if not isinstance(reprojection_summary, dict):
            reprojection_summary = {}
        for rigid_name, summary in by_rigid.items():
            if not isinstance(summary, dict):
                continue
            self._rigid_hint_events.append(
                {
                    "timestamp": int(timestamp),
                    "rigid_name": str(rigid_name),
                    "candidate_markers": int(summary.get("candidate_markers", 0)),
                    "markers_with_two_or_more_rays": int(
                        summary.get("markers_with_two_or_more_rays", 0)
                    ),
                    "single_ray_candidates": int(summary.get("single_ray_candidates", 0)),
                    "accepted_points": int(summary.get("accepted_points", 0)),
                    "rejected_markers": int(summary.get("rejected_markers", 0)),
                    "invalid_assignments": int(summary.get("invalid_assignments", 0)),
                    "reprojection_mean_px": float(reprojection_summary.get("mean", 0.0)),
                    "reprojection_p95_px": float(reprojection_summary.get("p95", 0.0)),
                }
            )

    def _record_rigid_hint_pose_events(
        self,
        timestamp: int,
        tracking: Optional[Dict[str, Any]] = None,
    ) -> None:
        if tracking is None:
            tracking = self.rigid_estimator.get_tracking_status()
        for rigid_name, status in tracking.items():
            if not isinstance(status, dict):
                continue
            hint_pose = status.get("rigid_hint_pose")
            if not isinstance(hint_pose, dict) or not hint_pose.get("evaluated"):
                continue
            score = hint_pose.get("score", {})
            generic_score = hint_pose.get("generic_score", {})
            if not isinstance(score, dict):
                score = {}
            if not isinstance(generic_score, dict):
                generic_score = {}
            self._rigid_hint_pose_events.append(
                {
                    "timestamp": int(timestamp),
                    "rigid_name": str(rigid_name),
                    "mode": str(status.get("mode", "")),
                    "reason": str(hint_pose.get("reason", "")),
                    "valid": bool(hint_pose.get("valid", False)),
                    "enforced": bool(hint_pose.get("enforced", False)),
                    "diagnostics_only": bool(hint_pose.get("diagnostics_only", True)),
                    "selected_for_pose": bool(hint_pose.get("selected_for_pose", False)),
                    "selection_reason": str(hint_pose.get("selection_reason", "")),
                    "generic_valid": bool(hint_pose.get("generic_valid", False)),
                    "would_improve_score": bool(hint_pose.get("would_improve_score", False)),
                    "candidate_points": int(hint_pose.get("candidate_points", 0)),
                    "observed_markers": int(hint_pose.get("observed_markers", 0)),
                    "real_ray_count": int(hint_pose.get("real_ray_count", 0)),
                    "virtual_marker_count": int(hint_pose.get("virtual_marker_count", 0)),
                    "rms_error_m": float(hint_pose.get("rms_error_m", 0.0)),
                    "generic_rms_error_m": float(hint_pose.get("generic_rms_error_m", 0.0)),
                    "score": float(score.get("score", 0.0)),
                    "generic_score": float(generic_score.get("score", 0.0)),
                    "score_delta": float(hint_pose.get("score_delta", 0.0)),
                    "position_delta_m": float(hint_pose.get("position_delta_m", 0.0)),
                    "rotation_delta_deg": float(hint_pose.get("rotation_delta_deg", 0.0)),
                    "matched_marker_views": int(score.get("matched_marker_views", 0)),
                    "p95_error_px": float(score.get("p95_error_px", 0.0)),
                }
            )

    def _record_subset_hypothesis_events(
        self,
        timestamp: int,
        tracking: Optional[Dict[str, Any]] = None,
    ) -> None:
        if tracking is None:
            tracking = self.rigid_estimator.get_tracking_status()
        for rigid_name, status in tracking.items():
            if not isinstance(status, dict):
                continue
            subset = status.get("subset_hypothesis")
            if not isinstance(subset, dict) or not subset.get("evaluated"):
                continue
            best = subset.get("best", {})
            second = subset.get("second", {})
            if not isinstance(best, dict):
                best = {}
            if not isinstance(second, dict):
                second = {}
            self._subset_hypothesis_events.append(
                {
                    "timestamp": int(timestamp),
                    "rigid_name": str(rigid_name),
                    "reason": str(subset.get("reason", "")),
                    "generic_valid": bool(subset.get("generic_valid", False)),
                    "candidate_count": int(subset.get("candidate_count", 0)),
                    "pruned_candidate_count": int(subset.get("pruned_candidate_count", 0)),
                    "valid_candidate_count": int(subset.get("valid_candidate_count", 0)),
                    "rejected_by_ambiguity": int(subset.get("rejected_by_ambiguity", 0)),
                    "rejected_by_2d_score": int(subset.get("rejected_by_2d_score", 0)),
                    "rejected_by_rms": int(subset.get("rejected_by_rms", 0)),
                    "flip_risk_count": int(subset.get("flip_risk_count", 0)),
                    "truncated": bool(subset.get("truncated", False)),
                    "time_budget_ms": float(subset.get("time_budget_ms", 0.0)),
                    "time_budget_exceeded": bool(subset.get("time_budget_exceeded", False)),
                    "effective_max_hypotheses": int(
                        subset.get("effective_max_hypotheses", 0)
                    ),
                    "best_source": str(best.get("source", "")),
                    "best_score": float(subset.get("best_score", 0.0)),
                    "second_score": float(subset.get("second_score", 0.0)),
                    "best_combined_score": float(subset.get("best_combined_score", 0.0)),
                    "second_combined_score": float(subset.get("second_combined_score", 0.0)),
                    "margin": float(subset.get("margin", 0.0)),
                    "combined_margin": float(subset.get("combined_margin", 0.0)),
                    "generic_score": float(subset.get("generic_score", 0.0)),
                    "score_delta": float(subset.get("score_delta", 0.0)),
                    "best_p95_error_px": float(best.get("p95_error_px", 0.0)),
                    "best_position_delta_m": float(best.get("position_delta_m", 0.0)),
                    "best_rotation_delta_deg": float(best.get("rotation_delta_deg", 0.0)),
                    "subset_adoption_ready": bool(subset.get("subset_adoption_ready", False)),
                    "diagnostics_only": bool(subset.get("diagnostics_only", True)),
                }
            )

    def get_reacquire_guard_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._reacquire_guard_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_object_gating_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._object_gating_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_pose_continuity_guard_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._pose_continuity_guard_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_position_continuity_guard_events(
        self,
        limit: Optional[int] = None,
    ) -> List[Dict[str, Any]]:
        events = list(self._position_continuity_guard_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_rigid_hint_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._rigid_hint_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_rigid_hint_pose_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._rigid_hint_pose_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def get_subset_hypothesis_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        events = list(self._subset_hypothesis_events)
        if limit is not None:
            events = events[-max(0, int(limit)):]
        return [dict(event) for event in events]

    def _record_slow_pair_event(
        self,
        paired_frames: PairedFrames,
        stage_ms: Dict[str, float],
    ) -> None:
        frame_blob_counts = {
            str(camera_id): int(len(getattr(frame, "blobs", []) or []))
            for camera_id, frame in paired_frames.frames.items()
        }
        event = {
            "timestamp": int(paired_frames.timestamp),
            "pair_timestamp_range_us": int(paired_frames.timestamp_range_us),
            "camera_count": int(len(paired_frames.frames)),
            "blob_count": int(sum(frame_blob_counts.values())),
            "frame_blob_counts": frame_blob_counts,
            "stage_ms": {str(name): float(value) for name, value in stage_ms.items()},
            "pipeline_pair_ms": float(stage_ms.get("pipeline_pair_ms", 0.0)),
            "decision": str(self._active_pair_decision or ""),
        }
        with self._slow_pair_lock:
            self._slow_pair_events.append(event)

    def get_slow_pair_events(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        with self._slow_pair_lock:
            events = [dict(event) for event in self._slow_pair_events]
        events.sort(key=lambda item: float(item.get("pipeline_pair_ms", 0.0)), reverse=True)
        if limit is not None:
            events = events[: max(0, int(limit))]
        return events

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
        tracking_status = self.rigid_estimator.get_tracking_status()
        return {
            "running": self._running,
            "calibration_loaded": self._calibration_loaded,
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "uptime_seconds": time.time() - self.start_time if self.start_time else 0,
            "receiver": self.frame_processor.get_stats(),
            "metrics": self.metrics.get_summary(),
            "tracking": tracking_status,
            "triangulation_quality": _copy_triangulation_quality(
                triangulation.get("triangulation_quality", _empty_triangulation_quality())
            ),
            "diagnostics": {
                "geometry": self._geometry_diagnostics(),
                "tracking": tracking_status,
                "reacquire_guard_events": self.get_reacquire_guard_events(limit=20),
                "object_gating": self._object_gating_from_tracking(tracking_status),
                "object_gating_events": self.get_object_gating_events(limit=20),
                "position_continuity_guard_events": (
                    self.get_position_continuity_guard_events(limit=20)
                ),
                "pose_continuity_guard_events": self.get_pose_continuity_guard_events(limit=20),
                "rigid_hint_events": self.get_rigid_hint_events(limit=20),
                "rigid_hint_pose_events": self.get_rigid_hint_pose_events(limit=20),
                "subset_hypothesis_events": self.get_subset_hypothesis_events(limit=20),
                "pipeline_stage_ms": self._stage_diagnostics(),
                "stage_ms_detail": self._stage_detail_diagnostics(),
                "variant_metrics": self._variant_metrics_snapshot(),
                "fallback_summary": self._fallback_summary(),
                "backpressure_summary": {},
                "slow_pair_events": self.get_slow_pair_events(limit=20),
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
                "raw_scene_points_3d": [
                    list(point)
                    for point in self._latest_triangulation_snapshot.get(
                        "raw_scene_points_3d",
                        self._latest_triangulation_snapshot["points_3d"],
                    )
                ],
                "raw_scene_timestamp": int(
                    self._latest_triangulation_snapshot.get(
                        "raw_scene_timestamp",
                        self._latest_triangulation_snapshot["timestamp"],
                    )
                ),
                "raw_scene_triangulation_quality": _copy_triangulation_quality(
                    self._latest_triangulation_snapshot.get(
                        "raw_scene_triangulation_quality",
                        self._latest_triangulation_snapshot.get(
                            "triangulation_quality",
                            _empty_triangulation_quality(),
                        ),
                    )
                ),
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
                "rigid_hint_points_3d": [
                    list(point)
                    for point in self._latest_triangulation_snapshot.get(
                        "rigid_hint_points_3d",
                        [],
                    )
                ],
                "rigid_hint_triangulated_points": [
                    dict(point)
                    for point in self._latest_triangulation_snapshot.get(
                        "rigid_hint_triangulated_points",
                        [],
                    )
                ],
                "reprojection_errors": list(self._latest_triangulation_snapshot["reprojection_errors"]),
                "rigid_hint_reprojection_errors": list(
                    self._latest_triangulation_snapshot.get(
                        "rigid_hint_reprojection_errors",
                        [],
                    )
                ),
                "triangulation_quality": _copy_triangulation_quality(
                    self._latest_triangulation_snapshot.get(
                        "triangulation_quality",
                        _empty_triangulation_quality(),
                    )
                ),
                "rigid_hint_quality": dict(
                    self._latest_triangulation_snapshot.get("rigid_hint_quality", {})
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
                "object_gating": dict(self._latest_triangulation_snapshot.get("object_gating", {})),
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
            log_dir=self.config.get("log_dir", "./logs"),
            rigid_stabilization=self.config.get("rigid_stabilization"),
            pipeline_variant=self.config.get("pipeline_variant", DEFAULT_PIPELINE_VARIANT),
            subset_diagnostics_mode=self.config.get("subset_diagnostics_mode"),
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
