"""Same-path replay harness for tracking stabilization diagnostics.

This module replays recorded JSONL frame logs through the existing host
pipeline after UDP reception: FrameProcessor -> FramePairer ->
TrackingPipeline._on_paired_frames -> GeometryPipeline -> RigidBodyEstimator.
It is intentionally diagnostic-only and does not change live tracking behavior.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

import numpy as np

from .receiver import Frame
from .replay import FrameReplay
from .rigid import (
    CHEST_PATTERN,
    HEAD_PATTERN,
    LEFT_FOOT_PATTERN,
    RIGHT_FOOT_PATTERN,
    WAIST_PATTERN,
    MarkerPattern,
    ReacquireGuardConfig,
    ObjectGatingConfig,
    PoseContinuityGuardConfig,
    PositionContinuityGuardConfig,
)
from .pipeline import (
    DEFAULT_PIPELINE_VARIANT,
    PIPELINE_VARIANTS,
    TrackingPipeline,
    default_subset_diagnostics_mode_for_variant,
    normalize_pipeline_variant,
)


BUILTIN_PATTERNS: Dict[str, MarkerPattern] = {
    pattern.name: pattern
    for pattern in (
        WAIST_PATTERN,
        HEAD_PATTERN,
        CHEST_PATTERN,
        LEFT_FOOT_PATTERN,
        RIGHT_FOOT_PATTERN,
    )
}


@dataclass
class TrackingReplaySummary:
    """Serializable summary emitted by the same-path replay harness."""

    log_path: str
    calibration_path: str
    frame_count: int
    pair_count: int
    frames_processed: int
    poses_estimated: int
    patterns: List[str]
    tracking: Dict[str, Dict[str, Any]]
    receiver: Dict[str, Any]
    geometry: Dict[str, Any]
    pipeline_stage_ms: Dict[str, Any]
    reacquire_guard_events: List[Dict[str, Any]]
    reacquire_guard_summary: Dict[str, Any]
    object_gating_events: List[Dict[str, Any]]
    object_gating_summary: Dict[str, Any]
    position_continuity_guard_events: List[Dict[str, Any]]
    position_continuity_guard_summary: Dict[str, Any]
    pose_continuity_guard_events: List[Dict[str, Any]]
    pose_continuity_guard_summary: Dict[str, Any]
    rigid_hint_events: List[Dict[str, Any]]
    rigid_hint_summary: Dict[str, Any]
    rigid_hint_pose_events: List[Dict[str, Any]]
    rigid_hint_pose_summary: Dict[str, Any]
    subset_hypothesis_events: List[Dict[str, Any]]
    subset_hypothesis_summary: Dict[str, Any]
    phase45_go_no_go: Dict[str, Any]
    pipeline_variant: str
    variant_metrics: Dict[str, Any]
    variant_go_no_go: Dict[str, Any]
    stage_ms_detail: Dict[str, Any]
    fallback_summary: Dict[str, Any]
    backpressure_summary: Dict[str, Any]
    slow_pair_events: List[Dict[str, Any]]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "log_path": self.log_path,
            "calibration_path": self.calibration_path,
            "frame_count": int(self.frame_count),
            "pair_count": int(self.pair_count),
            "frames_processed": int(self.frames_processed),
            "poses_estimated": int(self.poses_estimated),
            "patterns": list(self.patterns),
            "tracking": self.tracking,
            "receiver": self.receiver,
            "geometry": self.geometry,
            "pipeline_stage_ms": self.pipeline_stage_ms,
            "reacquire_guard_events": self.reacquire_guard_events,
            "reacquire_guard_summary": self.reacquire_guard_summary,
            "object_gating_events": self.object_gating_events,
            "object_gating_summary": self.object_gating_summary,
            "position_continuity_guard_events": self.position_continuity_guard_events,
            "position_continuity_guard_summary": self.position_continuity_guard_summary,
            "pose_continuity_guard_events": self.pose_continuity_guard_events,
            "pose_continuity_guard_summary": self.pose_continuity_guard_summary,
            "rigid_hint_events": self.rigid_hint_events,
            "rigid_hint_summary": self.rigid_hint_summary,
            "rigid_hint_pose_events": self.rigid_hint_pose_events,
            "rigid_hint_pose_summary": self.rigid_hint_pose_summary,
            "subset_hypothesis_events": self.subset_hypothesis_events,
            "subset_hypothesis_summary": self.subset_hypothesis_summary,
            "phase45_go_no_go": self.phase45_go_no_go,
            "pipeline_variant": self.pipeline_variant,
            "variant_metrics": self.variant_metrics,
            "variant_go_no_go": self.variant_go_no_go,
            "stage_ms_detail": self.stage_ms_detail,
            "fallback_summary": self.fallback_summary,
            "backpressure_summary": self.backpressure_summary,
            "slow_pair_events": self.slow_pair_events,
        }


def replay_tracking_log(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    reacquire_guard_enforced: bool = False,
    reacquire_guard_shadow_enabled: bool = True,
    reacquire_guard_event_logging: bool = False,
    reacquire_guard_post_reacquire_frames: int = 0,
    reacquire_guard_max_rotation_deg: float = 136.0,
    object_gating_enforced: bool = False,
    object_gating_activation_mode: str = "always",
    object_gating_ambiguous_blob_min_separation_px: float = 0.60,
    object_gating_ambiguous_blob_diameter_overlap_ratio: float = 0.30,
    object_gating_ambiguous_marker_assignment_min_margin_px: float = 0.29,
    pose_continuity_guard_enabled: bool = False,
    pose_continuity_guard_enforced: bool = False,
    pose_continuity_max_rotation_deg: float = 90.0,
    pose_continuity_max_angular_velocity_deg_s: float = 2500.0,
    pose_continuity_max_angular_accel_deg_s2: float = 200000.0,
    position_continuity_guard_enabled: bool = False,
    position_continuity_guard_enforced: bool = False,
    position_continuity_max_accel_m_s2: float = 60.0,
    position_continuity_max_velocity_m_s: float = 8.0,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    pipeline_variant: str = DEFAULT_PIPELINE_VARIANT,
    subset_diagnostics_mode: Optional[str] = None,
) -> TrackingReplaySummary:
    """Replay a tracking log through the existing host pipeline path."""
    log_path = Path(log_path)
    calibration_input_path = Path(calibration_path)
    calibration_path = _resolve_replay_calibration_path(calibration_input_path)
    selected_patterns = _load_patterns(patterns, rigids_path)
    pipeline_variant = normalize_pipeline_variant(pipeline_variant)
    subset_diagnostics_mode = (
        subset_diagnostics_mode
        or default_subset_diagnostics_mode_for_variant(pipeline_variant)
    )

    replay = FrameReplay(str(log_path))
    pipeline = TrackingPipeline(
        calibration_path=str(calibration_path),
        patterns=selected_patterns,
        enable_logging=False,
        epipolar_threshold_px=epipolar_threshold_px,
        reacquire_guard_config=ReacquireGuardConfig(
            shadow_enabled=bool(reacquire_guard_shadow_enabled),
            enforced=bool(reacquire_guard_enforced),
            post_reacquire_continue_frames=int(reacquire_guard_post_reacquire_frames),
            max_rotation_innovation_deg=float(reacquire_guard_max_rotation_deg),
        ),
        object_gating_config=ObjectGatingConfig(
            enforce=bool(object_gating_enforced),
            activation_mode=str(object_gating_activation_mode),
            ambiguous_blob_min_separation_px=float(
                object_gating_ambiguous_blob_min_separation_px
            ),
            ambiguous_blob_diameter_overlap_ratio=float(
                object_gating_ambiguous_blob_diameter_overlap_ratio
            ),
            ambiguous_marker_assignment_min_margin_px=float(
                object_gating_ambiguous_marker_assignment_min_margin_px
            ),
        ),
        pose_continuity_guard_config=PoseContinuityGuardConfig(
            enabled=bool(pose_continuity_guard_enabled),
            enforced=bool(pose_continuity_guard_enforced),
            max_rotation_innovation_deg=float(pose_continuity_max_rotation_deg),
            max_angular_velocity_deg_s=float(pose_continuity_max_angular_velocity_deg_s),
            max_angular_accel_deg_s2=float(pose_continuity_max_angular_accel_deg_s2),
        ),
        position_continuity_guard_config=PositionContinuityGuardConfig(
            enabled=bool(position_continuity_guard_enabled),
            enforced=bool(position_continuity_guard_enforced),
            max_accel_m_s2=float(position_continuity_max_accel_m_s2),
            max_velocity_m_s=float(position_continuity_max_velocity_m_s),
        ),
        reacquire_guard_event_logging=bool(reacquire_guard_event_logging),
        pipeline_variant=pipeline_variant,
        subset_diagnostics_mode=subset_diagnostics_mode,
    )
    if not pipeline._calibration_loaded:
        raise RuntimeError(f"calibration could not be loaded: {calibration_path}")

    pipeline._running = True
    # Recorded logs are often older than FrameBuffer's live stale-frame window.
    # Replay should preserve the original receive timing instead of dropping
    # frames because wall-clock time has moved on.
    frame_buffer = getattr(pipeline.frame_processor, "buffer", None)
    if frame_buffer is not None:
        frame_buffer.max_age_seconds = 1_000_000_000.0
    pipeline.frame_processor.set_paired_callback(pipeline._on_paired_frames)

    frame_count = 0
    received_times_s: List[float] = []
    start_received_s = _timestamp_to_seconds(start_received_at) if start_received_at else None
    end_received_s = _timestamp_to_seconds(end_received_at) if end_received_at else None
    for entry in replay.replay(realtime=False):
        timestamp_us = int(entry.data.get("timestamp", 0) or 0)
        if start_timestamp_us is not None and timestamp_us < int(start_timestamp_us):
            continue
        if end_timestamp_us is not None and timestamp_us > int(end_timestamp_us):
            continue
        received_s = _timestamp_to_seconds(entry.received_at)
        if start_received_s is not None and received_s < start_received_s:
            continue
        if end_received_s is not None and received_s > end_received_s:
            continue
        if max_frames is not None and frame_count >= int(max_frames):
            break
        received_times_s.append(received_s)
        pipeline.frame_processor._on_frame_received(_frame_from_log_entry(entry))
        frame_count += 1

    # Flush any pair that became matchable on the last injected frame.
    pipeline.frame_processor._process_pairs()
    pipeline._running = False

    status = pipeline.get_status()
    diagnostics = status.get("diagnostics", {})
    receiver = status.get("receiver", {})
    tracking = dict(status.get("tracking", {}))
    guard_events = (
        pipeline.get_reacquire_guard_events()
        if hasattr(pipeline, "get_reacquire_guard_events")
        else []
    )
    object_gating_events = (
        pipeline.get_object_gating_events()
        if hasattr(pipeline, "get_object_gating_events")
        else []
    )
    position_continuity_guard_events = (
        pipeline.get_position_continuity_guard_events()
        if hasattr(pipeline, "get_position_continuity_guard_events")
        else []
    )
    pose_continuity_guard_events = (
        pipeline.get_pose_continuity_guard_events()
        if hasattr(pipeline, "get_pose_continuity_guard_events")
        else []
    )
    rigid_hint_events = (
        pipeline.get_rigid_hint_events()
        if hasattr(pipeline, "get_rigid_hint_events")
        else []
    )
    rigid_hint_pose_events = (
        pipeline.get_rigid_hint_pose_events()
        if hasattr(pipeline, "get_rigid_hint_pose_events")
        else []
    )
    subset_hypothesis_events = (
        pipeline.get_subset_hypothesis_events()
        if hasattr(pipeline, "get_subset_hypothesis_events")
        else []
    )
    pair_count = int(
        receiver.get("pairer", {}).get("pairs_emitted", pipeline.frames_processed)
    )
    guard_summary = _summarize_reacquire_guard(tracking, guard_events)
    pipeline_stage_ms = dict(diagnostics.get("pipeline_stage_ms", {}))
    backpressure_summary = _simulate_backpressure(
        received_times_s,
        pair_count=pair_count,
        pipeline_stage_ms=pipeline_stage_ms,
        enabled=pipeline_variant == "fast_ABCDE",
    )

    return TrackingReplaySummary(
        log_path=str(log_path),
        calibration_path=str(calibration_path),
        frame_count=frame_count,
        pair_count=pair_count,
        frames_processed=int(status.get("frames_processed", pipeline.frames_processed)),
        poses_estimated=int(status.get("poses_estimated", pipeline.poses_estimated)),
        patterns=[pattern.name for pattern in selected_patterns],
        tracking=tracking,
        receiver=dict(receiver),
        geometry=dict(diagnostics.get("geometry", {})),
        pipeline_stage_ms=pipeline_stage_ms,
        reacquire_guard_events=guard_events,
        reacquire_guard_summary=guard_summary,
        object_gating_events=object_gating_events,
        object_gating_summary=_summarize_object_gating(tracking, object_gating_events),
        position_continuity_guard_events=position_continuity_guard_events,
        position_continuity_guard_summary=_summarize_position_continuity_guard(
            tracking,
            position_continuity_guard_events,
        ),
        pose_continuity_guard_events=pose_continuity_guard_events,
        pose_continuity_guard_summary=_summarize_pose_continuity_guard(
            tracking,
            pose_continuity_guard_events,
        ),
        rigid_hint_events=rigid_hint_events,
        rigid_hint_summary=_summarize_rigid_hints(rigid_hint_events),
        rigid_hint_pose_events=rigid_hint_pose_events,
        rigid_hint_pose_summary=_summarize_rigid_hint_poses(rigid_hint_pose_events),
        subset_hypothesis_events=subset_hypothesis_events,
        subset_hypothesis_summary=_summarize_subset_hypotheses(subset_hypothesis_events),
        phase45_go_no_go=_single_run_phase45_decision(
            tracking,
            guard_summary,
            enforced=bool(reacquire_guard_enforced),
        ),
        pipeline_variant=pipeline_variant,
        variant_metrics=dict(diagnostics.get("variant_metrics", {})),
        variant_go_no_go={},
        stage_ms_detail=dict(diagnostics.get("stage_ms_detail", {})),
        fallback_summary=dict(diagnostics.get("fallback_summary", {})),
        backpressure_summary=backpressure_summary,
        slow_pair_events=list(diagnostics.get("slow_pair_events", [])),
    )


def summarize_tracking_diagnostics(
    *,
    log_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    min_accepted_points: int = 4,
    jump_threshold_m: float = 0.10,
    flip_threshold_deg: float = 90.0,
    window_padding_s: float = 2.0,
) -> Dict[str, Any]:
    """Summarize live tracking diagnostics already embedded in a tracking log."""
    replay = FrameReplay(str(log_path))
    requested = [str(name) for name in (patterns or [WAIST_PATTERN.name]) if str(name).strip()]
    frame_timestamps = []
    for frame in replay.replay(realtime=False):
        timestamp = _frame_entry_timestamp(frame)
        if timestamp > 0:
            frame_timestamps.append(timestamp)
    first_frame_ts = min(frame_timestamps) if frame_timestamps else None
    last_frame_ts = max(frame_timestamps) if frame_timestamps else None
    diagnostics_events = [
        event for event in replay.events if event.event_type == "tracking_diagnostics"
    ]

    by_rigid: Dict[str, Dict[str, Any]] = {}
    for rigid_name in requested:
        event_summaries: List[Dict[str, Any]] = []
        accepted_points_values: List[float] = []
        reprojection_mean_values: List[float] = []
        epipolar_mean_values: List[float] = []
        rigid_ms_values: List[float] = []
        pipeline_pair_ms_values: List[float] = []
        valid_count = 0
        invalid_count = 0
        previous_max_jump_m = 0.0
        previous_max_flip_deg = 0.0

        for event_index, event in enumerate(diagnostics_events, start=1):
            data = event.data if isinstance(event.data, dict) else {}
            tracking = data.get("tracking", {})
            rigid_status = tracking.get(rigid_name, {}) if isinstance(tracking, dict) else {}
            if not isinstance(rigid_status, dict):
                continue

            geometry = data.get("geometry", {}) if isinstance(data.get("geometry"), dict) else {}
            quality = geometry.get("quality", {}) if isinstance(geometry.get("quality"), dict) else {}
            accepted_points = int(quality.get("accepted_points", 0) or 0)
            accepted_points_values.append(float(accepted_points))

            reprojection_summary = quality.get("reprojection_error_px_summary", {})
            if isinstance(reprojection_summary, dict):
                reprojection_mean_values.append(float(reprojection_summary.get("mean", 0.0) or 0.0))
            epipolar_summary = quality.get("epipolar_error_px_summary", {})
            if isinstance(epipolar_summary, dict):
                epipolar_mean_values.append(float(epipolar_summary.get("mean", 0.0) or 0.0))

            stage_ms = data.get("pipeline_stage_ms", {})
            if isinstance(stage_ms, dict):
                rigid_ms = stage_ms.get("rigid_ms", {})
                if isinstance(rigid_ms, dict):
                    rigid_ms_values.append(float(rigid_ms.get("last", 0.0) or 0.0))
                pipeline_pair_ms = stage_ms.get("pipeline_pair_ms", {})
                if isinstance(pipeline_pair_ms, dict):
                    pipeline_pair_ms_values.append(float(pipeline_pair_ms.get("last", 0.0) or 0.0))

            valid = bool(rigid_status.get("valid", False))
            if valid:
                valid_count += 1
            else:
                invalid_count += 1

            max_jump_m = float(rigid_status.get("max_pose_jump_m", 0.0) or 0.0)
            max_flip_deg = float(rigid_status.get("max_pose_flip_deg", 0.0) or 0.0)
            jump_increase_m = max(0.0, max_jump_m - previous_max_jump_m)
            flip_increase_deg = max(0.0, max_flip_deg - previous_max_flip_deg)
            previous_max_jump_m = max(previous_max_jump_m, max_jump_m)
            previous_max_flip_deg = max(previous_max_flip_deg, max_flip_deg)

            reasons: List[str] = []
            if not valid:
                reasons.append("invalid")
            if accepted_points < int(min_accepted_points):
                reasons.append("low_accepted_points")
            if jump_increase_m > 0.0 and max_jump_m >= float(jump_threshold_m):
                reasons.append("pose_jump")
            if flip_increase_deg > 0.0 and max_flip_deg >= float(flip_threshold_deg):
                reasons.append("pose_flip")

            if reasons:
                event_summaries.append(
                    {
                        "event_index": int(event_index),
                        "timestamp": event.timestamp,
                        "window_start": _iso_with_offset(event.timestamp, -float(window_padding_s)),
                        "window_end": _iso_with_offset(event.timestamp, float(window_padding_s)),
                        "reasons": reasons,
                        "mode": str(rigid_status.get("mode", "")),
                        "valid": valid,
                        "accepted_points": accepted_points,
                        "observed_markers": int(rigid_status.get("observed_markers", 0) or 0),
                        "invalid_reason": str(
                            rigid_status.get("invalid_reason")
                            or rigid_status.get("last_mode_reason")
                            or ""
                        ),
                        "max_pose_jump_m": max_jump_m,
                        "max_pose_flip_deg": max_flip_deg,
                        "jump_increase_m": jump_increase_m,
                        "flip_increase_deg": flip_increase_deg,
                    }
                )

        event_count = len(accepted_points_values)
        by_rigid[rigid_name] = {
            "event_count": int(event_count),
            "valid_count": int(valid_count),
            "invalid_count": int(invalid_count),
            "valid_ratio": float(valid_count / event_count) if event_count else 0.0,
            "accepted_points": _numeric_summary(accepted_points_values),
            "reprojection_mean_px": _numeric_summary(reprojection_mean_values),
            "epipolar_mean_px": _numeric_summary(epipolar_mean_values),
            "rigid_ms": _numeric_summary(rigid_ms_values),
            "pipeline_pair_ms": _numeric_summary(pipeline_pair_ms_values),
            "failure_event_count": int(len(event_summaries)),
            "failure_segments": _merge_failure_segments(event_summaries),
            "failure_events": event_summaries,
        }

    duration_s = (
        float((last_frame_ts - first_frame_ts) / 1_000_000.0)
        if first_frame_ts is not None and last_frame_ts is not None
        else 0.0
    )
    return {
        "log_path": str(log_path),
        "frame_count": int(len(frame_timestamps)),
        "diagnostics_event_count": int(len(diagnostics_events)),
        "duration_s": duration_s,
        "patterns": requested,
        "thresholds": {
            "min_accepted_points": int(min_accepted_points),
            "jump_threshold_m": float(jump_threshold_m),
            "flip_threshold_deg": float(flip_threshold_deg),
            "window_padding_s": float(window_padding_s),
        },
        "by_rigid": by_rigid,
    }


def compare_reacquire_guard_enforcement(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    reacquire_guard_post_reacquire_frames: int = 0,
    reacquire_guard_max_rotation_deg: float = 136.0,
) -> Dict[str, Any]:
    """Run shadow and enforced guard replays and compare Phase 4.5 go/no-go."""
    shadow = replay_tracking_log(
        log_path=log_path,
        calibration_path=calibration_path,
        patterns=patterns,
        rigids_path=rigids_path,
        epipolar_threshold_px=epipolar_threshold_px,
        reacquire_guard_enforced=False,
        reacquire_guard_shadow_enabled=True,
        start_timestamp_us=start_timestamp_us,
        end_timestamp_us=end_timestamp_us,
        start_received_at=start_received_at,
        end_received_at=end_received_at,
        max_frames=max_frames,
        reacquire_guard_post_reacquire_frames=reacquire_guard_post_reacquire_frames,
        reacquire_guard_max_rotation_deg=reacquire_guard_max_rotation_deg,
    ).to_dict()
    enforced = replay_tracking_log(
        log_path=log_path,
        calibration_path=calibration_path,
        patterns=patterns,
        rigids_path=rigids_path,
        epipolar_threshold_px=epipolar_threshold_px,
        reacquire_guard_enforced=True,
        reacquire_guard_shadow_enabled=True,
        start_timestamp_us=start_timestamp_us,
        end_timestamp_us=end_timestamp_us,
        start_received_at=start_received_at,
        end_received_at=end_received_at,
        max_frames=max_frames,
        reacquire_guard_post_reacquire_frames=reacquire_guard_post_reacquire_frames,
        reacquire_guard_max_rotation_deg=reacquire_guard_max_rotation_deg,
    ).to_dict()
    return {
        "shadow": shadow,
        "enforced": enforced,
        "phase45_go_no_go": _compare_phase45_go_no_go(shadow, enforced),
    }


def compare_object_gating_enforcement(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    object_gating_activation_mode: str = "always",
    object_gating_ambiguous_blob_min_separation_px: float = 0.60,
    object_gating_ambiguous_blob_diameter_overlap_ratio: float = 0.30,
    object_gating_ambiguous_marker_assignment_min_margin_px: float = 0.29,
) -> Dict[str, Any]:
    """Run diagnostics-only and enforced object-gating replays side by side."""
    diagnostics_only = replay_tracking_log(
        log_path=log_path,
        calibration_path=calibration_path,
        patterns=patterns,
        rigids_path=rigids_path,
        epipolar_threshold_px=epipolar_threshold_px,
        object_gating_enforced=False,
        object_gating_activation_mode=object_gating_activation_mode,
        object_gating_ambiguous_blob_min_separation_px=(
            object_gating_ambiguous_blob_min_separation_px
        ),
        object_gating_ambiguous_blob_diameter_overlap_ratio=(
            object_gating_ambiguous_blob_diameter_overlap_ratio
        ),
        object_gating_ambiguous_marker_assignment_min_margin_px=(
            object_gating_ambiguous_marker_assignment_min_margin_px
        ),
        start_timestamp_us=start_timestamp_us,
        end_timestamp_us=end_timestamp_us,
        start_received_at=start_received_at,
        end_received_at=end_received_at,
        max_frames=max_frames,
    ).to_dict()
    enforced = replay_tracking_log(
        log_path=log_path,
        calibration_path=calibration_path,
        patterns=patterns,
        rigids_path=rigids_path,
        epipolar_threshold_px=epipolar_threshold_px,
        object_gating_enforced=True,
        object_gating_activation_mode=object_gating_activation_mode,
        object_gating_ambiguous_blob_min_separation_px=(
            object_gating_ambiguous_blob_min_separation_px
        ),
        object_gating_ambiguous_blob_diameter_overlap_ratio=(
            object_gating_ambiguous_blob_diameter_overlap_ratio
        ),
        object_gating_ambiguous_marker_assignment_min_margin_px=(
            object_gating_ambiguous_marker_assignment_min_margin_px
        ),
        start_timestamp_us=start_timestamp_us,
        end_timestamp_us=end_timestamp_us,
        start_received_at=start_received_at,
        end_received_at=end_received_at,
        max_frames=max_frames,
    ).to_dict()
    return {
        "diagnostics_only": diagnostics_only,
        "enforced": enforced,
        "object_gating_go_no_go": _compare_object_gating_go_no_go(
            diagnostics_only,
            enforced,
        ),
    }


def compare_pipeline_variants(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    reacquire_guard_enforced: bool = False,
    reacquire_guard_shadow_enabled: bool = True,
    reacquire_guard_event_logging: bool = False,
    reacquire_guard_post_reacquire_frames: int = 0,
    reacquire_guard_max_rotation_deg: float = 136.0,
    object_gating_enforced: bool = False,
    object_gating_activation_mode: str = "always",
    pose_continuity_guard_enabled: bool = False,
    pose_continuity_guard_enforced: bool = False,
    pose_continuity_max_rotation_deg: float = 90.0,
    pose_continuity_max_angular_velocity_deg_s: float = 2500.0,
    pose_continuity_max_angular_accel_deg_s2: float = 200000.0,
    position_continuity_guard_enabled: bool = False,
    position_continuity_guard_enforced: bool = False,
    position_continuity_max_accel_m_s2: float = 60.0,
    position_continuity_max_velocity_m_s: float = 8.0,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    subset_diagnostics_mode: Optional[str] = None,
) -> Dict[str, Any]:
    """Replay baseline and all fast variants on identical inputs."""
    summaries: Dict[str, Dict[str, Any]] = {}
    for variant in PIPELINE_VARIANTS:
        mode = subset_diagnostics_mode or default_subset_diagnostics_mode_for_variant(variant)
        summary = replay_tracking_log(
            log_path=log_path,
            calibration_path=calibration_path,
            patterns=patterns,
            rigids_path=rigids_path,
            epipolar_threshold_px=epipolar_threshold_px,
            reacquire_guard_enforced=reacquire_guard_enforced,
            reacquire_guard_shadow_enabled=reacquire_guard_shadow_enabled,
            reacquire_guard_event_logging=reacquire_guard_event_logging,
            reacquire_guard_post_reacquire_frames=reacquire_guard_post_reacquire_frames,
            reacquire_guard_max_rotation_deg=reacquire_guard_max_rotation_deg,
            object_gating_enforced=object_gating_enforced,
            object_gating_activation_mode=object_gating_activation_mode,
            pose_continuity_guard_enabled=pose_continuity_guard_enabled,
            pose_continuity_guard_enforced=pose_continuity_guard_enforced,
            pose_continuity_max_rotation_deg=pose_continuity_max_rotation_deg,
            pose_continuity_max_angular_velocity_deg_s=pose_continuity_max_angular_velocity_deg_s,
            pose_continuity_max_angular_accel_deg_s2=pose_continuity_max_angular_accel_deg_s2,
            position_continuity_guard_enabled=position_continuity_guard_enabled,
            position_continuity_guard_enforced=position_continuity_guard_enforced,
            position_continuity_max_accel_m_s2=position_continuity_max_accel_m_s2,
            position_continuity_max_velocity_m_s=position_continuity_max_velocity_m_s,
            start_timestamp_us=start_timestamp_us,
            end_timestamp_us=end_timestamp_us,
            start_received_at=start_received_at,
            end_received_at=end_received_at,
            max_frames=max_frames,
            pipeline_variant=variant,
            subset_diagnostics_mode=mode,
        ).to_dict()
        summaries[variant] = summary

    baseline = summaries["baseline"]
    variants: Dict[str, Dict[str, Any]] = {}
    replacement_candidates: List[str] = []
    for variant in PIPELINE_VARIANTS:
        if variant == "baseline":
            continue
        candidate = dict(summaries[variant])
        decision = _compare_pipeline_variant_go_no_go(baseline, candidate)
        candidate["variant_go_no_go"] = decision
        variants[variant] = candidate
        if decision.get("replacement_candidate"):
            replacement_candidates.append(variant)

    return {
        "log_path": str(log_path),
        "calibration_path": str(calibration_path),
        "patterns": list(patterns or [WAIST_PATTERN.name]),
        "baseline": baseline,
        "variants": variants,
        "ranked_replacement_candidates": replacement_candidates,
    }


def compare_epipolar_pruning(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    reacquire_guard_enforced: bool = False,
    reacquire_guard_shadow_enabled: bool = True,
    reacquire_guard_event_logging: bool = False,
    reacquire_guard_post_reacquire_frames: int = 0,
    reacquire_guard_max_rotation_deg: float = 136.0,
    object_gating_enforced: bool = False,
    object_gating_activation_mode: str = "always",
    pose_continuity_guard_enabled: bool = False,
    pose_continuity_guard_enforced: bool = False,
    pose_continuity_max_rotation_deg: float = 90.0,
    pose_continuity_max_angular_velocity_deg_s: float = 2500.0,
    pose_continuity_max_angular_accel_deg_s2: float = 200000.0,
    position_continuity_guard_enabled: bool = False,
    position_continuity_guard_enforced: bool = False,
    position_continuity_max_accel_m_s2: float = 60.0,
    position_continuity_max_velocity_m_s: float = 8.0,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    subset_diagnostics_mode: Optional[str] = None,
) -> Dict[str, Any]:
    """Compare official fast_ABCD against fast_ABCDP epipolar pruning."""
    common = {
        "log_path": log_path,
        "calibration_path": calibration_path,
        "patterns": patterns,
        "rigids_path": rigids_path,
        "epipolar_threshold_px": epipolar_threshold_px,
        "reacquire_guard_enforced": reacquire_guard_enforced,
        "reacquire_guard_shadow_enabled": reacquire_guard_shadow_enabled,
        "reacquire_guard_event_logging": reacquire_guard_event_logging,
        "reacquire_guard_post_reacquire_frames": reacquire_guard_post_reacquire_frames,
        "reacquire_guard_max_rotation_deg": reacquire_guard_max_rotation_deg,
        "object_gating_enforced": object_gating_enforced,
        "object_gating_activation_mode": object_gating_activation_mode,
        "pose_continuity_guard_enabled": pose_continuity_guard_enabled,
        "pose_continuity_guard_enforced": pose_continuity_guard_enforced,
        "pose_continuity_max_rotation_deg": pose_continuity_max_rotation_deg,
        "pose_continuity_max_angular_velocity_deg_s": pose_continuity_max_angular_velocity_deg_s,
        "pose_continuity_max_angular_accel_deg_s2": pose_continuity_max_angular_accel_deg_s2,
        "position_continuity_guard_enabled": position_continuity_guard_enabled,
        "position_continuity_guard_enforced": position_continuity_guard_enforced,
        "position_continuity_max_accel_m_s2": position_continuity_max_accel_m_s2,
        "position_continuity_max_velocity_m_s": position_continuity_max_velocity_m_s,
        "start_timestamp_us": start_timestamp_us,
        "end_timestamp_us": end_timestamp_us,
        "start_received_at": start_received_at,
        "end_received_at": end_received_at,
        "max_frames": max_frames,
    }
    baseline = replay_tracking_log(
        **common,
        pipeline_variant="fast_ABCD",
        subset_diagnostics_mode=subset_diagnostics_mode
        or default_subset_diagnostics_mode_for_variant("fast_ABCD"),
    ).to_dict()
    candidate = replay_tracking_log(
        **common,
        pipeline_variant="fast_ABCDP",
        subset_diagnostics_mode=subset_diagnostics_mode
        or default_subset_diagnostics_mode_for_variant("fast_ABCDP"),
    ).to_dict()
    return {
        "log_path": str(log_path),
        "calibration_path": str(calibration_path),
        "patterns": list(patterns or [WAIST_PATTERN.name]),
        "baseline": baseline,
        "fast_ABCDP": candidate,
        "epipolar_pruning_go_no_go": _compare_epipolar_pruning_go_no_go(
            baseline,
            candidate,
        ),
    }


PERFORMANCE_UPGRADE_VARIANTS = (
    "fast_ABCD",
    "fast_ABCDS",
    "fast_ABCDG",
    "fast_ABCDF",
    "fast_ABCDH",
    "fast_ABCDHF",
    "fast_ABCDHR",
    "fast_ABCDHRF",
    "fast_ABCDR",
    "fast_ABCDX",
)


def compare_performance_upgrades(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
    reacquire_guard_enforced: bool = False,
    reacquire_guard_shadow_enabled: bool = True,
    reacquire_guard_event_logging: bool = False,
    reacquire_guard_post_reacquire_frames: int = 0,
    reacquire_guard_max_rotation_deg: float = 136.0,
    object_gating_enforced: bool = False,
    object_gating_activation_mode: str = "always",
    pose_continuity_guard_enabled: bool = False,
    pose_continuity_guard_enforced: bool = False,
    pose_continuity_max_rotation_deg: float = 90.0,
    pose_continuity_max_angular_velocity_deg_s: float = 2500.0,
    pose_continuity_max_angular_accel_deg_s2: float = 200000.0,
    position_continuity_guard_enabled: bool = False,
    position_continuity_guard_enforced: bool = False,
    position_continuity_max_accel_m_s2: float = 60.0,
    position_continuity_max_velocity_m_s: float = 8.0,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
    subset_diagnostics_mode: Optional[str] = None,
) -> Dict[str, Any]:
    """Compare performance-upgrade variants against the official fast_ABCD path."""
    common = {
        "log_path": log_path,
        "calibration_path": calibration_path,
        "patterns": patterns,
        "rigids_path": rigids_path,
        "epipolar_threshold_px": epipolar_threshold_px,
        "reacquire_guard_enforced": reacquire_guard_enforced,
        "reacquire_guard_shadow_enabled": reacquire_guard_shadow_enabled,
        "reacquire_guard_event_logging": reacquire_guard_event_logging,
        "reacquire_guard_post_reacquire_frames": reacquire_guard_post_reacquire_frames,
        "reacquire_guard_max_rotation_deg": reacquire_guard_max_rotation_deg,
        "object_gating_enforced": object_gating_enforced,
        "object_gating_activation_mode": object_gating_activation_mode,
        "pose_continuity_guard_enabled": pose_continuity_guard_enabled,
        "pose_continuity_guard_enforced": pose_continuity_guard_enforced,
        "pose_continuity_max_rotation_deg": pose_continuity_max_rotation_deg,
        "pose_continuity_max_angular_velocity_deg_s": pose_continuity_max_angular_velocity_deg_s,
        "pose_continuity_max_angular_accel_deg_s2": pose_continuity_max_angular_accel_deg_s2,
        "position_continuity_guard_enabled": position_continuity_guard_enabled,
        "position_continuity_guard_enforced": position_continuity_guard_enforced,
        "position_continuity_max_accel_m_s2": position_continuity_max_accel_m_s2,
        "position_continuity_max_velocity_m_s": position_continuity_max_velocity_m_s,
        "start_timestamp_us": start_timestamp_us,
        "end_timestamp_us": end_timestamp_us,
        "start_received_at": start_received_at,
        "end_received_at": end_received_at,
        "max_frames": max_frames,
    }
    summaries: Dict[str, Dict[str, Any]] = {}
    for variant in PERFORMANCE_UPGRADE_VARIANTS:
        summaries[variant] = replay_tracking_log(
            **common,
            pipeline_variant=variant,
            subset_diagnostics_mode=subset_diagnostics_mode
            or default_subset_diagnostics_mode_for_variant(variant),
        ).to_dict()

    baseline = summaries["fast_ABCD"]
    variants: Dict[str, Dict[str, Any]] = {}
    adopted: List[str] = []
    for variant in PERFORMANCE_UPGRADE_VARIANTS:
        if variant == "fast_ABCD":
            continue
        candidate = dict(summaries[variant])
        decision = _compare_pipeline_variant_go_no_go(baseline, candidate)
        candidate["variant_go_no_go"] = decision
        variants[variant] = candidate
        if decision.get("replacement_candidate"):
            adopted.append(variant)

    return {
        "log_path": str(log_path),
        "calibration_path": str(calibration_path),
        "patterns": list(patterns or [WAIST_PATTERN.name]),
        "baseline": baseline,
        "variants": variants,
        "ranked_replacement_candidates": adopted,
    }


def _stage_p95(summary: Dict[str, Any], name: str) -> float:
    stage = summary.get("pipeline_stage_ms", {}).get(name, {})
    if not isinstance(stage, dict):
        return 0.0
    try:
        return float(stage.get("p95", 0.0))
    except Exception:
        return 0.0


def _reprojection_p95(summary: Dict[str, Any]) -> float:
    quality = summary.get("triangulation_quality", {})
    if not isinstance(quality, dict):
        quality = summary.get("geometry", {}).get("quality", {})
    if not isinstance(quality, dict):
        return 0.0
    reprojection = quality.get("reprojection_error_px_summary", {})
    if not isinstance(reprojection, dict):
        return 0.0
    try:
        return float(reprojection.get("p95", reprojection.get("max", 0.0)) or 0.0)
    except Exception:
        return 0.0


def _compare_pipeline_variant_go_no_go(
    baseline: Dict[str, Any],
    candidate: Dict[str, Any],
) -> Dict[str, Any]:
    variant = str(candidate.get("pipeline_variant", ""))
    baseline_valid = int(baseline.get("poses_estimated", 0) or 0)
    candidate_valid = int(candidate.get("poses_estimated", 0) or 0)
    baseline_reacquire = _tracking_metric(baseline, "reacquire_count")
    candidate_reacquire = _tracking_metric(candidate, "reacquire_count")
    baseline_jumps = _tracking_metric(baseline, "pose_jump_count")
    candidate_jumps = _tracking_metric(candidate, "pose_jump_count")
    baseline_flips = _tracking_metric(baseline, "max_pose_flip_deg")
    candidate_flips = _tracking_metric(candidate, "max_pose_flip_deg")
    baseline_transitions = _tracking_metric(baseline, "mode_transition_count")
    candidate_transitions = _tracking_metric(candidate, "mode_transition_count")
    baseline_reprojection = _reprojection_p95(baseline)
    candidate_reprojection = _reprojection_p95(candidate)
    reprojection_limit = baseline_reprojection * 1.05 if baseline_reprojection > 0.0 else candidate_reprojection + 1e-9

    quality_ok = bool(
        candidate_valid >= baseline_valid
        and candidate_reacquire <= baseline_reacquire + 1e-9
        and candidate_jumps <= baseline_jumps + 1e-9
        and candidate_flips <= baseline_flips + 1e-9
        and candidate_transitions <= baseline_transitions + 1e-9
        and candidate_reprojection <= reprojection_limit + 1e-9
    )
    baseline_pipeline = _stage_p95(baseline, "pipeline_pair_ms")
    candidate_pipeline = _stage_p95(candidate, "pipeline_pair_ms")
    baseline_rigid = _stage_p95(baseline, "rigid_ms")
    candidate_rigid = _stage_p95(candidate, "rigid_ms")
    speed_ok = bool(
        baseline_pipeline > 0.0
        and baseline_rigid > 0.0
        and candidate_pipeline < baseline_pipeline
        and candidate_rigid < baseline_rigid
    )
    dropped = int(candidate.get("backpressure_summary", {}).get("dropped_pair_count", 0) or 0)
    drop_ok = variant != "fast_ABCDE" or dropped == 0
    replacement_candidate = bool(quality_ok and speed_ok and drop_ok)
    return {
        "decision": "replacement_candidate" if replacement_candidate else "keep_experimental",
        "replacement_candidate": replacement_candidate,
        "quality_ok": quality_ok,
        "speed_ok": speed_ok,
        "drop_ok": drop_ok,
        "criteria": {
            "valid_frames_non_decreasing": candidate_valid >= baseline_valid,
            "reacquire_non_increasing": candidate_reacquire <= baseline_reacquire + 1e-9,
            "pose_jumps_non_increasing": candidate_jumps <= baseline_jumps + 1e-9,
            "max_flip_non_increasing": candidate_flips <= baseline_flips + 1e-9,
            "mode_transitions_non_increasing": candidate_transitions <= baseline_transitions + 1e-9,
            "reprojection_p95_within_5_percent": candidate_reprojection <= reprojection_limit + 1e-9,
            "pipeline_pair_p95_improved": candidate_pipeline < baseline_pipeline,
            "rigid_p95_improved": candidate_rigid < baseline_rigid,
            "fast_abcde_no_drops": drop_ok,
        },
        "baseline": {
            "valid_frames": baseline_valid,
            "pipeline_pair_ms_p95": baseline_pipeline,
            "rigid_ms_p95": baseline_rigid,
            "reprojection_p95_px": baseline_reprojection,
        },
        "candidate": {
            "valid_frames": candidate_valid,
            "pipeline_pair_ms_p95": candidate_pipeline,
            "rigid_ms_p95": candidate_rigid,
            "reprojection_p95_px": candidate_reprojection,
            "dropped_pair_count": dropped,
        },
    }


def _assignment_diagnostic(summary: Dict[str, Any], key: str) -> float:
    assignment = (
        summary.get("geometry", {})
        .get("quality", {})
        .get("assignment_diagnostics", {})
    )
    if not isinstance(assignment, dict):
        return 0.0
    try:
        return float(assignment.get(key, 0.0) or 0.0)
    except Exception:
        return 0.0


def _accepted_points(summary: Dict[str, Any]) -> int:
    try:
        return int(summary.get("geometry", {}).get("quality", {}).get("accepted_points", 0) or 0)
    except Exception:
        return 0


def _compare_epipolar_pruning_go_no_go(
    baseline: Dict[str, Any],
    candidate: Dict[str, Any],
) -> Dict[str, Any]:
    exact_reprojection = abs(_reprojection_p95(candidate) - _reprojection_p95(baseline)) <= 1e-9
    exact_tracking = all(
        _tracking_metric(candidate, key) == _tracking_metric(baseline, key)
        for key in (
            "reacquire_count",
            "pose_jump_count",
            "mode_transition_count",
            "max_pose_flip_deg",
        )
    )
    exact_assignment = all(
        _assignment_diagnostic(candidate, key) == _assignment_diagnostic(baseline, key)
        for key in ("assignment_matches", "duplicate_blob_matches")
    )
    exact_quality = bool(
        int(candidate.get("poses_estimated", 0) or 0) == int(baseline.get("poses_estimated", 0) or 0)
        and exact_tracking
        and exact_reprojection
        and _accepted_points(candidate) == _accepted_points(baseline)
        and exact_assignment
    )
    pipeline_p95_ok = _stage_p95(candidate, "pipeline_pair_ms") <= _stage_p95(baseline, "pipeline_pair_ms") + 1e-9
    generic_p95_ok = _stage_p95(candidate, "generic_triangulation_ms") <= _stage_p95(baseline, "generic_triangulation_ms") + 1e-9
    replacement_candidate = bool(exact_quality and pipeline_p95_ok and generic_p95_ok)
    pruning_summary = (
        candidate.get("geometry", {})
        .get("quality", {})
        .get("assignment_diagnostics", {})
        .get("epipolar_pruning_summary", {})
    )
    if not isinstance(pruning_summary, dict):
        pruning_summary = {}
    return {
        "decision": "candidate_for_stress_replay" if replacement_candidate else "keep_experimental",
        "replacement_candidate": replacement_candidate,
        "exact_quality": exact_quality,
        "speed_non_regression": bool(pipeline_p95_ok and generic_p95_ok),
        "criteria": {
            "poses_estimated_equal": int(candidate.get("poses_estimated", 0) or 0) == int(baseline.get("poses_estimated", 0) or 0),
            "tracking_metrics_equal": exact_tracking,
            "accepted_points_equal": _accepted_points(candidate) == _accepted_points(baseline),
            "assignment_metrics_equal": exact_assignment,
            "reprojection_p95_equal": exact_reprojection,
            "pipeline_pair_p95_non_regression": pipeline_p95_ok,
            "generic_triangulation_p95_non_regression": generic_p95_ok,
        },
        "baseline": {
            "pipeline_pair_ms_p95": _stage_p95(baseline, "pipeline_pair_ms"),
            "generic_triangulation_ms_p95": _stage_p95(baseline, "generic_triangulation_ms"),
            "reprojection_p95_px": _reprojection_p95(baseline),
        },
        "candidate": {
            "pipeline_pair_ms_p95": _stage_p95(candidate, "pipeline_pair_ms"),
            "generic_triangulation_ms_p95": _stage_p95(candidate, "generic_triangulation_ms"),
            "reprojection_p95_px": _reprojection_p95(candidate),
            "epipolar_pruning_summary": dict(pruning_summary),
        },
    }


def _simulate_backpressure(
    received_times_s: List[float],
    *,
    pair_count: int,
    pipeline_stage_ms: Dict[str, Any],
    enabled: bool,
) -> Dict[str, Any]:
    if not enabled:
        return {"enabled": False, "dropped_pair_count": 0, "max_queue_depth": 0, "simulated_output_latency_ms": {}}
    pair_arrivals = list(received_times_s[1::2])[: max(0, int(pair_count))]
    duration_ms = float(
        pipeline_stage_ms.get("pipeline_pair_ms", {}).get(
            "mean",
            pipeline_stage_ms.get("pipeline_pair_ms", {}).get("p95", 0.0),
        )
        or 0.0
    )
    duration_s = max(0.0, duration_ms / 1000.0)
    available_at = 0.0
    dropped = 0
    latencies_ms: List[float] = []
    for arrival in pair_arrivals:
        if arrival < available_at:
            dropped += 1
            continue
        available_at = float(arrival) + duration_s
        latencies_ms.append(duration_ms)
    return {
        "enabled": True,
        "dropped_pair_count": int(dropped),
        "max_queue_depth": 1 if pair_arrivals else 0,
        "simulated_output_latency_ms": _numeric_summary(latencies_ms),
    }


def _frame_from_log_entry(entry: Any) -> Frame:
    data = dict(entry.data)
    host_received_at_us = data.get("host_received_at_us")
    if host_received_at_us is None:
        host_received_at_us = int(_timestamp_to_seconds(entry.received_at) * 1_000_000)
    host_received_at_us = int(host_received_at_us)
    frame_index = data.get("frame_index")
    return Frame(
        camera_id=str(data["camera_id"]),
        timestamp=int(data["timestamp"]),
        frame_index=int(frame_index) if frame_index is not None else None,
        blobs=list(data.get("blobs", [])),
        received_at=float(host_received_at_us) / 1_000_000.0,
        host_received_at_us=host_received_at_us,
        timestamp_source=str(data["timestamp_source"]) if data.get("timestamp_source") is not None else None,
        sensor_timestamp_ns=int(data["sensor_timestamp_ns"]) if data.get("sensor_timestamp_ns") is not None else None,
        sensor_to_dequeue_ms=float(data["sensor_to_dequeue_ms"]) if data.get("sensor_to_dequeue_ms") is not None else None,
        sensor_timestamp_stale=bool(data.get("sensor_timestamp_stale", False)),
        capture_to_process_ms=float(data["capture_to_process_ms"]) if data.get("capture_to_process_ms") is not None else None,
        capture_to_send_ms=float(data["capture_to_send_ms"]) if data.get("capture_to_send_ms") is not None else None,
    )


def _summarize_reacquire_guard(
    tracking: Dict[str, Dict[str, Any]],
    events: List[Dict[str, Any]],
) -> Dict[str, Any]:
    reason_counts: Dict[str, int] = {}
    for event in events:
        reason = str(event.get("reason") or "")
        for part in reason.split(","):
            item = part.strip()
            if not item:
                continue
            reason_counts[item] = reason_counts.get(item, 0) + 1

    by_rigid: Dict[str, Dict[str, Any]] = {}
    for name, status in tracking.items():
        guard = status.get("reacquire_guard", {}) if isinstance(status, dict) else {}
        by_rigid[str(name)] = {
            "enabled": bool(guard.get("enabled", False)),
            "enforced": bool(guard.get("enforced", False)),
            "evaluated_count": int(guard.get("evaluated_count", 0)),
            "would_reject_count": int(guard.get("would_reject_count", 0)),
            "rejected_count": int(guard.get("rejected_count", 0)),
        }

    return {
        "event_count": int(len(events)),
        "would_reject_event_count": int(sum(1 for event in events if event.get("would_reject"))),
        "enforced_reject_event_count": int(
            sum(1 for event in events if event.get("enforced") and event.get("would_reject"))
        ),
        "reason_counts": reason_counts,
        "by_rigid": by_rigid,
    }


def _summarize_object_gating(
    tracking: Dict[str, Dict[str, Any]],
    events: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    by_rigid: Dict[str, Any] = {}
    totals = {
        "evaluated_count": 0,
        "enforced_count": 0,
        "diagnostics_only_count": 0,
        "assigned_marker_views": 0,
        "candidate_window_count": 0,
        "markers_with_two_or_more_rays": 0,
        "single_ray_candidates": 0,
        "generic_fallback_blob_count": 0,
    }
    for name, status in tracking.items():
        gating = status.get("object_gating", {}) if isinstance(status, dict) else {}
        if not isinstance(gating, dict):
            continue
        evaluated = bool(gating.get("evaluated", False))
        summary = {
            "enabled": bool(gating.get("enabled", False)),
            "enforced": bool(gating.get("enforced", False)),
            "diagnostics_only": bool(gating.get("diagnostics_only", True)),
            "evaluated": evaluated,
            "reason": str(gating.get("reason", "")),
            "mode": str(gating.get("mode", "")),
            "confidence": float(gating.get("confidence", 0.0)),
            "pixel_gate_px": float(gating.get("pixel_gate_px", 0.0)),
            "assigned_marker_views": int(gating.get("assigned_marker_views", 0)),
            "candidate_window_count": int(gating.get("candidate_window_count", 0)),
            "markers_with_two_or_more_rays": int(gating.get("markers_with_two_or_more_rays", 0)),
            "single_ray_candidates": int(gating.get("single_ray_candidates", 0)),
            "generic_fallback_blob_count": int(gating.get("generic_fallback_blob_count", 0)),
        }
        by_rigid[str(name)] = summary
        if evaluated:
            totals["evaluated_count"] += 1
        if summary["enforced"]:
            totals["enforced_count"] += 1
        if summary["diagnostics_only"]:
            totals["diagnostics_only_count"] += 1
        for key in totals:
            if key in {"evaluated_count", "enforced_count", "diagnostics_only_count"}:
                continue
            totals[key] += int(summary.get(key, 0))
    event_list = list(events or [])
    event_totals = {
        "event_count": int(len(event_list)),
        "enforced_count": int(sum(1 for event in event_list if event.get("enforced"))),
        "diagnostics_only_count": int(
            sum(1 for event in event_list if event.get("diagnostics_only", True))
        ),
        "assigned_marker_views": int(sum(int(event.get("assigned_marker_views", 0)) for event in event_list)),
        "candidate_window_count": int(sum(int(event.get("candidate_window_count", 0)) for event in event_list)),
        "markers_with_two_or_more_rays": int(sum(int(event.get("markers_with_two_or_more_rays", 0)) for event in event_list)),
        "single_ray_candidates": int(sum(int(event.get("single_ray_candidates", 0)) for event in event_list)),
        "generic_fallback_blob_count": int(sum(int(event.get("generic_fallback_blob_count", 0)) for event in event_list)),
    }
    return {"by_rigid": by_rigid, "totals": totals, "event_totals": event_totals}


def _summarize_pose_continuity_guard(
    tracking: Dict[str, Dict[str, Any]],
    events: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    by_rigid: Dict[str, Any] = {}
    for name, status in tracking.items():
        guard = status.get("pose_continuity_guard", {}) if isinstance(status, dict) else {}
        if not isinstance(guard, dict):
            continue
        by_rigid[str(name)] = {
            "enabled": bool(guard.get("enabled", False)),
            "enforced": bool(guard.get("enforced", False)),
            "evaluated_count": int(guard.get("evaluated_count", 0)),
            "would_reject_count": int(guard.get("would_reject_count", 0)),
            "held_count": int(guard.get("held_count", 0)),
            "last_reason": str(guard.get("reason", "")),
        }

    event_list = list(events or [])
    reason_counts: Dict[str, int] = {}
    for event in event_list:
        reason = str(event.get("reason", ""))
        for part in reason.split(","):
            item = part.strip()
            if item:
                reason_counts[item] = int(reason_counts.get(item, 0)) + 1

    return {
        "by_rigid": by_rigid,
        "event_totals": {
            "event_count": int(len(event_list)),
            "would_reject_count": int(sum(1 for event in event_list if event.get("would_reject"))),
            "held_count": int(
                sum(
                    1
                    for event in event_list
                    if event.get("held_prediction") or event.get("held_rotation")
                )
            ),
            "occluded_count": int(sum(1 for event in event_list if event.get("occluded"))),
            "reason_counts": reason_counts,
            "position_innovation_m": _numeric_summary(
                [float(event.get("position_innovation_m", 0.0)) for event in event_list]
            ),
            "rotation_innovation_deg": _numeric_summary(
                [float(event.get("rotation_innovation_deg", 0.0)) for event in event_list]
            ),
            "angular_velocity_deg_s": _numeric_summary(
                [float(event.get("angular_velocity_deg_s", 0.0)) for event in event_list]
            ),
            "angular_accel_deg_s2": _numeric_summary(
                [float(event.get("angular_accel_deg_s2", 0.0)) for event in event_list]
            ),
        },
    }


def _summarize_position_continuity_guard(
    tracking: Dict[str, Dict[str, Any]],
    events: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    by_rigid: Dict[str, Any] = {}
    for name, status in tracking.items():
        guard = status.get("position_continuity_guard", {}) if isinstance(status, dict) else {}
        if not isinstance(guard, dict):
            continue
        by_rigid[str(name)] = {
            "enabled": bool(guard.get("enabled", False)),
            "enforced": bool(guard.get("enforced", False)),
            "evaluated_count": int(guard.get("evaluated_count", 0)),
            "would_reject_count": int(guard.get("would_reject_count", 0)),
            "clamped_count": int(guard.get("clamped_count", 0)),
            "last_reason": str(guard.get("reason", "")),
        }

    event_list = list(events or [])
    reason_counts: Dict[str, int] = {}
    for event in event_list:
        reason = str(event.get("reason", ""))
        for part in reason.split(","):
            item = part.strip()
            if item:
                reason_counts[item] = int(reason_counts.get(item, 0)) + 1

    return {
        "by_rigid": by_rigid,
        "event_totals": {
            "event_count": int(len(event_list)),
            "would_reject_count": int(sum(1 for event in event_list if event.get("would_reject"))),
            "clamped_count": int(
                sum(1 for event in event_list if event.get("clamped_position"))
            ),
            "occluded_count": int(sum(1 for event in event_list if event.get("occluded"))),
            "reason_counts": reason_counts,
            "position_innovation_m": _numeric_summary(
                [float(event.get("position_innovation_m", 0.0)) for event in event_list]
            ),
            "position_velocity_m_s": _numeric_summary(
                [float(event.get("position_velocity_m_s", 0.0)) for event in event_list]
            ),
            "previous_velocity_m_s": _numeric_summary(
                [float(event.get("previous_velocity_m_s", 0.0)) for event in event_list]
            ),
            "position_accel_m_s2": _numeric_summary(
                [float(event.get("position_accel_m_s2", 0.0)) for event in event_list]
            ),
            "limited_velocity_m_s": _numeric_summary(
                [float(event.get("limited_velocity_m_s", 0.0)) for event in event_list]
            ),
        },
    }


def _summarize_rigid_hints(events: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    event_list = list(events or [])
    by_rigid: Dict[str, Dict[str, Any]] = {}
    reprojection_means: List[float] = []
    reprojection_p95s: List[float] = []
    for event in event_list:
        rigid_name = str(event.get("rigid_name", ""))
        bucket = by_rigid.setdefault(
            rigid_name,
            {
                "event_count": 0,
                "candidate_markers": 0,
                "markers_with_two_or_more_rays": 0,
                "single_ray_candidates": 0,
                "accepted_points": 0,
                "rejected_markers": 0,
                "invalid_assignments": 0,
            },
        )
        bucket["event_count"] += 1
        for key in (
            "candidate_markers",
            "markers_with_two_or_more_rays",
            "single_ray_candidates",
            "accepted_points",
            "rejected_markers",
            "invalid_assignments",
        ):
            bucket[key] += int(event.get(key, 0))
        reprojection_means.append(float(event.get("reprojection_mean_px", 0.0)))
        reprojection_p95s.append(float(event.get("reprojection_p95_px", 0.0)))

    totals = {
        "event_count": int(len(event_list)),
        "candidate_markers": int(sum(int(event.get("candidate_markers", 0)) for event in event_list)),
        "markers_with_two_or_more_rays": int(
            sum(int(event.get("markers_with_two_or_more_rays", 0)) for event in event_list)
        ),
        "single_ray_candidates": int(
            sum(int(event.get("single_ray_candidates", 0)) for event in event_list)
        ),
        "accepted_points": int(sum(int(event.get("accepted_points", 0)) for event in event_list)),
        "rejected_markers": int(sum(int(event.get("rejected_markers", 0)) for event in event_list)),
        "invalid_assignments": int(sum(int(event.get("invalid_assignments", 0)) for event in event_list)),
        "reprojection_mean_px_summary": _numeric_summary(reprojection_means),
        "reprojection_p95_px_summary": _numeric_summary(reprojection_p95s),
    }
    return {"by_rigid": by_rigid, "totals": totals}


def _summarize_rigid_hint_poses(events: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    event_list = list(events or [])
    by_rigid: Dict[str, Dict[str, Any]] = {}
    score_deltas: List[float] = []
    position_deltas: List[float] = []
    rotation_deltas: List[float] = []
    p95_errors: List[float] = []
    for event in event_list:
        rigid_name = str(event.get("rigid_name", ""))
        bucket = by_rigid.setdefault(
            rigid_name,
            {
                "event_count": 0,
                "valid_count": 0,
                "selected_for_pose_count": 0,
                "generic_valid_count": 0,
                "would_improve_score_count": 0,
                "candidate_points": 0,
                "observed_markers": 0,
                "real_ray_count": 0,
                "virtual_marker_count": 0,
                "reason_counts": {},
            },
        )
        bucket["event_count"] += 1
        if event.get("valid"):
            bucket["valid_count"] += 1
        if event.get("selected_for_pose"):
            bucket["selected_for_pose_count"] += 1
        if event.get("generic_valid"):
            bucket["generic_valid_count"] += 1
        if event.get("would_improve_score"):
            bucket["would_improve_score_count"] += 1
        for key in (
            "candidate_points",
            "observed_markers",
            "real_ray_count",
            "virtual_marker_count",
        ):
            bucket[key] += int(event.get(key, 0))
        reason = str(event.get("reason", ""))
        reasons = bucket["reason_counts"]
        reasons[reason] = int(reasons.get(reason, 0)) + 1
        score_deltas.append(float(event.get("score_delta", 0.0)))
        position_deltas.append(float(event.get("position_delta_m", 0.0)))
        rotation_deltas.append(float(event.get("rotation_delta_deg", 0.0)))
        p95_errors.append(float(event.get("p95_error_px", 0.0)))

    totals = {
        "event_count": int(len(event_list)),
        "valid_count": int(sum(1 for event in event_list if event.get("valid"))),
        "selected_for_pose_count": int(
            sum(1 for event in event_list if event.get("selected_for_pose"))
        ),
        "generic_valid_count": int(
            sum(1 for event in event_list if event.get("generic_valid"))
        ),
        "would_improve_score_count": int(
            sum(1 for event in event_list if event.get("would_improve_score"))
        ),
        "score_delta_summary": _numeric_summary(score_deltas),
        "position_delta_m_summary": _numeric_summary(position_deltas),
        "rotation_delta_deg_summary": _numeric_summary(rotation_deltas),
        "p95_error_px_summary": _numeric_summary(p95_errors),
    }
    totals["hint_pose_adoption_ready"] = bool(
        totals["event_count"] > 0
        and totals["valid_count"] >= totals["generic_valid_count"]
        and float(totals["rotation_delta_deg_summary"]["p95"]) < 5.0
        and float(totals["p95_error_px_summary"]["p95"]) <= 2.0
    )
    totals["phase6_ready"] = bool(
        totals["event_count"] > 0
        and (totals["valid_count"] > 0 or totals["generic_valid_count"] > 0)
        and totals["p95_error_px_summary"]["count"] > 0
    )
    return {"by_rigid": by_rigid, "totals": totals}


def _summarize_subset_hypotheses(events: Optional[List[Dict[str, Any]]] = None) -> Dict[str, Any]:
    event_list = list(events or [])
    by_rigid: Dict[str, Dict[str, Any]] = {}
    margins: List[float] = []
    combined_margins: List[float] = []
    score_deltas: List[float] = []
    best_p95_errors: List[float] = []
    best_rotation_deltas: List[float] = []
    for event in event_list:
        rigid_name = str(event.get("rigid_name", ""))
        bucket = by_rigid.setdefault(
            rigid_name,
            {
                "event_count": 0,
                "candidate_count": 0,
                "pruned_candidate_count": 0,
                "valid_candidate_count": 0,
                "rejected_by_ambiguity": 0,
                "rejected_by_2d_score": 0,
                "rejected_by_rms": 0,
                "flip_risk_count": 0,
                "subset_adoption_ready_count": 0,
                "truncated_count": 0,
                "best_source_counts": {},
            },
        )
        bucket["event_count"] += 1
        for key in (
            "candidate_count",
            "pruned_candidate_count",
            "valid_candidate_count",
            "rejected_by_ambiguity",
            "rejected_by_2d_score",
            "rejected_by_rms",
            "flip_risk_count",
        ):
            bucket[key] += int(event.get(key, 0))
        if event.get("subset_adoption_ready"):
            bucket["subset_adoption_ready_count"] += 1
        if event.get("truncated"):
            bucket["truncated_count"] += 1
        source = str(event.get("best_source", ""))
        source_counts = bucket["best_source_counts"]
        source_counts[source] = int(source_counts.get(source, 0)) + 1
        margins.append(float(event.get("margin", 0.0)))
        combined_margins.append(float(event.get("combined_margin", 0.0)))
        score_deltas.append(float(event.get("score_delta", 0.0)))
        best_p95_errors.append(float(event.get("best_p95_error_px", 0.0)))
        best_rotation_deltas.append(float(event.get("best_rotation_delta_deg", 0.0)))

    adoption_ready_count = int(
        sum(1 for event in event_list if event.get("subset_adoption_ready"))
    )
    totals = {
        "event_count": int(len(event_list)),
        "candidate_count": int(sum(int(event.get("candidate_count", 0)) for event in event_list)),
        "pruned_candidate_count": int(
            sum(int(event.get("pruned_candidate_count", 0)) for event in event_list)
        ),
        "valid_candidate_count": int(
            sum(int(event.get("valid_candidate_count", 0)) for event in event_list)
        ),
        "rejected_by_ambiguity": int(
            sum(int(event.get("rejected_by_ambiguity", 0)) for event in event_list)
        ),
        "rejected_by_2d_score": int(
            sum(int(event.get("rejected_by_2d_score", 0)) for event in event_list)
        ),
        "rejected_by_rms": int(sum(int(event.get("rejected_by_rms", 0)) for event in event_list)),
        "flip_risk_count": int(sum(int(event.get("flip_risk_count", 0)) for event in event_list)),
        "subset_adoption_ready_count": adoption_ready_count,
        "truncated_count": int(sum(1 for event in event_list if event.get("truncated"))),
        "margin_summary": _numeric_summary(margins),
        "combined_margin_summary": _numeric_summary(combined_margins),
        "score_delta_summary": _numeric_summary(score_deltas),
        "best_p95_error_px_summary": _numeric_summary(best_p95_errors),
        "best_rotation_delta_deg_summary": _numeric_summary(best_rotation_deltas),
    }
    totals["subset_adoption_ready_ratio"] = (
        float(adoption_ready_count / len(event_list)) if event_list else 0.0
    )
    totals["phase6_complete"] = bool(
        totals["event_count"] > 0
        and totals["candidate_count"] > 0
        and totals["valid_candidate_count"] > 0
        and totals["truncated_count"] == 0
    )
    totals["subset_adoption_shadow"] = _summarize_subset_adoption_shadow(event_list)
    return {"by_rigid": by_rigid, "totals": totals}


def _summarize_subset_adoption_shadow(events: List[Dict[str, Any]]) -> Dict[str, Any]:
    adopted = [event for event in events if event.get("subset_adoption_ready")]
    baseline_valid = int(sum(1 for event in events if event.get("generic_valid", False)))
    shadow_valid = int(
        sum(
            1
            for event in events
            if event.get("generic_valid", False) or event.get("subset_adoption_ready")
        )
    )
    score_worse = int(
        sum(1 for event in adopted if float(event.get("score_delta", 0.0)) < -1e-9)
    )
    flip_worse = int(
        sum(1 for event in adopted if float(event.get("best_rotation_delta_deg", 0.0)) > 90.0)
    )
    jump_worse = int(
        sum(1 for event in adopted if float(event.get("best_position_delta_m", 0.0)) > 0.10)
    )
    adoption_ratio = float(len(adopted) / len(events)) if events else 0.0
    no_worse = bool(score_worse == 0 and flip_worse == 0 and jump_worse == 0)
    ready_for_enforcement = bool(no_worse and adoption_ratio >= 0.90)
    if ready_for_enforcement:
        decision = "go_candidate_for_enforcement_replay"
        reason = "shadow adoption shows broad coverage without score, flip, or jump regressions"
    elif no_worse:
        decision = "keep_shadow_until_coverage_improves"
        reason = "shadow adoption did not worsen candidate deltas, but coverage is still too low"
    else:
        decision = "no_go_tune_subset_adoption"
        reason = "shadow adoption has score, flip, or jump regressions"
    return {
        "decision": decision,
        "reason": reason,
        "event_count": int(len(events)),
        "adopted_event_count": int(len(adopted)),
        "adoption_ratio": adoption_ratio,
        "baseline_valid_events": baseline_valid,
        "shadow_valid_events": shadow_valid,
        "valid_event_delta": int(shadow_valid - baseline_valid),
        "score_worse_count": score_worse,
        "flip_worse_count": flip_worse,
        "jump_worse_count": jump_worse,
        "ready_for_enforcement_replay": ready_for_enforcement,
        "criteria": {
            "min_adoption_ratio": 0.90,
            "max_score_worse_count": 0,
            "max_flip_worse_count": 0,
            "max_jump_worse_count": 0,
            "jump_threshold_m": 0.10,
            "flip_threshold_deg": 90.0,
        },
    }


def _numeric_summary(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"count": 0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    sorted_values = sorted(float(value) for value in values)
    p95_index = min(len(sorted_values) - 1, int(round((len(sorted_values) - 1) * 0.95)))
    return {
        "count": int(len(sorted_values)),
        "mean": float(sum(sorted_values) / len(sorted_values)),
        "p95": float(sorted_values[p95_index]),
        "max": float(sorted_values[-1]),
    }


def _resolve_replay_calibration_path(path: Path) -> Path:
    """Resolve replay calibration inputs to the directory form needed by loaders."""
    if path.is_file() and path.name.startswith("extrinsics_pose_v2"):
        return path.parent
    return path


def _iso_with_offset(value: str, offset_s: float) -> str:
    if not value:
        return ""
    try:
        return (datetime.fromisoformat(value) + timedelta(seconds=float(offset_s))).isoformat()
    except Exception:
        return value


def _frame_entry_timestamp(frame: Any) -> int:
    data = getattr(frame, "data", {})
    if not isinstance(data, dict):
        data = {}
    return int(getattr(frame, "timestamp", 0) or data.get("timestamp", 0) or 0)


def _merge_failure_segments(events: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    segments: List[Dict[str, Any]] = []
    for event in events:
        try:
            start = datetime.fromisoformat(str(event.get("window_start", "")))
            end = datetime.fromisoformat(str(event.get("window_end", "")))
        except Exception:
            continue
        reasons = set(str(reason) for reason in event.get("reasons", []))
        if not segments:
            segments.append(
                {
                    "start": start.isoformat(),
                    "end": end.isoformat(),
                    "event_count": 1,
                    "reasons": sorted(reasons),
                    "max_pose_jump_m": float(event.get("max_pose_jump_m", 0.0) or 0.0),
                    "max_pose_flip_deg": float(event.get("max_pose_flip_deg", 0.0) or 0.0),
                    "min_accepted_points": int(event.get("accepted_points", 0) or 0),
                }
            )
            continue

        previous = segments[-1]
        previous_end = datetime.fromisoformat(str(previous["end"]))
        if start <= previous_end:
            previous["end"] = max(previous_end, end).isoformat()
            previous["event_count"] = int(previous["event_count"]) + 1
            previous["reasons"] = sorted(set(previous["reasons"]) | reasons)
            previous["max_pose_jump_m"] = max(
                float(previous.get("max_pose_jump_m", 0.0)),
                float(event.get("max_pose_jump_m", 0.0) or 0.0),
            )
            previous["max_pose_flip_deg"] = max(
                float(previous.get("max_pose_flip_deg", 0.0)),
                float(event.get("max_pose_flip_deg", 0.0) or 0.0),
            )
            previous["min_accepted_points"] = min(
                int(previous.get("min_accepted_points", 0)),
                int(event.get("accepted_points", 0) or 0),
            )
            continue

        segments.append(
            {
                "start": start.isoformat(),
                "end": end.isoformat(),
                "event_count": 1,
                "reasons": sorted(reasons),
                "max_pose_jump_m": float(event.get("max_pose_jump_m", 0.0) or 0.0),
                "max_pose_flip_deg": float(event.get("max_pose_flip_deg", 0.0) or 0.0),
                "min_accepted_points": int(event.get("accepted_points", 0) or 0),
            }
        )
    return segments


def _single_run_phase45_decision(
    tracking: Dict[str, Dict[str, Any]],
    guard_summary: Dict[str, Any],
    *,
    enforced: bool,
) -> Dict[str, Any]:
    would_reject_count = int(guard_summary.get("would_reject_event_count", 0))
    rejected_count = int(guard_summary.get("enforced_reject_event_count", 0))
    if not enforced:
        decision = "pending_enforcement_replay" if would_reject_count else "no_reacquire_reject_signal"
        return {
            "decision": decision,
            "reason": "shadow replay does not change acceptance",
            "needs_enforcement_replay": bool(would_reject_count),
            "would_reject_event_count": would_reject_count,
            "rejected_count": rejected_count,
        }
    return {
        "decision": "needs_shadow_comparison",
        "reason": "single enforced replay cannot determine go/no-go without shadow baseline",
        "needs_enforcement_replay": False,
        "would_reject_event_count": would_reject_count,
        "rejected_count": rejected_count,
    }


def _tracking_metric(summary: Dict[str, Any], key: str, default: float = 0.0) -> float:
    tracking = summary.get("tracking", {})
    if not isinstance(tracking, dict) or not tracking:
        return float(default)
    first = next(iter(tracking.values()))
    if not isinstance(first, dict):
        return float(default)
    try:
        return float(first.get(key, default))
    except Exception:
        return float(default)


def _subset_rotation_max(summary: Dict[str, Any]) -> float:
    try:
        return float(
            summary.get("subset_hypothesis_summary", {})
            .get("totals", {})
            .get("best_rotation_delta_deg_summary", {})
            .get("max", 0.0)
        )
    except Exception:
        return 0.0


def _compare_phase45_go_no_go(shadow: Dict[str, Any], enforced: Dict[str, Any]) -> Dict[str, Any]:
    shadow_valid = float(shadow.get("poses_estimated", 0) or 0)
    enforced_valid = float(enforced.get("poses_estimated", 0) or 0)
    valid_drop_ratio = (
        max(0.0, shadow_valid - enforced_valid) / shadow_valid
        if shadow_valid > 0.0
        else 0.0
    )
    shadow_flip = _tracking_metric(shadow, "max_pose_flip_deg")
    enforced_flip = _tracking_metric(enforced, "max_pose_flip_deg")
    shadow_jumps = _tracking_metric(shadow, "pose_jump_count")
    enforced_jumps = _tracking_metric(enforced, "pose_jump_count")
    rejected_count = int(
        enforced.get("reacquire_guard_summary", {})
        .get("enforced_reject_event_count", 0)
    )
    flip_not_worse = enforced_flip <= shadow_flip + 1e-9
    jumps_not_worse = enforced_jumps <= shadow_jumps + 1e-9
    valid_ok = valid_drop_ratio <= 0.02
    go_candidate = bool(valid_ok and flip_not_worse and jumps_not_worse)
    if not rejected_count:
        decision = "keep_shadow_only"
        reason = "enforcement did not reject any candidate in this replay"
    elif go_candidate:
        decision = "go_candidate"
        reason = "enforcement did not worsen replay metrics beyond configured thresholds"
    else:
        decision = "no_go_adjust_thresholds"
        reason = "enforcement worsened valid frames, pose flips, or pose jumps"
    return {
        "decision": decision,
        "reason": reason,
        "valid_drop_ratio": float(valid_drop_ratio),
        "valid_drop_frames": int(max(0.0, shadow_valid - enforced_valid)),
        "shadow_valid_frames": int(shadow_valid),
        "enforced_valid_frames": int(enforced_valid),
        "shadow_max_pose_flip_deg": float(shadow_flip),
        "enforced_max_pose_flip_deg": float(enforced_flip),
        "shadow_pose_jump_count": int(shadow_jumps),
        "enforced_pose_jump_count": int(enforced_jumps),
        "enforced_rejected_count": int(rejected_count),
        "criteria": {
            "max_valid_drop_ratio": 0.02,
            "pose_flip_not_worse": bool(flip_not_worse),
            "pose_jump_not_worse": bool(jumps_not_worse),
        },
    }


def _compare_object_gating_go_no_go(
    diagnostics_only: Dict[str, Any],
    enforced: Dict[str, Any],
) -> Dict[str, Any]:
    baseline_valid = float(diagnostics_only.get("poses_estimated", 0) or 0)
    enforced_valid = float(enforced.get("poses_estimated", 0) or 0)
    valid_drop_ratio = (
        max(0.0, baseline_valid - enforced_valid) / baseline_valid
        if baseline_valid > 0.0
        else 0.0
    )
    baseline_flip = _tracking_metric(diagnostics_only, "max_pose_flip_deg")
    enforced_flip = _tracking_metric(enforced, "max_pose_flip_deg")
    baseline_jumps = _tracking_metric(diagnostics_only, "pose_jump_count")
    enforced_jumps = _tracking_metric(enforced, "pose_jump_count")
    baseline_transitions = _tracking_metric(diagnostics_only, "mode_transition_count")
    enforced_transitions = _tracking_metric(enforced, "mode_transition_count")
    baseline_subset_rotation = _subset_rotation_max(diagnostics_only)
    enforced_subset_rotation = _subset_rotation_max(enforced)
    max_flip_regression_deg = 1.0
    valid_ok = valid_drop_ratio <= 0.02
    flip_delta_deg = float(enforced_flip - baseline_flip)
    flip_not_worse = flip_delta_deg <= max_flip_regression_deg
    jumps_not_worse = enforced_jumps <= baseline_jumps + 1e-9
    transitions_not_worse = enforced_transitions <= baseline_transitions + 1e-9
    subset_rotation_improved = enforced_subset_rotation < baseline_subset_rotation - 1e-9
    go_candidate = bool(
        valid_ok and flip_not_worse and jumps_not_worse and transitions_not_worse
    )
    if go_candidate and subset_rotation_improved:
        decision = "go_candidate"
        reason = "enforcement is stable and reduces subset rotation mismatch"
    elif go_candidate:
        decision = "go_candidate"
        reason = "enforcement stays within flip tolerance and does not worsen valid frames, jumps, or mode transitions"
    else:
        decision = "no_go_tune_object_gating"
        reason = "enforcement worsened valid frames, flips beyond tolerance, jumps, or mode transitions"
    return {
        "decision": decision,
        "reason": reason,
        "valid_drop_ratio": float(valid_drop_ratio),
        "valid_drop_frames": int(max(0.0, baseline_valid - enforced_valid)),
        "diagnostics_only_valid_frames": int(baseline_valid),
        "enforced_valid_frames": int(enforced_valid),
        "diagnostics_only_max_pose_flip_deg": float(baseline_flip),
        "enforced_max_pose_flip_deg": float(enforced_flip),
        "max_pose_flip_delta_deg": float(flip_delta_deg),
        "diagnostics_only_pose_jump_count": int(baseline_jumps),
        "enforced_pose_jump_count": int(enforced_jumps),
        "diagnostics_only_mode_transition_count": int(baseline_transitions),
        "enforced_mode_transition_count": int(enforced_transitions),
        "diagnostics_only_subset_rotation_max_deg": float(baseline_subset_rotation),
        "enforced_subset_rotation_max_deg": float(enforced_subset_rotation),
        "criteria": {
            "max_valid_drop_ratio": 0.02,
            "max_flip_regression_deg": float(max_flip_regression_deg),
            "pose_flip_within_tolerance": bool(flip_not_worse),
            "pose_jump_not_worse": bool(jumps_not_worse),
            "mode_transition_not_worse": bool(transitions_not_worse),
            "subset_rotation_improved": bool(subset_rotation_improved),
        },
    }


def _timestamp_to_seconds(value: str) -> float:
    if not value:
        return 0.0
    return datetime.fromisoformat(value).timestamp()


def _load_patterns(
    pattern_names: Optional[Iterable[str]],
    rigids_path: str | Path | None,
) -> List[MarkerPattern]:
    available = dict(BUILTIN_PATTERNS)
    available.update(_load_custom_patterns(rigids_path))
    requested = [name for name in (pattern_names or [WAIST_PATTERN.name]) if str(name).strip()]
    selected: List[MarkerPattern] = []
    missing: List[str] = []
    for name in requested:
        pattern = available.get(str(name).strip())
        if pattern is None:
            missing.append(str(name))
            continue
        selected.append(pattern)
    if missing:
        raise ValueError(f"unknown rigid body pattern(s): {', '.join(missing)}")
    return selected or [WAIST_PATTERN]


def _load_custom_patterns(rigids_path: str | Path | None) -> Dict[str, MarkerPattern]:
    if rigids_path is None:
        return {}
    path = Path(rigids_path)
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    definitions = payload.get("custom_rigids", []) if isinstance(payload, dict) else []
    if not isinstance(definitions, list):
        return {}

    patterns: Dict[str, MarkerPattern] = {}
    for definition in definitions:
        if not isinstance(definition, dict):
            continue
        name = str(definition.get("name") or "").strip()
        marker_positions = definition.get("marker_positions", [])
        if not name or name in BUILTIN_PATTERNS:
            continue
        points = np.asarray(marker_positions, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
            continue
        try:
            marker_diameter = float(definition.get("marker_diameter_m", 0.014) or 0.014)
        except Exception:
            marker_diameter = 0.014
        patterns[name] = MarkerPattern(
            name=name,
            marker_positions=points,
            marker_diameter=marker_diameter if marker_diameter > 0.0 else 0.014,
            metadata={
                "notes": str(definition.get("notes") or ""),
                "created_at": int(definition.get("created_at", 0) or 0),
                "source": str(definition.get("source") or "custom_selection"),
            },
        )
    return patterns


def _parse_pattern_names(raw: str | None) -> List[str]:
    if not raw:
        return [WAIST_PATTERN.name]
    return [item.strip() for item in raw.split(",") if item.strip()]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Replay tracking JSONL through the host tracking pipeline.")
    parser.add_argument("--log", required=True, help="Path to tracking JSONL log.")
    parser.add_argument("--calibration", required=True, help="Calibration directory or file.")
    parser.add_argument("--rigids", default="calibration/tracking_rigids.json", help="Custom rigid-body JSON path.")
    parser.add_argument("--patterns", default=WAIST_PATTERN.name, help="Comma-separated rigid-body pattern names.")
    parser.add_argument("--epipolar-threshold-px", type=float, default=None)
    parser.add_argument("--reacquire-guard-enforced", action="store_true", help="Replay with Phase 4.5 guard enforcement enabled.")
    parser.add_argument("--disable-reacquire-guard-shadow", action="store_true", help="Disable Phase 4.5 shadow guard diagnostics.")
    parser.add_argument("--reacquire-guard-event-logging", action="store_true", help="Enable live logger guard events during replay.")
    parser.add_argument("--reacquire-guard-post-reacquire-frames", type=int, default=0, help="Also evaluate the guard for this many continue frames after reacquire confirmation.")
    parser.add_argument("--reacquire-guard-max-rotation-deg", type=float, default=136.0, help="Rotation innovation threshold used by the guard.")
    parser.add_argument("--object-gating-enforced", action="store_true", help="Replay with Phase 5 object-gating rigid-hint pose enforcement enabled.")
    parser.add_argument("--object-gating-activation-mode", default="always", choices=["always", "reacquire_only", "boot_or_reacquire"], help="Limit object-gating evaluation to selected tracker modes.")
    parser.add_argument(
        "--object-gating-ambiguous-blob-min-separation-px",
        type=float,
        default=0.60,
        help=(
            "Drop object-gating assignments whose blob is this close to another "
            "same-camera blob; set 0 to disable."
        ),
    )
    parser.add_argument(
        "--object-gating-ambiguous-blob-diameter-overlap-ratio",
        type=float,
        default=0.30,
        help=(
            "Also drop assignments when blob centers are closer than this fraction "
            "of their average equivalent diameter."
        ),
    )
    parser.add_argument(
        "--object-gating-ambiguous-marker-assignment-min-margin-px",
        type=float,
        default=0.29,
        help=(
            "Drop object-gating assignments when a blob is nearly as close to a "
            "different marker projection."
        ),
    )
    parser.add_argument("--pose-continuity-guard-enabled", action="store_true", help="Evaluate low-marker temporal pose continuity guard diagnostics.")
    parser.add_argument("--pose-continuity-guard-enforced", action="store_true", help="Hold predicted pose when the low-marker continuity guard rejects a candidate.")
    parser.add_argument("--pose-continuity-max-rotation-deg", type=float, default=90.0, help="Maximum low-marker rotation innovation before continuity guard rejection.")
    parser.add_argument("--pose-continuity-max-angular-velocity-deg-s", type=float, default=2500.0, help="Maximum low-marker angular velocity before continuity guard rejection.")
    parser.add_argument("--pose-continuity-max-angular-accel-deg-s2", type=float, default=200000.0, help="Maximum low-marker angular acceleration before continuity guard rejection.")
    parser.add_argument("--position-continuity-guard-enabled", action="store_true", help="Evaluate low-marker position acceleration guard diagnostics.")
    parser.add_argument("--position-continuity-guard-enforced", action="store_true", help="Clamp low-marker position updates when acceleration or velocity is too high.")
    parser.add_argument("--position-continuity-max-accel-m-s2", type=float, default=60.0, help="Maximum low-marker position acceleration before position guard clamping.")
    parser.add_argument("--position-continuity-max-velocity-m-s", type=float, default=8.0, help="Maximum low-marker position velocity after acceleration limiting.")
    parser.add_argument("--compare-reacquire-guard-enforcement", action="store_true", help="Run both shadow and enforced Phase 4.5 replays and compare go/no-go.")
    parser.add_argument("--compare-object-gating-enforcement", action="store_true", help="Run diagnostics-only and enforced object-gating replays and compare go/no-go.")
    parser.add_argument("--pipeline-variant", default=DEFAULT_PIPELINE_VARIANT, choices=list(PIPELINE_VARIANTS), help="Replay through the official tracking path or an explicit comparison variant.")
    parser.add_argument("--compare-pipeline-variants", action="store_true", help="Run baseline and all fast-path variants on the same replay input.")
    parser.add_argument("--compare-epipolar-pruning", action="store_true", help="Run fast_ABCD and fast_ABCDP on the same replay input and compare exact quality plus speed.")
    parser.add_argument("--compare-performance-upgrades", action="store_true", help="Run fast_ABCD and staged performance-upgrade variants on the same replay input.")
    parser.add_argument("--subset-diagnostics-mode", default=None, choices=["full", "sampled", "off"], help="Control subset hypothesis diagnostics frequency during replay. Defaults follow the selected pipeline variant.")
    parser.add_argument("--ab-report-out", default=None, help="Optional JSON path for --compare-pipeline-variants output.")
    parser.add_argument("--diagnostics-summary", action="store_true", help="Summarize live tracking_diagnostics events without replaying frames.")
    parser.add_argument("--min-accepted-points", type=int, default=4, help="Minimum accepted points before a diagnostics event is flagged.")
    parser.add_argument("--jump-threshold-m", type=float, default=0.10, help="Pose jump threshold for diagnostics failure extraction.")
    parser.add_argument("--flip-threshold-deg", type=float, default=90.0, help="Pose flip threshold for diagnostics failure extraction.")
    parser.add_argument("--failure-window-padding-s", type=float, default=2.0, help="Padding around diagnostics failure events when merging segments.")
    parser.add_argument("--start-timestamp-us", type=int, default=None, help="Replay only frames at or after this log timestamp.")
    parser.add_argument("--end-timestamp-us", type=int, default=None, help="Replay only frames at or before this log timestamp.")
    parser.add_argument("--start-received-at", default=None, help="Replay only frames received at or after this ISO timestamp.")
    parser.add_argument("--end-received-at", default=None, help="Replay only frames received at or before this ISO timestamp.")
    parser.add_argument("--max-frames", type=int, default=None, help="Replay at most this many frames after filtering.")
    parser.add_argument("--quiet", action="store_true", help="Do not print the JSON summary when --out is set.")
    parser.add_argument("--out", default=None, help="Optional summary JSON output path.")
    args = parser.parse_args(argv)

    if args.diagnostics_summary:
        summary = summarize_tracking_diagnostics(
            log_path=args.log,
            patterns=_parse_pattern_names(args.patterns),
            min_accepted_points=args.min_accepted_points,
            jump_threshold_m=args.jump_threshold_m,
            flip_threshold_deg=args.flip_threshold_deg,
            window_padding_s=args.failure_window_padding_s,
        )
    elif args.compare_pipeline_variants:
        summary = compare_pipeline_variants(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            reacquire_guard_enforced=bool(args.reacquire_guard_enforced),
            reacquire_guard_shadow_enabled=not bool(args.disable_reacquire_guard_shadow),
            reacquire_guard_event_logging=bool(args.reacquire_guard_event_logging),
            reacquire_guard_post_reacquire_frames=int(args.reacquire_guard_post_reacquire_frames),
            reacquire_guard_max_rotation_deg=args.reacquire_guard_max_rotation_deg,
            object_gating_enforced=bool(args.object_gating_enforced),
            object_gating_activation_mode=args.object_gating_activation_mode,
            pose_continuity_guard_enabled=bool(args.pose_continuity_guard_enabled),
            pose_continuity_guard_enforced=bool(args.pose_continuity_guard_enforced),
            pose_continuity_max_rotation_deg=args.pose_continuity_max_rotation_deg,
            pose_continuity_max_angular_velocity_deg_s=args.pose_continuity_max_angular_velocity_deg_s,
            pose_continuity_max_angular_accel_deg_s2=args.pose_continuity_max_angular_accel_deg_s2,
            position_continuity_guard_enabled=bool(args.position_continuity_guard_enabled),
            position_continuity_guard_enforced=bool(args.position_continuity_guard_enforced),
            position_continuity_max_accel_m_s2=args.position_continuity_max_accel_m_s2,
            position_continuity_max_velocity_m_s=args.position_continuity_max_velocity_m_s,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            subset_diagnostics_mode=args.subset_diagnostics_mode,
        )
    elif args.compare_epipolar_pruning:
        summary = compare_epipolar_pruning(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            reacquire_guard_enforced=bool(args.reacquire_guard_enforced),
            reacquire_guard_shadow_enabled=not bool(args.disable_reacquire_guard_shadow),
            reacquire_guard_event_logging=bool(args.reacquire_guard_event_logging),
            reacquire_guard_post_reacquire_frames=int(args.reacquire_guard_post_reacquire_frames),
            reacquire_guard_max_rotation_deg=args.reacquire_guard_max_rotation_deg,
            object_gating_enforced=bool(args.object_gating_enforced),
            object_gating_activation_mode=args.object_gating_activation_mode,
            pose_continuity_guard_enabled=bool(args.pose_continuity_guard_enabled),
            pose_continuity_guard_enforced=bool(args.pose_continuity_guard_enforced),
            pose_continuity_max_rotation_deg=args.pose_continuity_max_rotation_deg,
            pose_continuity_max_angular_velocity_deg_s=args.pose_continuity_max_angular_velocity_deg_s,
            pose_continuity_max_angular_accel_deg_s2=args.pose_continuity_max_angular_accel_deg_s2,
            position_continuity_guard_enabled=bool(args.position_continuity_guard_enabled),
            position_continuity_guard_enforced=bool(args.position_continuity_guard_enforced),
            position_continuity_max_accel_m_s2=args.position_continuity_max_accel_m_s2,
            position_continuity_max_velocity_m_s=args.position_continuity_max_velocity_m_s,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            subset_diagnostics_mode=args.subset_diagnostics_mode,
        )
    elif args.compare_performance_upgrades:
        summary = compare_performance_upgrades(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            reacquire_guard_enforced=bool(args.reacquire_guard_enforced),
            reacquire_guard_shadow_enabled=not bool(args.disable_reacquire_guard_shadow),
            reacquire_guard_event_logging=bool(args.reacquire_guard_event_logging),
            reacquire_guard_post_reacquire_frames=int(args.reacquire_guard_post_reacquire_frames),
            reacquire_guard_max_rotation_deg=args.reacquire_guard_max_rotation_deg,
            object_gating_enforced=bool(args.object_gating_enforced),
            object_gating_activation_mode=args.object_gating_activation_mode,
            pose_continuity_guard_enabled=bool(args.pose_continuity_guard_enabled),
            pose_continuity_guard_enforced=bool(args.pose_continuity_guard_enforced),
            pose_continuity_max_rotation_deg=args.pose_continuity_max_rotation_deg,
            pose_continuity_max_angular_velocity_deg_s=args.pose_continuity_max_angular_velocity_deg_s,
            pose_continuity_max_angular_accel_deg_s2=args.pose_continuity_max_angular_accel_deg_s2,
            position_continuity_guard_enabled=bool(args.position_continuity_guard_enabled),
            position_continuity_guard_enforced=bool(args.position_continuity_guard_enforced),
            position_continuity_max_accel_m_s2=args.position_continuity_max_accel_m_s2,
            position_continuity_max_velocity_m_s=args.position_continuity_max_velocity_m_s,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            subset_diagnostics_mode=args.subset_diagnostics_mode,
        )
    elif args.compare_reacquire_guard_enforcement:
        summary = compare_reacquire_guard_enforcement(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            reacquire_guard_post_reacquire_frames=int(args.reacquire_guard_post_reacquire_frames),
            reacquire_guard_max_rotation_deg=args.reacquire_guard_max_rotation_deg,
        )
    elif args.compare_object_gating_enforcement:
        summary = compare_object_gating_enforcement(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            object_gating_activation_mode=args.object_gating_activation_mode,
            object_gating_ambiguous_blob_min_separation_px=args.object_gating_ambiguous_blob_min_separation_px,
            object_gating_ambiguous_blob_diameter_overlap_ratio=args.object_gating_ambiguous_blob_diameter_overlap_ratio,
            object_gating_ambiguous_marker_assignment_min_margin_px=args.object_gating_ambiguous_marker_assignment_min_margin_px,
        )
    else:
        summary = replay_tracking_log(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
            reacquire_guard_enforced=bool(args.reacquire_guard_enforced),
            reacquire_guard_shadow_enabled=not bool(args.disable_reacquire_guard_shadow),
            reacquire_guard_event_logging=bool(args.reacquire_guard_event_logging),
            reacquire_guard_post_reacquire_frames=int(args.reacquire_guard_post_reacquire_frames),
            reacquire_guard_max_rotation_deg=args.reacquire_guard_max_rotation_deg,
            object_gating_enforced=bool(args.object_gating_enforced),
            object_gating_activation_mode=args.object_gating_activation_mode,
            object_gating_ambiguous_blob_min_separation_px=args.object_gating_ambiguous_blob_min_separation_px,
            object_gating_ambiguous_blob_diameter_overlap_ratio=args.object_gating_ambiguous_blob_diameter_overlap_ratio,
            object_gating_ambiguous_marker_assignment_min_margin_px=args.object_gating_ambiguous_marker_assignment_min_margin_px,
            pose_continuity_guard_enabled=bool(args.pose_continuity_guard_enabled),
            pose_continuity_guard_enforced=bool(args.pose_continuity_guard_enforced),
            pose_continuity_max_rotation_deg=args.pose_continuity_max_rotation_deg,
            pose_continuity_max_angular_velocity_deg_s=args.pose_continuity_max_angular_velocity_deg_s,
            pose_continuity_max_angular_accel_deg_s2=args.pose_continuity_max_angular_accel_deg_s2,
            position_continuity_guard_enabled=bool(args.position_continuity_guard_enabled),
            position_continuity_guard_enforced=bool(args.position_continuity_guard_enforced),
            position_continuity_max_accel_m_s2=args.position_continuity_max_accel_m_s2,
            position_continuity_max_velocity_m_s=args.position_continuity_max_velocity_m_s,
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
            pipeline_variant=args.pipeline_variant,
            subset_diagnostics_mode=args.subset_diagnostics_mode,
        ).to_dict()

    text = json.dumps(summary, ensure_ascii=False, indent=2)
    out_path = args.out
    if args.compare_pipeline_variants:
        out_path = args.ab_report_out or args.out
        if out_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = str(Path("logs") / "archive" / f"fast_pipeline_ab_{timestamp}" / "comparison.json")
    elif args.compare_epipolar_pruning:
        out_path = args.ab_report_out or args.out
        if out_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = str(Path("logs") / "archive" / f"epipolar_pruning_ab_{timestamp}" / "comparison.json")
    elif args.compare_performance_upgrades:
        out_path = args.ab_report_out or args.out
        if out_path is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out_path = str(Path("logs") / "archive" / f"performance_upgrade_ab_{timestamp}" / "comparison.json")
    if out_path:
        Path(out_path).parent.mkdir(parents=True, exist_ok=True)
        Path(out_path).write_text(text + "\n", encoding="utf-8")
    if not (args.quiet and out_path):
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
