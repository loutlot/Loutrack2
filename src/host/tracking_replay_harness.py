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
)
from .pipeline import TrackingPipeline


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
    rigid_hint_events: List[Dict[str, Any]]
    rigid_hint_summary: Dict[str, Any]
    rigid_hint_pose_events: List[Dict[str, Any]]
    rigid_hint_pose_summary: Dict[str, Any]
    subset_hypothesis_events: List[Dict[str, Any]]
    subset_hypothesis_summary: Dict[str, Any]
    phase45_go_no_go: Dict[str, Any]

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
            "rigid_hint_events": self.rigid_hint_events,
            "rigid_hint_summary": self.rigid_hint_summary,
            "rigid_hint_pose_events": self.rigid_hint_pose_events,
            "rigid_hint_pose_summary": self.rigid_hint_pose_summary,
            "subset_hypothesis_events": self.subset_hypothesis_events,
            "subset_hypothesis_summary": self.subset_hypothesis_summary,
            "phase45_go_no_go": self.phase45_go_no_go,
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
    object_gating_enforced: bool = False,
    start_timestamp_us: Optional[int] = None,
    end_timestamp_us: Optional[int] = None,
    start_received_at: Optional[str] = None,
    end_received_at: Optional[str] = None,
    max_frames: Optional[int] = None,
) -> TrackingReplaySummary:
    """Replay a tracking log through the existing host pipeline path."""
    log_path = Path(log_path)
    calibration_input_path = Path(calibration_path)
    calibration_path = _resolve_replay_calibration_path(calibration_input_path)
    selected_patterns = _load_patterns(patterns, rigids_path)

    replay = FrameReplay(str(log_path))
    pipeline = TrackingPipeline(
        calibration_path=str(calibration_path),
        patterns=selected_patterns,
        enable_logging=False,
        epipolar_threshold_px=epipolar_threshold_px,
        reacquire_guard_config=ReacquireGuardConfig(
            shadow_enabled=bool(reacquire_guard_shadow_enabled),
            enforced=bool(reacquire_guard_enforced),
        ),
        object_gating_config=ObjectGatingConfig(enforce=bool(object_gating_enforced)),
        reacquire_guard_event_logging=bool(reacquire_guard_event_logging),
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
        pipeline_stage_ms=dict(diagnostics.get("pipeline_stage_ms", {})),
        reacquire_guard_events=guard_events,
        reacquire_guard_summary=guard_summary,
        object_gating_events=object_gating_events,
        object_gating_summary=_summarize_object_gating(tracking, object_gating_events),
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
    ).to_dict()
    return {
        "shadow": shadow,
        "enforced": enforced,
        "phase45_go_no_go": _compare_phase45_go_no_go(shadow, enforced),
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
    parser.add_argument("--object-gating-enforced", action="store_true", help="Replay with Phase 5 object-gating rigid-hint pose enforcement enabled.")
    parser.add_argument("--compare-reacquire-guard-enforcement", action="store_true", help="Run both shadow and enforced Phase 4.5 replays and compare go/no-go.")
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
            object_gating_enforced=bool(args.object_gating_enforced),
            start_timestamp_us=args.start_timestamp_us,
            end_timestamp_us=args.end_timestamp_us,
            start_received_at=args.start_received_at,
            end_received_at=args.end_received_at,
            max_frames=args.max_frames,
        ).to_dict()

    text = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
