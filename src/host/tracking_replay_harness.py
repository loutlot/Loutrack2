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
from datetime import datetime
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
) -> TrackingReplaySummary:
    """Replay a tracking log through the existing host pipeline path."""
    log_path = Path(log_path)
    calibration_path = Path(calibration_path)
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
    for entry in replay.replay(realtime=False):
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
        phase45_go_no_go=_single_run_phase45_decision(
            tracking,
            guard_summary,
            enforced=bool(reacquire_guard_enforced),
        ),
    )


def compare_reacquire_guard_enforcement(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
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
    ).to_dict()
    enforced = replay_tracking_log(
        log_path=log_path,
        calibration_path=calibration_path,
        patterns=patterns,
        rigids_path=rigids_path,
        epipolar_threshold_px=epipolar_threshold_px,
        reacquire_guard_enforced=True,
        reacquire_guard_shadow_enabled=True,
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
        for key in totals:
            if key == "evaluated_count":
                continue
            totals[key] += int(summary.get(key, 0))
    event_list = list(events or [])
    event_totals = {
        "event_count": int(len(event_list)),
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
    parser.add_argument("--compare-reacquire-guard-enforcement", action="store_true", help="Run both shadow and enforced Phase 4.5 replays and compare go/no-go.")
    parser.add_argument("--out", default=None, help="Optional summary JSON output path.")
    args = parser.parse_args(argv)

    if args.compare_reacquire_guard_enforcement:
        summary = compare_reacquire_guard_enforcement(
            log_path=args.log,
            calibration_path=args.calibration,
            patterns=_parse_pattern_names(args.patterns),
            rigids_path=args.rigids,
            epipolar_threshold_px=args.epipolar_threshold_px,
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
        ).to_dict()

    text = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
