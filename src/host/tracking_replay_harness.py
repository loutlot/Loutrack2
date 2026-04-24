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
        }


def replay_tracking_log(
    *,
    log_path: str | Path,
    calibration_path: str | Path,
    patterns: Optional[Iterable[str]] = None,
    rigids_path: str | Path | None = None,
    epipolar_threshold_px: Optional[float] = None,
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
    pair_count = int(
        receiver.get("pairer", {}).get("pairs_emitted", pipeline.frames_processed)
    )

    return TrackingReplaySummary(
        log_path=str(log_path),
        calibration_path=str(calibration_path),
        frame_count=frame_count,
        pair_count=pair_count,
        frames_processed=int(status.get("frames_processed", pipeline.frames_processed)),
        poses_estimated=int(status.get("poses_estimated", pipeline.poses_estimated)),
        patterns=[pattern.name for pattern in selected_patterns],
        tracking=dict(status.get("tracking", {})),
        receiver=dict(receiver),
        geometry=dict(diagnostics.get("geometry", {})),
        pipeline_stage_ms=dict(diagnostics.get("pipeline_stage_ms", {})),
    )


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
    parser.add_argument("--out", default=None, help="Optional summary JSON output path.")
    args = parser.parse_args(argv)

    summary = replay_tracking_log(
        log_path=args.log,
        calibration_path=args.calibration,
        patterns=_parse_pattern_names(args.patterns),
        rigids_path=args.rigids,
        epipolar_threshold_px=args.epipolar_threshold_px,
    ).to_dict()

    text = json.dumps(summary, ensure_ascii=False, indent=2)
    if args.out:
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(text + "\n", encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
