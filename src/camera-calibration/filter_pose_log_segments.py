#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, Sequence

SRC_ROOT = Path(__file__).resolve().parents[1]
MODULE_ROOT = Path(__file__).resolve().parent
for path in (SRC_ROOT, MODULE_ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from analyze_pose_segments import analyze_pose_segments  # type: ignore


def _extract_payload(entry: Dict[str, Any]) -> Dict[str, Any] | None:
    if entry.get("_type") == "frame" and isinstance(entry.get("data"), dict):
        return entry["data"]
    if "camera_id" in entry and "timestamp" in entry:
        return entry
    return None


def _time_ranges_to_remove(analysis: Dict[str, Any], median_ratio_threshold: float, remove_segment_ids: set[int] | None) -> list[tuple[int, int]]:
    global_summary = analysis.get("global_row_error_px", {})
    global_median = global_summary.get("median")
    if global_median is None:
        return []
    ranges: list[tuple[int, int]] = []
    for segment in analysis.get("time_segments", []):
        if not isinstance(segment, dict):
            continue
        segment_id = int(segment.get("segment_id", -1))
        if remove_segment_ids is not None and segment_id not in remove_segment_ids:
            continue
        median = segment.get("median")
        time_range = segment.get("time_range_us")
        if median is None or not isinstance(time_range, list) or len(time_range) != 2:
            continue
        if float(median) >= float(global_median) * median_ratio_threshold:
            ranges.append((int(time_range[0]), int(time_range[1])))
    return ranges


def _within_any_range(timestamp_us: int, ranges: Iterable[tuple[int, int]]) -> bool:
    for start_us, end_us in ranges:
        if start_us <= timestamp_us <= end_us:
            return True
    return False


def filter_pose_log_by_time_segments(
    *,
    intrinsics_path: str | Path,
    extrinsics_path: str | Path,
    pose_log_path: str | Path,
    output_path: str | Path,
    pair_window_us: int,
    time_bins: int = 6,
    position_bins: int = 4,
    image_grid_cols: int = 3,
    image_grid_rows: int = 3,
    median_ratio_threshold: float = 1.5,
    remove_segment_ids: set[int] | None = None,
) -> Dict[str, Any]:
    analysis = analyze_pose_segments(
        intrinsics_path=intrinsics_path,
        extrinsics_path=extrinsics_path,
        pose_log_path=pose_log_path,
        pair_window_us=pair_window_us,
        time_bins=time_bins,
        position_bins=position_bins,
        image_grid_cols=image_grid_cols,
        image_grid_rows=image_grid_rows,
    )
    remove_ranges = _time_ranges_to_remove(analysis, median_ratio_threshold, remove_segment_ids)
    source = Path(pose_log_path)
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)

    total_frames = 0
    removed_frames = 0
    kept_frames = 0
    with source.open("r", encoding="utf-8") as src, output.open("w", encoding="utf-8") as dst:
        for raw_line in src:
            stripped = raw_line.strip()
            if not stripped:
                continue
            entry = json.loads(stripped)
            payload = _extract_payload(entry)
            if payload is None:
                dst.write(raw_line)
                continue
            total_frames += 1
            timestamp_us = int(payload.get("timestamp", 0))
            if _within_any_range(timestamp_us, remove_ranges):
                removed_frames += 1
                continue
            kept_frames += 1
            dst.write(raw_line)

    return {
        "input_pose_log_path": str(source),
        "output_pose_log_path": str(output),
        "removed_time_ranges_us": [[start_us, end_us] for start_us, end_us in remove_ranges],
        "analysis": analysis,
        "median_ratio_threshold": float(median_ratio_threshold),
        "removed_segment_ids": sorted(remove_segment_ids) if remove_segment_ids is not None else None,
        "total_frames": int(total_frames),
        "removed_frames": int(removed_frames),
        "kept_frames": int(kept_frames),
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Filter pose log by removing time segments with poor reprojection error")
    parser.add_argument("--intrinsics", default="calibration", help="Intrinsics directory or file")
    parser.add_argument("--extrinsics", default="calibration/extrinsics_pose_v2.json", help="Extrinsics v2 JSON path")
    parser.add_argument("--pose-log", default="logs/extrinsics_pose_capture.jsonl", help="Pose capture JSONL path")
    parser.add_argument("--output", required=True, help="Filtered pose log output path")
    parser.add_argument("--pair-window-us", type=int, default=2000, help="Pairing window for row building")
    parser.add_argument("--time-bins", type=int, default=6, help="Number of time segments")
    parser.add_argument("--position-bins", type=int, default=4, help="Number of position quantile bins")
    parser.add_argument("--image-grid-cols", type=int, default=3, help="Anchor image grid columns")
    parser.add_argument("--image-grid-rows", type=int, default=3, help="Anchor image grid rows")
    parser.add_argument("--median-ratio-threshold", type=float, default=1.5, help="Remove time bins whose median reproj >= global median * threshold")
    parser.add_argument("--remove-segment-id", type=int, action="append", default=None, help="Explicit time segment id(s) to consider for removal; omit to auto-remove all bad bins")
    args = parser.parse_args(argv)

    result = filter_pose_log_by_time_segments(
        intrinsics_path=args.intrinsics,
        extrinsics_path=args.extrinsics,
        pose_log_path=args.pose_log,
        output_path=args.output,
        pair_window_us=args.pair_window_us,
        time_bins=args.time_bins,
        position_bins=args.position_bins,
        image_grid_cols=args.image_grid_cols,
        image_grid_rows=args.image_grid_rows,
        median_ratio_threshold=args.median_ratio_threshold,
        remove_segment_ids=set(args.remove_segment_id) if args.remove_segment_id else None,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
