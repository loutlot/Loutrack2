#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from collections import defaultdict, deque
from pathlib import Path
from typing import Any, Dict, List

import cv2
import numpy as np


def _extract_frame_payload(entry: Dict[str, Any]) -> Dict[str, Any] | None:
    if entry.get("_type") == "frame" and isinstance(entry.get("data"), dict):
        return entry["data"]
    if "camera_id" in entry and "timestamp" in entry and "blobs" in entry:
        return entry
    return None


def _load_frames(path: str | Path) -> Dict[str, List[Dict[str, Any]]]:
    frames_by_camera: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            entry = json.loads(line)
            payload = _extract_frame_payload(entry)
            if payload is None:
                continue
            camera_id = str(payload.get("camera_id", "")).strip()
            if not camera_id:
                continue
            frames_by_camera[camera_id].append(payload)
    for camera_id in frames_by_camera:
        frames_by_camera[camera_id].sort(
            key=lambda item: (int(item.get("timestamp", 0)), int(item.get("frame_index", 0)))
        )
    return frames_by_camera


def _infer_canvas_size(frames: List[Dict[str, Any]]) -> tuple[int, int]:
    max_x = 0.0
    max_y = 0.0
    for frame in frames:
        blobs = frame.get("blobs", [])
        if not isinstance(blobs, list):
            continue
        for blob in blobs:
            if not isinstance(blob, dict):
                continue
            max_x = max(max_x, float(blob.get("x", 0.0) or 0.0))
            max_y = max(max_y, float(blob.get("y", 0.0) or 0.0))
    width = max(640, int(np.ceil(max_x + 80.0)))
    height = max(480, int(np.ceil(max_y + 80.0)))
    return width, height


def render_pose_log_videos(
    *,
    log_path: str | Path,
    output_dir: str | Path,
    fps: int = 30,
    trail: int = 20,
    radius: int = 6,
) -> Dict[str, str]:
    frames_by_camera = _load_frames(log_path)
    if not frames_by_camera:
        raise ValueError("No frame entries found in pose log")

    output_root = Path(output_dir)
    output_root.mkdir(parents=True, exist_ok=True)
    outputs: Dict[str, str] = {}

    for camera_id, frames in frames_by_camera.items():
        width, height = _infer_canvas_size(frames)
        output_path = output_root / f"{camera_id}_2d_trace.mp4"
        writer = cv2.VideoWriter(
            str(output_path),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(fps),
            (width, height),
        )
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open VideoWriter for {output_path}")

        history: deque[tuple[int, int]] = deque(maxlen=max(trail, 1))
        for frame in frames:
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            blobs = frame.get("blobs", [])
            if not isinstance(blobs, list):
                blobs = []

            for idx, (hx, hy) in enumerate(history):
                alpha = (idx + 1) / max(len(history), 1)
                color = (0, int(140 * alpha), int(255 * alpha))
                cv2.circle(canvas, (hx, hy), max(2, radius // 2), color, -1, lineType=cv2.LINE_AA)

            for blob in blobs:
                if not isinstance(blob, dict):
                    continue
                x = int(round(float(blob.get("x", 0.0) or 0.0)))
                y = int(round(float(blob.get("y", 0.0) or 0.0)))
                cv2.circle(canvas, (x, y), radius, (0, 220, 255), 2, lineType=cv2.LINE_AA)
                cv2.circle(canvas, (x, y), max(2, radius // 2), (255, 255, 255), -1, lineType=cv2.LINE_AA)
                history.append((x, y))

            text_lines = [
                f"{camera_id}",
                f"frame={int(frame.get('frame_index', 0))}",
                f"timestamp={int(frame.get('timestamp', 0))}",
                f"blob_count={len(blobs)}",
            ]
            y0 = 28
            for line in text_lines:
                cv2.putText(
                    canvas,
                    line,
                    (16, y0),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (220, 220, 220),
                    2,
                    lineType=cv2.LINE_AA,
                )
                y0 += 28

            writer.write(canvas)

        writer.release()
        outputs[camera_id] = str(output_path)

    return outputs


def main() -> int:
    parser = argparse.ArgumentParser(description="Render per-camera 2D trace videos from pose capture JSONL")
    parser.add_argument("--log", default="logs/extrinsics_pose_capture.jsonl", help="Pose capture JSONL path")
    parser.add_argument("--output-dir", default="logs/pose_trace_videos", help="Directory for MP4 outputs")
    parser.add_argument("--fps", type=int, default=30, help="Output video FPS")
    parser.add_argument("--trail", type=int, default=20, help="Number of historical points to show")
    parser.add_argument("--radius", type=int, default=6, help="Blob marker radius in pixels")
    args = parser.parse_args()

    outputs = render_pose_log_videos(
        log_path=args.log,
        output_dir=args.output_dir,
        fps=args.fps,
        trail=args.trail,
        radius=args.radius,
    )
    print(json.dumps(outputs, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
