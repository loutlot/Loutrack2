from __future__ import annotations

"""Legacy pose/wand capture readers kept for older sample utilities.

The supported v2 extrinsics flow is `calibrate_extrinsics.py`, which consumes
full-blobs `pose_capture` rows and rejects non-single-blob observations on the
host side. This module remains for compatibility with older sample-building
tools that still collapse pose rows to a single best blob.
"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

from wand_label import canonicalize_wand_points


@dataclass(frozen=True)
class PoseCaptureObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_point: np.ndarray
    blob_area: float
    blob_count: int
    quality: float


@dataclass(frozen=True)
class WandMetricObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_points: np.ndarray
    raw_points: np.ndarray
    confidence: float


def _extract_frame_payload(entry: Dict[str, Any]) -> Dict[str, Any] | None:
    if entry.get("_type") == "frame" and isinstance(entry.get("data"), dict):
        return entry["data"]
    if "camera_id" in entry and "timestamp" in entry and "blobs" in entry:
        return entry
    return None


def _pick_best_blob(blobs: Sequence[Dict[str, Any]]) -> tuple[Dict[str, Any] | None, float]:
    if not blobs:
        return None, 0.0
    ranked = sorted(blobs, key=lambda item: float(item.get("area", 0.0)), reverse=True)
    best = ranked[0]
    blob_count = max(1, len(blobs))
    quality = 1.0 / float(blob_count)
    if float(best.get("area", 0.0)) <= 0.0:
        quality *= 0.25
    return best, float(max(0.0, min(1.0, quality)))


def load_pose_capture_observations(path: str | Path) -> Dict[str, List[PoseCaptureObservation]]:
    rows: Dict[str, List[PoseCaptureObservation]] = {}
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
            blobs = payload.get("blobs", [])
            if not isinstance(blobs, list):
                continue
            best_blob, quality = _pick_best_blob(blobs)
            if best_blob is None:
                continue
            payload_blob_count = payload.get("blob_count")
            if isinstance(payload_blob_count, (int, float)) and int(payload_blob_count) > 0:
                blob_count = int(payload_blob_count)
            else:
                blob_count = len(blobs)
            payload_quality = payload.get("quality")
            if isinstance(payload_quality, (int, float)):
                quality = float(max(0.0, min(1.0, float(payload_quality))))
            rows.setdefault(camera_id, []).append(
                PoseCaptureObservation(
                    camera_id=camera_id,
                    timestamp=int(payload.get("timestamp", 0)),
                    frame_index=int(payload.get("frame_index", 0)),
                    image_point=np.array(
                        [float(best_blob.get("x", 0.0)), float(best_blob.get("y", 0.0))],
                        dtype=np.float64,
                    ),
                    blob_area=float(best_blob.get("area", 0.0)),
                    blob_count=blob_count,
                    quality=quality,
                )
            )
    for camera_id in rows:
        rows[camera_id].sort(key=lambda item: item.timestamp)
    return rows


def load_wand_metric_observations(path: str | Path) -> Dict[str, List[WandMetricObservation]]:
    rows: Dict[str, List[WandMetricObservation]] = {}
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
            blobs = payload.get("blobs", [])
            if not isinstance(blobs, list) or len(blobs) < 4:
                continue
            top4 = sorted(blobs, key=lambda blob: float(blob.get("area", 0.0)), reverse=True)[:4]
            raw_points = np.array(
                [[float(blob.get("x", 0.0)), float(blob.get("y", 0.0))] for blob in top4],
                dtype=np.float64,
            )
            label = canonicalize_wand_points(blobs)
            if label is None:
                image_points = raw_points
                confidence = 0.0
            else:
                image_points = label.points.astype(np.float64)
                confidence = float(max(0.0, 1.0 - label.collinearity_error * 6.0))
            rows.setdefault(camera_id, []).append(
                WandMetricObservation(
                    camera_id=camera_id,
                    timestamp=int(payload.get("timestamp", 0)),
                    frame_index=int(payload.get("frame_index", 0)),
                    image_points=image_points,
                    raw_points=raw_points,
                    confidence=confidence,
                )
            )
    for camera_id in rows:
        rows[camera_id].sort(key=lambda item: item.timestamp)
    return rows


def summarize_pose_capture(observations: Dict[str, List[PoseCaptureObservation]]) -> Dict[str, Any]:
    total = sum(len(items) for items in observations.values())
    all_quality = [obs.quality for items in observations.values() for obs in items]
    all_blob_counts = [obs.blob_count for items in observations.values() for obs in items]
    return {
        "camera_count": len(observations),
        "observation_count": total,
        "quality_p50": float(np.percentile(all_quality, 50)) if all_quality else 0.0,
        "quality_p90": float(np.percentile(all_quality, 90)) if all_quality else 0.0,
        "single_blob_ratio": float(np.mean(np.array(all_blob_counts) == 1)) if all_blob_counts else 0.0,
    }
