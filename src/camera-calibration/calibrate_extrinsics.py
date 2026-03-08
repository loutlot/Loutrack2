#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

SRC_ROOT = Path(__file__).resolve().parents[1]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from host.geo import CalibrationLoader, Triangulator
from host.wand_session import WAND_MARKER_DIAMETER_MM, WAND_NAME, WAND_POINTS_MM


DEFAULT_PAIR_WINDOW_US = 12000
DEFAULT_MIN_PAIRS = 8
DEFAULT_OUTPUT = "calibration/calibration_extrinsics_v1.json"


@dataclass(frozen=True)
class FrameObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    blobs: List[Dict[str, float]]


@dataclass(frozen=True)
class PairedObservation:
    timestamp_ref: int
    timestamp_other: int
    ref_points: np.ndarray
    other_points: np.ndarray


def _ordered_wand_points(blobs: Sequence[Dict[str, float]]) -> Optional[np.ndarray]:
    if len(blobs) < 3:
        return None
    top3 = sorted(blobs, key=lambda blob: float(blob.get("area", 0.0)), reverse=True)[:3]
    pts = np.array([[float(blob["x"]), float(blob["y"])] for blob in top3], dtype=np.float64)

    d01 = np.linalg.norm(pts[0] - pts[1])
    d02 = np.linalg.norm(pts[0] - pts[2])
    d12 = np.linalg.norm(pts[1] - pts[2])
    distances = {(0, 1): d01, (0, 2): d02, (1, 2): d12}
    longest_pair = max(distances, key=distances.get)
    elbow_index = ({0, 1, 2} - set(longest_pair)).pop()
    endpoint_indices = [index for index in (0, 1, 2) if index != elbow_index]

    dist_a = np.linalg.norm(pts[endpoint_indices[0]] - pts[elbow_index])
    dist_b = np.linalg.norm(pts[endpoint_indices[1]] - pts[elbow_index])
    short_index, long_index = (
        (endpoint_indices[0], endpoint_indices[1])
        if dist_a <= dist_b
        else (endpoint_indices[1], endpoint_indices[0])
    )

    return np.stack([pts[elbow_index], pts[short_index], pts[long_index]], axis=0)


def load_wand_log(path: str | Path) -> Dict[str, List[FrameObservation]]:
    observations: Dict[str, List[FrameObservation]] = {}
    with open(path, "r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line:
                continue
            entry = json.loads(line)
            if entry.get("_type") == "frame":
                frame = entry.get("data", {})
            elif "camera_id" in entry and "blobs" in entry:
                frame = entry
            else:
                continue
            camera_id = frame.get("camera_id")
            if not camera_id:
                continue
            observations.setdefault(camera_id, []).append(
                FrameObservation(
                    camera_id=str(camera_id),
                    timestamp=int(frame["timestamp"]),
                    frame_index=int(frame.get("frame_index", 0)),
                    blobs=list(frame.get("blobs", [])),
                )
            )
    for camera_id in observations:
        observations[camera_id].sort(key=lambda item: item.timestamp)
    return observations


def pair_observations(
    ref_frames: Sequence[FrameObservation],
    other_frames: Sequence[FrameObservation],
    pair_window_us: int,
) -> List[PairedObservation]:
    pairs: List[PairedObservation] = []
    j = 0
    for ref in ref_frames:
        while j < len(other_frames) and other_frames[j].timestamp < ref.timestamp - pair_window_us:
            j += 1
        candidates: List[Tuple[int, FrameObservation]] = []
        for k in (j - 1, j, j + 1):
            if 0 <= k < len(other_frames):
                other = other_frames[k]
                delta = abs(other.timestamp - ref.timestamp)
                if delta <= pair_window_us:
                    candidates.append((delta, other))
        if not candidates:
            continue
        _, best = min(candidates, key=lambda item: item[0])
        ref_pts = _ordered_wand_points(ref.blobs)
        other_pts = _ordered_wand_points(best.blobs)
        if ref_pts is None or other_pts is None:
            continue
        pairs.append(
            PairedObservation(
                timestamp_ref=ref.timestamp,
                timestamp_other=best.timestamp,
                ref_points=ref_pts,
                other_points=other_pts,
            )
        )
    return pairs


def _normalized_points(camera: Any, points: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(
        points.reshape(-1, 1, 2),
        camera.intrinsic_matrix,
        camera.distortion_coeffs,
    ).reshape(-1, 2)


def estimate_pair_extrinsics(
    ref_camera: Any,
    other_camera: Any,
    pairs: Sequence[PairedObservation],
    min_pairs: int,
) -> Dict[str, Any]:
    if len(pairs) < min_pairs:
        raise ValueError(f"Not enough paired observations: {len(pairs)}")

    ref_norm = np.concatenate([_normalized_points(ref_camera, pair.ref_points) for pair in pairs], axis=0)
    other_norm = np.concatenate([_normalized_points(other_camera, pair.other_points) for pair in pairs], axis=0)

    essential, mask = cv2.findEssentialMat(
        ref_norm,
        other_norm,
        focal=1.0,
        pp=(0.0, 0.0),
        method=cv2.RANSAC,
        prob=0.999,
        threshold=1e-3,
    )
    if essential is None or mask is None:
        raise ValueError("findEssentialMat failed")

    _, rotation, translation_unit, pose_mask = cv2.recoverPose(
        essential,
        ref_norm,
        other_norm,
    )
    inlier_mask = (pose_mask.ravel() > 0) if pose_mask is not None else np.ones(ref_norm.shape[0], dtype=bool)

    proj_ref = np.hstack([np.eye(3), np.zeros((3, 1))])
    proj_other = np.hstack([rotation, translation_unit.reshape(3, 1)])

    object_points_mm = np.array(WAND_POINTS_MM, dtype=np.float64)
    known_edges_mm = np.array(
        [
            np.linalg.norm(object_points_mm[1] - object_points_mm[0]),
            np.linalg.norm(object_points_mm[2] - object_points_mm[0]),
            np.linalg.norm(object_points_mm[2] - object_points_mm[1]),
        ],
        dtype=np.float64,
    )

    scale_candidates: List[float] = []
    reprojection_errors: List[float] = []

    for pair in pairs:
        und_ref = _normalized_points(ref_camera, pair.ref_points)
        und_other = _normalized_points(other_camera, pair.other_points)
        points_4d = cv2.triangulatePoints(proj_ref, proj_other, und_ref.T, und_other.T)
        points_3d_unit = (points_4d[:3] / points_4d[3]).T

        reconstructed_edges = np.array(
            [
                np.linalg.norm(points_3d_unit[1] - points_3d_unit[0]),
                np.linalg.norm(points_3d_unit[2] - points_3d_unit[0]),
                np.linalg.norm(points_3d_unit[2] - points_3d_unit[1]),
            ],
            dtype=np.float64,
        )
        valid = reconstructed_edges > 1e-9
        if np.count_nonzero(valid) < 2:
            continue
        scale_candidates.append(float(np.median((known_edges_mm[valid] / 1000.0) / reconstructed_edges[valid])))

    if not scale_candidates:
        raise ValueError("Failed to estimate metric scale")
    scale_m_per_unit = float(np.median(scale_candidates))
    translation_m = translation_unit.reshape(3) * scale_m_per_unit

    triangulator = Triangulator(
        {
            ref_camera.camera_id: ref_camera,
            other_camera.camera_id: other_camera,
        }
    )
    other_camera.rotation = rotation
    other_camera.translation = translation_m

    for pair in pairs:
        points_4d = cv2.triangulatePoints(
            proj_ref,
            np.hstack([rotation, translation_unit.reshape(3, 1)]),
            _normalized_points(ref_camera, pair.ref_points).T,
            _normalized_points(other_camera, pair.other_points).T,
        )
        points_3d_m = ((points_4d[:3] / points_4d[3]).T) * scale_m_per_unit
        for index in range(3):
            error = triangulator.compute_reprojection_error(
                [tuple(pair.ref_points[index]), tuple(pair.other_points[index])],
                points_3d_m[index],
                [ref_camera.camera_id, other_camera.camera_id],
            )
            if error is not None:
                reprojection_errors.append(float(error))

    inlier_ratio = float(np.mean(inlier_mask.astype(np.float64))) if inlier_mask.size else 0.0
    baseline_m = float(np.linalg.norm(translation_m))
    median_reproj_error_px = float(np.median(reprojection_errors)) if reprojection_errors else 0.0

    return {
        "rotation_matrix": rotation.tolist(),
        "translation_m": translation_m.tolist(),
        "essential_matrix": essential.tolist(),
        "quality": {
            "pair_count": len(pairs),
            "point_count": int(len(pairs) * 3),
            "inlier_ratio": inlier_ratio,
            "median_reproj_error_px": median_reproj_error_px,
            "baseline_m": baseline_m,
            "scale_m_per_unit": scale_m_per_unit,
        },
    }


def solve_wand_extrinsics(
    intrinsics_path: str | Path,
    log_path: str | Path,
    output_path: str | Path = DEFAULT_OUTPUT,
    pair_window_us: int = DEFAULT_PAIR_WINDOW_US,
    min_pairs: int = DEFAULT_MIN_PAIRS,
    reference_camera_id: Optional[str] = None,
    session_id: Optional[str] = None,
) -> Dict[str, Any]:
    calibrations = CalibrationLoader.load_intrinsics(str(intrinsics_path))
    if not calibrations:
        raise ValueError("No intrinsic calibration files found")

    observations = load_wand_log(log_path)
    available_cameras = [camera_id for camera_id in calibrations.keys() if camera_id in observations]
    if len(available_cameras) < 2:
        raise ValueError("At least two cameras with intrinsics and wand observations are required")

    ref_camera_id = reference_camera_id or sorted(available_cameras)[0]
    if ref_camera_id not in calibrations or ref_camera_id not in observations:
        raise ValueError(f"Reference camera '{ref_camera_id}' not found in shared inputs")

    camera_params = {
        camera_id: CalibrationLoader.to_camera_params(calib)
        for camera_id, calib in calibrations.items()
        if camera_id in available_cameras
    }
    ref_camera = camera_params[ref_camera_id]

    cameras_output = [
        {
            "camera_id": ref_camera_id,
            "rotation_matrix": np.eye(3, dtype=np.float64).tolist(),
            "translation_m": [0.0, 0.0, 0.0],
            "quality": {
                "pair_count": 0,
                "point_count": 0,
                "inlier_ratio": 1.0,
                "median_reproj_error_px": 0.0,
                "baseline_m": 0.0,
                "scale_m_per_unit": 1.0,
            },
        }
    ]

    for camera_id in sorted(available_cameras):
        if camera_id == ref_camera_id:
            continue
        pairs = pair_observations(observations[ref_camera_id], observations[camera_id], pair_window_us)
        estimate = estimate_pair_extrinsics(ref_camera, camera_params[camera_id], pairs, min_pairs=min_pairs)
        estimate["camera_id"] = camera_id
        cameras_output.append(estimate)

    result = {
        "schema_version": "1.0",
        "reference_camera_id": ref_camera_id,
        "created_at": datetime.now(timezone.utc).isoformat(),
        "wand": {
            "name": WAND_NAME,
            "marker_diameter_mm": WAND_MARKER_DIAMETER_MM,
            "points_mm": [list(point) for point in WAND_POINTS_MM],
        },
        "session_meta": {
            "session_id": session_id,
            "log_path": str(Path(log_path)),
            "pair_window_us": pair_window_us,
            "min_pairs": min_pairs,
            "target_camera_ids": sorted(available_cameras),
        },
        "cameras": cameras_output,
    }

    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Estimate camera extrinsics from wand capture logs")
    parser.add_argument("--intrinsics", required=True, help="Path to intrinsic calibration directory or file")
    parser.add_argument("--log", required=True, help="Path to wand capture JSONL log")
    parser.add_argument("--output", default=DEFAULT_OUTPUT, help=f"Output JSON path (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--pair-window-us", type=int, default=DEFAULT_PAIR_WINDOW_US, help="Maximum timestamp delta for pairing")
    parser.add_argument("--min-pairs", type=int, default=DEFAULT_MIN_PAIRS, help="Minimum paired observations per camera pair")
    parser.add_argument("--reference-camera", default=None, help="Reference camera ID")
    parser.add_argument("--session-id", default=None, help="Optional session ID for metadata")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    result = solve_wand_extrinsics(
        intrinsics_path=args.intrinsics,
        log_path=args.log,
        output_path=args.output,
        pair_window_us=args.pair_window_us,
        min_pairs=args.min_pairs,
        reference_camera_id=args.reference_camera,
        session_id=args.session_id,
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
