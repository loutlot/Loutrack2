from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


def _load_module():
    module_path = Path(__file__).resolve().parents[1] / "src" / "camera-calibration" / "analyze_pose_segments.py"
    spec = importlib.util.spec_from_file_location("analyze_pose_segments", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load analyze_pose_segments module")
    module = importlib.util.module_from_spec(spec)
    sys.modules["analyze_pose_segments"] = module
    spec.loader.exec_module(module)
    return module


def _load_filter_module():
    module_path = Path(__file__).resolve().parents[1] / "src" / "camera-calibration" / "filter_pose_log_segments.py"
    spec = importlib.util.spec_from_file_location("filter_pose_log_segments", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load filter_pose_log_segments module")
    module = importlib.util.module_from_spec(spec)
    sys.modules["filter_pose_log_segments"] = module
    spec.loader.exec_module(module)
    return module


def _write_intrinsics(path: Path, camera_id: str, width: int = 1280, height: int = 960) -> None:
    payload = {
        "schema_version": "1.0",
        "camera_id": camera_id,
        "resolution": {"width": width, "height": height},
        "camera_matrix": {
            "fx": 900.0,
            "fy": 900.0,
            "cx": width / 2.0,
            "cy": height / 2.0,
            "matrix": [[900.0, 0.0, width / 2.0], [0.0, 900.0, height / 2.0], [0.0, 0.0, 1.0]],
        },
        "distortion_coefficients": {"array": [0.0, 0.0, 0.0, 0.0, 0.0]},
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def _project(point_w: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, k: np.ndarray) -> np.ndarray:
    points_2d, _ = cv2.projectPoints(
        point_w.reshape(1, 3).astype(np.float32),
        rvec.astype(np.float32),
        tvec.astype(np.float32),
        k.astype(np.float32),
        np.zeros(5, dtype=np.float32),
    )
    return points_2d.reshape(2)


def test_segment_analysis_flags_time_dominant_error(tmp_path: Path) -> None:
    mod = _load_module()
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    extrinsics_path = intrinsics_dir / "extrinsics_pose_v2.json"
    extrinsics_payload = {
        "schema_version": "2.0",
        "camera_order": ["pi-cam-01", "pi-cam-02"],
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": 1.0},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.3, 0.0, 0.0], "focal_scale": 1.0},
            ],
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    extrinsics_path.write_text(json.dumps(extrinsics_payload), encoding="utf-8")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    with pose_log.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        point_w = np.array([0.05, 0.02, 2.0], dtype=np.float64)
        uv1 = _project(point_w, np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64), k)
        uv2_base = _project(point_w, np.zeros(3, dtype=np.float64), np.array([0.3, 0.0, 0.0], dtype=np.float64), k)
        for frame_index in range(40):
            timestamp = 1_700_010_000_000_000 + frame_index * 16_000
            uv2 = uv2_base.copy()
            if frame_index >= 20:
                phase = frame_index - 20
                uv2 = uv2 + np.array(
                    [
                        12.0 + (phase % 3) * 3.0,
                        -8.0 if phase % 2 == 0 else 9.0,
                    ],
                    dtype=np.float64,
                )
            for camera_id, uv, ts_offset in (
                ("pi-cam-01", uv1, 0),
                ("pi-cam-02", uv2, 900),
            ):
                payload = {
                    "_type": "frame",
                    "data": {
                        "camera_id": camera_id,
                        "timestamp": timestamp + ts_offset,
                        "frame_index": frame_index,
                        "blobs": [{"x": float(uv[0]), "y": float(uv[1]), "area": 80.0}],
                        "blob_count": 1,
                    },
                }
                handle.write(json.dumps(payload) + "\n")

    result = mod.analyze_pose_segments(
        intrinsics_path=intrinsics_dir,
        extrinsics_path=extrinsics_path,
        pose_log_path=pose_log,
        pair_window_us=2000,
        time_bins=4,
        position_bins=3,
    )

    assert result["diagnosis"]["likely_driver"] == "time"
    assert len(result["time_segments"]) == 4
    first_median = result["time_segments"][0]["median"]
    last_median = result["time_segments"][-1]["median"]
    assert first_median is not None
    assert last_median is not None
    assert float(last_median) > float(first_median)


def test_filter_pose_log_removes_bad_time_segments(tmp_path: Path) -> None:
    analysis_mod = _load_module()
    filter_mod = _load_filter_module()
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    extrinsics_path = intrinsics_dir / "extrinsics_pose_v2.json"
    extrinsics_payload = {
        "schema_version": "2.0",
        "camera_order": ["pi-cam-01", "pi-cam-02"],
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": 1.0},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.3, 0.0, 0.0], "focal_scale": 1.0},
            ],
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    extrinsics_path.write_text(json.dumps(extrinsics_payload), encoding="utf-8")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    with pose_log.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        point_w = np.array([0.05, 0.02, 2.0], dtype=np.float64)
        uv1 = _project(point_w, np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64), k)
        uv2_base = _project(point_w, np.zeros(3, dtype=np.float64), np.array([0.3, 0.0, 0.0], dtype=np.float64), k)
        for frame_index in range(40):
            timestamp = 1_700_010_000_000_000 + frame_index * 16_000
            uv2 = uv2_base.copy()
            if frame_index >= 20:
                uv2 = uv2 + np.array([15.0, -8.0 if frame_index % 2 == 0 else 9.0], dtype=np.float64)
            for camera_id, uv, ts_offset in (
                ("pi-cam-01", uv1, 0),
                ("pi-cam-02", uv2, 900),
            ):
                payload = {
                    "_type": "frame",
                    "data": {
                        "camera_id": camera_id,
                        "timestamp": timestamp + ts_offset,
                        "frame_index": frame_index,
                        "blobs": [{"x": float(uv[0]), "y": float(uv[1]), "area": 80.0}],
                        "blob_count": 1,
                    },
                }
                handle.write(json.dumps(payload) + "\n")

    filtered_pose_log = tmp_path / "extrinsics_pose_capture_filtered.jsonl"
    result = filter_mod.filter_pose_log_by_time_segments(
        intrinsics_path=intrinsics_dir,
        extrinsics_path=extrinsics_path,
        pose_log_path=pose_log,
        output_path=filtered_pose_log,
        pair_window_us=2000,
        time_bins=4,
        position_bins=3,
        median_ratio_threshold=1.5,
    )

    assert result["removed_frames"] > 0
    filtered_analysis = analysis_mod.analyze_pose_segments(
        intrinsics_path=intrinsics_dir,
        extrinsics_path=extrinsics_path,
        pose_log_path=filtered_pose_log,
        pair_window_us=2000,
        time_bins=4,
        position_bins=3,
    )
    assert filtered_analysis["global_row_error_px"]["median"] < result["analysis"]["global_row_error_px"]["median"]
