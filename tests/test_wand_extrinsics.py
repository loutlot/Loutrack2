from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import GeometryPipeline
from host.wand_session import WAND_POINTS_MM


_module_path = Path(__file__).resolve().parents[1] / "src" / "camera-calibration" / "calibrate_extrinsics.py"
_spec = importlib.util.spec_from_file_location("calibrate_extrinsics", _module_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError("Failed to load calibrate_extrinsics module")
_mod = importlib.util.module_from_spec(_spec)
sys.modules["calibrate_extrinsics"] = _mod
_spec.loader.exec_module(_mod)


def _write_intrinsics(path: Path, camera_id: str, width: int = 1280, height: int = 960) -> None:
    intrinsics = {
        "schema_version": "1.0",
        "camera_id": camera_id,
        "camera_model": "pinhole",
        "resolution": {"width": width, "height": height},
        "camera_matrix": {
            "fx": 900.0,
            "fy": 900.0,
            "cx": width / 2.0,
            "cy": height / 2.0,
            "matrix": [[900.0, 0.0, width / 2.0], [0.0, 900.0, height / 2.0], [0.0, 0.0, 1.0]],
        },
        "distortion_coefficients": {
            "k1": 0.0,
            "k2": 0.0,
            "p1": 0.0,
            "p2": 0.0,
            "k3": 0.0,
            "array": [0.0, 0.0, 0.0, 0.0, 0.0],
        },
        "rms_error": 0.1,
        "captured_at": "2026-03-08T00:00:00Z",
    }
    path.write_text(json.dumps(intrinsics), encoding="utf-8")


def _project_point(point_w: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, k: np.ndarray) -> np.ndarray:
    points_2d, _ = cv2.projectPoints(
        point_w.reshape(1, 3).astype(np.float32),
        rvec.astype(np.float32),
        tvec.astype(np.float32),
        k.astype(np.float32),
        np.zeros(5, dtype=np.float32),
    )
    return points_2d.reshape(2)


def _write_pose_frame(handle, camera_id: str, frame_index: int, timestamp: int, uv: np.ndarray) -> None:
    entry = {
        "_type": "frame",
        "data": {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "frame_index": frame_index,
            "blobs": [
                {"x": float(uv[0]), "y": float(uv[1]), "area": 80.0},
            ],
        },
    }
    handle.write(json.dumps(entry) + "\n")


def _write_wand_frame(handle, camera_id: str, frame_index: int, timestamp: int, points_2d: np.ndarray) -> None:
    entry = {
        "_type": "frame",
        "data": {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "frame_index": frame_index,
            "blobs": [
                {"x": float(points_2d[3][0]), "y": float(points_2d[3][1]), "area": 60.0},
                {"x": float(points_2d[0][0]), "y": float(points_2d[0][1]), "area": 58.0},
                {"x": float(points_2d[2][0]), "y": float(points_2d[2][1]), "area": 56.0},
                {"x": float(points_2d[1][0]), "y": float(points_2d[1][1]), "area": 54.0},
            ],
        },
    }
    handle.write(json.dumps(entry) + "\n")


def test_pose_capture_solver_and_geo_loader(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    rvec_cam2, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(15.0), 0.0], dtype=np.float64))
    tvec_cam2 = np.array([0.38, 0.03, 0.02], dtype=np.float64)

    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    with open(pose_log, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(24):
            point_w = np.array(
                [
                    -0.25 + frame_index * 0.018,
                    -0.05 + frame_index * 0.008,
                    1.6 + (frame_index % 5) * 0.08,
                ],
                dtype=np.float64,
            )
            uv1 = _project_point(point_w, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), k)
            uv2 = _project_point(point_w, rvec_cam2, tvec_cam2, k)
            _write_pose_frame(handle, "pi-cam-01", frame_index, 1_700_000_000_000_000 + frame_index * 16_000, uv1)
            _write_pose_frame(handle, "pi-cam-02", frame_index, 1_700_000_000_000_000 + frame_index * 16_000 + 900, uv2)

    output_path = intrinsics_dir / "calibration_extrinsics_v1.json"
    result = _mod.solve_extrinsics(
        intrinsics_path=intrinsics_dir,
        pose_log_path=pose_log,
        output_path=output_path,
        min_pairs=8,
    )
    assert output_path.exists()
    assert result["session_meta"]["method"] == "pose_capture"
    assert result["session_meta"]["sample_count"] >= 8
    assert result["session_meta"]["pose_validation"]["median_reproj_error_px"] is not None
    assert result["session_meta"]["wand_metric_validation"] is None
    assert len(result["cameras"]) >= 2

    geometry = GeometryPipeline()
    loaded_count = geometry.load_calibration(str(intrinsics_dir))
    assert loaded_count >= 2
    assert "pi-cam-02" in geometry.camera_params


def test_pose_validation_uses_all_accepted_samples_not_only_ba_subset(tmp_path: Path, monkeypatch) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    rvec_cam2, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(12.0), 0.0], dtype=np.float64))
    tvec_cam2 = np.array([0.32, 0.01, 0.03], dtype=np.float64)

    pose_log = tmp_path / "extrinsics_pose_capture_many.jsonl"
    with open(pose_log, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(30):
            point_w = np.array(
                [-0.24 + frame_index * 0.015, -0.06 + frame_index * 0.007, 1.5 + (frame_index % 4) * 0.09],
                dtype=np.float64,
            )
            uv1 = _project_point(point_w, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), k)
            uv2 = _project_point(point_w, rvec_cam2, tvec_cam2, k)
            _write_pose_frame(handle, "pi-cam-01", frame_index, 1_700_000_300_000_000 + frame_index * 16_000, uv1)
            _write_pose_frame(handle, "pi-cam-02", frame_index, 1_700_000_300_000_000 + frame_index * 16_000 + 700, uv2)

    original_validate = _mod.validate_pose_capture_extrinsics
    seen_sample_counts: list[int] = []

    def _recording_validate(*args, **kwargs):
        seen_sample_counts.append(len(kwargs["samples"]))
        return original_validate(*args, **kwargs)

    monkeypatch.setattr(_mod, "validate_pose_capture_extrinsics", _recording_validate)
    result = _mod.solve_extrinsics(
        intrinsics_path=intrinsics_dir,
        pose_log_path=pose_log,
        output_path=intrinsics_dir / "calibration_extrinsics_v1.json",
        min_pairs=8,
        max_ba_samples=4,
    )
    assert result["session_meta"]["accepted_sample_count"] > result["session_meta"]["sample_count"]
    assert seen_sample_counts == [result["session_meta"]["accepted_sample_count"]]


def test_wand_metric_alignment_metadata(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    rvec_cam2, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(15.0), 0.0], dtype=np.float64))
    tvec_cam2 = np.array([0.38, 0.03, 0.02], dtype=np.float64)

    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    with open(pose_log, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(24):
            point_w = np.array([-0.2 + frame_index * 0.015, -0.02 + frame_index * 0.004, 1.7], dtype=np.float64)
            uv1 = _project_point(point_w, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), k)
            uv2 = _project_point(point_w, rvec_cam2, tvec_cam2, k)
            _write_pose_frame(handle, "pi-cam-01", frame_index, 1_700_000_100_000_000 + frame_index * 16_000, uv1)
            _write_pose_frame(handle, "pi-cam-02", frame_index, 1_700_000_100_000_000 + frame_index * 16_000 + 900, uv2)

    wand_log = tmp_path / "extrinsics_wand_metric.jsonl"
    wand_model = np.array(WAND_POINTS_MM, dtype=np.float64) / 1000.0
    with open(wand_log, "w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(8):
            center = np.array([0.05, -0.03, 1.3 + frame_index * 0.01], dtype=np.float64)
            wand_world = wand_model + center
            p1, _ = cv2.projectPoints(wand_world, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), k, np.zeros(5))
            p2, _ = cv2.projectPoints(wand_world, rvec_cam2, tvec_cam2, k, np.zeros(5))
            _write_wand_frame(handle, "pi-cam-01", frame_index, 1_700_000_200_000_000 + frame_index * 16_000, p1.reshape(-1, 2))
            _write_wand_frame(handle, "pi-cam-02", frame_index, 1_700_000_200_000_000 + frame_index * 16_000 + 900, p2.reshape(-1, 2))

    output_path = intrinsics_dir / "calibration_extrinsics_v1.json"
    result = _mod.solve_extrinsics(
        intrinsics_path=intrinsics_dir,
        pose_log_path=pose_log,
        output_path=output_path,
        min_pairs=8,
    )
    assert result["session_meta"]["scale_source"] == "none"
    payload = json.loads(output_path.read_text(encoding="utf-8"))
    payload["session_meta"]["validation"] = {"legacy": True}
    output_path.write_text(json.dumps(payload), encoding="utf-8")

    metric_result = _mod.apply_wand_scale_floor(
        intrinsics_path=intrinsics_dir,
        extrinsics_path=output_path,
        wand_metric_log_path=wand_log,
        output_path=output_path,
    )
    assert metric_result["session_meta"]["scale_source"] in ("wand_floor_metric", "none")
    assert metric_result["session_meta"]["floor_source"] in ("wand_floor_metric", "none")
    if metric_result["session_meta"]["scale_source"] == "wand_floor_metric":
        assert metric_result["session_meta"].get("origin_marker") == "elbow"
    assert metric_result["session_meta"]["pose_validation"]["median_reproj_error_px"] is not None
    assert "wand_metric_validation" in metric_result["session_meta"]
    assert "validation" not in metric_result["session_meta"]
