from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import GeometryPipeline


_module_path = Path(__file__).resolve().parents[1] / "src" / "camera-calibration" / "calibrate_extrinsics.py"
_spec = importlib.util.spec_from_file_location("calibrate_extrinsics", _module_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError("Failed to load calibrate_extrinsics module")
_mod = importlib.util.module_from_spec(_spec)
sys.modules["calibrate_extrinsics"] = _mod
_spec.loader.exec_module(_mod)


WAND_POINTS_4_MM = (
    (0.0, 0.0, 0.0),
    (168.0, 0.0, 0.0),
    (84.0, 121.5, 0.0),
    (0.0, 243.0, 0.0),
)


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


def _project_points(object_points_m: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K: np.ndarray) -> np.ndarray:
    points_2d, _ = cv2.projectPoints(
        object_points_m.astype(np.float32),
        rvec.astype(np.float32),
        tvec.astype(np.float32),
        K.astype(np.float32),
        np.zeros(5, dtype=np.float32),
    )
    return points_2d.reshape(-1, 2)


def _write_frame_entry(
    handle: Any,
    camera_id: str,
    frame_index: int,
    timestamp: int,
    points: np.ndarray,
) -> None:
    reorder = [3, 0, 2, 1]
    entry = {
        "_type": "frame",
        "data": {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "frame_index": frame_index,
            "blobs": [
                {"x": float(points[idx][0]), "y": float(points[idx][1]), "area": 80.0 - j}
                for j, idx in enumerate(reorder)
            ],
        },
    }
    handle.write(json.dumps(entry) + "\n")


def test_solve_wand_extrinsics_and_load_geo(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    old_wand_points = getattr(_mod, "WAND_POINTS_MM")
    setattr(_mod, "WAND_POINTS_MM", WAND_POINTS_4_MM)

    object_points_m = np.array(WAND_POINTS_4_MM, dtype=np.float64) / 1000.0
    K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)

    angle = np.deg2rad(18.0)
    rotation_y = np.array(
        [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ],
        dtype=np.float64,
    )
    rvec_cam2, _ = cv2.Rodrigues(rotation_y)
    tvec_cam2 = np.array([0.42, 0.01, 0.03], dtype=np.float64)

    try:
        log_path = tmp_path / "wand_capture.jsonl"
        with open(log_path, "w", encoding="utf-8") as handle:
            handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
            for frame_index in range(16):
                center = np.array(
                    [
                        -0.12 + frame_index * 0.012,
                        -0.06 + frame_index * 0.008,
                        1.8 + (frame_index % 4) * 0.07,
                    ],
                    dtype=np.float64,
                )
                wand_rotation = cv2.Rodrigues(np.array([0.08 * frame_index, -0.03, 0.05], dtype=np.float64))[0]
                world_points = (wand_rotation @ object_points_m.T).T + center

                cam1_points = _project_points(world_points, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), K)
                cam2_points = _project_points(world_points, rvec_cam2, tvec_cam2, K)

                _write_frame_entry(
                    handle,
                    "pi-cam-01",
                    frame_index,
                    1_700_000_000_000_000 + frame_index * 16_000,
                    cam1_points,
                )
                _write_frame_entry(
                    handle,
                    "pi-cam-02",
                    frame_index,
                    1_700_000_000_000_900 + frame_index * 16_000,
                    cam2_points,
                )

        output_path = intrinsics_dir / "calibration_extrinsics_v1.json"
        result = _mod.solve_wand_extrinsics(
            intrinsics_path=intrinsics_dir,
            log_path=log_path,
            output_path=output_path,
            min_pairs=8,
        )

        assert output_path.exists()
        assert result["reference_camera_id"] == "pi-cam-01"
        assert result["session_meta"]["pair_window_us"] == 8000
        assert len(result["cameras"]) == 2

        cam2 = next(camera for camera in result["cameras"] if camera["camera_id"] == "pi-cam-02")
        estimated_t = np.array(cam2["translation_m"], dtype=np.float64)
        assert np.isclose(np.linalg.norm(estimated_t), np.linalg.norm(tvec_cam2), atol=0.08)
        assert cam2["quality"]["pair_count"] >= 8
        assert cam2["quality"]["point_count"] == cam2["quality"]["pair_count"] * 4
        assert cam2["quality"]["median_reproj_error_px"] < 2.0

        output_override = intrinsics_dir / "calibration_extrinsics_v1_override.json"
        result_override = _mod.solve_wand_extrinsics(
            intrinsics_path=intrinsics_dir,
            log_path=log_path,
            output_path=output_override,
            min_pairs=8,
            pair_window_us=5000,
        )
        assert result_override["session_meta"]["pair_window_us"] == 5000

        geometry = GeometryPipeline()
        loaded_count = geometry.load_calibration(str(intrinsics_dir))
        assert loaded_count == 2
        assert "pi-cam-02" in geometry.camera_params
        loaded_translation = geometry.camera_params["pi-cam-02"].translation
        assert np.allclose(loaded_translation, estimated_t, atol=1e-6)
    finally:
        setattr(_mod, "WAND_POINTS_MM", old_wand_points)


def test_solve_wand_extrinsics_excludes_camera_with_insufficient_samples(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-03.json", "pi-cam-03")

    old_wand_points = getattr(_mod, "WAND_POINTS_MM")
    setattr(_mod, "WAND_POINTS_MM", WAND_POINTS_4_MM)
    try:
        object_points_m = np.array(WAND_POINTS_4_MM, dtype=np.float64) / 1000.0
        K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        rvec_cam2, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(18.0), 0.0], dtype=np.float64))
        tvec_cam2 = np.array([0.42, 0.01, 0.03], dtype=np.float64)
        rvec_cam3, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(-12.0), 0.0], dtype=np.float64))
        tvec_cam3 = np.array([-0.35, -0.02, 0.04], dtype=np.float64)

        log_path = tmp_path / "wand_capture.jsonl"
        with open(log_path, "w", encoding="utf-8") as handle:
            handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
            for frame_index in range(16):
                center = np.array(
                    [
                        -0.10 + frame_index * 0.010,
                        -0.04 + frame_index * 0.006,
                        1.8 + (frame_index % 3) * 0.08,
                    ],
                    dtype=np.float64,
                )
                wand_rotation = cv2.Rodrigues(np.array([0.06 * frame_index, -0.02, 0.04], dtype=np.float64))[0]
                world_points = (wand_rotation @ object_points_m.T).T + center
                cam1_points = _project_points(world_points, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), K)
                cam2_points = _project_points(world_points, rvec_cam2, tvec_cam2, K)
                _write_frame_entry(handle, "pi-cam-01", frame_index, 1_700_000_000_000_000 + frame_index * 16_000, cam1_points)
                _write_frame_entry(handle, "pi-cam-02", frame_index, 1_700_000_000_000_900 + frame_index * 16_000, cam2_points)
                if frame_index < 3:
                    cam3_points = _project_points(world_points, rvec_cam3, tvec_cam3, K)
                    _write_frame_entry(handle, "pi-cam-03", frame_index, 1_700_000_000_001_100 + frame_index * 16_000, cam3_points)

        result = _mod.solve_wand_extrinsics(
            intrinsics_path=intrinsics_dir,
            log_path=log_path,
            output_path=intrinsics_dir / "calibration_extrinsics_v1.json",
            min_pairs=8,
        )
        out_camera_ids = [camera["camera_id"] for camera in result["cameras"]]
        assert out_camera_ids == ["pi-cam-01", "pi-cam-02"]
        assert "pi-cam-03" in result["session_meta"]["excluded_camera_ids"]
        assert result["session_meta"]["excluded_camera_reasons"]["pi-cam-03"].startswith("insufficient_samples:")
    finally:
        setattr(_mod, "WAND_POINTS_MM", old_wand_points)


def test_solve_wand_extrinsics_enforces_min_pairs_per_camera(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    old_wand_points = getattr(_mod, "WAND_POINTS_MM")
    setattr(_mod, "WAND_POINTS_MM", WAND_POINTS_4_MM)
    try:
        object_points_m = np.array(WAND_POINTS_4_MM, dtype=np.float64) / 1000.0
        K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        rvec_cam2, _ = cv2.Rodrigues(np.array([0.0, np.deg2rad(18.0), 0.0], dtype=np.float64))
        tvec_cam2 = np.array([0.42, 0.01, 0.03], dtype=np.float64)

        log_path = tmp_path / "wand_capture_short.jsonl"
        with open(log_path, "w", encoding="utf-8") as handle:
            handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
            for frame_index in range(4):
                center = np.array([-0.1 + frame_index * 0.02, -0.04, 1.8], dtype=np.float64)
                world_points = object_points_m + center
                cam1_points = _project_points(world_points, np.zeros((3, 1), dtype=np.float64), np.zeros(3, dtype=np.float64), K)
                cam2_points = _project_points(world_points, rvec_cam2, tvec_cam2, K)
                _write_frame_entry(handle, "pi-cam-01", frame_index, 1_700_000_000_000_000 + frame_index * 16_000, cam1_points)
                _write_frame_entry(handle, "pi-cam-02", frame_index, 1_700_000_000_000_900 + frame_index * 16_000, cam2_points)

        with pytest.raises(ValueError, match="Not enough paired observations"):
            _mod.solve_wand_extrinsics(
                intrinsics_path=intrinsics_dir,
                log_path=log_path,
                output_path=intrinsics_dir / "calibration_extrinsics_v1.json",
                min_pairs=8,
            )
    finally:
        setattr(_mod, "WAND_POINTS_MM", old_wand_points)


if __name__ == "__main__":
    from tempfile import TemporaryDirectory

    with TemporaryDirectory() as tmp:
        test_solve_wand_extrinsics_and_load_geo(Path(tmp))
    print("wand_extrinsics tests passed")
