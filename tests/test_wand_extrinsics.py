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


def _project_points(object_points_m: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, K: np.ndarray) -> np.ndarray:
    points_2d, _ = cv2.projectPoints(
        object_points_m.astype(np.float32),
        rvec.astype(np.float32),
        tvec.astype(np.float32),
        K.astype(np.float32),
        np.zeros(5, dtype=np.float32),
    )
    return points_2d.reshape(-1, 2)


def test_solve_wand_extrinsics_and_load_geo(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    object_points_m = np.array(WAND_POINTS_MM, dtype=np.float64) / 1000.0
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

            entry_cam1 = {
                "_type": "frame",
                "data": {
                    "camera_id": "pi-cam-01",
                    "timestamp": 1_700_000_000_000_000 + frame_index * 16_000,
                    "frame_index": frame_index,
                    "blobs": [{"x": float(x), "y": float(y), "area": 80.0 - idx} for idx, (x, y) in enumerate(cam1_points)],
                },
            }
            entry_cam2 = {
                "_type": "frame",
                "data": {
                    "camera_id": "pi-cam-02",
                    "timestamp": 1_700_000_000_000_900 + frame_index * 16_000,
                    "frame_index": frame_index,
                    "blobs": [{"x": float(x), "y": float(y), "area": 80.0 - idx} for idx, (x, y) in enumerate(cam2_points)],
                },
            }
            handle.write(json.dumps(entry_cam1) + "\n")
            handle.write(json.dumps(entry_cam2) + "\n")

    output_path = intrinsics_dir / "calibration_extrinsics_v1.json"
    result = _mod.solve_wand_extrinsics(
        intrinsics_path=intrinsics_dir,
        log_path=log_path,
        output_path=output_path,
        min_pairs=8,
    )

    assert output_path.exists()
    assert result["reference_camera_id"] == "pi-cam-01"
    assert len(result["cameras"]) == 2

    cam2 = next(camera for camera in result["cameras"] if camera["camera_id"] == "pi-cam-02")
    estimated_t = np.array(cam2["translation_m"], dtype=np.float64)
    assert np.isclose(np.linalg.norm(estimated_t), np.linalg.norm(tvec_cam2), atol=0.08)
    assert cam2["quality"]["pair_count"] >= 8
    assert cam2["quality"]["median_reproj_error_px"] < 2.0

    geometry = GeometryPipeline()
    loaded_count = geometry.load_calibration(str(intrinsics_dir))
    assert loaded_count == 2
    assert "pi-cam-02" in geometry.camera_params
    loaded_translation = geometry.camera_params["pi-cam-02"].translation
    assert np.allclose(loaded_translation, estimated_t, atol=1e-6)


if __name__ == "__main__":
    from tempfile import TemporaryDirectory

    with TemporaryDirectory() as tmp:
        test_solve_wand_extrinsics_and_load_geo(Path(tmp))
    print("wand_extrinsics tests passed")
