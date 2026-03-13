from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))


_module_path = Path(__file__).resolve().parents[1] / "src" / "camera-calibration" / "calibrate_extrinsics.py"
_spec = importlib.util.spec_from_file_location("calibrate_extrinsics", _module_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError("Failed to load calibrate_extrinsics module")
_mod = importlib.util.module_from_spec(_spec)
sys.modules["calibrate_extrinsics"] = _mod
_spec.loader.exec_module(_mod)


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


def _write_pose_frame(
    handle,
    camera_id: str,
    frame_index: int,
    timestamp: int,
    blobs: list[dict[str, float]],
    blob_count: int | None = None,
) -> None:
    payload = {
        "_type": "frame",
        "data": {
            "camera_id": camera_id,
            "timestamp": timestamp,
            "frame_index": frame_index,
            "blobs": blobs,
        },
    }
    if blob_count is not None:
        payload["data"]["blob_count"] = blob_count
    handle.write(json.dumps(payload) + "\n")


def test_solver_outputs_v2_and_filters_non_single_blob_rows(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(intrinsics_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    rvec_cam2 = np.array([0.0, np.deg2rad(10.0), 0.0], dtype=np.float64)
    tvec_cam2 = np.array([0.30, 0.02, 0.01], dtype=np.float64)

    pose_log = tmp_path / "extrinsics_pose_capture.jsonl"
    with pose_log.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(28):
            point_w = np.array(
                [-0.20 + frame_index * 0.015, -0.04 + frame_index * 0.006, 1.8 + (frame_index % 4) * 0.05],
                dtype=np.float64,
            )
            uv1 = _project(point_w, np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64), k)
            uv2 = _project(point_w, rvec_cam2, tvec_cam2, k)
            timestamp = 1_700_000_000_000_000 + frame_index * 16_000
            if frame_index % 7 == 0:
                _write_pose_frame(
                    handle,
                    "pi-cam-01",
                    frame_index,
                    timestamp,
                    [
                        {"x": float(uv1[0]), "y": float(uv1[1]), "area": 80.0},
                        {"x": float(uv1[0] + 40.0), "y": float(uv1[1] + 10.0), "area": 20.0},
                    ],
                    blob_count=2,
                )
            else:
                _write_pose_frame(
                    handle,
                    "pi-cam-01",
                    frame_index,
                    timestamp,
                    [{"x": float(uv1[0]), "y": float(uv1[1]), "area": 80.0}],
                    blob_count=1,
                )
            _write_pose_frame(
                handle,
                "pi-cam-02",
                frame_index,
                timestamp + 900,
                [{"x": float(uv2[0]), "y": float(uv2[1]), "area": 80.0}],
                blob_count=1,
            )

    output_path = intrinsics_dir / "extrinsics_pose_v2.json"
    result = _mod.solve_extrinsics(
        intrinsics_path=intrinsics_dir,
        pose_log_path=pose_log,
        output_path=output_path,
        min_pairs=8,
    )

    assert output_path.exists()
    assert result["schema_version"] == "2.0"
    assert result["method"] == "reference_pose_capture"
    assert result["metric"]["status"] == "unresolved"
    assert result["world"]["status"] == "unresolved"
    assert result["pose"]["frame"] == "similarity_camera"
    assert result["pose"]["solve_summary"]["usable_rows"] >= 20
    assert len(result["pose"]["camera_poses"]) == 2


def test_camera_order_uses_pair_overlap_chain() -> None:
    def _obs(camera_id: str, timestamp: int) -> _mod.PoseObservation:
        return _mod.PoseObservation(
            camera_id=camera_id,
            timestamp=timestamp,
            frame_index=timestamp,
            image_point=np.array([100.0, 200.0], dtype=np.float64),
        )

    observations = {
        "cam-a": [_obs("cam-a", 1000 + idx * 10_000) for idx in range(20)],
        "cam-b": [_obs("cam-b", 1200 + idx * 10_000) for idx in range(20)],
        "cam-c": [_obs("cam-c", 1500 + idx * 10_000) for idx in range(15)] + [_obs("cam-c", 500_000 + idx * 20_000) for idx in range(5)],
    }
    camera_order, _, summary = _mod.build_camera_order_and_rows(observations, pair_window_us=3000)
    assert summary["pair_overlaps"]["cam-a|cam-b"] > summary["pair_overlaps"]["cam-b|cam-c"]
    adjacent_pairs = {tuple(camera_order[idx : idx + 2]) for idx in range(len(camera_order) - 1)}
    adjacent_pairs |= {tuple(reversed(pair)) for pair in adjacent_pairs}
    assert ("cam-a", "cam-b") in adjacent_pairs
    assert summary["pair_overlaps"]["cam-a|cam-b"] > summary["pair_overlaps"]["cam-a|cam-c"]


def test_three_camera_solver_recovers_pose_and_focal_bounds(tmp_path: Path) -> None:
    intrinsics_dir = tmp_path / "calibration"
    intrinsics_dir.mkdir()
    for camera_id in ("pi-cam-01", "pi-cam-02", "pi-cam-03"):
        _write_intrinsics(intrinsics_dir / f"calibration_intrinsics_v1_{camera_id}.json", camera_id)

    k = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    rvecs = {
        "pi-cam-01": np.zeros(3, dtype=np.float64),
        "pi-cam-02": np.array([0.0, np.deg2rad(8.0), 0.0], dtype=np.float64),
        "pi-cam-03": np.array([0.0, np.deg2rad(-8.0), 0.0], dtype=np.float64),
    }
    tvecs = {
        "pi-cam-01": np.zeros(3, dtype=np.float64),
        "pi-cam-02": np.array([0.32, 0.01, 0.00], dtype=np.float64),
        "pi-cam-03": np.array([-0.28, -0.02, 0.02], dtype=np.float64),
    }

    pose_log = tmp_path / "extrinsics_pose_capture_3cam.jsonl"
    with pose_log.open("w", encoding="utf-8") as handle:
        handle.write(json.dumps({"_type": "header", "schema_version": "1.0"}) + "\n")
        for frame_index in range(32):
            point_w = np.array(
                [-0.24 + frame_index * 0.012, -0.05 + frame_index * 0.007, 1.6 + (frame_index % 5) * 0.07],
                dtype=np.float64,
            )
            timestamp = 1_700_000_400_000_000 + frame_index * 16_000
            for idx, camera_id in enumerate(("pi-cam-01", "pi-cam-02", "pi-cam-03")):
                uv = _project(point_w, rvecs[camera_id], tvecs[camera_id], k)
                _write_pose_frame(
                    handle,
                    camera_id,
                    frame_index,
                    timestamp + idx * 700,
                    [{"x": float(uv[0]), "y": float(uv[1]), "area": 70.0}],
                    blob_count=1,
                )

    output_path = intrinsics_dir / "extrinsics_pose_v2.json"
    result = _mod.solve_extrinsics(
        intrinsics_path=intrinsics_dir,
        pose_log_path=pose_log,
        output_path=output_path,
        min_pairs=10,
    )
    assert len(result["pose"]["camera_poses"]) == 3
    assert result["pose"]["solve_summary"]["median_reproj_error_px"] is not None
    assert result["pose"]["solve_summary"]["median_reproj_error_px"] < 5.0
    for row in result["pose"]["camera_poses"]:
        assert 0.9 <= float(row["focal_scale"]) <= 1.1
