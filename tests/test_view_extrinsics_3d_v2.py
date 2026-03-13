from __future__ import annotations

import importlib.util
import json
import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")


def _write_intrinsics(path: Path, camera_id: str) -> None:
    payload = {
        "schema_version": "1.0",
        "camera_id": camera_id,
        "resolution": {"width": 1280, "height": 960},
        "camera_matrix": {
            "fx": 900.0,
            "fy": 900.0,
            "cx": 640.0,
            "cy": 480.0,
            "matrix": [[900.0, 0.0, 640.0], [0.0, 900.0, 480.0], [0.0, 0.0, 1.0]],
        },
        "distortion_coefficients": {"array": [0.0, 0.0, 0.0, 0.0, 0.0]},
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_viewer_renders_v2_pose_only_scene(tmp_path: Path, monkeypatch) -> None:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
    module_path = Path(__file__).resolve().parents[1] / "src" / "host" / "view_extrinsics_3d.py"
    spec = importlib.util.spec_from_file_location("view_extrinsics_3d", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError("Failed to load view_extrinsics_3d")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    for camera_id in ("pi-cam-01", "pi-cam-02"):
        _write_intrinsics(calibration_dir / f"calibration_intrinsics_v1_{camera_id}.json", camera_id)

    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "camera_order": ["pi-cam-01", "pi-cam-02"],
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": 1.0, "median_reproj_error_px": 0.4},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.2, 0.0, 0.0], "focal_scale": 1.0, "median_reproj_error_px": 0.6},
            ],
            "solve_summary": {"captured_rows": 30, "usable_rows": 22, "complete_rows": 20, "pair_overlaps": {"pi-cam-01|pi-cam-02": 22}, "median_reproj_error_px": 0.5, "p90_reproj_error_px": 1.0},
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text(json.dumps(payload), encoding="utf-8")
    output_path = tmp_path / "viewer.png"

    monkeypatch.setattr(module.plt, "show", lambda: None)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "view_extrinsics_3d.py",
            "--extrinsics",
            str(extrinsics_path),
            "--intrinsics-dir",
            str(calibration_dir),
            "--save",
            str(output_path),
        ],
    )
    module.main()
    assert output_path.exists()
