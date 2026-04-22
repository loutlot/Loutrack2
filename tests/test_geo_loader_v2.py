from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import CalibrationLoader, GeometryPipeline


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


def test_v2_loader_reads_pose_camera_poses_and_pipeline_applies_them(tmp_path: Path) -> None:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")
    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "camera_order": ["pi-cam-01", "pi-cam-02"],
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": 0.95, "median_reproj_error_px": 0.4},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.2, 0.0, 0.0], "focal_scale": 1.125, "median_reproj_error_px": 0.6},
            ],
            "solve_summary": {"captured_rows": 30, "usable_rows": 22, "complete_rows": 20, "pair_overlaps": {"pi-cam-01|pi-cam-02": 22}, "median_reproj_error_px": 0.5, "p90_reproj_error_px": 1.0},
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text(json.dumps(payload), encoding="utf-8")

    extrinsics = CalibrationLoader.load_extrinsics(str(calibration_dir), include_meta=True)
    assert "pi-cam-02" in extrinsics
    assert extrinsics["pi-cam-02"]["t"] == [0.2, 0.0, 0.0]

    pipeline = GeometryPipeline()
    loaded = pipeline.load_calibration(str(calibration_dir))
    assert loaded == 2
    assert np.allclose(pipeline.camera_params["pi-cam-02"].translation, np.array([0.2, 0.0, 0.0], dtype=np.float64))
    cam_02 = pipeline.camera_params["pi-cam-02"]
    assert cam_02.focal_scale == 1.125
    assert cam_02.fx == 1012.5
    assert cam_02.fy == 1012.5
    assert cam_02.cx == 640.0
    assert cam_02.cy == 480.0
    assert np.allclose(cam_02.distortion_coeffs, np.zeros(5, dtype=np.float64))
    diagnostics = pipeline.get_diagnostics()
    assert diagnostics["cameras"]["pi-cam-02"]["focal_scale"] == 1.125
    assert diagnostics["cameras"]["pi-cam-02"]["fx"] == 1012.5


def test_v2_loader_prefers_metric_world_wand_origin_when_available(tmp_path: Path) -> None:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-02.json", "pi-cam-02")
    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0]},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.2, 0.0, 0.0]},
            ],
        },
        "metric": {
            "status": "resolved",
            "frame": "metric_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": 1.2},
                {"camera_id": "pi-cam-02", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.2, 0.0, 0.0], "focal_scale": 0.85},
            ],
        },
        "world": {
            "status": "resolved",
            "frame": "world",
            "to_world_matrix": [
                [1, 0, 0, 10],
                [0, 1, 0, 20],
                [0, 0, 1, 30],
                [0, 0, 0, 1],
            ],
            "origin_marker": "elbow",
        },
    }
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text(json.dumps(payload), encoding="utf-8")

    extrinsics = CalibrationLoader.load_extrinsics(str(calibration_dir), include_meta=True)

    assert extrinsics["pi-cam-01"]["coordinate_origin"] == "wand"
    assert extrinsics["pi-cam-01"]["frame"] == "world"
    assert extrinsics["pi-cam-01"]["focal_scale"] == 1.2
    assert np.allclose(extrinsics["pi-cam-01"]["t"], np.array([-10.0, -20.0, -30.0]))
    assert np.allclose(extrinsics["pi-cam-02"]["t"], np.array([-9.8, -20.0, -30.0]))

    pipeline = GeometryPipeline()
    loaded = pipeline.load_calibration(str(calibration_dir))
    assert loaded == 2
    assert pipeline.coordinate_origin == "wand"
    assert pipeline.coordinate_origin_source == "floor_metric_capture"
    assert pipeline.coordinate_frame == "world"
    assert np.allclose(pipeline.camera_params["pi-cam-01"].translation, np.array([-10.0, -20.0, -30.0]))
    assert pipeline.camera_params["pi-cam-01"].focal_scale == 1.2
    assert pipeline.camera_params["pi-cam-01"].fx == 1080.0
    assert pipeline.camera_params["pi-cam-02"].focal_scale == 0.85
    assert pipeline.camera_params["pi-cam-02"].fy == 765.0


def test_v2_loader_defaults_missing_focal_scale_to_one(tmp_path: Path) -> None:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0]},
            ],
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    (calibration_dir / "extrinsics_pose_v2.json").write_text(json.dumps(payload), encoding="utf-8")

    pipeline = GeometryPipeline()
    assert pipeline.load_calibration(str(calibration_dir)) == 1
    assert pipeline.camera_params["pi-cam-01"].focal_scale == 1.0
    assert pipeline.camera_params["pi-cam-01"].fx == 900.0


def _write_intrinsics_with_params(
    path: Path,
    camera_id: str,
    *,
    fx: float,
    cx: float,
    width: int,
    height: int,
) -> None:
    payload = {
        "schema_version": "1.0",
        "camera_id": camera_id,
        "resolution": {"width": width, "height": height},
        "camera_matrix": {
            "fx": fx,
            "fy": fx,
            "cx": cx,
            "cy": cx,
            "matrix": [[fx, 0.0, cx], [0.0, fx, cx], [0.0, 0.0, 1.0]],
        },
        "distortion_coefficients": {"array": [0.0, 0.0, 0.0, 0.0, 0.0]},
    }
    path.write_text(json.dumps(payload), encoding="utf-8")


def test_loader_skips_non_canonical_intrinsics_files(tmp_path: Path) -> None:
    """Stale duplicates (" copy.json", scratch files) must not shadow the canonical file.

    Regression: nondeterministic `path.glob('*.json')` order previously let a
    stale 2304x1296 "copy" file overwrite the current 1536x864 calibration for
    the same camera_id.
    """
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics_with_params(
        calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json",
        "pi-cam-01",
        fx=994.0, cx=768.0, width=1536, height=864,
    )
    _write_intrinsics_with_params(
        calibration_dir / "calibration_intrinsics_v1_pi-cam-01 copy.json",
        "pi-cam-01",
        fx=961.0, cx=1152.0, width=2304, height=1296,
    )
    _write_intrinsics_with_params(
        calibration_dir / "tmp_task3.json",
        "pi-cam-01",
        fx=4654.0, cx=1261.0, width=2304, height=1296,
    )

    with pytest.warns(RuntimeWarning, match="non-canonical intrinsics file"):
        loaded = CalibrationLoader.load_intrinsics(str(calibration_dir))

    assert set(loaded.keys()) == {"pi-cam-01"}
    assert loaded["pi-cam-01"]["camera_matrix"]["fx"] == 994.0
    assert loaded["pi-cam-01"]["resolution"] == {"width": 1536, "height": 864}


def test_loader_skips_file_whose_content_camera_id_mismatches_filename(tmp_path: Path) -> None:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics_with_params(
        calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json",
        "pi-cam-99",
        fx=500.0, cx=640.0, width=1280, height=960,
    )
    _write_intrinsics_with_params(
        calibration_dir / "calibration_intrinsics_v1_pi-cam-02.json",
        "pi-cam-02",
        fx=501.0, cx=641.0, width=1280, height=960,
    )

    with pytest.warns(RuntimeWarning, match="does not match content camera_id"):
        loaded = CalibrationLoader.load_intrinsics(str(calibration_dir))

    assert set(loaded.keys()) == {"pi-cam-02"}


@pytest.mark.parametrize("bad_scale", [0.0, -1.0, float("nan"), float("inf"), "not-a-number"])
def test_v2_loader_rejects_invalid_focal_scale(tmp_path: Path, bad_scale: object) -> None:
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    _write_intrinsics(calibration_dir / "calibration_intrinsics_v1_pi-cam-01.json", "pi-cam-01")
    payload = {
        "schema_version": "2.0",
        "method": "reference_pose_capture",
        "pose": {
            "frame": "similarity_camera",
            "camera_poses": [
                {"camera_id": "pi-cam-01", "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "t": [0.0, 0.0, 0.0], "focal_scale": bad_scale},
            ],
        },
        "metric": {"status": "unresolved", "scale_m_per_unit": None, "source": None},
        "world": {"status": "unresolved", "frame": None, "to_world_matrix": None, "floor_plane": None, "source": None},
    }
    (calibration_dir / "extrinsics_pose_v2.json").write_text(json.dumps(payload), encoding="utf-8")

    with pytest.raises(ValueError, match="invalid focal_scale"):
        GeometryPipeline().load_calibration(str(calibration_dir))
