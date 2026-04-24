from __future__ import annotations

import os
import sys
import time

import cv2 as cv
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import (
    CameraParams,
    GeometryPipeline,
    Triangulator,
    create_dummy_calibration,
    normalize_epipolar_threshold_px,
)
from host.receiver import Frame, PairedFrames


def _project(camera, point: np.ndarray) -> tuple[float, float]:
    point_cam = camera.rotation @ point + camera.translation
    return (
        float(camera.fx * point_cam[0] / point_cam[2] + camera.cx),
        float(camera.fy * point_cam[1] / point_cam[2] + camera.cy),
    )


def _project_distorted(camera: CameraParams, point: np.ndarray) -> tuple[float, float]:
    rvec, _ = cv.Rodrigues(camera.rotation)
    projected, _ = cv.projectPoints(
        point.reshape(1, 3).astype(np.float32),
        rvec,
        camera.translation,
        camera.intrinsic_matrix,
        camera.distortion_coeffs,
    )
    coords = projected.reshape(-1)
    return float(coords[0]), float(coords[1])


def _blob(point: tuple[float, float]) -> dict[str, float]:
    return {"x": float(point[0]), "y": float(point[1]), "area": 4.0}


def _frame(camera_id: str, blobs: list[dict[str, float]]) -> Frame:
    now = time.time()
    return Frame(
        camera_id=camera_id,
        timestamp=1_000_000,
        frame_index=None,
        blobs=blobs,
        received_at=now,
        host_received_at_us=int(now * 1_000_000),
    )


def _paired(blobs_a: list[dict[str, float]], blobs_b: list[dict[str, float]]) -> PairedFrames:
    return PairedFrames(
        timestamp=1_000_000,
        frames={
            "cam0": _frame("cam0", blobs_a),
            "cam1": _frame("cam1", blobs_b),
        },
        timestamp_range_us=0,
    )


def _paired_multi(blobs_by_camera: dict[str, list[dict[str, float]]]) -> PairedFrames:
    return PairedFrames(
        timestamp=1_000_000,
        frames={
            camera_id: _frame(camera_id, blobs)
            for camera_id, blobs in blobs_by_camera.items()
        },
        timestamp_range_us=0,
    )


def _geometry_pipeline() -> GeometryPipeline:
    pipeline = GeometryPipeline()
    pipeline.camera_params = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    pipeline.triangulator = Triangulator(pipeline.camera_params)
    return pipeline


def _geometry_pipeline_with_params(params: dict[str, CameraParams]) -> GeometryPipeline:
    pipeline = GeometryPipeline()
    pipeline.camera_params = params
    pipeline.triangulator = Triangulator(params)
    pipeline.epipolar_threshold_px = pipeline.triangulator.epipolar_threshold_px
    return pipeline


def test_epipolar_threshold_defaults_to_tracking_slider_value() -> None:
    triangulator = Triangulator(create_dummy_calibration(["cam0", "cam1"]))

    assert triangulator.epipolar_threshold_px == 3.5
    assert normalize_epipolar_threshold_px(None) == 3.5
    assert normalize_epipolar_threshold_px(3.4) == 3.5
    assert normalize_epipolar_threshold_px(0.0) == 1.0
    assert normalize_epipolar_threshold_px(9.0) == 6.0


def test_epipolar_helper_uses_one_to_one_assignment(monkeypatch) -> None:
    triangulator = Triangulator(create_dummy_calibration(["cam0", "cam1"]))
    ref_blobs = [(float(i), 0.0) for i in range(4)]
    other_blobs = [(float(i), 0.0) for i in range(4)]
    # Rows 0 and 1 both prefer column 1. A greedy matcher would reuse it,
    # while linear assignment must pick distinct columns.
    costs = np.array(
        [
            [9.0, 1.0, 9.0, 9.0],
            [3.0, 2.0, 9.0, 9.0],
            [9.0, 9.0, 1.0, 9.0],
            [9.0, 9.0, 9.0, 1.0],
        ],
        dtype=np.float64,
    )

    monkeypatch.setattr(
        triangulator,
        "_epipolar_distance",
        lambda _f, pt1, pt2: float(costs[int(pt1[0]), int(pt2[0])]),
    )

    matches = triangulator._match_blobs_epipolar(
        ref_blobs,
        other_blobs,
        np.eye(3),
        threshold_px=10.0,
    )

    assert len(matches) == 4
    assert len(set(matches.values())) == 4
    assert matches[0] == 1
    assert matches[1] == 0


def test_epipolar_helper_scales_to_50_blobs_without_duplicate_matches(monkeypatch) -> None:
    triangulator = Triangulator(create_dummy_calibration(["cam0", "cam1"]))
    ref_blobs = [(float(i), 0.0) for i in range(50)]
    other_blobs = [(float(i), 0.0) for i in range(50)]

    monkeypatch.setattr(
        triangulator,
        "_epipolar_distance",
        lambda _f, pt1, pt2: abs(float(pt1[0]) - float(pt2[0])),
    )

    matches = triangulator._match_blobs_epipolar(ref_blobs, other_blobs, np.eye(3))

    assert len(matches) == 50
    assert len(set(matches.values())) == 50


def test_process_paired_frames_triangulates_shuffled_blobs_with_one_to_one_assignment() -> None:
    pipeline = _geometry_pipeline()
    params = pipeline.camera_params
    points_world = np.array(
        [
            [-0.10, -0.12, 2.5],
            [0.05, -0.04, 2.5],
            [0.11, 0.08, 2.5],
            [-0.05, 0.17, 2.5],
        ],
        dtype=np.float64,
    )
    blobs_a = [_blob(_project(params["cam0"], point)) for point in points_world]
    projected_b = [_blob(_project(params["cam1"], point)) for point in points_world]
    blobs_b = [projected_b[index] for index in [2, 0, 3, 1]]

    result = pipeline.process_paired_frames(_paired(blobs_a, blobs_b))
    reconstructed = np.asarray(result["points_3d"], dtype=np.float64)

    assert reconstructed.shape == (4, 3)
    assert result["assignment_diagnostics"]["duplicate_blob_matches"] == 0
    assert result["assignment_diagnostics"]["assignment_matches"] == 4
    for expected in points_world:
        assert np.min(np.linalg.norm(reconstructed - expected, axis=1)) < 1e-6


def test_process_paired_frames_exposes_observation_provenance() -> None:
    pipeline = _geometry_pipeline()
    params = pipeline.camera_params
    points_world = np.array(
        [
            [-0.10, -0.12, 2.5],
            [0.05, -0.04, 2.5],
        ],
        dtype=np.float64,
    )
    blobs_a = [_blob(_project(params["cam0"], point)) for point in points_world]
    blobs_b = [_blob(_project(params["cam1"], point)) for point in points_world]

    result = pipeline.process_paired_frames(_paired(blobs_a, blobs_b))

    assert len(result["points_3d"]) == 2
    assert set(result["observations_by_camera"]) == {"cam0", "cam1"}
    assert [obs["blob_index"] for obs in result["observations_by_camera"]["cam0"]] == [0, 1]
    assert "raw_uv" in result["observations_by_camera"]["cam0"][0]
    assert "undistorted_uv" in result["observations_by_camera"]["cam0"][0]

    triangulated_points = result["triangulated_points"]
    assert len(triangulated_points) == len(result["points_3d"])
    for point_payload, point_3d in zip(triangulated_points, result["points_3d"]):
        assert np.allclose(point_payload["point"], point_3d)
        assert point_payload["source"] == "generic"
        assert point_payload["is_virtual"] is False
        assert point_payload["contributing_rays"] == 2
        assert set(point_payload["camera_ids"]) == {"cam0", "cam1"}
        assert len(point_payload["observations"]) == 2
        assert len(point_payload["reprojection_errors_px"]) == 2


def test_process_paired_frames_does_not_force_unmatched_blob_into_3d_point() -> None:
    pipeline = _geometry_pipeline()
    params = pipeline.camera_params
    points_world = np.array(
        [
            [-0.10, -0.12, 2.5],
            [0.05, -0.04, 2.5],
            [0.11, 0.08, 2.5],
            [-0.05, 0.17, 2.5],
        ],
        dtype=np.float64,
    )
    blobs_a = [_blob(_project(params["cam0"], point)) for point in points_world]
    blobs_b = [_blob(_project(params["cam1"], point)) for point in points_world[:3]]

    result = pipeline.process_paired_frames(_paired(blobs_a, blobs_b))

    assert len(result["points_3d"]) == 3
    assert result["assignment_diagnostics"]["duplicate_blob_matches"] == 0
    assert result["assignment_diagnostics"]["assignment_matches"] == 3


def test_process_paired_frames_rejects_epipolar_outliers() -> None:
    pipeline = _geometry_pipeline()
    params = pipeline.camera_params
    point = np.array([0.0, 0.0, 2.5], dtype=np.float64)
    blob_a = _blob(_project(params["cam0"], point))
    blob_b = _blob(_project(params["cam1"], point))
    blob_b["y"] += 100.0

    result = pipeline.process_paired_frames(_paired([blob_a], [blob_b]))

    assert result["points_3d"] == []
    assert result["assignment_diagnostics"]["assignment_rejected_epipolar"] == 1


def test_process_paired_frames_rejects_high_reprojection_error(monkeypatch) -> None:
    pipeline = _geometry_pipeline()
    params = pipeline.camera_params
    point = np.array([0.0, 0.0, 2.5], dtype=np.float64)
    blobs_a = [_blob(_project(params["cam0"], point))]
    blobs_b = [_blob(_project(params["cam1"], point))]

    assert pipeline.triangulator is not None
    monkeypatch.setattr(
        pipeline.triangulator,
        "compute_reprojection_errors",
        lambda *_args, **_kwargs: [9.0, 9.0],
    )

    result = pipeline.process_paired_frames(_paired(blobs_a, blobs_b))

    assert result["points_3d"] == []
    assert result["assignment_diagnostics"]["assignment_rejected_reprojection"] == 1


def test_process_paired_frames_matches_on_undistorted_coordinates_for_distorted_cameras() -> None:
    params = create_dummy_calibration(["cam0", "cam1"], focal_length=900.0)
    distortion = np.array([-0.42, 0.18, 0.001, -0.001, 0.0], dtype=np.float64)
    for camera in params.values():
        camera.distortion_coeffs = distortion.copy()

    triangulator = Triangulator(params)
    F = triangulator._compute_fundamental_matrix("cam0", "cam1")
    assert F is not None

    selected: tuple[np.ndarray, tuple[float, float], tuple[float, float], float, float] | None = None
    for z in (1.6, 2.0, 2.4):
        for y in np.linspace(-0.45, 0.45, 19):
            for x in np.linspace(0.15, 0.95, 33):
                point = np.array([x, y, z], dtype=np.float64)
                raw0 = _project_distorted(params["cam0"], point)
                raw1 = _project_distorted(params["cam1"], point)
                width, height = params["cam0"].resolution
                if not (
                    0.0 <= raw0[0] <= width
                    and 0.0 <= raw0[1] <= height
                    and 0.0 <= raw1[0] <= width
                    and 0.0 <= raw1[1] <= height
                ):
                    continue
                raw_error = triangulator._epipolar_distance(F, raw0, raw1)
                und0 = triangulator._undistort_point("cam0", raw0)
                und1 = triangulator._undistort_point("cam1", raw1)
                und_error = triangulator._epipolar_distance(F, und0, und1)
                if raw_error > triangulator.epipolar_threshold_px and und_error < 0.1:
                    selected = (point, raw0, raw1, raw_error, und_error)
                    break
            if selected is not None:
                break
        if selected is not None:
            break

    assert selected is not None
    point, raw0, raw1, raw_error, und_error = selected

    pipeline = _geometry_pipeline_with_params(params)
    result = pipeline.process_paired_frames(
        _paired_multi({"cam0": [_blob(raw0)], "cam1": [_blob(raw1)]})
    )
    reconstructed = np.asarray(result["points_3d"], dtype=np.float64)

    assert raw_error > triangulator.epipolar_threshold_px
    assert und_error < 0.1
    assert reconstructed.shape == (1, 3)
    assert np.linalg.norm(reconstructed[0] - point) < 1e-3


def test_process_paired_frames_requires_ba_focal_scale_for_epipolar_match() -> None:
    point = np.array([0.08, 0.24, 2.5], dtype=np.float64)
    scaled_params = create_dummy_calibration(["cam0", "cam1"], focal_length=900.0)
    scaled_params["cam0"].focal_scale = 1.2
    scaled_params["cam1"].focal_scale = 0.85
    for camera in scaled_params.values():
        camera.intrinsic_matrix[0, 0] *= camera.focal_scale
        camera.intrinsic_matrix[1, 1] *= camera.focal_scale

    blobs = {
        "cam0": [_blob(_project(scaled_params["cam0"], point))],
        "cam1": [_blob(_project(scaled_params["cam1"], point))],
    }

    unscaled_pipeline = _geometry_pipeline_with_params(
        create_dummy_calibration(["cam0", "cam1"], focal_length=900.0)
    )
    rejected = unscaled_pipeline.process_paired_frames(_paired_multi(blobs))
    assert rejected["points_3d"] == []
    assert rejected["assignment_diagnostics"]["assignment_rejected_epipolar"] == 1

    scaled_pipeline = _geometry_pipeline_with_params(scaled_params)
    accepted = scaled_pipeline.process_paired_frames(_paired_multi(blobs))
    reconstructed = np.asarray(accepted["points_3d"], dtype=np.float64)

    assert accepted["assignment_diagnostics"]["assignment_matches"] == 1
    assert reconstructed.shape == (1, 3)
    assert np.linalg.norm(reconstructed[0] - point) < 1e-6


def test_process_paired_frames_drops_worst_view_and_keeps_inlier_fit() -> None:
    params = create_dummy_calibration(["cam0", "cam1", "cam2", "cam3"], focal_length=800.0)
    pipeline = _geometry_pipeline_with_params(params)
    point = np.array([0.04, 0.03, 2.5], dtype=np.float64)

    blobs = {
        camera_id: [_blob(_project(camera, point))]
        for camera_id, camera in params.items()
    }
    blobs["cam1"][0]["x"] += 18.0

    result = pipeline.process_paired_frames(_paired_multi(blobs))
    reconstructed = np.asarray(result["points_3d"], dtype=np.float64)

    assert reconstructed.shape == (1, 3)
    assert np.linalg.norm(reconstructed[0] - point) < 1e-3
    assert result["assignment_diagnostics"]["dropped_views_for_inlier_fit"] == 1
    assert result["triangulation_quality"]["accepted_points"] == 1
    assert result["triangulation_quality"]["contributing_rays"]["per_point"] == [3]


def test_process_paired_frames_rejects_low_parallax_candidates() -> None:
    params = create_dummy_calibration(["cam0", "cam1"], focal_length=900.0)
    params["cam1"].translation = np.array([-0.02, 0.0, 0.0], dtype=np.float64)
    pipeline = _geometry_pipeline_with_params(params)
    point = np.array([0.0, 0.0, 5.0], dtype=np.float64)

    result = pipeline.process_paired_frames(
        _paired_multi(
            {
                "cam0": [_blob(_project(params["cam0"], point))],
                "cam1": [_blob(_project(params["cam1"], point))],
            }
        )
    )

    assert result["points_3d"] == []
    assert result["assignment_diagnostics"]["rejected_low_parallax"] == 1
