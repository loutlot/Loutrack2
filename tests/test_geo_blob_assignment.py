from __future__ import annotations

import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.geo import GeometryPipeline, Triangulator, create_dummy_calibration
from host.receiver import Frame, PairedFrames


def _project(camera, point: np.ndarray) -> tuple[float, float]:
    point_cam = camera.rotation @ point + camera.translation
    return (
        float(camera.fx * point_cam[0] / point_cam[2] + camera.cx),
        float(camera.fy * point_cam[1] / point_cam[2] + camera.cy),
    )


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


def _geometry_pipeline() -> GeometryPipeline:
    pipeline = GeometryPipeline()
    pipeline.camera_params = create_dummy_calibration(["cam0", "cam1"], focal_length=800.0)
    pipeline.triangulator = Triangulator(pipeline.camera_params)
    return pipeline


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

    matches = triangulator._match_blobs_epipolar(ref_blobs, other_blobs, np.eye(3))

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
    monkeypatch.setattr(pipeline.triangulator, "compute_reprojection_error", lambda *_args, **_kwargs: 9.0)

    result = pipeline.process_paired_frames(_paired(blobs_a, blobs_b))

    assert result["points_3d"] == []
    assert result["assignment_diagnostics"]["assignment_rejected_reprojection"] == 1
