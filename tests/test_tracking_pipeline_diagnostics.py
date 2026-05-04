from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.pipeline import TrackingPipeline
from host.receiver import Frame, PairedFrames
from host.rigid import RigidBodyPose
from host.wand_session import FIXED_PAIR_WINDOW_US


def _frame(camera_id: str, timestamp: int, received_at: float) -> Frame:
    return Frame(
        camera_id=camera_id,
        timestamp=timestamp,
        frame_index=None,
        blobs=[{"x": 10.0, "y": 20.0, "area": 4.0}],
        received_at=received_at,
        host_received_at_us=int(received_at * 1_000_000),
        timestamp_source="sensor_metadata",
        capture_to_process_ms=1.0,
        capture_to_send_ms=2.0,
    )


def _triangulation_quality(accepted_points: int = 1) -> dict[str, object]:
    return {
        "accepted_points": accepted_points,
        "contributing_rays": {
            "per_point": [2] * accepted_points,
            "summary": {
                "count": accepted_points,
                "mean": 2.0 if accepted_points else 0.0,
                "median": 2.0 if accepted_points else 0.0,
                "p90": 2.0 if accepted_points else 0.0,
                "p95": 2.0 if accepted_points else 0.0,
                "min": 2.0 if accepted_points else 0.0,
                "max": 2.0 if accepted_points else 0.0,
            },
        },
        "reprojection_error_px_summary": {
            "count": accepted_points,
            "mean": 0.25 if accepted_points else 0.0,
            "median": 0.25 if accepted_points else 0.0,
            "p90": 0.25 if accepted_points else 0.0,
            "p95": 0.25 if accepted_points else 0.0,
            "min": 0.25 if accepted_points else 0.0,
            "max": 0.25 if accepted_points else 0.0,
        },
        "epipolar_error_px_summary": {
            "count": accepted_points,
            "mean": 0.1 if accepted_points else 0.0,
            "median": 0.1 if accepted_points else 0.0,
            "p90": 0.1 if accepted_points else 0.0,
            "p95": 0.1 if accepted_points else 0.0,
            "min": 0.1 if accepted_points else 0.0,
            "max": 0.1 if accepted_points else 0.0,
        },
        "triangulation_angle_deg_summary": {
            "count": accepted_points,
            "mean": 3.0 if accepted_points else 0.0,
            "median": 3.0 if accepted_points else 0.0,
            "p90": 3.0 if accepted_points else 0.0,
            "p95": 3.0 if accepted_points else 0.0,
            "min": 3.0 if accepted_points else 0.0,
            "max": 3.0 if accepted_points else 0.0,
        },
        "assignment_diagnostics": {"assignment_matches": accepted_points},
    }


def test_tracking_pipeline_reports_stage_and_logger_diagnostics(tmp_path: Path) -> None:
    pipeline = TrackingPipeline(enable_logging=True, log_dir=str(tmp_path))
    pipeline._running = True
    pipeline._calibration_loaded = True
    assert pipeline.logger is not None
    log_path = Path(pipeline.logger.start_recording(session_name="pipeline_diagnostics"))
    pipeline._diagnostics_event_interval_s = 0.0

    class _Geometry:
        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = (min_inlier_views, object_gating)
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "observations_by_camera": {
                    "pi-cam-01": [
                        {
                            "camera_id": "pi-cam-01",
                            "blob_index": 0,
                            "raw_uv": [10.0, 20.0],
                            "undistorted_uv": [10.0, 20.0],
                            "area": 4.0,
                        }
                    ]
                },
                "triangulated_points": [
                    {
                        "point": [1.0, 2.0, 3.0],
                        "camera_ids": ["pi-cam-01", "pi-cam-02"],
                        "blob_indices": [0, 0],
                        "contributing_rays": 2,
                        "observations": [],
                        "reprojection_errors_px": [0.25, 0.25],
                        "epipolar_errors_px": [0.1],
                        "triangulation_angles_deg": [3.0],
                        "source": "generic",
                        "rigid_name": None,
                        "marker_idx": None,
                        "is_virtual": False,
                    }
                ],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {"assignment_matches": 1},
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, timestamp):
            return {
                "waist": RigidBodyPose(
                    timestamp=timestamp,
                    position=np.array([1.0, 2.0, 3.0]),
                    rotation=np.eye(3),
                    quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    rms_error=0.1,
                    observed_markers=1,
                    valid=True,
                )
            }

        def get_tracking_status(self):
            return {"ok": True}

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]
    callbacks = []
    pipeline.set_pose_callback(callbacks.append)

    now = time.time()
    pair = PairedFrames(
        timestamp=1_000_000,
        frames={
            "pi-cam-01": _frame("pi-cam-01", 1_000_000, now),
            "pi-cam-02": _frame("pi-cam-02", 1_000_100, now + 0.001),
        },
        timestamp_range_us=100,
    )

    pipeline._on_paired_frames(pair)
    status = pipeline.get_status()
    metadata = pipeline.logger.stop_recording()
    pipeline._running = False

    stage_ms = status["diagnostics"]["pipeline_stage_ms"]
    assert stage_ms["triangulation_ms"]["max"] >= 0.0
    assert stage_ms["rigid_ms"]["max"] >= 0.0
    assert stage_ms["metrics_update_ms"]["max"] >= 0.0
    assert stage_ms["log_enqueue_ms"]["max"] >= 0.0
    assert stage_ms["pose_callback_ms"]["max"] >= 0.0
    assert stage_ms["pipeline_pair_ms"]["max"] >= 0.0
    assert status["diagnostics"]["logger"]["recording"] is True
    assert status["diagnostics"]["tracking"] == {"ok": True}
    assert status["diagnostics"]["slow_pair_events"]
    assert status["diagnostics"]["slow_pair_events"][0]["blob_count"] == 2
    assert callbacks
    assert status["triangulation_quality"]["accepted_points"] == 1
    snapshot = pipeline.get_latest_triangulation_snapshot()
    assert snapshot["observations_by_camera"]["pi-cam-01"][0]["blob_index"] == 0
    assert snapshot["triangulated_points"][0]["point"] == [1.0, 2.0, 3.0]
    assert metadata["total_frames"] == 2

    events = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert any(
        entry.get("_type") == "event" and entry.get("event_type") == "tracking_diagnostics"
        for entry in events
    )
    diagnostics_events = [
        entry
        for entry in events
        if entry.get("_type") == "event" and entry.get("event_type") == "tracking_diagnostics"
    ]
    assert diagnostics_events[-1]["data"]["tracking"] == {"ok": True}


def test_fast_filtered_generic_falls_back_without_double_tracker_update() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCD")
    pipeline._running = True
    pipeline._calibration_loaded = True

    class _Geometry:
        camera_params = {}

        def __init__(self) -> None:
            self.calls = []

        def process_paired_frames(
            self,
            _paired_frames,
            *,
            min_inlier_views=2,
            object_gating=None,
            generic_blob_indices_by_camera=None,
        ):
            self.calls.append(generic_blob_indices_by_camera)
            if generic_blob_indices_by_camera is not None:
                return {
                    "points_3d": [],
                    "reprojection_errors": [],
                    "assignment_diagnostics": {},
                    "triangulation_quality": _triangulation_quality(0),
                    "rigid_hint_quality": {"by_rigid": {}},
                }
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {"assignment_matches": 1},
                "triangulation_quality": _triangulation_quality(),
                "rigid_hint_quality": {
                    "by_rigid": {
                        "waist": {
                            "accepted_points": 3,
                            "markers_with_two_or_more_rays": 3,
                            "invalid_assignments": 0,
                        }
                    },
                    "reprojection_error_px_summary": {"p95": 0.25},
                },
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _ObjectGatingConfig:
        min_enforced_markers = 3

    class _Rigid:
        object_gating_config = _ObjectGatingConfig()
        reprojection_match_gate_px = 12.0

        def __init__(self) -> None:
            self.process_count = 0

        def evaluate_object_conditioned_gating(self, **_kwargs):
            return {
                "waist": {
                    "evaluated": True,
                    "reason": "ok",
                    "assigned_marker_views": 6,
                    "markers_with_two_or_more_rays": 3,
                    "per_camera": {
                        "pi-cam-01": {
                            "assignments": [
                                {"marker_idx": 0, "blob_index": 0},
                                {"marker_idx": 1, "blob_index": 1},
                                {"marker_idx": 2, "blob_index": 2},
                            ]
                        },
                        "pi-cam-02": {
                            "assignments": [
                                {"marker_idx": 0, "blob_index": 0},
                                {"marker_idx": 1, "blob_index": 1},
                                {"marker_idx": 2, "blob_index": 2},
                            ]
                        },
                    },
                }
            }

        def process_context(self, _points, timestamp, **_kwargs):
            self.process_count += 1
            return {
                "waist": RigidBodyPose(
                    timestamp=timestamp,
                    position=np.array([1.0, 2.0, 3.0]),
                    rotation=np.eye(3),
                    quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    rms_error=0.1,
                    observed_markers=3,
                    valid=True,
                )
            }

        def get_tracking_status(self):
            return {"waist": {"valid": True}}

    geometry = _Geometry()
    rigid = _Rigid()
    pipeline.geometry = geometry  # type: ignore[assignment]
    pipeline.rigid_estimator = rigid  # type: ignore[assignment]
    now = time.time()
    pair = PairedFrames(
        timestamp=1_000_000,
        frames={
            "pi-cam-01": _frame("pi-cam-01", 1_000_000, now),
            "pi-cam-02": _frame("pi-cam-02", 1_000_100, now + 0.001),
        },
        timestamp_range_us=100,
    )

    pipeline._on_paired_frames(pair)
    pipeline._running = False
    status = pipeline.get_status()

    assert len(geometry.calls) == 2
    assert geometry.calls[0] is not None
    assert geometry.calls[1] is None
    assert rigid.process_count == 1
    assert status["diagnostics"]["fallback_summary"]["fallback_count"] == 1
    assert status["diagnostics"]["fallback_summary"]["fallback_reason_counts"]["no_rigid_hint_candidates"] == 1


def test_tracking_pipeline_logs_reacquire_guard_events(tmp_path: Path) -> None:
    pipeline = TrackingPipeline(
        enable_logging=True,
        log_dir=str(tmp_path),
        reacquire_guard_event_logging=True,
    )
    pipeline._running = True
    pipeline._calibration_loaded = True
    assert pipeline.logger is not None
    log_path = Path(pipeline.logger.start_recording(session_name="guard_events"))

    class _Geometry:
        camera_params = {}

        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = (min_inlier_views, object_gating)
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {"assignment_matches": 1},
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, timestamp):
            return {
                "waist": RigidBodyPose(
                    timestamp=timestamp,
                    position=np.array([1.0, 2.0, 3.0]),
                    rotation=np.eye(3),
                    quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                    valid=True,
                )
            }

        def get_tracking_status(self):
            return {
                "waist": {
                    "valid": True,
                    "mode": "reacquire",
                    "reacquire_guard": {
                        "evaluated": True,
                        "would_reject": True,
                        "passed": False,
                        "reason": "mean_reprojection_error_too_high",
                        "score": {"mean_error_px": 9.0},
                        "position_innovation_m": 0.01,
                        "rotation_innovation_deg": 2.0,
                        "enforced": False,
                        "rejected_count": 0,
                    },
                }
            }

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]

    now = time.time()
    pair = PairedFrames(
        timestamp=1_000_000,
        frames={
            "pi-cam-01": _frame("pi-cam-01", 1_000_000, now),
            "pi-cam-02": _frame("pi-cam-02", 1_000_100, now + 0.001),
        },
        timestamp_range_us=100,
    )
    pipeline._on_paired_frames(pair)
    metadata = pipeline.logger.stop_recording()
    pipeline._running = False

    assert metadata["total_frames"] == 2
    guard_events = pipeline.get_reacquire_guard_events()
    assert len(guard_events) == 1
    assert guard_events[0]["would_reject"] is True

    events = [
        json.loads(line)
        for line in log_path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert any(
        entry.get("_type") == "event" and entry.get("event_type") == "reacquire_guard"
        for entry in events
    )


def test_tracking_pipeline_uses_half_frame_timestamp_pairing_window() -> None:
    pipeline = TrackingPipeline(enable_logging=False)

    assert pipeline.frame_processor.pairer.timestamp_tolerance_us == FIXED_PAIR_WINDOW_US
    assert pipeline.frame_processor.pairer.frame_index_fallback is False
    assert pipeline.pipeline_variant == "fast_ABCD"
    assert pipeline.subset_diagnostics_mode == "sampled"


def test_tracking_pipeline_can_still_select_baseline_path() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="baseline")

    assert pipeline.pipeline_variant == "baseline"
    assert pipeline.subset_diagnostics_mode == "full"


def test_tracking_pipeline_fast_abcds_enables_subset_budget() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCDS")

    assert pipeline.pipeline_variant == "fast_ABCDS"
    assert pipeline.subset_diagnostics_mode == "sampled"
    assert pipeline.rigid_estimator.subset_time_budget_ms == 6.0
    assert pipeline.rigid_estimator.subset_max_hypotheses == 512


def test_tracking_pipeline_fast_abcdhrf_enables_combined_stress_path() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCDHRF")

    assert pipeline.pipeline_variant == "fast_ABCDHRF"
    assert pipeline.rigid_estimator.subset_time_budget_ms == 6.0
    assert pipeline.rigid_estimator.rigid_candidate_separation_enabled is True


def test_tracking_pipeline_breaks_down_object_gating_filter_reasons() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCDG")

    generic_filter, reason = pipeline._fast_generic_filter(
        {
            "waist": {
                "evaluated": True,
                "reason": "ok",
                "assigned_marker_views": 2,
                "markers_with_two_or_more_rays": 1,
                "single_ray_candidates": 0,
                "per_camera": {},
            }
        }
    )

    assert generic_filter is None
    assert reason == "insufficient_object_gating:assigned_marker_views_below_min"
    metrics = pipeline._variant_metrics_snapshot()
    assert metrics["object_gating_filter_reason_counts"]["assigned_marker_views_below_min"] == 1


def test_tracking_pipeline_filters_visible_rigids_when_one_relaxed_gate_is_partial() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCDHRF")

    generic_filter, reason = pipeline._fast_generic_filter(
        {
            "waist": {
                "evaluated": True,
                "reason": "ok",
                "assigned_marker_views": 0,
                "markers_with_two_or_more_rays": 0,
                "single_ray_candidates": 0,
                "per_camera": {
                    "pi-cam-01": {"assignments": []},
                    "pi-cam-02": {"assignments": []},
                },
            },
            "wand": {
                "evaluated": True,
                "reason": "ok",
                "assigned_marker_views": 8,
                "markers_with_two_or_more_rays": 4,
                "single_ray_candidates": 0,
                "per_camera": {
                    "pi-cam-01": {
                        "assignments": [
                            {"marker_idx": 0, "blob_index": 4},
                            {"marker_idx": 1, "blob_index": 7},
                            {"marker_idx": 2, "blob_index": 6},
                            {"marker_idx": 3, "blob_index": 5},
                        ]
                    },
                    "pi-cam-02": {
                        "assignments": [
                            {"marker_idx": 0, "blob_index": 6},
                            {"marker_idx": 1, "blob_index": 7},
                            {"marker_idx": 2, "blob_index": 5},
                            {"marker_idx": 3, "blob_index": 4},
                        ]
                    },
                },
            },
        }
    )

    assert generic_filter == {
        "pi-cam-01": {4, 5, 6, 7},
        "pi-cam-02": {4, 5, 6, 7},
    }
    assert reason == "partial_object_gating_filtered:assigned_marker_views_below_min"
    metrics = pipeline._variant_metrics_snapshot()
    assert metrics["object_gating_filter_reason_counts"]["assigned_marker_views_below_min"] == 1


def test_tracking_pipeline_falls_back_when_one_rigid_has_no_prediction() -> None:
    pipeline = TrackingPipeline(enable_logging=False, pipeline_variant="fast_ABCDHRF")

    generic_filter, reason = pipeline._fast_generic_filter(
        {
            "waist": {
                "evaluated": True,
                "reason": "no_prediction",
                "assigned_marker_views": 0,
                "markers_with_two_or_more_rays": 0,
                "single_ray_candidates": 0,
                "per_camera": {},
            },
            "wand": {
                "evaluated": True,
                "reason": "ok",
                "assigned_marker_views": 8,
                "markers_with_two_or_more_rays": 4,
                "single_ray_candidates": 0,
                "per_camera": {
                    "pi-cam-01": {
                        "assignments": [
                            {"marker_idx": 0, "blob_index": 4},
                            {"marker_idx": 1, "blob_index": 7},
                            {"marker_idx": 2, "blob_index": 6},
                            {"marker_idx": 3, "blob_index": 5},
                        ]
                    },
                    "pi-cam-02": {
                        "assignments": [
                            {"marker_idx": 0, "blob_index": 6},
                            {"marker_idx": 1, "blob_index": 7},
                            {"marker_idx": 2, "blob_index": 5},
                            {"marker_idx": 3, "blob_index": 4},
                        ]
                    },
                },
            },
        }
    )

    assert generic_filter is None
    assert reason == "partial_object_gating:gating_reason_no_prediction"


def test_tracking_pipeline_maps_rigid_stabilization_flags_to_configs() -> None:
    pipeline = TrackingPipeline(
        enable_logging=False,
        rigid_stabilization={
            "reacquire_guard_shadow_enabled": False,
            "reacquire_guard_enforced": True,
            "reacquire_guard_post_reacquire_frames": 1,
            "reacquire_guard_max_rotation_deg": 90.0,
            "object_conditioned_gating": False,
            "object_gating_enforced": True,
            "object_gating_activation_mode": "reacquire_only",
            "object_gating_ambiguous_blob_min_separation_px": 0.45,
            "object_gating_ambiguous_blob_diameter_overlap_ratio": 0.25,
            "object_gating_ambiguous_marker_assignment_min_margin_px": 0.20,
            "pose_continuity_guard_enabled": True,
            "pose_continuity_guard_enforced": True,
            "pose_continuity_max_rotation_deg": 45.0,
            "pose_continuity_max_angular_velocity_deg_s": 900.0,
            "pose_continuity_max_angular_accel_deg_s2": 12000.0,
            "position_continuity_guard_enabled": True,
            "position_continuity_guard_enforced": True,
            "position_continuity_max_accel_m_s2": 25.0,
            "position_continuity_max_velocity_m_s": 4.0,
            "subset_ransac": False,
            "reacquire_guard_event_logging": True,
        },
    )

    assert pipeline.reacquire_guard_event_logging is True
    assert pipeline.rigid_estimator.reacquire_guard_config.shadow_enabled is False
    assert pipeline.rigid_estimator.reacquire_guard_config.enforced is True
    assert pipeline.rigid_estimator.reacquire_guard_config.post_reacquire_continue_frames == 1
    assert pipeline.rigid_estimator.reacquire_guard_config.max_rotation_innovation_deg == 90.0
    assert pipeline.rigid_estimator.object_gating_config.enabled is False
    assert pipeline.rigid_estimator.object_gating_config.enforce is True
    assert pipeline.rigid_estimator.object_gating_config.activation_mode == "reacquire_only"
    assert pipeline.rigid_estimator.object_gating_config.ambiguous_blob_min_separation_px == 0.45
    assert pipeline.rigid_estimator.object_gating_config.ambiguous_blob_diameter_overlap_ratio == 0.25
    assert pipeline.rigid_estimator.object_gating_config.ambiguous_marker_assignment_min_margin_px == 0.20
    assert pipeline.rigid_estimator.pose_continuity_guard_config.enabled is True
    assert pipeline.rigid_estimator.pose_continuity_guard_config.enforced is True
    assert pipeline.rigid_estimator.pose_continuity_guard_config.max_rotation_innovation_deg == 45.0
    assert pipeline.rigid_estimator.pose_continuity_guard_config.max_angular_velocity_deg_s == 900.0
    assert pipeline.rigid_estimator.pose_continuity_guard_config.max_angular_accel_deg_s2 == 12000.0
    assert pipeline.rigid_estimator.position_continuity_guard_config.enabled is True
    assert pipeline.rigid_estimator.position_continuity_guard_config.enforced is True
    assert pipeline.rigid_estimator.position_continuity_guard_config.max_accel_m_s2 == 25.0
    assert pipeline.rigid_estimator.position_continuity_guard_config.max_velocity_m_s == 4.0
    assert pipeline.rigid_estimator.subset_solve_config.enabled is False

    disabled_ambiguous_guard = TrackingPipeline(
        enable_logging=False,
        rigid_stabilization={
            "object_gating_ambiguous_blob_min_separation_px": 0.0,
        },
    )
    disabled_config = disabled_ambiguous_guard.rigid_estimator.object_gating_config
    assert (
        disabled_config.ambiguous_blob_min_separation_px == 0.0
    )


def test_tracking_pipeline_keeps_fixed_pair_window_and_omits_sync_status() -> None:
    pipeline = TrackingPipeline(enable_logging=False)
    pipeline._running = True
    pipeline._calibration_loaded = True

    min_inlier_view_calls: list[int] = []

    class _Geometry:
        def process_paired_frames(self, _paired_frames, *, min_inlier_views=2, object_gating=None):
            _ = object_gating
            min_inlier_view_calls.append(int(min_inlier_views))
            return {
                "points_3d": [np.array([1.0, 2.0, 3.0])],
                "reprojection_errors": [0.25],
                "assignment_diagnostics": {
                    "assignment_matches": 1,
                    "dropped_views_for_inlier_fit": 0,
                },
                "triangulation_quality": _triangulation_quality(),
            }

        def get_diagnostics(self):
            return {"quality": _triangulation_quality()}

    class _Rigid:
        def process_points(self, _points, _timestamp):
            return {}

        def get_tracking_status(self):
            return {"ok": True}

    pipeline.geometry = _Geometry()  # type: ignore[assignment]
    pipeline.rigid_estimator = _Rigid()  # type: ignore[assignment]

    now = time.time()
    spreads = [200, 1500, 2200, 9000]
    for index, spread in enumerate(spreads):
        pair = PairedFrames(
            timestamp=1_000_000 + index * 10_000,
            frames={
                "pi-cam-01": _frame("pi-cam-01", 1_000_000 + index * 10_000, now + index * 0.01),
                "pi-cam-02": _frame(
                    "pi-cam-02",
                    1_000_000 + index * 10_000 + spread,
                    now + index * 0.01 + 0.001,
                ),
                "pi-cam-03": _frame(
                    "pi-cam-03",
                    1_000_000 + index * 10_000 + min(spread + 50, 2500),
                    now + index * 0.01 + 0.002,
                ),
            },
            timestamp_range_us=spread,
        )
        pipeline._on_paired_frames(pair)

    status = pipeline.get_status()
    snapshot = pipeline.get_latest_triangulation_snapshot()

    assert pipeline.frame_processor.pairer.timestamp_tolerance_us == FIXED_PAIR_WINDOW_US
    assert min_inlier_view_calls == [2, 2, 2, 2]
    assert "sync" not in status
    assert status["triangulation_quality"]["reprojection_error_px_summary"]["count"] == 1
    assert "sync_precision_mode" not in snapshot
    assert "epipolar_error_px_summary" in snapshot
    assert "triangulation_angle_deg_summary" in snapshot


def test_object_gating_events_include_enforcement_state() -> None:
    pipeline = TrackingPipeline(enable_logging=False)

    pipeline._record_object_gating_events(
        1_000_000,
        {
            "waist": {
                "evaluated": True,
                "mode": "continue",
                "enforced": True,
                "diagnostics_only": False,
                "reason": "ok",
                "confidence": 0.9,
                "pixel_gate_px": 4.0,
                "assigned_marker_views": 8,
                "candidate_window_count": 8,
                "markers_with_two_or_more_rays": 4,
                "single_ray_candidates": 0,
                "generic_fallback_blob_count": 0,
            }
        },
    )

    event = pipeline.get_object_gating_events()[0]
    assert event["enforced"] is True
    assert event["diagnostics_only"] is False
