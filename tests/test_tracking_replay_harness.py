from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host import tracking_replay_harness as harness


@dataclass
class _Entry:
    received_at: str
    data: dict


@dataclass
class _Event:
    event_type: str
    timestamp: str
    data: dict


class _FakeReplay:
    def __init__(self, _log_path: str) -> None:
        self.entries = [
            _Entry(
                received_at="2026-04-24T00:00:00",
                data={
                    "camera_id": "pi-cam-01",
                    "timestamp": 1000,
                    "host_received_at_us": 10_000,
                    "blobs": [{"x": 1.0, "y": 2.0, "area": 3.0}],
                },
            ),
            _Entry(
                received_at="2026-04-24T00:00:00.001000",
                data={
                    "camera_id": "pi-cam-02",
                    "timestamp": 1100,
                    "host_received_at_us": 11_000,
                    "blobs": [{"x": 4.0, "y": 5.0, "area": 6.0}],
                },
            ),
        ]
        self._events = []

    def replay(self, *, realtime: bool = False):
        assert realtime is False
        yield from self.entries

    @property
    def events(self):
        return list(self._events)


class _FakeDiagnosticsReplay(_FakeReplay):
    def __init__(self, _log_path: str) -> None:
        super().__init__(_log_path)
        self._events = [
            _Event(
                event_type="tracking_diagnostics",
                timestamp="2026-04-24T00:00:01",
                data={
                    "geometry": {
                        "quality": {
                            "accepted_points": 4,
                            "reprojection_error_px_summary": {"mean": 0.2},
                            "epipolar_error_px_summary": {"mean": 0.3},
                        }
                    },
                    "pipeline_stage_ms": {
                        "rigid_ms": {"last": 0.5},
                        "pipeline_pair_ms": {"last": 2.0},
                    },
                    "tracking": {
                        "waist": {
                            "valid": True,
                            "mode": "continue",
                            "observed_markers": 4,
                            "max_pose_jump_m": 0.0,
                            "max_pose_flip_deg": 0.0,
                        }
                    },
                },
            ),
            _Event(
                event_type="tracking_diagnostics",
                timestamp="2026-04-24T00:00:03",
                data={
                    "geometry": {
                        "quality": {
                            "accepted_points": 3,
                            "reprojection_error_px_summary": {"mean": 0.8},
                            "epipolar_error_px_summary": {"mean": 1.2},
                        }
                    },
                    "pipeline_stage_ms": {
                        "rigid_ms": {"last": 1.5},
                        "pipeline_pair_ms": {"last": 4.0},
                    },
                    "tracking": {
                        "waist": {
                            "valid": True,
                            "mode": "continue",
                            "observed_markers": 3,
                            "max_pose_jump_m": 0.12,
                            "max_pose_flip_deg": 120.0,
                            "last_mode_reason": "continue_measurement_accepted",
                        }
                    },
                },
            ),
            _Event(
                event_type="tracking_diagnostics",
                timestamp="2026-04-24T00:00:05",
                data={
                    "geometry": {
                        "quality": {
                            "accepted_points": 2,
                            "reprojection_error_px_summary": {"mean": 0.0},
                            "epipolar_error_px_summary": {"mean": 0.0},
                        }
                    },
                    "tracking": {
                        "waist": {
                            "valid": False,
                            "mode": "reacquire",
                            "observed_markers": 2,
                            "max_pose_jump_m": 0.12,
                            "max_pose_flip_deg": 120.0,
                            "invalid_reason": "no_valid_candidate",
                        }
                    },
                },
            ),
        ]


class _FakeFrameProcessor:
    def __init__(self) -> None:
        self.callback = None
        self.frames = []
        self.pairs_emitted = 0

    def set_paired_callback(self, callback):
        self.callback = callback

    def _on_frame_received(self, frame):
        self.frames.append(frame)
        if len(self.frames) == 2 and self.callback is not None:
            self.pairs_emitted += 1
            self.callback({"paired": True})

    def _process_pairs(self):
        return None

    def get_stats(self):
        return {"pairer": {"pairs_emitted": self.pairs_emitted}}


class _FakePipeline:
    last_kwargs = {}

    def __init__(self, **_kwargs) -> None:
        type(self).last_kwargs = dict(_kwargs)
        self._calibration_loaded = True
        self._running = False
        self.frame_processor = _FakeFrameProcessor()
        self.frames_processed = 0
        self.poses_estimated = 0

    def _on_paired_frames(self, _paired_frames):
        self.frames_processed += 1
        self.poses_estimated += 1

    def get_status(self):
        return {
            "frames_processed": self.frames_processed,
            "poses_estimated": self.poses_estimated,
            "receiver": self.frame_processor.get_stats(),
            "tracking": {
                "waist": {
                    "valid": True,
                    "reacquire_count": 0,
                    "object_gating": {
                        "enabled": True,
                        "enforced": True,
                        "diagnostics_only": False,
                        "evaluated": True,
                        "reason": "ok",
                        "assigned_marker_views": 8,
                        "candidate_window_count": 8,
                    },
                }
            },
            "diagnostics": {
                "geometry": {"quality": {"accepted_points": 4}},
                "pipeline_stage_ms": {"rigid_ms": {"p95": 0.1}},
            },
        }

    def get_rigid_hint_events(self):
        return [
            {
                "rigid_name": "waist",
                "candidate_markers": 4,
                "markers_with_two_or_more_rays": 4,
                "single_ray_candidates": 0,
                "accepted_points": 4,
                "rejected_markers": 0,
                "invalid_assignments": 0,
                "reprojection_mean_px": 0.5,
                "reprojection_p95_px": 0.75,
            }
        ]

    def get_rigid_hint_pose_events(self):
        return [
            {
                "rigid_name": "waist",
                "valid": True,
                "generic_valid": True,
                "would_improve_score": False,
                "candidate_points": 4,
                "observed_markers": 4,
                "real_ray_count": 8,
                "virtual_marker_count": 0,
                "score_delta": 0.0,
                "position_delta_m": 0.0,
                "rotation_delta_deg": 0.0,
                "p95_error_px": 0.75,
                "reason": "ok",
            }
        ]

    def get_subset_hypothesis_events(self):
        return [
            {
                "rigid_name": "waist",
                "candidate_count": 12,
                "pruned_candidate_count": 2,
                "valid_candidate_count": 4,
                "rejected_by_ambiguity": 2,
                "rejected_by_2d_score": 3,
                "rejected_by_rms": 1,
                "flip_risk_count": 0,
                "truncated": False,
                "best_source": "rigid_hint_subset",
                "best_score": 0.99,
                "second_score": 0.91,
                "margin": 0.08,
                "generic_score": 0.95,
                "score_delta": 0.04,
                "best_p95_error_px": 0.7,
                "best_position_delta_m": 0.0,
                "best_rotation_delta_deg": 0.0,
                "subset_adoption_ready": True,
                "diagnostics_only": True,
            }
        ]


def test_replay_tracking_log_injects_frames_through_frame_processor(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    summary = harness.replay_tracking_log(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
    ).to_dict()

    assert summary["frame_count"] == 2
    assert summary["pair_count"] == 1
    assert summary["frames_processed"] == 1
    assert summary["poses_estimated"] == 1
    assert summary["tracking"]["waist"]["valid"] is True
    assert "reacquire_guard_summary" in summary
    assert "object_gating_summary" in summary
    assert summary["object_gating_summary"]["totals"]["enforced_count"] == 1
    assert summary["object_gating_summary"]["totals"]["diagnostics_only_count"] == 0
    assert summary["rigid_hint_summary"]["totals"]["accepted_points"] == 4
    assert summary["rigid_hint_pose_summary"]["totals"]["phase6_ready"] is True
    assert summary["subset_hypothesis_summary"]["totals"]["phase6_complete"] is True
    shadow = summary["subset_hypothesis_summary"]["totals"]["subset_adoption_shadow"]
    assert shadow["adopted_event_count"] == 1
    assert shadow["decision"] == "go_candidate_for_enforcement_replay"
    assert summary["phase45_go_no_go"]["decision"] in {
        "no_reacquire_reject_signal",
        "pending_enforcement_replay",
    }


def test_replay_tracking_log_resolves_extrinsics_file_to_calibration_dir(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)
    calibration_dir = tmp_path / "calibration"
    calibration_dir.mkdir()
    extrinsics_path = calibration_dir / "extrinsics_pose_v2.json"
    extrinsics_path.write_text("{}", encoding="utf-8")

    summary = harness.replay_tracking_log(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=extrinsics_path,
        patterns=["waist"],
    ).to_dict()

    assert summary["calibration_path"] == str(calibration_dir)
    assert _FakePipeline.last_kwargs["calibration_path"] == str(calibration_dir)


def test_replay_tracking_log_can_limit_injected_frames(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    summary = harness.replay_tracking_log(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
        max_frames=1,
    ).to_dict()

    assert summary["frame_count"] == 1
    assert summary["pair_count"] == 0
    assert summary["poses_estimated"] == 0


def test_replay_tracking_log_can_filter_by_received_at(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    summary = harness.replay_tracking_log(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
        start_received_at="2026-04-24T00:00:00.000500",
    ).to_dict()

    assert summary["frame_count"] == 1
    assert summary["poses_estimated"] == 0


def test_summarize_tracking_diagnostics_extracts_failure_segments(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeDiagnosticsReplay)

    summary = harness.summarize_tracking_diagnostics(
        log_path=tmp_path / "tracking_gui.jsonl",
        patterns=["waist"],
        window_padding_s=1.0,
    )

    waist = summary["by_rigid"]["waist"]
    assert summary["frame_count"] == 2
    assert summary["diagnostics_event_count"] == 3
    assert waist["valid_count"] == 2
    assert waist["invalid_count"] == 1
    assert waist["failure_event_count"] == 2
    assert waist["accepted_points"]["mean"] == 3.0
    assert waist["failure_events"][0]["reasons"] == [
        "low_accepted_points",
        "pose_jump",
        "pose_flip",
    ]
    assert waist["failure_segments"][0]["event_count"] == 2
    assert "invalid" in waist["failure_segments"][0]["reasons"]


def test_compare_reacquire_guard_enforcement_returns_go_no_go(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    comparison = harness.compare_reacquire_guard_enforcement(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
    )

    assert set(comparison) == {"shadow", "enforced", "phase45_go_no_go"}
    assert comparison["shadow"]["poses_estimated"] == 1
    assert comparison["enforced"]["poses_estimated"] == 1
    assert "decision" in comparison["phase45_go_no_go"]


def test_compare_object_gating_enforcement_returns_go_no_go(
    monkeypatch, tmp_path: Path
) -> None:
    monkeypatch.setattr(harness, "FrameReplay", _FakeReplay)
    monkeypatch.setattr(harness, "TrackingPipeline", _FakePipeline)

    comparison = harness.compare_object_gating_enforcement(
        log_path=tmp_path / "tracking_gui.jsonl",
        calibration_path=tmp_path,
        patterns=["waist"],
    )

    assert set(comparison) == {
        "diagnostics_only",
        "enforced",
        "object_gating_go_no_go",
    }
    assert comparison["diagnostics_only"]["poses_estimated"] == 1
    assert comparison["enforced"]["poses_estimated"] == 1
    assert comparison["object_gating_go_no_go"]["decision"] == "go_candidate"


def test_object_gating_go_no_go_allows_tiny_max_flip_regression() -> None:
    baseline = {
        "poses_estimated": 100,
        "tracking": {
            "waist": {
                "max_pose_flip_deg": 177.80,
                "pose_jump_count": 2,
                "mode_transition_count": 10,
            }
        },
        "subset_hypothesis_summary": {
            "totals": {"best_rotation_delta_deg_summary": {"max": 170.0}}
        },
    }
    enforced = {
        "poses_estimated": 101,
        "tracking": {
            "waist": {
                "max_pose_flip_deg": 177.81,
                "pose_jump_count": 2,
                "mode_transition_count": 9,
            }
        },
        "subset_hypothesis_summary": {
            "totals": {"best_rotation_delta_deg_summary": {"max": 1.0}}
        },
    }

    decision = harness._compare_object_gating_go_no_go(baseline, enforced)

    assert decision["decision"] == "go_candidate"
    assert decision["criteria"]["pose_flip_within_tolerance"] is True
    assert decision["max_pose_flip_delta_deg"] > 0.0


def test_object_gating_go_no_go_rejects_large_max_flip_regression() -> None:
    baseline = {
        "poses_estimated": 100,
        "tracking": {
            "waist": {
                "max_pose_flip_deg": 30.0,
                "pose_jump_count": 2,
                "mode_transition_count": 10,
            }
        },
    }
    enforced = {
        "poses_estimated": 100,
        "tracking": {
            "waist": {
                "max_pose_flip_deg": 35.0,
                "pose_jump_count": 2,
                "mode_transition_count": 10,
            }
        },
    }

    decision = harness._compare_object_gating_go_no_go(baseline, enforced)

    assert decision["decision"] == "no_go_tune_object_gating"
    assert decision["criteria"]["pose_flip_within_tolerance"] is False


def test_subset_adoption_shadow_handles_low_coverage_and_missing_generic_valid() -> None:
    summary = harness._summarize_subset_adoption_shadow(
        [
            {"subset_adoption_ready": True, "score_delta": 0.02},
            {"generic_valid": True, "subset_adoption_ready": False},
        ]
    )

    assert summary["baseline_valid_events"] == 1
    assert summary["shadow_valid_events"] == 2
    assert summary["valid_event_delta"] == 1
    assert summary["adoption_ratio"] == 0.5
    assert summary["decision"] == "keep_shadow_until_coverage_improves"


def test_subset_adoption_shadow_blocks_regressions() -> None:
    summary = harness._summarize_subset_adoption_shadow(
        [
            {
                "generic_valid": True,
                "subset_adoption_ready": True,
                "score_delta": -0.01,
                "best_rotation_delta_deg": 91.0,
                "best_position_delta_m": 0.11,
            }
        ]
    )

    assert summary["score_worse_count"] == 1
    assert summary["flip_worse_count"] == 1
    assert summary["jump_worse_count"] == 1
    assert summary["decision"] == "no_go_tune_subset_adoption"
