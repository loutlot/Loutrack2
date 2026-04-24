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

    def replay(self, *, realtime: bool = False):
        assert realtime is False
        yield from self.entries


class _FakeFrameProcessor:
    def __init__(self) -> None:
        self.callback = None
        self.frames = []

    def set_paired_callback(self, callback):
        self.callback = callback

    def _on_frame_received(self, frame):
        self.frames.append(frame)
        if len(self.frames) == 2 and self.callback is not None:
            self.callback({"paired": True})

    def _process_pairs(self):
        return None

    def get_stats(self):
        return {"pairer": {"pairs_emitted": 1}}


class _FakePipeline:
    def __init__(self, **_kwargs) -> None:
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
