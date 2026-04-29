import json
import math
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
assert SRC.exists(), f"Source path not found: {SRC}"
sys.path.insert(0, str(SRC))

from tools.sim import (  # type: ignore
    MultiRigidScenarioConfig,
    assert_metrics,
    run_closed_loop,
    run_multi_rigid_scenario,
    verify_frame_log,
)


def _summary_metadata(summary: dict) -> dict:
    metadata = dict(summary.get("metadata") or {})
    for key in ("camera_rig_source", "marker_layout", "rigid_names"):
        if key in summary:
            metadata[key] = summary[key]
    eval_json = summary.get("eval_json")
    if eval_json:
        eval_summary = json.loads(Path(eval_json).read_text(encoding="utf-8"))
        metadata.update(eval_summary.get("metadata") or {})
        for key in ("camera_rig_source", "marker_layout", "rigid_names"):
            if key in eval_summary:
                metadata[key] = eval_summary[key]
    return metadata


def test_multi_rigid_summary_includes_mvp_metadata(tmp_path):
    summary = run_multi_rigid_scenario(
        MultiRigidScenarioConfig(
            frames=2,
            fps=118,
            seed=7,
            camera_rig_source="dummy",
            scenario="waist_wand_static_clean",
        ),
        out_dir=str(tmp_path / "sim_out"),
    )

    metadata = _summary_metadata(summary)
    assert metadata["camera_rig_source"] == "dummy"
    assert metadata["marker_layout"] == "current_4marker"
    assert tuple(metadata["rigid_names"]) == ("waist", "wand")
    assert "performance_budget" in summary
    assert "variant_metrics" in summary
    assert "pipeline_pair_no_sustained_over_8_475ms" in summary["production_go_no_go"]


def test_sim_closed_loop_static_no_noise_metrics_and_logs(tmp_path):
    out = run_closed_loop(
        frames=120,
        fps=60,
        noise_px=0.0,
        marker_dropout=0.0,
        camera_dropout=0.0,
        trajectory="static",
        seed=0,
        out_dir=str(tmp_path / "legacy_static"),
    )

    for key in (
        "mean_position_error_m",
        "mean_rotation_error_deg",
        "drop_rate",
        "frame_log",
        "pose_log",
        "eval_json",
    ):
        assert key in out

    assert_metrics(out, max_mean_position_error_m=0.001, max_mean_rotation_error_deg=1.0)
    assert Path(out["frame_log"]).exists()
    assert Path(out["pose_log"]).exists()
    assert Path(out["eval_json"]).exists()
    assert verify_frame_log(out["frame_log"]).get("valid", False)


def test_sim_closed_loop_mild_noise_still_runs(tmp_path):
    out = run_closed_loop(
        frames=120,
        fps=60,
        noise_px=0.5,
        marker_dropout=0.0,
        camera_dropout=0.0,
        trajectory="static",
        seed=1,
        out_dir=str(tmp_path / "legacy_noise"),
    )

    assert math.isfinite(float(out["drop_rate"]))
    assert float(out["drop_rate"]) <= 1.0
    assert math.isfinite(float(out["mean_position_error_m"]))
    assert float(out["mean_position_error_m"]) < 0.05
    assert math.isfinite(float(out["mean_rotation_error_deg"]))
    assert float(out["mean_rotation_error_deg"]) < 25.0


def test_assert_metrics_error_message_includes_metric_name():
    with pytest.raises(AssertionError) as exc:
        assert_metrics(
            {"mean_position_error_m": 0.01, "mean_rotation_error_deg": 0.0},
            max_mean_position_error_m=0.0,
        )
    assert "mean_position_error_m" in str(exc.value)


def test_cli_summary_file_includes_mvp_metadata(tmp_path):
    out_dir = tmp_path / "cli_out"
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "tools.sim",
            "--scenario",
            "waist_wand_static_clean",
            "--frames",
            "2",
            "--fps",
            "118",
            "--noise-px",
            "0",
            "--marker-dropout",
            "0",
            "--camera-dropout",
            "0",
            "--out-dir",
            str(out_dir),
        ],
        cwd=ROOT,
        env={**os.environ, "PYTHONPATH": str(SRC)},
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    eval_json = out_dir / "eval.json"
    assert eval_json.exists()
    metadata = _summary_metadata({"eval_json": str(eval_json)})
    assert metadata["camera_rig_source"] in {"real_2cam", "dummy"}
    assert metadata["marker_layout"] == "current_4marker"
    assert tuple(metadata["rigid_names"]) == ("waist", "wand")


def test_cli_generated_4cam_summary_uses_intrinsics_copy_source(tmp_path):
    proc = subprocess.run(
        [
            sys.executable,
            "-m",
            "tools.sim",
            "--scenario",
            "waist_wand_static_clean",
            "--camera-rig-source",
            "generated_4cam_from_1_2_intrinsics",
            "--frames",
            "1",
            "--out-dir",
            str(tmp_path / "future_4cam"),
        ],
        cwd=ROOT,
        env={**os.environ, "PYTHONPATH": str(SRC)},
        text=True,
        capture_output=True,
        check=False,
    )

    assert proc.returncode == 0, proc.stderr
    metadata = _summary_metadata({"eval_json": str(tmp_path / "future_4cam" / "eval.json")})
    assert metadata["camera_rig_source"] == "generated_4cam_from_1_2_intrinsics"
