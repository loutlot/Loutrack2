import os
import sys
import math
from pathlib import Path

import pytest

# Ensure src is on sys.path for module imports
ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / 'src'
assert SRC.exists(), f"Source path not found: {SRC}"
sys.path.insert(0, str(SRC))

# Public API surface used in tests is in host.sim
from host.sim import run_closed_loop, verify_frame_log, assert_metrics  # type: ignore


def test_sim_closed_loop_static_no_noise_metrics_and_logs(tmp_path):
    out_dir = str(tmp_path / 'sim_out')
    os.makedirs(out_dir, exist_ok=True)

    # Run a short static no-noise closed-loop
    out = run_closed_loop(
        frames=120,
        fps=60,
        noise_px=0.0,
        marker_dropout=0.0,
        camera_dropout=0.0,
        trajectory='static',
        seed=0,
        out_dir=out_dir,
    )

    # Basic metrics and logs assertions
    assert isinstance(out, dict)
    # Top-level keys must match the actual return from host.sim.run_closed_loop
    for k in (
        'mean_position_error_m',
        'mean_rotation_error_deg',
        'drop_rate',
        'frame_log',
        'pose_log',
        'eval_json',
    ):
        assert k in out, f"missing key {k} in run_closed_loop output"

    # Validate metrics thresholds
    assert_metrics(out, max_mean_position_error_m=0.001, max_mean_rotation_error_deg=1.0)

    # Artifact existence checks (frame log should exist)
    frame_log_path = out['frame_log']
    pose_log_path = out['pose_log']
    eval_json_path = out['eval_json']
    assert Path(frame_log_path).exists(), frame_log_path
    assert Path(pose_log_path).exists(), pose_log_path
    assert Path(eval_json_path).exists(), eval_json_path

    # Verify that the frame log is structurally valid
    rep = verify_frame_log(out['frame_log'])
    assert rep.get('valid', False)


def test_sim_closed_loop_mild_noise_still_runs(tmp_path):
    out_dir = str(tmp_path / 'sim_out_noise')
    os.makedirs(out_dir, exist_ok=True)

    out = run_closed_loop(
        frames=120,
        fps=60,
        noise_px=0.5,
        marker_dropout=0.0,
        camera_dropout=0.0,
        trajectory='static',
        seed=1,
        out_dir=out_dir,
    )

    assert isinstance(out, dict)
    # Do NOT rely on a 'metrics' dict; top-level keys are returned by run_closed_loop
    drop_rate = out.get('drop_rate')
    mean_pos_err = out.get('mean_position_error_m')
    mean_rot_err = out.get('mean_rotation_error_deg')

    assert drop_rate is not None and math.isfinite(float(drop_rate))
    assert float(drop_rate) <= 1.0
    assert mean_pos_err is not None and math.isfinite(float(mean_pos_err))
    assert mean_pos_err < 0.05
    assert mean_rot_err is not None and math.isfinite(float(mean_rot_err))
    assert mean_rot_err < 20.0


def test_assert_metrics_error_message_includes_metric_name():
    from host.sim import assert_metrics as am  # local import to ensure path is set

    with pytest.raises(AssertionError) as exc:
        am({'mean_position_error_m': 0.01, 'mean_rotation_error_deg': 0.0}, max_mean_position_error_m=0.0)
    assert 'mean_position_error_m' in str(exc.value)
