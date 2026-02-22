import sys
import os
import importlib.util

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from dataclasses import dataclass
from typing import Dict

_sync_eval_path = os.path.join(os.path.dirname(__file__), "..", "src", "host", "sync_eval.py")
_spec = importlib.util.spec_from_file_location("sync_eval", _sync_eval_path)
if _spec is None or _spec.loader is None:
    raise RuntimeError("Failed to load sync_eval module")
_mod = importlib.util.module_from_spec(_spec)
sys.modules["sync_eval"] = _mod
_spec.loader.exec_module(_mod)
SyncEvaluator = _mod.SyncEvaluator


@dataclass
class _Frame:
    timestamp: int
    frame_index: int


@dataclass
class _Pair:
    timestamp: int
    frames: Dict[str, _Frame]


def test_sync_evaluator_basic() -> None:
    ev = SyncEvaluator(tolerance_windows_us=(1000, 2000, 5000), target_range_us=(2000, 5000))

    p1 = _Pair(
        timestamp=1_000_000,
        frames={
            "cam_a": _Frame(timestamp=1_000_000, frame_index=10),
            "cam_b": _Frame(timestamp=1_001_000, frame_index=10),
        },
    )
    p2 = _Pair(
        timestamp=2_000_000,
        frames={
            "cam_a": _Frame(timestamp=2_000_000, frame_index=11),
            "cam_b": _Frame(timestamp=2_004_500, frame_index=13),
        },
    )

    ev.evaluate_pair(p1)
    ev.evaluate_pair(p2)
    status = ev.get_status()

    assert status["pair_count"] == 2
    assert "pair_spread_us" in status
    assert "window_sweep" in status
    assert "camera_offsets" in status
    assert status["missing_behavior"]["total_missing_frames"] >= 1
    assert status["recommendation"]["recommended_window_us"] is not None


def test_sync_evaluator_reset() -> None:
    ev = SyncEvaluator()
    p = _Pair(
        timestamp=100,
        frames={
            "cam_a": _Frame(timestamp=100, frame_index=1),
            "cam_b": _Frame(timestamp=101, frame_index=1),
        },
    )
    ev.evaluate_pair(p)
    assert ev.get_status()["pair_count"] == 1
    ev.reset()
    assert ev.get_status()["pair_count"] == 0


if __name__ == "__main__":
    test_sync_evaluator_basic()
    test_sync_evaluator_reset()
    print("sync_eval tests passed")
