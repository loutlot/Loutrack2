from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from statistics import mean, median
from threading import Lock
from typing import Any, Dict, Iterable, List, Optional, Tuple


def _percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    if p <= 0:
        return float(min(values))
    if p >= 100:
        return float(max(values))
    sorted_values = sorted(values)
    rank = (len(sorted_values) - 1) * (p / 100.0)
    lower = int(rank)
    upper = min(lower + 1, len(sorted_values) - 1)
    weight = rank - lower
    return sorted_values[lower] * (1.0 - weight) + sorted_values[upper] * weight


@dataclass
class _CameraState:
    last_frame_index: Optional[int] = None
    offset_samples: deque = field(default_factory=lambda: deque(maxlen=2000))
    missing_total: int = 0
    gap_events: int = 0
    longest_gap: int = 0


@dataclass
class _WindowState:
    tolerance_us: int
    pass_pairs: int = 0


class SyncEvaluator:
    def __init__(
        self,
        tolerance_windows_us: Iterable[int] = (1000, 2000, 5000, 10000),
        target_range_us: Tuple[int, int] = (2000, 5000),
        coverage_target: float = 0.95,
        history_size: int = 2000,
    ):
        self.tolerance_windows_us = sorted(set(int(v) for v in tolerance_windows_us if int(v) > 0))
        if not self.tolerance_windows_us:
            self.tolerance_windows_us = [5000]
        self.target_range_us = target_range_us
        self.coverage_target = coverage_target
        self.history_size = history_size
        self._lock = Lock()
        self._pair_count = 0
        self._pair_spreads_us: deque = deque(maxlen=history_size)
        self._window_states: Dict[int, _WindowState] = {
            tol: _WindowState(tolerance_us=tol) for tol in self.tolerance_windows_us
        }
        self._camera_states: Dict[str, _CameraState] = {}

    def reset(self) -> None:
        with self._lock:
            self._pair_count = 0
            self._pair_spreads_us.clear()
            self._window_states = {
                tol: _WindowState(tolerance_us=tol) for tol in self.tolerance_windows_us
            }
            self._camera_states = {}

    def evaluate_pair(self, paired_frames: Any) -> Dict[str, Any]:
        timestamp = int(getattr(paired_frames, "timestamp", 0))
        frames = getattr(paired_frames, "frames", {}) or {}
        if not isinstance(frames, dict) or len(frames) < 2:
            return {
                "pair_count": self._pair_count,
                "pair_spread_us": 0,
                "camera_count": len(frames) if isinstance(frames, dict) else 0,
            }

        timestamps = [int(getattr(frame, "timestamp", timestamp)) for frame in frames.values()]
        spread_us = max(timestamps) - min(timestamps)

        with self._lock:
            self._pair_count += 1
            self._pair_spreads_us.append(float(spread_us))

            for tol, state in self._window_states.items():
                if spread_us <= tol:
                    state.pass_pairs += 1

            for camera_id, frame in frames.items():
                state = self._camera_states.setdefault(camera_id, _CameraState())
                frame_ts = int(getattr(frame, "timestamp", timestamp))
                frame_index = int(getattr(frame, "frame_index", 0))
                offset_us = frame_ts - timestamp
                state.offset_samples.append((frame_ts, offset_us))

                if state.last_frame_index is not None and frame_index > state.last_frame_index + 1:
                    gap = frame_index - state.last_frame_index - 1
                    state.missing_total += gap
                    state.gap_events += 1
                    if gap > state.longest_gap:
                        state.longest_gap = gap
                state.last_frame_index = frame_index

        return {
            "pair_count": self._pair_count,
            "pair_spread_us": spread_us,
            "camera_count": len(frames),
        }

    def _camera_offset_summary(self) -> Dict[str, Dict[str, float]]:
        result: Dict[str, Dict[str, float]] = {}
        for camera_id, state in self._camera_states.items():
            offsets = [float(o) for _, o in state.offset_samples]
            if not offsets:
                continue
            avg_offset = mean(offsets)
            result[camera_id] = {
                "mean_offset_us": avg_offset,
                "median_offset_us": median(offsets),
                "p95_offset_us": _percentile(offsets, 95),
                "p99_offset_us": _percentile(offsets, 99),
                "jitter_std_us": (mean([(x - avg_offset) ** 2 for x in offsets])) ** 0.5,
                "drift_us_per_s": self._estimate_drift_us_per_s(state.offset_samples),
            }
        return result

    @staticmethod
    def _estimate_drift_us_per_s(samples: deque) -> float:
        if len(samples) < 2:
            return 0.0
        first_ts, first_offset = samples[0]
        last_ts, last_offset = samples[-1]
        dt_s = (float(last_ts) - float(first_ts)) / 1_000_000.0
        if dt_s <= 0:
            return 0.0
        return (float(last_offset) - float(first_offset)) / dt_s

    def _recommend_window(self, p95_spread_us: float) -> Dict[str, Any]:
        if self._pair_count <= 0:
            return {
                "recommended_window_us": None,
                "status": "insufficient_data",
                "reason": "no_pairs",
            }

        recommended: Optional[int] = None
        for tol in self.tolerance_windows_us:
            pass_rate = self._window_states[tol].pass_pairs / self._pair_count
            if pass_rate >= self.coverage_target and p95_spread_us <= tol:
                recommended = tol
                break

        if recommended is None:
            recommended = max(self.tolerance_windows_us)

        target_low, target_high = self.target_range_us
        if p95_spread_us <= target_high:
            status = "recommended"
            reason = "p95_within_target"
        elif p95_spread_us <= target_high * 2:
            status = "borderline"
            reason = "p95_above_target"
        else:
            status = "unsafe"
            reason = "p95_far_above_target"

        return {
            "recommended_window_us": recommended,
            "status": status,
            "reason": reason,
            "target_range_us": [target_low, target_high],
        }

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            spreads = [float(v) for v in self._pair_spreads_us]
            p95_spread = _percentile(spreads, 95)

            sweep = []
            for tol in self.tolerance_windows_us:
                pass_pairs = self._window_states[tol].pass_pairs
                pass_rate = (pass_pairs / self._pair_count) if self._pair_count else 0.0
                sweep.append(
                    {
                        "tolerance_us": tol,
                        "pass_pairs": pass_pairs,
                        "pass_rate": pass_rate,
                    }
                )

            missing_total = sum(state.missing_total for state in self._camera_states.values())
            gap_events = sum(state.gap_events for state in self._camera_states.values())
            longest_gap = max((state.longest_gap for state in self._camera_states.values()), default=0)

            recommendation = self._recommend_window(p95_spread)
            camera_offsets = self._camera_offset_summary()

            return {
                "pair_count": self._pair_count,
                "pair_spread_us": {
                    "mean": mean(spreads) if spreads else 0.0,
                    "median": median(spreads) if spreads else 0.0,
                    "p95": p95_spread,
                    "p99": _percentile(spreads, 99),
                    "max": max(spreads) if spreads else 0.0,
                },
                "window_sweep": sweep,
                "camera_offsets": camera_offsets,
                "missing_behavior": {
                    "total_missing_frames": missing_total,
                    "gap_events": gap_events,
                    "longest_gap": longest_gap,
                    "per_camera": {
                        camera_id: {
                            "missing_total": state.missing_total,
                            "gap_events": state.gap_events,
                            "longest_gap": state.longest_gap,
                        }
                        for camera_id, state in self._camera_states.items()
                    },
                },
                "recommendation": recommendation,
            }
