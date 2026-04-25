from __future__ import annotations

import argparse
import inspect
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from host.metrics import MetricsCollector
from host.pipeline import TrackingPipeline, default_subset_diagnostics_mode_for_variant
from host.receiver import Frame, PairedFrames
from host.rigid import RigidBodyPose, WAIST_PATTERN


DEFAULT_VARIANTS = (
    "fast_ABCD",
    "fast_ABCDF",
    "fast_ABCDH",
    "fast_ABCDHR",
    "fast_ABCDHRF",
)
DEFAULT_CAMERA_IDS = tuple(f"pi-cam-{index:02d}" for index in range(1, 5))


@dataclass(frozen=True)
class SyntheticBenchmarkConfig:
    camera_ids: tuple[str, ...] = DEFAULT_CAMERA_IDS
    blobs_per_camera: int = 32
    pairs: int = 120
    warmup_pairs: int = 8
    seed: int = 32032
    timestamp_start_us: int = 1_000_000
    timestamp_step_us: int = 8_475
    rigid_position_m: tuple[float, float, float] = (0.60, 0.05, 2.80)
    subset_diagnostics_mode: str | None = None


def _project(camera: Any, point_world: np.ndarray) -> tuple[float, float]:
    point_camera = camera.rotation @ point_world + camera.translation
    return (
        float(camera.fx * point_camera[0] / point_camera[2] + camera.cx),
        float(camera.fy * point_camera[1] / point_camera[2] + camera.cy),
    )


def _blob(xy: tuple[float, float], *, area: float = 4.0) -> dict[str, float]:
    return {"x": float(xy[0]), "y": float(xy[1]), "area": float(area)}


class SyntheticPairFactory:
    def __init__(
        self,
        *,
        camera_params: dict[str, Any],
        config: SyntheticBenchmarkConfig,
    ) -> None:
        self.camera_params = camera_params
        self.config = config
        self.rigid_position = np.asarray(config.rigid_position_m, dtype=np.float64)
        self.marker_points = WAIST_PATTERN.marker_positions + self.rigid_position.reshape(1, 3)

    def pair(self, pair_index: int) -> PairedFrames:
        timestamp = int(
            self.config.timestamp_start_us
            + int(pair_index) * self.config.timestamp_step_us
        )
        frames: dict[str, Frame] = {}
        base_received_at = 1_700_000_000.0 + float(pair_index) * 0.0085
        for camera_index, camera_id in enumerate(self.config.camera_ids):
            camera = self.camera_params[camera_id]
            blobs = [
                _blob(_project(camera, marker_point), area=40.0)
                for marker_point in self.marker_points
            ]
            rng = np.random.default_rng(
                self.config.seed + pair_index * 101 + camera_index * 10_007
            )
            width, height = camera.resolution
            while len(blobs) < self.config.blobs_per_camera:
                blobs.append(
                    _blob(
                        (
                            float(rng.uniform(20.0, width - 20.0)),
                            float(rng.uniform(20.0, height - 20.0)),
                        ),
                        area=4.0,
                    )
                )

            received_at = base_received_at + camera_index * 0.0001
            frame_timestamp = timestamp + camera_index * 5
            frames[camera_id] = Frame(
                camera_id=camera_id,
                timestamp=frame_timestamp,
                frame_index=None,
                blobs=blobs,
                received_at=received_at,
                host_received_at_us=int(received_at * 1_000_000),
                timestamp_source="synthetic",
                capture_to_process_ms=0.0,
                capture_to_send_ms=0.0,
            )

        return PairedFrames(
            timestamp=timestamp,
            frames=frames,
            timestamp_range_us=(len(frames) - 1) * 5,
            pair_emitted_at_us=int((base_received_at + 0.0005) * 1_000_000),
            pair_age_ms=0.0,
            pair_host_receive_span_ms=max(0.0, (len(frames) - 1) * 0.1),
            oldest_frame_age_ms=0.0,
            newest_frame_age_ms=0.0,
        )


def _install_geometry_kwarg_compatibility(pipeline: TrackingPipeline) -> bool:
    """Keep this helper usable across in-flight test/prototype API branches."""
    original = pipeline.geometry.process_paired_frames
    accepted = set(inspect.signature(original).parameters)
    unsupported = {"use_rigid_hint_as_generic"} - accepted
    if not unsupported:
        return False

    def wrapped(*args: Any, **kwargs: Any) -> Any:
        return original(
            *args,
            **{key: value for key, value in kwargs.items() if key in accepted},
        )

    pipeline.geometry.process_paired_frames = wrapped  # type: ignore[method-assign]
    return True


def _seed_waist_tracker(pipeline: TrackingPipeline, config: SyntheticBenchmarkConfig) -> None:
    tracker = pipeline.rigid_estimator.trackers.get(WAIST_PATTERN.name)
    if tracker is None:
        return
    position = np.asarray(config.rigid_position_m, dtype=np.float64)
    for index in range(3):
        tracker.update(
            RigidBodyPose(
                timestamp=config.timestamp_start_us - (3 - index) * config.timestamp_step_us,
                position=position.copy(),
                rotation=np.eye(3, dtype=np.float64),
                quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                rms_error=0.0,
                observed_markers=WAIST_PATTERN.num_markers,
                valid=True,
            )
        )


def _reset_diagnostics(pipeline: TrackingPipeline) -> None:
    pipeline.frames_processed = 0
    pipeline.poses_estimated = 0
    pipeline.metrics = MetricsCollector()
    for values in getattr(pipeline, "_stage_ms", {}).values():
        values.clear()
    for name in (
        "_variant_counts",
        "_fallback_reason_counts",
        "_object_gating_filter_reason_counts",
    ):
        counter = getattr(pipeline, name, None)
        if counter is not None:
            counter.clear()
    for name in (
        "_slow_pair_events",
        "_reacquire_guard_events",
        "_object_gating_events",
        "_rigid_hint_events",
        "_rigid_hint_pose_events",
        "_subset_hypothesis_events",
    ):
        events = getattr(pipeline, name, None)
        if events is not None:
            events.clear()
    rigid_estimator = getattr(pipeline, "rigid_estimator", None)
    if rigid_estimator is not None:
        for name in (
            "_subset_frame_index",
            "_subset_sampled_count",
            "_subset_skipped_count",
            "_subset_budget_exceeded_count",
            "_rigid_candidate_separated_count",
            "_rigid_candidate_fallback_count",
        ):
            if hasattr(rigid_estimator, name):
                setattr(rigid_estimator, name, 0)
        fallback_reasons = getattr(
            rigid_estimator,
            "_rigid_candidate_fallback_reason_counts",
            None,
        )
        if fallback_reasons is not None:
            fallback_reasons.clear()


def _summary_value(summary: dict[str, Any], metric: str) -> float:
    try:
        return float(summary.get(metric, 0.0) or 0.0)
    except Exception:
        return 0.0


def _compact_status(
    *,
    variant: str,
    status: dict[str, Any],
    elapsed_ms: float,
    errors: list[str],
    compatibility_wrapper_used: bool,
) -> dict[str, Any]:
    diagnostics = status.get("diagnostics", {})
    quality = status.get("triangulation_quality", {})
    tracking = status.get("tracking", {})
    waist = tracking.get(WAIST_PATTERN.name, {}) if isinstance(tracking, dict) else {}
    frames_processed = int(status.get("frames_processed", 0) or 0)
    return {
        "pipeline_variant": variant,
        "frames_processed": frames_processed,
        "poses_estimated": int(status.get("poses_estimated", 0) or 0),
        "elapsed_wall_ms": float(elapsed_ms),
        "pairs_per_second": (
            float(frames_processed / (elapsed_ms / 1000.0)) if elapsed_ms > 0.0 else 0.0
        ),
        "waist_valid": bool(waist.get("valid", False)),
        "waist_mode": str(waist.get("mode", "")),
        "accepted_points": int(quality.get("accepted_points", 0) or 0),
        "reprojection_p95_px": float(
            quality.get("reprojection_error_px_summary", {}).get("p95", 0.0) or 0.0
        ),
        "pipeline_pair_ms": dict(diagnostics.get("pipeline_stage_ms", {}).get("pipeline_pair_ms", {})),
        "triangulation_ms": dict(diagnostics.get("pipeline_stage_ms", {}).get("triangulation_ms", {})),
        "rigid_ms": dict(diagnostics.get("pipeline_stage_ms", {}).get("rigid_ms", {})),
        "stage_ms_detail": dict(diagnostics.get("stage_ms_detail", {})),
        "variant_metrics": dict(diagnostics.get("variant_metrics", {})),
        "fallback_summary": dict(diagnostics.get("fallback_summary", {})),
        "assignment_diagnostics": dict(quality.get("assignment_diagnostics", {})),
        "slow_pair_count": len(diagnostics.get("slow_pair_events", []) or []),
        "error_count": len(errors),
        "errors": errors,
        "compatibility_wrapper_used": bool(compatibility_wrapper_used),
    }


def run_variant(variant: str, config: SyntheticBenchmarkConfig) -> dict[str, Any]:
    pipeline = TrackingPipeline(
        enable_logging=False,
        pipeline_variant=variant,
        subset_diagnostics_mode=(
            config.subset_diagnostics_mode
            or default_subset_diagnostics_mode_for_variant(variant)
        ),
    )
    pipeline.use_dummy_calibration(list(config.camera_ids))
    compatibility_wrapper_used = _install_geometry_kwarg_compatibility(pipeline)
    _seed_waist_tracker(pipeline, config)
    pair_factory = SyntheticPairFactory(
        camera_params=pipeline.geometry.camera_params,
        config=config,
    )
    errors: list[str] = []
    pipeline.set_error_callback(lambda exc: errors.append(repr(exc)))
    pipeline._running = True

    for pair_index in range(config.warmup_pairs):
        pipeline._on_paired_frames(pair_factory.pair(-config.warmup_pairs + pair_index))
    _reset_diagnostics(pipeline)

    started_ns = time.perf_counter_ns()
    for pair_index in range(config.pairs):
        pipeline._on_paired_frames(pair_factory.pair(pair_index))
    elapsed_ms = float(time.perf_counter_ns() - started_ns) / 1_000_000.0
    pipeline._running = False

    return _compact_status(
        variant=variant,
        status=pipeline.get_status(),
        elapsed_ms=elapsed_ms,
        errors=errors,
        compatibility_wrapper_used=compatibility_wrapper_used,
    )


def _compare_to_baseline(
    baseline: dict[str, Any],
    candidate: dict[str, Any],
) -> dict[str, float]:
    base_p95 = _summary_value(baseline.get("pipeline_pair_ms", {}), "p95")
    cand_p95 = _summary_value(candidate.get("pipeline_pair_ms", {}), "p95")
    base_mean = _summary_value(baseline.get("pipeline_pair_ms", {}), "mean")
    cand_mean = _summary_value(candidate.get("pipeline_pair_ms", {}), "mean")
    return {
        "pipeline_pair_p95_delta_ms": float(cand_p95 - base_p95),
        "pipeline_pair_p95_ratio": float(cand_p95 / base_p95) if base_p95 > 0.0 else 0.0,
        "pipeline_pair_mean_delta_ms": float(cand_mean - base_mean),
        "pipeline_pair_mean_ratio": float(cand_mean / base_mean) if base_mean > 0.0 else 0.0,
        "accepted_points_delta": float(
            int(candidate.get("accepted_points", 0) or 0)
            - int(baseline.get("accepted_points", 0) or 0)
        ),
        "poses_estimated_delta": float(
            int(candidate.get("poses_estimated", 0) or 0)
            - int(baseline.get("poses_estimated", 0) or 0)
        ),
    }


def run_benchmark(
    *,
    variants: Iterable[str] = DEFAULT_VARIANTS,
    config: SyntheticBenchmarkConfig = SyntheticBenchmarkConfig(),
) -> dict[str, Any]:
    selected_variants = tuple(str(variant) for variant in variants)
    results = {variant: run_variant(variant, config) for variant in selected_variants}
    baseline_name = selected_variants[0] if selected_variants else ""
    baseline = results.get(baseline_name, {})
    comparisons = {
        variant: _compare_to_baseline(baseline, result)
        for variant, result in results.items()
        if variant != baseline_name
    }
    return {
        "benchmark": "synthetic_4cam_32_blobs_per_camera",
        "config": {
            "camera_ids": list(config.camera_ids),
            "blobs_per_camera": int(config.blobs_per_camera),
            "pairs": int(config.pairs),
            "warmup_pairs": int(config.warmup_pairs),
            "seed": int(config.seed),
            "timestamp_step_us": int(config.timestamp_step_us),
            "rigid": WAIST_PATTERN.name,
            "subset_diagnostics_mode": config.subset_diagnostics_mode,
        },
        "baseline_variant": baseline_name,
        "variants": results,
        "comparisons_vs_baseline": comparisons,
    }


def _parse_variants(value: str) -> tuple[str, ...]:
    return tuple(item.strip() for item in value.split(",") if item.strip())


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run deterministic 4-camera 32-blobs/camera synthetic pipeline AB benchmarks."
    )
    parser.add_argument("--pairs", type=int, default=120)
    parser.add_argument("--warmup-pairs", type=int, default=8)
    parser.add_argument("--blobs-per-camera", type=int, default=32)
    parser.add_argument("--seed", type=int, default=32032)
    parser.add_argument("--subset-diagnostics-mode", choices=["full", "sampled", "off"], default=None)
    parser.add_argument(
        "--variants",
        default=",".join(DEFAULT_VARIANTS),
        help="Comma-separated pipeline variants. First variant is the AB baseline.",
    )
    args = parser.parse_args(argv)

    config = SyntheticBenchmarkConfig(
        blobs_per_camera=int(args.blobs_per_camera),
        pairs=int(args.pairs),
        warmup_pairs=int(args.warmup_pairs),
        seed=int(args.seed),
        subset_diagnostics_mode=args.subset_diagnostics_mode,
    )
    report = run_benchmark(variants=_parse_variants(args.variants), config=config)
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
