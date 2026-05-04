"""Gate 3 projection screen for rigid-body design candidates."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Sequence

import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from tools.rigid_body_design_search import _load_seed_patterns
from tools.sim.multi_rigid import (
    DEFAULT_BODY_RIGIDS,
    DEFAULT_GENERATED_CAMERA_IDS,
    MultiRigidFrameGenerator,
    MultiRigidScenarioConfig,
)


DEFAULT_OUT = Path("logs/design/rigid_body_v2/projection")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare projected marker separation for rigid-body candidates.")
    parser.add_argument("--candidate", required=True, help="Candidate JSON from rigid_body_design_search.py.")
    parser.add_argument("--baseline", default=None, help="Optional baseline candidate JSON. Defaults to design_5marker_seed.")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--scenario", default="five_rigid_body_occlusion_v1")
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    baseline_patterns = _load_seed_patterns(args.baseline)
    candidate_patterns = _load_seed_patterns(args.candidate)
    baseline = _screen_patterns(
        "baseline",
        baseline_patterns,
        frames=int(args.frames),
        scenario=str(args.scenario),
    )
    candidate = _screen_patterns(
        "candidate",
        candidate_patterns,
        frames=int(args.frames),
        scenario=str(args.scenario),
    )
    comparison = _compare_projection(baseline, candidate)
    payload = {
        "scenario": str(args.scenario),
        "frames": int(args.frames),
        "baseline": baseline,
        "candidate": candidate,
        "comparison": comparison,
    }
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "summary.json"
    out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"summary": str(out_path), "pass": comparison["pass"], "comparison": comparison}, indent=2))
    return 0


def _screen_patterns(
    label: str,
    patterns: Sequence[Any],
    *,
    frames: int,
    scenario: str,
) -> Dict[str, Any]:
    config = MultiRigidScenarioConfig(
        scenario=scenario,
        frames=max(1, int(frames)),
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        rigid_names=DEFAULT_BODY_RIGIDS,
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout="design_5marker_seed",
    )
    generator = MultiRigidFrameGenerator(config, patterns=list(patterns))
    camera_pair_distances: Dict[str, List[float]] = {
        camera_id: [] for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    worst_pair: Dict[str, Any] | None = None
    subset_areas: Dict[str, Dict[str, List[float]]] = {
        body.pattern.name: {"strong_4": [], "strong_3": []}
        for body in generator._bodies.values()
    }
    policies = {
        pattern.name: dict((pattern.metadata or {}).get("tracking_policy", {}))
        for pattern in patterns
    }
    for frame_index in range(max(1, int(frames))):
        body_points = {
            name: body.marker_positions_world(frame_index)
            for name, body in generator._bodies.items()
        }
        for camera_id, camera in generator._cameras.items():
            projected_by_body = {
                name: camera.project_points(points)
                for name, points in body_points.items()
            }
            visible = [
                (name, marker_index, point)
                for name, points in projected_by_body.items()
                for marker_index, point in enumerate(points)
                if point is not None
            ]
            for left_index in range(len(visible)):
                for right_index in range(left_index + 1, len(visible)):
                    left = np.asarray(visible[left_index][2], dtype=np.float64)
                    right = np.asarray(visible[right_index][2], dtype=np.float64)
                    distance = float(np.linalg.norm(left - right))
                    camera_pair_distances[camera_id].append(distance)
                    if worst_pair is None or distance < float(worst_pair["distance_px"]):
                        worst_pair = {
                            "distance_px": distance,
                            "frame_index": int(frame_index),
                            "camera_id": camera_id,
                            "left": {
                                "body": visible[left_index][0],
                                "marker": int(visible[left_index][1]),
                                "uv": [float(value) for value in visible[left_index][2]],
                            },
                            "right": {
                                "body": visible[right_index][0],
                                "marker": int(visible[right_index][1]),
                                "uv": [float(value) for value in visible[right_index][2]],
                            },
                        }
            for body_name, points in projected_by_body.items():
                policy = policies.get(body_name, {})
                _append_subset_areas(
                    subset_areas[body_name]["strong_4"],
                    points,
                    policy.get("strong_4_subsets", []),
                )
                _append_subset_areas(
                    subset_areas[body_name]["strong_3"],
                    points,
                    policy.get("strong_3_subsets", []),
                )
    return {
        "label": label,
        "marker_pair_separation_px": {
            camera_id: _numeric_summary(values)
            for camera_id, values in camera_pair_distances.items()
        },
        "worst_marker_pair_separation_px": min(
            (
                _numeric_summary(values)["min"]
                for values in camera_pair_distances.values()
                if values
            ),
            default=0.0,
        ),
        "worst_marker_pair": worst_pair,
        "marker_pair_separation_p05_px": min(
            (
                _numeric_summary(values)["p05"]
                for values in camera_pair_distances.values()
                if values
            ),
            default=0.0,
        ),
        "subset_projected_area_px2": {
            body_name: {
                subset_kind: _numeric_summary(values)
                for subset_kind, values in body_values.items()
            }
            for body_name, body_values in subset_areas.items()
        },
        "worst_strong_subset_area_px2": min(
            (
                _numeric_summary(values)["min"]
                for body_values in subset_areas.values()
                for values in body_values.values()
                if values
            ),
            default=0.0,
        ),
    }


def _append_subset_areas(
    target: List[float],
    projected_points: Sequence[tuple[float, float] | None],
    subsets: Iterable[Iterable[int]],
) -> None:
    for subset in subsets:
        points = []
        for marker_index in subset:
            marker_index = int(marker_index)
            if marker_index < 0 or marker_index >= len(projected_points):
                points = []
                break
            point = projected_points[marker_index]
            if point is None:
                points = []
                break
            points.append(point)
        if len(points) >= 3:
            target.append(_projected_area(points))


def _projected_area(points: Sequence[tuple[float, float]]) -> float:
    array = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    hull = cv2.convexHull(array)
    return float(cv2.contourArea(hull))


def _numeric_summary(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {"count": 0.0, "min": 0.0, "p05": 0.0, "median": 0.0, "p95": 0.0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": float(len(array)),
        "min": float(np.min(array)),
        "p05": float(np.percentile(array, 5)),
        "median": float(np.percentile(array, 50)),
        "p95": float(np.percentile(array, 95)),
    }


def _compare_projection(baseline: Dict[str, Any], candidate: Dict[str, Any]) -> Dict[str, Any]:
    baseline_p05 = float(baseline["marker_pair_separation_p05_px"])
    candidate_p05 = float(candidate["marker_pair_separation_p05_px"])
    baseline_worst = float(baseline["worst_marker_pair_separation_px"])
    candidate_worst = float(candidate["worst_marker_pair_separation_px"])
    baseline_area = float(baseline["worst_strong_subset_area_px2"])
    candidate_area = float(candidate["worst_strong_subset_area_px2"])
    return {
        "pass": bool(
            candidate_p05 >= baseline_p05
            and candidate_worst >= baseline_worst
            and candidate_area >= baseline_area
        ),
        "marker_pair_p05_delta_px": float(candidate_p05 - baseline_p05),
        "worst_marker_pair_delta_px": float(candidate_worst - baseline_worst),
        "worst_strong_subset_area_delta_px2": float(candidate_area - baseline_area),
        "baseline_marker_pair_p05_px": baseline_p05,
        "candidate_marker_pair_p05_px": candidate_p05,
        "baseline_worst_marker_pair_px": baseline_worst,
        "candidate_worst_marker_pair_px": candidate_worst,
        "baseline_worst_strong_subset_area_px2": baseline_area,
        "candidate_worst_strong_subset_area_px2": candidate_area,
    }


if __name__ == "__main__":
    raise SystemExit(main())
