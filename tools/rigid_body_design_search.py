"""Search manufacturable 5-marker rigid-body design candidates.

The search is intentionally simple and deterministic: it mutates a seed inside
the 65mm hemisphere, rejects physically invalid layouts, and scores each
candidate with host.pattern_evaluator Gate 2 metrics.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Sequence

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
SRC_ROOT = REPO_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from host.pattern_evaluator import evaluate_subset_design
from host.rigid import MarkerPattern
from tools.sim.multi_rigid import DEFAULT_BODY_RIGIDS, _design_5marker_patterns


MAX_RADIUS_M = 0.065
MIN_Z_M = 0.0
DEFAULT_OUT = Path("logs/design/rigid_body_v2/candidates")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run Gate 2 rigid-body design search.")
    parser.add_argument("--iterations", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260503)
    parser.add_argument("--mutation-mm", type=float, default=8.0)
    parser.add_argument("--keep", type=int, default=10)
    parser.add_argument("--random-restart-rate", type=float, default=0.15)
    parser.add_argument("--seed-candidate", default=None, help="Candidate JSON to continue searching from.")
    parser.add_argument("--focus-worst-rate", type=float, default=0.0, help="Mutate only failing or weakest bodies at this rate.")
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    rng = np.random.default_rng(int(args.seed))
    seed_patterns = _load_seed_patterns(args.seed_candidate)
    best: List[Dict[str, Any]] = []
    baseline = _record_candidate("baseline", seed_patterns)
    best.append(baseline)

    current_patterns = seed_patterns
    mutation_m = float(args.mutation_mm) / 1000.0
    for index in range(max(0, int(args.iterations))):
        if rng.random() < float(args.random_restart_rate):
            candidate = _random_patterns(rng=rng)
        elif rng.random() < float(args.focus_worst_rate):
            candidate = _mutate_focus_worst(
                current_patterns,
                rng=rng,
                mutation_m=mutation_m,
            )
        else:
            candidate = _mutate_patterns(current_patterns, rng=rng, mutation_m=mutation_m)
        if candidate is None:
            continue
        record = _record_candidate(f"candidate_{index:05d}", candidate)
        best.append(record)
        best.sort(key=lambda item: float(item["score"]), reverse=True)
        best = best[: max(1, int(args.keep))]
        current_patterns = _patterns_from_record(best[0])

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    summary = {
        "search": {
            "iterations": int(args.iterations),
            "seed": int(args.seed),
            "mutation_mm": float(args.mutation_mm),
            "random_restart_rate": float(args.random_restart_rate),
            "focus_worst_rate": float(args.focus_worst_rate),
            "seed_candidate": args.seed_candidate,
        },
        "best": best,
    }
    summary_path = out_dir / "summary.json"
    summary_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    for rank, record in enumerate(best):
        candidate_path = out_dir / f"rank_{rank:02d}_{record['name']}.json"
        candidate_path.write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"summary": str(summary_path), "best_score": best[0]["score"], "best_pass": best[0]["gate_2"]["pass"]}, indent=2))
    return 0


def _record_candidate(name: str, patterns: Sequence[MarkerPattern]) -> Dict[str, Any]:
    gate_2 = evaluate_subset_design(patterns)
    bodies = {pattern.name: _points_mm(pattern) for pattern in patterns}
    metrics = _score_gate(gate_2)
    return {
        "name": name,
        "score": metrics["score"],
        "metrics": metrics,
        "marker_positions_mm": bodies,
        "tracking_policy": gate_2["tracking_policy"],
        "gate_2": gate_2,
    }


def _score_gate(gate_2: Dict[str, Any]) -> Dict[str, float]:
    body_scores = []
    min_spacing = 0.0
    for body in gate_2.get("bodies", []):
        best4 = body.get("best_4_subset") or {}
        best3 = body.get("best_3_subset") or {}
        body_scores.append(float(best4.get("score_mm", 0.0)))
        body_scores.append(float(best3.get("score_mm", 0.0)))
        spacing = float(body.get("min_marker_spacing_mm", 0.0))
        min_spacing = spacing if min_spacing == 0.0 else min(min_spacing, spacing)
    worst_subset = min(body_scores) if body_scores else 0.0
    full_cross = float(gate_2.get("full_cross", {}).get("worst_full_cross_mm", 0.0))
    full_ratio = min(1.5, full_cross / 20.0)
    subset_ratio = min(1.5, worst_subset / 8.0)
    spacing_ratio = min(1.5, min_spacing / 35.0)
    spacing_penalty = max(0.0, 35.0 - min_spacing) * 0.75
    pass_bonus = 10.0 if gate_2.get("pass") else 0.0
    score = pass_bonus + full_ratio * 4.0 + subset_ratio * 5.0 + spacing_ratio - spacing_penalty
    return {
        "score": float(score),
        "worst_full_cross_mm": full_cross,
        "worst_best_subset_mm": float(worst_subset),
        "min_marker_spacing_mm": float(min_spacing),
    }


def _mutate_patterns(
    patterns: Sequence[MarkerPattern],
    *,
    rng: np.random.Generator,
    mutation_m: float,
) -> List[MarkerPattern] | None:
    mutated: List[MarkerPattern] = []
    for pattern in patterns:
        points = np.asarray(pattern.marker_positions, dtype=np.float64).copy()
        body_scale = mutation_m * (0.5 + rng.random())
        points += rng.normal(0.0, body_scale, size=points.shape)
        points[:, 2] = np.maximum(points[:, 2], MIN_Z_M)
        norms = np.linalg.norm(points, axis=1)
        too_far = norms > MAX_RADIUS_M
        if np.any(too_far):
            points[too_far] *= (MAX_RADIUS_M / norms[too_far])[:, None]
            points[:, 2] = np.maximum(points[:, 2], MIN_Z_M)
        if _min_spacing(points) < 0.035:
            return None
        mutated.append(
            MarkerPattern(
                name=pattern.name,
                marker_positions=points,
                marker_diameter=pattern.marker_diameter,
                metadata=dict(pattern.metadata),
            )
        )
    return mutated


def _mutate_focus_worst(
    patterns: Sequence[MarkerPattern],
    *,
    rng: np.random.Generator,
    mutation_m: float,
) -> List[MarkerPattern] | None:
    gate_2 = evaluate_subset_design(patterns)
    focus_names = set(_worst_body_names(gate_2))
    if not focus_names:
        return _mutate_patterns(patterns, rng=rng, mutation_m=mutation_m)

    mutated: List[MarkerPattern] = []
    for pattern in patterns:
        if pattern.name not in focus_names:
            mutated.append(
                MarkerPattern(
                    name=pattern.name,
                    marker_positions=np.asarray(pattern.marker_positions, dtype=np.float64).copy(),
                    marker_diameter=pattern.marker_diameter,
                    metadata=dict(pattern.metadata),
                )
            )
            continue
        points = np.asarray(pattern.marker_positions, dtype=np.float64).copy()
        points = _mutate_one_body(points, rng=rng, mutation_m=mutation_m * 1.25)
        if _min_spacing(points) < 0.035:
            return None
        mutated.append(
            MarkerPattern(
                name=pattern.name,
                marker_positions=points,
                marker_diameter=pattern.marker_diameter,
                metadata=dict(pattern.metadata),
            )
        )
    return mutated


def _mutate_one_body(
    points: np.ndarray,
    *,
    rng: np.random.Generator,
    mutation_m: float,
) -> np.ndarray:
    body_scale = mutation_m * (0.5 + rng.random())
    marker_mask = rng.random(points.shape[0]) < 0.65
    if not np.any(marker_mask):
        marker_mask[int(rng.integers(0, points.shape[0]))] = True
    points[marker_mask] += rng.normal(0.0, body_scale, size=(int(np.sum(marker_mask)), 3))
    if rng.random() < 0.35:
        centroid = np.mean(points, axis=0)
        points += (points - centroid) * rng.uniform(0.02, 0.10)
    points[:, 2] = np.maximum(points[:, 2], MIN_Z_M)
    norms = np.linalg.norm(points, axis=1)
    too_far = norms > MAX_RADIUS_M
    if np.any(too_far):
        points[too_far] *= (MAX_RADIUS_M / norms[too_far])[:, None]
        points[:, 2] = np.maximum(points[:, 2], MIN_Z_M)
    return points


def _worst_body_names(gate_2: Dict[str, Any]) -> List[str]:
    scored: List[tuple[float, str]] = []
    for body in gate_2.get("bodies", []):
        best4 = body.get("best_4_subset") or {}
        best3 = body.get("best_3_subset") or {}
        score = min(float(best4.get("score_mm", 0.0)), float(best3.get("score_mm", 0.0)))
        if not body.get("gate_2_pass"):
            score -= 4.0
        scored.append((score, str(body.get("name", ""))))
    scored.sort()
    return [name for _score, name in scored[:2] if name]


def _random_patterns(*, rng: np.random.Generator) -> List[MarkerPattern] | None:
    patterns: List[MarkerPattern] = []
    for name in DEFAULT_BODY_RIGIDS:
        points: List[np.ndarray] = []
        attempts = 0
        while len(points) < 5 and attempts < 500:
            attempts += 1
            point = rng.normal(0.0, 1.0, size=3)
            point[2] = abs(point[2])
            norm = float(np.linalg.norm(point))
            if norm <= 1e-9:
                continue
            radius = MAX_RADIUS_M * (0.55 + 0.45 * rng.random())
            point = point / norm * radius
            if points and min(float(np.linalg.norm(point - other)) for other in points) < 0.035:
                continue
            points.append(point)
        if len(points) != 5:
            return None
        patterns.append(
            MarkerPattern(
                name=name,
                marker_positions=np.asarray(points, dtype=np.float64),
                marker_diameter=0.014,
                metadata={"source": "rigid_body_design_random_search"},
            )
        )
    return patterns


def _patterns_from_record(record: Dict[str, Any]) -> List[MarkerPattern]:
    patterns: List[MarkerPattern] = []
    positions = record["marker_positions_mm"]
    policies = record.get("tracking_policy", {})
    for name in DEFAULT_BODY_RIGIDS:
        patterns.append(
            MarkerPattern(
                name=name,
                marker_positions=np.asarray(positions[name], dtype=np.float64) * 0.001,
                marker_diameter=0.014,
                metadata={
                    "source": "rigid_body_design_search",
                    "tracking_policy": dict(policies.get(name, {})) if isinstance(policies, dict) else {},
                },
            )
        )
    return patterns


def _load_seed_patterns(seed_candidate: str | None) -> List[MarkerPattern]:
    if not seed_candidate:
        return _design_5marker_patterns()
    payload = json.loads(Path(seed_candidate).read_text(encoding="utf-8"))
    return _patterns_from_record(payload)


def _points_mm(pattern: MarkerPattern) -> List[List[float]]:
    points = np.asarray(pattern.marker_positions, dtype=np.float64) * 1000.0
    return [[float(value) for value in row] for row in points]


def _min_spacing(points: np.ndarray) -> float:
    best = float("inf")
    for a in range(len(points)):
        for b in range(a + 1, len(points)):
            best = min(best, float(np.linalg.norm(points[a] - points[b])))
    return best


if __name__ == "__main__":
    raise SystemExit(main())
