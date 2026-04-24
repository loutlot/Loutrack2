"""Rigid-body marker pattern ambiguity evaluator.

The evaluator is diagnostic-only.  It helps identify rigid-body definitions
that are likely to flip, swap, or reacquire ambiguously before changing any
tracking logic.
"""

from __future__ import annotations

import argparse
import itertools
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

from .rigid import (
    CHEST_PATTERN,
    HEAD_PATTERN,
    LEFT_FOOT_PATTERN,
    RIGHT_FOOT_PATTERN,
    WAIST_PATTERN,
    KabschEstimator,
    MarkerPattern,
)


BUILTIN_PATTERNS: Dict[str, MarkerPattern] = {
    pattern.name: pattern
    for pattern in (
        WAIST_PATTERN,
        HEAD_PATTERN,
        CHEST_PATTERN,
        LEFT_FOOT_PATTERN,
        RIGHT_FOOT_PATTERN,
    )
}


@dataclass(frozen=True)
class PatternEvaluationConfig:
    """Thresholds and bounds for pattern ambiguity reporting."""

    ambiguous_rms_m: float = 0.015
    risky_rms_m: float = 0.025
    subset_distance_threshold_m: float = 0.005
    exhaustive_marker_limit: int = 7


def self_symmetry_score(
    pattern: MarkerPattern,
    *,
    config: PatternEvaluationConfig = PatternEvaluationConfig(),
) -> Dict[str, Any]:
    """Evaluate how well a pattern fits itself under non-identity permutations."""
    markers = _marker_array(pattern)
    n_markers = len(markers)
    if n_markers < 3:
        return _empty_self_result(pattern, reason="need_at_least_three_markers")

    best: Optional[Tuple[float, Tuple[int, ...], np.ndarray]] = None
    checked = 0
    for permutation in _candidate_permutations(n_markers, config.exhaustive_marker_limit):
        if permutation == tuple(range(n_markers)):
            continue
        permuted = markers[list(permutation)]
        try:
            rotation, _translation, rms = KabschEstimator.estimate(permuted, markers)
        except Exception:
            continue
        checked += 1
        if best is None or rms < best[0]:
            best = (float(rms), tuple(int(item) for item in permutation), rotation)

    if best is None:
        return _empty_self_result(pattern, reason="no_valid_permutation")

    min_rms_m, best_permutation, rotation = best
    return {
        "name": pattern.name,
        "marker_count": int(n_markers),
        "checked_permutations": int(checked),
        "exhaustive": bool(n_markers <= config.exhaustive_marker_limit),
        "min_self_symmetry_m": float(min_rms_m),
        "min_self_symmetry_mm": float(min_rms_m * 1000.0),
        "best_permutation": list(best_permutation),
        "rotation_angle_deg": _rotation_angle_deg(rotation),
        "verdict": _verdict(min_rms_m, config),
    }


def subset_ambiguity_score(
    pattern: MarkerPattern,
    *,
    subset_size: int = 3,
    config: PatternEvaluationConfig = PatternEvaluationConfig(),
) -> Dict[str, Any]:
    """Report distance-profile collisions among marker subsets."""
    markers = _marker_array(pattern)
    if len(markers) < subset_size:
        return {
            "name": pattern.name,
            "subset_size": int(subset_size),
            "subset_count": 0,
            "ambiguous_pair_count": 0,
            "min_subset_distance_delta_mm": 0.0,
            "closest_pair": None,
        }

    profiles: List[Tuple[Tuple[int, ...], np.ndarray]] = []
    for subset in itertools.combinations(range(len(markers)), subset_size):
        profiles.append((tuple(subset), _subset_distance_profile(markers[list(subset)])))

    ambiguous_pairs = 0
    closest: Optional[Tuple[float, Tuple[int, ...], Tuple[int, ...]]] = None
    for (subset_a, profile_a), (subset_b, profile_b) in itertools.combinations(profiles, 2):
        delta = float(np.sqrt(np.mean((profile_a - profile_b) ** 2)))
        if closest is None or delta < closest[0]:
            closest = (delta, subset_a, subset_b)
        if delta <= config.subset_distance_threshold_m:
            ambiguous_pairs += 1

    return {
        "name": pattern.name,
        "subset_size": int(subset_size),
        "subset_count": int(len(profiles)),
        "ambiguous_pair_count": int(ambiguous_pairs),
        "min_subset_distance_delta_m": float(closest[0]) if closest else 0.0,
        "min_subset_distance_delta_mm": float(closest[0] * 1000.0) if closest else 0.0,
        "closest_pair": {
            "a": list(closest[1]),
            "b": list(closest[2]),
        } if closest else None,
    }


def cross_pattern_match_distance(
    pattern_a: MarkerPattern,
    pattern_b: MarkerPattern,
    *,
    config: PatternEvaluationConfig = PatternEvaluationConfig(),
) -> Dict[str, Any]:
    """Find the best rigid fit between two marker patterns or equal-sized subsets."""
    markers_a = _marker_array(pattern_a)
    markers_b = _marker_array(pattern_b)
    subset_size = min(len(markers_a), len(markers_b))
    if subset_size < 3:
        return {
            "pattern_a": pattern_a.name,
            "pattern_b": pattern_b.name,
            "subset_size": int(subset_size),
            "min_cross_pattern_m": 0.0,
            "min_cross_pattern_mm": 0.0,
            "verdict": "insufficient",
        }

    best: Optional[Tuple[float, Tuple[int, ...], Tuple[int, ...], Tuple[int, ...]]] = None
    combos_a = itertools.combinations(range(len(markers_a)), subset_size)
    combos_b = list(itertools.combinations(range(len(markers_b)), subset_size))

    for subset_a in combos_a:
        points_a = markers_a[list(subset_a)]
        for subset_b in combos_b:
            points_b = markers_b[list(subset_b)]
            for permutation in _candidate_permutations(subset_size, config.exhaustive_marker_limit):
                permuted_a = points_a[list(permutation)]
                try:
                    _rotation, _translation, rms = KabschEstimator.estimate(permuted_a, points_b)
                except Exception:
                    continue
                if best is None or rms < best[0]:
                    best = (
                        float(rms),
                        tuple(int(item) for item in subset_a),
                        tuple(int(item) for item in subset_b),
                        tuple(int(item) for item in permutation),
                    )

    if best is None:
        return {
            "pattern_a": pattern_a.name,
            "pattern_b": pattern_b.name,
            "subset_size": int(subset_size),
            "min_cross_pattern_m": 0.0,
            "min_cross_pattern_mm": 0.0,
            "verdict": "no_valid_fit",
        }

    rms_m, subset_a, subset_b, permutation = best
    return {
        "pattern_a": pattern_a.name,
        "pattern_b": pattern_b.name,
        "subset_size": int(subset_size),
        "min_cross_pattern_m": float(rms_m),
        "min_cross_pattern_mm": float(rms_m * 1000.0),
        "subset_a": list(subset_a),
        "subset_b": list(subset_b),
        "permutation_a_to_b": list(permutation),
        "verdict": _verdict(rms_m, config),
    }


def evaluate_patterns(
    patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig = PatternEvaluationConfig(),
) -> Dict[str, Any]:
    """Evaluate self, subset, and pairwise ambiguity for a set of patterns."""
    pattern_list = list(patterns)
    self_results = [self_symmetry_score(pattern, config=config) for pattern in pattern_list]
    subset_results = [subset_ambiguity_score(pattern, config=config) for pattern in pattern_list]
    cross_results = [
        cross_pattern_match_distance(pattern_a, pattern_b, config=config)
        for pattern_a, pattern_b in itertools.combinations(pattern_list, 2)
    ]
    return {
        "config": {
            "ambiguous_rms_mm": float(config.ambiguous_rms_m * 1000.0),
            "risky_rms_mm": float(config.risky_rms_m * 1000.0),
            "subset_distance_threshold_mm": float(config.subset_distance_threshold_m * 1000.0),
            "exhaustive_marker_limit": int(config.exhaustive_marker_limit),
        },
        "patterns": self_results,
        "subsets": subset_results,
        "cross_pattern": cross_results,
    }


def format_markdown_report(evaluation: Dict[str, Any]) -> str:
    """Render an evaluation payload as a compact markdown report."""
    lines = ["# Rigid Pattern Ambiguity Report", ""]
    lines.append("## Self Symmetry")
    lines.append("| pattern | markers | min RMS mm | rotation deg | permutation | verdict |")
    lines.append("|---|---:|---:|---:|---|---|")
    for item in evaluation.get("patterns", []):
        lines.append(
            "| {name} | {marker_count} | {rms:.2f} | {angle:.1f} | `{perm}` | {verdict} |".format(
                name=item.get("name", ""),
                marker_count=int(item.get("marker_count", 0)),
                rms=float(item.get("min_self_symmetry_mm", 0.0)),
                angle=float(item.get("rotation_angle_deg", 0.0)),
                perm=item.get("best_permutation", []),
                verdict=item.get("verdict", ""),
            )
        )

    lines.extend(["", "## 3-Marker Subsets"])
    lines.append("| pattern | subsets | ambiguous pairs | closest delta mm | closest pair |")
    lines.append("|---|---:|---:|---:|---|")
    for item in evaluation.get("subsets", []):
        lines.append(
            "| {name} | {count} | {ambiguous} | {delta:.2f} | `{pair}` |".format(
                name=item.get("name", ""),
                count=int(item.get("subset_count", 0)),
                ambiguous=int(item.get("ambiguous_pair_count", 0)),
                delta=float(item.get("min_subset_distance_delta_mm", 0.0)),
                pair=item.get("closest_pair"),
            )
        )

    lines.extend(["", "## Cross Pattern"])
    lines.append("| pattern A | pattern B | min RMS mm | subset size | verdict |")
    lines.append("|---|---|---:|---:|---|")
    for item in evaluation.get("cross_pattern", []):
        lines.append(
            "| {a} | {b} | {rms:.2f} | {size} | {verdict} |".format(
                a=item.get("pattern_a", ""),
                b=item.get("pattern_b", ""),
                rms=float(item.get("min_cross_pattern_mm", 0.0)),
                size=int(item.get("subset_size", 0)),
                verdict=item.get("verdict", ""),
            )
        )
    return "\n".join(lines) + "\n"


def load_patterns_from_rigids_file(path: str | Path | None) -> Dict[str, MarkerPattern]:
    """Load custom rigid definitions from calibration/tracking_rigids.json."""
    if path is None:
        return {}
    rigids_path = Path(path)
    if not rigids_path.exists():
        return {}
    try:
        payload = json.loads(rigids_path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    definitions = payload.get("custom_rigids", []) if isinstance(payload, dict) else []
    if not isinstance(definitions, list):
        return {}

    patterns: Dict[str, MarkerPattern] = {}
    for definition in definitions:
        if not isinstance(definition, dict):
            continue
        name = str(definition.get("name") or "").strip()
        if not name or name in BUILTIN_PATTERNS:
            continue
        try:
            positions = np.asarray(definition.get("marker_positions", []), dtype=np.float64)
        except Exception:
            continue
        if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) < 3:
            continue
        try:
            marker_diameter = float(definition.get("marker_diameter_m", 0.014) or 0.014)
        except Exception:
            marker_diameter = 0.014
        patterns[name] = MarkerPattern(
            name=name,
            marker_positions=positions,
            marker_diameter=marker_diameter if marker_diameter > 0.0 else 0.014,
            metadata={
                "notes": str(definition.get("notes") or ""),
                "created_at": int(definition.get("created_at", 0) or 0),
                "source": str(definition.get("source") or "custom_selection"),
            },
        )
    return patterns


def load_available_patterns(rigids_path: str | Path | None = None) -> Dict[str, MarkerPattern]:
    patterns = dict(BUILTIN_PATTERNS)
    patterns.update(load_patterns_from_rigids_file(rigids_path))
    return patterns


def _marker_array(pattern: MarkerPattern) -> np.ndarray:
    markers = np.asarray(pattern.marker_positions, dtype=np.float64)
    if markers.ndim != 2 or markers.shape[1] != 3:
        raise ValueError(f"invalid marker positions for pattern {pattern.name!r}")
    if not np.isfinite(markers).all():
        raise ValueError(f"non-finite marker positions for pattern {pattern.name!r}")
    return markers


def _candidate_permutations(n_items: int, exhaustive_limit: int) -> Iterable[Tuple[int, ...]]:
    identity = tuple(range(n_items))
    if n_items <= exhaustive_limit:
        yield from itertools.permutations(range(n_items))
        return

    yield identity
    for a, b in itertools.combinations(range(n_items), 2):
        permutation = list(identity)
        permutation[a], permutation[b] = permutation[b], permutation[a]
        yield tuple(permutation)


def _subset_distance_profile(points: np.ndarray) -> np.ndarray:
    distances: List[float] = []
    for a, b in itertools.combinations(range(len(points)), 2):
        distances.append(float(np.linalg.norm(points[a] - points[b])))
    return np.asarray(sorted(distances), dtype=np.float64)


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    trace = float(np.trace(rotation))
    cos_theta = min(1.0, max(-1.0, (trace - 1.0) / 2.0))
    return float(np.degrees(np.arccos(cos_theta)))


def _verdict(rms_m: float, config: PatternEvaluationConfig) -> str:
    if rms_m <= config.ambiguous_rms_m:
        return "ambiguous"
    if rms_m <= config.risky_rms_m:
        return "risky"
    return "unique"


def _empty_self_result(pattern: MarkerPattern, *, reason: str) -> Dict[str, Any]:
    return {
        "name": pattern.name,
        "marker_count": int(pattern.num_markers),
        "checked_permutations": 0,
        "exhaustive": False,
        "min_self_symmetry_m": 0.0,
        "min_self_symmetry_mm": 0.0,
        "best_permutation": [],
        "rotation_angle_deg": 0.0,
        "verdict": reason,
    }


def _select_patterns(available: Dict[str, MarkerPattern], names: Optional[str]) -> List[MarkerPattern]:
    if not names:
        return list(available.values())
    requested = [item.strip() for item in names.split(",") if item.strip()]
    missing = [name for name in requested if name not in available]
    if missing:
        raise ValueError(f"unknown pattern(s): {', '.join(missing)}")
    return [available[name] for name in requested]


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Evaluate rigid-body marker pattern ambiguity.")
    parser.add_argument("--rigids", default="calibration/tracking_rigids.json")
    parser.add_argument("--patterns", default=None, help="Comma-separated names. Defaults to all available patterns.")
    parser.add_argument("--json", action="store_true", help="Emit JSON instead of markdown.")
    parser.add_argument("--ambiguous-rms-mm", type=float, default=15.0)
    parser.add_argument("--risky-rms-mm", type=float, default=25.0)
    parser.add_argument("--subset-threshold-mm", type=float, default=5.0)
    args = parser.parse_args(argv)

    config = PatternEvaluationConfig(
        ambiguous_rms_m=float(args.ambiguous_rms_mm) / 1000.0,
        risky_rms_m=float(args.risky_rms_mm) / 1000.0,
        subset_distance_threshold_m=float(args.subset_threshold_mm) / 1000.0,
    )
    available = load_available_patterns(args.rigids)
    evaluation = evaluate_patterns(_select_patterns(available, args.patterns), config=config)
    if args.json:
        print(json.dumps(evaluation, ensure_ascii=False, indent=2))
    else:
        print(format_markdown_report(evaluation), end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
