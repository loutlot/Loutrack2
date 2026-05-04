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
    full_cross_target_m: float = 0.020
    strong_4_cross_target_m: float = 0.014
    strong_3_profile_target_m: float = 0.008
    min_marker_spacing_target_m: float = 0.035
    low_4_volume_target_m3: float = 1.0e-5
    low_3_area_target_m2: float = 7.5e-4
    repeated_edge_delta_m: float = 0.003
    near_isosceles_delta_m: float = 0.005


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
    subset_design = evaluate_subset_design(pattern_list, config=config)
    return {
        "config": {
            "ambiguous_rms_mm": float(config.ambiguous_rms_m * 1000.0),
            "risky_rms_mm": float(config.risky_rms_m * 1000.0),
            "subset_distance_threshold_mm": float(config.subset_distance_threshold_m * 1000.0),
            "exhaustive_marker_limit": int(config.exhaustive_marker_limit),
            "full_cross_target_mm": float(config.full_cross_target_m * 1000.0),
            "strong_4_cross_target_mm": float(config.strong_4_cross_target_m * 1000.0),
            "strong_3_profile_target_mm": float(config.strong_3_profile_target_m * 1000.0),
            "min_marker_spacing_target_mm": float(config.min_marker_spacing_target_m * 1000.0),
        },
        "patterns": self_results,
        "subsets": subset_results,
        "cross_pattern": cross_results,
        "subset_design": subset_design,
    }


def evaluate_subset_design(
    patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig = PatternEvaluationConfig(),
) -> Dict[str, Any]:
    """Score 5-marker design candidates and emit strong subset metadata."""
    pattern_list = list(patterns)
    body_results = [
        _evaluate_body_subsets(pattern, pattern_list, config=config) for pattern in pattern_list
    ]
    full_cross = _full_cross_summary(pattern_list, config=config)
    pass_gate_2 = bool(body_results) and all(
        item["gate_2_pass"] for item in body_results
    ) and full_cross["worst_full_cross_m"] >= config.full_cross_target_m
    return {
        "gate": "geometry",
        "pass": pass_gate_2,
        "full_cross": full_cross,
        "bodies": body_results,
        "tracking_policy": {
            item["name"]: {
                "boot_min_markers": 4,
                "reacquire_min_markers": 4,
                "continue_min_markers": 3,
                "strong_4_subsets": item["strong_4_subsets"],
                "strong_3_subsets": item["strong_3_subsets"],
            }
            for item in body_results
        },
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
                "tracking_policy": definition.get("tracking_policy", {}),
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


def _evaluate_body_subsets(
    pattern: MarkerPattern,
    all_patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig,
) -> Dict[str, Any]:
    markers = _marker_array(pattern)
    subset4 = [
        _score_subset(pattern, subset, all_patterns, config=config)
        for subset in itertools.combinations(range(len(markers)), 4)
    ]
    subset3 = [
        _score_subset(pattern, subset, all_patterns, config=config)
        for subset in itertools.combinations(range(len(markers)), 3)
    ]
    subset4.sort(key=lambda item: float(item["score_m"]), reverse=True)
    subset3.sort(key=lambda item: float(item["score_m"]), reverse=True)
    strong4 = [item for item in subset4 if item["strong"]]
    strong3 = [item for item in subset3 if item["strong"]]
    spacing = _marker_spacing_summary(markers)
    gate_2_pass = bool(strong4) and bool(strong3) and spacing["min_spacing_m"] >= config.min_marker_spacing_target_m
    return {
        "name": pattern.name,
        "marker_count": int(len(markers)),
        "min_marker_spacing_m": spacing["min_spacing_m"],
        "min_marker_spacing_mm": spacing["min_spacing_mm"],
        "closest_marker_pair": spacing["closest_pair"],
        "strong_4_subsets": [item["subset"] for item in strong4],
        "strong_3_subsets": [item["subset"] for item in strong3],
        "best_4_subset": _public_subset_score(subset4[0]) if subset4 else None,
        "best_3_subset": _public_subset_score(subset3[0]) if subset3 else None,
        "subsets_4": [_public_subset_score(item) for item in subset4],
        "subsets_3": [_public_subset_score(item) for item in subset3],
        "gate_2_pass": gate_2_pass,
    }


def _score_subset(
    pattern: MarkerPattern,
    subset: Tuple[int, ...],
    all_patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig,
) -> Dict[str, Any]:
    points = _marker_array(pattern)[list(subset)]
    subset_size = len(subset)
    profile_margin = _profile_margin(pattern.name, subset, points, all_patterns)
    cross_margin = _cross_subset_margin(pattern.name, subset, points, all_patterns, config=config)
    geometry = _subset_geometry(points)
    if subset_size == 4:
        margin = cross_margin["margin_m"]
        target = config.strong_4_cross_target_m
    else:
        margin = profile_margin["margin_m"]
        target = config.strong_3_profile_target_m
    reasons = _subset_rejection_reasons(
        subset_size,
        margin,
        target,
        geometry,
        cross_margin,
        profile_margin,
        config=config,
    )
    return {
        "subset": [int(item) for item in subset],
        "subset_size": int(subset_size),
        "score_m": float(margin),
        "score_mm": float(margin * 1000.0),
        "strong": not reasons,
        "rejection_reasons": reasons,
        "nearest_cross_body_neighbor": cross_margin["nearest"],
        "nearest_profile_neighbor": profile_margin["nearest"],
        **geometry,
    }


def _cross_subset_margin(
    source_name: str,
    source_subset: Tuple[int, ...],
    source_points: np.ndarray,
    all_patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig,
) -> Dict[str, Any]:
    subset_size = len(source_subset)
    best: Optional[Tuple[float, str, Tuple[int, ...], Tuple[int, ...]]] = None
    for other in all_patterns:
        if other.name == source_name:
            continue
        other_markers = _marker_array(other)
        if len(other_markers) < subset_size:
            continue
        for other_subset in itertools.combinations(range(len(other_markers)), subset_size):
            other_points = other_markers[list(other_subset)]
            for permutation in _candidate_permutations(subset_size, config.exhaustive_marker_limit):
                try:
                    _rotation, _translation, rms = KabschEstimator.estimate(
                        source_points[list(permutation)], other_points
                    )
                except Exception:
                    continue
                if best is None or rms < best[0]:
                    best = (float(rms), other.name, tuple(other_subset), tuple(permutation))
    if best is None:
        return {"margin_m": 0.0, "nearest": None}
    rms, other_name, other_subset, permutation = best
    return {
        "margin_m": float(rms),
        "nearest": {
            "body": other_name,
            "subset": list(other_subset),
            "permutation": list(permutation),
            "rms_m": float(rms),
            "rms_mm": float(rms * 1000.0),
        },
    }


def _profile_margin(
    source_name: str,
    source_subset: Tuple[int, ...],
    source_points: np.ndarray,
    all_patterns: Sequence[MarkerPattern],
) -> Dict[str, Any]:
    source_profile = _subset_distance_profile(source_points)
    subset_size = len(source_subset)
    best: Optional[Tuple[float, str, Tuple[int, ...]]] = None
    for other in all_patterns:
        other_markers = _marker_array(other)
        for other_subset in itertools.combinations(range(len(other_markers)), subset_size):
            if other.name == source_name and tuple(other_subset) == source_subset:
                continue
            other_profile = _subset_distance_profile(other_markers[list(other_subset)])
            delta = float(np.sqrt(np.mean((source_profile - other_profile) ** 2)))
            if best is None or delta < best[0]:
                best = (delta, other.name, tuple(other_subset))
    if best is None:
        return {"margin_m": 0.0, "nearest": None}
    delta, other_name, other_subset = best
    return {
        "margin_m": float(delta),
        "nearest": {
            "body": other_name,
            "subset": list(other_subset),
            "profile_delta_m": float(delta),
            "profile_delta_mm": float(delta * 1000.0),
        },
    }


def _subset_geometry(points: np.ndarray) -> Dict[str, Any]:
    edges = _subset_distance_profile(points)
    result: Dict[str, Any] = {
        "min_edge_m": float(edges[0]) if len(edges) else 0.0,
        "min_edge_mm": float(edges[0] * 1000.0) if len(edges) else 0.0,
        "edge_lengths_mm": [float(item * 1000.0) for item in edges],
    }
    if len(points) == 3:
        area = 0.5 * float(np.linalg.norm(np.cross(points[1] - points[0], points[2] - points[0])))
        result["triangle_area_m2"] = area
        result["triangle_area_mm2"] = float(area * 1_000_000.0)
    if len(points) == 4:
        volume = abs(float(np.linalg.det(np.vstack([points[1] - points[0], points[2] - points[0], points[3] - points[0]])))) / 6.0
        result["tetra_volume_m3"] = volume
        result["tetra_volume_mm3"] = float(volume * 1_000_000_000.0)
    return result


def _subset_rejection_reasons(
    subset_size: int,
    margin: float,
    target: float,
    geometry: Dict[str, Any],
    cross_margin: Dict[str, Any],
    profile_margin: Dict[str, Any],
    *,
    config: PatternEvaluationConfig,
) -> List[str]:
    reasons: List[str] = []
    if margin < target:
        reasons.append("low_margin_against_another_body_or_subset")
    if float(geometry.get("min_edge_m", 0.0)) < config.min_marker_spacing_target_m:
        reasons.append("low_marker_spacing")
    if subset_size == 4 and float(geometry.get("tetra_volume_m3", 0.0)) < config.low_4_volume_target_m3:
        reasons.append("low_4_point_volume")
    if subset_size == 3 and float(geometry.get("triangle_area_m2", 0.0)) < config.low_3_area_target_m2:
        reasons.append("low_3_point_triangle_area")
    edges = [float(item) / 1000.0 for item in geometry.get("edge_lengths_mm", [])]
    margin_is_low = margin < target
    if margin_is_low and _has_repeated_edges(edges, config.repeated_edge_delta_m):
        reasons.append("repeated_edge_lengths")
    if subset_size == 3 and margin_is_low and _has_repeated_edges(edges, config.near_isosceles_delta_m):
        reasons.append("near_isosceles_or_symmetric_profile")
    if cross_margin.get("nearest") is None or profile_margin.get("nearest") is None:
        reasons.append("nearest_cross_body_neighbor_unavailable")
    return reasons


def _has_repeated_edges(edges: Sequence[float], threshold: float) -> bool:
    for a, b in itertools.combinations(edges, 2):
        if abs(float(a) - float(b)) <= threshold:
            return True
    return False


def _marker_spacing_summary(markers: np.ndarray) -> Dict[str, Any]:
    best: Optional[Tuple[float, Tuple[int, int]]] = None
    for a, b in itertools.combinations(range(len(markers)), 2):
        distance = float(np.linalg.norm(markers[a] - markers[b]))
        if best is None or distance < best[0]:
            best = (distance, (a, b))
    if best is None:
        return {"min_spacing_m": 0.0, "min_spacing_mm": 0.0, "closest_pair": None}
    return {
        "min_spacing_m": float(best[0]),
        "min_spacing_mm": float(best[0] * 1000.0),
        "closest_pair": list(best[1]),
    }


def _full_cross_summary(
    patterns: Sequence[MarkerPattern],
    *,
    config: PatternEvaluationConfig,
) -> Dict[str, Any]:
    results = [
        cross_pattern_match_distance(pattern_a, pattern_b, config=config)
        for pattern_a, pattern_b in itertools.combinations(patterns, 2)
    ]
    if not results:
        return {"worst_full_cross_m": 0.0, "worst_full_cross_mm": 0.0, "closest_pair": None}
    worst = min(results, key=lambda item: float(item.get("min_cross_pattern_m", 0.0)))
    return {
        "worst_full_cross_m": float(worst.get("min_cross_pattern_m", 0.0)),
        "worst_full_cross_mm": float(worst.get("min_cross_pattern_mm", 0.0)),
        "closest_pair": worst,
        "pairs": results,
    }


def _public_subset_score(item: Dict[str, Any]) -> Dict[str, Any]:
    public = dict(item)
    return public


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
