from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.pattern_evaluator import (
    BUILTIN_PATTERNS,
    PatternEvaluationConfig,
    evaluate_patterns,
    format_markdown_report,
    load_patterns_from_rigids_file,
    main,
    self_symmetry_score,
    subset_ambiguity_score,
)
from host.rigid import MarkerPattern, WAIST_PATTERN


def _symmetric_square_pattern() -> MarkerPattern:
    return MarkerPattern(
        name="square",
        marker_positions=np.array(
            [
                [-0.05, -0.05, 0.0],
                [0.05, -0.05, 0.0],
                [0.05, 0.05, 0.0],
                [-0.05, 0.05, 0.0],
            ],
            dtype=np.float64,
        ),
    )


def test_self_symmetry_marks_symmetric_pattern_ambiguous() -> None:
    result = self_symmetry_score(_symmetric_square_pattern())

    assert result["verdict"] == "ambiguous"
    assert result["min_self_symmetry_mm"] < 1e-6
    assert result["best_permutation"] != [0, 1, 2, 3]
    assert result["rotation_angle_deg"] > 1.0


def test_self_symmetry_reports_builtin_waist_values() -> None:
    result = self_symmetry_score(WAIST_PATTERN)

    assert result["name"] == "waist"
    assert result["marker_count"] == 4
    assert result["checked_permutations"] == 23
    assert result["min_self_symmetry_mm"] > 0.0
    assert result["verdict"] in {"ambiguous", "risky", "unique"}


def test_subset_ambiguity_reports_closest_triplets() -> None:
    result = subset_ambiguity_score(WAIST_PATTERN)

    assert result["subset_size"] == 3
    assert result["subset_count"] == 4
    assert result["min_subset_distance_delta_mm"] >= 0.0
    assert result["closest_pair"] is not None


def test_evaluate_patterns_and_markdown_report_include_sections() -> None:
    evaluation = evaluate_patterns([WAIST_PATTERN, _symmetric_square_pattern()])
    report = format_markdown_report(evaluation)

    assert len(evaluation["patterns"]) == 2
    assert len(evaluation["cross_pattern"]) == 1
    assert "Self Symmetry" in report
    assert "Cross Pattern" in report
    assert "waist" in report


def test_load_patterns_from_rigids_file(tmp_path: Path) -> None:
    rigids_path = tmp_path / "tracking_rigids.json"
    rigids_path.write_text(
        json.dumps(
            {
                "custom_rigids": [
                    {
                        "name": "custom",
                        "marker_positions": [[0, 0, 0], [0.1, 0, 0], [0, 0.1, 0]],
                        "marker_diameter_m": 0.012,
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    patterns = load_patterns_from_rigids_file(rigids_path)

    assert set(patterns) == {"custom"}
    assert patterns["custom"].marker_diameter == 0.012


def test_cli_json_output_for_builtin_pattern(capsys) -> None:
    assert "waist" in BUILTIN_PATTERNS

    exit_code = main(["--patterns", "waist", "--json"])

    captured = capsys.readouterr()
    payload = json.loads(captured.out)
    assert exit_code == 0
    assert payload["patterns"][0]["name"] == "waist"
    assert payload["subsets"][0]["name"] == "waist"
