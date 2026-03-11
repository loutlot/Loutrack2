from __future__ import annotations

from typing import Any, Dict, Tuple


WAND_NAME = "wand_l_b5_v1"
WAND_MARKER_DIAMETER_MM = 14.0
WAND_OUTER_SHORT_MM = 182.0
WAND_OUTER_LONG_MM = 257.0
WAND_MID_X_MM = 132.0

# Canonical order: elbow, short, mid, long.
WAND_POINTS_MM: Tuple[Tuple[float, float, float], ...] = (
    (0.0, 0.0, 0.0),
    (0.0, WAND_OUTER_SHORT_MM, 0.0),
    (WAND_MID_X_MM, 0.0, 0.0),
    (WAND_OUTER_LONG_MM, 0.0, 0.0),
)

WAND_MID_RATIO = WAND_MID_X_MM / WAND_OUTER_LONG_MM


def wand_payload() -> Dict[str, Any]:
    return {
        "name": WAND_NAME,
        "marker_diameter_mm": WAND_MARKER_DIAMETER_MM,
        "points_mm": [list(point) for point in WAND_POINTS_MM],
    }
