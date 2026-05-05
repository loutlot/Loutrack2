"""Generate printable rigid-body marker fixture prototypes from marker centers.

Coordinates are in millimeters. The generated default model uses a 30 mm
diameter central cylinder, 14 mm marker spheres at each supplied coordinate,
and 5 mm diameter stems from the nearest point on a 25 mm diameter hub circle
to each marker center.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cadquery as cq
from cadquery import Vector


Point3 = tuple[float, float, float]
Point2 = tuple[float, float]


PRESETS: dict[str, list[Point3]] = {
    "head": [
        (39.5, -48.1, 18.9),
        (39.0, 26.9, 29.1),
        (-34.5, 53.8, 12.0),
        (-8.4, -63.9, 8.3),
        (64.0, 6.4, 9.2),
    ],
    "waist": [
        (18.0, -50.0, 37.5),
        (0.6, 61.8, 20.2),
        (-6.6, -13.0, 63.3),
        (22.9, -24.1, 55.9),
        (58.2, -28.6, 4.8),
    ],
    "chest": [
        (3.4, 39.5, 24.7),
        (17.6, -30.4, 54.7),
        (24.4, 58.4, 14.7),
        (63.9, -1.0, 11.8),
        (-23.3, -42.3, 32.6),
    ],
    "left_foot": [
        (-29.7, -52.0, 25.3),
        (62.1, -10.7, 16.1),
        (0.7, 47.4, 44.4),
        (-17.3, -17.8, 49.1),
        (-63.6, -3.5, 13.0),
    ],
    "right_foot": [
        (7.2, 45.2, 46.1),
        (-44.2, 34.2, 33.1),
        (24.0, -22.4, 56.1),
        (-38.0, -5.0, 52.5),
        (-59.5, 24.3, 9.8),
    ],
}


def _load_points(path: Path, body_name: str | None) -> list[Point3]:
    data = json.loads(path.read_text(encoding="utf-8"))
    scale = 1.0
    if isinstance(data, dict) and "marker_positions" in data:
        points = data["marker_positions"]
        if "marker_diameter_m" in data:
            scale = 1000.0
    elif isinstance(data, dict) and "custom_rigids" in data and body_name:
        rigids = data["custom_rigids"]
        if not isinstance(rigids, list):
            raise ValueError("custom_rigids must be a list")
        matches = [
            rigid
            for rigid in rigids
            if isinstance(rigid, dict) and str(rigid.get("name", "")) == str(body_name)
        ]
        if not matches:
            raise ValueError(f"Body {body_name!r} not found in custom_rigids")
        body = matches[0]
        points = body.get("marker_positions")
        if "marker_diameter_m" in body:
            scale = 1000.0
    elif isinstance(data, dict) and body_name:
        body = data[body_name]
        points = body.get("marker_positions", body) if isinstance(body, dict) else body
        if isinstance(body, dict) and "marker_diameter_m" in body:
            scale = 1000.0
    else:
        points = data

    if not isinstance(points, list) or len(points) < 3:
        raise ValueError("Expected at least three marker positions")

    parsed: list[Point3] = []
    for index, point in enumerate(points):
        if not isinstance(point, list | tuple) or len(point) != 3:
            raise ValueError(f"Marker {index} must be a 3D coordinate")
        parsed.append(
            (
                float(point[0]) * scale,
                float(point[1]) * scale,
                float(point[2]) * scale,
            )
        )
    return parsed


def _sphere(radius: float, center: Point3) -> cq.Workplane:
    return cq.Workplane("XY").sphere(radius).translate(center)


def _cylinder_between(start: Point3, end: Point3, radius: float) -> cq.Workplane:
    start_v = Vector(*start)
    end_v = Vector(*end)
    direction = end_v - start_v
    height = direction.Length
    if height <= 0:
        raise ValueError("Stem endpoints must be different")

    cylinder = cq.Workplane("XY").circle(radius).extrude(height)
    z_axis = Vector(0, 0, 1)
    axis = z_axis.cross(direction)
    angle = math.degrees(z_axis.getAngle(direction))
    if axis.Length > 1e-9:
        cylinder = cylinder.rotate((0, 0, 0), axis.toTuple(), angle)
    elif direction.z < 0:
        cylinder = cylinder.rotate((0, 0, 0), (1, 0, 0), 180)
    return cylinder.translate(start)


def _nearest_hub_point(point: Point3, hub_radius: float, z: float) -> Point3:
    x, y, _ = point
    radial = math.hypot(x, y)
    if radial <= 1e-9:
        return (hub_radius, 0.0, z)
    return (hub_radius * x / radial, hub_radius * y / radial, z)


def _distance_point_to_segment_2d(point: Point2, start: Point2, end: Point2) -> float:
    px, py = point
    sx, sy = start
    ex, ey = end
    vx = ex - sx
    vy = ey - sy
    wx = px - sx
    wy = py - sy
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return math.hypot(px - sx, py - sy)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    closest_x = sx + t * vx
    closest_y = sy + t * vy
    return math.hypot(px - closest_x, py - closest_y)


def _hole_clearance_score(
    holes: list[tuple[Point2, float]],
    stem_segments: list[tuple[Point2, Point2]],
    base_radius: float,
    stem_radius: float,
) -> float:
    score = math.inf
    for center, radius in holes:
        cx, cy = center
        score = min(score, base_radius - math.hypot(cx, cy) - radius)
        for start, end in stem_segments:
            score = min(score, _distance_point_to_segment_2d(center, start, end) - radius - stem_radius)
    return score


def _auto_hole_axis(
    points: list[Point3],
    *,
    hub_radius: float,
    base_radius: float,
    hex_hole_offset: float,
    hex_hole_radius: float,
    round_hole_offset: float,
    round_hole_radius: float,
    stem_radius: float,
    samples: int = 720,
) -> Point2:
    stem_segments = [
        (
            (_nearest_hub_point(point, hub_radius, 0.0)[0], _nearest_hub_point(point, hub_radius, 0.0)[1]),
            (point[0], point[1]),
        )
        for point in points
    ]
    best_axis = (1.0, 0.0)
    best_score = -math.inf
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        axis = (math.cos(angle), math.sin(angle))
        holes = [
            ((axis[0] * hex_hole_offset, axis[1] * hex_hole_offset), hex_hole_radius),
            ((axis[0] * round_hole_offset, axis[1] * round_hole_offset), round_hole_radius),
        ]
        score = _hole_clearance_score(holes, stem_segments, base_radius, stem_radius)
        if score > best_score:
            best_axis = axis
            best_score = score
    return best_axis


def _fillet_edges_near(model: cq.Workplane, center: Point3, radius: float, half_size: float) -> cq.Workplane:
    x, y, z = center
    selector = cq.selectors.BoxSelector(
        (x - half_size, y - half_size, z - half_size),
        (x + half_size, y + half_size, z + half_size),
    )
    return model.edges(selector).fillet(radius)


def _extend_start_toward_base(start: Point3, end: Point3, distance: float) -> Point3:
    start_v = Vector(*start)
    end_v = Vector(*end)
    direction = end_v - start_v
    length = direction.Length
    if length <= 0:
        raise ValueError("Stem endpoints must be different")
    extended = start_v - direction.normalized().multiply(distance)
    return extended.toTuple()


def build_fixture(
    points: list[Point3],
    *,
    marker_diameter: float = 14.0,
    marker_z_lift: float = 7.0,
    stem_diameter: float = 5.0,
    hub_diameter: float = 25.0,
    base_diameter: float = 30.0,
    base_thickness: float = 3.0,
    base_top_chamfer: float = 1.0,
    hex_hole_offset: float = 4.0,
    hex_hole_inscribed_diameter: float = 2.7,
    round_hole_offset: float = -10.0,
    round_hole_diameter: float = 5.2,
    fillet_radius: float = 1.0,
    stem_base_extension: float = 1.0,
) -> cq.Workplane:
    if len(points) < 3:
        raise ValueError("At least three marker centers are required")

    lifted_points = [(x, y, z + marker_z_lift) for x, y, z in points]
    base_radius = base_diameter / 2.0
    hub_radius = hub_diameter / 2.0
    stem_radius = stem_diameter / 2.0
    hex_hole_radius = (hex_hole_inscribed_diameter / 2.0) / math.cos(math.pi / 6.0)
    round_hole_radius = round_hole_diameter / 2.0
    hole_axis = _auto_hole_axis(
        lifted_points,
        hub_radius=hub_radius,
        base_radius=base_radius,
        hex_hole_offset=hex_hole_offset,
        hex_hole_radius=hex_hole_radius,
        round_hole_offset=round_hole_offset,
        round_hole_radius=round_hole_radius,
        stem_radius=stem_radius,
    )
    hex_hole_center = (hole_axis[0] * hex_hole_offset, hole_axis[1] * hex_hole_offset)
    round_hole_center = (hole_axis[0] * round_hole_offset, hole_axis[1] * round_hole_offset)

    base = cq.Workplane("XY").circle(base_radius).extrude(base_thickness)
    if base_top_chamfer > 0:
        base = base.edges(">Z").chamfer(base_top_chamfer)
    if hex_hole_inscribed_diameter > 0:
        base = (
            base.faces(">Z")
            .workplane()
            .pushPoints([hex_hole_center])
            .polygon(6, hex_hole_inscribed_diameter, circumscribed=True)
            .cutThruAll()
        )
    if round_hole_diameter > 0:
        base = (
            base.faces(">Z")
            .workplane()
            .pushPoints([round_hole_center])
            .circle(round_hole_diameter / 2.0)
            .cutThruAll()
        )
    model = base
    joint_centers: list[Point3] = []

    for point in lifted_points:
        stem_start = _nearest_hub_point(point, hub_radius, base_thickness)
        extended_stem_start = _extend_start_toward_base(stem_start, point, stem_base_extension)
        stem = _cylinder_between(extended_stem_start, point, stem_radius)
        stem_foot = _sphere(stem_radius, extended_stem_start)
        marker = _sphere(marker_diameter / 2.0, point)
        model = model.union(stem_foot).union(stem).union(marker)
        joint_centers.append(point)

    if fillet_radius > 0:
        for center in joint_centers:
            try:
                model = _fillet_edges_near(
                    model,
                    center,
                    fillet_radius,
                    marker_diameter / 2.0 + stem_diameter,
                )
            except Exception:
                pass

    return model


def export_fixture(model: cq.Workplane, stl_path: Path, step_path: Path | None = None) -> None:
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    cq.exporters.export(model, str(stl_path))
    if step_path is not None:
        step_path.parent.mkdir(parents=True, exist_ok=True)
        cq.exporters.export(model, str(step_path))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--preset", choices=sorted(PRESETS), help="Bundled coordinate preset to export")
    source.add_argument("--all-presets", action="store_true", help="Export every bundled coordinate preset")
    source.add_argument("--input", type=Path, help="JSON file containing marker coordinates")
    parser.add_argument("--body", help="Body name to select when --input contains multiple bodies")
    parser.add_argument("--out", type=Path, required=True, help="STL output path, or output directory with --all-presets")
    parser.add_argument("--step", type=Path, help="Optional STEP output path")
    parser.add_argument("--marker-diameter", type=float, default=14.0, help="Reflective marker diameter in mm")
    parser.add_argument("--marker-z-lift", type=float, default=7.0, help="Extra Z offset applied to marker centers during CAD generation in mm")
    parser.add_argument("--stem-diameter", type=float, default=5.0, help="Stem diameter in mm")
    parser.add_argument("--hub-diameter", type=float, default=25.0, help="Stem start circle diameter in mm")
    parser.add_argument("--base-diameter", type=float, default=30.0, help="Central base diameter in mm")
    parser.add_argument("--base-thickness", type=float, default=4.0, help="Base thickness in mm")
    parser.add_argument("--base-top-chamfer", type=float, default=1.0, help="Top outside edge chamfer for the central base in mm")
    parser.add_argument("--hex-hole-offset", type=float, default=4.0, help="X offset for the hexagonal base hole in mm")
    parser.add_argument("--hex-hole-inscribed-diameter", type=float, default=2.7, help="Inscribed circle diameter for the hexagonal base hole in mm")
    parser.add_argument("--round-hole-offset", type=float, default=-10.0, help="X offset for the round base hole in mm")
    parser.add_argument("--round-hole-diameter", type=float, default=5.2, help="Round base hole diameter in mm")
    parser.add_argument("--fillet-radius", type=float, default=1.0, help="Joint fillet radius in mm")
    parser.add_argument("--stem-base-extension", type=float, default=1.0, help="Stem extension past the hub circle toward the base in mm")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    point_sets = PRESETS.items() if args.all_presets else [(args.preset or args.body or "fixture", PRESETS[args.preset] if args.preset else _load_points(args.input, args.body))]
    for name, points in point_sets:
        model = build_fixture(
            points,
            marker_diameter=args.marker_diameter,
            marker_z_lift=args.marker_z_lift,
            stem_diameter=args.stem_diameter,
            hub_diameter=args.hub_diameter,
            base_diameter=args.base_diameter,
            base_thickness=args.base_thickness,
            base_top_chamfer=args.base_top_chamfer,
            hex_hole_offset=args.hex_hole_offset,
            hex_hole_inscribed_diameter=args.hex_hole_inscribed_diameter,
            round_hole_offset=args.round_hole_offset,
            round_hole_diameter=args.round_hole_diameter,
            fillet_radius=args.fillet_radius,
            stem_base_extension=args.stem_base_extension,
        )
        stl_path = args.out / f"{name}_fixture.stl" if args.all_presets else args.out
        export_fixture(model, stl_path, args.step)


if __name__ == "__main__":
    main()
