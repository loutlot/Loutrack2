#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast

import cv2
import numpy as np


A4_WIDTH_MM = 210.0
A4_HEIGHT_MM = 297.0

DEFAULT_SQUARES_X = 6
DEFAULT_SQUARES_Y = 8
DEFAULT_SQUARE_LENGTH_MM = 30.0
DEFAULT_MARKER_LENGTH_MM = 20.0
DEFAULT_DICTIONARY = "DICT_6X6_250"
DEFAULT_DPI = 300


@dataclass
class BoardConfig:
    output: str
    squares_x: int
    squares_y: int
    square_length_mm: float
    marker_length_mm: float
    dictionary: str
    dpi: int
    write_metadata: bool


def parse_args() -> BoardConfig:
    parser = argparse.ArgumentParser(description="Generate a Charuco board as an A4 PDF")
    _ = parser.add_argument(
        "--output",
        default="calibration/boards/charuco_6x8_30mm_a4.pdf",
        help="Output PDF path",
    )
    _ = parser.add_argument(
        "--squares-x",
        type=int,
        default=DEFAULT_SQUARES_X,
        help="Number of Charuco squares in X direction",
    )
    _ = parser.add_argument(
        "--squares-y",
        type=int,
        default=DEFAULT_SQUARES_Y,
        help="Number of Charuco squares in Y direction",
    )
    _ = parser.add_argument(
        "--square-length-mm",
        type=float,
        default=DEFAULT_SQUARE_LENGTH_MM,
        help="Square side length in millimeters",
    )
    _ = parser.add_argument(
        "--marker-length-mm",
        type=float,
        default=DEFAULT_MARKER_LENGTH_MM,
        help="Marker side length in millimeters",
    )
    _ = parser.add_argument(
        "--dictionary",
        default=DEFAULT_DICTIONARY,
        help="OpenCV ArUco dictionary name (e.g. DICT_6X6_250)",
    )
    _ = parser.add_argument(
        "--dpi",
        type=int,
        default=DEFAULT_DPI,
        help="Print DPI used for A4 rasterization",
    )
    _ = parser.add_argument(
        "--write-metadata",
        action="store_true",
        help="Also write JSON metadata alongside the PDF",
    )
    namespace = parser.parse_args()

    output = getattr(namespace, "output", "calibration/boards/charuco_6x8_30mm_a4.pdf")
    squares_x = getattr(namespace, "squares_x", DEFAULT_SQUARES_X)
    squares_y = getattr(namespace, "squares_y", DEFAULT_SQUARES_Y)
    square_length_mm = getattr(namespace, "square_length_mm", DEFAULT_SQUARE_LENGTH_MM)
    marker_length_mm = getattr(namespace, "marker_length_mm", DEFAULT_MARKER_LENGTH_MM)
    dictionary = getattr(namespace, "dictionary", DEFAULT_DICTIONARY)
    dpi = getattr(namespace, "dpi", DEFAULT_DPI)
    write_metadata = getattr(namespace, "write_metadata", False)

    if not isinstance(output, str):
        raise SystemExit("--output must be a string")
    if not isinstance(squares_x, int):
        raise SystemExit("--squares-x must be an integer")
    if not isinstance(squares_y, int):
        raise SystemExit("--squares-y must be an integer")
    if not isinstance(square_length_mm, float):
        raise SystemExit("--square-length-mm must be a number")
    if not isinstance(marker_length_mm, float):
        raise SystemExit("--marker-length-mm must be a number")
    if not isinstance(dictionary, str):
        raise SystemExit("--dictionary must be a string")
    if not isinstance(dpi, int):
        raise SystemExit("--dpi must be an integer")
    if not isinstance(write_metadata, bool):
        raise SystemExit("--write-metadata must be a boolean flag")

    return BoardConfig(
        output=output,
        squares_x=squares_x,
        squares_y=squares_y,
        square_length_mm=square_length_mm,
        marker_length_mm=marker_length_mm,
        dictionary=dictionary,
        dpi=dpi,
        write_metadata=write_metadata,
    )


def mm_to_px(mm_value: float, dpi: int) -> int:
    inches = mm_value / 25.4
    return int(round(inches * dpi))


def get_dictionary(dictionary_name: str) -> Any:
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dictionary_name, None)
    if dictionary_id is None:
        supported = sorted(name for name in dir(aruco) if name.startswith("DICT_"))
        joined = ", ".join(supported)
        raise ValueError(
            f"Invalid dictionary '{dictionary_name}'. Supported values: {joined}"
        )

    get_predefined = getattr(aruco, "getPredefinedDictionary", None)
    if callable(get_predefined):
        return get_predefined(dictionary_id)

    dictionary_get = getattr(aruco, "Dictionary_get", None)
    if callable(dictionary_get):
        return dictionary_get(dictionary_id)

    raise RuntimeError("OpenCV ArUco dictionary API not found")


def create_charuco_board(
    squares_x: int,
    squares_y: int,
    square_length: float,
    marker_length: float,
    dictionary: Any,
) -> Any:
    aruco = cv2.aruco
    if hasattr(aruco, "CharucoBoard"):
        board_cls = aruco.CharucoBoard
        try:
            return board_cls((squares_x, squares_y), square_length, marker_length, dictionary)
        except TypeError:
            create_fn = getattr(board_cls, "create", None)
            if callable(create_fn):
                return create_fn(
                    squares_x,
                    squares_y,
                    square_length,
                    marker_length,
                    dictionary,
                )

    create_legacy = getattr(aruco, "CharucoBoard_create", None)
    if callable(create_legacy):
        return create_legacy(
            squares_x,
            squares_y,
            square_length,
            marker_length,
            dictionary,
        )

    raise RuntimeError("OpenCV CharucoBoard API not found")


def draw_charuco_image(board: Any, width_px: int, height_px: int) -> np.ndarray:
    size = (int(width_px), int(height_px))
    generate_image = getattr(board, "generateImage", None)
    if callable(generate_image):
        return cast(np.ndarray, generate_image(size))

    draw_fn = getattr(board, "draw", None)
    if callable(draw_fn):
        return cast(np.ndarray, draw_fn(size))
    raise RuntimeError("OpenCV Charuco draw API not found")


def main() -> int:
    args = parse_args()

    if args.squares_x <= 1 or args.squares_y <= 1:
        raise SystemExit("--squares-x and --squares-y must be >= 2")
    if args.square_length_mm <= 0:
        raise SystemExit("--square-length-mm must be > 0")
    if args.marker_length_mm <= 0:
        raise SystemExit("--marker-length-mm must be > 0")
    if args.marker_length_mm >= args.square_length_mm:
        raise SystemExit("--marker-length-mm must be smaller than --square-length-mm")
    if args.dpi <= 0:
        raise SystemExit("--dpi must be > 0")

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    a4_width_px = mm_to_px(A4_WIDTH_MM, args.dpi)
    a4_height_px = mm_to_px(A4_HEIGHT_MM, args.dpi)

    square_length_px = mm_to_px(args.square_length_mm, args.dpi)
    marker_length_px = int(round(square_length_px * (args.marker_length_mm / args.square_length_mm)))

    board_width_px = args.squares_x * square_length_px
    board_height_px = args.squares_y * square_length_px
    if board_width_px > a4_width_px or board_height_px > a4_height_px:
        raise SystemExit(
            "Board does not fit on A4 at this DPI and square length. "
            + "Use a lower DPI or smaller square length."
        )

    try:
        dictionary = get_dictionary(args.dictionary)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    board = create_charuco_board(
        squares_x=args.squares_x,
        squares_y=args.squares_y,
        square_length=float(square_length_px),
        marker_length=float(marker_length_px),
        dictionary=dictionary,
    )

    board_img = draw_charuco_image(board, board_width_px, board_height_px)
    if board_img.dtype != np.uint8:
        board_img = board_img.astype(np.uint8)

    page_img = np.full((a4_height_px, a4_width_px), 255, dtype=np.uint8)
    x0 = (a4_width_px - board_width_px) // 2
    y0 = (a4_height_px - board_height_px) // 2
    page_img[y0 : y0 + board_height_px, x0 : x0 + board_width_px] = board_img

    pil_image_module = __import__("PIL.Image", fromlist=["fromarray"])
    fromarray_fn = getattr(pil_image_module, "fromarray")
    pil_page = fromarray_fn(page_img, mode="L")
    pil_page.save(output_path, "PDF", resolution=float(args.dpi))

    if args.write_metadata:
        metadata_path = output_path.with_suffix(".json")
        metadata = {
            "squares_x": int(args.squares_x),
            "squares_y": int(args.squares_y),
            "square_length_mm": float(args.square_length_mm),
            "marker_length_mm": float(args.marker_length_mm),
            "dictionary": str(args.dictionary),
            "dpi": int(args.dpi),
            "a4_mm": {"width": A4_WIDTH_MM, "height": A4_HEIGHT_MM},
            "pixel_size": {
                "a4": {"width": a4_width_px, "height": a4_height_px},
                "board": {"width": board_width_px, "height": board_height_px},
                "square": square_length_px,
                "marker": marker_length_px,
            },
            "notes": [
                "Board is centered on A4 with preserved square geometry.",
                "Print at 100% scale with no fit-to-page.",
            ],
        }
        _ = metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
