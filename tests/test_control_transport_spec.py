from __future__ import annotations

from pathlib import Path


def test_pi_control_transport_doc_smoke() -> None:
    doc_path = Path(__file__).resolve().parents[1] / "docs" / "pi_control_transport.md"
    text = doc_path.read_text(encoding="utf-8")

    required = [
        "NDJSON",
        "8554",
        "error_code",
        "ping",
        "start",
        "stop",
        "mask_start",
        "255.255.255.255:5000",
    ]
    for token in required:
        assert token in text, f"Missing required token in spec doc: {token}"
