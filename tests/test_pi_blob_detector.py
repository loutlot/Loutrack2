from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.pi.capture import (
    ControlServer,
    ControlServerConfig,
    DummyBackend,
    DummyBackendConfig,
    detect_blobs,
    resolve_debug_preview_enabled,
)


def _expected_centers(cfg: DummyBackendConfig, frame_index: int) -> list[tuple[int, int]]:
    radius = max(1, int(cfg.dot_radius))
    x_low = radius
    y_low = radius
    x_high = max(x_low + 1, cfg.width - radius)
    y_high = max(y_low + 1, cfg.height - radius)

    rng = np.random.default_rng(cfg.seed + frame_index)
    dot_count = max(0, int(cfg.num_dots))
    min_center_distance_sq = float((2 * radius + 1) ** 2)
    centers: list[tuple[int, int]] = []
    max_attempts = max(1, dot_count * 50)
    attempts = 0
    while len(centers) < dot_count and attempts < max_attempts:
        attempts += 1
        x = int(rng.integers(x_low, x_high))
        y = int(rng.integers(y_low, y_high))
        if any((x - cx) ** 2 + (y - cy) ** 2 < min_center_distance_sq for cx, cy in centers):
            continue
        centers.append((x, y))

    centers.sort(key=lambda pt: (pt[1], pt[0]))
    return centers


def test_detect_blobs_finds_three_dots_with_expected_centroids() -> None:
    config = DummyBackendConfig(width=160, height=120, num_dots=3, seed=42, dot_radius=2)

    backend_a = DummyBackend(config)
    backend_b = DummyBackend(config)
    frame = backend_a.next_frame()
    frame_again = backend_b.next_frame()
    assert np.array_equal(frame, frame_again)

    blobs, diagnostics = detect_blobs(frame, threshold=200)
    assert len(blobs) == 3
    assert diagnostics["accepted_blob_count"] == 3

    expected = _expected_centers(config, frame_index=0)
    for blob, (expected_x, expected_y) in zip(blobs, expected):
        assert abs(blob["x"] - float(expected_x)) <= 1.0
        assert abs(blob["y"] - float(expected_y)) <= 1.0
        assert blob["area"] > 0.0


def test_detect_blobs_threshold_255_returns_zero() -> None:
    backend = DummyBackend(DummyBackendConfig(width=120, height=90, num_dots=3, seed=123, dot_radius=2))
    frame = backend.next_frame()
    blobs, diagnostics = detect_blobs(frame, threshold=255)
    assert blobs == []
    assert diagnostics["accepted_blob_count"] == 0


def test_control_server_ping_includes_blob_diagnostics() -> None:
    server = ControlServer(ControlServerConfig(camera_id="pi-cam-01", debug_preview=False))
    response = server._handle_ping("req-1", "pi-cam-01")

    assert response["ack"] is True
    result = response["result"]
    assert isinstance(result, dict)
    assert result["debug_preview_enabled"] is False
    assert result["debug_preview_active"] is False
    assert result["blob_diagnostics"]["threshold"] == 200


def test_resolve_debug_preview_enabled_requires_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.delenv("DISPLAY", raising=False)
    assert resolve_debug_preview_enabled(True) is False


def test_resolve_debug_preview_enabled_accepts_display(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("DISPLAY", ":0")
    assert resolve_debug_preview_enabled(True) is True
