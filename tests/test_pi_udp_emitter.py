from __future__ import annotations

import json
import socket
import sys
from pathlib import Path
from typing import cast

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.pi.capture import DummyBackend, DummyBackendConfig, UDPFrameEmitter


def test_udp_frame_emitter_sends_three_frames() -> None:
    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.bind(("127.0.0.1", 0))
    recv_sock.settimeout(1.0)
    sockname = cast(tuple[str, int], recv_sock.getsockname())
    _host, port = sockname

    t_us = [1_700_000_000_000_000]

    def time_us_fn() -> int:
        t_us[0] += 100
        return t_us[0]

    backend = DummyBackend(DummyBackendConfig(width=80, height=60, num_dots=2, seed=1, dot_radius=2))
    emitter = UDPFrameEmitter(
        camera_id="pi-cam-test",
        udp_host="127.0.0.1",
        udp_port=int(port),
        target_fps=200.0,
        backend=backend,
        threshold=200,
        time_us_fn=time_us_fn,
        max_frames=3,
    )

    try:
        emitter.start()

        received: list[dict[str, object]] = []
        for _ in range(3):
            data, addr = cast(tuple[bytes, tuple[str, int]], recv_sock.recvfrom(65536))
            _addr = addr
            text = data.decode("utf-8")
            assert "\n" not in text
            msg_obj = cast(object, json.loads(text))
            assert isinstance(msg_obj, dict)
            msg = cast(dict[str, object], msg_obj)
            received.append(msg)
    finally:
        emitter.stop()
        recv_sock.close()

    assert len(received) == 3

    required_keys = {"camera_id", "timestamp", "frame_index", "blobs"}
    frame_indices: list[int] = []
    for msg in received:
        assert set(msg.keys()) == required_keys
        assert msg["camera_id"] == "pi-cam-test"
        assert isinstance(msg["timestamp"], int)
        assert msg["timestamp"] >= 0
        assert isinstance(msg["frame_index"], int)
        frame_indices.append(msg["frame_index"])
        assert isinstance(msg["blobs"], list)

    assert frame_indices[0] < frame_indices[1] < frame_indices[2]
