#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any, Dict

MODULE_SRC_ROOT = Path(__file__).resolve().parents[2]
if __package__ in (None, ""):
    if str(MODULE_SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_SRC_ROOT))
    from host.logger import FrameLogger
    from host.receiver import UDPReceiver
    from host.wand_session import CalibrationSession, CalibrationSessionConfig, FIXED_FPS
else:
    from .logger import FrameLogger
    from .receiver import UDPReceiver
    from .wand_session import CalibrationSession, CalibrationSessionConfig, FIXED_FPS


DEFAULT_OUTPUT = Path("logs") / "extrinsics_wand_metric.jsonl"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture short wand floor/metric log with auto-stop")
    parser.add_argument("--inventory", default=None, help="Optional hosts.ini path")
    parser.add_argument("--udp-port", type=int, default=5000, help="UDP receiver port")
    parser.add_argument("--output", default=str(DEFAULT_OUTPUT), help="Output JSONL path")
    parser.add_argument("--duration-s", type=float, default=3.0, help="Capture duration in seconds")
    parser.add_argument("--exposure-us", type=int, default=8000)
    parser.add_argument("--gain", type=float, default=6.0)
    parser.add_argument("--threshold", type=int, default=150)
    parser.add_argument("--blob-min-diameter-px", type=float, default=2.0)
    parser.add_argument("--blob-max-diameter-px", type=float, default=42.5)
    parser.add_argument("--mask-threshold", type=int, default=120)
    parser.add_argument("--mask-seconds", type=float, default=0.5)
    parser.add_argument("--camera-id", action="append", dest="camera_ids", default=None, help="Optional camera id filter")
    return parser.parse_args()


def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
    return all(bool(resp.get("ack")) for resp in responses.values())


def main() -> int:
    args = parse_args()
    receiver = UDPReceiver(port=args.udp_port)
    receiver.start()
    session = CalibrationSession(
        inventory_path=Path(args.inventory) if args.inventory else None,
        receiver=receiver,
    )
    logger = FrameLogger(log_dir=str(Path(args.output).expanduser().resolve().parent))
    result: Dict[str, Any] = {}
    try:
        targets = session.prepare_targets(camera_ids=args.camera_ids)
        if not targets:
            raise RuntimeError("No healthy cameras found")

        output_path = Path(args.output).expanduser().resolve()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        if output_path.exists():
            output_path.unlink(missing_ok=True)
        log_file = Path(logger.start_recording(session_name=output_path.stem))

        receiver.set_frame_callback(lambda frame: logger.log_frame(frame.to_dict() if hasattr(frame, "to_dict") else dict(frame)))

        config = CalibrationSessionConfig(
            exposure_us=args.exposure_us,
            gain=args.gain,
            fps=FIXED_FPS,
            threshold=args.threshold,
            blob_min_diameter_px=args.blob_min_diameter_px,
            blob_max_diameter_px=args.blob_max_diameter_px,
            duration_s=args.duration_s,
            camera_ids=args.camera_ids,
            mask_params={
                "threshold": args.mask_threshold,
                "seconds": args.mask_seconds,
            },
            capture_kind="wand_metric_capture",
        )

        ack_history = []

        def record(step: str, responses: Dict[str, Dict[str, Any]]) -> None:
            ack_history.append({"step": step, "responses": responses})

        for step, kwargs in (
            ("set_exposure", {"value": config.exposure_us}),
            ("set_gain", {"value": config.gain}),
            ("set_threshold", {"value": config.threshold}),
            ("set_blob_diameter", {"min_px": config.blob_min_diameter_px, "max_px": config.blob_max_diameter_px}),
            ("mask_start", {"threshold": args.mask_threshold, "seconds": args.mask_seconds}),
            ("start", {"mode": "wand_metric_capture"}),
        ):
            responses = session._broadcast(targets, step, **kwargs)
            record(step, responses)
            if not _all_acked(responses):
                raise RuntimeError(f"{step} failed on one or more cameras")

        time.sleep(config.duration_s)
        stop_resp = session._broadcast(targets, "stop")
        record("stop", stop_resp)
        capture_meta = logger.stop_recording()
        result = {
            "ok": True,
            "capture_kind": "wand_metric_capture",
            "duration_s": config.duration_s,
            "log_path": str(log_file),
            "capture_log": capture_meta,
            "targets": [{"camera_id": target.camera_id, "ip": target.ip} for target in targets],
            "ack_history": ack_history,
        }
    finally:
        try:
            meta = logger.stop_recording()
            if result and "capture_log" not in result:
                result["capture_log"] = meta
        except Exception:
            pass
        try:
            receiver.stop()
        except Exception:
            pass
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
