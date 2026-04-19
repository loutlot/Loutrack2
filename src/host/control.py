import json
import socket
import uuid
import argparse
import sys
from pathlib import Path
from typing import Optional, Dict, Any


MODULE_SRC_ROOT = Path(__file__).resolve().parents[1]
if str(MODULE_SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(MODULE_SRC_ROOT))

from pi.service.control_manifest import HOST_CONTROL_CLI_COMMANDS


def public_cli_commands() -> tuple[str, ...]:
    return HOST_CONTROL_CLI_COMMANDS


def _recv_line(sock: socket.socket, _buf: bytearray = None) -> str:
    """
    Read bytes from socket until a newline; return the decoded line (no newline).

    The function reads exactly one NDJSON line.  Any bytes that arrive after
    the first newline in the same recv() call are discarded — this is safe
    because the current protocol is strictly request/response (1:1) per
    connection and the server closes the connection after one reply.
    """
    buf = bytearray()
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Socket closed by peer before newline")
        buf.extend(chunk)
        nl = buf.find(b"\n")
        if nl >= 0:
            return buf[:nl].decode("utf-8")


def _build_request(
    camera_id: str,
    cmd: str,
    params: Optional[Dict[str, Any]] = None,
    request_id: Optional[str] = None,
) -> Dict[str, Any]:
    """Construct a well-formed NDJSON control request dict."""
    req: Dict[str, Any] = {
        "request_id": request_id or str(uuid.uuid4()),
        "camera_id": camera_id,
        "cmd": cmd,
    }
    if params:
        req["params"] = params
    return req


def send_request(ip: str, port: int, request_dict: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
    """Send a single NDJSON request and read the NDJSON response.

    Returns the parsed JSON response as a dict.
    """
    if not isinstance(request_dict, dict):
        raise TypeError("request_dict must be a dict")

    req = dict(request_dict)
    if not req.get("request_id"):
        req["request_id"] = str(uuid.uuid4())
    if not req.get("camera_id"):
        raise ValueError("request must include camera_id")

    payload = json.dumps(req, separators=(",", ":")).encode("utf-8") + b"\n"

    with socket.create_connection((ip, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(payload)
        resp_line = _recv_line(sock)
    try:
        return json.loads(resp_line)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON from server: {resp_line!r}") from e


def _send(
    ip: str,
    port: int,
    camera_id: str,
    cmd: str,
    params: Optional[Dict[str, Any]] = None,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    """Internal helper: build and send a control request."""
    return send_request(
        ip, port, _build_request(camera_id, cmd, params, request_id), timeout=timeout
    )


def ping(ip: str, port: int, camera_id: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "ping", request_id=request_id, timeout=timeout)


def start(ip: str, port: int, camera_id: str, mode: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "start", {"mode": mode}, request_id=request_id, timeout=timeout)


def stop(ip: str, port: int, camera_id: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "stop", request_id=request_id, timeout=timeout)


def set_exposure(ip: str, port: int, camera_id: str, value: int, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_exposure", {"value": int(value)}, request_id=request_id, timeout=timeout)


def set_gain(ip: str, port: int, camera_id: str, value: float, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_gain", {"value": float(value)}, request_id=request_id, timeout=timeout)


def set_fps(ip: str, port: int, camera_id: str, value: int, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_fps", {"value": int(value)}, request_id=request_id, timeout=timeout)


def set_focus(ip: str, port: int, camera_id: str, value: float, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_focus", {"value": float(value)}, request_id=request_id, timeout=timeout)


def set_threshold(ip: str, port: int, camera_id: str, value: int, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_threshold", {"value": int(value)}, request_id=request_id, timeout=timeout)


def set_blob_diameter(
    ip: str,
    port: int,
    camera_id: str,
    min_px: Optional[float] = None,
    max_px: Optional[float] = None,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    params: Dict[str, Any] = {}
    if min_px is not None:
        params["min_px"] = float(min_px)
    if max_px is not None:
        params["max_px"] = float(max_px)
    return _send(ip, port, camera_id, "set_blob_diameter", params or None, request_id=request_id, timeout=timeout)


def set_circularity_min(
    ip: str,
    port: int,
    camera_id: str,
    value: float,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "set_circularity_min", {"value": float(value)}, request_id=request_id, timeout=timeout)


def set_preview(
    ip: str,
    port: int,
    camera_id: str,
    render_enabled: Optional[bool] = None,
    overlays: Optional[Dict[str, bool]] = None,
    charuco: Optional[Dict[str, Any]] = None,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    params: Dict[str, Any] = {}
    if render_enabled is not None:
        params["render_enabled"] = bool(render_enabled)
    if overlays:
        params["overlays"] = dict(overlays)
    if charuco:
        params["charuco"] = dict(charuco)
    return _send(ip, port, camera_id, "set_preview", params or None, request_id=request_id, timeout=timeout)


def intrinsics_start(
    ip: str,
    port: int,
    camera_id: str,
    *,
    square_length_mm: float,
    marker_length_mm: float | None = None,
    squares_x: int = 6,
    squares_y: int = 8,
    min_frames: int = 25,
    cooldown_s: float = 1.5,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    params: Dict[str, Any] = {
        "camera_id": str(camera_id),
        "square_length_mm": float(square_length_mm),
        "squares_x": int(squares_x),
        "squares_y": int(squares_y),
        "min_frames": int(min_frames),
        "cooldown_s": float(cooldown_s),
    }
    if marker_length_mm is not None:
        params["marker_length_mm"] = float(marker_length_mm)
    return _send(ip, port, camera_id, "intrinsics_start", params, request_id=request_id, timeout=timeout)


def intrinsics_stop(
    ip: str,
    port: int,
    camera_id: str,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "intrinsics_stop", request_id=request_id, timeout=timeout)


def intrinsics_clear(
    ip: str,
    port: int,
    camera_id: str,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "intrinsics_clear", request_id=request_id, timeout=timeout)


def intrinsics_get_corners(
    ip: str,
    port: int,
    camera_id: str,
    *,
    start_index: int = 0,
    max_frames: Optional[int] = None,
    request_id: Optional[str] = None,
    timeout: float = 10.0,
) -> Dict[str, Any]:
    params: Dict[str, Any] = {"start_index": int(start_index)}
    if max_frames is not None:
        params["max_frames"] = int(max_frames)
    return _send(
        ip,
        port,
        camera_id,
        "intrinsics_get_corners",
        params,
        request_id=request_id,
        timeout=timeout,
    )


def intrinsics_calibrate(
    ip: str,
    port: int,
    camera_id: str,
    request_id: Optional[str] = None,
    timeout: float = 15.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "intrinsics_calibrate", request_id=request_id, timeout=timeout)


def intrinsics_status(
    ip: str,
    port: int,
    camera_id: str,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "intrinsics_status", request_id=request_id, timeout=timeout)


def _parse_bool_arg(value: str) -> bool:
    raw = str(value).strip().lower()
    if raw in {"1", "true", "t", "yes", "y", "on"}:
        return True
    if raw in {"0", "false", "f", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"invalid boolean value: {value}")


def mask_start(
    ip: str,
    port: int,
    camera_id: str,
    threshold: Optional[int] = None,
    frames: Optional[int] = None,
    seconds: Optional[float] = None,
    hit_ratio: Optional[float] = None,
    min_area: Optional[int] = None,
    dilate: Optional[int] = None,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    params: Dict[str, Any] = {}
    if threshold is not None:
        params["threshold"] = int(threshold)
    if frames is not None:
        params["frames"] = int(frames)
    if seconds is not None:
        params["seconds"] = float(seconds)
    if hit_ratio is not None:
        params["hit_ratio"] = float(hit_ratio)
    if min_area is not None:
        params["min_area"] = int(min_area)
    if dilate is not None:
        params["dilate"] = int(dilate)
    return _send(ip, port, camera_id, "mask_start", params or None, request_id=request_id, timeout=timeout)


def mask_stop(
    ip: str,
    port: int,
    camera_id: str,
    request_id: Optional[str] = None,
    timeout: float = 5.0,
) -> Dict[str, Any]:
    return _send(ip, port, camera_id, "mask_stop", request_id=request_id, timeout=timeout)


def _print_json_and_exit(resp: Dict[str, Any]) -> None:
    print(json.dumps(resp))
    if isinstance(resp, dict) and resp.get("ack") is True:
        sys.exit(0)
    else:
        sys.exit(1)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="host.control", description="Pi host control client (NDJSON over TCP)")
    parser.add_argument("--ip", required=True, help="IP address of the Pi control server")
    parser.add_argument("--port", type=int, default=8554, help="Port of the Pi control server (default 8554)")
    parser.add_argument("--camera-id", dest="camera_id", required=True, help="Camera ID on the Pi (e.g., pi-cam-01)")
    parser.add_argument("--request-id", dest="request_id", default=None, help="Optional request_id to attach to the request")

    sub = parser.add_subparsers(dest="cmd", required=True)

    sub.add_parser("ping", help="Send a ping to the server")

    start_p = sub.add_parser("start", help="Start server in given mode")
    start_p.add_argument("--mode", required=True, help="Mode to start (e.g., 'capture')")

    sub.add_parser("stop", help="Stop server")

    exp_p = sub.add_parser("set_exposure", help="Set sensor exposure")
    exp_p.add_argument("--value", type=int, required=True, help="Exposure value to set (μs)")

    gain_p = sub.add_parser("set_gain", help="Set sensor gain")
    gain_p.add_argument("--value", type=float, required=True, help="Gain value to set")

    fps_p = sub.add_parser("set_fps", help="Set frames per second")
    fps_p.add_argument("--value", type=int, required=True, help="FPS value to set")

    focus_p = sub.add_parser("set_focus", help="Set lens focus (LensPosition)")
    focus_p.add_argument("--value", type=float, required=True, help="Focus value to set")

    threshold_p = sub.add_parser("set_threshold", help="Set blob threshold brightness")
    threshold_p.add_argument("--value", type=int, required=True, help="Threshold value (0-255)")

    blob_diameter_p = sub.add_parser("set_blob_diameter", help="Set blob diameter filter (px)")
    blob_diameter_p.add_argument("--min-px", dest="min_px", type=float, default=None, help="Minimum diameter in px")
    blob_diameter_p.add_argument("--max-px", dest="max_px", type=float, default=None, help="Maximum diameter in px")

    circularity_p = sub.add_parser("set_circularity_min", help="Set minimum blob circularity")
    circularity_p.add_argument("--value", type=float, required=True, help="Circularity lower bound [0,1]")

    mask_start_p = sub.add_parser("mask_start", help="Initialize static mask on Pi")
    mask_start_p.add_argument("--threshold", type=int, help="Threshold value")
    mask_start_p.add_argument("--frames", type=int, help="Initialization frame count")
    mask_start_p.add_argument("--seconds", type=float, help="Initialization duration in seconds")
    mask_start_p.add_argument("--hit-ratio", dest="hit_ratio", type=float, help="Hit ratio (0,1]")
    mask_start_p.add_argument("--min-area", dest="min_area", type=int, help="Minimum mask area")
    mask_start_p.add_argument("--dilate", type=int, help="Dilate radius")

    sub.add_parser("mask_stop", help="Clear static mask on Pi")

    preview_p = sub.add_parser("set_preview", help="Configure MJPEG preview rendering")
    preview_p.add_argument("--render-enabled", type=_parse_bool_arg, default=None, help="Enable/disable MJPEG rendering")
    preview_p.add_argument("--overlay-blob", type=_parse_bool_arg, default=None, help="Overlay blob circles")
    preview_p.add_argument("--overlay-mask", type=_parse_bool_arg, default=None, help="Overlay static mask")
    preview_p.add_argument("--overlay-text", type=_parse_bool_arg, default=None, help="Overlay diagnostics text")
    preview_p.add_argument("--overlay-charuco", type=_parse_bool_arg, default=None, help="Overlay Charuco corners")
    preview_p.add_argument("--charuco-dictionary", default=None, help="Charuco dictionary name (e.g. DICT_6X6_250)")
    preview_p.add_argument("--charuco-squares-x", type=int, default=None, help="Charuco board squares in X")
    preview_p.add_argument("--charuco-squares-y", type=int, default=None, help="Charuco board squares in Y")
    preview_p.add_argument("--charuco-square-length-mm", type=float, default=None, help="Charuco square length (mm)")
    preview_p.add_argument("--charuco-marker-length-mm", type=float, default=None, help="Charuco marker length (mm)")

    intrinsics_start_p = sub.add_parser("intrinsics_start", help="Start Pi-side intrinsics frame capture")
    intrinsics_start_p.add_argument("--square-length-mm", type=float, required=True, help="Charuco square length in mm")
    intrinsics_start_p.add_argument("--marker-length-mm", type=float, default=None, help="Charuco marker length in mm")
    intrinsics_start_p.add_argument("--squares-x", type=int, default=6, help="Charuco board squares in X")
    intrinsics_start_p.add_argument("--squares-y", type=int, default=8, help="Charuco board squares in Y")
    intrinsics_start_p.add_argument("--min-frames", type=int, default=25, help="Minimum frames before calibration")
    intrinsics_start_p.add_argument("--cooldown-s", type=float, default=1.5, help="Minimum capture interval in seconds")

    sub.add_parser("intrinsics_stop", help="Stop Pi-side intrinsics frame capture")
    sub.add_parser("intrinsics_clear", help="Clear Pi-side buffered intrinsics frames")

    intrinsics_get_corners_p = sub.add_parser("intrinsics_get_corners", help="Fetch buffered Pi-side intrinsics corners")
    intrinsics_get_corners_p.add_argument("--start-index", type=int, default=0, help="Starting buffered frame index")
    intrinsics_get_corners_p.add_argument("--max-frames", type=int, default=None, help="Maximum buffered frames to return")

    sub.add_parser("intrinsics_status", help="Read Pi-side intrinsics capture status")

    return parser


def _build_cli_and_run() -> None:
    parser = build_parser()
    args = parser.parse_args()

    ip = args.ip
    port = int(args.port) if args.port is not None else 8554
    camera_id = args.camera_id
    req_id = args.request_id

    try:
        if args.cmd == "ping":
            resp = ping(ip, port, camera_id=camera_id, request_id=req_id)
        elif args.cmd == "start":
            resp = start(ip, port, camera_id=camera_id, mode=args.mode, request_id=req_id)
        elif args.cmd == "stop":
            resp = stop(ip, port, camera_id=camera_id, request_id=req_id)
        elif args.cmd == "set_exposure":
            resp = set_exposure(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "set_gain":
            resp = set_gain(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "set_fps":
            resp = set_fps(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "set_focus":
            resp = set_focus(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "set_threshold":
            resp = set_threshold(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "set_blob_diameter":
            resp = set_blob_diameter(
                ip,
                port,
                camera_id=camera_id,
                min_px=args.min_px,
                max_px=args.max_px,
                request_id=req_id,
            )
        elif args.cmd == "set_circularity_min":
            resp = set_circularity_min(ip, port, camera_id=camera_id, value=args.value, request_id=req_id)
        elif args.cmd == "mask_start":
            resp = mask_start(
                ip,
                port,
                camera_id=camera_id,
                threshold=args.threshold,
                frames=args.frames,
                seconds=args.seconds,
                hit_ratio=args.hit_ratio,
                min_area=args.min_area,
                dilate=args.dilate,
                request_id=req_id,
            )
        elif args.cmd == "mask_stop":
            resp = mask_stop(ip, port, camera_id=camera_id, request_id=req_id)
        elif args.cmd == "set_preview":
            overlays: Dict[str, bool] = {}
            if args.overlay_blob is not None:
                overlays["blob"] = bool(args.overlay_blob)
            if args.overlay_mask is not None:
                overlays["mask"] = bool(args.overlay_mask)
            if args.overlay_text is not None:
                overlays["text"] = bool(args.overlay_text)
            if args.overlay_charuco is not None:
                overlays["charuco"] = bool(args.overlay_charuco)

            charuco: Dict[str, Any] = {}
            if args.charuco_dictionary is not None:
                charuco["dictionary"] = str(args.charuco_dictionary)
            if args.charuco_squares_x is not None:
                charuco["squares_x"] = int(args.charuco_squares_x)
            if args.charuco_squares_y is not None:
                charuco["squares_y"] = int(args.charuco_squares_y)
            if args.charuco_square_length_mm is not None:
                charuco["square_length_mm"] = float(args.charuco_square_length_mm)
            if args.charuco_marker_length_mm is not None:
                charuco["marker_length_mm"] = float(args.charuco_marker_length_mm)

            resp = set_preview(
                ip,
                port,
                camera_id=camera_id,
                render_enabled=args.render_enabled,
                overlays=overlays if overlays else None,
                charuco=charuco if charuco else None,
                request_id=req_id,
            )
        elif args.cmd == "intrinsics_start":
            resp = intrinsics_start(
                ip,
                port,
                camera_id=camera_id,
                square_length_mm=args.square_length_mm,
                marker_length_mm=args.marker_length_mm,
                squares_x=args.squares_x,
                squares_y=args.squares_y,
                min_frames=args.min_frames,
                cooldown_s=args.cooldown_s,
                request_id=req_id,
            )
        elif args.cmd == "intrinsics_stop":
            resp = intrinsics_stop(ip, port, camera_id=camera_id, request_id=req_id)
        elif args.cmd == "intrinsics_clear":
            resp = intrinsics_clear(ip, port, camera_id=camera_id, request_id=req_id)
        elif args.cmd == "intrinsics_get_corners":
            resp = intrinsics_get_corners(
                ip,
                port,
                camera_id=camera_id,
                start_index=args.start_index,
                max_frames=args.max_frames,
                request_id=req_id,
            )
        elif args.cmd == "intrinsics_status":
            resp = intrinsics_status(ip, port, camera_id=camera_id, request_id=req_id)
        else:
            raise SystemExit("Unknown command")

        _print_json_and_exit(resp)
    except Exception as e:
        error_resp = {"error": str(e)}
        print(json.dumps(error_resp))
        sys.exit(2)


if __name__ == "__main__":
    _build_cli_and_run()
