import json
import socket
import uuid
import argparse
import sys
from typing import Optional, Dict, Any


def _recv_line(sock: socket.socket) -> str:
    """Read bytes from socket until a newline, return the decoded line (without the newline)."""
    buf = b""
    while True:
        chunk = sock.recv(4096)
        if not chunk:
            raise ConnectionError("Socket closed by peer before newline")
        buf += chunk
        if b"\n" in buf:
            line, _, rest = buf.partition(b"\n")
    
            return line.decode("utf-8")


def send_request(ip: str, port: int, request_dict: Dict[str, Any], timeout: float = 5.0) -> Dict[str, Any]:
    """Send a single NDJSON request line and read a single NDJSON response line.
    Uses stdlib only. Ensures a request_id and camera_id are present when provided.
    Returns the parsed JSON response as a dict.
    """
    if not isinstance(request_dict, dict):
        raise TypeError("request_dict must be a dict")

    req = dict(request_dict)
    if not req.get("request_id"):
        req["request_id"] = str(uuid.uuid4())
    if not req.get("camera_id"):
        raise ValueError("request must include camera_id")

    payload = (json.dumps(req, separators=(",", ":")).encode("utf-8") + b"\n")

    with socket.create_connection((ip, port), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall(payload)
        resp_line = _recv_line(sock)
    try:
        return json.loads(resp_line)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON from server: {resp_line!r}") from e


def ping(ip: str, port: int, camera_id: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "ping", "camera_id": camera_id}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def start(ip: str, port: int, camera_id: str, mode: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "start", "camera_id": camera_id, "params": {"mode": mode}}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def stop(ip: str, port: int, camera_id: str, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "stop", "camera_id": camera_id}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def set_exposure(ip: str, port: int, camera_id: str, value: int, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "set_exposure", "camera_id": camera_id, "params": {"value": int(value)}}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def set_gain(ip: str, port: int, camera_id: str, value: float, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "set_gain", "camera_id": camera_id, "params": {"value": float(value)}}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def set_fps(ip: str, port: int, camera_id: str, value: int, request_id: Optional[str] = None, timeout: float = 5.0) -> Dict[str, Any]:
    req: Dict[str, Any] = {"cmd": "set_fps", "camera_id": camera_id, "params": {"value": int(value)}}
    if request_id:
        req["request_id"] = request_id
    return send_request(ip, port, req, timeout=timeout)


def _print_json_and_exit(resp: Dict[str, Any]) -> None:
    print(json.dumps(resp))
    if isinstance(resp, dict) and resp.get("ack") is True:
        sys.exit(0)
    else:
        sys.exit(1)


def _build_cli_and_run() -> None:
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
        else:
            raise SystemExit("Unknown command")

        _print_json_and_exit(resp)
    except Exception as e:
        error_resp = {"error": str(e)}
        print(json.dumps(error_resp))
        sys.exit(2)


if __name__ == "__main__":
    _build_cli_and_run()
