from __future__ import annotations

import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, Optional


class GuiIntrinsicsService:
    """Intrinsics session orchestration for the host GUI backend."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def start_intrinsics_capture(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        owner = self._owner
        owner._assert_calibration_unlocked("intrinsics capture")
        bundle = owner._load_settings_bundle()
        if payload:
            intrinsics_patch = {
                key: payload[key]
                for key in (
                    "camera_id",
                    "mjpeg_url",
                    "square_length_mm",
                    "marker_length_mm",
                    "squares_x",
                    "squares_y",
                    "min_frames",
                    "cooldown_s",
                )
                if key in payload
            }
            if intrinsics_patch:
                bundle = owner._apply_draft_patch(bundle, {"intrinsics": intrinsics_patch})

        validation = bundle.get("validation", {}).get("intrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"intrinsics settings invalid: {validation}")

        committed = bundle.get("intrinsics", {}).get("committed", {})
        if not isinstance(committed, dict):
            committed = owner._default_intrinsics_payload()
        camera_id = str(committed.get("camera_id", "pi-cam-01")).strip()
        self._assert_capability(camera_id)

        if owner._intrinsics_host_session is not None:
            try:
                owner._intrinsics_host_session.stop()
            except Exception:
                pass

        owner._intrinsics_host_session = self._build_intrinsics_host_session(committed)
        owner._intrinsics_host_session.start()
        owner._persist_intrinsics_settings(committed)
        return {"ok": True, "camera_id": camera_id, "status": owner._intrinsics_host_session.get_status()}

    def stop_intrinsics_capture(self) -> Dict[str, Any]:
        owner = self._owner
        if owner._intrinsics_host_session is not None:
            owner._intrinsics_host_session.stop()
            return {"ok": True, "status": self.get_intrinsics_status()}
        return {"ok": True, "status": self._idle_status(None)}

    def clear_intrinsics_frames(self) -> Dict[str, Any]:
        owner = self._owner
        owner._assert_calibration_unlocked("intrinsics clear")
        if owner._intrinsics_host_session is None:
            raise ValueError("No intrinsics capture session active")
        owner._intrinsics_host_session.clear()
        return {"ok": True, "status": self.get_intrinsics_status()}

    def trigger_intrinsics_calibration(self) -> Dict[str, Any]:
        self._owner._assert_calibration_unlocked("intrinsics calibration")
        session = self._ensure_intrinsics_host_session_from_settings()
        session.trigger_calibration()
        return {"ok": True, "status": session.get_status()}

    def get_intrinsics_status(self) -> Dict[str, Any]:
        owner = self._owner
        targets = owner.session.discover_targets(None)
        camera_list = [{"camera_id": t.camera_id, "ip": t.ip} for t in targets]
        if owner._intrinsics_host_session is None:
            try:
                self._ensure_intrinsics_host_session_from_settings()
            except ValueError:
                status = self._idle_status(None)
                status["cameras"] = camera_list
                return status
        assert owner._intrinsics_host_session is not None
        status = owner._intrinsics_host_session.get_status()
        camera_id = str(status.get("camera_id", "")).strip()
        if camera_id:
            try:
                remote_status = self._send_command(camera_id, "intrinsics_status")
            except Exception as exc:
                owner._intrinsics_host_session.record_remote_status_error(str(exc))
            else:
                owner._intrinsics_host_session.apply_remote_status(remote_status)
            status = owner._intrinsics_host_session.get_status()
        status["cameras"] = camera_list
        return status

    def get_intrinsics_jpeg(self) -> Optional[bytes]:
        owner = self._owner
        if owner._intrinsics_host_session is not None:
            return owner._intrinsics_host_session.get_latest_jpeg()
        bundle = owner._load_settings_bundle()
        intrinsics = bundle.get("intrinsics", {})
        draft = intrinsics.get("draft", {}) if isinstance(intrinsics, dict) else {}
        committed = intrinsics.get("committed", {}) if isinstance(intrinsics, dict) else {}
        mjpeg_url = ""
        if isinstance(draft, dict):
            mjpeg_url = str(draft.get("mjpeg_url", "")).strip()
        if not mjpeg_url and isinstance(committed, dict):
            mjpeg_url = str(committed.get("mjpeg_url", "")).strip()
        if not mjpeg_url:
            return None
        return self._fetch_single_mjpeg_frame(mjpeg_url)

    def _build_intrinsics_host_session(self, committed: Dict[str, Any]) -> Any:
        owner = self._owner
        camera_id = str(committed.get("camera_id", "pi-cam-01")).strip()
        if not camera_id:
            raise ValueError("intrinsics camera_id is required")
        square_length_mm = float(committed.get("square_length_mm", 60.0))
        marker_length_mm_raw = committed.get("marker_length_mm")
        marker_length_mm = (
            float(marker_length_mm_raw)
            if marker_length_mm_raw is not None
            else square_length_mm * 0.75
        )
        config = owner._build_intrinsics_host_session_config(
            camera_id=camera_id,
            mjpeg_url=str(committed.get("mjpeg_url", "")).strip(),
            square_length_mm=square_length_mm,
            marker_length_mm=marker_length_mm,
            squares_x=int(committed.get("squares_x", 6)),
            squares_y=int(committed.get("squares_y", 8)),
            min_frames=int(committed.get("min_frames", 25)),
            cooldown_s=float(committed.get("cooldown_s", 1.5)),
        )
        broadcast_fn = lambda fn, **kw: self._send_command(camera_id, fn, **kw)  # noqa: E731
        return owner._create_intrinsics_host_session(config, broadcast_fn)

    def _ensure_intrinsics_host_session_from_settings(self) -> Any:
        owner = self._owner
        if owner._intrinsics_host_session is not None:
            return owner._intrinsics_host_session
        bundle = owner._load_settings_bundle()
        validation = bundle.get("validation", {}).get("intrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"intrinsics settings invalid: {validation}")
        intrinsics = bundle.get("intrinsics", {})
        committed = intrinsics.get("committed", {}) if isinstance(intrinsics, dict) else {}
        if not isinstance(committed, dict):
            committed = owner._default_intrinsics_payload()
        session = self._build_intrinsics_host_session(committed)
        owner._intrinsics_host_session = session
        return session

    def _idle_status(self, camera_id: str | None) -> Dict[str, Any]:
        return {
            "phase": "idle",
            "camera_id": camera_id,
            "frames_captured": 0,
            "frames_needed": 25,
            "frames_target": 50,
            "frames_rejected_cooldown": 0,
            "frames_rejected_spatial": 0,
            "frames_rejected_detection": 0,
            "grid_coverage": [[0, 0, 0], [0, 0, 0], [0, 0, 0]],
            "last_error": None,
            "calibration_result": None,
            "output_path": None,
        }

    def _find_target(self, camera_id: str) -> Any | None:
        owner = self._owner
        for target in owner.session.discover_targets(None):
            if str(target.camera_id).strip() == camera_id:
                return target
        return None

    def _assert_capability(self, camera_id: str) -> None:
        owner = self._owner
        target = self._find_target(camera_id)
        if target is None:
            raise ValueError(f"intrinsics camera not discovered: {camera_id}")
        ping_resp = owner.session._broadcast([target], "ping").get(camera_id, {})
        if not bool(ping_resp.get("ack")):
            raise ValueError(ping_resp.get("error") or ping_resp.get("error_message") or "ping failed")
        result = ping_resp.get("result", {})
        supported_obj = result.get("supported_commands") if isinstance(result, dict) else None
        supported = set(supported_obj) if isinstance(supported_obj, list) else set()
        required = {
            "intrinsics_start",
            "intrinsics_stop",
            "intrinsics_clear",
            "intrinsics_get_corners",
            "intrinsics_status",
        }
        missing = sorted(required - supported)
        if missing:
            raise ValueError(
                "intrinsics capability missing on Pi. "
                f"camera_id={camera_id} missing={','.join(missing)}"
            )

    def _send_command(self, camera_id: str, fn_name: str, **kwargs: Any) -> Dict[str, Any]:
        owner = self._owner
        target = self._find_target(camera_id)
        if target is None:
            raise ValueError(f"intrinsics camera not discovered: {camera_id}")
        response = owner.session._broadcast([target], fn_name, **kwargs).get(camera_id, {})
        if not bool(response.get("ack")):
            raise ValueError(response.get("error") or response.get("error_message") or f"{fn_name} failed")
        result = response.get("result", {})
        return result if isinstance(result, dict) else {}

    def _fetch_single_mjpeg_frame(self, url: str) -> Optional[bytes]:
        try:
            with urllib.request.urlopen(url, timeout=1.5) as response:
                content_type = str(response.headers.get("Content-Type", ""))
                boundary = b""
                for part in content_type.split(";"):
                    token = part.strip()
                    if token.startswith("boundary="):
                        boundary = token.split("=", 1)[1].encode("utf-8")
                        break

                content_length = -1
                while True:
                    line = response.readline()
                    if not line:
                        return None
                    stripped = line.strip()
                    if stripped.lower().startswith(b"content-length:"):
                        try:
                            content_length = int(stripped.split(b":", 1)[1].strip())
                        except Exception:
                            content_length = -1
                    if stripped == b"":
                        break

                if content_length > 0:
                    data = response.read(content_length)
                    _ = response.read(2)
                    return bytes(data) if data else None

                data = bytearray()
                deadline = time.monotonic() + 1.2
                while time.monotonic() < deadline:
                    line = response.readline()
                    if not line:
                        break
                    if boundary and boundary in line:
                        break
                    data.extend(line)
                frame = bytes(data).strip()
                return frame if frame else None
        except Exception:
            return None
