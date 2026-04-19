from __future__ import annotations

from pathlib import Path
import threading
from typing import Any, Dict, List

from .logger import FrameLogger


class GuiCaptureLogService:
    """Lifecycle manager for pose and wand-metric capture logs."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def on_frame_received(self, frame: Any) -> None:
        owner = self._owner
        previous = owner._receiver_frame_callback
        if callable(previous):
            previous(frame)
        logger: FrameLogger | None = None
        with owner.lock:
            if owner._capture_log_active:
                logger = owner._capture_logger
        if logger is None:
            return
        frame_dict = frame.to_dict() if hasattr(frame, "to_dict") else dict(frame)
        try:
            logger.log_frame(frame_dict)
        except Exception:
            return

    def start_capture(self, command: str, payload: Dict[str, Any], targets: List[Any]) -> Dict[str, Any]:
        owner = self._owner
        if command in ("start", "start_pose_capture"):
            log_path = owner._start_capture_log("pose_capture")
            owner._update_runtime_hints(pose_log_path=str(log_path))
            result = owner.session._broadcast(targets, "start", mode="pose_capture")
            payload_out: Dict[str, Any] = {command: result, "capture_log": {"path": str(log_path)}}
            if not self.all_acked(result):
                stop_meta = owner._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            owner._update_camera_status({command: result})
            owner.last_result = payload_out
            return payload_out

        bundle = owner._load_settings_bundle()
        committed_calibration = bundle.get("calibration", {}).get("committed", {})
        committed_extrinsics = bundle.get("extrinsics", {}).get("committed", {})
        default_duration = committed_calibration.get("wand_metric_seconds", owner.config.duration_s)
        duration_s = float(payload.get("duration_s", default_duration))
        if duration_s <= 0.0:
            raise ValueError("duration_s must be > 0")
        wand_log_raw = str(
            payload.get("wand_metric_log_path", committed_extrinsics.get("wand_metric_log_path", ""))
        ).strip()
        if wand_log_raw:
            owner.wand_metric_log_path = owner._resolve_project_path(wand_log_raw, owner._default_wand_metric_log_path())
        log_path = owner._start_capture_log("wand_metric_capture")
        owner._update_runtime_hints(wand_metric_log_path=str(log_path))
        result = owner.session._broadcast(targets, "start", mode="wand_metric_capture")
        payload_out = {
            command: result,
            "capture_log": {"path": str(log_path)},
            "duration_s": duration_s,
        }
        if not self.all_acked(result):
            stop_meta = owner._stop_capture_log()
            if stop_meta is not None:
                payload_out["capture_log"].update(stop_meta)
        else:
            owner._schedule_auto_stop([target.camera_id for target in targets], duration_s, "wand_metric_capture")
        owner._update_camera_status({command: result})
        owner.last_result = payload_out
        return payload_out

    def stop_capture(self, command: str, targets: List[Any]) -> Dict[str, Any]:
        owner = self._owner
        owner._cancel_capture_timer()
        result = owner.session._broadcast(targets, "stop")
        payload_out = {command: result}
        with owner.lock:
            active_kind = owner._active_capture_kind
        stop_meta = owner._stop_capture_log()
        if stop_meta is not None:
            payload_out["capture_log"] = stop_meta
            log_file = stop_meta.get("log_file") if isinstance(stop_meta, dict) else None
            if isinstance(log_file, str):
                if command == "stop_wand_metric_capture" or active_kind == "wand_metric_capture":
                    owner._update_runtime_hints(wand_metric_log_path=log_file)
                else:
                    owner._update_runtime_hints(pose_log_path=log_file)
        owner._update_camera_status({command: result})
        owner.last_result = payload_out
        return payload_out

    def start_capture_log(self, capture_kind: str) -> Path:
        owner = self._owner
        with owner.lock:
            if owner._capture_logger is not None and owner._capture_log_active:
                if owner._active_capture_kind != capture_kind:
                    raise ValueError(f"capture already active: {owner._active_capture_kind}")
                default_path = owner.pose_capture_log_path if capture_kind == "pose_capture" else owner.wand_metric_log_path
                return Path(owner._capture_logger.current_log_file or str(default_path))
            owner.capture_log_dir.mkdir(parents=True, exist_ok=True)
            target_path = owner.pose_capture_log_path if capture_kind == "pose_capture" else owner.wand_metric_log_path
            if target_path.exists():
                target_path.unlink(missing_ok=True)
            logger = FrameLogger(log_dir=str(owner.capture_log_dir))
            log_file = logger.start_recording(session_name=target_path.stem)
            owner._capture_logger = logger
            owner._capture_log_active = True
            owner._active_capture_kind = capture_kind
            owner._capture_completed[capture_kind] = False
            if capture_kind == "pose_capture":
                owner.pose_capture_log_path = Path(log_file)
            else:
                owner.wand_metric_log_path = Path(log_file)
            return Path(log_file)

    def stop_capture_log(self) -> Dict[str, Any] | None:
        owner = self._owner
        with owner.lock:
            logger = owner._capture_logger
            active = owner._capture_log_active
        if logger is None or not active:
            return None
        metadata = logger.stop_recording()
        with owner.lock:
            owner._capture_log_active = False
            active_kind = owner._active_capture_kind
            owner._active_capture_kind = None
            if isinstance(metadata.get("log_file"), str):
                path = Path(str(metadata["log_file"]))
                if active_kind == "pose_capture":
                    owner.pose_capture_log_path = path
                elif active_kind == "wand_metric_capture":
                    owner.wand_metric_log_path = path
            if active_kind == "pose_capture":
                owner._capture_completed["pose_capture"] = (
                    owner.pose_capture_log_path.exists() and owner.pose_capture_log_path.is_file()
                )
            elif active_kind == "wand_metric_capture":
                owner._capture_completed["wand_metric_capture"] = (
                    owner.wand_metric_log_path.exists() and owner.wand_metric_log_path.is_file()
                )
        return metadata

    def cancel_capture_timer(self) -> None:
        owner = self._owner
        with owner.lock:
            timer = owner._capture_auto_stop_timer
            owner._capture_auto_stop_timer = None
        if timer is not None:
            timer.cancel()

    def schedule_auto_stop(self, camera_ids: List[str], duration_s: float, capture_kind: str) -> None:
        owner = self._owner

        def _auto_stop() -> None:
            try:
                targets = owner.session.discover_targets(camera_ids or None)
                result = owner.session._broadcast(targets, "stop")
                payload_out: Dict[str, Any] = {f"stop_{capture_kind}": result}
                stop_meta = self.stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"] = stop_meta
                    log_file = stop_meta.get("log_file") if isinstance(stop_meta, dict) else None
                    if isinstance(log_file, str):
                        if capture_kind == "pose_capture":
                            owner._update_runtime_hints(pose_log_path=log_file)
                        elif capture_kind == "wand_metric_capture":
                            owner._update_runtime_hints(wand_metric_log_path=log_file)
                owner._update_camera_status({"stop": result})
                owner.last_result = payload_out
            finally:
                with owner.lock:
                    owner._capture_auto_stop_timer = None

        timer = threading.Timer(duration_s, _auto_stop)
        timer.daemon = True
        with owner.lock:
            if owner._capture_auto_stop_timer is not None:
                owner._capture_auto_stop_timer.cancel()
            owner._capture_auto_stop_timer = timer
        timer.start()

    @staticmethod
    def all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return all(bool(resp.get("ack")) for resp in responses.values())
