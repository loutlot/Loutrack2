from __future__ import annotations

import time
import uuid
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

from . import control as control_module
from .scheduled_start import epoch_us, scheduled_start_kwargs
from calibration.targets.wand import (
    WAND_MARKER_DIAMETER_MM,
    WAND_NAME,
    WAND_OUTER_LONG_MM,
    WAND_OUTER_SHORT_MM,
    WAND_POINTS_MM,
)

FIXED_FOCUS = 0.325
FIXED_CIRCULARITY_MIN = 0.2
FIXED_FPS = 118
MAX_EXPOSURE_US = 8000
FIXED_PAIR_WINDOW_US = 4166
DEFAULT_THRESHOLD = 150
DEFAULT_MASK_THRESHOLD = 120


@dataclass(frozen=True)
class CameraTarget:
    camera_id: str
    ip: str
    control_port: int = 8554


@dataclass
class CalibrationSessionConfig:
    exposure_us: int
    gain: float
    fps: int = FIXED_FPS
    focus: float = FIXED_FOCUS
    threshold: int = DEFAULT_THRESHOLD
    blob_min_diameter_px: Optional[float] = None
    blob_max_diameter_px: Optional[float] = None
    circularity_min: float = FIXED_CIRCULARITY_MIN
    duration_s: float = 60.0
    camera_ids: Optional[List[str]] = None
    mask_params: Optional[Dict[str, Any]] = None
    mask_retry: int = 1
    output_dir: Optional[Path] = None
    capture_kind: str = "pose_capture"


class CalibrationSession:
    def __init__(
        self,
        inventory_path: Optional[Path] = None,
        receiver: Optional[Any] = None,
        control: Any = control_module,
        timeout_s: float = 2.0,
        mask_timeout_s: float = 30.0,
    ) -> None:
        self.inventory_path = inventory_path or (
            Path(__file__).resolve().parents[1] / "deploy" / "hosts.ini"
        )
        self.receiver = receiver
        self.control = control
        self.timeout_s = timeout_s
        self.mask_timeout_s = mask_timeout_s
        self.default_output_dir = Path("logs") / "wand_sessions"

    def load_inventory(self) -> Dict[str, CameraTarget]:
        targets: Dict[str, CameraTarget] = {}
        if not self.inventory_path.exists():
            return targets

        text = self.inventory_path.read_text(encoding="utf-8")
        for raw_line in text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 3:
                continue
            _, ip, camera_id = parts[0], parts[1], parts[2]
            targets[camera_id] = CameraTarget(camera_id=camera_id, ip=ip)
        return targets

    def discover_targets(self, camera_ids: Optional[Iterable[str]] = None) -> List[CameraTarget]:
        merged = self.load_inventory()

        if self.receiver is not None:
            discovered = self.receiver.get_camera_addresses()
            for camera_id, addr in discovered.items():
                if not addr:
                    continue
                ip = addr[0]
                merged[camera_id] = CameraTarget(camera_id=camera_id, ip=ip)

        if camera_ids is not None:
            requested = set(camera_ids)
            merged = {cid: target for cid, target in merged.items() if cid in requested}

        return sorted(merged.values(), key=lambda t: t.camera_id)

    def prepare_targets(self, camera_ids: Optional[Iterable[str]] = None) -> List[CameraTarget]:
        discovered = self.discover_targets(camera_ids=camera_ids)
        healthy: List[CameraTarget] = []
        for target in discovered:
            if self._is_healthy(target):
                healthy.append(target)
        return healthy

    def _is_healthy(self, target: CameraTarget) -> bool:
        try:
            resp = self.control.ping(
                target.ip,
                target.control_port,
                camera_id=target.camera_id,
                timeout=self.timeout_s,
            )
        except Exception:
            return False
        return bool(resp.get("ack"))

    def _broadcast(self, targets: Iterable[CameraTarget], fn_name: str, **kwargs: Any) -> Dict[str, Dict[str, Any]]:
        fn = getattr(self.control, fn_name)
        timeout = self.mask_timeout_s if fn_name == "mask_start" else self.timeout_s
        if fn_name == "start" and kwargs.get("start_at_us") is not None:
            lead_s = max(0.0, (int(kwargs["start_at_us"]) - epoch_us()) / 1_000_000.0)
            timeout = max(timeout, lead_s + 2.0)
        target_list = list(targets)
        results: Dict[str, Dict[str, Any]] = {}

        def send_one(target: CameraTarget) -> tuple[str, Dict[str, Any]]:
            try:
                resp = fn(
                    target.ip,
                    target.control_port,
                    camera_id=target.camera_id,
                    timeout=timeout,
                    **kwargs,
                )
            except Exception as exc:
                resp = {"ack": False, "error": str(exc)}
            return target.camera_id, resp

        if fn_name == "start" and kwargs.get("start_at_us") is not None and len(target_list) > 1:
            with ThreadPoolExecutor(max_workers=len(target_list)) as executor:
                futures = [executor.submit(send_one, target) for target in target_list]
                for future in as_completed(futures):
                    camera_id, resp = future.result()
                    results[camera_id] = resp
            return results

        for target in target_list:
            camera_id, resp = send_one(target)
            results[camera_id] = resp
        return results

    def run_session(self, config: CalibrationSessionConfig) -> Dict[str, Any]:
        targets = self.prepare_targets(camera_ids=config.camera_ids)
        if not targets:
            raise RuntimeError("No healthy cameras found")

        fixed_focus = FIXED_FOCUS
        fixed_circularity_min = FIXED_CIRCULARITY_MIN
        session_id = str(uuid.uuid4())
        started_at = datetime.now(timezone.utc).isoformat()
        ack_history: List[Dict[str, Any]] = []

        def record(step: str, responses: Dict[str, Dict[str, Any]]) -> None:
            ack_history.append({"step": step, "responses": responses})

        exp_resp = self._broadcast(targets, "set_exposure", value=config.exposure_us)
        record("set_exposure", exp_resp)

        gain_resp = self._broadcast(targets, "set_gain", value=config.gain)
        record("set_gain", gain_resp)

        threshold_resp = self._broadcast(targets, "set_threshold", value=config.threshold)
        record("set_threshold", threshold_resp)

        blob_diameter_resp = self._broadcast(
            targets,
            "set_blob_diameter",
            min_px=config.blob_min_diameter_px,
            max_px=config.blob_max_diameter_px,
        )
        record("set_blob_diameter", blob_diameter_resp)

        mask_kwargs = dict(config.mask_params or {})
        mask_resp = self._broadcast(targets, "mask_start", **mask_kwargs)
        record("mask_start", mask_resp)
        if not self._all_acked(mask_resp):
            for _ in range(max(0, config.mask_retry)):
                mask_resp = self._broadcast(targets, "mask_start", **mask_kwargs)
                record("mask_start_retry", mask_resp)
                if self._all_acked(mask_resp):
                    break
            if not self._all_acked(mask_resp):
                raise RuntimeError("mask_start failed on one or more cameras")

        start_mode = config.capture_kind if config.capture_kind else "pose_capture"
        start_kwargs = scheduled_start_kwargs(start_mode)
        start_resp = self._broadcast(targets, "start", **start_kwargs)
        record("start", start_resp)
        if not self._all_acked(start_resp):
            stop_resp = self._broadcast(targets, "stop")
            record("stop_after_start_failure", stop_resp)
            raise RuntimeError("start failed on one or more cameras")

        time.sleep(config.duration_s)

        stop_resp = self._broadcast(targets, "stop")
        record("stop", stop_resp)

        result = {
            "session_id": session_id,
            "started_at": started_at,
            "ended_at": datetime.now(timezone.utc).isoformat(),
            "config": {
                "exposure_us": config.exposure_us,
                "gain": config.gain,
                "focus": fixed_focus,
                "threshold": config.threshold,
                "blob_min_diameter_px": config.blob_min_diameter_px,
                "blob_max_diameter_px": config.blob_max_diameter_px,
                "circularity_min": fixed_circularity_min,
                "duration_s": config.duration_s,
                "mask_params": dict(config.mask_params or {}),
                "mask_retry": config.mask_retry,
                "capture_kind": start_mode,
                "scheduled_start_at_us": start_kwargs["start_at_us"],
            },
            "wand": {
                "name": WAND_NAME,
                "marker_diameter_mm": WAND_MARKER_DIAMETER_MM,
                "points_mm": WAND_POINTS_MM,
            },
            "targets": [
                {
                    "camera_id": t.camera_id,
                    "ip": t.ip,
                    "control_port": t.control_port,
                }
                for t in targets
            ],
            "ack_history": ack_history,
        }
        result["metadata_path"] = str(self._persist_session_metadata(result, config))
        return result

    @staticmethod
    def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return all(bool(resp.get("ack")) for resp in responses.values())

    def _persist_session_metadata(self, result: Dict[str, Any], config: CalibrationSessionConfig) -> Path:
        output_dir = config.output_dir or self.default_output_dir
        output_dir.mkdir(parents=True, exist_ok=True)
        path = output_dir / f"{result['session_id']}.json"
        path.write_text(self._json_dumps(result), encoding="utf-8")
        return path

    @staticmethod
    def _json_dumps(value: Dict[str, Any]) -> str:
        import json

        return json.dumps(value, indent=2, ensure_ascii=False)
