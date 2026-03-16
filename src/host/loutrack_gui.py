#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
import threading
import importlib.util
import urllib.request
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, List, Optional
from collections import Counter

PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_SRC_ROOT = Path(__file__).resolve().parents[1]

if __package__ in (None, ""):
    if str(MODULE_SRC_ROOT) not in sys.path:
        sys.path.insert(0, str(MODULE_SRC_ROOT))
    from host.receiver import UDPReceiver
    from host.logger import FrameLogger
    from host.tracking_runtime import TrackingRuntime
    from host.wand_session import CalibrationSession, CalibrationSessionConfig
    from host.extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry
else:
    from .receiver import UDPReceiver
    from .logger import FrameLogger
    from .tracking_runtime import TrackingRuntime
    from .wand_session import CalibrationSession, CalibrationSessionConfig
    from .extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry


DEFAULT_SETTINGS_PATH = Path("logs") / "loutrack_gui_settings.json"
OLD_SETTINGS_PATH = Path("logs") / "wand_gui_settings.json"
DEFAULT_POSE_LOG_PATH = Path("logs") / "extrinsics_pose_capture.jsonl"
DEFAULT_WAND_METRIC_LOG_PATH = Path("logs") / "extrinsics_wand_metric.jsonl"
DEFAULT_EXTRINSICS_OUTPUT_PATH = Path("calibration") / "extrinsics_pose_v2.json"
DEFAULT_CAPTURE_LOG_DIR = Path("logs")
DEFAULT_WAND_METRIC_DURATION_S = 3.0
STATIC_DIR_CANDIDATES = (
    PROJECT_ROOT / "static",
    MODULE_SRC_ROOT / "static",
)


def _load_html_page() -> str:
    """Load index.html from static/ directory; fall back to an error page."""
    for static_dir in STATIC_DIR_CANDIDATES:
        candidate = static_dir / "index.html"
        if candidate.is_file():
            return candidate.read_text(encoding="utf-8")
    # Emergency fallback — should not normally be reached when static/ is present.
    return (
        "<!doctype html><html><body>"
        "<h1>Loutrack GUI</h1>"
        "<p>static/index.html not found. Run from the project root.</p>"
        "</body></html>"
    )


HTML_PAGE = _load_html_page()


class LoutrackGuiState:
    def __init__(
        self,
        session: CalibrationSession,
        receiver: UDPReceiver,
        settings_path: Path | None = None,
        tracking_runtime: TrackingRuntime | None = None,
    ) -> None:
        self.session = session
        self.receiver = receiver
        self.lock = threading.Lock()
        self._uses_default_settings_path = settings_path is None
        self.settings_path = settings_path or DEFAULT_SETTINGS_PATH
        self._migrate_settings_once()
        self.selected_camera_ids: List[str] = []
        self.config = self._load_initial_config()
        self.camera_status: Dict[str, Dict[str, Any]] = {}
        self.last_result: Dict[str, Any] = {"status": "idle"}
        self.capture_log_dir: Path = DEFAULT_CAPTURE_LOG_DIR
        self.pose_capture_log_path: Path = DEFAULT_POSE_LOG_PATH
        self.wand_metric_log_path: Path = DEFAULT_WAND_METRIC_LOG_PATH
        self._capture_logger: FrameLogger | None = None
        self._capture_log_active: bool = False
        self._capture_completed: Dict[str, bool] = {"pose_capture": False, "wand_metric_capture": False}
        self._active_capture_kind: str | None = None
        self._capture_auto_stop_timer: threading.Timer | None = None
        self._receiver_frame_callback = getattr(receiver, "_frame_callback", None)
        if hasattr(self.receiver, "set_frame_callback"):
            self.receiver.set_frame_callback(self._on_frame_received)
        self._generate_extrinsics_registry: ExtrinsicsMethodRegistry = build_default_extrinsics_registry(
            blob_pose_solver=_load_extrinsics_solver(),
        )
        # Kept for backward-compatibility with existing tests and callers.
        self._generate_extrinsics_solver = self._generate_extrinsics_registry.get("blob_pose_v2").solve
        self.tracking_runtime = tracking_runtime or TrackingRuntime()
        self.latest_extrinsics_path: Path | None = None
        self.latest_extrinsics_quality: Dict[str, Any] | None = None
        self._restore_latest_extrinsics(DEFAULT_EXTRINSICS_OUTPUT_PATH)
        self._intrinsics_active_camera_id: str | None = None
        self._intrinsics_last_saved_signature: tuple[str, float] | None = None
        settings_bundle = self._load_settings_bundle()
        ui_payload = settings_bundle.get("ui", {})
        if isinstance(ui_payload, dict):
            selected_ids = ui_payload.get("selected_camera_ids", [])
            if isinstance(selected_ids, list):
                self.selected_camera_ids = [str(item).strip() for item in selected_ids if str(item).strip()]

    @staticmethod
    def _default_settings_payload() -> Dict[str, Any]:
        calibration_defaults = LoutrackGuiState._default_calibration_payload()
        intrinsics_defaults = LoutrackGuiState._default_intrinsics_payload()
        extrinsics_defaults = LoutrackGuiState._default_extrinsics_payload()
        return {
            "meta": {"version": 2, "updated_at": int(time.time())},
            "calibration": {
                "draft": dict(calibration_defaults),
                "committed": dict(calibration_defaults),
            },
            "intrinsics": {
                "draft": dict(intrinsics_defaults),
                "committed": dict(intrinsics_defaults),
            },
            "extrinsics": {
                "draft": dict(extrinsics_defaults),
                "committed": dict(extrinsics_defaults),
                "locks": {
                    "pose_log_path_manual": False,
                    "wand_metric_log_path_manual": False,
                },
            },
            "ui": {
                "active_page": "calibration",
                "active_view": "cameras",
                "selected_camera_ids": [],
            },
            "runtime_hints": {
                "pose_log_path": str(DEFAULT_POSE_LOG_PATH),
                "wand_metric_log_path": str(DEFAULT_WAND_METRIC_LOG_PATH),
            },
        }

    @staticmethod
    def _default_calibration_payload() -> Dict[str, Any]:
        return {
            "exposure_us": 12000,
            "gain": 8.0,
            "fps": 56,
            "focus": 5.215,
            "threshold": 200,
            "blob_min_diameter_px": None,
            "blob_max_diameter_px": None,
            "circularity_min": 0.0,
            "mask_threshold": 200,
            "mask_seconds": 0.5,
            "wand_metric_seconds": DEFAULT_WAND_METRIC_DURATION_S,
        }

    @staticmethod
    def _default_intrinsics_payload() -> Dict[str, Any]:
        return {
            "camera_id": "pi-cam-01",
            "mjpeg_url": "",
            "square_length_mm": 30.0,
            "marker_length_mm": None,
            "squares_x": 6,
            "squares_y": 8,
            "min_frames": 25,
            "cooldown_s": 1.5,
        }

    @staticmethod
    def _default_extrinsics_payload() -> Dict[str, Any]:
        return {
            "intrinsics_path": "calibration",
            "pose_log_path": str(DEFAULT_POSE_LOG_PATH),
            "wand_metric_log_path": str(DEFAULT_WAND_METRIC_LOG_PATH),
            "output_path": str(DEFAULT_EXTRINSICS_OUTPUT_PATH),
            "pair_window_us": 2000,
            "wand_pair_window_us": 8000,
            "min_pairs": 8,
        }

    def _migrate_settings_once(self) -> None:
        if not self._uses_default_settings_path:
            return
        old_path = OLD_SETTINGS_PATH
        if not old_path.exists() or old_path == self.settings_path:
            return
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        if self.settings_path.exists():
            old_path.unlink(missing_ok=True)
            return
        try:
            payload = json.loads(old_path.read_text(encoding="utf-8"))
            if not isinstance(payload, dict):
                raise ValueError("legacy settings is not an object")
            self.settings_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            old_path.unlink(missing_ok=True)
            return
        except Exception:
            backup_path = old_path.with_suffix(old_path.suffix + ".invalid.bak")
            try:
                shutil.move(str(old_path), str(backup_path))
            except Exception:
                old_path.unlink(missing_ok=True)
            self.settings_path.write_text(
                json.dumps(self._default_settings_payload(), ensure_ascii=False, indent=2),
                encoding="utf-8",
            )

    def _default_config(self) -> CalibrationSessionConfig:
        return CalibrationSessionConfig(exposure_us=12000, gain=8.0, fps=56, duration_s=DEFAULT_WAND_METRIC_DURATION_S)

    def _read_settings_payload(self) -> Dict[str, Any]:
        try:
            payload = json.loads(self.settings_path.read_text(encoding="utf-8"))
        except FileNotFoundError:
            return {}
        except Exception:
            return {}
        if isinstance(payload, dict):
            return payload
        return {}

    def _write_settings_payload(self, payload: Dict[str, Any]) -> None:
        payload.setdefault("meta", {})
        if isinstance(payload["meta"], dict):
            payload["meta"]["version"] = 2
            payload["meta"]["updated_at"] = int(time.time())
        self.settings_path.parent.mkdir(parents=True, exist_ok=True)
        self.settings_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    @staticmethod
    def _to_int(value: Any) -> int:
        if isinstance(value, bool):
            raise ValueError("boolean is not an integer")
        if isinstance(value, str):
            text = value.strip()
            if text == "":
                raise ValueError("empty string")
            return int(float(text)) if "." in text else int(text)
        return int(value)

    @staticmethod
    def _to_float(value: Any) -> float:
        if isinstance(value, bool):
            raise ValueError("boolean is not a number")
        if isinstance(value, str):
            text = value.strip()
            if text == "":
                raise ValueError("empty string")
            return float(text)
        return float(value)

    @staticmethod
    def _to_nullable_float(value: Any) -> float | None:
        if value is None:
            return None
        if isinstance(value, str) and value.strip() == "":
            return None
        return LoutrackGuiState._to_float(value)

    @staticmethod
    def _to_string(value: Any) -> str:
        if value is None:
            return ""
        return str(value).strip()

    def _normalize_settings_payload(self, raw_payload: Dict[str, Any]) -> Dict[str, Any]:
        defaults = self._default_settings_payload()
        if not isinstance(raw_payload, dict):
            return defaults

        has_new_shape = all(
            key in raw_payload for key in ("calibration", "intrinsics", "extrinsics", "ui", "runtime_hints")
        )
        if has_new_shape:
            normalized = defaults

            calibration_src = raw_payload.get("calibration")
            if isinstance(calibration_src, dict):
                for key in normalized["calibration"]["draft"]:
                    if key in calibration_src.get("draft", {}):
                        normalized["calibration"]["draft"][key] = calibration_src["draft"][key]
                    if key in calibration_src.get("committed", {}):
                        normalized["calibration"]["committed"][key] = calibration_src["committed"][key]

            intrinsics_src = raw_payload.get("intrinsics")
            if isinstance(intrinsics_src, dict):
                for key in normalized["intrinsics"]["draft"]:
                    if key in intrinsics_src.get("draft", {}):
                        normalized["intrinsics"]["draft"][key] = intrinsics_src["draft"][key]
                    if key in intrinsics_src.get("committed", {}):
                        normalized["intrinsics"]["committed"][key] = intrinsics_src["committed"][key]

            extrinsics_src = raw_payload.get("extrinsics")
            if isinstance(extrinsics_src, dict):
                for key in normalized["extrinsics"]["draft"]:
                    if key in extrinsics_src.get("draft", {}):
                        normalized["extrinsics"]["draft"][key] = extrinsics_src["draft"][key]
                    if key in extrinsics_src.get("committed", {}):
                        normalized["extrinsics"]["committed"][key] = extrinsics_src["committed"][key]
                locks = extrinsics_src.get("locks", {})
                if isinstance(locks, dict):
                    normalized["extrinsics"]["locks"]["pose_log_path_manual"] = bool(
                        locks.get("pose_log_path_manual", False)
                    )
                    normalized["extrinsics"]["locks"]["wand_metric_log_path_manual"] = bool(
                        locks.get("wand_metric_log_path_manual", False)
                    )

            ui_src = raw_payload.get("ui")
            if isinstance(ui_src, dict):
                normalized["ui"]["active_page"] = str(
                    ui_src.get("active_page", normalized["ui"]["active_page"])
                )
                normalized["ui"]["active_view"] = str(
                    ui_src.get("active_view", normalized["ui"]["active_view"])
                )
                selected_ids = ui_src.get("selected_camera_ids", [])
                if isinstance(selected_ids, list):
                    normalized["ui"]["selected_camera_ids"] = [
                        str(item).strip() for item in selected_ids if str(item).strip()
                    ]

            hints_src = raw_payload.get("runtime_hints")
            if isinstance(hints_src, dict):
                normalized["runtime_hints"]["pose_log_path"] = str(
                    hints_src.get("pose_log_path", normalized["runtime_hints"]["pose_log_path"])
                )
                normalized["runtime_hints"]["wand_metric_log_path"] = str(
                    hints_src.get("wand_metric_log_path", normalized["runtime_hints"]["wand_metric_log_path"])
                )

            meta_src = raw_payload.get("meta")
            if isinstance(meta_src, dict):
                normalized["meta"]["updated_at"] = int(meta_src.get("updated_at", normalized["meta"]["updated_at"]))
            return normalized

        # Legacy shape migration.
        normalized = defaults
        calibration_keys = set(normalized["calibration"]["draft"].keys())
        for key in calibration_keys:
            if key in raw_payload:
                normalized["calibration"]["draft"][key] = raw_payload[key]
                normalized["calibration"]["committed"][key] = raw_payload[key]

        intrinsics_src = raw_payload.get("intrinsics", {})
        if isinstance(intrinsics_src, dict):
            for key in normalized["intrinsics"]["draft"]:
                if key in intrinsics_src:
                    normalized["intrinsics"]["draft"][key] = intrinsics_src[key]
                    normalized["intrinsics"]["committed"][key] = intrinsics_src[key]

        if "selected_camera_ids" in raw_payload and isinstance(raw_payload["selected_camera_ids"], list):
            normalized["ui"]["selected_camera_ids"] = [
                str(item).strip() for item in raw_payload["selected_camera_ids"] if str(item).strip()
            ]
        if "active_page" in raw_payload:
            normalized["ui"]["active_page"] = str(raw_payload["active_page"])
        if "active_view" in raw_payload:
            normalized["ui"]["active_view"] = str(raw_payload["active_view"])
        return normalized

    def _load_settings_bundle(self) -> Dict[str, Any]:
        raw_payload = self._read_settings_payload()
        bundle = self._normalize_settings_payload(raw_payload)
        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        try:
            self._write_settings_payload(
                {
                    "meta": bundle.get("meta", {}),
                    "calibration": bundle.get("calibration", {}),
                    "intrinsics": bundle.get("intrinsics", {}),
                    "extrinsics": bundle.get("extrinsics", {}),
                    "ui": bundle.get("ui", {}),
                    "runtime_hints": bundle.get("runtime_hints", {}),
                }
            )
        except Exception:
            pass
        return bundle

    def _save_settings_bundle(self, bundle: Dict[str, Any]) -> None:
        payload = {
            "meta": bundle.get("meta", {}),
            "calibration": bundle.get("calibration", {}),
            "intrinsics": bundle.get("intrinsics", {}),
            "extrinsics": bundle.get("extrinsics", {}),
            "ui": bundle.get("ui", {}),
            "runtime_hints": bundle.get("runtime_hints", {}),
        }
        self._write_settings_payload(payload)

    def _validate_calibration(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}
        numeric_specs = {
            "exposure_us": ("int", lambda v: v > 0, "must be integer > 0"),
            "gain": ("float", lambda v: v > 0.0, "must be > 0"),
            "fps": ("int", lambda v: v > 0, "must be integer > 0"),
            "focus": ("float", lambda v: True, "must be numeric"),
            "threshold": ("int", lambda v: 0 <= v <= 255, "must be in [0,255]"),
            "circularity_min": ("float", lambda v: 0.0 <= v <= 1.0, "must be in [0,1]"),
            "mask_threshold": ("int", lambda v: 0 <= v <= 255, "must be in [0,255]"),
            "mask_seconds": ("float", lambda v: v > 0.0, "must be > 0"),
            "wand_metric_seconds": ("float", lambda v: v > 0.0, "must be > 0"),
        }
        for key, (kind, predicate, message) in numeric_specs.items():
            if key not in draft:
                continue
            raw_value = draft.get(key)
            try:
                parsed = self._to_int(raw_value) if kind == "int" else self._to_float(raw_value)
                if not predicate(parsed):
                    raise ValueError(message)
                next_committed[key] = parsed
            except Exception:
                errors[key] = message

        for key in ("blob_min_diameter_px", "blob_max_diameter_px"):
            if key not in draft:
                continue
            raw_value = draft.get(key)
            try:
                parsed_nullable = self._to_nullable_float(raw_value)
                if parsed_nullable is not None and parsed_nullable <= 0.0:
                    raise ValueError("must be > 0 when set")
                next_committed[key] = parsed_nullable
            except Exception:
                errors[key] = "must be empty or > 0"

        min_blob = next_committed.get("blob_min_diameter_px")
        max_blob = next_committed.get("blob_max_diameter_px")
        if isinstance(min_blob, (int, float)) and isinstance(max_blob, (int, float)) and min_blob > max_blob:
            errors["blob_max_diameter_px"] = "must be >= blob_min_diameter_px"
        return next_committed, errors

    def _validate_intrinsics(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}

        if "camera_id" in draft:
            value = self._to_string(draft.get("camera_id"))
            if value:
                next_committed["camera_id"] = value
            else:
                errors["camera_id"] = "must not be empty"
        if "mjpeg_url" in draft:
            value = self._to_string(draft.get("mjpeg_url"))
            if value:
                next_committed["mjpeg_url"] = value
            else:
                errors["mjpeg_url"] = "must not be empty"

        int_specs = {
            "squares_x": (2, "must be integer >= 2"),
            "squares_y": (2, "must be integer >= 2"),
            "min_frames": (5, "must be integer >= 5"),
        }
        for key, (lower_bound, message) in int_specs.items():
            if key not in draft:
                continue
            try:
                value = self._to_int(draft.get(key))
                if value < lower_bound:
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message

        float_specs = {
            "square_length_mm": (lambda v: v > 0.0, "must be > 0"),
            "cooldown_s": (lambda v: v > 0.0, "must be > 0"),
        }
        for key, (predicate, message) in float_specs.items():
            if key not in draft:
                continue
            try:
                value = self._to_float(draft.get(key))
                if not predicate(value):
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message

        if "marker_length_mm" in draft:
            try:
                marker_value = self._to_nullable_float(draft.get("marker_length_mm"))
                if marker_value is not None and marker_value <= 0.0:
                    raise ValueError("must be > 0 when set")
                next_committed["marker_length_mm"] = marker_value
            except Exception:
                errors["marker_length_mm"] = "must be empty or > 0"

        square_length = next_committed.get("square_length_mm")
        marker_length = next_committed.get("marker_length_mm")
        if (
            isinstance(square_length, (int, float))
            and isinstance(marker_length, (int, float))
            and marker_length >= square_length
        ):
            errors["marker_length_mm"] = "must be smaller than square_length_mm"
        return next_committed, errors

    def _validate_extrinsics(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        next_committed = dict(committed)
        errors: Dict[str, str] = {}
        text_keys = ("intrinsics_path", "pose_log_path", "output_path")
        for key in text_keys:
            if key not in draft:
                continue
            value = self._to_string(draft.get(key))
            if value:
                next_committed[key] = value
            else:
                errors[key] = "must not be empty"

        if "wand_metric_log_path" in draft:
            value = self._to_string(draft.get("wand_metric_log_path"))
            next_committed["wand_metric_log_path"] = value

        int_specs = {
            "pair_window_us": ("must be integer >= 1", 1),
            "wand_pair_window_us": ("must be integer >= 1", 1),
            "min_pairs": ("must be integer >= 1", 1),
        }
        for key, (message, lower_bound) in int_specs.items():
            if key not in draft:
                continue
            try:
                value = self._to_int(draft.get(key))
                if value < lower_bound:
                    raise ValueError(message)
                next_committed[key] = value
            except Exception:
                errors[key] = message
        return next_committed, errors

    def _refresh_committed_from_draft(self, bundle: Dict[str, Any]) -> Dict[str, Dict[str, str]]:
        calibration = bundle.get("calibration", {})
        intrinsics = bundle.get("intrinsics", {})
        extrinsics = bundle.get("extrinsics", {})

        cal_committed, cal_errors = self._validate_calibration(
            calibration.get("draft", {}),
            calibration.get("committed", {}),
        )
        int_committed, int_errors = self._validate_intrinsics(
            intrinsics.get("draft", {}),
            intrinsics.get("committed", {}),
        )
        ext_committed, ext_errors = self._validate_extrinsics(
            extrinsics.get("draft", {}),
            extrinsics.get("committed", {}),
        )
        calibration["committed"] = cal_committed
        intrinsics["committed"] = int_committed
        extrinsics["committed"] = ext_committed
        return {
            "calibration": cal_errors,
            "intrinsics": int_errors,
            "extrinsics": ext_errors,
        }

    def _apply_draft_patch(self, bundle: Dict[str, Any], patch: Dict[str, Any]) -> Dict[str, Any]:
        calibration_patch = patch.get("calibration", {})
        if isinstance(calibration_patch, dict):
            bundle["calibration"]["draft"].update(calibration_patch)

        intrinsics_patch = patch.get("intrinsics", {})
        if isinstance(intrinsics_patch, dict):
            bundle["intrinsics"]["draft"].update(intrinsics_patch)

        extrinsics_patch = patch.get("extrinsics", {})
        if isinstance(extrinsics_patch, dict):
            bundle["extrinsics"]["draft"].update(extrinsics_patch)
            locks = bundle["extrinsics"]["locks"]
            if "pose_log_path" in extrinsics_patch:
                locks["pose_log_path_manual"] = True
            if "wand_metric_log_path" in extrinsics_patch:
                locks["wand_metric_log_path_manual"] = True

        reset_runtime = patch.get("reset_runtime")
        if isinstance(reset_runtime, list):
            hints = bundle.get("runtime_hints", {})
            for key in reset_runtime:
                if key == "pose_log_path":
                    bundle["extrinsics"]["locks"]["pose_log_path_manual"] = False
                    latest_pose = str(hints.get("pose_log_path", DEFAULT_POSE_LOG_PATH))
                    bundle["extrinsics"]["draft"]["pose_log_path"] = latest_pose
                if key == "wand_metric_log_path":
                    bundle["extrinsics"]["locks"]["wand_metric_log_path_manual"] = False
                    latest_wand = str(hints.get("wand_metric_log_path", DEFAULT_WAND_METRIC_LOG_PATH))
                    bundle["extrinsics"]["draft"]["wand_metric_log_path"] = latest_wand

        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        self._save_settings_bundle(bundle)
        return bundle

    def _update_runtime_hints(
        self,
        *,
        pose_log_path: str | None = None,
        wand_metric_log_path: str | None = None,
    ) -> None:
        bundle = self._load_settings_bundle()
        hints = bundle["runtime_hints"]
        locks = bundle["extrinsics"]["locks"]
        if pose_log_path is not None:
            hints["pose_log_path"] = str(pose_log_path)
            if not bool(locks.get("pose_log_path_manual")):
                bundle["extrinsics"]["draft"]["pose_log_path"] = str(pose_log_path)
        if wand_metric_log_path is not None:
            hints["wand_metric_log_path"] = str(wand_metric_log_path)
            if not bool(locks.get("wand_metric_log_path_manual")):
                bundle["extrinsics"]["draft"]["wand_metric_log_path"] = str(wand_metric_log_path)
        validation = self._refresh_committed_from_draft(bundle)
        bundle["validation"] = validation
        self._save_settings_bundle(bundle)

    def get_settings(self) -> Dict[str, Any]:
        bundle = self._load_settings_bundle()
        return {
            "calibration": bundle.get("calibration", {}),
            "intrinsics": bundle.get("intrinsics", {}),
            "extrinsics": bundle.get("extrinsics", {}),
            "ui": bundle.get("ui", {}),
            "runtime_hints": bundle.get("runtime_hints", {}),
            "validation": bundle.get("validation", {}),
        }

    def apply_settings_draft(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self._load_settings_bundle()
        bundle = self._apply_draft_patch(bundle, payload)
        calibration_committed = bundle.get("calibration", {}).get("committed", {})
        if isinstance(calibration_committed, dict):
            self.config = self._build_session_config(calibration_committed)
        return {
            "ok": True,
            "calibration": bundle.get("calibration", {}),
            "intrinsics": bundle.get("intrinsics", {}),
            "extrinsics": bundle.get("extrinsics", {}),
            "validation": bundle.get("validation", {}),
            "runtime_hints": bundle.get("runtime_hints", {}),
        }

    def apply_settings_ui(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self._load_settings_bundle()
        ui_payload = bundle.get("ui", {})
        if "active_page" in payload:
            ui_payload["active_page"] = str(payload.get("active_page") or "calibration")
        if "active_view" in payload:
            ui_payload["active_view"] = str(payload.get("active_view") or "cameras")
        if "selected_camera_ids" in payload:
            selected_ids = payload.get("selected_camera_ids")
            if isinstance(selected_ids, list):
                cleaned = list(dict.fromkeys(str(item).strip() for item in selected_ids if str(item).strip()))
                ui_payload["selected_camera_ids"] = cleaned
                self.selected_camera_ids = cleaned
        bundle["ui"] = ui_payload
        self._save_settings_bundle(bundle)
        return {"ok": True, "ui": bundle.get("ui", {})}

    def _load_initial_config(self) -> CalibrationSessionConfig:
        config = self._default_config()
        bundle = self._load_settings_bundle()
        calibration = bundle.get("calibration", {})
        committed = calibration.get("committed", {}) if isinstance(calibration, dict) else {}
        if not isinstance(committed, dict):
            return config
        return self._build_session_config(committed, base_config=config)

    def _persist_config(self) -> None:
        payload = self._config_payload()
        bundle = self._load_settings_bundle()
        bundle["calibration"]["draft"].update(payload)
        bundle["calibration"]["committed"].update(payload)
        bundle["validation"] = self._refresh_committed_from_draft(bundle)
        self._save_settings_bundle(bundle)

    def _mask_params(self, config: CalibrationSessionConfig | None = None) -> Dict[str, Any]:
        source = config or self.config
        mask = dict(source.mask_params or {})
        return {
            "threshold": int(mask.get("threshold", 200)),
            "seconds": float(mask.get("seconds", 0.5)),
            "hit_ratio": float(mask.get("hit_ratio", 0.7)),
            "min_area": int(mask.get("min_area", 4)),
            "dilate": int(mask.get("dilate", 2)),
        }

    def _config_payload(self) -> Dict[str, Any]:
        mask = self._mask_params()
        return {
            "exposure_us": self.config.exposure_us,
            "gain": self.config.gain,
            "fps": self.config.fps,
            "focus": self.config.focus,
            "threshold": self.config.threshold,
            "blob_min_diameter_px": self.config.blob_min_diameter_px,
            "blob_max_diameter_px": self.config.blob_max_diameter_px,
            "circularity_min": self.config.circularity_min,
            "mask_threshold": mask["threshold"],
            "mask_seconds": mask["seconds"],
            "wand_metric_seconds": self.config.duration_s,
        }

    def _build_session_config(
        self,
        payload: Dict[str, Any],
        base_config: CalibrationSessionConfig | None = None,
    ) -> CalibrationSessionConfig:
        source = base_config or self.config
        mask = self._mask_params(source)
        mask["threshold"] = int(payload.get("mask_threshold", mask["threshold"]))
        mask["seconds"] = float(payload.get("mask_seconds", mask["seconds"]))
        return CalibrationSessionConfig(
            exposure_us=int(payload.get("exposure_us", source.exposure_us)),
            gain=float(payload.get("gain", source.gain)),
            fps=int(payload.get("fps", source.fps)),
            focus=float(payload.get("focus", source.focus)),
            threshold=int(payload.get("threshold", source.threshold)),
            blob_min_diameter_px=payload.get("blob_min_diameter_px", source.blob_min_diameter_px),
            blob_max_diameter_px=payload.get("blob_max_diameter_px", source.blob_max_diameter_px),
            circularity_min=float(payload.get("circularity_min", source.circularity_min)),
            duration_s=float(payload.get("wand_metric_seconds", source.duration_s)),
            camera_ids=self.selected_camera_ids or None,
            mask_params=mask,
            mask_retry=source.mask_retry,
            output_dir=source.output_dir,
        )

    def _apply_capture_settings(self, targets: List[Any]) -> Dict[str, Dict[str, Dict[str, Any]]]:
        return {
            "set_exposure": self.session._broadcast(targets, "set_exposure", value=self.config.exposure_us),
            "set_gain": self.session._broadcast(targets, "set_gain", value=self.config.gain),
            "set_fps": self.session._broadcast(targets, "set_fps", value=self.config.fps),
            "set_focus": self.session._broadcast(targets, "set_focus", value=self.config.focus),
            "set_threshold": self.session._broadcast(targets, "set_threshold", value=self.config.threshold),
            "set_blob_diameter": self.session._broadcast(
                targets,
                "set_blob_diameter",
                min_px=self.config.blob_min_diameter_px,
                max_px=self.config.blob_max_diameter_px,
            ),
            "set_circularity_min": self.session._broadcast(
                targets, "set_circularity_min", value=self.config.circularity_min
            ),
        }

    def refresh_targets(self) -> List[Dict[str, Any]]:
        discovered = self.session.discover_targets(self.selected_camera_ids or None)
        status_rows: List[Dict[str, Any]] = []
        ping_results = self.session._broadcast(discovered, "ping") if discovered else {}
        with self.lock:
            for target in discovered:
                ping_result = ping_results.get(target.camera_id, {})
                healthy = bool(ping_result.get("ack"))
                diagnostics = ping_result.get("result", {}) if isinstance(ping_result.get("result"), dict) else {}
                previous = self.camera_status.get(target.camera_id, {})
                self.camera_status[target.camera_id] = {
                    **previous,
                    "camera_id": target.camera_id,
                    "ip": target.ip,
                    "healthy": healthy,
                    "selected": not self.selected_camera_ids or target.camera_id in self.selected_camera_ids,
                    "diagnostics": diagnostics,
                    "last_ack": bool(ping_result.get("ack")) if ping_result else previous.get("last_ack"),
                    "last_error": ping_result.get("error") or ping_result.get("error_message"),
                }
            status_rows = sorted(self.camera_status.values(), key=lambda item: item["camera_id"])
        return status_rows

    def _workflow_summary(self, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
        selected = [camera for camera in cameras if bool(camera.get("selected"))]
        active_cameras = selected if selected else cameras
        with self.lock:
            active_capture_kind = self._active_capture_kind

        def diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = camera.get("diagnostics")
            return value if isinstance(value, dict) else {}

        def blob_diagnostics(camera: Dict[str, Any]) -> Dict[str, Any]:
            value = diagnostics(camera).get("blob_diagnostics")
            return value if isinstance(value, dict) else {}

        total_count = len(cameras)
        selected_count = len(active_cameras)
        healthy_count = sum(1 for camera in active_cameras if bool(camera.get("healthy")))
        blob_ready_count = sum(
            1
            for camera in active_cameras
            if int(blob_diagnostics(camera).get("last_blob_count", 0) or 0) > 0
        )
        mask_ready_count = sum(
            1
            for camera in active_cameras
            if diagnostics(camera).get("state") in ("READY", "RUNNING")
            and float(diagnostics(camera).get("mask_pixels", 0) or 0) > 0.0
        )
        running_count = sum(
            1 for camera in active_cameras if diagnostics(camera).get("state") == "RUNNING"
        )
        preview_enabled_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_enabled"))
        )
        preview_active_count = sum(
            1 for camera in active_cameras if bool(diagnostics(camera).get("debug_preview_active"))
        )
        pose_capture_exists = self.pose_capture_log_path.exists() and self.pose_capture_log_path.is_file()
        pose_capture_complete = pose_capture_exists or bool(self._capture_completed.get("pose_capture"))
        wand_metric_exists = self.wand_metric_log_path.exists() and self.wand_metric_log_path.is_file()
        wand_metric_complete = wand_metric_exists or bool(self._capture_completed.get("wand_metric_capture"))
        extrinsics_ready = self.latest_extrinsics_path is not None and self.latest_extrinsics_path.exists() and self.latest_extrinsics_path.is_file()

        active_segment = "blob"
        if running_count > 0:
            active_segment = "wand"
        elif pose_capture_complete and not wand_metric_complete:
            active_segment = "floor"
        elif pose_capture_complete:
            active_segment = "extrinsics"
        elif selected_count > 0 and mask_ready_count >= selected_count:
            active_segment = "wand"
        elif blob_ready_count > 0:
            active_segment = "mask"

        return {
            "total_count": total_count,
            "selected_count": selected_count,
            "healthy_count": healthy_count,
            "blob_ready_count": blob_ready_count,
            "mask_ready_count": mask_ready_count,
            "running_count": running_count,
            "preview_enabled_count": preview_enabled_count,
            "preview_active_count": preview_active_count,
            "pose_capture_complete": pose_capture_complete,
            "pose_capture_exists": pose_capture_exists,
            "pose_capture_log_path": str(self.pose_capture_log_path),
            "wand_metric_complete": wand_metric_complete,
            "wand_metric_exists": wand_metric_exists,
            "wand_metric_log_path": str(self.wand_metric_log_path),
            "extrinsics_ready": extrinsics_ready,
            "latest_extrinsics_path": str(self.latest_extrinsics_path) if self.latest_extrinsics_path else None,
            "latest_extrinsics_quality": self.latest_extrinsics_quality,
            "active_segment": active_segment,
            "active_capture_kind": active_capture_kind,
        }

    def _tracking_extrinsics_ready(self) -> bool:
        path = self.latest_extrinsics_path
        return bool(path is not None and path.exists() and path.is_file())

    @staticmethod
    def _summarize_extrinsics_quality(camera_rows: Any) -> Dict[str, Any]:
        quality_rows: List[Dict[str, Any]] = []
        if isinstance(camera_rows, list):
            for row in camera_rows:
                if isinstance(row, dict) and isinstance(row.get("quality"), dict):
                    quality_rows.append(row["quality"])
        return {
            "pair_count_total": int(sum(float(item.get("pair_count", 0) or 0) for item in quality_rows)),
            "median_reproj_error_px_max": float(
                max((float(item.get("median_reproj_error_px", 0.0) or 0.0) for item in quality_rows), default=0.0)
            ),
            "inlier_ratio_min": float(
                min((float(item.get("inlier_ratio", 1.0) or 1.0) for item in quality_rows), default=1.0)
            ),
        }

    @staticmethod
    def _summarize_validation(validation: Any) -> Dict[str, Any]:
        if not isinstance(validation, dict):
            return {}
        summary: Dict[str, Any] = {}
        for key in (
            "median_reproj_error_px",
            "p90_reproj_error_px",
            "positive_depth_ratio",
            "triangulation_angle_deg_p50",
            "triangulation_angle_deg_p90",
            "floor_residual_mm",
            "world_up_consistency",
            "baseline_range_units",
            "baseline_range_m",
        ):
            if key in validation:
                summary[key] = validation.get(key)
        return summary

    def _restore_latest_extrinsics(self, path: Path) -> None:
        if not path.exists() or not path.is_file():
            return
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        self.latest_extrinsics_path = path
        solve_summary = payload.get("pose", {}).get("solve_summary", {}) if isinstance(payload.get("pose"), dict) else {}
        if isinstance(solve_summary, dict):
            self.latest_extrinsics_quality = {
                key: solve_summary.get(key)
                for key in (
                    "usable_rows",
                    "complete_rows",
                    "median_reproj_error_px",
                    "p90_reproj_error_px",
                    "matched_delta_us_p50",
                    "matched_delta_us_p90",
                    "matched_delta_us_max",
                )
                if key in solve_summary
            }
        else:
            self.latest_extrinsics_quality = {}

    @staticmethod
    def _resolve_project_path(raw_path: str, fallback: Path) -> Path:
        candidate = raw_path.strip() if raw_path.strip() else str(fallback)
        resolved = Path(candidate)
        if not resolved.is_absolute():
            resolved = PROJECT_ROOT / resolved
        return resolved

    def _resolve_tracking_calibration_path(self, raw_path: str) -> Path:
        candidate = raw_path.strip()
        if not candidate and self.latest_extrinsics_path is not None:
            candidate = str(self.latest_extrinsics_path)
        if not candidate:
            raise ValueError("generate extrinsics first")

        resolved = Path(candidate)
        if not resolved.is_absolute():
            resolved = PROJECT_ROOT / resolved
        if resolved.is_dir():
            return resolved
        if resolved.is_file() and resolved.name.startswith("extrinsics_pose_v2"):
            return resolved.parent
        raise ValueError("tracking calibration_path must be a calibration directory or extrinsics file")

    def get_tracking_status(self) -> Dict[str, Any]:
        status = self.tracking_runtime.status()
        start_allowed = self._tracking_extrinsics_ready()
        return {
            **status,
            "start_allowed": start_allowed,
            "empty_state": None if start_allowed else "Generate extrinsics first",
            "latest_extrinsics_path": str(self.latest_extrinsics_path) if self.latest_extrinsics_path else None,
            "latest_extrinsics_quality": self.latest_extrinsics_quality,
        }

    def get_tracking_scene(self) -> Dict[str, Any]:
        start_allowed = self._tracking_extrinsics_ready()
        if not start_allowed:
            return {
                "tracking": {
                    "running": False,
                    "frames_processed": 0,
                    "poses_estimated": 0,
                },
                "cameras": [],
                "rigid_bodies": [],
                "raw_points": [],
                "timestamp_us": int(time.time() * 1_000_000),
                "empty_state": "Generate extrinsics first",
            }
        return self.tracking_runtime.scene_snapshot()

    def start_tracking(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        resolved = self._resolve_tracking_calibration_path(str(payload.get("calibration_path") or ""))
        patterns = payload.get("patterns", ["waist"])
        if not isinstance(patterns, list):
            patterns = ["waist"]
        status = self.tracking_runtime.start(str(resolved), [str(item) for item in patterns])
        response = {"ok": True, **status, "running": True}
        self.last_result = {"tracking_start": response}
        return response

    def stop_tracking(self) -> Dict[str, Any]:
        stop_result = self.tracking_runtime.stop()
        summary = stop_result.get("summary", stop_result) if isinstance(stop_result, dict) else {}
        status = self.tracking_runtime.status()
        response = {"ok": True, "summary": summary, **status, "running": False}
        self.last_result = {"tracking_stop": response}
        return response

    # ── Intrinsics capture ─────────────────────────────────────────────

    def start_intrinsics_capture(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self._load_settings_bundle()
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
                bundle = self._apply_draft_patch(bundle, {"intrinsics": intrinsics_patch})

        validation = bundle.get("validation", {}).get("intrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"intrinsics settings invalid: {validation}")

        committed = bundle.get("intrinsics", {}).get("committed", {})
        if not isinstance(committed, dict):
            committed = self._default_intrinsics_payload()
        camera_id = str(committed.get("camera_id", "pi-cam-01")).strip()
        if not camera_id:
            raise ValueError("intrinsics camera_id is required")
        self._intrinsics_assert_capability(camera_id)
        square_length_mm = float(committed.get("square_length_mm", 30.0))
        marker_length_mm_raw = committed.get("marker_length_mm")
        marker_length_mm = float(marker_length_mm_raw) if marker_length_mm_raw is not None else None
        squares_x = int(committed.get("squares_x", 6))
        squares_y = int(committed.get("squares_y", 8))
        min_frames = int(committed.get("min_frames", 25))
        cooldown_s = float(committed.get("cooldown_s", 1.5))

        status = self._intrinsics_send_command(
            camera_id,
            "intrinsics_start",
            square_length_mm=square_length_mm,
            marker_length_mm=marker_length_mm,
            squares_x=squares_x,
            squares_y=squares_y,
            min_frames=min_frames,
            cooldown_s=cooldown_s,
        )
        self._intrinsics_active_camera_id = camera_id
        self._persist_intrinsics_settings(committed)
        output_path = self._save_intrinsics_result_if_ready(camera_id, status)
        if output_path is not None:
            status["output_path"] = output_path
        return {"ok": True, "camera_id": camera_id, "status": status}

    def stop_intrinsics_capture(self) -> Dict[str, Any]:
        camera_id = self._intrinsics_active_camera_id or self._intrinsics_camera_from_settings()
        if camera_id:
            self._intrinsics_send_command(camera_id, "intrinsics_stop")
        self._intrinsics_active_camera_id = None
        return {"ok": True}

    def clear_intrinsics_frames(self) -> Dict[str, Any]:
        camera_id = self._intrinsics_active_camera_id or self._intrinsics_camera_from_settings()
        if not camera_id:
            raise ValueError("No intrinsics camera configured")
        status = self._intrinsics_send_command(camera_id, "intrinsics_clear")
        return {"ok": True, "status": status}

    def trigger_intrinsics_calibration(self) -> Dict[str, Any]:
        camera_id = self._intrinsics_active_camera_id or self._intrinsics_camera_from_settings()
        if not camera_id:
            raise ValueError("No intrinsics camera configured")
        status = self._intrinsics_send_command(camera_id, "intrinsics_calibrate")
        output_path = self._save_intrinsics_result_if_ready(camera_id, status)
        if output_path is not None:
            status["output_path"] = output_path
        return {"ok": True, "status": status}

    def discard_intrinsics_capture(self) -> Dict[str, Any]:
        camera_id = self._intrinsics_active_camera_id or self._intrinsics_camera_from_settings()
        if camera_id:
            try:
                self._intrinsics_send_command(camera_id, "intrinsics_stop")
            except Exception:
                pass
        self._intrinsics_active_camera_id = None
        self._intrinsics_last_saved_signature = None
        return {"ok": True}

    def get_intrinsics_status(self) -> Dict[str, Any]:
        targets = self.session.discover_targets(None)
        camera_list = [{"camera_id": t.camera_id, "ip": t.ip} for t in targets]
        camera_id = self._intrinsics_active_camera_id or self._intrinsics_camera_from_settings()
        if not camera_id:
            status = self._intrinsics_idle_status(None)
            status["cameras"] = camera_list
            return status

        status = self._intrinsics_idle_status(camera_id)
        try:
            target = self._intrinsics_find_target(camera_id)
            if target is None:
                raise ValueError(f"intrinsics camera not discovered: {camera_id}")
            remote_status = self._intrinsics_send_command(camera_id, "intrinsics_status")
            status.update(remote_status)
        except Exception as exc:
            status["phase"] = "error"
            status["last_error"] = str(exc)

        output_path = self._save_intrinsics_result_if_ready(camera_id, status)
        status["output_path"] = output_path
        status["cameras"] = camera_list
        return status

    def get_intrinsics_jpeg(self) -> Optional[bytes]:
        bundle = self._load_settings_bundle()
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

    def _intrinsics_idle_status(self, camera_id: str | None) -> Dict[str, Any]:
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

    def _intrinsics_camera_from_settings(self) -> str | None:
        bundle = self._load_settings_bundle()
        committed = bundle.get("intrinsics", {}).get("committed", {})
        if not isinstance(committed, dict):
            return None
        camera_id = str(committed.get("camera_id", "")).strip()
        return camera_id or None

    def _intrinsics_find_target(self, camera_id: str) -> Any | None:
        for target in self.session.discover_targets(None):
            if str(target.camera_id).strip() == camera_id:
                return target
        return None

    def _intrinsics_assert_capability(self, camera_id: str) -> None:
        target = self._intrinsics_find_target(camera_id)
        if target is None:
            raise ValueError(f"intrinsics camera not discovered: {camera_id}")
        ping_resp = self.session._broadcast([target], "ping").get(camera_id, {})
        if not bool(ping_resp.get("ack")):
            raise ValueError(ping_resp.get("error") or ping_resp.get("error_message") or "ping failed")
        result = ping_resp.get("result", {})
        supported_obj = result.get("supported_commands") if isinstance(result, dict) else None
        supported = set(supported_obj) if isinstance(supported_obj, list) else set()
        required = {
            "intrinsics_start",
            "intrinsics_stop",
            "intrinsics_clear",
            "intrinsics_calibrate",
            "intrinsics_status",
        }
        missing = sorted(required - supported)
        if missing:
            raise ValueError(
                "intrinsics capability missing on Pi. "
                f"camera_id={camera_id} missing={','.join(missing)}"
            )

    def _intrinsics_send_command(self, camera_id: str, fn_name: str, **kwargs: Any) -> Dict[str, Any]:
        target = self._intrinsics_find_target(camera_id)
        if target is None:
            raise ValueError(f"intrinsics camera not discovered: {camera_id}")
        response = self.session._broadcast([target], fn_name, **kwargs).get(camera_id, {})
        if not bool(response.get("ack")):
            raise ValueError(response.get("error") or response.get("error_message") or f"{fn_name} failed")
        result = response.get("result", {})
        return result if isinstance(result, dict) else {}

    def _save_intrinsics_result_if_ready(self, camera_id: str, status: Dict[str, Any]) -> str | None:
        calibration_result = status.get("calibration_result")
        if status.get("phase") != "done" or not isinstance(calibration_result, dict):
            return None
        signature = (
            camera_id,
            hash(json.dumps(calibration_result, ensure_ascii=False, sort_keys=True)),
        )
        output_path = PROJECT_ROOT / "calibration" / f"calibration_intrinsics_v1_{camera_id}.json"
        if self._intrinsics_last_saved_signature != signature:
            output_path.parent.mkdir(parents=True, exist_ok=True)
            output_path.write_text(
                json.dumps(calibration_result, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            self._intrinsics_last_saved_signature = signature
        return str(output_path)

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

    def _persist_intrinsics_settings(self, payload: Dict[str, Any]) -> None:
        bundle = self._load_settings_bundle()
        bundle["intrinsics"]["draft"].update(payload)
        bundle["intrinsics"]["committed"].update(payload)
        bundle["validation"] = self._refresh_committed_from_draft(bundle)
        self._save_settings_bundle(bundle)

    def _load_intrinsics_settings(self) -> Dict[str, Any]:
        defaults: Dict[str, Any] = self._default_intrinsics_payload()
        bundle = self._load_settings_bundle()
        intrinsics = bundle.get("intrinsics", {})
        draft = intrinsics.get("draft", {}) if isinstance(intrinsics, dict) else {}
        if isinstance(draft, dict):
            for key in defaults:
                if key in draft:
                    defaults[key] = draft[key]
        return defaults

    # ── end Intrinsics capture ─────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        return {
            "config": self._config_payload(),
            "cameras": cameras,
            "workflow": self._workflow_summary(cameras),
            "extrinsics_methods": self._generate_extrinsics_registry.to_payload(),
            "last_result": self.last_result,
            "receiver": self.receiver.stats,
            "tracking": self.get_tracking_status(),
            "intrinsics_settings": self._load_intrinsics_settings(),
        }

    def _sync_selected_camera_ids(self, payload: Dict[str, Any]) -> None:
        if "camera_ids" not in payload:
            return
        raw_camera_ids = payload.get("camera_ids")
        if raw_camera_ids is None:
            self.selected_camera_ids = []
            try:
                bundle = self._load_settings_bundle()
                bundle["ui"]["selected_camera_ids"] = []
                self._save_settings_bundle(bundle)
            except Exception:
                pass
            return
        if not isinstance(raw_camera_ids, list):
            raise ValueError("camera_ids must be a list")
        self.selected_camera_ids = [
            str(camera_id).strip()
            for camera_id in raw_camera_ids
            if str(camera_id).strip()
        ]
        self.selected_camera_ids = list(dict.fromkeys(self.selected_camera_ids))
        try:
            bundle = self._load_settings_bundle()
            bundle["ui"]["selected_camera_ids"] = list(self.selected_camera_ids)
            self._save_settings_bundle(bundle)
        except Exception:
            pass

    def apply_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        self._sync_selected_camera_ids(payload)
        self.config = self._build_session_config(payload)
        self._persist_config()

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        result = self._apply_capture_settings(targets)
        self._update_camera_status(result)
        self.last_result = result
        return result

    def run_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        command = str(payload.get("command", "")).strip()
        self._sync_selected_camera_ids(payload)

        if command == "refresh":
            result = {"cameras": self.refresh_targets()}
            self.last_result = result
            return result

        targets = self.session.discover_targets(self.selected_camera_ids or None)
        command_handlers = {
            "ping": lambda: self.session._broadcast(targets, "ping"),
            "mask_start": lambda: self.session._broadcast(targets, "mask_start", **self._mask_params()),
            "mask_stop": lambda: self.session._broadcast(targets, "mask_stop"),
        }
        handler = command_handlers.get(command)
        if command == "set_preview":
            kwargs: Dict[str, Any] = {}
            render_enabled = payload.get("render_enabled")
            if render_enabled is not None:
                kwargs["render_enabled"] = bool(render_enabled)
            overlays = payload.get("overlays")
            if isinstance(overlays, dict):
                kwargs["overlays"] = overlays
            charuco = payload.get("charuco")
            if isinstance(charuco, dict):
                kwargs["charuco"] = charuco
            result = self.session._broadcast(targets, "set_preview", **kwargs)
            self._update_camera_status({command: result})
            self.last_result = {command: result}
            return self.last_result
        if command in ("start", "start_pose_capture"):
            log_path = self._start_capture_log("pose_capture")
            self._update_runtime_hints(pose_log_path=str(log_path))
            result = self.session._broadcast(targets, "start", mode="pose_capture")
            payload_out: Dict[str, Any] = {command: result, "capture_log": {"path": str(log_path)}}
            if not self._all_acked(result):
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if command == "start_wand_metric_capture":
            bundle = self._load_settings_bundle()
            committed_calibration = bundle.get("calibration", {}).get("committed", {})
            committed_extrinsics = bundle.get("extrinsics", {}).get("committed", {})
            default_duration = committed_calibration.get("wand_metric_seconds", self.config.duration_s)
            duration_s = float(payload.get("duration_s", default_duration))
            if duration_s <= 0.0:
                raise ValueError("duration_s must be > 0")
            wand_log_raw = str(payload.get("wand_metric_log_path", committed_extrinsics.get("wand_metric_log_path", ""))).strip()
            if wand_log_raw:
                self.wand_metric_log_path = self._resolve_project_path(wand_log_raw, DEFAULT_WAND_METRIC_LOG_PATH)
            log_path = self._start_capture_log("wand_metric_capture")
            self._update_runtime_hints(wand_metric_log_path=str(log_path))
            result = self.session._broadcast(targets, "start", mode="wand_metric_capture")
            payload_out = {
                command: result,
                "capture_log": {"path": str(log_path)},
                "duration_s": duration_s,
            }
            if not self._all_acked(result):
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"].update(stop_meta)
            else:
                self._schedule_auto_stop([target.camera_id for target in targets], duration_s, "wand_metric_capture")
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if command in ("stop", "stop_pose_capture", "stop_wand_metric_capture"):
            self._cancel_capture_timer()
            result = self.session._broadcast(targets, "stop")
            payload_out = {command: result}
            with self.lock:
                active_kind = self._active_capture_kind
            stop_meta = self._stop_capture_log()
            if stop_meta is not None:
                payload_out["capture_log"] = stop_meta
                log_file = stop_meta.get("log_file") if isinstance(stop_meta, dict) else None
                if isinstance(log_file, str):
                    if command == "stop_wand_metric_capture" or active_kind == "wand_metric_capture":
                        self._update_runtime_hints(wand_metric_log_path=log_file)
                    else:
                        self._update_runtime_hints(pose_log_path=log_file)
            self._update_camera_status({command: result})
            self.last_result = payload_out
            return self.last_result
        if handler is None:
            raise ValueError(f"Unsupported command: {command}")
        result = handler()

        self._update_camera_status({command: result})
        self.last_result = {command: result}
        return self.last_result

    def generate_extrinsics(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        bundle = self._load_settings_bundle()
        if payload:
            extrinsics_patch = {
                key: payload[key]
                for key in (
                    "intrinsics_path",
                    "pose_log_path",
                    "wand_metric_log_path",
                    "output_path",
                    "pair_window_us",
                    "wand_pair_window_us",
                    "min_pairs",
                )
                if key in payload
            }
            if "log_path" in payload and "pose_log_path" not in extrinsics_patch:
                extrinsics_patch["pose_log_path"] = payload["log_path"]
            if extrinsics_patch:
                bundle = self._apply_draft_patch(bundle, {"extrinsics": extrinsics_patch})
        validation = bundle.get("validation", {}).get("extrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"extrinsics settings invalid: {validation}")

        committed_extrinsics = bundle.get("extrinsics", {}).get("committed", {})
        if not isinstance(committed_extrinsics, dict):
            committed_extrinsics = self._default_extrinsics_payload()

        extrinsics_method = str(payload.get("extrinsics_method", "blob_pose_v2")).strip() or "blob_pose_v2"
        intrinsics_raw = str(committed_extrinsics.get("intrinsics_path", "calibration")).strip()
        log_raw = str(committed_extrinsics.get("pose_log_path", str(DEFAULT_POSE_LOG_PATH))).strip()
        output_raw = str(committed_extrinsics.get("output_path", str(DEFAULT_EXTRINSICS_OUTPUT_PATH))).strip()
        intrinsics_path = self._resolve_project_path(intrinsics_raw, Path("calibration"))
        resolved_log_path = self._resolve_project_path(log_raw, DEFAULT_POSE_LOG_PATH)
        wand_log_raw = str(
            committed_extrinsics.get("wand_metric_log_path", str(self.wand_metric_log_path))
        ).strip()
        resolved_output = self._resolve_project_path(output_raw, DEFAULT_EXTRINSICS_OUTPUT_PATH)
        resolved_wand_log_path = self._resolve_project_path(wand_log_raw, DEFAULT_WAND_METRIC_LOG_PATH)
        pair_window_us = int(committed_extrinsics.get("pair_window_us", 2000))
        min_pairs = int(committed_extrinsics.get("min_pairs", 8))
        wand_pair_window_us = int(committed_extrinsics.get("wand_pair_window_us", 8000))
        if pair_window_us < 1:
            raise ValueError("pair_window_us must be >= 1")
        if min_pairs < 1:
            raise ValueError("min_pairs must be >= 1")
        if wand_pair_window_us < 1:
            raise ValueError("wand_pair_window_us must be >= 1")

        if not resolved_log_path.exists():
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path does not exist: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result
        if not resolved_log_path.is_file():
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path is not a file: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result

        try:
            if extrinsics_method == "blob_pose_v2" and callable(self._generate_extrinsics_solver):
                solve_fn = self._generate_extrinsics_solver
            else:
                solve_fn = self._generate_extrinsics_registry.get(extrinsics_method).solve
            raw_result = solve_fn(
                intrinsics_path=str(intrinsics_path),
                pose_log_path=str(resolved_log_path),
                output_path=str(resolved_output),
                pair_window_us=pair_window_us,
                min_pairs=min_pairs,
                wand_metric_log_path=str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
                wand_pair_window_us=wand_pair_window_us,
            )
        except (FileNotFoundError, ValueError) as exc:
            reason = str(exc)
            if isinstance(exc, FileNotFoundError):
                missing_path = Path(getattr(exc, "filename", "") or str(resolved_log_path))
                reason = f"log_path does not exist: {missing_path}"
            summary = self._build_generate_extrinsics_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=reason,
                resolved_wand_log_path=resolved_wand_log_path,
            )
            self.last_result = {"generate_extrinsics": summary}
            return self.last_result
        if not isinstance(raw_result, dict):
            raise ValueError("extrinsics solver returned invalid response")

        pose_section = raw_result.get("pose", {})
        camera_rows = pose_section.get("camera_poses", []) if isinstance(pose_section, dict) else []
        camera_count = len(camera_rows) if isinstance(camera_rows, list) else 0
        solve_summary = pose_section.get("solve_summary", {}) if isinstance(pose_section, dict) else {}
        if not isinstance(solve_summary, dict):
            solve_summary = {}
        quality_summary = {
            key: solve_summary.get(key)
            for key in (
                "usable_rows",
                "complete_rows",
                "median_reproj_error_px",
                "p90_reproj_error_px",
                "matched_delta_us_p50",
                "matched_delta_us_p90",
                "matched_delta_us_max",
            )
            if key in solve_summary
        }

        summary = {
            "ok": True,
            "extrinsics_method": extrinsics_method,
            "camera_order": raw_result.get("camera_order", []),
            "camera_count": camera_count,
            "output_path": str(resolved_output),
            "quality": quality_summary,
            "metric_status": raw_result.get("metric", {}).get("status"),
            "world_status": raw_result.get("world", {}).get("status"),
            "wand_metric_log_path": str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
        }
        self.latest_extrinsics_path = resolved_output
        self.latest_extrinsics_quality = quality_summary
        self.last_result = {"generate_extrinsics": summary}
        self._update_runtime_hints(
            pose_log_path=str(resolved_log_path),
            wand_metric_log_path=str(resolved_wand_log_path),
        )
        return self.last_result

    @staticmethod
    def _extract_pose_log_payload(line: str) -> Dict[str, Any] | None:
        line = line.strip()
        if not line:
            return None
        payload = json.loads(line)
        if isinstance(payload, dict) and payload.get("_type") == "frame":
            frame = payload.get("data")
            if isinstance(frame, dict):
                return frame
        return payload if isinstance(payload, dict) else None

    def _summarize_pose_log(self, log_path: Path) -> Dict[str, Any]:
        total_by_camera: Counter[str] = Counter()
        single_blob_by_camera: Counter[str] = Counter()
        invalid_rows = 0
        try:
            with log_path.open("r", encoding="utf-8") as handle:
                for raw_line in handle:
                    try:
                        payload = self._extract_pose_log_payload(raw_line)
                    except json.JSONDecodeError:
                        invalid_rows += 1
                        continue
                    if not payload:
                        continue
                    camera_id = str(payload.get("camera_id", "")).strip()
                    if not camera_id:
                        continue
                    blobs = payload.get("blobs", [])
                    if not isinstance(blobs, list):
                        blobs = []
                    blob_count = payload.get("blob_count", len(blobs))
                    try:
                        blob_count = int(blob_count)
                    except (TypeError, ValueError):
                        blob_count = len(blobs)
                    total_by_camera[camera_id] += 1
                    if blob_count == 1 and len(blobs) == 1:
                        single_blob_by_camera[camera_id] += 1
        except OSError:
            return {
                "exists": False,
                "is_file": False,
                "rows_by_camera": {},
                "single_blob_rows_by_camera": {},
                "usable_camera_count": 0,
                "invalid_rows": invalid_rows,
            }
        return {
            "exists": log_path.exists(),
            "is_file": log_path.is_file(),
            "rows_by_camera": dict(total_by_camera),
            "single_blob_rows_by_camera": dict(single_blob_by_camera),
            "usable_camera_count": sum(1 for count in single_blob_by_camera.values() if count > 0),
            "invalid_rows": invalid_rows,
        }

    def _build_generate_extrinsics_failure(
        self,
        *,
        resolved_log_path: Path,
        resolved_output: Path,
        reason: str,
        resolved_wand_log_path: Path,
    ) -> Dict[str, Any]:
        pose_log_summary = self._summarize_pose_log(resolved_log_path) if resolved_log_path.exists() and resolved_log_path.is_file() else {
            "exists": resolved_log_path.exists(),
            "is_file": resolved_log_path.is_file(),
            "rows_by_camera": {},
            "single_blob_rows_by_camera": {},
            "usable_camera_count": 0,
            "invalid_rows": 0,
        }
        return {
            "ok": False,
            "output_path": str(resolved_output),
            "error": reason,
            "pose_log_path": str(resolved_log_path),
            "pose_log_summary": pose_log_summary,
            "wand_metric_log_path": str(resolved_wand_log_path) if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file() else None,
        }

    def _update_camera_status(self, result: Dict[str, Dict[str, Dict[str, Any]]]) -> None:
        with self.lock:
            for responses in result.values():
                for camera_id, response in responses.items():
                    entry = self.camera_status.setdefault(camera_id, {"camera_id": camera_id, "ip": "unknown"})
                    entry["last_ack"] = bool(response.get("ack"))
                    entry["last_error"] = response.get("error") or response.get("error_message")

    def _on_frame_received(self, frame: Any) -> None:
        previous = self._receiver_frame_callback
        if callable(previous):
            previous(frame)
        logger: FrameLogger | None = None
        with self.lock:
            if self._capture_log_active:
                logger = self._capture_logger
        if logger is None:
            return
        frame_dict = frame.to_dict() if hasattr(frame, "to_dict") else dict(frame)
        try:
            logger.log_frame(frame_dict)
        except Exception:
            return

    def _start_capture_log(self, capture_kind: str) -> Path:
        with self.lock:
            if self._capture_logger is not None and self._capture_log_active:
                if self._active_capture_kind != capture_kind:
                    raise ValueError(f"capture already active: {self._active_capture_kind}")
                default_path = (
                    self.pose_capture_log_path
                    if capture_kind == "pose_capture"
                    else self.wand_metric_log_path
                )
                return Path(self._capture_logger.current_log_file or str(default_path))
            self.capture_log_dir.mkdir(parents=True, exist_ok=True)
            target_path = self.pose_capture_log_path if capture_kind == "pose_capture" else self.wand_metric_log_path
            if target_path.exists():
                target_path.unlink(missing_ok=True)
            logger = FrameLogger(log_dir=str(self.capture_log_dir))
            log_file = logger.start_recording(session_name=target_path.stem)
            self._capture_logger = logger
            self._capture_log_active = True
            self._active_capture_kind = capture_kind
            self._capture_completed[capture_kind] = False
            if capture_kind == "pose_capture":
                self.pose_capture_log_path = Path(log_file)
            else:
                self.wand_metric_log_path = Path(log_file)
            return Path(log_file)

    def _stop_capture_log(self) -> Dict[str, Any] | None:
        with self.lock:
            logger = self._capture_logger
            active = self._capture_log_active
        if logger is None or not active:
            return None
        metadata = logger.stop_recording()
        with self.lock:
            self._capture_log_active = False
            active_kind = self._active_capture_kind
            self._active_capture_kind = None
            if isinstance(metadata.get("log_file"), str):
                path = Path(str(metadata["log_file"]))
                if active_kind == "pose_capture":
                    self.pose_capture_log_path = path
                elif active_kind == "wand_metric_capture":
                    self.wand_metric_log_path = path
            if active_kind == "pose_capture":
                self._capture_completed["pose_capture"] = self.pose_capture_log_path.exists() and self.pose_capture_log_path.is_file()
            elif active_kind == "wand_metric_capture":
                self._capture_completed["wand_metric_capture"] = self.wand_metric_log_path.exists() and self.wand_metric_log_path.is_file()
        return metadata

    def _cancel_capture_timer(self) -> None:
        with self.lock:
            timer = self._capture_auto_stop_timer
            self._capture_auto_stop_timer = None
        if timer is not None:
            timer.cancel()

    def _schedule_auto_stop(self, camera_ids: List[str], duration_s: float, capture_kind: str) -> None:
        def _auto_stop() -> None:
            try:
                targets = self.session.discover_targets(camera_ids or None)
                result = self.session._broadcast(targets, "stop")
                payload_out: Dict[str, Any] = {f"stop_{capture_kind}": result}
                stop_meta = self._stop_capture_log()
                if stop_meta is not None:
                    payload_out["capture_log"] = stop_meta
                    log_file = stop_meta.get("log_file") if isinstance(stop_meta, dict) else None
                    if isinstance(log_file, str):
                        if capture_kind == "pose_capture":
                            self._update_runtime_hints(pose_log_path=log_file)
                        elif capture_kind == "wand_metric_capture":
                            self._update_runtime_hints(wand_metric_log_path=log_file)
                self._update_camera_status({"stop": result})
                self.last_result = payload_out
            finally:
                with self.lock:
                    self._capture_auto_stop_timer = None

        timer = threading.Timer(duration_s, _auto_stop)
        timer.daemon = True
        with self.lock:
            if self._capture_auto_stop_timer is not None:
                self._capture_auto_stop_timer.cancel()
            self._capture_auto_stop_timer = timer
        timer.start()

    @staticmethod
    def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return all(bool(resp.get("ack")) for resp in responses.values())


class LoutrackGuiHandler(BaseHTTPRequestHandler):
    state: LoutrackGuiState

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path.startswith("/static/"):
            self._serve_static(path[len("/static/"):])
            return
        if path in ("/", "/index.html"):
            # Prefer the on-disk static file so edits take effect without restart.
            target = _resolve_static_asset("index.html")
            if target is not None:
                self._serve_static("index.html")
            else:
                self._send_html(HTML_PAGE)
            return
        if path == "/api/state":
            self._send_json(self.state.get_state())
            return
        if path == "/api/settings":
            self._send_json(self.state.get_settings())
            return
        if path == "/api/tracking/status":
            self._send_json(self.state.get_tracking_status())
            return
        if path == "/api/tracking/scene":
            self._send_json(self.state.get_tracking_scene())
            return
        if path == "/api/intrinsics/status":
            self._send_json(self.state.get_intrinsics_status())
            return
        if path == "/api/intrinsics/frame.jpg":
            jpeg = self.state.get_intrinsics_jpeg()
            if jpeg is None:
                self.send_response(HTTPStatus.NO_CONTENT)
                self.end_headers()
            else:
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(jpeg)
            return
        self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/config":
                self._send_json(self.state.apply_config(payload))
                return
            if self.path == "/api/settings/draft":
                self._send_json(self.state.apply_settings_draft(payload))
                return
            if self.path == "/api/settings/ui":
                self._send_json(self.state.apply_settings_ui(payload))
                return
            if self.path == "/api/command":
                self._debug_log(
                    f"POST /api/command command={payload.get('command')} camera_ids={payload.get('camera_ids')}"
                )
                self._send_json(self.state.run_command(payload))
                return
            if self.path == "/api/generate_extrinsics":
                self._debug_log("POST /api/generate_extrinsics")
                self._send_json(self.state.generate_extrinsics(payload))
                return
            if self.path == "/api/tracking/start":
                self._debug_log("POST /api/tracking/start")
                self._send_json(self.state.start_tracking(payload))
                return
            if self.path == "/api/tracking/stop":
                self._debug_log("POST /api/tracking/stop")
                self._send_json(self.state.stop_tracking())
                return
            if self.path == "/api/intrinsics/start":
                self._send_json(self.state.start_intrinsics_capture(payload))
                return
            if self.path == "/api/intrinsics/stop":
                self._send_json(self.state.stop_intrinsics_capture())
                return
            if self.path == "/api/intrinsics/clear":
                self._send_json(self.state.clear_intrinsics_frames())
                return
            if self.path == "/api/intrinsics/calibrate":
                self._send_json(self.state.trigger_intrinsics_calibration())
                return
            if self.path == "/api/intrinsics/discard":
                self._send_json(self.state.discard_intrinsics_capture())
                return
            self.send_error(HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            self._debug_log(f"BAD_REQUEST path={self.path} error={exc}")
            self._send_json({"error": str(exc)}, status=HTTPStatus.BAD_REQUEST)
        except Exception as exc:  # noqa: BLE001
            self._debug_log(f"INTERNAL_ERROR path={self.path} error={exc}")
            self._send_json({"error": f"internal_error: {exc}"}, status=HTTPStatus.INTERNAL_SERVER_ERROR)

    def log_message(self, format: str, *args: Any) -> None:
        return

    @staticmethod
    def _debug_log(message: str) -> None:
        print(f"[loutrack_gui] {message}", flush=True)

    def _read_json(self) -> Dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length > 0 else b"{}"
        return json.loads(raw.decode("utf-8"))

    def _send_html(self, body: str) -> None:
        data = body.encode("utf-8")
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_json(self, payload: Dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_static(self, rel_path: str) -> None:
        target = _resolve_static_asset(rel_path)
        if target is None:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        try:
            data = target.read_bytes()
        except Exception:
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", self._guess_content_type(target.suffix))
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    @staticmethod
    def _guess_content_type(suffix: str) -> str:
        mapping = {
            ".js": "application/javascript",
            ".mjs": "application/javascript",
            ".css": "text/css",
            ".json": "application/json",
            ".html": "text/html",
        }
        return mapping.get(suffix.lower(), "application/octet-stream")


def _resolve_static_asset(rel_path: str) -> Optional[Path]:
    candidate = Path(rel_path.lstrip("/"))
    for static_dir in STATIC_DIR_CANDIDATES:
        try:
            static_root = static_dir.resolve()
            target = (static_root / candidate).resolve()
            target.relative_to(static_root)
        except Exception:
            continue
        if target.exists() and target.is_file():
            return target
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Loutrack GUI for calibration and tracking control")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind host")
    parser.add_argument("--port", type=int, default=8765, help="HTTP bind port")
    parser.add_argument("--udp-port", type=int, default=5000, help="Passive discovery UDP port")
    parser.add_argument("--inventory", default=None, help="Optional hosts.ini path")
    return parser.parse_args()


def _load_extrinsics_solver():
    script_path = MODULE_SRC_ROOT / "camera-calibration" / "calibrate_extrinsics.py"
    spec = importlib.util.spec_from_file_location("calibrate_extrinsics", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load solver module: {script_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules["calibrate_extrinsics"] = module
    spec.loader.exec_module(module)
    solve_extrinsics = getattr(module, "solve_extrinsics", None)
    if not callable(solve_extrinsics):
        raise RuntimeError("solve_extrinsics is missing in calibrate_extrinsics.py")
    return solve_extrinsics


def main() -> None:
    args = parse_args()
    receiver = UDPReceiver(port=args.udp_port)
    receiver.start()
    session = CalibrationSession(
        inventory_path=Path(args.inventory) if args.inventory else None,
        receiver=receiver,
    )
    state = LoutrackGuiState(session=session, receiver=receiver)
    LoutrackGuiHandler.state = state
    server = ThreadingHTTPServer((args.host, args.port), LoutrackGuiHandler)
    try:
        print(f"Loutrack GUI listening on http://{args.host}:{args.port}")
        server.serve_forever()
    finally:
        server.server_close()
        receiver.stop()


if __name__ == "__main__":
    main()
