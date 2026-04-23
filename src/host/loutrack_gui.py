#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
import time
import threading
import importlib.util
import urllib.error
import urllib.parse
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
    from host.wand_session import (
        CalibrationSession,
        CalibrationSessionConfig,
        DEFAULT_MASK_THRESHOLD,
        DEFAULT_THRESHOLD,
        FIXED_CIRCULARITY_MIN,
        FIXED_FPS,
        FIXED_FOCUS,
    )
    from host.extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry
    from host.intrinsics_host_session import IntrinsicsHostSession, IntrinsicsHostSessionConfig
    from host.gui_settings_store import GuiSettingsStore
    from host.gui_tracking_service import GuiTrackingService
    from host.gui_intrinsics_service import GuiIntrinsicsService
    from host.gui_capture_log_service import GuiCaptureLogService
    from host.gui_extrinsics_service import GuiExtrinsicsService
    from host.gui_camera_status_store import GuiCameraStatusStore
    from host.gui_workflow_presenter import GuiWorkflowPresenter
    from host.gui_state_presenter import GuiStatePresenter
    from host.gui_calibration_config_service import GuiCalibrationConfigService
    from host.gui_command_service import GuiCommandService
else:
    from .receiver import UDPReceiver
    from .logger import FrameLogger
    from .tracking_runtime import TrackingRuntime
    from .wand_session import (
        CalibrationSession,
        CalibrationSessionConfig,
        DEFAULT_MASK_THRESHOLD,
        DEFAULT_THRESHOLD,
        FIXED_CIRCULARITY_MIN,
        FIXED_FPS,
        FIXED_FOCUS,
    )
    from .extrinsics_methods import ExtrinsicsMethodRegistry, build_default_extrinsics_registry
    from .intrinsics_host_session import IntrinsicsHostSession, IntrinsicsHostSessionConfig
    from .gui_settings_store import GuiSettingsStore
    from .gui_tracking_service import GuiTrackingService
    from .gui_intrinsics_service import GuiIntrinsicsService
    from .gui_capture_log_service import GuiCaptureLogService
    from .gui_extrinsics_service import GuiExtrinsicsService
    from .gui_camera_status_store import GuiCameraStatusStore
    from .gui_workflow_presenter import GuiWorkflowPresenter
    from .gui_state_presenter import GuiStatePresenter
    from .gui_calibration_config_service import GuiCalibrationConfigService
    from .gui_command_service import GuiCommandService


DEFAULT_SETTINGS_PATH = Path("logs") / "loutrack_gui_settings.json"
OLD_SETTINGS_PATH = Path("logs") / "wand_gui_settings.json"
DEFAULT_POSE_LOG_PATH = Path("logs") / "extrinsics_pose_capture.jsonl"
DEFAULT_WAND_METRIC_LOG_PATH = Path("logs") / "extrinsics_wand_metric.jsonl"
DEFAULT_EXTRINSICS_OUTPUT_PATH = Path("calibration") / "extrinsics_pose_v2.json"
DEFAULT_CAPTURE_LOG_DIR = Path("logs")
DEFAULT_WAND_METRIC_DURATION_S = 1.0
STATIC_DIR_CANDIDATES = (
    PROJECT_ROOT / "static",
    MODULE_SRC_ROOT / "static",
)


def _summary_values(values: List[float]) -> Dict[str, float]:
    if not values:
        return {"last": 0.0, "mean": 0.0, "p95": 0.0, "max": 0.0}
    sorted_values = sorted(float(v) for v in values)
    p95_index = min(len(sorted_values) - 1, int(round((len(sorted_values) - 1) * 0.95)))
    return {
        "last": float(values[-1]),
        "mean": sum(sorted_values) / len(sorted_values),
        "p95": sorted_values[p95_index],
        "max": sorted_values[-1],
    }


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
        self.lock = threading.RLock()
        self._uses_default_settings_path = settings_path is None
        default_settings_path = DEFAULT_SETTINGS_PATH
        if not default_settings_path.is_absolute():
            default_settings_path = (PROJECT_ROOT / default_settings_path).resolve()
        self.settings_path = (settings_path or default_settings_path).resolve()
        old_settings_path = OLD_SETTINGS_PATH
        if not old_settings_path.is_absolute():
            old_settings_path = (PROJECT_ROOT / old_settings_path).resolve()
        self._settings_store = GuiSettingsStore(
            project_root=PROJECT_ROOT,
            settings_path=self.settings_path,
            old_settings_path=old_settings_path,
            uses_default_settings_path=self._uses_default_settings_path,
            default_pose_log_path=self._default_pose_log_path(),
            default_wand_metric_log_path=self._default_wand_metric_log_path(),
            default_extrinsics_output_path=self._default_extrinsics_output_path(),
            default_wand_metric_duration_s=DEFAULT_WAND_METRIC_DURATION_S,
        )
        self.settings_path = self._settings_store.settings_path
        self._migrate_settings_once()
        self.selected_camera_ids: List[str] = []
        self.config = self._load_initial_config()
        self.camera_status: Dict[str, Dict[str, Any]] = {}
        self.last_result: Dict[str, Any] = {"status": "idle"}
        self.capture_log_dir: Path = DEFAULT_CAPTURE_LOG_DIR
        self.pose_capture_log_path: Path = self._default_pose_log_path()
        self.wand_metric_log_path: Path = self._default_wand_metric_log_path()
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
        self._receiver_paused_for_tracking = False
        self._tracking_camera_ids: List[str] = []
        self._tracking_sse_clients = 0
        self._tracking_sse_events_sent = 0
        self._tracking_sse_keepalives = 0
        self._tracking_sse_broken_pipes = 0
        self._tracking_sse_write_ms: List[float] = []
        self._tracking_sse_payload_bytes: List[float] = []
        self._preview_proxy_errors: Dict[str, str] = {}
        self.latest_extrinsics_path: Path | None = None
        self.latest_extrinsics_quality: Dict[str, Any] | None = None
        self.latest_extrinsics_result: Dict[str, Any] | None = None
        self._restore_latest_extrinsics(self._default_extrinsics_output_path())
        self._intrinsics_host_session: IntrinsicsHostSession | None = None
        self._tracking_service = GuiTrackingService(self)
        self._intrinsics_service = GuiIntrinsicsService(self)
        self._capture_log_service = GuiCaptureLogService(self)
        self._extrinsics_service = GuiExtrinsicsService(self)
        self._camera_status_store = GuiCameraStatusStore(self)
        self._workflow_presenter = GuiWorkflowPresenter(self)
        self._state_presenter = GuiStatePresenter(self)
        self._calibration_config_service = GuiCalibrationConfigService(self)
        self._command_service = GuiCommandService(self)
        self._ensure_settings_file_exists()
        settings_bundle = self._load_settings_bundle()
        ui_payload = settings_bundle.get("ui", {})
        if isinstance(ui_payload, dict):
            selected_ids = ui_payload.get("selected_camera_ids", [])
            if isinstance(selected_ids, list):
                self.selected_camera_ids = [str(item).strip() for item in selected_ids if str(item).strip()]

    @staticmethod
    def _default_settings_payload() -> Dict[str, Any]:
        store = GuiSettingsStore(
            project_root=PROJECT_ROOT,
            settings_path=DEFAULT_SETTINGS_PATH if DEFAULT_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / DEFAULT_SETTINGS_PATH),
            old_settings_path=OLD_SETTINGS_PATH if OLD_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / OLD_SETTINGS_PATH),
            uses_default_settings_path=True,
            default_pose_log_path=DEFAULT_POSE_LOG_PATH,
            default_wand_metric_log_path=DEFAULT_WAND_METRIC_LOG_PATH,
            default_extrinsics_output_path=DEFAULT_EXTRINSICS_OUTPUT_PATH,
            default_wand_metric_duration_s=DEFAULT_WAND_METRIC_DURATION_S,
        )
        return store.default_settings_payload()

    @staticmethod
    def _default_calibration_payload() -> Dict[str, Any]:
        store = GuiSettingsStore(
            project_root=PROJECT_ROOT,
            settings_path=DEFAULT_SETTINGS_PATH if DEFAULT_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / DEFAULT_SETTINGS_PATH),
            old_settings_path=OLD_SETTINGS_PATH if OLD_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / OLD_SETTINGS_PATH),
            uses_default_settings_path=True,
            default_pose_log_path=DEFAULT_POSE_LOG_PATH,
            default_wand_metric_log_path=DEFAULT_WAND_METRIC_LOG_PATH,
            default_extrinsics_output_path=DEFAULT_EXTRINSICS_OUTPUT_PATH,
            default_wand_metric_duration_s=DEFAULT_WAND_METRIC_DURATION_S,
        )
        return store.default_calibration_payload()

    @staticmethod
    def _default_intrinsics_payload() -> Dict[str, Any]:
        return GuiSettingsStore.default_intrinsics_payload()

    @staticmethod
    def _default_extrinsics_payload() -> Dict[str, Any]:
        store = GuiSettingsStore(
            project_root=PROJECT_ROOT,
            settings_path=DEFAULT_SETTINGS_PATH if DEFAULT_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / DEFAULT_SETTINGS_PATH),
            old_settings_path=OLD_SETTINGS_PATH if OLD_SETTINGS_PATH.is_absolute() else (PROJECT_ROOT / OLD_SETTINGS_PATH),
            uses_default_settings_path=True,
            default_pose_log_path=DEFAULT_POSE_LOG_PATH,
            default_wand_metric_log_path=DEFAULT_WAND_METRIC_LOG_PATH,
            default_extrinsics_output_path=DEFAULT_EXTRINSICS_OUTPUT_PATH,
            default_wand_metric_duration_s=DEFAULT_WAND_METRIC_DURATION_S,
        )
        return store.default_extrinsics_payload()

    def _migrate_settings_once(self) -> None:
        self._settings_store.migrate_once()

    def _default_config(self) -> CalibrationSessionConfig:
        return CalibrationSessionConfig(
            exposure_us=5000,
            gain=8.0,
            fps=FIXED_FPS,
            focus=FIXED_FOCUS,
            threshold=DEFAULT_THRESHOLD,
            blob_min_diameter_px=None,
            blob_max_diameter_px=None,
            circularity_min=FIXED_CIRCULARITY_MIN,
            duration_s=DEFAULT_WAND_METRIC_DURATION_S,
        )

    @staticmethod
    def _default_pose_log_path() -> Path:
        return DEFAULT_POSE_LOG_PATH

    @staticmethod
    def _default_wand_metric_log_path() -> Path:
        return DEFAULT_WAND_METRIC_LOG_PATH

    @staticmethod
    def _default_extrinsics_output_path() -> Path:
        return DEFAULT_EXTRINSICS_OUTPUT_PATH

    def _build_intrinsics_host_session_config(
        self,
        *,
        camera_id: str,
        mjpeg_url: str,
        square_length_mm: float,
        marker_length_mm: float,
        squares_x: int,
        squares_y: int,
        min_frames: int,
        cooldown_s: float,
    ) -> IntrinsicsHostSessionConfig:
        return IntrinsicsHostSessionConfig(
            camera_id=camera_id,
            mjpeg_url=mjpeg_url,
            square_length_mm=square_length_mm,
            marker_length_mm=marker_length_mm,
            squares_x=squares_x,
            squares_y=squares_y,
            min_frames=min_frames,
            poll_interval_s=cooldown_s,
            output_dir=PROJECT_ROOT / "calibration",
        )

    def _create_intrinsics_host_session(
        self,
        config: IntrinsicsHostSessionConfig,
        broadcast_fn: Any,
    ) -> IntrinsicsHostSession:
        return IntrinsicsHostSession(config, broadcast_fn)

    def _read_settings_payload(self) -> Dict[str, Any]:
        return self._settings_store._read_settings_payload()

    def _write_settings_payload(self, payload: Dict[str, Any]) -> None:
        self._settings_store._write_settings_payload(payload)

    def _ensure_settings_file_exists(self) -> None:
        self._settings_store.ensure_settings_file_exists()

    @staticmethod
    def _to_int(value: Any) -> int:
        return GuiSettingsStore._to_int(value)

    @staticmethod
    def _to_float(value: Any) -> float:
        return GuiSettingsStore._to_float(value)

    @staticmethod
    def _to_nullable_float(value: Any) -> float | None:
        return GuiSettingsStore._to_nullable_float(value)

    @staticmethod
    def _to_string(value: Any) -> str:
        return GuiSettingsStore._to_string(value)

    def _normalize_settings_payload(self, raw_payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._settings_store._normalize_settings_payload(raw_payload)

    def _load_settings_bundle(self) -> Dict[str, Any]:
        return self._settings_store.load_bundle()

    def _save_settings_bundle(self, bundle: Dict[str, Any]) -> None:
        self._settings_store.save_bundle(bundle)

    def _validate_calibration(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        return GuiSettingsStore.validate_calibration(draft, committed)

    def _validate_intrinsics(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        return GuiSettingsStore.validate_intrinsics(draft, committed)

    def _validate_extrinsics(self, draft: Dict[str, Any], committed: Dict[str, Any]) -> tuple[Dict[str, Any], Dict[str, str]]:
        return GuiSettingsStore.validate_extrinsics(draft, committed)

    def _refresh_committed_from_draft(self, bundle: Dict[str, Any]) -> Dict[str, Dict[str, str]]:
        return self._settings_store._refresh_committed_from_draft(bundle)

    def _apply_draft_patch(self, bundle: Dict[str, Any], patch: Dict[str, Any]) -> Dict[str, Any]:
        return self._settings_store.apply_draft_patch(bundle, patch)

    def _update_runtime_hints(
        self,
        *,
        pose_log_path: str | None = None,
        wand_metric_log_path: str | None = None,
    ) -> None:
        self._settings_store.update_runtime_hints(
            pose_log_path=pose_log_path,
            wand_metric_log_path=wand_metric_log_path,
        )

    def get_settings(self) -> Dict[str, Any]:
        return self._settings_store.get_settings()

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
        return self._settings_store.apply_ui(payload, self.selected_camera_ids)

    def _load_initial_config(self) -> CalibrationSessionConfig:
        config = self._default_config()
        bundle = self._load_settings_bundle()
        calibration = bundle.get("calibration", {})
        committed = calibration.get("committed", {}) if isinstance(calibration, dict) else {}
        if not isinstance(committed, dict):
            return config
        return self._build_session_config(committed, base_config=config)

    def _persist_config(self) -> None:
        self._settings_store.persist_calibration_payload(self._config_payload())

    def _mask_params(self, config: CalibrationSessionConfig | None = None) -> Dict[str, Any]:
        source = config or self.config
        mask = dict(source.mask_params or {})
        return {
            "threshold": int(mask.get("threshold", DEFAULT_MASK_THRESHOLD)),
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
            fps=FIXED_FPS,
            focus=FIXED_FOCUS,
            threshold=int(payload.get("threshold", source.threshold)),
            blob_min_diameter_px=payload.get("blob_min_diameter_px", source.blob_min_diameter_px),
            blob_max_diameter_px=payload.get("blob_max_diameter_px", source.blob_max_diameter_px),
            circularity_min=FIXED_CIRCULARITY_MIN,
            duration_s=float(payload.get("wand_metric_seconds", source.duration_s)),
            camera_ids=None,
            mask_params=mask,
            mask_retry=source.mask_retry,
            output_dir=source.output_dir,
        )

    @staticmethod
    def _normalize_camera_ids(camera_ids_obj: Any) -> List[str]:
        if not isinstance(camera_ids_obj, list):
            return []
        cleaned = [str(item).strip() for item in camera_ids_obj if str(item).strip()]
        return list(dict.fromkeys(cleaned))

    def _discover_workflow_targets(self, camera_ids: List[str] | None = None) -> List[Any]:
        return self.session.discover_targets(camera_ids or None)

    def _resolve_requested_targets(self, payload: Dict[str, Any], *, refresh: bool = False) -> List[Any]:
        if refresh:
            return self.session.discover_targets(None)
        camera_ids = self._normalize_camera_ids(payload.get("camera_ids"))
        if not camera_ids:
            raise ValueError("camera_ids must not be empty")
        targets = self._discover_workflow_targets(camera_ids)
        if not targets:
            raise ValueError(f"no target cameras discovered for camera_ids={camera_ids}")
        return targets

    def _ensure_calibration_settings_valid(self) -> Dict[str, Any]:
        return self._settings_store.ensure_calibration_settings_valid()

    def _apply_capture_settings(self, targets: List[Any]) -> Dict[str, Dict[str, Dict[str, Any]]]:
        return {
            "set_exposure": self.session._broadcast(targets, "set_exposure", value=self.config.exposure_us),
            "set_gain": self.session._broadcast(targets, "set_gain", value=self.config.gain),
            "set_threshold": self.session._broadcast(targets, "set_threshold", value=self.config.threshold),
            "set_blob_diameter": self.session._broadcast(
                targets,
                "set_blob_diameter",
                min_px=self.config.blob_min_diameter_px,
                max_px=self.config.blob_max_diameter_px,
            ),
        }

    def refresh_targets(self) -> List[Dict[str, Any]]:
        return self._camera_status_store.refresh_targets()

    def _workflow_summary(self, cameras: List[Dict[str, Any]]) -> Dict[str, Any]:
        return self._workflow_presenter.build(cameras)

    def _tracking_extrinsics_ready(self) -> bool:
        path = self.latest_extrinsics_path
        return bool(path is not None and path.exists() and path.is_file())

    def find_camera_target(self, camera_id: str) -> Any | None:
        normalized = str(camera_id).strip()
        if not normalized:
            return None
        for target in self.session.discover_targets(None):
            if str(target.camera_id).strip() == normalized:
                return target
        return None

    @staticmethod
    def get_preview_proxy_path(camera_id: str) -> str:
        return f"/api/cameras/{urllib.parse.quote(str(camera_id).strip(), safe='')}/mjpeg"

    def record_preview_proxy_result(self, camera_id: str, error: str | None = None) -> None:
        normalized = str(camera_id).strip()
        if not normalized:
            return
        with self.lock:
            if error:
                self._preview_proxy_errors[normalized] = str(error)
            else:
                self._preview_proxy_errors.pop(normalized, None)

    def get_preview_proxy_error(self, camera_id: str) -> str | None:
        normalized = str(camera_id).strip()
        if not normalized:
            return None
        with self.lock:
            return self._preview_proxy_errors.get(normalized)

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
        metric_section = payload.get("metric", {})
        if not isinstance(metric_section, dict):
            metric_section = {}
        world_section = payload.get("world", {})
        if not isinstance(world_section, dict):
            world_section = {}
        pose_rows = payload.get("pose", {}).get("camera_poses", []) if isinstance(payload.get("pose"), dict) else []
        floor_plane = world_section.get("floor_plane", {})
        if not isinstance(floor_plane, dict):
            floor_plane = {}
        metric_validation = metric_section.get("validation", {})
        if not isinstance(metric_validation, dict):
            metric_validation = {}
        world_validation = world_section.get("validation", {})
        if not isinstance(world_validation, dict):
            world_validation = {}
        self.latest_extrinsics_result = {
            "ok": True,
            "status": "restored",
            "extrinsics_method": payload.get("method"),
            "camera_order": payload.get("camera_order", []),
            "camera_count": len(pose_rows) if isinstance(pose_rows, list) else None,
            "pose_log_path": payload.get("pose_capture_log_path"),
            "output_path": str(path),
            "quality": self.latest_extrinsics_quality,
            "metric_status": metric_section.get("status"),
            "metric": {
                "status": metric_section.get("status"),
                "frame": metric_section.get("frame"),
                "scale_m_per_unit": metric_section.get("scale_m_per_unit"),
                "source": metric_section.get("source"),
                "wand_metric_frames": metric_section.get("wand_metric_frames"),
                "shape_rms_error_mm": metric_section.get("shape_rms_error_mm"),
                "wand_face": metric_section.get("wand_face"),
                "validation": metric_validation,
            },
            "world_status": world_section.get("status"),
            "world": {
                "status": world_section.get("status"),
                "frame": world_section.get("frame"),
                "up_axis": floor_plane.get("axis"),
                "floor_normal": floor_plane.get("normal"),
                "source": world_section.get("source"),
                "origin_world": world_section.get("origin_world"),
                "aligned_axis_world": world_section.get("aligned_axis_world"),
                "wand_face": world_section.get("wand_face"),
                "floor_normal_sign_source": world_section.get("floor_normal_sign_source"),
                "wand_face_alignment": world_section.get("wand_face_alignment"),
                "validation": world_validation,
            },
            "wand_metric_log_path": payload.get("wand_metric_log_path"),
        }

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

    def get_tracking_status(self, cameras: List[Dict[str, Any]] | None = None) -> Dict[str, Any]:
        return self._tracking_service.get_tracking_status(cameras=cameras)

    def get_tracking_scene(self) -> Dict[str, Any]:
        return self._tracking_service.get_tracking_scene()

    def wait_tracking_scene(self, last_sequence: int | None, timeout: float = 15.0) -> Dict[str, Any]:
        return self._tracking_service.wait_tracking_scene(last_sequence, timeout=timeout)

    def tracking_sse_client_opened(self) -> None:
        with self.lock:
            self._tracking_sse_clients += 1

    def tracking_sse_client_closed(self, *, broken_pipe: bool = False) -> None:
        with self.lock:
            self._tracking_sse_clients = max(0, self._tracking_sse_clients - 1)
            if broken_pipe:
                self._tracking_sse_broken_pipes += 1

    def record_tracking_sse_event(self, *, write_ms: float, payload_bytes: int) -> None:
        with self.lock:
            self._tracking_sse_events_sent += 1
            self._tracking_sse_write_ms.append(float(write_ms))
            self._tracking_sse_payload_bytes.append(float(payload_bytes))
            if len(self._tracking_sse_write_ms) > 240:
                self._tracking_sse_write_ms = self._tracking_sse_write_ms[-240:]
            if len(self._tracking_sse_payload_bytes) > 240:
                self._tracking_sse_payload_bytes = self._tracking_sse_payload_bytes[-240:]

    def record_tracking_sse_keepalive(self) -> None:
        with self.lock:
            self._tracking_sse_keepalives += 1

    def get_tracking_sse_diagnostics(self) -> Dict[str, Any]:
        with self.lock:
            write_ms = list(self._tracking_sse_write_ms)
            payload_bytes = list(self._tracking_sse_payload_bytes)
            return {
                "clients": int(self._tracking_sse_clients),
                "events_sent": int(self._tracking_sse_events_sent),
                "keepalives": int(self._tracking_sse_keepalives),
                "broken_pipes": int(self._tracking_sse_broken_pipes),
                "write_ms": _summary_values(write_ms),
                "payload_bytes": _summary_values(payload_bytes),
            }

    def start_tracking(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._tracking_service.start_tracking(payload)

    @staticmethod
    def _all_acked_or_already_running(result: Dict[str, Any]) -> bool:
        for response in result.values():
            if not isinstance(response, dict):
                return False
            if bool(response.get("ack")):
                continue
            error_message = str(response.get("error_message") or response.get("error") or "")
            if "already_running" in error_message:
                continue
            return False
        return True

    def _resolve_tracking_targets(self, payload: Dict[str, Any]) -> List[Any]:
        camera_ids = self._normalize_camera_ids(payload.get("camera_ids"))
        if not camera_ids:
            camera_ids = list(self.selected_camera_ids)
        targets = self._discover_workflow_targets(camera_ids if camera_ids else None)
        if not targets:
            raise ValueError("no target cameras discovered for tracking")
        return targets

    def _prepare_pis_for_tracking(self, targets: List[Any]) -> Dict[str, Any]:
        result = self.session._broadcast(
            targets,
            "set_preview",
            render_enabled=False,
            overlays={"blob": False, "mask": False, "text": False, "charuco": False},
        )
        return {
            "ok": True,
            "camera_ids": [target.camera_id for target in targets],
            "preview_render_enabled": False,
            "overlays": {"blob": False, "mask": False, "text": False, "charuco": False},
            "result": result,
        }

    def stop_tracking(self) -> Dict[str, Any]:
        return self._tracking_service.stop_tracking()

    def _pause_receiver_for_tracking(self) -> None:
        if self._receiver_paused_for_tracking:
            return
        is_running = bool(getattr(self.receiver, "is_running", False))
        stop = getattr(self.receiver, "stop", None)
        if is_running and callable(stop):
            stop()
            self._receiver_paused_for_tracking = True

    def _resume_receiver_after_tracking(self) -> None:
        if not self._receiver_paused_for_tracking:
            return
        start = getattr(self.receiver, "start", None)
        if callable(start):
            start()
        self._receiver_paused_for_tracking = False

    # ── Intrinsics capture ─────────────────────────────────────────────

    def _build_intrinsics_host_session(self, committed: Dict[str, Any]) -> IntrinsicsHostSession:
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
        squares_x = int(committed.get("squares_x", 6))
        squares_y = int(committed.get("squares_y", 8))
        min_frames = int(committed.get("min_frames", 25))
        cooldown_s = float(committed.get("cooldown_s", 1.5))
        mjpeg_url = str(committed.get("mjpeg_url", "")).strip()
        config = IntrinsicsHostSessionConfig(
            camera_id=camera_id,
            mjpeg_url=mjpeg_url,
            square_length_mm=square_length_mm,
            marker_length_mm=marker_length_mm,
            squares_x=squares_x,
            squares_y=squares_y,
            min_frames=min_frames,
            poll_interval_s=cooldown_s,
            output_dir=PROJECT_ROOT / "calibration",
        )
        broadcast_fn = lambda fn, **kw: self._intrinsics_send_command(camera_id, fn, **kw)  # noqa: E731
        return IntrinsicsHostSession(config, broadcast_fn)

    def _ensure_intrinsics_host_session_from_settings(self) -> IntrinsicsHostSession:
        if self._intrinsics_host_session is not None:
            return self._intrinsics_host_session
        bundle = self._load_settings_bundle()
        validation = bundle.get("validation", {}).get("intrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"intrinsics settings invalid: {validation}")
        intrinsics = bundle.get("intrinsics", {})
        committed = intrinsics.get("committed", {}) if isinstance(intrinsics, dict) else {}
        if not isinstance(committed, dict):
            committed = self._default_intrinsics_payload()
        session = self._build_intrinsics_host_session(committed)
        self._intrinsics_host_session = session
        return session

    def start_intrinsics_capture(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._intrinsics_service.start_intrinsics_capture(payload)

    def stop_intrinsics_capture(self) -> Dict[str, Any]:
        return self._intrinsics_service.stop_intrinsics_capture()

    def clear_intrinsics_frames(self) -> Dict[str, Any]:
        return self._intrinsics_service.clear_intrinsics_frames()

    def trigger_intrinsics_calibration(self) -> Dict[str, Any]:
        return self._intrinsics_service.trigger_intrinsics_calibration()

    def get_intrinsics_status(self) -> Dict[str, Any]:
        return self._intrinsics_service.get_intrinsics_status()

    def get_intrinsics_jpeg(self) -> Optional[bytes]:
        return self._intrinsics_service.get_intrinsics_jpeg()

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
            "intrinsics_get_corners",
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
        self._settings_store.persist_intrinsics_payload(payload)

    def _load_intrinsics_settings(self) -> Dict[str, Any]:
        return self._settings_store.load_intrinsics_settings()

    # ── end Intrinsics capture ─────────────────────────────────────────

    def get_state(self) -> Dict[str, Any]:
        cameras = self.refresh_targets()
        bundle = self._load_settings_bundle()
        tracking_status = self.get_tracking_status(cameras=cameras)
        return self._state_presenter.build(cameras=cameras, bundle=bundle, tracking_status=tracking_status)

    def apply_config(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._calibration_config_service.apply_config(payload)

    def run_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._command_service.run_command(payload)

    def generate_extrinsics(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        return self._extrinsics_service.generate_extrinsics(payload)

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
        self._camera_status_store.update_camera_status(result)

    def _on_frame_received(self, frame: Any) -> None:
        self._capture_log_service.on_frame_received(frame)

    def _start_capture_log(self, capture_kind: str) -> Path:
        return self._capture_log_service.start_capture_log(capture_kind)

    def _stop_capture_log(self) -> Dict[str, Any] | None:
        return self._capture_log_service.stop_capture_log()

    def _cancel_capture_timer(self) -> None:
        self._capture_log_service.cancel_capture_timer()

    def _schedule_auto_stop(self, camera_ids: List[str], duration_s: float, capture_kind: str) -> None:
        self._capture_log_service.schedule_auto_stop(camera_ids, duration_s, capture_kind)

    @staticmethod
    def _all_acked(responses: Dict[str, Dict[str, Any]]) -> bool:
        return GuiCaptureLogService.all_acked(responses)


class LoutrackGuiHandler(BaseHTTPRequestHandler):
    state: LoutrackGuiState
    _MJPEG_PROXY_TIMEOUT_S = 2.0
    _MJPEG_PROXY_CHUNK_BYTES = 64 * 1024

    def do_GET(self) -> None:
        path = self.path.split("?", 1)[0]
        if path.startswith("/static/"):
            self._serve_static(path[len("/static/"):])
            return
        if path.startswith("/api/cameras/") and path.endswith("/mjpeg"):
            camera_token = path[len("/api/cameras/"):-len("/mjpeg")]
            camera_id = urllib.parse.unquote(camera_token.strip("/"))
            self._proxy_camera_mjpeg(camera_id)
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
        if path == "/api/tracking/stream":
            self._send_tracking_stream()
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
            self.send_error(HTTPStatus.NOT_FOUND)
        except ValueError as exc:
            error_text = str(exc).strip() or "<empty>"
            self._debug_log(
                f"BAD_REQUEST path={self.path} error={error_text} type={type(exc).__name__} repr={exc!r}"
            )
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

    def _send_text(self, body: str, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _proxy_camera_mjpeg(self, camera_id: str) -> None:
        normalized = str(camera_id).strip()
        if not normalized:
            self._send_text("camera_id is required", status=HTTPStatus.BAD_REQUEST)
            return
        target = self.state.find_camera_target(normalized)
        if target is None:
            self.state.record_preview_proxy_result(normalized, "camera_not_found")
            self._send_text(f"unknown camera_id: {normalized}", status=HTTPStatus.NOT_FOUND)
            return
        upstream_url = f"http://{target.ip}:8555/mjpeg"
        request = urllib.request.Request(upstream_url, headers={"User-Agent": "LoutrackGUI/1.0"})
        try:
            with urllib.request.urlopen(request, timeout=self._MJPEG_PROXY_TIMEOUT_S) as response:
                content_type = str(response.headers.get("Content-Type", "multipart/x-mixed-replace; boundary=frame"))
                self.state.record_preview_proxy_result(normalized, None)
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                while True:
                    try:
                        chunk = response.read(self._MJPEG_PROXY_CHUNK_BYTES)
                    except (TimeoutError, OSError) as exc:
                        self.state.record_preview_proxy_result(
                            normalized,
                            f"upstream_unreachable: {getattr(exc, 'reason', exc)}",
                        )
                        return
                    if not chunk:
                        break
                    try:
                        self.wfile.write(chunk)
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError):
                        self.state.record_preview_proxy_result(normalized, None)
                        return
        except urllib.error.HTTPError as exc:
            self.state.record_preview_proxy_result(normalized, f"upstream_http_{exc.code}")
            self._send_text(
                f"preview upstream failed for {normalized}: http_{exc.code}",
                status=HTTPStatus.BAD_GATEWAY,
            )
        except (BrokenPipeError, ConnectionResetError):
            self.state.record_preview_proxy_result(normalized, None)
        except (urllib.error.URLError, TimeoutError, OSError) as exc:
            reason = getattr(exc, "reason", exc)
            self.state.record_preview_proxy_result(normalized, f"upstream_unreachable: {reason}")
            try:
                self._send_text(
                    f"preview upstream unreachable for {normalized}: {reason}",
                    status=HTTPStatus.BAD_GATEWAY,
                )
            except (BrokenPipeError, ConnectionResetError):
                return

    # Cap SSE scene emission at ~60 Hz so the client renderer (also 60 Hz via
    # requestAnimationFrame) isn't overwhelmed by the pipeline's ~100 Hz pose
    # callback. Intermediate scene updates are coalesced: the latest snapshot
    # is always read when the throttle sleep completes.
    _TRACKING_SSE_MIN_EMIT_INTERVAL_S: float = 1.0 / 60.0

    def _send_tracking_stream(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        last_sequence: int | None = None
        last_emit_monotonic: float = 0.0
        self.state.tracking_sse_client_opened()
        broken_pipe = False
        try:
            while True:
                scene = self.state.wait_tracking_scene(last_sequence, timeout=15.0)
                sequence = int(scene.get("sequence", 0))
                if last_sequence is not None and sequence <= last_sequence:
                    keepalive_started = time.perf_counter()
                    self.wfile.write(b": keepalive\n\n")
                    self.wfile.flush()
                    self.state.record_tracking_sse_keepalive()
                    self.state.record_tracking_sse_event(
                        write_ms=(time.perf_counter() - keepalive_started) * 1000.0,
                        payload_bytes=len(b": keepalive\n\n"),
                    )
                    time.sleep(15.0)
                    continue

                if last_emit_monotonic > 0.0:
                    elapsed = time.monotonic() - last_emit_monotonic
                    if elapsed < self._TRACKING_SSE_MIN_EMIT_INTERVAL_S:
                        time.sleep(self._TRACKING_SSE_MIN_EMIT_INTERVAL_S - elapsed)
                        scene = self.state.get_tracking_scene()
                        sequence = int(scene.get("sequence", 0))
                        if last_sequence is not None and sequence <= last_sequence:
                            last_sequence = sequence
                            continue

                scene_payload = dict(scene)
                scene_payload["sse_emit_realtime_us"] = int(time.time() * 1_000_000)
                scene_payload["sse_emit_monotonic_ms"] = time.monotonic() * 1000.0
                scene_payload["sse_payload_bytes"] = 0
                payload = json.dumps(scene_payload, ensure_ascii=False, separators=(",", ":"))
                base_payload_bytes = len(payload.encode("utf-8"))
                payload_bytes = base_payload_bytes
                while True:
                    next_payload_bytes = base_payload_bytes - 1 + len(str(payload_bytes))
                    if next_payload_bytes == payload_bytes:
                        break
                    payload_bytes = next_payload_bytes
                payload = payload.replace('"sse_payload_bytes":0', f'"sse_payload_bytes":{payload_bytes}', 1)
                message = f"id: {sequence}\nevent: scene\ndata: {payload}\n\n".encode("utf-8")
                write_started = time.perf_counter()
                self.wfile.write(message)
                self.wfile.flush()
                self.state.record_tracking_sse_event(
                    write_ms=(time.perf_counter() - write_started) * 1000.0,
                    payload_bytes=len(message),
                )
                last_sequence = sequence
                last_emit_monotonic = time.monotonic()
        except (BrokenPipeError, ConnectionResetError, TimeoutError):
            broken_pipe = True
            return
        finally:
            self.state.tracking_sse_client_closed(broken_pipe=broken_pipe)

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
