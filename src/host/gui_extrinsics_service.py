from __future__ import annotations

import json
from datetime import datetime, timezone
from collections import Counter
from pathlib import Path
from typing import Any, Dict


from .wand_session import FIXED_PAIR_WINDOW_US


class GuiExtrinsicsService:
    """Extrinsics generation orchestration for the host GUI backend."""

    def __init__(self, owner: Any) -> None:
        self._owner = owner

    def generate_extrinsics(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        owner = self._owner
        owner._assert_calibration_unlocked("extrinsics generation")
        bundle = owner._load_settings_bundle()
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
                    "wand_face",
                )
                if key in payload
            }
            if "log_path" in payload and "pose_log_path" not in extrinsics_patch:
                extrinsics_patch["pose_log_path"] = payload["log_path"]
            if extrinsics_patch:
                bundle = owner._apply_draft_patch(bundle, {"extrinsics": extrinsics_patch})
        validation = bundle.get("validation", {}).get("extrinsics", {})
        if isinstance(validation, dict) and validation:
            raise ValueError(f"extrinsics settings invalid: {validation}")

        committed_extrinsics = bundle.get("extrinsics", {}).get("committed", {})
        if not isinstance(committed_extrinsics, dict):
            committed_extrinsics = owner._default_extrinsics_payload()

        explicit_payload = payload if isinstance(payload, dict) else {}
        extrinsics_method = str(explicit_payload.get("extrinsics_method", "blob_pose_v2")).strip() or "blob_pose_v2"
        runtime_hints = bundle.get("runtime_hints", {})
        if not isinstance(runtime_hints, dict):
            runtime_hints = {}
        default_extrinsics = owner._default_extrinsics_payload()
        intrinsics_raw = str(
            explicit_payload.get("intrinsics_path")
            or default_extrinsics.get("intrinsics_path", "calibration")
        ).strip()
        log_raw = str(
            explicit_payload.get("pose_log_path")
            or explicit_payload.get("log_path")
            or runtime_hints.get("pose_log_path")
            or owner._default_pose_log_path()
        ).strip()
        output_raw = str(
            explicit_payload.get("output_path")
            or owner._default_extrinsics_output_path()
        ).strip()
        intrinsics_path = owner._resolve_project_path(intrinsics_raw, Path("calibration"))
        resolved_log_path = owner._resolve_project_path(log_raw, owner._default_pose_log_path())
        wand_log_raw = str(
            explicit_payload.get("wand_metric_log_path")
            or runtime_hints.get("wand_metric_log_path")
            or owner.wand_metric_log_path
        ).strip()
        resolved_output = owner._resolve_project_path(output_raw, owner._default_extrinsics_output_path())
        resolved_wand_log_path = owner._resolve_project_path(
            wand_log_raw,
            owner._default_wand_metric_log_path(),
        )
        pair_window_us = int(committed_extrinsics.get("pair_window_us", FIXED_PAIR_WINDOW_US))
        min_pairs = int(committed_extrinsics.get("min_pairs", 8))
        wand_pair_window_us = int(committed_extrinsics.get("wand_pair_window_us", FIXED_PAIR_WINDOW_US))
        wand_face = str(committed_extrinsics.get("wand_face", "front_up")).strip().lower() or "front_up"
        if pair_window_us < 1:
            raise ValueError("pair_window_us must be >= 1")
        if pair_window_us > FIXED_PAIR_WINDOW_US:
            raise ValueError(f"pair_window_us must be <= {FIXED_PAIR_WINDOW_US}")
        if min_pairs < 1:
            raise ValueError("min_pairs must be >= 1")
        if wand_pair_window_us < 1:
            raise ValueError("wand_pair_window_us must be >= 1")
        if wand_pair_window_us > FIXED_PAIR_WINDOW_US:
            raise ValueError(f"wand_pair_window_us must be <= {FIXED_PAIR_WINDOW_US}")
        if wand_face not in ("front_up", "back_up"):
            raise ValueError("wand_face must be front_up or back_up")

        if not resolved_log_path.exists():
            summary = self._build_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path does not exist: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            owner.last_result = {"generate_extrinsics": summary}
            return owner.last_result
        if not resolved_log_path.is_file():
            summary = self._build_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=f"log_path is not a file: {resolved_log_path}",
                resolved_wand_log_path=resolved_wand_log_path,
            )
            owner.last_result = {"generate_extrinsics": summary}
            return owner.last_result

        try:
            if extrinsics_method == "blob_pose_v2" and callable(owner._generate_extrinsics_solver):
                solve_fn = owner._generate_extrinsics_solver
            else:
                solve_fn = owner._generate_extrinsics_registry.get(extrinsics_method).solve
            raw_result = solve_fn(
                intrinsics_path=str(intrinsics_path),
                pose_log_path=str(resolved_log_path),
                output_path=str(resolved_output),
                pair_window_us=pair_window_us,
                min_pairs=min_pairs,
                wand_metric_log_path=(
                    str(resolved_wand_log_path)
                    if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file()
                    else None
                ),
                wand_pair_window_us=wand_pair_window_us,
                wand_face=wand_face,
            )
        except (FileNotFoundError, ValueError) as exc:
            reason = str(exc)
            if isinstance(exc, FileNotFoundError):
                missing_path = Path(getattr(exc, "filename", "") or str(resolved_log_path))
                reason = f"log_path does not exist: {missing_path}"
            summary = self._build_failure(
                resolved_log_path=resolved_log_path,
                resolved_output=resolved_output,
                reason=reason,
                resolved_wand_log_path=resolved_wand_log_path,
            )
            owner.last_result = {"generate_extrinsics": summary}
            return owner.last_result
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
        metric_section = raw_result.get("metric", {})
        if not isinstance(metric_section, dict):
            metric_section = {}
        world_section = raw_result.get("world", {})
        if not isinstance(world_section, dict):
            world_section = {}
        pose_log_summary = self._summarize_pose_log(resolved_log_path)

        summary = {
            "ok": True,
            "status": "success",
            "generated_at": datetime.now(timezone.utc).isoformat(),
            "extrinsics_method": extrinsics_method,
            "camera_order": raw_result.get("camera_order", []),
            "camera_count": camera_count,
            "intrinsics_path": str(intrinsics_path),
            "pose_log_path": str(resolved_log_path),
            "output_path": str(resolved_output),
            "parameters": {
                "pair_window_us": pair_window_us,
                "wand_pair_window_us": wand_pair_window_us,
                "min_pairs": min_pairs,
                "wand_face": wand_face,
            },
            "pose_log_summary": pose_log_summary,
            "quality": quality_summary,
            "metric_status": metric_section.get("status"),
            "metric": self._summarize_metric_section(metric_section),
            "world_status": world_section.get("status"),
            "world": self._summarize_world_section(world_section),
            "wand_metric_log_path": (
                str(resolved_wand_log_path)
                if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file()
                else None
            ),
        }
        owner.latest_extrinsics_path = resolved_output
        owner.latest_extrinsics_quality = quality_summary
        owner.latest_extrinsics_result = summary
        owner.last_result = {"generate_extrinsics": summary}
        owner._update_runtime_hints(
            pose_log_path=str(resolved_log_path),
            wand_metric_log_path=str(resolved_wand_log_path),
        )
        return owner.last_result

    @staticmethod
    def _summarize_metric_section(metric_section: Dict[str, Any]) -> Dict[str, Any]:
        validation = metric_section.get("validation", {})
        if not isinstance(validation, dict):
            validation = {}
        return {
            "status": metric_section.get("status"),
            "frame": metric_section.get("frame"),
            "scale_m_per_unit": metric_section.get("scale_m_per_unit"),
            "source": metric_section.get("source"),
            "wand_metric_frames": metric_section.get("wand_metric_frames"),
            "shape_rms_error_mm": metric_section.get("shape_rms_error_mm"),
            "wand_face": metric_section.get("wand_face"),
            "validation": validation,
        }

    @staticmethod
    def _summarize_world_section(world_section: Dict[str, Any]) -> Dict[str, Any]:
        validation = world_section.get("validation", {})
        if not isinstance(validation, dict):
            validation = {}
        floor_plane = world_section.get("floor_plane", {})
        if not isinstance(floor_plane, dict):
            floor_plane = {}
        return {
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
            "validation": validation,
        }

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

    def _build_failure(
        self,
        *,
        resolved_log_path: Path,
        resolved_output: Path,
        reason: str,
        resolved_wand_log_path: Path,
    ) -> Dict[str, Any]:
        pose_log_summary = (
            self._summarize_pose_log(resolved_log_path)
            if resolved_log_path.exists() and resolved_log_path.is_file()
            else {
                "exists": resolved_log_path.exists(),
                "is_file": resolved_log_path.is_file(),
                "rows_by_camera": {},
                "single_blob_rows_by_camera": {},
                "usable_camera_count": 0,
                "invalid_rows": 0,
            }
        )
        return {
            "ok": False,
            "status": "failed",
            "output_path": str(resolved_output),
            "error": reason,
            "pose_log_path": str(resolved_log_path),
            "pose_log_summary": pose_log_summary,
            "wand_metric_log_path": (
                str(resolved_wand_log_path)
                if resolved_wand_log_path.exists() and resolved_wand_log_path.is_file()
                else None
            ),
            "metric_status": "skipped",
            "metric": {"status": "skipped", "reason": "generation_failed"},
            "world_status": "skipped",
            "world": {"status": "skipped", "reason": "generation_failed"},
        }
