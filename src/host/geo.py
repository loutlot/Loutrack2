"""
Geometry module for 3D reconstruction from multi-camera 2D observations.

Provides functionality to:
- Load and manage camera intrinsic/extrinsic parameters
- Triangulate 3D points from 2D observations (DLT with lens-distortion correction)
- Compute reprojection errors for quality assessment
- Epipolar-based correspondence matching across cameras
"""

import json
import math
import re
import time
import warnings
import numpy as np
import cv2 as cv
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple, Callable
from dataclasses import dataclass, field
from scipy.optimize import linear_sum_assignment
from itertools import combinations


_CANONICAL_INTRINSICS_FILENAME_RE = re.compile(
    r"^calibration_intrinsics_v1_(?P<camera_id>[A-Za-z0-9][A-Za-z0-9_\-]*)\.json$"
)


@dataclass
class CameraParams:
    """Camera intrinsic and extrinsic parameters."""
    camera_id: str
    intrinsic_matrix: np.ndarray  # 3x3
    distortion_coeffs: np.ndarray  # 5+ coefficients
    rotation: np.ndarray  # 3x3 rotation matrix (extrinsic)
    translation: np.ndarray  # 3x1 translation vector (extrinsic)
    resolution: Tuple[int, int] = (1280, 960)  # width, height
    focal_scale: float = 1.0

    @property
    def fx(self) -> float:
        return self.intrinsic_matrix[0, 0]

    @property
    def fy(self) -> float:
        return self.intrinsic_matrix[1, 1]

    @property
    def cx(self) -> float:
        return self.intrinsic_matrix[0, 2]

    @property
    def cy(self) -> float:
        return self.intrinsic_matrix[1, 2]

    @property
    def projection_matrix(self) -> np.ndarray:
        """Projection matrix P = K @ [R|t]."""
        RT = np.hstack([self.rotation, self.translation.reshape(3, 1)])
        return self.intrinsic_matrix @ RT


class CalibrationLoader:
    """
    Load camera calibration data from JSON files.

    Supports loading from loutrack2's calibration_intrinsics_v1.json format
    and converting from reference implementation's camera-params.json format.
    """

    @staticmethod
    def load_intrinsics(filepath: str) -> Dict[str, Dict[str, Any]]:
        """Load intrinsic calibration from loutrack2 schema format.

        In directory mode only files whose name matches the canonical
        `calibration_intrinsics_v1_<camera_id>.json` pattern are read,
        and the camera_id embedded in the filename must match the JSON
        content. Non-canonical files (backups like `*.json copy.json`,
        scratch files, stale duplicates with the same content camera_id)
        are skipped so the loader is deterministic regardless of filesystem
        iteration order.
        """
        path = Path(filepath)

        if path.is_dir():
            calibrations: Dict[str, Dict[str, Any]] = {}
            for f in sorted(path.glob("*.json")):
                match = _CANONICAL_INTRINSICS_FILENAME_RE.match(f.name)
                try:
                    with open(f, "r", encoding="utf-8") as handle:
                        data = json.load(handle)
                except Exception:
                    continue
                if not isinstance(data, dict):
                    continue
                if "camera_id" not in data or "camera_matrix" not in data:
                    continue
                if match is None:
                    warnings.warn(
                        f"ignoring non-canonical intrinsics file {f.name!r}: "
                        f"expected name 'calibration_intrinsics_v1_<camera_id>.json'",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    continue
                filename_camera_id = match.group("camera_id")
                content_camera_id = str(data["camera_id"])
                if filename_camera_id != content_camera_id:
                    warnings.warn(
                        f"ignoring intrinsics file {f.name!r}: filename camera_id "
                        f"{filename_camera_id!r} does not match content camera_id "
                        f"{content_camera_id!r}",
                        RuntimeWarning,
                        stacklevel=2,
                    )
                    continue
                calibrations[content_camera_id] = data
            return calibrations

        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)

        if isinstance(data, list):
            return {d.get("camera_id", f"cam_{i}"): d for i, d in enumerate(data)}
        elif "camera_id" in data:
            return {data["camera_id"]: data}
        else:
            return {"default": data}

    @staticmethod
    def to_camera_params(calibration: Dict[str, Any]) -> "CameraParams":
        """Convert calibration JSON to CameraParams object."""
        cam_matrix = calibration.get("camera_matrix", {})
        if "matrix" in cam_matrix:
            K = np.array(cam_matrix["matrix"], dtype=np.float64)
        else:
            fx = cam_matrix.get("fx", 500.0)
            fy = cam_matrix.get("fy", 500.0)
            cx = cam_matrix.get("cx", 640.0)
            cy = cam_matrix.get("cy", 480.0)
            K = np.array(
                [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64
            )

        dist_data = calibration.get("distortion_coefficients", {})
        if "array" in dist_data:
            dist = np.array(dist_data["array"], dtype=np.float64)
        else:
            k1 = dist_data.get("k1", 0.0)
            k2 = dist_data.get("k2", 0.0)
            p1 = dist_data.get("p1", 0.0)
            p2 = dist_data.get("p2", 0.0)
            k3 = dist_data.get("k3", 0.0)
            dist = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

        res = calibration.get("resolution", {"width": 1280, "height": 960})
        resolution = (res.get("width", 1280), res.get("height", 960))

        return CameraParams(
            camera_id=calibration.get("camera_id", "unknown"),
            intrinsic_matrix=K,
            distortion_coeffs=dist,
            rotation=np.eye(3, dtype=np.float64),
            translation=np.zeros(3, dtype=np.float64),
            resolution=resolution,
        )

    @staticmethod
    def load_extrinsics(
        filepath: str, include_meta: bool = False
    ) -> Dict[str, Dict[str, Any]]:
        path = Path(filepath)
        extrinsics_path: Optional[Path] = None

        if path.is_dir():
            matches = sorted(path.glob("extrinsics_pose_v2*.json"))
            if matches:
                extrinsics_path = matches[0]
        elif path.name.startswith("extrinsics_pose_v2"):
            extrinsics_path = path

        if extrinsics_path is None or not extrinsics_path.exists():
            return {}

        with open(extrinsics_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)

        cameras = CalibrationLoader._select_v2_camera_pose_rows(data)
        result: Dict[str, Dict[str, Any]] = {}
        for camera in cameras:
            camera_id = camera.get("camera_id")
            if camera_id:
                result[camera_id] = camera
        if include_meta:
            result["__meta__"] = data if isinstance(data, dict) else {}
        return result

    @staticmethod
    def _select_v2_camera_pose_rows(data: Dict[str, Any]) -> List[Dict[str, Any]]:
        metric_section = data.get("metric", {})
        world_section = data.get("world", {})
        if (
            isinstance(metric_section, dict)
            and metric_section.get("status") == "resolved"
            and isinstance(metric_section.get("camera_poses"), list)
        ):
            metric_rows = [
                dict(row)
                for row in metric_section.get("camera_poses", [])
                if isinstance(row, dict)
            ]
            world_matrix = CalibrationLoader._v2_world_transform(world_section)
            if world_matrix is not None:
                return [
                    CalibrationLoader._transform_metric_pose_to_world(row, world_matrix)
                    for row in metric_rows
                ]
            return metric_rows

        pose_section = data.get("pose", {})
        if not isinstance(pose_section, dict):
            return []
        return [
            dict(row)
            for row in pose_section.get("camera_poses", [])
            if isinstance(row, dict)
        ]

    @staticmethod
    def _v2_world_transform(world_section: Any) -> np.ndarray | None:
        if not isinstance(world_section, dict) or world_section.get("status") != "resolved":
            return None
        raw_matrix = world_section.get("to_world_matrix")
        if raw_matrix is None:
            return None
        try:
            matrix = np.asarray(raw_matrix, dtype=np.float64).reshape(4, 4)
        except Exception:
            return None
        if not np.all(np.isfinite(matrix)):
            return None
        return matrix

    @staticmethod
    def _transform_metric_pose_to_world(
        row: Dict[str, Any],
        to_world_matrix: np.ndarray,
    ) -> Dict[str, Any]:
        rotation = np.asarray(row.get("R", np.eye(3)), dtype=np.float64).reshape(3, 3)
        translation = np.asarray(row.get("t", [0.0, 0.0, 0.0]), dtype=np.float64).reshape(3)
        metric_to_world_r = to_world_matrix[:3, :3]
        metric_to_world_t = to_world_matrix[:3, 3]

        world_to_metric_r = metric_to_world_r.T
        rotation_world = rotation @ world_to_metric_r
        translation_world = translation - rotation_world @ metric_to_world_t

        out = dict(row)
        out["R"] = rotation_world.tolist()
        out["t"] = translation_world.tolist()
        out["frame"] = "world"
        out["source_frame"] = "metric_camera"
        out["coordinate_origin"] = "wand"
        return out

    @staticmethod
    def apply_extrinsics(
        camera_params: Dict[str, "CameraParams"],
        extrinsics: Dict[str, Dict[str, Any]],
        apply_world_transform: bool = True,
    ) -> None:
        for camera_id, extrinsic in extrinsics.items():
            if str(camera_id).startswith("__"):
                continue
            if camera_id not in camera_params:
                continue
            focal_scale = CalibrationLoader._validate_focal_scale(
                extrinsic.get("focal_scale", 1.0),
                str(camera_id),
            )
            rotation = np.array(
                extrinsic.get("R", np.eye(3)), dtype=np.float64
            )
            translation = np.array(
                extrinsic.get("t", [0.0, 0.0, 0.0]), dtype=np.float64
            )
            camera = camera_params[camera_id]
            previous_scale = CalibrationLoader._validate_focal_scale(
                getattr(camera, "focal_scale", 1.0),
                str(camera_id),
            )
            camera.intrinsic_matrix[0, 0] = (
                float(camera.intrinsic_matrix[0, 0]) / previous_scale * focal_scale
            )
            camera.intrinsic_matrix[1, 1] = (
                float(camera.intrinsic_matrix[1, 1]) / previous_scale * focal_scale
            )
            camera.focal_scale = focal_scale
            camera.rotation = rotation.reshape(3, 3)
            camera.translation = translation.reshape(3)

    @staticmethod
    def _validate_focal_scale(value: Any, camera_id: str) -> float:
        try:
            focal_scale = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"invalid focal_scale for {camera_id}: {value!r}") from exc
        if not np.isfinite(focal_scale) or focal_scale <= 0.0:
            raise ValueError(f"invalid focal_scale for {camera_id}: {value!r}")
        return focal_scale

    @staticmethod
    def load_from_reference_format(filepath: str) -> Dict[str, "CameraParams"]:
        """Load from reference implementation's camera-params.json format."""
        with open(filepath, "r") as f:
            data = json.load(f)

        params = {}
        for i, cam_data in enumerate(data):
            K = np.array(cam_data["intrinsic_matrix"], dtype=np.float64)
            dist = np.array(cam_data["distortion_coef"], dtype=np.float64)
            params[f"cam_{i}"] = CameraParams(
                camera_id=f"cam_{i}",
                intrinsic_matrix=K,
                distortion_coeffs=dist,
                rotation=np.eye(3, dtype=np.float64),
                translation=np.zeros(3, dtype=np.float64),
            )

        return params


# --------------------------------------------------------------------------- #
# Epipolar and triangulation thresholds are tuned for precision-first live
# tracking on the host: prefer dropping ambiguous points over accepting
# low-quality reconstructions that would later destabilize rigid solving.
# --------------------------------------------------------------------------- #
_EPIPOLAR_THRESHOLD_PX: float = 3.5
_EPIPOLAR_THRESHOLD_PX_MIN: float = 1.0
_EPIPOLAR_THRESHOLD_PX_MAX: float = 6.0
_EPIPOLAR_THRESHOLD_PX_STEP: float = 0.5
_MAX_REPROJECTION_ERROR_PX: float = 2.5
_P90_REPROJECTION_ERROR_PX: float = 1.5
_MIN_TRIANGULATION_ANGLE_DEG: float = 1.5
_INVALID_ASSIGNMENT_COST: float = 1e9


def normalize_epipolar_threshold_px(value: Optional[Any]) -> float:
    """Normalize GUI/API epipolar gate input to the supported slider range."""
    if value is None:
        return float(_EPIPOLAR_THRESHOLD_PX)
    try:
        threshold = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("epipolar_threshold_px must be numeric") from exc
    if not np.isfinite(threshold):
        raise ValueError("epipolar_threshold_px must be finite")
    threshold = min(_EPIPOLAR_THRESHOLD_PX_MAX, max(_EPIPOLAR_THRESHOLD_PX_MIN, threshold))
    steps = math.floor((threshold / _EPIPOLAR_THRESHOLD_PX_STEP) + 0.5)
    return float(steps * _EPIPOLAR_THRESHOLD_PX_STEP)


@dataclass(frozen=True)
class BlobObservation2D:
    """A 2D blob observation with both raw and undistorted pixel coordinates."""

    camera_id: str
    raw_uv: Tuple[float, float]
    undistorted_uv: Tuple[float, float]
    area: float = 0.0
    blob_index: int = -1

    def to_dict(self) -> Dict[str, Any]:
        return {
            "camera_id": self.camera_id,
            "blob_index": int(self.blob_index),
            "raw_uv": [float(self.raw_uv[0]), float(self.raw_uv[1])],
            "undistorted_uv": [
                float(self.undistorted_uv[0]),
                float(self.undistorted_uv[1]),
            ],
            "area": float(self.area),
        }


@dataclass(frozen=True)
class TriangulatedPoint:
    """A 3D point plus the 2D observations and quality metrics that created it."""

    point: np.ndarray
    observations: Tuple[BlobObservation2D, ...]
    reprojection_errors_px: Tuple[float, ...]
    epipolar_errors_px: Tuple[float, ...]
    triangulation_angles_deg: Tuple[float, ...]
    source: str = "generic"
    rigid_name: Optional[str] = None
    marker_idx: Optional[int] = None
    is_virtual: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "point": [float(value) for value in np.asarray(self.point, dtype=np.float64).reshape(3)],
            "observations": [observation.to_dict() for observation in self.observations],
            "camera_ids": [observation.camera_id for observation in self.observations],
            "blob_indices": [int(observation.blob_index) for observation in self.observations],
            "contributing_rays": int(len(self.observations)),
            "reprojection_errors_px": [float(value) for value in self.reprojection_errors_px],
            "epipolar_errors_px": [float(value) for value in self.epipolar_errors_px],
            "triangulation_angles_deg": [float(value) for value in self.triangulation_angles_deg],
            "source": self.source,
            "rigid_name": self.rigid_name,
            "marker_idx": self.marker_idx,
            "is_virtual": bool(self.is_virtual),
        }


_BlobObservation = BlobObservation2D


def _metric_summary(values: List[float]) -> Dict[str, float]:
    if not values:
        return {
            "count": 0,
            "mean": 0.0,
            "median": 0.0,
            "p90": 0.0,
            "p95": 0.0,
            "min": 0.0,
            "max": 0.0,
        }
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "median": float(np.percentile(arr, 50)),
        "p90": float(np.percentile(arr, 90)),
        "p95": float(np.percentile(arr, 95)),
        "min": float(arr.min()),
        "max": float(arr.max()),
    }


def _increment_diagnostic(diagnostics: Dict[str, float], *keys: str, amount: float = 1.0) -> None:
    for key in keys:
        diagnostics[key] = float(diagnostics.get(key, 0.0)) + float(amount)


def _empty_assignment_diagnostics() -> Dict[str, float]:
    return {
        "assignment_candidates": 0,
        "assignment_matches": 0,
        "assignment_rejected_epipolar": 0,
        "assignment_rejected_triangulation": 0,
        "assignment_rejected_reprojection": 0,
        "assignment_rejected_low_parallax": 0,
        "duplicate_blob_matches": 0,
        "assignment_cost_ms": 0.0,
        "full_candidate_pair_count": 0,
        "pruned_candidate_pair_count": 0,
        "candidate_reduction_ratio": 0.0,
        "component_count": 0,
        "largest_component_rows": 0,
        "largest_component_cols": 0,
        "pruning_fallback_count": 0,
        "pruning_fallback_reason_counts": {},
        "pruned_assignment_cost_ms": 0.0,
        "epipolar_pruning_summary": {},
        "triangulated_pairs": 0,
        "rejected_epipolar": 0,
        "rejected_triangulation": 0,
        "rejected_reprojection": 0,
        "rejected_low_parallax": 0,
        "dropped_views_for_inlier_fit": 0,
    }


class Triangulator:
    """
    Triangulate 3D points from 2D observations across multiple cameras.

    Matching runs in the ideal pinhole image plane, while triangulation and
    reprojection checks keep the original distorted observations around for
    diagnostics and residual-based inlier selection.
    """

    def __init__(
        self,
        camera_params: Dict[str, CameraParams],
        epipolar_threshold_px: float = _EPIPOLAR_THRESHOLD_PX,
        *,
        fast_geometry: bool = False,
        epipolar_pruning_enabled: bool = False,
        stage_callback: Optional[Callable[[str, float], None]] = None,
    ):
        self.camera_params = camera_params
        self.epipolar_threshold_px = float(epipolar_threshold_px)
        self.fast_geometry = bool(fast_geometry)
        self.epipolar_pruning_enabled = bool(epipolar_pruning_enabled)
        self._stage_callback = stage_callback
        self._proj_matrices: Dict[str, np.ndarray] = {}
        self._rvecs: Dict[str, np.ndarray] = {}
        self._camera_centers: Dict[str, np.ndarray] = {}
        self._fundamental_matrices: Dict[Tuple[str, str], np.ndarray] = {}
        self._last_assignment_diagnostics: Dict[str, float] = _empty_assignment_diagnostics()
        self._last_quality_metrics: Dict[str, Any] = self._empty_quality_metrics(
            self._last_assignment_diagnostics
        )
        self._last_observations_by_camera: Dict[str, List[BlobObservation2D]] = {}
        self._last_triangulated_points: List[TriangulatedPoint] = []
        self._refresh_cached_matrices()

    @staticmethod
    def _record_pruning_fallback(diagnostics: Dict[str, Any], reason: str) -> None:
        _increment_diagnostic(diagnostics, "pruning_fallback_count")
        reasons = diagnostics.setdefault("pruning_fallback_reason_counts", {})
        if isinstance(reasons, dict):
            key = str(reason or "unknown")
            reasons[key] = int(reasons.get(key, 0)) + 1

    @staticmethod
    def _finalize_epipolar_pruning_summary(diagnostics: Dict[str, Any]) -> None:
        full_count = int(diagnostics.get("full_candidate_pair_count", 0) or 0)
        pruned_count = int(diagnostics.get("pruned_candidate_pair_count", 0) or 0)
        reduction = (
            max(0.0, 1.0 - float(pruned_count) / float(full_count))
            if full_count > 0
            else 0.0
        )
        diagnostics["candidate_reduction_ratio"] = float(reduction)
        diagnostics["epipolar_pruning_summary"] = {
            "full_candidate_pair_count": full_count,
            "pruned_candidate_pair_count": pruned_count,
            "candidate_reduction_ratio": float(reduction),
            "component_count": int(diagnostics.get("component_count", 0) or 0),
            "largest_component_rows": int(diagnostics.get("largest_component_rows", 0) or 0),
            "largest_component_cols": int(diagnostics.get("largest_component_cols", 0) or 0),
            "pruning_fallback_count": int(diagnostics.get("pruning_fallback_count", 0) or 0),
            "pruning_fallback_reason_counts": dict(
                diagnostics.get("pruning_fallback_reason_counts", {}) or {}
            ),
            "pruned_assignment_cost_ms": float(
                diagnostics.get("pruned_assignment_cost_ms", 0.0) or 0.0
            ),
        }

    def _record_stage_detail(self, name: str, started_ns: int) -> None:
        if self._stage_callback is None:
            return
        self._stage_callback(name, float(time.perf_counter_ns() - started_ns) / 1_000_000.0)

    @property
    def last_assignment_diagnostics(self) -> Dict[str, float]:
        """Return diagnostics from the most recent blob assignment pass."""
        return dict(self._last_assignment_diagnostics)

    @property
    def last_quality_metrics(self) -> Dict[str, Any]:
        """Return quality metrics from the most recent triangulation pass."""
        metrics = self._last_quality_metrics
        contributing = metrics.get("contributing_rays", {})
        return {
            "accepted_points": int(metrics.get("accepted_points", 0)),
            "contributing_rays": {
                "per_point": list(contributing.get("per_point", [])),
                "summary": dict(contributing.get("summary", {})),
            },
            "reprojection_error_px_summary": dict(
                metrics.get("reprojection_error_px_summary", {})
            ),
            "epipolar_error_px_summary": dict(
                metrics.get("epipolar_error_px_summary", {})
            ),
            "triangulation_angle_deg_summary": dict(
                metrics.get("triangulation_angle_deg_summary", {})
            ),
            "assignment_diagnostics": dict(
                metrics.get("assignment_diagnostics", {})
            ),
        }

    @property
    def last_observations_by_camera(self) -> Dict[str, List[Dict[str, Any]]]:
        """Return serialized 2D observations from the most recent frame pair."""
        return {
            camera_id: [observation.to_dict() for observation in observations]
            for camera_id, observations in self._last_observations_by_camera.items()
        }

    @property
    def last_triangulated_points(self) -> List[Dict[str, Any]]:
        """Return serialized triangulated points with observation provenance."""
        return [point.to_dict() for point in self._last_triangulated_points]

    @staticmethod
    def _empty_quality_metrics(assignment_diagnostics: Optional[Dict[str, float]] = None) -> Dict[str, Any]:
        return {
            "accepted_points": 0,
            "contributing_rays": {"per_point": [], "summary": _metric_summary([])},
            "reprojection_error_px_summary": _metric_summary([]),
            "epipolar_error_px_summary": _metric_summary([]),
            "triangulation_angle_deg_summary": _metric_summary([]),
            "assignment_diagnostics": dict(assignment_diagnostics or _empty_assignment_diagnostics()),
        }

    @staticmethod
    def _empty_rigid_hint_quality(reason: str = "") -> Dict[str, Any]:
        return {
            "reason": str(reason),
            "rigid_count": 0,
            "candidate_markers": 0,
            "markers_with_two_or_more_rays": 0,
            "single_ray_candidates": 0,
            "accepted_points": 0,
            "rejected_markers": 0,
            "invalid_assignments": 0,
            "by_rigid": {},
            "contributing_rays": {"per_point": [], "summary": _metric_summary([])},
            "reprojection_error_px_summary": _metric_summary([]),
            "triangulation_angle_deg_summary": _metric_summary([]),
            "assignment_diagnostics": _empty_assignment_diagnostics(),
        }

    # ---------------------------------------------------------------------- #
    # Cache management
    # ---------------------------------------------------------------------- #

    def _refresh_cached_matrices(self) -> None:
        """Precompute per-camera matrices used across matching and triangulation."""
        self._proj_matrices = {}
        self._rvecs = {}
        self._camera_centers = {}
        self._fundamental_matrices = {}
        for cam_id, cam in self.camera_params.items():
            RT = np.hstack([cam.rotation, cam.translation.reshape(3, 1)])
            self._proj_matrices[cam_id] = cam.intrinsic_matrix @ RT
            rvec, _ = cv.Rodrigues(cam.rotation)
            self._rvecs[cam_id] = rvec
            self._camera_centers[cam_id] = -cam.rotation.T @ cam.translation

    # ---------------------------------------------------------------------- #
    # Observation handling and core triangulation
    # ---------------------------------------------------------------------- #

    def _undistort_point(self, cam_id: str, point: Tuple[float, float]) -> Tuple[float, float]:
        cam = self.camera_params[cam_id]
        pts_arr = np.array([[[point[0], point[1]]]], dtype=np.float64)
        undistorted = cv.undistortPoints(
            pts_arr,
            cam.intrinsic_matrix,
            cam.distortion_coeffs,
            P=cam.intrinsic_matrix,
        )
        return float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])

    def _build_observations(
        self,
        cam_id: str,
        blobs: List[Dict[str, Any]],
        include_blob_indices: Optional[set[int]] = None,
    ) -> List[_BlobObservation]:
        observations: List[_BlobObservation] = []
        if include_blob_indices is not None and not include_blob_indices:
            return observations

        selected: List[Tuple[int, Dict[str, Any]]] = [
            (blob_index, blob)
            for blob_index, blob in enumerate(blobs)
            if include_blob_indices is None or blob_index in include_blob_indices
        ]
        if not selected:
            return observations

        if self.fast_geometry:
            started_ns = time.perf_counter_ns()
            cam = self.camera_params[cam_id]
            raw_points = np.asarray(
                [[float(blob["x"]), float(blob["y"])] for _idx, blob in selected],
                dtype=np.float64,
            ).reshape(-1, 1, 2)
            undistorted = cv.undistortPoints(
                raw_points,
                cam.intrinsic_matrix,
                cam.distortion_coeffs,
                P=cam.intrinsic_matrix,
            ).reshape(-1, 2)
            self._record_stage_detail("undistort_ms", started_ns)
            for row_index, (blob_index, blob) in enumerate(selected):
                raw_uv = (float(raw_points[row_index, 0, 0]), float(raw_points[row_index, 0, 1]))
                observations.append(
                    _BlobObservation(
                        camera_id=cam_id,
                        raw_uv=raw_uv,
                        undistorted_uv=(
                            float(undistorted[row_index, 0]),
                            float(undistorted[row_index, 1]),
                        ),
                        area=float(blob.get("area", 0.0)),
                        blob_index=int(blob_index),
                    )
                )
            return observations

        for blob_index, blob in enumerate(blobs):
            if include_blob_indices is not None and blob_index not in include_blob_indices:
                continue
            raw_uv = (float(blob["x"]), float(blob["y"]))
            observations.append(
                _BlobObservation(
                    camera_id=cam_id,
                    raw_uv=raw_uv,
                    undistorted_uv=self._undistort_point(cam_id, raw_uv),
                    area=float(blob.get("area", 0.0)),
                    blob_index=int(blob_index),
                )
            )
        return observations

    @staticmethod
    def _epipolar_distance_matrix(
        F: np.ndarray,
        source_points: List[Tuple[float, float]],
        target_points: List[Tuple[float, float]],
    ) -> np.ndarray:
        if not source_points or not target_points:
            return np.empty((len(source_points), len(target_points)), dtype=np.float64)
        source_h = np.column_stack(
            [
                np.asarray([pt[0] for pt in source_points], dtype=np.float64),
                np.asarray([pt[1] for pt in source_points], dtype=np.float64),
                np.ones(len(source_points), dtype=np.float64),
            ]
        )
        target_h = np.column_stack(
            [
                np.asarray([pt[0] for pt in target_points], dtype=np.float64),
                np.asarray([pt[1] for pt in target_points], dtype=np.float64),
                np.ones(len(target_points), dtype=np.float64),
            ]
        )
        lines = source_h @ F.T
        denom = np.sqrt(lines[:, 0] ** 2 + lines[:, 1] ** 2)
        numer = np.abs(lines @ target_h.T)
        distances = numer / np.maximum(denom[:, None], 1e-12)
        distances[denom < 1e-12, :] = float("inf")
        return distances

    @staticmethod
    def _line_rectangle_segment(
        line: np.ndarray,
        target_points: np.ndarray,
        margin_px: float,
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        a, b, c = (float(line[0]), float(line[1]), float(line[2]))
        if abs(a) < 1e-12 and abs(b) < 1e-12:
            return None
        x_min = float(np.min(target_points[:, 0]) - margin_px)
        x_max = float(np.max(target_points[:, 0]) + margin_px)
        y_min = float(np.min(target_points[:, 1]) - margin_px)
        y_max = float(np.max(target_points[:, 1]) + margin_px)
        intersections: List[Tuple[float, float]] = []
        if abs(b) >= 1e-12:
            for x in (x_min, x_max):
                y = -(a * x + c) / b
                if y_min - 1e-9 <= y <= y_max + 1e-9:
                    intersections.append((x, y))
        if abs(a) >= 1e-12:
            for y in (y_min, y_max):
                x = -(b * y + c) / a
                if x_min - 1e-9 <= x <= x_max + 1e-9:
                    intersections.append((x, y))
        unique: List[Tuple[float, float]] = []
        for point in intersections:
            if not any(np.linalg.norm(np.asarray(point) - np.asarray(other)) < 1e-6 for other in unique):
                unique.append(point)
        if len(unique) < 2:
            return None
        best_pair = (np.asarray(unique[0], dtype=np.float64), np.asarray(unique[1], dtype=np.float64))
        best_distance = -1.0
        for i in range(len(unique)):
            for j in range(i + 1, len(unique)):
                distance = float(np.linalg.norm(np.asarray(unique[i]) - np.asarray(unique[j])))
                if distance > best_distance:
                    best_distance = distance
                    best_pair = (
                        np.asarray(unique[i], dtype=np.float64),
                        np.asarray(unique[j], dtype=np.float64),
                    )
        return best_pair

    def _epipolar_candidate_pairs(
        self,
        F_ref_to_other: np.ndarray,
        ref_points: List[Tuple[float, float]],
        other_points: List[Tuple[float, float]],
        gate_px: float,
    ) -> Optional[set[Tuple[int, int]]]:
        if not ref_points or not other_points:
            return set()
        forward_matrix = self._epipolar_distance_matrix(
            F_ref_to_other,
            ref_points,
            other_points,
        )
        rows, cols = np.nonzero(forward_matrix <= float(gate_px))
        return {(int(row), int(col)) for row, col in zip(rows, cols)}

    @staticmethod
    def _connected_components_from_edges(
        edges: Dict[Tuple[int, int], Tuple[float, float, float]],
    ) -> List[Tuple[List[int], List[int]]]:
        rows_by_col: Dict[int, set[int]] = {}
        cols_by_row: Dict[int, set[int]] = {}
        for row, col in edges:
            rows_by_col.setdefault(int(col), set()).add(int(row))
            cols_by_row.setdefault(int(row), set()).add(int(col))
        components: List[Tuple[List[int], List[int]]] = []
        seen_rows: set[int] = set()
        seen_cols: set[int] = set()
        for start_row in sorted(cols_by_row):
            if start_row in seen_rows:
                continue
            stack_rows = [start_row]
            component_rows: set[int] = set()
            component_cols: set[int] = set()
            while stack_rows:
                row = stack_rows.pop()
                if row in component_rows:
                    continue
                component_rows.add(row)
                seen_rows.add(row)
                for col in cols_by_row.get(row, set()):
                    if col in component_cols:
                        continue
                    component_cols.add(col)
                    seen_cols.add(col)
                    for next_row in rows_by_col.get(col, set()):
                        if next_row not in component_rows:
                            stack_rows.append(next_row)
            components.append((sorted(component_rows), sorted(component_cols)))
        for start_col in sorted(rows_by_col):
            if start_col in seen_cols:
                continue
            components.append((sorted(rows_by_col[start_col]), [start_col]))
        return components

    def _triangulate_from_undistorted_points(
        self,
        image_points: List[Tuple[float, float]],
        camera_ids: List[str],
    ) -> Optional[np.ndarray]:
        if len(image_points) < 2:
            return None

        valid_points: List[Tuple[float, float]] = []
        valid_cameras: List[str] = []
        for pt, cam_id in zip(image_points, camera_ids):
            if pt is None or cam_id not in self.camera_params:
                continue
            valid_points.append((float(pt[0]), float(pt[1])))
            valid_cameras.append(cam_id)

        if len(valid_points) < 2:
            return None

        A = []
        for pt, cam_id in zip(valid_points, valid_cameras):
            P = self._proj_matrices[cam_id]
            u, v = pt
            A.append(v * P[2, :] - P[1, :])
            A.append(P[0, :] - u * P[2, :])

        try:
            _, _, Vh = np.linalg.svd(np.asarray(A, dtype=np.float64), full_matrices=True)
        except np.linalg.LinAlgError:
            return None

        X_hom = Vh[-1, :]
        if abs(float(X_hom[3])) < 1e-10:
            return None

        point_3d = X_hom[:3] / X_hom[3]
        for cam_id in valid_cameras:
            cam = self.camera_params[cam_id]
            depth = float(cam.rotation[2, :] @ point_3d + cam.translation[2])
            if depth <= 0.0:
                return None
        return point_3d

    def triangulate_point(
        self,
        image_points: List[Tuple[float, float]],
        camera_ids: List[str],
    ) -> Optional[np.ndarray]:
        """
        Triangulate a single 3D point from distorted pixel observations.
        """
        if len(image_points) < 2:
            return None

        undistorted: List[Tuple[float, float]] = []
        valid_cameras: List[str] = []
        for pt, cam_id in zip(image_points, camera_ids):
            if pt is None or cam_id not in self.camera_params:
                continue
            undistorted.append(self._undistort_point(cam_id, pt))
            valid_cameras.append(cam_id)
        return self._triangulate_from_undistorted_points(undistorted, valid_cameras)

    def _triangulate_from_observations(
        self,
        observations: List[_BlobObservation],
    ) -> Optional[np.ndarray]:
        return self._triangulate_from_undistorted_points(
            [obs.undistorted_uv for obs in observations],
            [obs.camera_id for obs in observations],
        )

    # ---------------------------------------------------------------------- #
    # Reprojection error
    # ---------------------------------------------------------------------- #

    def compute_reprojection_errors(
        self,
        image_points: List[Tuple[float, float]],
        object_point: np.ndarray,
        camera_ids: List[str],
    ) -> List[float]:
        """Compute per-view reprojection residuals in distorted pixel space."""
        errors: List[float] = []
        obj_pts = object_point.reshape(1, 3).astype(np.float32)

        for pt, cam_id in zip(image_points, camera_ids):
            if pt is None or cam_id not in self.camera_params:
                continue
            cam = self.camera_params[cam_id]
            rvec = self._rvecs.get(cam_id)
            if rvec is None:
                continue
            try:
                projected, _ = cv.projectPoints(
                    obj_pts,
                    rvec,
                    cam.translation,
                    cam.intrinsic_matrix,
                    cam.distortion_coeffs,
                )
                observed = np.array(pt, dtype=np.float64)
                errors.append(float(np.linalg.norm(observed - projected.squeeze())))
            except Exception:
                continue

        return errors

    def compute_reprojection_error(
        self,
        image_points: List[Tuple[float, float]],
        object_point: np.ndarray,
        camera_ids: List[str],
    ) -> Optional[float]:
        """
        Compute mean reprojection error for a triangulated 3D point.
        """
        errors = self.compute_reprojection_errors(image_points, object_point, camera_ids)
        return float(np.mean(errors)) if errors else None

    def _compute_reprojection_errors_for_observations(
        self,
        observations: List[_BlobObservation],
        object_point: np.ndarray,
    ) -> List[float]:
        return self.compute_reprojection_errors(
            [obs.raw_uv for obs in observations],
            object_point,
            [obs.camera_id for obs in observations],
        )

    # ---------------------------------------------------------------------- #
    # Epipolar correspondence
    # ---------------------------------------------------------------------- #

    def _compute_fundamental_matrix(
        self, cam1_id: str, cam2_id: str
    ) -> Optional[np.ndarray]:
        """
        Compute the fundamental matrix F such that x2.T @ F @ x1 = 0.
        """
        key = (cam1_id, cam2_id)
        cached = self._fundamental_matrices.get(key)
        if cached is not None:
            return cached

        if cam1_id not in self.camera_params or cam2_id not in self.camera_params:
            return None
        cam1 = self.camera_params[cam1_id]
        cam2 = self.camera_params[cam2_id]

        R1, t1 = cam1.rotation, cam1.translation
        R2, t2 = cam2.rotation, cam2.translation

        R_rel = R2 @ R1.T
        t_rel = t2 - R_rel @ t1
        tx = np.array(
            [
                [0.0, -t_rel[2], t_rel[1]],
                [t_rel[2], 0.0, -t_rel[0]],
                [-t_rel[1], t_rel[0], 0.0],
            ],
            dtype=np.float64,
        )
        E = tx @ R_rel
        K1_inv = np.linalg.inv(cam1.intrinsic_matrix)
        K2_inv_T = np.linalg.inv(cam2.intrinsic_matrix).T
        F = K2_inv_T @ E @ K1_inv
        self._fundamental_matrices[key] = F
        return F

    @staticmethod
    def _epipolar_distance(
        F: np.ndarray,
        pt1: Tuple[float, float],
        pt2: Tuple[float, float],
    ) -> float:
        """Distance of pt2 from the epipolar line induced by pt1."""
        p1 = np.array([pt1[0], pt1[1], 1.0], dtype=np.float64)
        p2 = np.array([pt2[0], pt2[1], 1.0], dtype=np.float64)
        l2 = F @ p1
        denom = l2[0] ** 2 + l2[1] ** 2
        if denom < 1e-12:
            return float("inf")
        return abs(float(p2 @ l2)) / np.sqrt(denom)

    def _match_blobs_epipolar(
        self,
        ref_blobs: List[Tuple[float, float]],
        other_blobs: List[Tuple[float, float]],
        F: np.ndarray,
        threshold_px: float = _EPIPOLAR_THRESHOLD_PX,
    ) -> Dict[int, int]:
        """
        Legacy helper kept for tests and callers that already operate in a
        common image plane.
        """
        if not ref_blobs or not other_blobs:
            return {}

        cost = np.full(
            (len(ref_blobs), len(other_blobs)),
            _INVALID_ASSIGNMENT_COST,
            dtype=np.float64,
        )
        for i, ref_pt in enumerate(ref_blobs):
            for j, other_pt in enumerate(other_blobs):
                distance = self._epipolar_distance(F, ref_pt, other_pt)
                if distance <= threshold_px:
                    cost[i, j] = distance

        row_ind, col_ind = linear_sum_assignment(cost)
        matches: Dict[int, int] = {}
        for row, col in zip(row_ind, col_ind):
            if cost[row, col] < _INVALID_ASSIGNMENT_COST:
                matches[int(row)] = int(col)
        return matches

    def _match_observations_symmetric_epipolar(
        self,
        ref_cam: str,
        other_cam: str,
        ref_observations: List[_BlobObservation],
        other_observations: List[_BlobObservation],
        diagnostics: Dict[str, float],
        threshold_px: Optional[float] = None,
    ) -> Dict[int, Tuple[int, float]]:
        """Solve one-to-one assignment using undistorted symmetric epipolar distance."""
        if not ref_observations or not other_observations:
            return {}

        import time

        gate_px = float(self.epipolar_threshold_px if threshold_px is None else threshold_px)
        started_ns = time.perf_counter_ns()
        _increment_diagnostic(
            diagnostics,
            "assignment_candidates",
            amount=len(ref_observations) * len(other_observations),
        )
        _increment_diagnostic(
            diagnostics,
            "full_candidate_pair_count",
            amount=len(ref_observations) * len(other_observations),
        )

        F_ref_to_other = self._compute_fundamental_matrix(ref_cam, other_cam)
        F_other_to_ref = self._compute_fundamental_matrix(other_cam, ref_cam)
        if F_ref_to_other is None or F_other_to_ref is None:
            rejected_count = len(ref_observations) * len(other_observations)
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_epipolar",
                "rejected_epipolar",
                amount=rejected_count,
            )
            diagnostics["assignment_cost_ms"] += float(
                time.perf_counter_ns() - started_ns
            ) / 1_000_000.0
            return {}

        if self.epipolar_pruning_enabled:
            pruned = self._match_observations_symmetric_epipolar_pruned(
                F_ref_to_other,
                F_other_to_ref,
                ref_observations,
                other_observations,
                diagnostics,
                gate_px=gate_px,
            )
            if pruned is not None:
                diagnostics["assignment_cost_ms"] += float(
                    time.perf_counter_ns() - started_ns
                ) / 1_000_000.0
                return pruned

        if self.fast_geometry:
            detail_started_ns = time.perf_counter_ns()
            ref_points = [obs.undistorted_uv for obs in ref_observations]
            other_points = [obs.undistorted_uv for obs in other_observations]
            forward_matrix = self._epipolar_distance_matrix(
                F_ref_to_other,
                ref_points,
                other_points,
            )
            backward_matrix = self._epipolar_distance_matrix(
                F_other_to_ref,
                other_points,
                ref_points,
            ).T
            symmetric_errors = np.maximum(forward_matrix, backward_matrix)
            accepted_mask = symmetric_errors <= gate_px
            rejected_count = int(accepted_mask.size - int(np.count_nonzero(accepted_mask)))
            if rejected_count:
                _increment_diagnostic(
                    diagnostics,
                    "assignment_rejected_epipolar",
                    "rejected_epipolar",
                    amount=rejected_count,
                )
            cost = np.where(
                accepted_mask,
                forward_matrix + backward_matrix,
                _INVALID_ASSIGNMENT_COST,
            )
            self._record_stage_detail("epipolar_match_ms", detail_started_ns)
        else:
            cost = np.full(
                (len(ref_observations), len(other_observations)),
                _INVALID_ASSIGNMENT_COST,
                dtype=np.float64,
            )
            symmetric_errors = np.zeros_like(cost)
            for i, ref_obs in enumerate(ref_observations):
                for j, other_obs in enumerate(other_observations):
                    forward = self._epipolar_distance(
                        F_ref_to_other,
                        ref_obs.undistorted_uv,
                        other_obs.undistorted_uv,
                    )
                    backward = self._epipolar_distance(
                        F_other_to_ref,
                        other_obs.undistorted_uv,
                        ref_obs.undistorted_uv,
                    )
                    symmetric_error = max(forward, backward)
                    symmetric_errors[i, j] = float(symmetric_error)
                    if symmetric_error > gate_px:
                        _increment_diagnostic(
                            diagnostics,
                            "assignment_rejected_epipolar",
                            "rejected_epipolar",
                        )
                        continue
                    cost[i, j] = forward + backward
        _increment_diagnostic(
            diagnostics,
            "pruned_candidate_pair_count",
            amount=len(ref_observations) * len(other_observations),
        )

        row_ind, col_ind = linear_sum_assignment(cost)
        matches: Dict[int, Tuple[int, float]] = {}
        selected_cols: set[int] = set()
        for row, col in zip(row_ind, col_ind):
            if cost[row, col] >= _INVALID_ASSIGNMENT_COST:
                continue
            if int(col) in selected_cols:
                _increment_diagnostic(diagnostics, "duplicate_blob_matches")
                continue
            selected_cols.add(int(col))
            matches[int(row)] = (int(col), float(symmetric_errors[int(row), int(col)]))

        diagnostics["assignment_cost_ms"] += float(
            time.perf_counter_ns() - started_ns
        ) / 1_000_000.0
        return matches

    def _match_observations_symmetric_epipolar_pruned(
        self,
        F_ref_to_other: np.ndarray,
        F_other_to_ref: np.ndarray,
        ref_observations: List[_BlobObservation],
        other_observations: List[_BlobObservation],
        diagnostics: Dict[str, Any],
        *,
        gate_px: float,
    ) -> Optional[Dict[int, Tuple[int, float]]]:
        full_count = len(ref_observations) * len(other_observations)
        if full_count < 1024:
            self._record_pruning_fallback(diagnostics, "small_candidate_matrix")
            return None

        detail_started_ns = time.perf_counter_ns()
        ref_points = [obs.undistorted_uv for obs in ref_observations]
        other_points = [obs.undistorted_uv for obs in other_observations]
        forward_matrix = self._epipolar_distance_matrix(
            F_ref_to_other,
            ref_points,
            other_points,
        )
        backward_matrix = self._epipolar_distance_matrix(
            F_other_to_ref,
            other_points,
            ref_points,
        ).T
        symmetric_errors = np.maximum(forward_matrix, backward_matrix)
        accepted_mask = symmetric_errors <= gate_px
        rows, cols = np.nonzero(accepted_mask)
        candidate_pairs = {(int(row), int(col)) for row, col in zip(rows, cols)}
        if not candidate_pairs:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_epipolar",
                "rejected_epipolar",
                amount=full_count,
            )
            _increment_diagnostic(diagnostics, "component_count", amount=0)
            diagnostics["pruned_assignment_cost_ms"] += float(
                time.perf_counter_ns() - detail_started_ns
            ) / 1_000_000.0
            self._record_stage_detail("epipolar_match_ms", detail_started_ns)
            return {}
        if len(candidate_pairs) >= int(full_count * 0.95):
            self._record_pruning_fallback(diagnostics, "low_candidate_reduction")
            return None

        edges: Dict[Tuple[int, int], Tuple[float, float, float]] = {}
        for row, col in sorted(candidate_pairs):
            edges[(int(row), int(col))] = (
                float(forward_matrix[row, col] + backward_matrix[row, col]),
                float(symmetric_errors[row, col]),
                float(backward_matrix[row, col]),
            )

        _increment_diagnostic(diagnostics, "pruned_candidate_pair_count", amount=len(candidate_pairs))
        rejected_count = int(full_count - len(edges))
        if rejected_count:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_epipolar",
                "rejected_epipolar",
                amount=rejected_count,
            )
        if not edges:
            diagnostics["pruned_assignment_cost_ms"] += float(
                time.perf_counter_ns() - detail_started_ns
            ) / 1_000_000.0
            self._record_stage_detail("epipolar_match_ms", detail_started_ns)
            return {}

        components = self._connected_components_from_edges(edges)
        _increment_diagnostic(diagnostics, "component_count", amount=len(components))
        if components:
            diagnostics["largest_component_rows"] = max(
                float(diagnostics.get("largest_component_rows", 0.0)),
                float(max(len(rows) for rows, _cols in components)),
            )
            diagnostics["largest_component_cols"] = max(
                float(diagnostics.get("largest_component_cols", 0.0)),
                float(max(len(cols) for _rows, cols in components)),
            )

        matches: Dict[int, Tuple[int, float]] = {}
        selected_cols: set[int] = set()
        for rows, cols in components:
            cost = np.full((len(rows), len(cols)), _INVALID_ASSIGNMENT_COST, dtype=np.float64)
            symmetric_errors = np.zeros_like(cost)
            row_lookup = {row: index for index, row in enumerate(rows)}
            col_lookup = {col: index for index, col in enumerate(cols)}
            for (row, col), (pair_cost, symmetric_error, _backward) in edges.items():
                if row in row_lookup and col in col_lookup:
                    row_index = row_lookup[row]
                    col_index = col_lookup[col]
                    cost[row_index, col_index] = pair_cost
                    symmetric_errors[row_index, col_index] = symmetric_error
            row_ind, col_ind = linear_sum_assignment(cost)
            for row_pos, col_pos in zip(row_ind, col_ind):
                if cost[row_pos, col_pos] >= _INVALID_ASSIGNMENT_COST:
                    continue
                row = int(rows[int(row_pos)])
                col = int(cols[int(col_pos)])
                if col in selected_cols:
                    _increment_diagnostic(diagnostics, "duplicate_blob_matches")
                    continue
                selected_cols.add(col)
                matches[row] = (col, float(symmetric_errors[int(row_pos), int(col_pos)]))

        diagnostics["pruned_assignment_cost_ms"] += float(
            time.perf_counter_ns() - detail_started_ns
        ) / 1_000_000.0
        self._record_stage_detail("epipolar_match_ms", detail_started_ns)
        return matches

    def _match_blobs_symmetric_epipolar(
        self,
        ref_cam: str,
        other_cam: str,
        ref_blobs: List[Tuple[float, float]],
        other_blobs: List[Tuple[float, float]],
        diagnostics: Dict[str, float],
        threshold_px: float = _EPIPOLAR_THRESHOLD_PX,
    ) -> Dict[int, int]:
        ref_observations = [
            _BlobObservation(ref_cam, pt, pt) for pt in ref_blobs
        ]
        other_observations = [
            _BlobObservation(other_cam, pt, pt) for pt in other_blobs
        ]
        matches = self._match_observations_symmetric_epipolar(
            ref_cam,
            other_cam,
            ref_observations,
            other_observations,
            diagnostics,
            threshold_px=threshold_px,
        )
        return {ref_idx: other_idx for ref_idx, (other_idx, _error) in matches.items()}

    def _triangulation_angles_deg(
        self,
        point_3d: np.ndarray,
        camera_ids: List[str],
    ) -> List[float]:
        rays: List[np.ndarray] = []
        for cam_id in camera_ids:
            center = self._camera_centers.get(cam_id)
            if center is None:
                continue
            ray = point_3d - center
            norm = float(np.linalg.norm(ray))
            if norm <= 1e-12:
                continue
            rays.append(ray / norm)

        angles: List[float] = []
        for ray_a, ray_b in combinations(rays, 2):
            dot = float(np.clip(np.dot(ray_a, ray_b), -1.0, 1.0))
            angles.append(float(np.degrees(np.arccos(dot))))
        return angles

    def _refine_candidate_observation(
        self,
        observations: List[_BlobObservation],
        diagnostics: Dict[str, float],
        *,
        required_inlier_views: int = 2,
        epipolar_errors_by_camera: Optional[Dict[str, float]] = None,
    ) -> Optional[Dict[str, Any]]:
        if len(observations) < 2:
            return None

        current_observations = list(observations)
        epipolar_errors = dict(epipolar_errors_by_camera or {})

        while len(current_observations) >= 2:
            point_3d = self._triangulate_from_observations(current_observations)
            if point_3d is None:
                _increment_diagnostic(
                    diagnostics,
                    "assignment_rejected_triangulation",
                    "rejected_triangulation",
                )
                return None

            residuals = self._compute_reprojection_errors_for_observations(
                current_observations,
                point_3d,
            )
            if not residuals:
                _increment_diagnostic(
                    diagnostics,
                    "assignment_rejected_triangulation",
                    "rejected_triangulation",
                )
                return None

            worst_index = int(np.argmax(residuals))
            worst_residual = float(residuals[worst_index])
            if worst_residual <= _MAX_REPROJECTION_ERROR_PX:
                break
            if len(current_observations) <= 2:
                _increment_diagnostic(
                    diagnostics,
                    "assignment_rejected_reprojection",
                    "rejected_reprojection",
                )
                return None
            dropped = current_observations.pop(worst_index)
            epipolar_errors.pop(dropped.camera_id, None)
            _increment_diagnostic(diagnostics, "dropped_views_for_inlier_fit")

        if len(current_observations) < 2:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_triangulation",
                "rejected_triangulation",
            )
            return None

        point_3d = self._triangulate_from_observations(current_observations)
        if point_3d is None:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_triangulation",
                "rejected_triangulation",
            )
            return None

        residuals = self._compute_reprojection_errors_for_observations(
            current_observations,
            point_3d,
        )
        if not residuals:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_triangulation",
                "rejected_triangulation",
            )
            return None
        residual_summary = _metric_summary(residuals)
        if residual_summary["p90"] > _P90_REPROJECTION_ERROR_PX:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_reprojection",
                "rejected_reprojection",
            )
            return None

        if len(current_observations) < max(2, int(required_inlier_views)):
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_triangulation",
                "rejected_triangulation",
            )
            return None

        inlier_camera_ids = [obs.camera_id for obs in current_observations]
        triangulation_angles = self._triangulation_angles_deg(point_3d, inlier_camera_ids)
        best_angle = max(triangulation_angles) if triangulation_angles else 0.0
        if best_angle < _MIN_TRIANGULATION_ANGLE_DEG:
            _increment_diagnostic(
                diagnostics,
                "assignment_rejected_low_parallax",
                "rejected_low_parallax",
            )
            return None

        epipolar_values = [
            float(epipolar_errors[camera_id])
            for camera_id in inlier_camera_ids
            if camera_id in epipolar_errors
        ]
        return {
            "point_3d": point_3d,
            "inlier_observations": current_observations,
            "contributing_rays": len(current_observations),
            "reprojection_errors": residuals,
            "reprojection_error_px_summary": residual_summary,
            "epipolar_errors": epipolar_values,
            "epipolar_error_px_summary": _metric_summary(epipolar_values),
            "triangulation_angles_deg": triangulation_angles,
            "triangulation_angle_deg_summary": _metric_summary(triangulation_angles),
        }

    # ---------------------------------------------------------------------- #
    # Paired-frame triangulation (epipolar correspondence)
    # ---------------------------------------------------------------------- #

    def triangulate_paired_frames(
        self,
        paired_frames,
        blob_matcher=None,
        *,
        min_inlier_views: int = 2,
        blob_indices_by_camera: Optional[Dict[str, set[int]]] = None,
    ) -> Tuple[List[np.ndarray], List[float]]:
        """
        Triangulate all matched blobs from paired frames using epipolar
        geometry for correspondence.
        """
        _ = blob_matcher
        points_3d: List[np.ndarray] = []
        errors: List[float] = []
        triangulated_points: List[TriangulatedPoint] = []
        diagnostics = _empty_assignment_diagnostics()
        self._last_assignment_diagnostics = dict(diagnostics)
        self._last_quality_metrics = self._empty_quality_metrics(diagnostics)
        self._last_observations_by_camera = {}
        self._last_triangulated_points = []

        camera_ids = [cid for cid in paired_frames.camera_ids if cid in self.camera_params]
        if len(camera_ids) < 2:
            self._finalize_epipolar_pruning_summary(diagnostics)
            self._last_assignment_diagnostics = dict(diagnostics)
            self._last_quality_metrics = self._empty_quality_metrics(diagnostics)
            return points_3d, errors

        observations_by_camera: Dict[str, List[_BlobObservation]] = {}
        for cam_id in camera_ids:
            frame = paired_frames.frames.get(cam_id)
            if frame is not None and frame.blobs:
                include_indices = None
                if blob_indices_by_camera is not None:
                    include_indices = set(blob_indices_by_camera.get(cam_id, set()))
                observations_by_camera[cam_id] = self._build_observations(
                    cam_id,
                    frame.blobs,
                    include_blob_indices=include_indices,
                )
        self._last_observations_by_camera = {
            camera_id: list(observations)
            for camera_id, observations in observations_by_camera.items()
        }

        active_cams = [camera_id for camera_id in camera_ids if observations_by_camera.get(camera_id)]
        if len(active_cams) < 2:
            self._finalize_epipolar_pruning_summary(diagnostics)
            self._last_assignment_diagnostics = dict(diagnostics)
            self._last_quality_metrics = self._empty_quality_metrics(diagnostics)
            return points_3d, errors

        ref_cam = max(active_cams, key=lambda camera_id: len(observations_by_camera[camera_id]))
        ref_observations = observations_by_camera[ref_cam]

        matches_by_camera: Dict[str, Dict[int, Tuple[int, float]]] = {}
        for cam_id in active_cams:
            if cam_id == ref_cam:
                continue
            matches_by_camera[cam_id] = self._match_observations_symmetric_epipolar(
                ref_cam,
                cam_id,
                ref_observations,
                observations_by_camera[cam_id],
                diagnostics,
            )

        accepted_contributing_rays: List[int] = []
        accepted_reprojection_errors: List[float] = []
        accepted_epipolar_errors: List[float] = []
        accepted_triangulation_angles: List[float] = []

        for ref_idx, ref_obs in enumerate(ref_observations):
            candidate_observations = [ref_obs]
            candidate_epipolar_errors: Dict[str, float] = {}
            for cam_id, matches in matches_by_camera.items():
                matched = matches.get(ref_idx)
                if matched is None:
                    continue
                other_idx, epi_error = matched
                candidate_observations.append(observations_by_camera[cam_id][other_idx])
                candidate_epipolar_errors[cam_id] = float(epi_error)

            if len(candidate_observations) < 2:
                continue

            candidate = self._refine_candidate_observation(
                candidate_observations,
                diagnostics,
                required_inlier_views=min_inlier_views,
                epipolar_errors_by_camera=candidate_epipolar_errors,
            )
            if candidate is None:
                continue

            candidate_point = np.asarray(candidate["point_3d"], dtype=np.float64)
            candidate_reprojection = list(candidate["reprojection_errors"])
            points_3d.append(candidate_point)
            errors.append(float(np.mean(candidate_reprojection)))
            _increment_diagnostic(diagnostics, "assignment_matches", "triangulated_pairs")

            accepted_contributing_rays.append(int(candidate["contributing_rays"]))
            accepted_reprojection_errors.extend(float(v) for v in candidate_reprojection)
            accepted_epipolar_errors.extend(float(v) for v in candidate["epipolar_errors"])
            angle_summary = candidate["triangulation_angle_deg_summary"]
            if angle_summary.get("count", 0) > 0:
                accepted_triangulation_angles.append(float(angle_summary["max"]))
            triangulated_points.append(
                TriangulatedPoint(
                    point=candidate_point,
                    observations=tuple(candidate["inlier_observations"]),
                    reprojection_errors_px=tuple(float(v) for v in candidate_reprojection),
                    epipolar_errors_px=tuple(float(v) for v in candidate["epipolar_errors"]),
                    triangulation_angles_deg=tuple(
                        float(v) for v in candidate["triangulation_angles_deg"]
                    ),
                    source="generic",
                )
            )

        self._finalize_epipolar_pruning_summary(diagnostics)
        self._last_assignment_diagnostics = dict(diagnostics)
        self._last_triangulated_points = list(triangulated_points)
        self._last_quality_metrics = {
            "accepted_points": len(points_3d),
            "contributing_rays": {
                "per_point": list(accepted_contributing_rays),
                "summary": _metric_summary([float(v) for v in accepted_contributing_rays]),
            },
            "reprojection_error_px_summary": _metric_summary(accepted_reprojection_errors),
            "epipolar_error_px_summary": _metric_summary(accepted_epipolar_errors),
            "triangulation_angle_deg_summary": _metric_summary(accepted_triangulation_angles),
            "assignment_diagnostics": dict(diagnostics),
        }
        return points_3d, errors

    def triangulate_rigid_hints(
        self,
        object_gating: Optional[Dict[str, Any]],
        *,
        min_inlier_views: int = 2,
    ) -> Tuple[List[np.ndarray], List[float], List[Dict[str, Any]], Dict[str, Any]]:
        """
        Triangulate object-conditioned marker assignments beside generic matching.

        The returned points are diagnostics-only. They intentionally do not
        replace the generic epipolar triangulation output until adoption metrics
        show that the hinted lane is safer for pose estimation.
        """
        if not object_gating:
            return [], [], [], self._empty_rigid_hint_quality("no_object_gating")
        if not self._last_observations_by_camera:
            return [], [], [], self._empty_rigid_hint_quality("no_observations")

        points_3d: List[np.ndarray] = []
        errors: List[float] = []
        triangulated_points: List[TriangulatedPoint] = []
        diagnostics = _empty_assignment_diagnostics()
        accepted_contributing_rays: List[int] = []
        accepted_reprojection_errors: List[float] = []
        accepted_triangulation_angles: List[float] = []
        by_rigid: Dict[str, Any] = {}

        total_candidate_markers = 0
        total_two_or_more_rays = 0
        total_single_ray = 0
        total_rejected = 0
        total_invalid = 0

        for rigid_name, gating in object_gating.items():
            if not isinstance(gating, dict) or not gating.get("evaluated"):
                continue
            per_camera = gating.get("per_camera")
            if not isinstance(per_camera, dict):
                continue

            observations_by_marker: Dict[int, List[_BlobObservation]] = {}
            invalid_assignments = 0
            for camera_id, camera_payload in per_camera.items():
                if not isinstance(camera_payload, dict):
                    continue
                camera_observations = self._last_observations_by_camera.get(str(camera_id), [])
                observations_by_blob_index = {
                    int(observation.blob_index): observation
                    for observation in camera_observations
                }
                assignments = camera_payload.get("assignments", [])
                if not isinstance(assignments, list):
                    continue
                for assignment in assignments:
                    if not isinstance(assignment, dict):
                        continue
                    try:
                        marker_idx = int(assignment["marker_idx"])
                        blob_index = int(assignment["blob_index"])
                    except (KeyError, TypeError, ValueError):
                        invalid_assignments += 1
                        continue
                    observation = observations_by_blob_index.get(blob_index)
                    if marker_idx < 0 or blob_index < 0 or observation is None:
                        invalid_assignments += 1
                        continue
                    observations_by_marker.setdefault(marker_idx, []).append(observation)

            candidate_markers = int(len(observations_by_marker))
            two_or_more = int(
                sum(1 for observations in observations_by_marker.values() if len(observations) >= 2)
            )
            single_ray = int(
                sum(1 for observations in observations_by_marker.values() if len(observations) == 1)
            )
            accepted_for_rigid = 0
            rejected_for_rigid = 0

            for marker_idx, observations in sorted(observations_by_marker.items()):
                if len(observations) < 2:
                    continue
                candidate = self._refine_candidate_observation(
                    observations,
                    diagnostics,
                    required_inlier_views=min_inlier_views,
                )
                if candidate is None:
                    rejected_for_rigid += 1
                    continue

                candidate_point = np.asarray(candidate["point_3d"], dtype=np.float64)
                candidate_reprojection = [float(v) for v in candidate["reprojection_errors"]]
                points_3d.append(candidate_point)
                errors.append(float(np.mean(candidate_reprojection)))
                accepted_for_rigid += 1
                _increment_diagnostic(diagnostics, "assignment_matches", "triangulated_pairs")

                accepted_contributing_rays.append(int(candidate["contributing_rays"]))
                accepted_reprojection_errors.extend(candidate_reprojection)
                angle_summary = candidate["triangulation_angle_deg_summary"]
                if angle_summary.get("count", 0) > 0:
                    accepted_triangulation_angles.append(float(angle_summary["max"]))
                triangulated_points.append(
                    TriangulatedPoint(
                        point=candidate_point,
                        observations=tuple(candidate["inlier_observations"]),
                        reprojection_errors_px=tuple(candidate_reprojection),
                        epipolar_errors_px=tuple(float(v) for v in candidate["epipolar_errors"]),
                        triangulation_angles_deg=tuple(
                            float(v) for v in candidate["triangulation_angles_deg"]
                        ),
                        source="rigid_hint",
                        rigid_name=str(rigid_name),
                        marker_idx=int(marker_idx),
                    )
                )

            total_candidate_markers += candidate_markers
            total_two_or_more_rays += two_or_more
            total_single_ray += single_ray
            total_rejected += rejected_for_rigid
            total_invalid += invalid_assignments
            by_rigid[str(rigid_name)] = {
                "candidate_markers": candidate_markers,
                "markers_with_two_or_more_rays": two_or_more,
                "single_ray_candidates": single_ray,
                "accepted_points": int(accepted_for_rigid),
                "rejected_markers": int(rejected_for_rigid),
                "invalid_assignments": int(invalid_assignments),
            }

        serialized_points = [point.to_dict() for point in triangulated_points]
        quality = {
            "reason": "ok" if by_rigid else "no_evaluated_rigids",
            "rigid_count": int(len(by_rigid)),
            "candidate_markers": int(total_candidate_markers),
            "markers_with_two_or_more_rays": int(total_two_or_more_rays),
            "single_ray_candidates": int(total_single_ray),
            "accepted_points": int(len(points_3d)),
            "rejected_markers": int(total_rejected),
            "invalid_assignments": int(total_invalid),
            "by_rigid": by_rigid,
            "contributing_rays": {
                "per_point": list(accepted_contributing_rays),
                "summary": _metric_summary([float(v) for v in accepted_contributing_rays]),
            },
            "reprojection_error_px_summary": _metric_summary(accepted_reprojection_errors),
            "triangulation_angle_deg_summary": _metric_summary(accepted_triangulation_angles),
            "assignment_diagnostics": dict(diagnostics),
        }
        return points_3d, errors, serialized_points, quality

    def triangulate_points(
        self,
        image_points_list: List[List[Tuple[float, float]]],
        camera_ids: List[str],
    ) -> List[Optional[np.ndarray]]:
        """Triangulate a list of 3D points (one set of observations each)."""
        return [
            self.triangulate_point(image_points, camera_ids)
            for image_points in image_points_list
        ]


class GeometryPipeline:
    """
    Complete geometry processing pipeline.

    Integrates calibration loading, triangulation, and error computation.
    """

    def __init__(
        self,
        *,
        pipeline_variant: str = "baseline",
        stage_callback: Optional[Callable[[str, float], None]] = None,
    ) -> None:
        self.pipeline_variant = str(pipeline_variant or "baseline")
        self._stage_callback = stage_callback
        self.camera_params: Dict[str, CameraParams] = {}
        self.triangulator: Optional[Triangulator] = None
        self.coordinate_frame = "camera_similarity"
        self.coordinate_origin = "reference_camera"
        self.coordinate_origin_source = "extrinsics_pose_reference"
        self.calibration_validation: Dict[str, Any] = {}
        self.epipolar_threshold_px = _EPIPOLAR_THRESHOLD_PX
        self._latest_quality_metrics: Dict[str, Any] = Triangulator._empty_quality_metrics()
        self._latest_rigid_hint_quality: Dict[str, Any] = Triangulator._empty_rigid_hint_quality()

    @staticmethod
    def _extract_validation_summary(meta: Dict[str, Any]) -> Dict[str, Any]:
        if not isinstance(meta, dict):
            return {}

        for section_name in ("metric", "pose"):
            section = meta.get(section_name)
            if not isinstance(section, dict):
                continue
            validation = section.get("validation")
            if isinstance(validation, dict):
                summary = dict(validation)
                summary["source"] = section_name
                return summary
            solve_summary = section.get("solve_summary")
            if isinstance(solve_summary, dict):
                summary = dict(solve_summary)
                summary["source"] = f"{section_name}.solve_summary"
                return summary

        validation = meta.get("validation")
        if isinstance(validation, dict):
            summary = dict(validation)
            summary["source"] = "root.validation"
            return summary
        return {}

    @classmethod
    def _derive_epipolar_threshold_px(cls, validation_summary: Dict[str, Any]) -> float:
        median_error = validation_summary.get("median_reproj_error_px")
        if isinstance(median_error, (int, float)) and np.isfinite(float(median_error)):
            return float(
                min(
                    _EPIPOLAR_THRESHOLD_PX_MAX,
                    max(_EPIPOLAR_THRESHOLD_PX, 2.0 * float(median_error)),
                )
            )
        return float(_EPIPOLAR_THRESHOLD_PX)

    def load_calibration(self, filepath: str) -> int:
        """Load camera calibration data. Returns number of cameras loaded."""
        calibrations = CalibrationLoader.load_intrinsics(filepath)

        if not calibrations:
            calibrations = CalibrationLoader.load_from_reference_format(filepath)

        self.camera_params = {}
        self.coordinate_frame = "camera_similarity"
        self.coordinate_origin = "reference_camera"
        self.coordinate_origin_source = "extrinsics_pose_reference"
        self.calibration_validation = {}
        self.epipolar_threshold_px = _EPIPOLAR_THRESHOLD_PX
        self._latest_quality_metrics = Triangulator._empty_quality_metrics()
        self._latest_rigid_hint_quality = Triangulator._empty_rigid_hint_quality()
        for cam_id, calib in calibrations.items():
            if isinstance(calib, CameraParams):
                self.camera_params[cam_id] = calib
            else:
                self.camera_params[cam_id] = CalibrationLoader.to_camera_params(
                    calib
                )

        extrinsics = CalibrationLoader.load_extrinsics(filepath, include_meta=True)
        if extrinsics:
            self.calibration_validation = self._extract_validation_summary(
                extrinsics.get("__meta__", {})
            )
            self.epipolar_threshold_px = self._derive_epipolar_threshold_px(
                self.calibration_validation
            )
            CalibrationLoader.apply_extrinsics(self.camera_params, extrinsics)
            camera_rows = [
                row
                for camera_id, row in extrinsics.items()
                if not str(camera_id).startswith("__") and isinstance(row, dict)
            ]
            if any(row.get("coordinate_origin") == "wand" for row in camera_rows):
                self.coordinate_frame = "world"
                self.coordinate_origin = "wand"
                self.coordinate_origin_source = "floor_metric_capture"
            elif camera_rows:
                self.coordinate_frame = str(camera_rows[0].get("frame", "camera_similarity"))
                self.coordinate_origin = "reference_camera"
                self.coordinate_origin_source = "extrinsics_pose_reference"

        if self.camera_params:
            self.triangulator = Triangulator(
                self.camera_params,
                epipolar_threshold_px=self.epipolar_threshold_px,
                fast_geometry=self.pipeline_variant != "baseline",
                epipolar_pruning_enabled=self.pipeline_variant == "fast_ABCDP",
                stage_callback=self._stage_callback,
            )

        return len(self.camera_params)

    def set_extrinsics(
        self,
        camera_id: str,
        rotation: np.ndarray,
        translation: np.ndarray,
    ) -> None:
        """Update extrinsic parameters for a camera and refresh cached matrices."""
        if camera_id not in self.camera_params:
            return
        self.camera_params[camera_id].rotation = rotation
        self.camera_params[camera_id].translation = translation

        if self.triangulator is not None:
            self.triangulator.camera_params = self.camera_params
            self.triangulator._refresh_cached_matrices()

    def process_paired_frames(
        self,
        paired_frames,
        *,
        min_inlier_views: int = 2,
        object_gating: Optional[Dict[str, Any]] = None,
        generic_blob_indices_by_camera: Optional[Dict[str, set[int]]] = None,
    ) -> Dict[str, Any]:
        """Process paired frames to extract 3D points."""
        if not self.triangulator:
            return {"error": "No calibration loaded"}

        stage_started_ns = time.perf_counter_ns()
        points_3d, errors = self.triangulator.triangulate_paired_frames(
            paired_frames,
            min_inlier_views=min_inlier_views,
            blob_indices_by_camera=generic_blob_indices_by_camera,
        )
        if self._stage_callback is not None:
            self._stage_callback(
                "generic_triangulation_ms",
                float(time.perf_counter_ns() - stage_started_ns) / 1_000_000.0,
            )
        assignment_diagnostics = self.triangulator.last_assignment_diagnostics
        quality_metrics = self.triangulator.last_quality_metrics
        observations_by_camera = self.triangulator.last_observations_by_camera
        triangulated_points = self.triangulator.last_triangulated_points
        self._latest_quality_metrics = quality_metrics
        stage_started_ns = time.perf_counter_ns()
        (
            rigid_hint_points,
            rigid_hint_errors,
            rigid_hint_triangulated_points,
            rigid_hint_quality,
        ) = self.triangulator.triangulate_rigid_hints(
            object_gating,
            min_inlier_views=min_inlier_views,
        )
        if self._stage_callback is not None:
            self._stage_callback(
                "rigid_hint_triangulation_ms",
                float(time.perf_counter_ns() - stage_started_ns) / 1_000_000.0,
            )
        self._latest_rigid_hint_quality = rigid_hint_quality

        return {
            "timestamp": paired_frames.timestamp,
            "camera_ids": paired_frames.camera_ids,
            "points_3d": points_3d,
            "rigid_hint_points_3d": rigid_hint_points,
            "observations_by_camera": observations_by_camera,
            "triangulated_points": triangulated_points,
            "rigid_hint_triangulated_points": rigid_hint_triangulated_points,
            "reprojection_errors": errors,
            "rigid_hint_reprojection_errors": rigid_hint_errors,
            "assignment_diagnostics": assignment_diagnostics,
            "triangulation_quality": quality_metrics,
            "rigid_hint_quality": rigid_hint_quality,
            "mean_error": float(np.mean(errors)) if errors else 0.0,
            "point_count": len(points_3d),
        }

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return geometry diagnostics for status/log snapshots."""
        camera_diagnostics = {
            camera_id: {
                "focal_scale": float(camera.focal_scale),
                "fx": float(camera.fx),
                "fy": float(camera.fy),
                "cx": float(camera.cx),
                "cy": float(camera.cy),
                "resolution": {
                    "width": int(camera.resolution[0]),
                    "height": int(camera.resolution[1]),
                },
            }
            for camera_id, camera in self.camera_params.items()
        }
        if self.triangulator is None:
            return {
                "assignment": _empty_assignment_diagnostics(),
                "quality": Triangulator._empty_quality_metrics(),
                "rigid_hint": Triangulator._empty_rigid_hint_quality(),
                "epipolar_threshold_px": float(self.epipolar_threshold_px),
                "calibration_validation": dict(self.calibration_validation),
                "cameras": camera_diagnostics,
            }
        return {
            "assignment": self.triangulator.last_assignment_diagnostics,
            "quality": self.triangulator.last_quality_metrics,
            "rigid_hint": dict(self._latest_rigid_hint_quality),
            "epipolar_threshold_px": float(self.epipolar_threshold_px),
            "calibration_validation": dict(self.calibration_validation),
            "cameras": camera_diagnostics,
        }

    def get_camera_ids(self) -> List[str]:
        return list(self.camera_params.keys())

    def get_baseline(self, cam1_id: str, cam2_id: str) -> Optional[float]:
        """Compute baseline distance between two cameras in world units."""
        if cam1_id not in self.camera_params or cam2_id not in self.camera_params:
            return None
        cam1 = self.camera_params[cam1_id]
        cam2 = self.camera_params[cam2_id]
        C1 = -cam1.rotation.T @ cam1.translation
        C2 = -cam2.rotation.T @ cam2.translation
        return float(np.linalg.norm(C1 - C2))


# --------------------------------------------------------------------------- #
# Utility functions
# --------------------------------------------------------------------------- #

def undistort_points(
    points: List[Tuple[float, float]],
    camera_params: CameraParams,
) -> List[Tuple[float, float]]:
    """Undistort 2D points to ideal pixel coordinates."""
    if not points:
        return []
    pts = np.array(points, dtype=np.float64).reshape(-1, 1, 2)
    undistorted = cv.undistortPoints(
        pts,
        camera_params.intrinsic_matrix,
        camera_params.distortion_coeffs,
        None,
        camera_params.intrinsic_matrix,
    )
    return [(float(p[0, 0]), float(p[0, 1])) for p in undistorted]


def create_dummy_calibration(
    camera_ids: List[str],
    resolution: Tuple[int, int] = (1280, 960),
    focal_length: float = 500.0,
) -> Dict[str, CameraParams]:
    """Create dummy calibration for testing (identity extrinsics, no distortion)."""
    params = {}
    width, height = resolution
    cx, cy = width / 2.0, height / 2.0

    for i, cam_id in enumerate(camera_ids):
        K = np.array(
            [[focal_length, 0, cx], [0, focal_length, cy], [0, 0, 1]],
            dtype=np.float64,
        )
        # Cameras placed in a line: C = [i*0.5, 0, 0], t = -R @ C = -C
        camera_center = np.array([i * 0.5, 0.0, 0.0], dtype=np.float64)
        translation = -camera_center

        params[cam_id] = CameraParams(
            camera_id=cam_id,
            intrinsic_matrix=K,
            distortion_coeffs=np.zeros(5, dtype=np.float64),
            rotation=np.eye(3, dtype=np.float64),
            translation=translation,
            resolution=resolution,
        )

    return params
