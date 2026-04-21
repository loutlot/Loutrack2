"""
Geometry module for 3D reconstruction from multi-camera 2D observations.

Provides functionality to:
- Load and manage camera intrinsic/extrinsic parameters
- Triangulate 3D points from 2D observations (DLT with lens-distortion correction)
- Compute reprojection errors for quality assessment
- Epipolar-based correspondence matching across cameras
"""

import json
import numpy as np
import cv2 as cv
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
from scipy.optimize import linear_sum_assignment


@dataclass
class CameraParams:
    """Camera intrinsic and extrinsic parameters."""
    camera_id: str
    intrinsic_matrix: np.ndarray  # 3x3
    distortion_coeffs: np.ndarray  # 5+ coefficients
    rotation: np.ndarray  # 3x3 rotation matrix (extrinsic)
    translation: np.ndarray  # 3x1 translation vector (extrinsic)
    resolution: Tuple[int, int] = (1280, 960)  # width, height

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
        """Load intrinsic calibration from loutrack2 schema format."""
        path = Path(filepath)

        if path.is_dir():
            calibrations = {}
            for f in path.glob("*.json"):
                try:
                    with open(f, "r", encoding="utf-8") as handle:
                        data = json.load(handle)
                    if "camera_id" in data and "camera_matrix" in data:
                        calibrations[data["camera_id"]] = data
                except Exception:
                    continue
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
            rotation = np.array(
                extrinsic.get("R", np.eye(3)), dtype=np.float64
            )
            translation = np.array(
                extrinsic.get("t", [0.0, 0.0, 0.0]), dtype=np.float64
            )
            camera_params[camera_id].rotation = rotation.reshape(3, 3)
            camera_params[camera_id].translation = translation.reshape(3)

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
# Epipolar threshold: maximum one-sided distance (px) for a blob to be
# considered on an epipolar line.  12 px is generous enough to tolerate
# sub-pixel calibration error while rejecting true mismatches.
# --------------------------------------------------------------------------- #
_EPIPOLAR_THRESHOLD_PX: float = 12.0
_MAX_REPROJECTION_ERROR_PX: float = 3.0
_INVALID_ASSIGNMENT_COST: float = 1e9


def _empty_assignment_diagnostics() -> Dict[str, float]:
    return {
        "assignment_candidates": 0,
        "assignment_matches": 0,
        "assignment_rejected_epipolar": 0,
        "assignment_rejected_triangulation": 0,
        "assignment_rejected_reprojection": 0,
        "duplicate_blob_matches": 0,
        "assignment_cost_ms": 0.0,
        "triangulated_pairs": 0,
    }


class Triangulator:
    """
    Triangulate 3D points from 2D observations across multiple cameras.

    Uses Direct Linear Transform (DLT) with:
    - Lens distortion correction via cv.undistortPoints before DLT
    - SVD applied directly to the constraint matrix A (not A.T @ A) for
      numerical stability
    - Epipolar-based blob correspondence instead of index-based matching
    """

    def __init__(self, camera_params: Dict[str, CameraParams]):
        self.camera_params = camera_params
        # Precomputed per-camera matrices — call _refresh_cached_matrices()
        # whenever camera_params R/t are updated.
        self._proj_matrices: Dict[str, np.ndarray] = {}
        self._rvecs: Dict[str, np.ndarray] = {}
        self._last_assignment_diagnostics: Dict[str, float] = _empty_assignment_diagnostics()
        self._refresh_cached_matrices()

    @property
    def last_assignment_diagnostics(self) -> Dict[str, float]:
        """Return diagnostics from the most recent blob assignment pass."""
        return dict(self._last_assignment_diagnostics)

    # ---------------------------------------------------------------------- #
    # Cache management
    # ---------------------------------------------------------------------- #

    def _refresh_cached_matrices(self) -> None:
        """Precompute projection matrices and rotation vectors for all cameras."""
        self._proj_matrices = {}
        self._rvecs = {}
        for cam_id, cam in self.camera_params.items():
            RT = np.hstack([cam.rotation, cam.translation.reshape(3, 1)])
            self._proj_matrices[cam_id] = cam.intrinsic_matrix @ RT
            rvec, _ = cv.Rodrigues(cam.rotation)
            self._rvecs[cam_id] = rvec

    # ---------------------------------------------------------------------- #
    # Core triangulation
    # ---------------------------------------------------------------------- #

    def triangulate_point(
        self,
        image_points: List[Tuple[float, float]],
        camera_ids: List[str],
    ) -> Optional[np.ndarray]:
        """
        Triangulate a single 3D point from 2D observations.

        Applies lens undistortion before building the DLT constraint matrix,
        and solves with SVD applied directly to A (not the normal equations).

        Args:
            image_points: Observed (possibly distorted) 2D pixel coordinates.
            camera_ids:   Camera IDs corresponding to each point.

        Returns:
            3D point [x, y, z] or None if insufficient/degenerate data.
        """
        if len(image_points) < 2:
            return None

        # Collect valid (point, camera) pairs
        valid_points = []
        valid_cameras = []
        for pt, cam_id in zip(image_points, camera_ids):
            if pt is not None and cam_id in self.camera_params:
                valid_points.append(pt)
                valid_cameras.append(cam_id)

        if len(valid_points) < 2:
            return None

        # --- Step 1: Undistort each 2D observation to ideal pixel coords ----
        undistorted: List[Tuple[float, float]] = []
        for pt, cam_id in zip(valid_points, valid_cameras):
            cam = self.camera_params[cam_id]
            pts_arr = np.array([[[pt[0], pt[1]]]], dtype=np.float64)
            und = cv.undistortPoints(
                pts_arr,
                cam.intrinsic_matrix,
                cam.distortion_coeffs,
                P=cam.intrinsic_matrix,  # map back to pixel space
            )
            undistorted.append((float(und[0, 0, 0]), float(und[0, 0, 1])))

        # --- Step 2: Build DLT constraint matrix A (2N × 4) -----------------
        A = []
        for pt, cam_id in zip(undistorted, valid_cameras):
            P = self._proj_matrices[cam_id]
            u, v = pt
            A.append(v * P[2, :] - P[1, :])
            A.append(P[0, :] - u * P[2, :])

        A_arr = np.array(A, dtype=np.float64)

        # --- Step 3: Solve via SVD of A (not A.T @ A for stability) ---------
        try:
            _, _, Vh = np.linalg.svd(A_arr, full_matrices=True)
            X_hom = Vh[-1, :]  # null-space vector (smallest singular value)

            if abs(X_hom[3]) < 1e-10:
                return None

            point_3d = X_hom[:3] / X_hom[3]

            # Cheirality check: point must lie in front of every camera
            # (positive depth in each camera's coordinate frame).
            for cam_id in valid_cameras:
                cam = self.camera_params[cam_id]
                depth = float(cam.rotation[2, :] @ point_3d + cam.translation[2])
                if depth <= 0.0:
                    return None

            return point_3d
        except np.linalg.LinAlgError:
            return None

    # ---------------------------------------------------------------------- #
    # Reprojection error
    # ---------------------------------------------------------------------- #

    def compute_reprojection_error(
        self,
        image_points: List[Tuple[float, float]],
        object_point: np.ndarray,
        camera_ids: List[str],
    ) -> Optional[float]:
        """
        Compute mean reprojection error for a triangulated 3D point.

        Uses pre-cached rvecs to avoid repeated Rodrigues conversions.
        """
        errors = []
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
                errors.append(np.linalg.norm(observed - projected.squeeze()))
            except Exception:
                continue

        return float(np.mean(errors)) if errors else None

    # ---------------------------------------------------------------------- #
    # Epipolar correspondence
    # ---------------------------------------------------------------------- #

    def _compute_fundamental_matrix(
        self, cam1_id: str, cam2_id: str
    ) -> Optional[np.ndarray]:
        """
        Compute the fundamental matrix F such that  x2.T @ F @ x1 = 0.

        Uses the known calibration; no point correspondences required.
        """
        if cam1_id not in self.camera_params or cam2_id not in self.camera_params:
            return None
        cam1 = self.camera_params[cam1_id]
        cam2 = self.camera_params[cam2_id]

        R1, t1 = cam1.rotation, cam1.translation
        R2, t2 = cam2.rotation, cam2.translation

        # Relative pose: world → cam1 then cam1 → cam2
        R_rel = R2 @ R1.T
        t_rel = t2 - R_rel @ t1

        # Skew-symmetric cross-product matrix of t_rel
        tx = np.array(
            [
                [0.0, -t_rel[2], t_rel[1]],
                [t_rel[2], 0.0, -t_rel[0]],
                [-t_rel[1], t_rel[0], 0.0],
            ]
        )
        E = tx @ R_rel  # essential matrix

        K1_inv = np.linalg.inv(cam1.intrinsic_matrix)
        K2_inv_T = np.linalg.inv(cam2.intrinsic_matrix).T
        return K2_inv_T @ E @ K1_inv

    @staticmethod
    def _epipolar_distance(
        F: np.ndarray,
        pt1: Tuple[float, float],
        pt2: Tuple[float, float],
    ) -> float:
        """
        One-sided epipolar distance: distance of pt2 from its epipolar line
        l2 = F @ pt1.
        """
        p1 = np.array([pt1[0], pt1[1], 1.0])
        p2 = np.array([pt2[0], pt2[1], 1.0])
        l2 = F @ p1  # epipolar line in cam2
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
        One-to-one nearest-epipolar-line matching.

        Builds a dense cost matrix and solves a global assignment so an
        observation in the other camera cannot be reused for multiple
        reference blobs. This keeps the legacy helper name/API while replacing
        its previous greedy behavior.

        Returns:
            Dict mapping ref_blob_index → other_blob_index.
        """
        if not ref_blobs or not other_blobs:
            return {}

        cost = np.full((len(ref_blobs), len(other_blobs)), _INVALID_ASSIGNMENT_COST, dtype=np.float64)
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

    def _match_blobs_symmetric_epipolar(
        self,
        ref_cam: str,
        other_cam: str,
        ref_blobs: List[Tuple[float, float]],
        other_blobs: List[Tuple[float, float]],
        diagnostics: Dict[str, float],
        threshold_px: float = _EPIPOLAR_THRESHOLD_PX,
    ) -> Dict[int, int]:
        """Solve one-to-one assignment using symmetric epipolar distance."""
        if not ref_blobs or not other_blobs:
            return {}

        import time

        started_ns = time.perf_counter_ns()
        diagnostics["assignment_candidates"] += len(ref_blobs) * len(other_blobs)

        F_ref_to_other = self._compute_fundamental_matrix(ref_cam, other_cam)
        F_other_to_ref = self._compute_fundamental_matrix(other_cam, ref_cam)
        if F_ref_to_other is None or F_other_to_ref is None:
            diagnostics["assignment_rejected_epipolar"] += len(ref_blobs) * len(other_blobs)
            diagnostics["assignment_cost_ms"] += float(time.perf_counter_ns() - started_ns) / 1_000_000.0
            return {}

        cost = np.full((len(ref_blobs), len(other_blobs)), _INVALID_ASSIGNMENT_COST, dtype=np.float64)
        for i, ref_pt in enumerate(ref_blobs):
            for j, other_pt in enumerate(other_blobs):
                forward = self._epipolar_distance(F_ref_to_other, ref_pt, other_pt)
                backward = self._epipolar_distance(F_other_to_ref, other_pt, ref_pt)
                if max(forward, backward) > threshold_px:
                    diagnostics["assignment_rejected_epipolar"] += 1
                    continue
                cost[i, j] = forward + backward

        row_ind, col_ind = linear_sum_assignment(cost)
        matches: Dict[int, int] = {}
        selected_cols: set[int] = set()
        for row, col in zip(row_ind, col_ind):
            if cost[row, col] >= _INVALID_ASSIGNMENT_COST:
                continue
            if int(col) in selected_cols:
                diagnostics["duplicate_blob_matches"] += 1
                continue
            selected_cols.add(int(col))
            matches[int(row)] = int(col)

        diagnostics["assignment_cost_ms"] += float(time.perf_counter_ns() - started_ns) / 1_000_000.0
        return matches

    def _append_triangulated_observation(
        self,
        image_pts: List[Tuple[float, float]],
        cam_ids_for_pt: List[str],
        points_3d: List[np.ndarray],
        errors: List[float],
        diagnostics: Dict[str, float],
        max_reprojection_error_px: float = _MAX_REPROJECTION_ERROR_PX,
    ) -> None:
        """Triangulate one candidate and append it if it passes geometry checks."""
        pt3d = self.triangulate_point(image_pts, cam_ids_for_pt)
        if pt3d is None:
            diagnostics["assignment_rejected_triangulation"] += 1
            return

        err = self.compute_reprojection_error(image_pts, pt3d, cam_ids_for_pt)
        err_value = float(err if err is not None else 0.0)
        if err is None or err_value > max_reprojection_error_px:
            diagnostics["assignment_rejected_reprojection"] += 1
            return

        points_3d.append(pt3d)
        errors.append(err_value)
        diagnostics["assignment_matches"] += 1
        diagnostics["triangulated_pairs"] += 1

    # ---------------------------------------------------------------------- #
    # Paired-frame triangulation (epipolar correspondence)
    # ---------------------------------------------------------------------- #

    def triangulate_paired_frames(
        self,
        paired_frames,
        blob_matcher=None,
    ) -> Tuple[List[np.ndarray], List[float]]:
        """
        Triangulate all matched blobs from paired frames using epipolar
        geometry for correspondence.

        Replaces the previous index-based approach that silently produced
        wrong 3D points when blob detection order differed across cameras.

        Args:
            paired_frames: PairedFrames object from receiver.
            blob_matcher:  Unused; kept for API compatibility.

        Returns:
            (list of 3D points, list of mean reprojection errors).
        """
        points_3d: List[np.ndarray] = []
        errors: List[float] = []
        diagnostics = _empty_assignment_diagnostics()
        self._last_assignment_diagnostics = dict(diagnostics)

        camera_ids = [
            cid
            for cid in paired_frames.camera_ids
            if cid in self.camera_params
        ]
        if len(camera_ids) < 2:
            self._last_assignment_diagnostics = dict(diagnostics)
            return points_3d, errors

        # Collect blobs per camera
        blobs: Dict[str, List[Tuple[float, float]]] = {}
        for cam_id in camera_ids:
            frame = paired_frames.frames.get(cam_id)
            if frame is not None:
                blobs[cam_id] = [(b["x"], b["y"]) for b in frame.blobs]

        active_cams = [c for c in camera_ids if blobs.get(c)]
        if len(active_cams) < 2:
            self._last_assignment_diagnostics = dict(diagnostics)
            return points_3d, errors

        # Use the camera with the most blobs as the reference
        ref_cam = max(active_cams, key=lambda c: len(blobs[c]))
        ref_blobs = blobs[ref_cam]

        if len(active_cams) == 2:
            other_cam = next(cam_id for cam_id in active_cams if cam_id != ref_cam)
            matches = self._match_blobs_symmetric_epipolar(
                ref_cam,
                other_cam,
                ref_blobs,
                blobs[other_cam],
                diagnostics,
            )
            for ref_idx, other_idx in matches.items():
                self._append_triangulated_observation(
                    [ref_blobs[ref_idx], blobs[other_cam][other_idx]],
                    [ref_cam, other_cam],
                    points_3d,
                    errors,
                    diagnostics,
                )
            self._last_assignment_diagnostics = dict(diagnostics)
            return points_3d, errors

        # For 3+ cameras, keep the reference-camera flow but solve each
        # reference→camera edge with one-to-one assignment. Observations that
        # share a reference blob are triangulated together.
        matches_by_camera: Dict[str, Dict[int, int]] = {}
        for cam_id in active_cams:
            if cam_id == ref_cam:
                continue
            matches_by_camera[cam_id] = self._match_blobs_symmetric_epipolar(
                ref_cam,
                cam_id,
                ref_blobs,
                blobs[cam_id],
                diagnostics,
            )

        for ref_idx, ref_pt in enumerate(ref_blobs):
            image_pts = [ref_pt]
            cam_ids_for_pt = [ref_cam]
            for cam_id, matches in matches_by_camera.items():
                other_idx = matches.get(ref_idx)
                if other_idx is None:
                    continue
                image_pts.append(blobs[cam_id][other_idx])
                cam_ids_for_pt.append(cam_id)
            if len(image_pts) < 2:
                continue
            self._append_triangulated_observation(
                image_pts,
                cam_ids_for_pt,
                points_3d,
                errors,
                diagnostics,
            )

        self._last_assignment_diagnostics = dict(diagnostics)
        return points_3d, errors

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

    def __init__(self) -> None:
        self.camera_params: Dict[str, CameraParams] = {}
        self.triangulator: Optional[Triangulator] = None
        self.coordinate_frame = "camera_similarity"
        self.coordinate_origin = "reference_camera"
        self.coordinate_origin_source = "extrinsics_pose_reference"

    def load_calibration(self, filepath: str) -> int:
        """Load camera calibration data. Returns number of cameras loaded."""
        calibrations = CalibrationLoader.load_intrinsics(filepath)

        if not calibrations:
            calibrations = CalibrationLoader.load_from_reference_format(filepath)

        self.camera_params = {}
        self.coordinate_frame = "camera_similarity"
        self.coordinate_origin = "reference_camera"
        self.coordinate_origin_source = "extrinsics_pose_reference"
        for cam_id, calib in calibrations.items():
            if isinstance(calib, CameraParams):
                self.camera_params[cam_id] = calib
            else:
                self.camera_params[cam_id] = CalibrationLoader.to_camera_params(
                    calib
                )

        extrinsics = CalibrationLoader.load_extrinsics(filepath, include_meta=True)
        if extrinsics:
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
            self.triangulator = Triangulator(self.camera_params)

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

    def process_paired_frames(self, paired_frames) -> Dict[str, Any]:
        """Process paired frames to extract 3D points."""
        if not self.triangulator:
            return {"error": "No calibration loaded"}

        points_3d, errors = self.triangulator.triangulate_paired_frames(
            paired_frames
        )
        assignment_diagnostics = self.triangulator.last_assignment_diagnostics

        return {
            "timestamp": paired_frames.timestamp,
            "camera_ids": paired_frames.camera_ids,
            "points_3d": points_3d,
            "reprojection_errors": errors,
            "assignment_diagnostics": assignment_diagnostics,
            "mean_error": float(np.mean(errors)) if errors else 0.0,
            "point_count": len(points_3d),
        }

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return geometry diagnostics for status/log snapshots."""
        if self.triangulator is None:
            return {"assignment": _empty_assignment_diagnostics()}
        return {"assignment": self.triangulator.last_assignment_diagnostics}

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
