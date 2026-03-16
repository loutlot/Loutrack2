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

        pose_section = data.get("pose", {})
        cameras = (
            pose_section.get("camera_poses", [])
            if isinstance(pose_section, dict)
            else []
        )
        result: Dict[str, Dict[str, Any]] = {}
        for camera in cameras:
            camera_id = camera.get("camera_id")
            if camera_id:
                result[camera_id] = camera
        if include_meta:
            result["__meta__"] = data if isinstance(data, dict) else {}
        return result

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
        self._refresh_cached_matrices()

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
        Greedy nearest-epipolar-line matching.

        For each blob in the reference camera find the closest (by epipolar
        distance) unmatched blob in the other camera within threshold_px.

        Returns:
            Dict mapping ref_blob_index → other_blob_index.
        """
        used: set = set()
        matches: Dict[int, int] = {}
        for i, ref_pt in enumerate(ref_blobs):
            best_j, best_d = -1, threshold_px
            for j, other_pt in enumerate(other_blobs):
                if j in used:
                    continue
                d = self._epipolar_distance(F, ref_pt, other_pt)
                if d < best_d:
                    best_d = d
                    best_j = j
            if best_j >= 0:
                matches[i] = best_j
                used.add(best_j)
        return matches

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

        camera_ids = [
            cid
            for cid in paired_frames.camera_ids
            if cid in self.camera_params
        ]
        if len(camera_ids) < 2:
            return points_3d, errors

        # Collect blobs per camera
        blobs: Dict[str, List[Tuple[float, float]]] = {}
        for cam_id in camera_ids:
            frame = paired_frames.frames.get(cam_id)
            if frame is not None:
                blobs[cam_id] = [(b["x"], b["y"]) for b in frame.blobs]

        active_cams = [c for c in camera_ids if blobs.get(c)]
        if len(active_cams) < 2:
            return points_3d, errors

        # Use the camera with the most blobs as the reference
        ref_cam = max(active_cams, key=lambda c: len(blobs[c]))

        # Build fundamental matrices from ref_cam to every other camera
        F_matrices: Dict[str, Optional[np.ndarray]] = {}
        for cam_id in active_cams:
            if cam_id == ref_cam:
                continue
            F_matrices[cam_id] = self._compute_fundamental_matrix(
                ref_cam, cam_id
            )

        ref_blobs = blobs[ref_cam]

        # For each reference blob, collect epipolar-matched observations
        for ref_idx, ref_pt in enumerate(ref_blobs):
            image_pts: List[Tuple[float, float]] = [ref_pt]
            cam_ids_for_pt: List[str] = [ref_cam]

            for cam_id in active_cams:
                if cam_id == ref_cam:
                    continue
                F = F_matrices.get(cam_id)
                other = blobs[cam_id]
                if not other:
                    continue

                if F is not None:
                    # Find the epipolar-closest blob
                    best_j, best_d = -1, _EPIPOLAR_THRESHOLD_PX
                    for j, pt in enumerate(other):
                        d = self._epipolar_distance(F, ref_pt, pt)
                        if d < best_d:
                            best_d = d
                            best_j = j
                    if best_j >= 0:
                        image_pts.append(other[best_j])
                        cam_ids_for_pt.append(cam_id)
                else:
                    # Fundamental matrix unavailable — skip this camera
                    continue

            if len(image_pts) < 2:
                continue

            pt3d = self.triangulate_point(image_pts, cam_ids_for_pt)
            if pt3d is not None:
                points_3d.append(pt3d)
                err = self.compute_reprojection_error(
                    image_pts, pt3d, cam_ids_for_pt
                )
                errors.append(err if err is not None else 0.0)

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

    def load_calibration(self, filepath: str) -> int:
        """Load camera calibration data. Returns number of cameras loaded."""
        calibrations = CalibrationLoader.load_intrinsics(filepath)

        if not calibrations:
            calibrations = CalibrationLoader.load_from_reference_format(filepath)

        self.camera_params = {}
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

        return {
            "timestamp": paired_frames.timestamp,
            "camera_ids": paired_frames.camera_ids,
            "points_3d": points_3d,
            "reprojection_errors": errors,
            "mean_error": float(np.mean(errors)) if errors else 0.0,
            "point_count": len(points_3d),
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
