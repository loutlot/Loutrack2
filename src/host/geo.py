"""
Geometry module for 3D reconstruction from multi-camera 2D observations.

Provides functionality to:
- Load and manage camera intrinsic/extrinsic parameters
- Triangulate 3D points from 2D observations (DLT)
- Compute reprojection errors for quality assessment
- Simple correspondence matching based on epipolar geometry
"""

import json
import numpy as np
import cv2 as cv
from pathlib import Path
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
from scipy import linalg


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
        """Compute projection matrix P = K @ [R|t]."""
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
        """
        Load intrinsic calibration from loutrack2 schema format.
        
        Args:
            filepath: Path to calibration_intrinsics_v1.json or directory
            
        Returns:
            Dict mapping camera_id to calibration data
        """
        path = Path(filepath)
        
        # If directory, look for calibration files
        if path.is_dir():
            calibrations = {}
            for f in path.glob("*.json"):
                try:
                    data = json.load(open(f))
                    if "camera_id" in data:
                        calibrations[data["camera_id"]] = data
                except:
                    continue
            return calibrations
        
        # Single file
        with open(path, 'r') as f:
            data = json.load(f)
        
        # Single camera or multiple?
        if isinstance(data, list):
            return {d.get("camera_id", f"cam_{i}"): d for i, d in enumerate(data)}
        elif "camera_id" in data:
            return {data["camera_id"]: data}
        else:
            return {"default": data}
    
    @staticmethod
    def to_camera_params(calibration: Dict[str, Any]) -> CameraParams:
        """
        Convert calibration JSON to CameraParams object.
        
        Args:
            calibration: Calibration dict from load_intrinsics
            
        Returns:
            CameraParams with intrinsic matrix and distortion
        """
        # Extract camera matrix
        cam_matrix = calibration.get("camera_matrix", {})
        if "matrix" in cam_matrix:
            K = np.array(cam_matrix["matrix"], dtype=np.float64)
        else:
            fx = cam_matrix.get("fx", 500.0)
            fy = cam_matrix.get("fy", 500.0)
            cx = cam_matrix.get("cx", 640.0)
            cy = cam_matrix.get("cy", 480.0)
            K = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ], dtype=np.float64)
        
        # Extract distortion coefficients
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
        
        # Resolution
        res = calibration.get("resolution", {"width": 1280, "height": 960})
        resolution = (res.get("width", 1280), res.get("height", 960))
        
        return CameraParams(
            camera_id=calibration.get("camera_id", "unknown"),
            intrinsic_matrix=K,
            distortion_coeffs=dist,
            rotation=np.eye(3, dtype=np.float64),  # Identity (to be set by extrinsic calibration)
            translation=np.zeros(3, dtype=np.float64),  # Zero (to be set by extrinsic calibration)
            resolution=resolution
        )
    
    @staticmethod
    def load_from_reference_format(filepath: str) -> Dict[str, CameraParams]:
        """
        Load from reference implementation's camera-params.json format.
        
        Args:
            filepath: Path to camera-params.json
            
        Returns:
            Dict mapping camera index to CameraParams
        """
        with open(filepath, 'r') as f:
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
                translation=np.zeros(3, dtype=np.float64)
            )
        
        return params


class Triangulator:
    """
    Triangulate 3D points from 2D observations across multiple cameras.
    
    Uses Direct Linear Transform (DLT) algorithm via SVD decomposition.
    """
    
    def __init__(self, camera_params: Dict[str, CameraParams]):
        """
        Initialize triangulator with camera parameters.
        
        Args:
            camera_params: Dict mapping camera_id to CameraParams
        """
        self.camera_params = camera_params
    
    def triangulate_point(
        self,
        image_points: List[Tuple[float, float]],
        camera_ids: List[str]
    ) -> Optional[np.ndarray]:
        """
        Triangulate a single 3D point from 2D observations.
        
        Uses DLT (Direct Linear Transform) with SVD.
        
        Args:
            image_points: List of 2D points [(x, y), ...]
            camera_ids: List of camera IDs corresponding to each point
            
        Returns:
            3D point as numpy array [x, y, z] or None if insufficient data
        """
        if len(image_points) < 2:
            return None
        
        # Filter valid points
        valid_points = []
        valid_cameras = []
        for pt, cam_id in zip(image_points, camera_ids):
            if pt is not None and cam_id in self.camera_params:
                valid_points.append(pt)
                valid_cameras.append(cam_id)
        
        if len(valid_points) < 2:
            return None
        
        # Build DLT matrix
        A = []
        for pt, cam_id in zip(valid_points, valid_cameras):
            cam = self.camera_params[cam_id]
            P = cam.projection_matrix
            u, v = pt[0], pt[1]
            
            # DLT equations: v * P3 - P2, P1 - u * P3
            A.append(v * P[2, :] - P[1, :])
            A.append(P[0, :] - u * P[2, :])
        
        A = np.array(A, dtype=np.float64)
        
        # Solve via SVD
        try:
            _, _, Vh = linalg.svd(A.T @ A, full_matrices=False)
            X_homogeneous = Vh[3, :]
            
            # Convert from homogeneous coordinates
            if abs(X_homogeneous[3]) < 1e-10:
                return None
            
            X = X_homogeneous[:3] / X_homogeneous[3]
            return X
            
        except Exception:
            return None
    
    def triangulate_points(
        self,
        image_points_list: List[List[Tuple[float, float]]],
        camera_ids: List[str]
    ) -> List[Optional[np.ndarray]]:
        """
        Triangulate multiple 3D points.
        
        Args:
            image_points_list: List of point lists, one per point to triangulate
            camera_ids: Camera IDs (same order for all points)
            
        Returns:
            List of 3D points or None for each input
        """
        results = []
        for image_points in image_points_list:
            pt = self.triangulate_point(image_points, camera_ids)
            results.append(pt)
        return results
    
    def triangulate_paired_frames(
        self,
        paired_frames,
        blob_matcher=None
    ) -> Tuple[List[np.ndarray], List[float]]:
        """
        Triangulate all points from paired frames.
        
        Args:
            paired_frames: PairedFrames object from receiver
            blob_matcher: Optional matcher for blob correspondence
            
        Returns:
            Tuple of (list of 3D points, list of reprojection errors)
        """
        points_3d = []
        errors = []
        
        camera_ids = paired_frames.camera_ids
        
        # Simple approach: triangulate all blob combinations
        # In practice, you'd want proper correspondence matching
        
        # Get blobs from each camera
        all_blobs = {}
        for cam_id, frame in paired_frames.frames.items():
            all_blobs[cam_id] = [(b['x'], b['y']) for b in frame.blobs]
        
        # For now, just triangulate first blob from each camera
        # TODO: Implement proper correspondence matching
        if len(all_blobs) >= 2:
            min_blobs = min(len(blobs) for blobs in all_blobs.values())
            
            for i in range(min_blobs):
                image_points = [all_blobs[cam_id][i] for cam_id in camera_ids]
                point_3d = self.triangulate_point(image_points, camera_ids)
                
                if point_3d is not None:
                    points_3d.append(point_3d)
                    
                    # Compute reprojection error
                    error = self.compute_reprojection_error(
                        image_points, point_3d, camera_ids
                    )
                    errors.append(error if error else 0.0)
        
        return points_3d, errors
    
    def compute_reprojection_error(
        self,
        image_points: List[Tuple[float, float]],
        object_point: np.ndarray,
        camera_ids: List[str]
    ) -> Optional[float]:
        """
        Compute mean reprojection error for a 3D point.
        
        Args:
            image_points: Observed 2D points
            object_point: 3D point
            camera_ids: Camera IDs
            
        Returns:
            Mean reprojection error in pixels, or None
        """
        errors = []
        
        for pt, cam_id in zip(image_points, camera_ids):
            if pt is None or cam_id not in self.camera_params:
                continue
            
            cam = self.camera_params[cam_id]
            
            # Project 3D point to 2D
            try:
                # Convert rotation matrix to Rodrigues vector for projectPoints
                rvec, _ = cv.Rodrigues(cam.rotation)
                projected, _ = cv.projectPoints(
                    object_point.reshape(1, 3).astype(np.float32),
                    rvec,
                    cam.translation,
                    cam.intrinsic_matrix,
                    cam.distortion_coeffs
                )
                projected = projected.squeeze()
                
                # Compute pixel error
                observed = np.array(pt, dtype=np.float64)
                error = np.linalg.norm(observed - projected)
                errors.append(error)
                
            except Exception:
                continue
        
        return np.mean(errors) if errors else None


class GeometryPipeline:
    """
    Complete geometry processing pipeline.
    
    Integrates calibration loading, triangulation, and error computation.
    """
    
    def __init__(self):
        """Initialize geometry pipeline."""
        self.camera_params: Dict[str, CameraParams] = {}
        self.triangulator: Optional[Triangulator] = None
    
    def load_calibration(self, filepath: str) -> int:
        """
        Load camera calibration data.
        
        Args:
            filepath: Path to calibration file or directory
            
        Returns:
            Number of cameras loaded
        """
        # Try loutrack2 format first
        calibrations = CalibrationLoader.load_intrinsics(filepath)
        
        if not calibrations:
            # Try reference format
            calibrations = CalibrationLoader.load_from_reference_format(filepath)
        
        # Convert to CameraParams
        self.camera_params = {}
        for cam_id, calib in calibrations.items():
            if isinstance(calib, CameraParams):
                self.camera_params[cam_id] = calib
            else:
                self.camera_params[cam_id] = CalibrationLoader.to_camera_params(calib)
        
        # Create triangulator
        if self.camera_params:
            self.triangulator = Triangulator(self.camera_params)
        
        return len(self.camera_params)
    
    def set_extrinsics(
        self,
        camera_id: str,
        rotation: np.ndarray,
        translation: np.ndarray
    ) -> None:
        """
        Set extrinsic parameters for a camera.
        
        Args:
            camera_id: Camera identifier
            rotation: 3x3 rotation matrix
            translation: 3-element translation vector
        """
        if camera_id in self.camera_params:
            self.camera_params[camera_id].rotation = rotation
            self.camera_params[camera_id].translation = translation
            
            # Update triangulator
            if self.triangulator:
                self.triangulator.camera_params = self.camera_params
    
    def process_paired_frames(self, paired_frames) -> Dict[str, Any]:
        """
        Process paired frames to extract 3D points.
        
        Args:
            paired_frames: PairedFrames from receiver
            
        Returns:
            Dict with points_3d, errors, and metadata
        """
        if not self.triangulator:
            return {"error": "No calibration loaded"}
        
        points_3d, errors = self.triangulator.triangulate_paired_frames(paired_frames)
        
        return {
            "timestamp": paired_frames.timestamp,
            "camera_ids": paired_frames.camera_ids,
            "points_3d": points_3d,
            "reprojection_errors": errors,
            "mean_error": np.mean(errors) if errors else 0.0,
            "point_count": len(points_3d)
        }
    
    def get_camera_ids(self) -> List[str]:
        """Get list of loaded camera IDs."""
        return list(self.camera_params.keys())
    
    def get_baseline(self, cam1_id: str, cam2_id: str) -> Optional[float]:
        """
        Compute baseline distance between two cameras.
        
        Args:
            cam1_id: First camera ID
            cam2_id: Second camera ID
            
        Returns:
            Baseline distance in world units, or None
        """
        if cam1_id not in self.camera_params or cam2_id not in self.camera_params:
            return None
        
        cam1 = self.camera_params[cam1_id]
        cam2 = self.camera_params[cam2_id]
        
        # Camera center in world: C = -R.T @ t
        C1 = -cam1.rotation.T @ cam1.translation
        C2 = -cam2.rotation.T @ cam2.translation
        
        return np.linalg.norm(C1 - C2)


def undistort_points(
    points: List[Tuple[float, float]],
    camera_params: CameraParams
) -> List[Tuple[float, float]]:
    """
    Undistort 2D points using camera calibration.
    
    Args:
        points: List of distorted 2D points
        camera_params: Camera parameters with distortion
        
    Returns:
        List of undistorted 2D points
    """
    if not points:
        return []
    
    pts = np.array(points, dtype=np.float64).reshape(-1, 1, 2)
    
    undistorted = cv.undistortPoints(
        pts,
        camera_params.intrinsic_matrix,
        camera_params.distortion_coeffs,
        None,
        camera_params.intrinsic_matrix
    )
    
    return [(p[0, 0], p[0, 1]) for p in undistorted]


def create_dummy_calibration(
    camera_ids: List[str],
    resolution: Tuple[int, int] = (1280, 960),
    focal_length: float = 500.0
) -> Dict[str, CameraParams]:
    """
    Create dummy calibration for testing without real calibration data.
    
    Args:
        camera_ids: List of camera identifiers
        resolution: Image resolution (width, height)
        focal_length: Focal length in pixels
        
    Returns:
        Dict mapping camera_id to CameraParams with identity extrinsics
    """
    params = {}
    width, height = resolution
    cx, cy = width / 2, height / 2
    
    for i, cam_id in enumerate(camera_ids):
        K = np.array([
            [focal_length, 0, cx],
            [0, focal_length, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        
        # Place cameras in a line for testing
        # Camera center in world: C = [i*0.5, 0, 0]
        # Translation: t = -R @ C (world->camera convention)
        # With R=I: t = -C
        camera_center = np.array([i * 0.5, 0, 0], dtype=np.float64)
        translation = -camera_center  # t = -R @ C = -C for R=I
        
        params[cam_id] = CameraParams(
            camera_id=cam_id,
            intrinsic_matrix=K,
            distortion_coeffs=np.zeros(5, dtype=np.float64),
            rotation=np.eye(3, dtype=np.float64),
            translation=translation,
            resolution=resolution
        )
    
    return params
