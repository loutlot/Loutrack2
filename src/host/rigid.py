"""
Rigid body tracking module for pose estimation from 3D point clouds.

Provides functionality to:
- Cluster 3D points using DBSCAN
- Estimate rigid body pose using Kabsch algorithm
- Fallback to PnP for limited observations
- Track rigid bodies over time with Kalman filtering
"""

import numpy as np
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass, field
from collections import deque
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import cdist
import threading


@dataclass
class MarkerPattern:
    """Known marker configuration for a rigid body."""
    name: str
    marker_positions: np.ndarray  # Nx3 array of marker positions in body frame
    marker_diameter: float = 0.014  # 14mm default
    
    @property
    def num_markers(self) -> int:
        return len(self.marker_positions)
    
    @property
    def centroid(self) -> np.ndarray:
        return np.mean(self.marker_positions, axis=0)


# Predefined rigid body patterns (from context/request.md)
# All positions in meters
WAIST_PATTERN = MarkerPattern(
    name="waist",
    marker_positions=np.array([
        [51.962, -30.00, 7.00],     # 0 右前方
        [42.426, 42.426, 7.00],     # 1 右後方
        [-51.962, -30.00, 7.00],    # 2 左前方
        [-30.00, 30.00, 49.426],    # 3 上方突出（非対称キー）
    ], dtype=np.float64) * 0.001,  # Convert mm to meters
    marker_diameter=0.014  # 14mm in meters
)

# Simplified patterns for other body parts (to be refined)
# All positions in meters
HEAD_PATTERN = MarkerPattern(
    name="head",
    marker_positions=np.array([
        [40.0, 0.0, 0.0],
        [-40.0, 0.0, 0.0],
        [0.0, 40.0, 0.0],
        [0.0, 0.0, 50.0],
    ], dtype=np.float64) * 0.001,  # Convert mm to meters
    marker_diameter=0.014  # 14mm in meters
)


@dataclass
class RigidBodyPose:
    """Estimated pose of a rigid body."""
    timestamp: int
    position: np.ndarray  # 3D position
    rotation: np.ndarray  # 3x3 rotation matrix
    quaternion: np.ndarray  # [w, x, y, z]
    rms_error: float = 0.0
    observed_markers: int = 0
    valid: bool = True
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": self.timestamp,
            "position": self.position.tolist(),
            "quaternion": self.quaternion.tolist(),
            "rms_error": self.rms_error,
            "observed_markers": self.observed_markers,
            "valid": self.valid
        }


class PointClusterer:
    """
    Cluster 3D points using DBSCAN algorithm.
    
    Separates individual rigid bodies from a combined point cloud.
    """
    
    def __init__(
        self,
        marker_diameter: float = 0.014,
        eps_scale: float = 0.8,
        min_samples: int = 3
    ):
        """
        Initialize clusterer.
        
        Args:
            marker_diameter: Marker diameter in meters (default 14mm)
            eps_scale: Multiplier for eps = marker_diameter * eps_scale
            min_samples: Minimum points for a cluster
        """
        self.marker_diameter = marker_diameter
        self.eps = marker_diameter * eps_scale
        self.min_samples = min_samples
    
    def cluster(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster 3D points using DBSCAN.
        
        Args:
            points: Nx3 array of 3D points
            
        Returns:
            List of point arrays, one per cluster
        """
        if len(points) < self.min_samples:
            return [points] if len(points) > 0 else []
        
        # Compute pairwise distances
        distances = cdist(points, points)
        
        # DBSCAN implementation
        labels = np.full(len(points), -1, dtype=int)
        cluster_id = 0
        
        for i in range(len(points)):
            if labels[i] != -1:
                continue
            
            # Find neighbors
            neighbors = np.where(distances[i] <= self.eps)[0]
            
            if len(neighbors) < self.min_samples:
                continue  # Noise point
            
            # Start new cluster
            labels[neighbors] = cluster_id
            
            # Expand cluster
            seed_set = list(neighbors)
            seed_set.remove(i)
            
            j = 0
            while j < len(seed_set):
                q = seed_set[j]
                
                q_neighbors = np.where(distances[q] <= self.eps)[0]
                
                if len(q_neighbors) >= self.min_samples:
                    for n in q_neighbors:
                        if labels[n] == -1:
                            labels[n] = cluster_id
                            seed_set.append(n)
                
                j += 1
            
            cluster_id += 1
        
        # Extract clusters
        clusters = []
        for c in range(cluster_id):
            cluster_points = points[labels == c]
            if len(cluster_points) > 0:
                clusters.append(cluster_points)
        
        return clusters


class KabschEstimator:
    """
    Estimate rigid body pose using Kabsch algorithm.
    
    Finds optimal rotation and translation to align observed points
    with known marker positions.
    """
    
    @staticmethod
    def estimate(
        observed_points: np.ndarray,
        reference_points: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Estimate rotation and translation using Kabsch algorithm.
        
        Args:
            observed_points: Nx3 array of observed 3D points
            reference_points: Nx3 array of reference marker positions
            
        Returns:
            Tuple of (rotation_matrix, translation_vector, rms_error)
        """
        if len(observed_points) != len(reference_points):
            raise ValueError("Point counts must match")
        
        if len(observed_points) < 3:
            raise ValueError("Need at least 3 points")
        
        # Center the point sets
        obs_centroid = np.mean(observed_points, axis=0)
        ref_centroid = np.mean(reference_points, axis=0)
        
        obs_centered = observed_points - obs_centroid
        ref_centered = reference_points - ref_centroid
        
        # Compute cross-covariance matrix
        H = obs_centered.T @ ref_centered
        
        # SVD
        U, S, Vt = np.linalg.svd(H)
        
        # Compute rotation
        R = Vt.T @ U.T
        
        # Handle reflection case
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Compute translation
        t = ref_centroid - R @ obs_centroid
        
        # Compute RMS error
        transformed = (R @ observed_points.T).T + t
        errors = np.linalg.norm(transformed - reference_points, axis=1)
        rms_error = np.sqrt(np.mean(errors ** 2))
        
        return R, t, rms_error
    
    @staticmethod
    def find_correspondence(
        observed_points: np.ndarray,
        reference_points: np.ndarray,
        max_iterations: int = 100
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Find best correspondence between observed and reference points.
        
        Uses iterative closest point (ICP) approach for initial alignment.
        
        Args:
            observed_points: Mx3 array of observed points
            reference_points: Nx3 array of reference points
            max_iterations: Maximum ICP iterations
            
        Returns:
            Tuple of (matched_observed, matched_reference, rms_error)
        """
        if len(observed_points) < 3 or len(reference_points) < 3:
            return np.array([]), np.array([]), float('inf')
        
        # If same number, try direct matching with Kabsch
        if len(observed_points) == len(reference_points):
            R, t, error = KabschEstimator.estimate(observed_points, reference_points)
            return observed_points, reference_points, error
        
        # For different counts, find best subset
        n_ref = len(reference_points)
        n_obs = len(observed_points)
        
        if n_obs < n_ref:
            # More reference points than observed - can't fully estimate
            # Try to match observed to closest reference points
            best_error = float('inf')
            best_R, best_t = np.eye(3), np.zeros(3)
            
            # Use all observed points, find closest reference
            distances = cdist(observed_points, reference_points)
            closest_refs = np.argmin(distances, axis=1)
            matched_ref = reference_points[closest_refs]
            
            R, t, error = KabschEstimator.estimate(observed_points, matched_ref)
            return observed_points, matched_ref, error
        
        else:
            # More observed than reference - find best subset
            # Use distance-based matching
            from itertools import combinations
            
            best_error = float('inf')
            best_obs = None
            
            # For small datasets, try all combinations
            if n_obs <= 10 and n_ref <= 5:
                for combo in combinations(range(n_obs), n_ref):
                    obs_subset = observed_points[list(combo)]
                    try:
                        R, t, error = KabschEstimator.estimate(obs_subset, reference_points)
                        if error < best_error:
                            best_error = error
                            best_obs = obs_subset
                    except:
                        continue
                
                if best_obs is not None:
                    return best_obs, reference_points, best_error
            
            # For larger datasets, use greedy matching
            distances = cdist(reference_points, observed_points)
            matched_obs_indices = []
            
            for i in range(n_ref):
                # Find closest unmatched observed point
                min_dist = float('inf')
                min_j = -1
                for j in range(n_obs):
                    if j not in matched_obs_indices:
                        if distances[i, j] < min_dist:
                            min_dist = distances[i, j]
                            min_j = j
                matched_obs_indices.append(min_j)
            
            matched_obs = observed_points[matched_obs_indices]
            R, t, error = KabschEstimator.estimate(matched_obs, reference_points)
            
            return matched_obs, reference_points, error


class PnPEstimator:
    """
    Estimate pose using PnP (Perspective-n-Point) algorithm.
    
    Fallback when insufficient 3D points for Kabsch.
    """
    
    @staticmethod
    def estimate(
        image_points: List[Tuple[float, float]],
        object_points: np.ndarray,
        camera_params
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Estimate pose from 2D-3D correspondences using PnP.
        
        Args:
            image_points: List of 2D image coordinates
            object_points: Nx3 array of 3D object points
            camera_params: CameraParams with intrinsics
            
        Returns:
            Tuple of (rotation_matrix, translation_vector, reprojection_error)
        """
        import cv2 as cv
        
        if len(image_points) < 4:
            raise ValueError("Need at least 4 points for PnP")
        
        image_points = np.array(image_points, dtype=np.float64).reshape(-1, 1, 2)
        object_points = object_points.reshape(-1, 1, 3).astype(np.float32)
        
        success, rvec, tvec = cv.solvePnP(
            object_points,
            image_points,
            camera_params.intrinsic_matrix,
            camera_params.distortion_coeffs
        )
        
        if not success:
            raise RuntimeError("PnP failed")
        
        # Convert rotation vector to matrix
        R, _ = cv.Rodrigues(rvec)
        t = tvec.flatten()
        
        # Compute reprojection error
        projected, _ = cv.projectPoints(
            object_points,
            rvec,
            tvec,
            camera_params.intrinsic_matrix,
            camera_params.distortion_coeffs
        )
        
        errors = np.linalg.norm(image_points.squeeze() - projected.squeeze(), axis=1)
        rms_error = np.sqrt(np.mean(errors ** 2))
        
        return R, t, rms_error


class RigidBodyTracker:
    """
    Track a single rigid body over time.
    
    Maintains pose history and provides prediction for occlusion handling.
    """
    
    def __init__(
        self,
        pattern: MarkerPattern,
        history_size: int = 30
    ):
        """
        Initialize tracker.
        
        Args:
            pattern: MarkerPattern for this rigid body
            history_size: Number of poses to keep
        """
        self.pattern = pattern
        self.history_size = history_size
        
        self._pose_history: deque = deque(maxlen=history_size)
        self._lock = threading.Lock()
        
        # Kalman filter state (simplified)
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        
        # Tracking statistics
        self.track_count = 0
        self.lost_frames = 0
        self.total_frames = 0
    
    def update(self, pose: RigidBodyPose) -> None:
        """Update tracker with new pose estimate."""
        with self._lock:
            self._pose_history.append(pose)
            self.total_frames += 1
            
            if pose.valid:
                self._position = pose.position.copy()
                self._quaternion = pose.quaternion.copy()
                self.track_count += 1
                self.lost_frames = 0
            else:
                self.lost_frames += 1
    
    def predict(self, dt: float = 0.016) -> RigidBodyPose:
        """
        Predict current pose based on history.
        
        Args:
            dt: Time since last observation
            
        Returns:
            Predicted RigidBodyPose
        """
        with self._lock:
            # Simple linear prediction
            predicted_position = self._position + self._velocity * dt
            
            return RigidBodyPose(
                timestamp=0,  # Will be set by caller
                position=predicted_position,
                rotation=Rotation.from_quat([
                    self._quaternion[1], self._quaternion[2],
                    self._quaternion[3], self._quaternion[0]
                ]).as_matrix(),
                quaternion=self._quaternion.copy(),
                rms_error=0.0,
                observed_markers=0,
                valid=False  # Mark as prediction
            )
    
    def get_latest_pose(self) -> Optional[RigidBodyPose]:
        """Get most recent valid pose."""
        with self._lock:
            for pose in reversed(self._pose_history):
                if pose.valid:
                    return pose
            return None
    
    @property
    def is_tracking(self) -> bool:
        """Check if tracker is actively tracking."""
        return self.lost_frames < 10  # Lost for less than 10 frames
    
    @property
    def confidence(self) -> float:
        """Get tracking confidence (0-1)."""
        if self.total_frames == 0:
            return 0.0
        return self.track_count / self.total_frames


class RigidBodyEstimator:
    """
    Complete rigid body estimation pipeline.
    
    Integrates clustering, pose estimation, and tracking.
    """
    
    def __init__(
        self,
        patterns: Optional[List[MarkerPattern]] = None,
        marker_diameter: float = 0.014
    ):
        """
        Initialize estimator.
        
        Args:
            patterns: List of MarkerPatterns to track
            marker_diameter: Default marker diameter
        """
        self.patterns = patterns or [WAIST_PATTERN]
        self.marker_diameter = marker_diameter
        
        self.clusterer = PointClusterer(marker_diameter=marker_diameter)
        
        # Create trackers for each pattern
        self.trackers: Dict[str, RigidBodyTracker] = {
            p.name: RigidBodyTracker(p) for p in self.patterns
        }
        
        # Kabsch estimator
        self.kabsch = KabschEstimator()
    
    def estimate_pose(
        self,
        points_3d: np.ndarray,
        pattern: MarkerPattern,
        timestamp: int
    ) -> RigidBodyPose:
        """
        Estimate pose of a single rigid body.
        
        Args:
            points_3d: Nx3 array of 3D points
            pattern: MarkerPattern for this body
            timestamp: Frame timestamp
            
        Returns:
            RigidBodyPose estimate
        """
        if len(points_3d) < 3:
            return RigidBodyPose(
                timestamp=timestamp,
                position=np.zeros(3),
                rotation=np.eye(3),
                quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                valid=False
            )
        
        try:
            # Find correspondence and estimate pose
            matched_obs, matched_ref, rms_error = KabschEstimator.find_correspondence(
                points_3d, pattern.marker_positions
            )
            
            if len(matched_obs) < 3:
                raise ValueError("Insufficient matched points")
            
            # Final Kabsch: estimate body->world transform
            # Swap args: estimate(ref, obs) returns transform X_world = R @ X_body + t
            R, t, error = KabschEstimator.estimate(matched_ref, matched_obs)
            # Convert to quaternion
            rot = Rotation.from_matrix(R)
            quat = rot.as_quat()  # [x, y, z, w]
            quat = np.array([quat[3], quat[0], quat[1], quat[2]])  # [w, x, y, z]
            
            return RigidBodyPose(
                timestamp=timestamp,
                position=t,
                rotation=R,
                quaternion=quat,
                rms_error=error,
                observed_markers=len(matched_obs),
                valid=True
            )
            
        except Exception as e:
            # Return invalid pose on failure
            return RigidBodyPose(
                timestamp=timestamp,
                position=np.zeros(3),
                rotation=np.eye(3),
                quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
                valid=False
            )
    
    def process_points(
        self,
        points_3d: np.ndarray,
        timestamp: int
    ) -> Dict[str, RigidBodyPose]:
        """
        Process 3D points and estimate poses for all tracked bodies.
        
        Args:
            points_3d: Combined Nx3 array from triangulation
            timestamp: Frame timestamp
            
        Returns:
            Dict mapping body name to RigidBodyPose
        """
        if len(points_3d) == 0:
            # Return predictions for all bodies
            return {
                name: tracker.predict()
                for name, tracker in self.trackers.items()
            }
        
        # Cluster points
        clusters = self.clusterer.cluster(points_3d)
        
        # Estimate pose for each pattern
        poses = {}
        used_clusters = set()
        
        for pattern in self.patterns:
            best_pose = None
            best_error = float('inf')
            best_cluster_idx = -1
            
            for idx, cluster in enumerate(clusters):
                if idx in used_clusters:
                    continue
                
                if len(cluster) < 3:
                    continue
                
                # Check if cluster size matches pattern
                if len(cluster) < pattern.num_markers - 1:
                    continue  # Too few points
                
                pose = self.estimate_pose(cluster, pattern, timestamp)
                
                if pose.valid and pose.rms_error < best_error:
                    best_pose = pose
                    best_error = pose.rms_error
                    best_cluster_idx = idx
            
            if best_pose is not None:
                poses[pattern.name] = best_pose
                used_clusters.add(best_cluster_idx)
                self.trackers[pattern.name].update(best_pose)
            else:
                # Use prediction
                predicted = self.trackers[pattern.name].predict()
                predicted.timestamp = timestamp
                poses[pattern.name] = predicted
                self.trackers[pattern.name].update(predicted)
        
        return poses
    
    def get_tracking_status(self) -> Dict[str, Dict[str, Any]]:
        """Get tracking status for all bodies."""
        return {
            name: {
                "is_tracking": tracker.is_tracking,
                "confidence": tracker.confidence,
                "lost_frames": tracker.lost_frames,
                "track_count": tracker.track_count
            }
            for name, tracker in self.trackers.items()
        }
