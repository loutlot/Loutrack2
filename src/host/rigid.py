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
from itertools import permutations
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import cdist
from scipy.spatial import cKDTree
import threading


@dataclass
class MarkerPattern:
    """Known marker configuration for a rigid body."""
    name: str
    marker_positions: np.ndarray  # Nx3 array of marker positions in body frame
    marker_diameter: float = 0.014  # 14mm default
    metadata: Dict[str, Any] = field(default_factory=dict)
    
    @property
    def num_markers(self) -> int:
        return len(self.marker_positions)
    
    @property
    def centroid(self) -> np.ndarray:
        return np.mean(self.marker_positions, axis=0)


def marker_pattern_from_points(
    name: str,
    points_world: np.ndarray,
    *,
    marker_diameter: float = 0.014,
    metadata: Optional[Dict[str, Any]] = None,
) -> MarkerPattern:
    """Build a rigid marker pattern from a sampled set of world-space points."""
    points = np.asarray(points_world, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points_world must be an Nx3 array")
    if len(points) < 3:
        raise ValueError("at least three points are required to define a rigid body")
    if not np.isfinite(points).all():
        raise ValueError("points_world must contain only finite coordinates")
    if not np.isfinite(marker_diameter) or float(marker_diameter) <= 0.0:
        raise ValueError("marker_diameter must be a finite positive value")
    centroid = np.mean(points, axis=0)
    local_points = points - centroid
    return MarkerPattern(
        name=str(name).strip(),
        marker_positions=local_points,
        marker_diameter=float(marker_diameter),
        metadata=dict(metadata or {}),
    )


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

# Additional patterns for other body parts
# All positions in meters
HEAD_PATTERN = MarkerPattern(
    name="head",
    marker_positions=np.array([
        [36.0, 12.0, 18.0],
        [-34.0, 10.0, 16.0],
        [2.0, -38.0, 17.0],
        [0.0, 18.0, 66.0],
    ], dtype=np.float64) * 0.001,  # Convert mm to meters
    marker_diameter=0.014  # 14mm in meters
)

CHEST_PATTERN = MarkerPattern(
    name="chest",
    marker_positions=np.array([
        [46.0, -6.0, 14.0],
        [-10.0, 42.0, 16.0],
        [-42.0, -18.0, 15.0],
        [18.0, 14.0, 62.0],
    ], dtype=np.float64) * 0.001,  # Convert mm to meters
    marker_diameter=0.014  # 14mm in meters
)

LEFT_FOOT_PATTERN = MarkerPattern(
    name="left_foot",
    marker_positions=np.array([
        [54.0, -16.0, 13.0],
        [-20.0, -38.0, 15.0],
        [-40.0, 22.0, 14.0],
        [8.0, 28.0, 61.0],
    ], dtype=np.float64) * 0.001,  # Convert mm to meters
    marker_diameter=0.014  # 14mm in meters
)

RIGHT_FOOT_PATTERN = MarkerPattern(
    name="right_foot",
    marker_positions=np.array([
        [24.0, -42.0, 12.0],
        [-36.0, -20.0, 18.0],
        [28.0, 18.0, 16.0],
        [-18.0, 30.0, 61.0],
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
        cluster_radius_m: float = 0.08,
        min_samples: int = 3
    ):
        """
        Initialize clusterer.
        
        Args:
            marker_diameter: Marker diameter in meters (default 14mm)
            eps_scale: Legacy multiplier for marker-diameter clustering
            cluster_radius_m: Rigid-body clustering radius in meters
            min_samples: Minimum points for a cluster
        """
        self.marker_diameter = marker_diameter
        self.eps_scale = eps_scale
        self.cluster_radius_m = float(cluster_radius_m)
        self.eps = self.cluster_radius_m
        self.min_samples = min_samples
    
    def cluster(self, points: np.ndarray) -> List[np.ndarray]:
        """
        Cluster 3D points using DBSCAN via a KD-tree for O(n log n) queries.

        Args:
            points: Nx3 array of 3D points

        Returns:
            List of point arrays, one per cluster (noise points excluded).
        """
        n = len(points)
        if n == 0:
            return []
        if n < self.min_samples:
            return [points]

        # Build KD-tree for efficient radius queries
        tree = cKDTree(points)
        neighbor_lists = tree.query_ball_point(points, self.eps)

        labels = np.full(n, -1, dtype=np.intp)
        cluster_id = 0

        for i in range(n):
            if labels[i] != -1:
                continue
            neighbors = neighbor_lists[i]
            if len(neighbors) < self.min_samples:
                continue  # noise candidate — may be absorbed later

            labels[i] = cluster_id
            seed_set = list(neighbors)

            j = 0
            while j < len(seed_set):
                q = seed_set[j]
                if labels[q] == -1:
                    labels[q] = cluster_id
                    q_neighbors = neighbor_lists[q]
                    if len(q_neighbors) >= self.min_samples:
                        for nb in q_neighbors:
                            if labels[nb] == -1:
                                seed_set.append(nb)
                elif labels[q] != cluster_id:
                    labels[q] = cluster_id
                j += 1

            cluster_id += 1

        return [points[labels == c] for c in range(cluster_id)]


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
        
        # If same number, solve correspondence via inter-point distance profiles.
        # This is pose-invariant: distances between markers are unchanged by rotation
        # and translation, so we can match them without knowing the current pose.
        # Each point's "fingerprint" is its sorted distance profile to all other points;
        # Hungarian assignment on these fingerprints finds the best one-to-one mapping.
        # This correctly handles blob detection order changes across frames.
        if len(observed_points) == len(reference_points):
            obs_dists = cdist(observed_points, observed_points)
            ref_dists = cdist(reference_points, reference_points)
            # Sort each row → permutation-invariant distance profile per point
            obs_profiles = np.sort(obs_dists, axis=1)
            ref_profiles = np.sort(ref_dists, axis=1)
            # cost[i, j] = dissimilarity between obs point i and reference point j
            cost = cdist(obs_profiles, ref_profiles)
            row_ind, col_ind = linear_sum_assignment(cost)
            # reordered_obs[ref_idx] = observed_points[obs_idx]
            reordered_obs = np.empty_like(observed_points)
            for obs_idx, ref_idx in zip(row_ind, col_ind):
                reordered_obs[ref_idx] = observed_points[obs_idx]
            R, t, error = KabschEstimator.estimate(reordered_obs, reference_points)
            return reordered_obs, reference_points, error
        
        # For different counts, find best subset
        n_ref = len(reference_points)
        n_obs = len(observed_points)
        
        if n_obs < n_ref:
            # More reference points than observed — select the globally optimal
            # one-to-one subset of reference points via the Hungarian algorithm.
            distances = cdist(observed_points, reference_points)  # n_obs × n_ref
            _, col_ind = linear_sum_assignment(distances)
            matched_ref = reference_points[col_ind]

            R, t, error = KabschEstimator.estimate(observed_points, matched_ref)
            return observed_points, matched_ref, error
        
        else:
            # More observed than reference - find best subset
            # Use distance-based matching
            from itertools import combinations
            
            best_error = float('inf')
            best_obs = None
            
            # For small datasets, try all combinations AND permutations within each subset.
            if n_obs <= 10 and n_ref <= 5:
                for combo in combinations(range(n_obs), n_ref):
                    obs_subset = observed_points[list(combo)]
                    for perm in permutations(range(n_ref)):
                        obs_perm = obs_subset[list(perm)]
                        try:
                            _, _, error = KabschEstimator.estimate(obs_perm, reference_points)
                            if error < best_error:
                                best_error = error
                                best_obs = obs_perm
                        except Exception:
                            continue
                
                if best_obs is not None:
                    return best_obs, reference_points, best_error
            
            # For larger datasets, use the Hungarian algorithm for a globally
            # optimal one-to-one assignment (replaces greedy nearest-neighbour).
            distances = cdist(reference_points, observed_points)  # n_ref × n_obs
            _, col_ind = linear_sum_assignment(distances)
            matched_obs = observed_points[col_ind]
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

        # Tracked state
        self._position = np.zeros(3)
        self._velocity = np.zeros(3)
        self._quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # [w, x, y, z]
        self._last_valid_timestamp: int = 0  # microseconds

        # Tracking statistics
        self.track_count = 0
        self.lost_frames = 0
        self.total_frames = 0
    
    def update(self, pose: RigidBodyPose) -> None:
        """Update tracker with new pose estimate, including velocity estimation."""
        with self._lock:
            self._pose_history.append(pose)
            self.total_frames += 1

            if pose.valid:
                if self.track_count > 0 and self._last_valid_timestamp > 0:
                    dt_s = (pose.timestamp - self._last_valid_timestamp) / 1_000_000.0
                    # Only update velocity if dt is sane (0–500 ms)
                    if 0.0 < dt_s < 0.5:
                        self._velocity = (pose.position - self._position) / dt_s
                    else:
                        self._velocity = np.zeros(3)
                self._position = pose.position.copy()
                self._quaternion = pose.quaternion.copy()
                self._last_valid_timestamp = pose.timestamp
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
        marker_diameter: float = 0.014,
        cluster_radius_m: float = 0.08,
        max_rms_error_m: float = 0.055,
    ):
        """
        Initialize estimator.
        
        Args:
            patterns: List of MarkerPatterns to track
            marker_diameter: Default marker diameter
            cluster_radius_m: Radius for grouping markers into rigid-body candidates
            max_rms_error_m: Maximum allowed rigid-fit RMS error in meters
        """
        if not np.isfinite(max_rms_error_m) or float(max_rms_error_m) <= 0.0:
            raise ValueError("max_rms_error_m must be a finite positive value")
        self.patterns = patterns or [WAIST_PATTERN]
        self.marker_diameter = marker_diameter
        self.cluster_radius_m = float(cluster_radius_m)
        self.max_rms_error_m = float(max_rms_error_m)
        
        self.clusterer = PointClusterer(
            marker_diameter=marker_diameter,
            cluster_radius_m=self.cluster_radius_m,
        )
        
        # Create trackers for each pattern
        self.trackers: Dict[str, RigidBodyTracker] = {
            p.name: RigidBodyTracker(p) for p in self.patterns
        }
        
        # Kabsch estimator
        self.kabsch = KabschEstimator()

    def set_patterns(self, patterns: List[MarkerPattern]) -> None:
        """Replace tracked rigid-body patterns while preserving existing trackers when possible."""
        next_patterns = list(patterns or [])
        next_trackers: Dict[str, RigidBodyTracker] = {}
        for pattern in next_patterns:
            tracker = self.trackers.get(pattern.name)
            if tracker is None:
                tracker = RigidBodyTracker(pattern)
            else:
                tracker.pattern = pattern
            next_trackers[pattern.name] = tracker
        self.patterns = next_patterns
        self.trackers = next_trackers
    
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
        if len(self.patterns) == 1 and len(points_3d) >= 3:
            min_points = max(3, self.patterns[0].num_markers - 1)
            if not any(len(cluster) >= min_points for cluster in clusters):
                clusters.append(points_3d)
        
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
                
                if (
                    pose.valid
                    and pose.rms_error <= self.max_rms_error_m
                    and pose.rms_error < best_error
                ):
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
