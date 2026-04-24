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
from enum import Enum
from itertools import permutations
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation
from scipy.spatial.distance import cdist
from scipy.spatial import cKDTree
import threading
import cv2 as cv


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


@dataclass
class PredictedPose:
    """Side-effect-free pose prediction used by stabilization phases."""

    timestamp: int
    position: np.ndarray
    rotation: np.ndarray
    quaternion: np.ndarray
    velocity: np.ndarray
    dt_s: float = 0.0
    valid: bool = False
    confidence: float = 0.0
    position_sigma_m: float = 0.0
    rotation_sigma_deg: float = 0.0
    lost_frames: int = 0

    def to_pose(self) -> RigidBodyPose:
        return RigidBodyPose(
            timestamp=self.timestamp,
            position=self.position.copy(),
            rotation=self.rotation.copy(),
            quaternion=self.quaternion.copy(),
            rms_error=0.0,
            observed_markers=0,
            valid=False,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": int(self.timestamp),
            "position": self.position.tolist(),
            "quaternion": self.quaternion.tolist(),
            "velocity": self.velocity.tolist(),
            "dt_s": float(self.dt_s),
            "valid": bool(self.valid),
            "confidence": float(self.confidence),
            "position_sigma_m": float(self.position_sigma_m),
            "rotation_sigma_deg": float(self.rotation_sigma_deg),
            "lost_frames": int(self.lost_frames),
        }


class TrackMode(Enum):
    """Rigid-body tracking lifecycle mode."""

    BOOT = "boot"
    CONTINUE = "continue"
    REACQUIRE = "reacquire"
    LOST = "lost"


@dataclass(frozen=True)
class TrackModeConfig:
    """Thresholds for mode transitions without changing pose acceptance yet."""

    boot_consecutive_accepts: int = 3
    reacquire_consecutive_accepts: int = 2
    reacquire_lost_frames: int = 5
    continue_position_gate_m: float = 0.25
    continue_rotation_gate_deg: float = 120.0


def _empty_reprojection_score(reason: str = "not_scored") -> Dict[str, Any]:
    return {
        "scored": False,
        "reason": reason,
        "coordinate_space": "raw_pixel",
        "score": 0.0,
        "mean_error_px": 0.0,
        "p95_error_px": 0.0,
        "max_error_px": 0.0,
        "matched_marker_views": 0,
        "expected_marker_views": 0,
        "missing_marker_views": 0,
        "duplicate_assignment_count": 0,
        "unexpected_blob_count": 0,
        "camera_count": 0,
        "match_gate_px": 0.0,
    }


@dataclass
class TrackingStats:
    """Rolling rigid-body tracking diagnostics for stabilization work."""

    valid_runs: List[int] = field(default_factory=list)
    lost_runs: List[int] = field(default_factory=list)
    current_valid_run: int = 0
    current_lost_run: int = 0
    total_valid_frames: int = 0
    total_invalid_frames: int = 0
    reacquire_count: int = 0
    short_valid_count: int = 0
    pose_jump_count: int = 0
    last_pose_jump_m: float = 0.0
    max_pose_jump_m: float = 0.0
    last_pose_flip_deg: float = 0.0
    max_pose_flip_deg: float = 0.0
    last_commit_position: Optional[np.ndarray] = None
    last_commit_quaternion: Optional[np.ndarray] = None
    invalid_reason: str = ""
    hypothesis_margin: float = 0.0

    def record(self, pose: RigidBodyPose, *, previous_valid: bool) -> None:
        """Record a valid/lost transition without changing tracking behavior."""
        if pose.valid:
            was_reacquire = not previous_valid and self.total_valid_frames > 0
            if previous_valid:
                self.current_valid_run += 1
            else:
                if self.current_lost_run > 0:
                    self.lost_runs.append(self.current_lost_run)
                self.current_valid_run = 1
                self.current_lost_run = 0
                if was_reacquire:
                    self.reacquire_count += 1
                    self._record_reacquire_jump(pose)
                    self._record_reacquire_flip(pose)

            self.total_valid_frames += 1
            self.last_commit_position = pose.position.copy()
            self.last_commit_quaternion = _normalize_quaternion(pose.quaternion)
            self.invalid_reason = ""
            return

        if previous_valid:
            if self.current_valid_run > 0:
                self.valid_runs.append(self.current_valid_run)
                if self.current_valid_run <= 5:
                    self.short_valid_count += 1
            self.current_valid_run = 0
            self.current_lost_run = 1
        else:
            self.current_lost_run += 1
        self.total_invalid_frames += 1
        self.invalid_reason = "no_valid_candidate"

    def snapshot(self) -> Dict[str, Any]:
        valid_runs = list(self.valid_runs)
        if self.current_valid_run > 0:
            valid_runs.append(self.current_valid_run)
        lost_runs = list(self.lost_runs)
        if self.current_lost_run > 0:
            lost_runs.append(self.current_lost_run)

        completed_valid_run_count = len(self.valid_runs)
        short_valid_count = self.short_valid_count

        return {
            "mean_valid_run_frames": float(np.mean(valid_runs)) if valid_runs else 0.0,
            "max_valid_run_frames": int(max(valid_runs)) if valid_runs else 0,
            "current_valid_run_frames": int(self.current_valid_run),
            "current_lost_run_frames": int(self.current_lost_run),
            "mean_lost_run_frames": float(np.mean(lost_runs)) if lost_runs else 0.0,
            "reacquire_count": int(self.reacquire_count),
            "short_valid_count": int(short_valid_count),
            "short_valid_ratio": (
                float(short_valid_count / completed_valid_run_count)
                if completed_valid_run_count
                else 0.0
            ),
            "pose_jump_count": int(self.pose_jump_count),
            "last_pose_jump_m": float(self.last_pose_jump_m),
            "max_pose_jump_m": float(self.max_pose_jump_m),
            "last_pose_flip_deg": float(self.last_pose_flip_deg),
            "max_pose_flip_deg": float(self.max_pose_flip_deg),
            "invalid_reason": self.invalid_reason,
            "hypothesis_margin": float(self.hypothesis_margin),
        }

    def _record_reacquire_jump(self, pose: RigidBodyPose) -> None:
        if self.last_commit_position is None:
            return
        jump = float(np.linalg.norm(pose.position - self.last_commit_position))
        self.last_pose_jump_m = jump
        self.max_pose_jump_m = max(self.max_pose_jump_m, jump)
        if jump > 0.10:
            self.pose_jump_count += 1

    def _record_reacquire_flip(self, pose: RigidBodyPose) -> None:
        if self.last_commit_quaternion is None:
            return
        angle_deg = _quaternion_angle_deg(self.last_commit_quaternion, pose.quaternion)
        self.last_pose_flip_deg = angle_deg
        self.max_pose_flip_deg = max(self.max_pose_flip_deg, angle_deg)


def _normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    quat = np.asarray(quaternion, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return quat / norm


def _quaternion_angle_deg(previous: np.ndarray, current: np.ndarray) -> float:
    q_prev = _normalize_quaternion(previous)
    q_curr = _normalize_quaternion(current)
    dot = abs(float(np.dot(q_prev, q_curr)))
    dot = min(1.0, max(-1.0, dot))
    return float(np.degrees(2.0 * np.arccos(dot)))


def _rotation_from_wxyz(quaternion: np.ndarray) -> np.ndarray:
    quat = _normalize_quaternion(quaternion)
    return Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()


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
        history_size: int = 30,
        mode_config: TrackModeConfig = TrackModeConfig(),
    ):
        """
        Initialize tracker.
        
        Args:
            pattern: MarkerPattern for this rigid body
            history_size: Number of poses to keep
        """
        self.pattern = pattern
        self.history_size = history_size
        self.mode_config = mode_config
        
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
        self.stats = TrackingStats()

        # Mode transition state. This is intentionally diagnostic-first in
        # Phase 3: pose acceptance remains owned by the estimator.
        self._mode = TrackMode.BOOT
        self._mode_entered_timestamp: int = 0
        self._mode_frame_count = 0
        self._mode_consecutive_accepts = 0
        self._mode_consecutive_rejects = 0
        self._mode_transition_count = 0
        self._last_mode_transition = "init:boot"
        self._last_position_innovation_m = 0.0
        self._max_position_innovation_m = 0.0
        self._last_rotation_innovation_deg = 0.0
        self._max_rotation_innovation_deg = 0.0
        self._last_mode_reason = "initializing"
        self._last_reprojection_score: Dict[str, Any] = _empty_reprojection_score()
    
    def update(self, pose: RigidBodyPose) -> None:
        """Update tracker with new pose estimate, including velocity estimation."""
        with self._lock:
            previous_valid = self._pose_history[-1].valid if self._pose_history else False
            self._update_mode_locked(pose)
            self._pose_history.append(pose)
            self.total_frames += 1
            self.stats.record(pose, previous_valid=previous_valid)

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

    def peek_prediction(self, timestamp_us: int) -> PredictedPose:
        """Predict pose at timestamp_us without mutating tracker state."""
        with self._lock:
            if self.track_count == 0 or self._last_valid_timestamp <= 0:
                quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                return PredictedPose(
                    timestamp=int(timestamp_us),
                    position=np.zeros(3, dtype=np.float64),
                    rotation=np.eye(3, dtype=np.float64),
                    quaternion=quat,
                    velocity=np.zeros(3, dtype=np.float64),
                    valid=False,
                    confidence=0.0,
                    position_sigma_m=1.0,
                    rotation_sigma_deg=180.0,
                    lost_frames=int(self.lost_frames),
                )

            dt_s = max(0.0, float(timestamp_us - self._last_valid_timestamp) / 1_000_000.0)
            velocity = self._velocity.copy()
            if dt_s > 0.5:
                # Long gaps make constant velocity less trustworthy; do not mutate
                # the stored velocity here because this is a pure peek.
                velocity = np.zeros(3, dtype=np.float64)
            position = self._position + velocity * min(dt_s, 0.5)
            quaternion = _normalize_quaternion(self._quaternion)
            confidence = self._rolling_confidence_locked()
            return PredictedPose(
                timestamp=int(timestamp_us),
                position=position.copy(),
                rotation=_rotation_from_wxyz(quaternion),
                quaternion=quaternion.copy(),
                velocity=velocity.copy(),
                dt_s=float(dt_s),
                valid=True,
                confidence=confidence,
                position_sigma_m=self._position_sigma_m(dt_s, confidence),
                rotation_sigma_deg=self._rotation_sigma_deg(dt_s, confidence),
                lost_frames=int(self.lost_frames),
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
        return self._mode in {TrackMode.BOOT, TrackMode.CONTINUE, TrackMode.REACQUIRE}
    
    @property
    def confidence(self) -> float:
        """Get tracking confidence (0-1)."""
        if self.total_frames == 0:
            return 0.0
        return self.track_count / self.total_frames

    @property
    def rolling_confidence(self) -> float:
        """Recent confidence for gating diagnostics without changing behavior."""
        with self._lock:
            return self._rolling_confidence_locked()

    def get_prediction_diagnostics(self, timestamp_us: Optional[int] = None) -> Dict[str, Any]:
        """Return side-effect-free prediction diagnostics for status/replay logs."""
        with self._lock:
            target_ts = int(timestamp_us if timestamp_us is not None else self._last_valid_timestamp)
        prediction = self.peek_prediction(target_ts)
        return prediction.to_dict()

    def record_reprojection_score(self, score: Dict[str, Any]) -> None:
        """Attach latest 2D reprojection diagnostics without affecting tracking state."""
        with self._lock:
            self._last_reprojection_score = dict(score or _empty_reprojection_score())

    def get_diagnostics(self) -> Dict[str, Any]:
        """Return diagnostics used by tracking logs and replay summaries."""
        latest = self._pose_history[-1] if self._pose_history else None
        latest_valid = bool(latest.valid) if latest is not None else False
        latest_rms = float(latest.rms_error) if latest is not None else 0.0
        latest_observed = int(latest.observed_markers) if latest is not None else 0
        prediction = self.get_prediction_diagnostics(
            latest.timestamp if latest is not None else self._last_valid_timestamp
        )
        return {
            "valid": latest_valid,
            "mode": self._mode.value,
            "mode_transition_count": int(self._mode_transition_count),
            "last_mode_transition": self._last_mode_transition,
            "mode_frame_count": int(self._mode_frame_count),
            "mode_consecutive_accepts": int(self._mode_consecutive_accepts),
            "mode_consecutive_rejects": int(self._mode_consecutive_rejects),
            "last_mode_reason": self._last_mode_reason,
            "confidence": float(self.confidence),
            "rolling_confidence": float(prediction["confidence"]),
            "prediction": prediction,
            "lost_frames": int(self.lost_frames),
            "track_count": int(self.track_count),
            "total_frames": int(self.total_frames),
            "last_position_innovation_m": float(self._last_position_innovation_m),
            "max_position_innovation_m": float(self._max_position_innovation_m),
            "last_rotation_innovation_deg": float(self._last_rotation_innovation_deg),
            "max_rotation_innovation_deg": float(self._max_rotation_innovation_deg),
            "reprojection_score": dict(self._last_reprojection_score),
            "rms_error_m": latest_rms,
            "observed_markers": latest_observed,
            "real_ray_count": latest_observed,
            "virtual_marker_count": 0,
            **self.stats.snapshot(),
        }

    def _rolling_confidence_locked(self) -> float:
        history = list(self._pose_history)
        if not history:
            return 0.0
        window = history[-min(len(history), self.history_size):]
        valid_window = [pose for pose in window if pose.valid]
        valid_ratio = len(valid_window) / len(window)
        lost_penalty = min(1.0, self.lost_frames / 10.0)

        if valid_window:
            rms_values = [max(0.0, float(pose.rms_error)) for pose in valid_window]
            median_rms = float(np.median(rms_values))
            rms_score = 1.0 / (1.0 + median_rms / 0.010)
        else:
            rms_score = 0.0

        pose_jump_penalty = min(0.35, self.stats.last_pose_jump_m / 0.30)
        flip_penalty = min(0.35, self.stats.last_pose_flip_deg / 360.0)
        confidence = (
            0.60 * valid_ratio
            + 0.25 * rms_score
            + 0.15 * (1.0 - lost_penalty)
            - pose_jump_penalty
            - flip_penalty
        )
        return float(np.clip(confidence, 0.0, 1.0))

    def _position_sigma_m(self, dt_s: float, confidence: float) -> float:
        speed = float(np.linalg.norm(self._velocity))
        return float(0.003 + 0.020 * (1.0 - confidence) + min(0.20, speed * dt_s * 0.25))

    @staticmethod
    def _rotation_sigma_deg(dt_s: float, confidence: float) -> float:
        return float(1.0 + 20.0 * (1.0 - confidence) + min(45.0, dt_s * 30.0))

    def _update_mode_locked(self, pose: RigidBodyPose) -> None:
        if pose.valid:
            self._mode_consecutive_accepts += 1
            self._mode_consecutive_rejects = 0
            self._record_innovation_locked(pose)
        else:
            self._mode_consecutive_rejects += 1
            self._mode_consecutive_accepts = 0
            self._last_mode_reason = "measurement_rejected"

        next_mode = self._mode
        reason = self._last_mode_reason

        if self._mode == TrackMode.BOOT:
            if pose.valid:
                reason = "boot_accepting"
                if self._mode_consecutive_accepts >= self.mode_config.boot_consecutive_accepts:
                    next_mode = TrackMode.CONTINUE
                    reason = "boot_confirmed"
            else:
                reason = "boot_waiting_for_shape"

        elif self._mode == TrackMode.CONTINUE:
            if pose.valid:
                reason = "continue_measurement_accepted"
            else:
                next_mode = TrackMode.REACQUIRE
                reason = "continue_measurement_rejected"

        elif self._mode == TrackMode.REACQUIRE:
            if pose.valid:
                if self._measurement_matches_prediction_locked(pose):
                    reason = "reacquire_candidate_consistent"
                    if (
                        self._mode_consecutive_accepts
                        >= self.mode_config.reacquire_consecutive_accepts
                    ):
                        next_mode = TrackMode.CONTINUE
                        reason = "reacquire_confirmed"
                else:
                    reason = "reacquire_candidate_large_innovation"
            elif self._mode_consecutive_rejects >= self.mode_config.reacquire_lost_frames:
                next_mode = TrackMode.LOST
                reason = "reacquire_timeout"
            else:
                reason = "reacquire_waiting"

        elif self._mode == TrackMode.LOST:
            if pose.valid:
                next_mode = TrackMode.BOOT
                reason = "lost_shape_found"
            else:
                reason = "lost_waiting_for_shape"

        self._last_mode_reason = reason
        self._mode_frame_count += 1
        if next_mode != self._mode:
            self._transition_mode_locked(next_mode, pose.timestamp, reason)

    def _transition_mode_locked(
        self,
        next_mode: TrackMode,
        timestamp_us: int,
        reason: str,
    ) -> None:
        previous = self._mode
        self._mode = next_mode
        self._mode_entered_timestamp = int(timestamp_us)
        self._mode_frame_count = 0
        self._mode_transition_count += 1
        self._last_mode_transition = f"{previous.value}->{next_mode.value}:{reason}"

    def _record_innovation_locked(self, pose: RigidBodyPose) -> None:
        if self.track_count == 0 or self._last_valid_timestamp <= 0:
            self._last_position_innovation_m = 0.0
            self._last_rotation_innovation_deg = 0.0
            self._last_mode_reason = "first_measurement"
            return

        dt_s = max(0.0, float(pose.timestamp - self._last_valid_timestamp) / 1_000_000.0)
        velocity = self._velocity if dt_s <= 0.5 else np.zeros(3, dtype=np.float64)
        predicted_position = self._position + velocity * min(dt_s, 0.5)
        self._last_position_innovation_m = float(np.linalg.norm(pose.position - predicted_position))
        self._max_position_innovation_m = max(
            self._max_position_innovation_m,
            self._last_position_innovation_m,
        )
        self._last_rotation_innovation_deg = _quaternion_angle_deg(
            self._quaternion,
            pose.quaternion,
        )
        self._max_rotation_innovation_deg = max(
            self._max_rotation_innovation_deg,
            self._last_rotation_innovation_deg,
        )
        self._last_mode_reason = "measurement_innovation_recorded"

    def _measurement_matches_prediction_locked(self, pose: RigidBodyPose) -> bool:
        if self.track_count == 0 or self._last_valid_timestamp <= 0:
            return True
        return (
            self._last_position_innovation_m <= self.mode_config.continue_position_gate_m
            and self._last_rotation_innovation_deg <= self.mode_config.continue_rotation_gate_deg
        )


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
        reprojection_match_gate_px: float = 12.0,
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
        if not np.isfinite(reprojection_match_gate_px) or float(reprojection_match_gate_px) <= 0.0:
            raise ValueError("reprojection_match_gate_px must be a finite positive value")
        self.patterns = patterns or [WAIST_PATTERN]
        self.marker_diameter = marker_diameter
        self.cluster_radius_m = float(cluster_radius_m)
        self.max_rms_error_m = float(max_rms_error_m)
        self.reprojection_match_gate_px = float(reprojection_match_gate_px)
        
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
        """Process 3D points using the legacy 3D-only path."""
        return self._process_points(
            points_3d,
            timestamp,
            camera_params=None,
            observations_by_camera=None,
            coordinate_space="raw_pixel",
        )

    def process_context(
        self,
        points_3d: np.ndarray,
        timestamp: int,
        *,
        camera_params: Optional[Dict[str, Any]] = None,
        observations_by_camera: Optional[Dict[str, List[Any]]] = None,
        coordinate_space: str = "raw_pixel",
    ) -> Dict[str, RigidBodyPose]:
        """Process 3D points with 2D observation context for diagnostics."""
        return self._process_points(
            points_3d,
            timestamp,
            camera_params=camera_params,
            observations_by_camera=observations_by_camera,
            coordinate_space=coordinate_space,
        )

    def _process_points(
        self,
        points_3d: np.ndarray,
        timestamp: int,
        *,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        coordinate_space: str,
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
            for tracker in self.trackers.values():
                tracker.record_reprojection_score(_empty_reprojection_score("no_3d_points"))
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
                score = self._score_pose_reprojection(
                    best_pose,
                    pattern,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                )
                self.trackers[pattern.name].record_reprojection_score(score)
                self.trackers[pattern.name].update(best_pose)
            else:
                # Use prediction
                predicted = self.trackers[pattern.name].predict()
                predicted.timestamp = timestamp
                poses[pattern.name] = predicted
                self.trackers[pattern.name].record_reprojection_score(
                    _empty_reprojection_score("no_valid_pose")
                )
                self.trackers[pattern.name].update(predicted)
        
        return poses

    def _score_pose_reprojection(
        self,
        pose: RigidBodyPose,
        pattern: MarkerPattern,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Dict[str, Any]:
        if not pose.valid:
            return _empty_reprojection_score("invalid_pose")
        if not camera_params or not observations_by_camera:
            return _empty_reprojection_score("no_2d_context")

        space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"
        markers_world = (pose.rotation @ pattern.marker_positions.T).T + pose.position.reshape(1, 3)
        residuals: List[float] = []
        matched = 0
        expected = 0
        duplicate_assignments = 0
        unexpected_blobs = 0
        scored_cameras = 0

        for camera_id, observations in observations_by_camera.items():
            camera = camera_params.get(camera_id)
            if camera is None:
                continue
            observed_uvs = self._extract_observation_uvs(observations, space)
            if not observed_uvs:
                continue

            projected_uvs = self._project_markers_to_camera(markers_world, camera, space)
            if not projected_uvs:
                continue

            scored_cameras += 1
            expected += len(projected_uvs)
            nearest_blob_indices: List[int] = []
            for uv in projected_uvs:
                distances = [
                    float(np.linalg.norm(np.asarray(uv, dtype=np.float64) - np.asarray(obs, dtype=np.float64)))
                    for obs in observed_uvs
                ]
                nearest_idx = int(np.argmin(distances))
                error = float(distances[nearest_idx])
                if error <= self.reprojection_match_gate_px:
                    matched += 1
                    residuals.append(error)
                    nearest_blob_indices.append(nearest_idx)

            unique_blob_indices = set(nearest_blob_indices)
            duplicate_assignments += max(0, len(nearest_blob_indices) - len(unique_blob_indices))
            unexpected_blobs += max(0, len(observed_uvs) - len(unique_blob_indices))

        if expected <= 0:
            return _empty_reprojection_score("no_projectable_markers")

        missing = max(0, expected - matched)
        mean_error = float(np.mean(residuals)) if residuals else 0.0
        p95_error = float(np.percentile(residuals, 95)) if residuals else 0.0
        max_error = float(np.max(residuals)) if residuals else 0.0
        coverage = float(matched / expected) if expected else 0.0
        residual_score = 1.0 / (1.0 + mean_error / 5.0) if residuals else 0.0
        duplicate_penalty = min(1.0, duplicate_assignments / max(1, matched))
        score = float(np.clip(0.70 * residual_score + 0.30 * coverage - 0.30 * duplicate_penalty, 0.0, 1.0))

        return {
            "scored": True,
            "reason": "ok",
            "coordinate_space": space,
            "score": score,
            "mean_error_px": mean_error,
            "p95_error_px": p95_error,
            "max_error_px": max_error,
            "matched_marker_views": int(matched),
            "expected_marker_views": int(expected),
            "missing_marker_views": int(missing),
            "duplicate_assignment_count": int(duplicate_assignments),
            "unexpected_blob_count": int(unexpected_blobs),
            "camera_count": int(scored_cameras),
            "match_gate_px": float(self.reprojection_match_gate_px),
        }

    @staticmethod
    def _extract_observation_uvs(observations: List[Any], coordinate_space: str) -> List[Tuple[float, float]]:
        key = "undistorted_uv" if coordinate_space == "undistorted_pixel" else "raw_uv"
        uvs: List[Tuple[float, float]] = []
        for observation in observations:
            value = None
            if isinstance(observation, dict):
                value = observation.get(key)
            else:
                value = getattr(observation, key, None)
            if value is None or len(value) < 2:
                continue
            uvs.append((float(value[0]), float(value[1])))
        return uvs

    @staticmethod
    def _project_markers_to_camera(
        markers_world: np.ndarray,
        camera: Any,
        coordinate_space: str,
    ) -> List[Tuple[float, float]]:
        points_world = np.asarray(markers_world, dtype=np.float64).reshape(-1, 3)
        rotation = np.asarray(camera.rotation, dtype=np.float64).reshape(3, 3)
        translation = np.asarray(camera.translation, dtype=np.float64).reshape(3)
        points_cam = (rotation @ points_world.T).T + translation.reshape(1, 3)
        in_front = points_cam[:, 2] > 1e-9
        if not np.any(in_front):
            return []

        points_world = points_world[in_front]
        if coordinate_space == "undistorted_pixel":
            points_cam = points_cam[in_front]
            fx = float(camera.intrinsic_matrix[0, 0])
            fy = float(camera.intrinsic_matrix[1, 1])
            cx = float(camera.intrinsic_matrix[0, 2])
            cy = float(camera.intrinsic_matrix[1, 2])
            return [
                (float(fx * point[0] / point[2] + cx), float(fy * point[1] / point[2] + cy))
                for point in points_cam
            ]

        rvec, _ = cv.Rodrigues(rotation)
        projected, _ = cv.projectPoints(
            points_world.astype(np.float32),
            rvec,
            translation,
            camera.intrinsic_matrix,
            camera.distortion_coeffs,
        )
        return [(float(point[0]), float(point[1])) for point in projected.reshape(-1, 2)]
    
    def get_tracking_status(self) -> Dict[str, Dict[str, Any]]:
        """Get tracking status for all bodies."""
        return {
            name: {
                "is_tracking": tracker.is_tracking,
                **tracker.get_diagnostics(),
            }
            for name, tracker in self.trackers.items()
        }
