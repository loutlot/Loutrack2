"""
Rigid body tracking module for pose estimation from 3D point clouds.

Provides functionality to:
- Cluster 3D points using DBSCAN
- Estimate rigid body pose using Kabsch algorithm
- Fallback to PnP for limited observations
- Track rigid bodies over time with Kalman filtering
"""

import numpy as np
from typing import Optional, Dict, Any, List, Tuple, Callable
from dataclasses import dataclass, field
from collections import Counter, deque
from enum import Enum
from itertools import combinations, permutations
import time
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


@dataclass(frozen=True)
class ReacquireGuardConfig:
    """2D score guard used to evaluate reacquire candidates."""

    shadow_enabled: bool = True
    enforced: bool = False
    post_reacquire_continue_frames: int = 0
    min_matched_marker_views: int = 6
    max_missing_marker_views: int = 2
    max_mean_reprojection_error_px: float = 4.0
    max_p95_reprojection_error_px: float = 8.0
    allow_duplicate_assignment: bool = False
    max_position_innovation_m: float = 0.25
    max_rotation_innovation_deg: float = 136.0

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "min_matched_marker_views": int(self.min_matched_marker_views),
            "max_missing_marker_views": int(self.max_missing_marker_views),
            "max_mean_reprojection_error_px": float(self.max_mean_reprojection_error_px),
            "max_p95_reprojection_error_px": float(self.max_p95_reprojection_error_px),
            "allow_duplicate_assignment": bool(self.allow_duplicate_assignment),
            "max_position_innovation_m": float(self.max_position_innovation_m),
            "max_rotation_innovation_deg": float(self.max_rotation_innovation_deg),
            "post_reacquire_continue_frames": int(self.post_reacquire_continue_frames),
        }


@dataclass(frozen=True)
class ObjectGatingConfig:
    """Predicted-pose 2D marker gates before generic triangulation."""

    enabled: bool = True
    enforce: bool = False
    activation_mode: str = "always"
    min_enforced_markers: int = 3
    pixel_min: float = 4.0
    pixel_max: float = 16.0
    single_ray_confidence_min: float = 0.75

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "pixel_min": float(self.pixel_min),
            "pixel_max": float(self.pixel_max),
            "single_ray_confidence_min": float(self.single_ray_confidence_min),
            "enforce": bool(self.enforce),
            "activation_mode": str(self.activation_mode),
            "min_enforced_markers": int(self.min_enforced_markers),
        }


@dataclass(frozen=True)
class SubsetSolveConfig:
    """Phase 6 subset hypothesis diagnostics and weighted rigid solve settings."""

    enabled: bool = True
    diagnostics_only: bool = True
    subset_sizes: Tuple[int, ...] = (3, 4)
    max_observed_points: int = 10
    max_hypotheses: int = 4096
    max_generic_observed_subsets: int = 24
    min_score: float = 0.65
    max_p95_error_px: float = 8.0
    min_margin: float = 0.02
    ambiguous_subset_delta_m: float = 0.005
    prediction_gate_m: float = 0.08
    max_rotation_delta_deg: float = 90.0
    source_priority_bonus: float = 0.04
    coverage_weight: float = 0.08
    temporal_penalty_weight: float = 0.18
    flip_penalty: float = 0.25
    adoption_min_valid_ratio: float = 0.98

    def __post_init__(self) -> None:
        if not self.diagnostics_only:
            raise ValueError("SubsetSolveConfig adoption is not implemented; diagnostics_only must remain True")

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "diagnostics_only": bool(self.diagnostics_only),
            "subset_sizes": [int(value) for value in self.subset_sizes],
            "max_observed_points": int(self.max_observed_points),
            "max_hypotheses": int(self.max_hypotheses),
            "max_generic_observed_subsets": int(self.max_generic_observed_subsets),
            "min_score": float(self.min_score),
            "max_p95_error_px": float(self.max_p95_error_px),
            "min_margin": float(self.min_margin),
            "ambiguous_subset_delta_m": float(self.ambiguous_subset_delta_m),
            "prediction_gate_m": float(self.prediction_gate_m),
            "max_rotation_delta_deg": float(self.max_rotation_delta_deg),
            "source_priority_bonus": float(self.source_priority_bonus),
            "coverage_weight": float(self.coverage_weight),
            "temporal_penalty_weight": float(self.temporal_penalty_weight),
            "flip_penalty": float(self.flip_penalty),
            "adoption_min_valid_ratio": float(self.adoption_min_valid_ratio),
        }


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


def _empty_object_gating(
    *,
    enabled: bool = True,
    enforced: bool = False,
    reason: str = "not_evaluated",
    thresholds: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "enabled": bool(enabled),
        "enforced": bool(enforced),
        "evaluated": False,
        "reason": reason,
        "mode": "",
        "prediction_valid": False,
        "confidence": 0.0,
        "pixel_gate_px": 0.0,
        "camera_count": 0,
        "marker_count": 0,
        "candidate_window_count": 0,
        "assigned_marker_views": 0,
        "unmatched_marker_views": 0,
        "duplicate_assignment_count": 0,
        "markers_with_two_or_more_rays": 0,
        "markers_with_one_ray": 0,
        "single_ray_candidates": 0,
        "generic_fallback_blob_count": 0,
        "allow_single_ray": False,
        "thresholds": dict(thresholds or {}),
        "per_marker_ray_count": [],
        "per_camera": {},
    }


def _empty_rigid_hint_pose(reason: str = "not_evaluated") -> Dict[str, Any]:
    return {
        "evaluated": False,
        "reason": str(reason),
        "diagnostics_only": True,
        "enforced": False,
        "selected_for_pose": False,
        "selection_reason": "",
        "valid": False,
        "generic_valid": False,
        "would_improve_score": False,
        "candidate_points": 0,
        "observed_markers": 0,
        "real_ray_count": 0,
        "virtual_marker_count": 0,
        "rms_error_m": 0.0,
        "generic_rms_error_m": 0.0,
        "score": _empty_reprojection_score("not_scored"),
        "generic_score": _empty_reprojection_score("not_scored"),
        "score_delta": 0.0,
        "position_delta_m": 0.0,
        "rotation_delta_deg": 0.0,
        "marker_indices": [],
        "invalid_points": 0,
        "pose": {},
    }


def _empty_subset_hypothesis(
    reason: str = "not_evaluated",
    thresholds: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "evaluated": False,
        "reason": str(reason),
        "diagnostics_only": True,
        "enabled": True,
        "thresholds": dict(thresholds or {}),
        "candidate_count": 0,
        "pruned_candidate_count": 0,
        "valid_candidate_count": 0,
        "rejected_by_ambiguity": 0,
        "rejected_by_2d_score": 0,
        "rejected_by_rms": 0,
        "flip_risk_count": 0,
        "truncated": False,
        "best": {},
        "second": {},
        "best_score": 0.0,
        "second_score": 0.0,
        "best_combined_score": 0.0,
        "second_combined_score": 0.0,
        "margin": 0.0,
        "combined_margin": 0.0,
        "generic_score": 0.0,
        "score_delta": 0.0,
        "subset_adoption_ready": False,
        "weighted_solve": {},
    }


def _empty_reacquire_guard(
    *,
    enabled: bool = True,
    enforced: bool = False,
    rejected_count: int = 0,
    evaluated_count: int = 0,
    would_reject_count: int = 0,
    reason: str = "not_evaluated",
    thresholds: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "enabled": bool(enabled),
        "enforced": bool(enforced),
        "evaluated": False,
        "passed": True,
        "would_reject": False,
        "reason": reason,
        "thresholds": dict(thresholds or {}),
        "score": {},
        "position_innovation_m": 0.0,
        "rotation_innovation_deg": 0.0,
        "evaluated_count": int(evaluated_count),
        "would_reject_count": int(would_reject_count),
        "rejected_count": int(rejected_count),
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

    def record(
        self,
        pose: RigidBodyPose,
        *,
        previous_valid: bool,
        invalid_reason: Optional[str] = None,
    ) -> None:
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
        self.invalid_reason = invalid_reason or "no_valid_candidate"

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
    def estimate_weighted(
        observed_points: np.ndarray,
        reference_points: np.ndarray,
        weights: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Weighted Kabsch alignment with the same observed->reference convention.
        """
        if len(observed_points) != len(reference_points):
            raise ValueError("Point counts must match")
        if len(observed_points) != len(weights):
            raise ValueError("Weight count must match point count")
        if len(observed_points) < 3:
            raise ValueError("Need at least 3 points")

        observed = np.asarray(observed_points, dtype=np.float64)
        reference = np.asarray(reference_points, dtype=np.float64)
        weight_arr = np.asarray(weights, dtype=np.float64).reshape(-1)
        weight_arr = np.maximum(weight_arr, 1e-9)
        weight_sum = float(np.sum(weight_arr))
        norm_weights = weight_arr / weight_sum

        obs_centroid = np.sum(observed * norm_weights[:, None], axis=0)
        ref_centroid = np.sum(reference * norm_weights[:, None], axis=0)
        obs_centered = observed - obs_centroid
        ref_centered = reference - ref_centroid
        H = obs_centered.T @ (ref_centered * norm_weights[:, None])

        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = ref_centroid - R @ obs_centroid

        transformed = (R @ observed.T).T + t
        errors = np.linalg.norm(transformed - reference, axis=1)
        rms_error = float(np.sqrt(np.sum(norm_weights * (errors ** 2))))
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
            best_error = float('inf')
            best_ref = None
            best_obs = None
            for subset in combinations(range(n_ref), n_obs):
                ref_subset = reference_points[list(subset)]
                for perm in permutations(range(n_obs)):
                    ref_perm = ref_subset[list(perm)]
                    try:
                        _, _, error = KabschEstimator.estimate(ref_perm, observed_points)
                    except Exception:
                        continue
                    if error < best_error:
                        best_error = float(error)
                        best_ref = ref_perm
                        best_obs = observed_points

            if best_ref is None or best_obs is None:
                return np.array([]), np.array([]), float('inf')
            return best_obs, best_ref, best_error
        
        else:
            # More observed than reference - find best subset
            # Use distance-based matching
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
        self._last_object_gating: Dict[str, Any] = _empty_object_gating()
        self._last_rigid_hint_pose: Dict[str, Any] = _empty_rigid_hint_pose()
        self._last_subset_hypothesis: Dict[str, Any] = _empty_subset_hypothesis()
        self._reacquire_guard_evaluated_count = 0
        self._reacquire_guard_would_reject_count = 0
        self._reacquire_guard_rejected_count = 0
        self._last_reacquire_guard: Dict[str, Any] = _empty_reacquire_guard()
    
    def update(self, pose: RigidBodyPose, *, invalid_reason: Optional[str] = None) -> None:
        """Update tracker with new pose estimate, including velocity estimation."""
        with self._lock:
            previous_valid = self._pose_history[-1].valid if self._pose_history else False
            self._update_mode_locked(pose)
            self._pose_history.append(pose)
            self.total_frames += 1
            self.stats.record(
                pose,
                previous_valid=previous_valid,
                invalid_reason=invalid_reason,
            )

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

    def record_object_gating(self, gating: Dict[str, Any]) -> None:
        """Attach latest object-conditioned gating diagnostics."""
        with self._lock:
            self._last_object_gating = dict(gating or _empty_object_gating())

    def record_rigid_hint_pose(self, diagnostic: Dict[str, Any]) -> None:
        """Attach latest rigid-hint pose comparison diagnostics."""
        with self._lock:
            self._last_rigid_hint_pose = dict(diagnostic or _empty_rigid_hint_pose())

    def record_subset_hypothesis(self, diagnostic: Dict[str, Any]) -> None:
        """Attach latest Phase 6 subset hypothesis diagnostics."""
        with self._lock:
            self._last_subset_hypothesis = dict(diagnostic or _empty_subset_hypothesis())

    @property
    def mode(self) -> TrackMode:
        """Return the current lifecycle mode."""
        with self._lock:
            return self._mode

    def should_guard_post_reacquire_continue(self, frame_count: int) -> bool:
        """Return true for the first continue frames after reacquire confirmation."""
        if frame_count <= 0:
            return False
        with self._lock:
            return (
                self._mode == TrackMode.CONTINUE
                and self._mode_frame_count < int(frame_count)
                and self._last_mode_transition.startswith("reacquire->continue:")
            )

    def record_reacquire_guard(self, guard: Dict[str, Any]) -> None:
        """Attach latest reacquire guard diagnostics without affecting tracking state."""
        with self._lock:
            payload = dict(guard or _empty_reacquire_guard())
            if payload.get("evaluated"):
                self._reacquire_guard_evaluated_count += 1
            if payload.get("would_reject"):
                self._reacquire_guard_would_reject_count += 1
            if payload.get("enforced") and payload.get("would_reject"):
                self._reacquire_guard_rejected_count += 1
            payload["evaluated_count"] = int(self._reacquire_guard_evaluated_count)
            payload["would_reject_count"] = int(self._reacquire_guard_would_reject_count)
            payload["rejected_count"] = int(self._reacquire_guard_rejected_count)
            self._last_reacquire_guard = payload

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
            "object_gating": dict(self._last_object_gating),
            "rigid_hint_pose": dict(self._last_rigid_hint_pose),
            "subset_hypothesis": dict(self._last_subset_hypothesis),
            "reacquire_guard": dict(self._last_reacquire_guard),
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
            if self._mode != TrackMode.REACQUIRE:
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
                    self._mode_consecutive_accepts += 1
                    reason = "reacquire_candidate_consistent"
                    if (
                        self._mode_consecutive_accepts
                        >= self.mode_config.reacquire_consecutive_accepts
                    ):
                        next_mode = TrackMode.CONTINUE
                        reason = "reacquire_confirmed"
                else:
                    self._mode_consecutive_accepts = 0
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
        reacquire_guard_config: ReacquireGuardConfig = ReacquireGuardConfig(),
        object_gating_config: ObjectGatingConfig = ObjectGatingConfig(),
        subset_solve_config: SubsetSolveConfig = SubsetSolveConfig(),
        subset_diagnostics_mode: str = "full",
        subset_time_budget_ms: Optional[float] = None,
        subset_max_hypotheses: Optional[int] = None,
        rigid_candidate_separation_enabled: bool = False,
        stage_callback: Optional[Callable[[str, float], None]] = None,
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
        self.reacquire_guard_config = reacquire_guard_config
        self.object_gating_config = object_gating_config
        self.subset_solve_config = subset_solve_config
        self.subset_diagnostics_mode = str(subset_diagnostics_mode or "full")
        if self.subset_diagnostics_mode not in {"full", "sampled", "off"}:
            raise ValueError("subset_diagnostics_mode must be full, sampled, or off")
        self.subset_sample_interval = 30
        self.subset_time_budget_ms = (
            float(subset_time_budget_ms)
            if subset_time_budget_ms is not None and float(subset_time_budget_ms) > 0.0
            else None
        )
        self.subset_max_hypotheses = (
            int(subset_max_hypotheses)
            if subset_max_hypotheses is not None and int(subset_max_hypotheses) > 0
            else None
        )
        self.rigid_candidate_separation_enabled = bool(rigid_candidate_separation_enabled)
        self._stage_callback = stage_callback
        self._variant_metric_lock = threading.Lock()
        self._subset_frame_index = 0
        self._subset_sampled_count = 0
        self._subset_skipped_count = 0
        self._subset_budget_exceeded_count = 0
        self._rigid_candidate_separated_count = 0
        self._rigid_candidate_fallback_count = 0
        self._rigid_candidate_fallback_reason_counts: Counter[str] = Counter()
        
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

    def get_variant_metrics(self) -> Dict[str, Any]:
        with self._variant_metric_lock:
            return {
                "subset_sampled_count": int(self._subset_sampled_count),
                "subset_skipped_count": int(self._subset_skipped_count),
                "subset_budget_exceeded_count": int(self._subset_budget_exceeded_count),
                "subset_diagnostics_mode": self.subset_diagnostics_mode,
                "subset_time_budget_ms": self.subset_time_budget_ms,
                "subset_max_hypotheses": self.subset_max_hypotheses,
                "rigid_candidate_separation_enabled": self.rigid_candidate_separation_enabled,
                "rigid_candidate_separated_count": int(self._rigid_candidate_separated_count),
                "rigid_candidate_fallback_count": int(self._rigid_candidate_fallback_count),
                "rigid_candidate_fallback_reason_counts": dict(
                    self._rigid_candidate_fallback_reason_counts
                ),
            }

    def _record_subset_stage(self, started_ns: int) -> None:
        if self._stage_callback is None:
            return
        self._stage_callback("subset_ms", float(time.perf_counter_ns() - started_ns) / 1_000_000.0)

    def _record_rigid_candidate_separated(self) -> None:
        with self._variant_metric_lock:
            self._rigid_candidate_separated_count += 1

    def _record_rigid_candidate_fallback(self, reason: str) -> None:
        with self._variant_metric_lock:
            self._rigid_candidate_fallback_count += 1
            self._rigid_candidate_fallback_reason_counts[str(reason or "unknown")] += 1

    def _rigid_hint_candidate_cluster(
        self,
        pattern: MarkerPattern,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
    ) -> Optional[np.ndarray]:
        marker_points = self._rigid_hint_candidate_marker_points(
            pattern,
            rigid_hint_triangulated_points,
        )
        if marker_points is None:
            return None
        _marker_indices, points = marker_points
        return points

    def _rigid_hint_candidate_marker_points(
        self,
        pattern: MarkerPattern,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
    ) -> Optional[Tuple[List[int], np.ndarray]]:
        hint_markers = self._rigid_hint_markers_by_index(pattern, rigid_hint_triangulated_points)
        if len(hint_markers) < max(3, pattern.num_markers - 1):
            return None
        marker_indices = [int(index) for index in sorted(hint_markers)]
        ordered = [hint_markers[index]["point"] for index in marker_indices]
        points = np.asarray(ordered, dtype=np.float64).reshape(-1, 3)
        if len(points) < 3 or not np.isfinite(points).all():
            return None
        return marker_indices, points

    def _try_rigid_separated_candidate(
        self,
        pattern: MarkerPattern,
        timestamp: int,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Optional[Tuple[RigidBodyPose, Dict[str, Any]]]:
        marker_points = self._rigid_hint_candidate_marker_points(
            pattern,
            rigid_hint_triangulated_points,
        )
        if marker_points is None:
            self._record_rigid_candidate_fallback("insufficient_rigid_hint_points")
            return None
        marker_indices, observed = marker_points
        reference = np.asarray(
            [pattern.marker_positions[index] for index in marker_indices],
            dtype=np.float64,
        )
        try:
            rotation, position, rms_error = KabschEstimator.estimate(reference, observed)
            quat_xyzw = Rotation.from_matrix(rotation).as_quat()
            quaternion = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=np.float64,
            )
            pose = RigidBodyPose(
                timestamp=timestamp,
                position=position,
                rotation=rotation,
                quaternion=quaternion,
                rms_error=float(rms_error),
                observed_markers=int(len(marker_indices)),
                valid=bool(float(rms_error) <= self.max_rms_error_m),
            )
        except Exception:
            self._record_rigid_candidate_fallback("rigid_candidate_solve_failed")
            return None
        if not pose.valid or pose.rms_error > self.max_rms_error_m:
            self._record_rigid_candidate_fallback("invalid_rigid_candidate_pose")
            return None
        score = self._score_pose_reprojection(
            pose,
            pattern,
            camera_params,
            observations_by_camera,
            coordinate_space=coordinate_space,
        )
        if score.get("scored", False):
            p95_error = float(score.get("p95_error_px", 0.0))
            min_views = max(3, int(self.object_gating_config.min_enforced_markers)) * 2
            matched_views = int(score.get("matched_marker_views", 0))
            if p95_error > self.reprojection_match_gate_px or matched_views < min_views:
                self._record_rigid_candidate_fallback("rigid_candidate_2d_score_gate")
                return None
        self._record_rigid_candidate_separated()
        return pose, score

    def _subset_diagnostics_for_frame(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        timestamp: int,
        points_3d: np.ndarray,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
        generic_pose: Optional[RigidBodyPose],
        generic_score: Dict[str, Any],
    ) -> Dict[str, Any]:
        mode = self.subset_diagnostics_mode
        thresholds = self.subset_solve_config.thresholds_dict()
        if mode == "off":
            with self._variant_metric_lock:
                self._subset_skipped_count += 1
            diagnostic = _empty_subset_hypothesis("disabled_by_mode", thresholds)
            diagnostic["enabled"] = False
            diagnostic["sampled"] = False
            return diagnostic

        evaluate = True
        sample_reason = "full"
        if mode == "sampled":
            with self._variant_metric_lock:
                self._subset_frame_index += 1
                frame_index = self._subset_frame_index
            risky = (
                tracker.mode != TrackMode.CONTINUE
                or generic_pose is None
                or not generic_pose.valid
                or bool((generic_score or {}).get("flip_risk", False))
            )
            evaluate = risky or frame_index % self.subset_sample_interval == 0
            sample_reason = "risk" if risky else "interval" if evaluate else "sampled_skip"
            if not evaluate:
                with self._variant_metric_lock:
                    self._subset_skipped_count += 1
                diagnostic = _empty_subset_hypothesis("sampled_skip", thresholds)
                diagnostic["sampled"] = True
                diagnostic["sample_reason"] = sample_reason
                return diagnostic

        started_ns = time.perf_counter_ns()
        diagnostic = self._evaluate_subset_hypotheses(
            pattern,
            timestamp,
            points_3d,
            rigid_hint_triangulated_points,
            camera_params,
            observations_by_camera,
            coordinate_space=coordinate_space,
            generic_pose=generic_pose,
            generic_score=generic_score,
        )
        self._record_subset_stage(started_ns)
        if diagnostic.get("time_budget_exceeded"):
            with self._variant_metric_lock:
                self._subset_budget_exceeded_count += 1
        if mode == "sampled":
            with self._variant_metric_lock:
                self._subset_sampled_count += 1
            diagnostic["sampled"] = True
            diagnostic["sample_reason"] = sample_reason
        else:
            diagnostic["sampled"] = False
            diagnostic["sample_reason"] = "full"
        return diagnostic

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

    def evaluate_object_conditioned_gating(
        self,
        *,
        timestamp: int,
        camera_params: Optional[Dict[str, Any]],
        frames_by_camera: Optional[Dict[str, Any]],
        coordinate_space: str = "raw_pixel",
    ) -> Dict[str, Dict[str, Any]]:
        """Evaluate predicted marker-to-blob 2D gates without changing triangulation."""
        results: Dict[str, Dict[str, Any]] = {}
        for pattern in self.patterns:
            tracker = self.trackers.get(pattern.name)
            if tracker is None:
                continue
            result = self._evaluate_object_gating_for_pattern(
                pattern,
                tracker,
                timestamp=timestamp,
                camera_params=camera_params,
                frames_by_camera=frames_by_camera,
                coordinate_space=coordinate_space,
            )
            tracker.record_object_gating(result)
            results[pattern.name] = result
        return results

    def _evaluate_object_gating_for_pattern(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        *,
        timestamp: int,
        camera_params: Optional[Dict[str, Any]],
        frames_by_camera: Optional[Dict[str, Any]],
        coordinate_space: str,
    ) -> Dict[str, Any]:
        config = self.object_gating_config
        thresholds = config.thresholds_dict()
        if not config.enabled:
            return _empty_object_gating(
                enabled=False,
                enforced=config.enforce,
                reason="disabled",
                thresholds=thresholds,
            )
        active, inactive_reason = self._object_gating_active_for_tracker(tracker)
        if not active:
            result = _empty_object_gating(
                enabled=True,
                enforced=config.enforce,
                reason=inactive_reason,
                thresholds=thresholds,
            )
            result["mode"] = tracker.mode.value
            return result
        if not camera_params or not frames_by_camera:
            return _empty_object_gating(
                enabled=True,
                enforced=config.enforce,
                reason="no_camera_context",
                thresholds=thresholds,
            )

        prediction = tracker.peek_prediction(timestamp)
        mode = tracker.mode.value
        if not prediction.valid:
            result = _empty_object_gating(
                enabled=True,
                enforced=config.enforce,
                reason="no_prediction",
                thresholds=thresholds,
            )
            result["mode"] = mode
            return result

        pixel_gate_px = self._object_gate_px(prediction, config)
        markers_world = (
            prediction.rotation @ pattern.marker_positions.T
        ).T + prediction.position.reshape(1, 3)
        marker_count = int(pattern.num_markers)
        marker_ray_counts = [0 for _ in range(marker_count)]
        assigned_marker_views = 0
        unmatched_marker_views = 0
        duplicate_assignment_count = 0
        candidate_window_count = 0
        per_camera: Dict[str, Any] = {}
        total_blobs = 0
        total_assigned_blobs = 0
        space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"

        for camera_id, frame in frames_by_camera.items():
            camera = camera_params.get(camera_id)
            blobs = list(getattr(frame, "blobs", []) or [])
            if camera is None or not blobs:
                continue
            blob_uvs = [
                self._frame_blob_uv(camera, blob, space)
                for blob in blobs
            ]
            projected = self._project_markers_to_camera(markers_world, camera, space)
            if not projected:
                continue

            total_blobs += len(blob_uvs)
            candidate_window_count += len(projected)
            cost = np.full((len(projected), len(blob_uvs)), 1e9, dtype=np.float64)
            for marker_idx, uv in enumerate(projected):
                for blob_idx, blob_uv in enumerate(blob_uvs):
                    distance = float(
                        np.linalg.norm(
                            np.asarray(uv, dtype=np.float64)
                            - np.asarray(blob_uv, dtype=np.float64)
                        )
                    )
                    if distance <= pixel_gate_px:
                        cost[marker_idx, blob_idx] = distance

            assignments: List[Dict[str, Any]] = []
            used_blobs: set[int] = set()
            if cost.size:
                row_ind, col_ind = linear_sum_assignment(cost)
                for row, col in zip(row_ind, col_ind):
                    if cost[row, col] >= 1e9:
                        continue
                    marker_idx = int(row)
                    blob_idx = int(col)
                    if blob_idx in used_blobs:
                        duplicate_assignment_count += 1
                        continue
                    used_blobs.add(blob_idx)
                    marker_ray_counts[marker_idx] += 1
                    assigned_marker_views += 1
                    assignments.append(
                        {
                            "marker_idx": marker_idx,
                            "blob_index": blob_idx,
                            "distance_px": float(cost[row, col]),
                        }
                    )
            unmatched_marker_views += max(0, len(projected) - len(assignments))
            total_assigned_blobs += len(used_blobs)
            per_camera[str(camera_id)] = {
                "blob_count": int(len(blob_uvs)),
                "projected_marker_count": int(len(projected)),
                "assigned_marker_views": int(len(assignments)),
                "unmatched_marker_views": int(max(0, len(projected) - len(assignments))),
                "assignments": assignments,
            }

        markers_with_two_or_more = sum(1 for count in marker_ray_counts if count >= 2)
        markers_with_one = sum(1 for count in marker_ray_counts if count == 1)
        allow_single_ray = prediction.confidence >= config.single_ray_confidence_min
        return {
            "enabled": True,
            "enforced": bool(config.enforce),
            "diagnostics_only": not bool(config.enforce),
            "evaluated": True,
            "reason": "ok",
            "mode": mode,
            "prediction_valid": True,
            "confidence": float(prediction.confidence),
            "pixel_gate_px": float(pixel_gate_px),
            "camera_count": int(len(per_camera)),
            "marker_count": marker_count,
            "candidate_window_count": int(candidate_window_count),
            "assigned_marker_views": int(assigned_marker_views),
            "unmatched_marker_views": int(unmatched_marker_views),
            "duplicate_assignment_count": int(duplicate_assignment_count),
            "markers_with_two_or_more_rays": int(markers_with_two_or_more),
            "markers_with_one_ray": int(markers_with_one),
            "single_ray_candidates": int(markers_with_one if allow_single_ray else 0),
            "generic_fallback_blob_count": int(max(0, total_blobs - total_assigned_blobs)),
            "allow_single_ray": bool(allow_single_ray),
            "thresholds": thresholds,
            "per_marker_ray_count": [int(value) for value in marker_ray_counts],
            "per_camera": per_camera,
        }

    def _object_gating_active_for_tracker(
        self,
        tracker: RigidBodyTracker,
    ) -> Tuple[bool, str]:
        mode = str(self.object_gating_config.activation_mode or "always")
        tracker_mode = tracker.mode
        if mode == "always":
            return True, "ok"
        if mode == "reacquire_only":
            return tracker_mode == TrackMode.REACQUIRE, "inactive_mode"
        if mode == "boot_or_reacquire":
            return tracker_mode in {TrackMode.BOOT, TrackMode.REACQUIRE}, "inactive_mode"
        return True, "unknown_activation_mode"

    @staticmethod
    def _object_gate_px(prediction: PredictedPose, config: ObjectGatingConfig) -> float:
        uncertainty_px = (
            4.0
            + 80.0 * float(prediction.position_sigma_m)
            + 0.08 * float(prediction.rotation_sigma_deg)
        )
        return float(np.clip(uncertainty_px, config.pixel_min, config.pixel_max))

    @staticmethod
    def _frame_blob_uv(camera: Any, blob: Dict[str, Any], coordinate_space: str) -> Tuple[float, float]:
        raw = (float(blob.get("x", 0.0)), float(blob.get("y", 0.0)))
        if coordinate_space != "undistorted_pixel":
            return raw
        pts_arr = np.array([[[raw[0], raw[1]]]], dtype=np.float64)
        undistorted = cv.undistortPoints(
            pts_arr,
            camera.intrinsic_matrix,
            camera.distortion_coeffs,
            P=camera.intrinsic_matrix,
        )
        return float(undistorted[0, 0, 0]), float(undistorted[0, 0, 1])
    
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
            rigid_hint_triangulated_points=None,
            coordinate_space="raw_pixel",
        )

    def process_context(
        self,
        points_3d: np.ndarray,
        timestamp: int,
        *,
        camera_params: Optional[Dict[str, Any]] = None,
        observations_by_camera: Optional[Dict[str, List[Any]]] = None,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]] = None,
        coordinate_space: str = "raw_pixel",
    ) -> Dict[str, RigidBodyPose]:
        """Process 3D points with 2D observation context."""
        return self._process_points(
            points_3d,
            timestamp,
            camera_params=camera_params,
            observations_by_camera=observations_by_camera,
            rigid_hint_triangulated_points=rigid_hint_triangulated_points,
            coordinate_space=coordinate_space,
        )

    def _process_points(
        self,
        points_3d: np.ndarray,
        timestamp: int,
        *,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
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
        has_rigid_hints = bool(rigid_hint_triangulated_points)
        object_gating_enforced = bool(
            self.object_gating_config.enabled and self.object_gating_config.enforce
        )
        if len(points_3d) == 0 and not has_rigid_hints:
            # Return predictions for all bodies
            for tracker in self.trackers.values():
                tracker.record_reprojection_score(_empty_reprojection_score("no_3d_points"))
                tracker.record_rigid_hint_pose(_empty_rigid_hint_pose("no_rigid_hint_points"))
                tracker.record_subset_hypothesis(
                    _empty_subset_hypothesis(
                        "no_3d_points",
                        self.subset_solve_config.thresholds_dict(),
                    )
                )
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
            best_score_from_separated: Optional[Dict[str, Any]] = None

            if self.rigid_candidate_separation_enabled and has_rigid_hints:
                separated = self._try_rigid_separated_candidate(
                    pattern,
                    timestamp,
                    rigid_hint_triangulated_points,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                )
                if separated is not None:
                    best_pose, best_score_from_separated = separated
                    best_error = float(best_pose.rms_error)
                    best_cluster_idx = -1
            
            if best_pose is None:
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
            
            tracker = self.trackers[pattern.name]
            if best_pose is not None:
                score = (
                    dict(best_score_from_separated)
                    if best_score_from_separated is not None
                    else self._score_pose_reprojection(
                        best_pose,
                        pattern,
                        camera_params,
                        observations_by_camera,
                        coordinate_space=coordinate_space,
                    )
                )
                hint_diagnostic = self._evaluate_rigid_hint_pose(
                    pattern,
                    timestamp,
                    rigid_hint_triangulated_points,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                    generic_pose=best_pose,
                    generic_score=score,
                )
                selected_pose = best_pose
                selected_score = score
                selected_source = "generic"
                enforce_hint = self._should_select_rigid_hint_pose(hint_diagnostic)
                if enforce_hint:
                    hint_pose = self._pose_from_payload(
                        hint_diagnostic.get("pose"),
                        fallback_timestamp=timestamp,
                    )
                    if hint_pose is not None and hint_pose.valid:
                        selected_pose = hint_pose
                        selected_score = dict(
                            hint_diagnostic.get("score") or _empty_reprojection_score()
                        )
                        selected_source = "rigid_hint"
                        hint_diagnostic["selected_for_pose"] = True
                        hint_diagnostic["selection_reason"] = "object_gating_enforced"
                    else:
                        hint_diagnostic["selected_for_pose"] = False
                        hint_diagnostic["selection_reason"] = "invalid_pose_payload"
                else:
                    hint_diagnostic["selected_for_pose"] = False
                    if object_gating_enforced:
                        if not hint_diagnostic.get("selection_reason"):
                            hint_diagnostic["selection_reason"] = "not_enforceable"
                    else:
                        hint_diagnostic["selection_reason"] = "diagnostics_only"
                subset_diagnostic = self._subset_diagnostics_for_frame(
                    pattern,
                    tracker,
                    timestamp,
                    points_3d,
                    rigid_hint_triangulated_points,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                    generic_pose=selected_pose,
                    generic_score=selected_score,
                )
                guard = self._evaluate_reacquire_guard(tracker, selected_pose, selected_score)
                tracker.record_reprojection_score(selected_score)
                tracker.record_rigid_hint_pose(hint_diagnostic)
                tracker.record_subset_hypothesis(subset_diagnostic)
                tracker.record_reacquire_guard(guard)
                if guard.get("enforced") and guard.get("would_reject"):
                    rejected_pose = self._invalid_pose(timestamp)
                    poses[pattern.name] = rejected_pose
                    tracker.update(
                        rejected_pose,
                        invalid_reason="reprojection_guard_rejected",
                    )
                else:
                    poses[pattern.name] = selected_pose
                    if selected_source == "generic" and best_cluster_idx >= 0:
                        used_clusters.add(best_cluster_idx)
                    tracker.update(selected_pose)
            else:
                # Use prediction
                predicted = self.trackers[pattern.name].predict()
                predicted.timestamp = timestamp
                score = _empty_reprojection_score("no_valid_pose")
                tracker.record_reprojection_score(score)
                hint_diagnostic = self._evaluate_rigid_hint_pose(
                    pattern,
                    timestamp,
                    rigid_hint_triangulated_points,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                    generic_pose=None,
                    generic_score=score,
                )
                selected_pose = predicted
                if self._should_select_rigid_hint_pose(hint_diagnostic):
                    hint_pose = self._pose_from_payload(
                        hint_diagnostic.get("pose"),
                        fallback_timestamp=timestamp,
                    )
                    if hint_pose is not None and hint_pose.valid:
                        selected_pose = hint_pose
                        score = dict(hint_diagnostic.get("score") or _empty_reprojection_score())
                        hint_diagnostic["selected_for_pose"] = True
                        hint_diagnostic["selection_reason"] = "object_gating_enforced"
                        tracker.record_reprojection_score(score)
                    else:
                        hint_diagnostic["selected_for_pose"] = False
                        hint_diagnostic["selection_reason"] = "invalid_pose_payload"
                elif object_gating_enforced:
                    if not hint_diagnostic.get("selection_reason"):
                        hint_diagnostic["selection_reason"] = "not_enforceable"
                else:
                    hint_diagnostic["selection_reason"] = "diagnostics_only"

                poses[pattern.name] = selected_pose
                tracker.record_rigid_hint_pose(hint_diagnostic)
                tracker.record_subset_hypothesis(
                    self._subset_diagnostics_for_frame(
                        pattern,
                        tracker,
                        timestamp,
                        points_3d,
                        rigid_hint_triangulated_points,
                        camera_params,
                        observations_by_camera,
                        coordinate_space=coordinate_space,
                        generic_pose=selected_pose if selected_pose.valid else None,
                        generic_score=score,
                    )
                )
                if selected_pose.valid:
                    guard = self._evaluate_reacquire_guard(tracker, selected_pose, score)
                else:
                    guard = _empty_reacquire_guard(
                        enabled=self.reacquire_guard_config.shadow_enabled,
                        enforced=self.reacquire_guard_config.enforced,
                        reason="no_valid_pose",
                        thresholds=self.reacquire_guard_config.thresholds_dict(),
                    )
                tracker.record_reacquire_guard(guard)
                if guard.get("enforced") and guard.get("would_reject"):
                    rejected_pose = self._invalid_pose(timestamp)
                    poses[pattern.name] = rejected_pose
                    tracker.update(
                        rejected_pose,
                        invalid_reason="reprojection_guard_rejected",
                    )
                    continue
                self.trackers[pattern.name].update(selected_pose)
        
        return poses

    def _should_select_rigid_hint_pose(self, diagnostic: Dict[str, Any]) -> bool:
        if not self.object_gating_config.enabled or not self.object_gating_config.enforce:
            return False
        if not isinstance(diagnostic, dict):
            return False
        diagnostic["enforced"] = True
        candidate_points = int(diagnostic.get("candidate_points", 0))
        min_markers = max(3, int(self.object_gating_config.min_enforced_markers))
        if candidate_points < min_markers:
            diagnostic["selection_reason"] = "insufficient_rigid_hint_points"
            return False
        if not diagnostic.get("valid"):
            diagnostic["selection_reason"] = "invalid_rigid_hint_pose"
            return False
        score = diagnostic.get("score", {})
        if not isinstance(score, dict) or not score.get("scored", False):
            diagnostic["selection_reason"] = "unscored_rigid_hint_pose"
            return False
        if int(score.get("matched_marker_views", 0)) < min_markers * 2:
            diagnostic["selection_reason"] = "insufficient_matched_marker_views"
            return False
        diagnostic["selection_reason"] = "object_gating_enforced"
        return True

    @staticmethod
    def _pose_from_payload(
        payload: Any,
        *,
        fallback_timestamp: int,
    ) -> Optional[RigidBodyPose]:
        if not isinstance(payload, dict):
            return None
        try:
            position = np.asarray(payload["position"], dtype=np.float64).reshape(3)
            quaternion = _normalize_quaternion(
                np.asarray(payload["quaternion"], dtype=np.float64).reshape(4)
            )
            timestamp = int(payload.get("timestamp", fallback_timestamp))
            rms_error = float(payload.get("rms_error", 0.0))
            observed_markers = int(payload.get("observed_markers", 0))
            valid = bool(payload.get("valid", False))
        except (KeyError, TypeError, ValueError):
            return None
        if not np.isfinite(position).all() or not np.isfinite(quaternion).all():
            return None
        return RigidBodyPose(
            timestamp=timestamp,
            position=position,
            rotation=_rotation_from_wxyz(quaternion),
            quaternion=quaternion,
            rms_error=rms_error,
            observed_markers=observed_markers,
            valid=valid,
        )

    def _evaluate_rigid_hint_pose(
        self,
        pattern: MarkerPattern,
        timestamp: int,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
        generic_pose: Optional[RigidBodyPose],
        generic_score: Dict[str, Any],
    ) -> Dict[str, Any]:
        if not rigid_hint_triangulated_points:
            diagnostic = _empty_rigid_hint_pose("no_rigid_hint_points")
            diagnostic["enforced"] = bool(self.object_gating_config.enforce)
            diagnostic["generic_valid"] = bool(generic_pose.valid) if generic_pose is not None else False
            diagnostic["generic_score"] = dict(generic_score or _empty_reprojection_score())
            diagnostic["generic_rms_error_m"] = (
                float(generic_pose.rms_error)
                if generic_pose is not None and generic_pose.valid
                else 0.0
            )
            return diagnostic

        by_marker: Dict[int, Dict[str, Any]] = {}
        invalid_points = 0
        for payload in rigid_hint_triangulated_points:
            if not isinstance(payload, dict):
                invalid_points += 1
                continue
            if str(payload.get("rigid_name", pattern.name)) != pattern.name:
                continue
            if bool(payload.get("is_virtual", False)):
                continue
            try:
                marker_idx = int(payload["marker_idx"])
                point = np.asarray(payload["point"], dtype=np.float64).reshape(3)
            except (KeyError, TypeError, ValueError):
                invalid_points += 1
                continue
            if marker_idx < 0 or marker_idx >= pattern.num_markers or not np.isfinite(point).all():
                invalid_points += 1
                continue
            errors = payload.get("reprojection_errors_px", [])
            mean_error = (
                float(np.mean(np.asarray(errors, dtype=np.float64)))
                if isinstance(errors, list) and errors
                else 0.0
            )
            previous = by_marker.get(marker_idx)
            previous_error = float(previous.get("_mean_error_px", float("inf"))) if previous else float("inf")
            if previous is None or mean_error < previous_error:
                by_marker[marker_idx] = {
                    "point": point,
                    "contributing_rays": int(payload.get("contributing_rays", 0)),
                    "_mean_error_px": mean_error,
                }

        marker_indices = sorted(by_marker)
        candidate_points = len(marker_indices)
        generic_valid = bool(generic_pose.valid) if generic_pose is not None else False
        generic_rms = (
            float(generic_pose.rms_error)
            if generic_pose is not None and generic_pose.valid
            else 0.0
        )
        if candidate_points < 3:
            diagnostic = _empty_rigid_hint_pose("insufficient_rigid_hint_points")
            diagnostic.update(
                {
                    "evaluated": True,
                    "enforced": bool(self.object_gating_config.enforce),
                    "candidate_points": int(candidate_points),
                    "generic_valid": generic_valid,
                    "generic_rms_error_m": generic_rms,
                    "generic_score": dict(generic_score or _empty_reprojection_score()),
                    "marker_indices": [int(index) for index in marker_indices],
                }
            )
            return diagnostic

        observed = np.asarray([by_marker[index]["point"] for index in marker_indices], dtype=np.float64)
        reference = np.asarray(
            [pattern.marker_positions[index] for index in marker_indices],
            dtype=np.float64,
        )
        try:
            rotation, position, rms_error = KabschEstimator.estimate(reference, observed)
            quat_xyzw = Rotation.from_matrix(rotation).as_quat()
            quaternion = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=np.float64,
            )
            pose = RigidBodyPose(
                timestamp=timestamp,
                position=position,
                rotation=rotation,
                quaternion=quaternion,
                rms_error=float(rms_error),
                observed_markers=int(candidate_points),
                valid=bool(float(rms_error) <= self.max_rms_error_m),
            )
        except Exception:
            diagnostic = _empty_rigid_hint_pose("rigid_hint_solve_failed")
            diagnostic.update(
                {
                    "evaluated": True,
                    "enforced": bool(self.object_gating_config.enforce),
                    "candidate_points": int(candidate_points),
                    "generic_valid": generic_valid,
                    "generic_rms_error_m": generic_rms,
                    "generic_score": dict(generic_score or _empty_reprojection_score()),
                    "marker_indices": [int(index) for index in marker_indices],
                    "invalid_points": int(invalid_points),
                }
            )
            return diagnostic

        score = self._score_pose_reprojection(
            pose,
            pattern,
            camera_params,
            observations_by_camera,
            coordinate_space=coordinate_space,
        )
        generic_score_value = float((generic_score or {}).get("score", 0.0))
        hint_score_value = float(score.get("score", 0.0))
        position_delta_m = (
            float(np.linalg.norm(pose.position - generic_pose.position))
            if generic_pose is not None and generic_pose.valid and pose.valid
            else 0.0
        )
        rotation_delta_deg = (
            _quaternion_angle_deg(generic_pose.quaternion, pose.quaternion)
            if generic_pose is not None and generic_pose.valid and pose.valid
            else 0.0
        )
        reason = "ok"
        if not pose.valid:
            reason = "rms_error_too_high"
        return {
            "evaluated": True,
            "reason": reason,
            "diagnostics_only": not bool(self.object_gating_config.enforce),
            "enforced": bool(self.object_gating_config.enforce),
            "selected_for_pose": False,
            "selection_reason": "",
            "valid": bool(pose.valid),
            "generic_valid": generic_valid,
            "would_improve_score": bool(pose.valid and hint_score_value > generic_score_value),
            "candidate_points": int(candidate_points),
            "observed_markers": int(candidate_points),
            "real_ray_count": int(
                sum(int(by_marker[index].get("contributing_rays", 0)) for index in marker_indices)
            ),
            "virtual_marker_count": 0,
            "rms_error_m": float(pose.rms_error),
            "generic_rms_error_m": generic_rms,
            "score": dict(score),
            "generic_score": dict(generic_score or _empty_reprojection_score()),
            "score_delta": float(hint_score_value - generic_score_value),
            "position_delta_m": position_delta_m,
            "rotation_delta_deg": rotation_delta_deg,
            "marker_indices": [int(index) for index in marker_indices],
            "invalid_points": int(invalid_points),
            "pose": pose.to_dict(),
        }

    def _evaluate_subset_hypotheses(
        self,
        pattern: MarkerPattern,
        timestamp: int,
        points_3d: np.ndarray,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
        generic_pose: Optional[RigidBodyPose],
        generic_score: Dict[str, Any],
    ) -> Dict[str, Any]:
        config = self.subset_solve_config
        thresholds = config.thresholds_dict()
        budget_ms = self.subset_time_budget_ms
        effective_max_hypotheses = int(
            min(config.max_hypotheses, self.subset_max_hypotheses)
            if self.subset_max_hypotheses is not None
            else config.max_hypotheses
        )
        thresholds["effective_max_hypotheses"] = effective_max_hypotheses
        thresholds["time_budget_ms"] = float(budget_ms or 0.0)
        if not config.enabled:
            diagnostic = _empty_subset_hypothesis("disabled", thresholds)
            diagnostic["enabled"] = False
            return diagnostic

        started_ns = time.perf_counter_ns()
        generic_points = np.asarray(points_3d, dtype=np.float64).reshape(-1, 3)
        candidates: List[Dict[str, Any]] = []
        candidate_count = 0
        pruned_candidate_count = 0
        rejected_by_ambiguity = 0
        rejected_by_2d = 0
        rejected_by_rms = 0
        flip_risk_count = 0
        truncated = False
        time_budget_exceeded = False

        def budget_exceeded() -> bool:
            if budget_ms is None:
                return False
            elapsed_ms = float(time.perf_counter_ns() - started_ns) / 1_000_000.0
            return elapsed_ms >= float(budget_ms)

        def add_candidate(
            *,
            source: str,
            observed: np.ndarray,
            marker_indices: Tuple[int, ...],
            weights: np.ndarray,
            observed_indices: Optional[Tuple[int, ...]] = None,
        ) -> bool:
            nonlocal candidate_count
            nonlocal pruned_candidate_count
            nonlocal rejected_by_ambiguity
            nonlocal rejected_by_2d
            nonlocal rejected_by_rms
            nonlocal flip_risk_count
            nonlocal truncated
            nonlocal time_budget_exceeded

            if candidate_count >= effective_max_hypotheses:
                truncated = True
                return False
            if budget_exceeded():
                time_budget_exceeded = True
                truncated = True
                return False
            if self._subset_candidate_pruned(
                source=source,
                pattern=pattern,
                observed=observed,
                marker_indices=marker_indices,
                generic_pose=generic_pose,
            ):
                pruned_candidate_count += 1
                return True
            candidate_count += 1
            candidate = self._build_subset_candidate(
                pattern,
                timestamp,
                source=source,
                observed=observed,
                marker_indices=marker_indices,
                weights=weights,
                camera_params=camera_params,
                observations_by_camera=observations_by_camera,
                coordinate_space=coordinate_space,
                generic_pose=generic_pose,
                generic_score=generic_score,
                observed_indices=observed_indices,
            )
            if candidate.get("ambiguous"):
                rejected_by_ambiguity += 1
            if candidate.get("rejected_by_2d_score"):
                rejected_by_2d += 1
            if candidate.get("rejected_by_rms"):
                rejected_by_rms += 1
            if candidate.get("flip_risk"):
                flip_risk_count += 1
            if candidate.get("rankable"):
                candidates.append(candidate)
            return True

        hint_markers = self._rigid_hint_markers_by_index(pattern, rigid_hint_triangulated_points)
        hint_indices = sorted(hint_markers)
        for subset_size in config.subset_sizes:
            size = int(subset_size)
            if size < 3 or size > pattern.num_markers or size > len(hint_indices):
                continue
            for marker_subset in combinations(hint_indices, size):
                observed = np.asarray(
                    [hint_markers[index]["point"] for index in marker_subset],
                    dtype=np.float64,
                )
                weights = np.asarray(
                    [hint_markers[index]["weight"] for index in marker_subset],
                    dtype=np.float64,
                )
                if not add_candidate(
                    source="rigid_hint_subset",
                    observed=observed,
                    marker_indices=tuple(int(index) for index in marker_subset),
                    weights=weights,
                ):
                    break
            if truncated:
                break

        if not truncated and 3 <= len(generic_points) <= config.max_observed_points:
            observed_subsets = self._prioritized_generic_observed_subsets(
                generic_points,
                pattern,
                generic_pose,
            )
            marker_range = tuple(range(pattern.num_markers))
            for observed_subset in observed_subsets:
                size = len(observed_subset)
                if size not in {int(value) for value in config.subset_sizes}:
                    continue
                if size < 3 or size > pattern.num_markers:
                    continue
                observed = generic_points[list(observed_subset)]
                for marker_subset in combinations(marker_range, size):
                    for marker_order in permutations(marker_subset):
                        weights = np.ones(size, dtype=np.float64)
                        if not add_candidate(
                            source="generic_subset",
                            observed=observed,
                            marker_indices=tuple(int(index) for index in marker_order),
                            weights=weights,
                            observed_indices=tuple(int(index) for index in observed_subset),
                        ):
                            break
                    if truncated:
                        break
                if truncated:
                    break

        if candidate_count <= 0:
            diagnostic = _empty_subset_hypothesis(
                "time_budget_exceeded" if time_budget_exceeded else "no_subset_candidates",
                thresholds,
            )
            diagnostic["evaluated"] = True
            diagnostic["time_budget_ms"] = float(budget_ms or 0.0)
            diagnostic["time_budget_exceeded"] = bool(time_budget_exceeded)
            diagnostic["effective_max_hypotheses"] = int(effective_max_hypotheses)
            diagnostic["truncated"] = bool(truncated)
            return diagnostic

        ranked = sorted(
            candidates,
            key=lambda item: (
                float(item.get("combined_score", 0.0)),
                float(item.get("score", 0.0)),
                int(item.get("matched_marker_views", 0)),
                -float(item.get("p95_error_px", 0.0)),
                -float(item.get("rms_error_m", 0.0)),
            ),
            reverse=True,
        )
        best = ranked[0] if ranked else {}
        second = ranked[1] if len(ranked) > 1 else {}
        best_score = float(best.get("score", 0.0)) if best else 0.0
        second_score = float(second.get("score", 0.0)) if second else 0.0
        best_combined_score = float(best.get("combined_score", 0.0)) if best else 0.0
        second_combined_score = float(second.get("combined_score", 0.0)) if second else 0.0
        margin = float(best_score - second_score) if best else 0.0
        combined_margin = float(best_combined_score - second_combined_score) if best else 0.0
        generic_score_value = float((generic_score or {}).get("score", 0.0))
        weighted_solve = self._subset_weighted_solve_summary(best)
        subset_adoption_ready = bool(
            best
            and best.get("valid")
            and not best.get("flip_risk")
            and combined_margin >= config.min_margin
            and best_score >= max(config.min_score, generic_score_value)
            and int(best.get("observed_markers", 0)) >= max(3, pattern.num_markers - 1)
        )
        return {
            "evaluated": True,
            "reason": "ok" if ranked else "no_rankable_candidates",
            "diagnostics_only": bool(config.diagnostics_only),
            "enabled": True,
            "thresholds": thresholds,
            "time_budget_ms": float(budget_ms or 0.0),
            "time_budget_exceeded": bool(time_budget_exceeded),
            "effective_max_hypotheses": int(effective_max_hypotheses),
            "generic_valid": bool(generic_pose.valid) if generic_pose is not None else False,
            "candidate_count": int(candidate_count),
            "pruned_candidate_count": int(pruned_candidate_count),
            "valid_candidate_count": int(len(ranked)),
            "rejected_by_ambiguity": int(rejected_by_ambiguity),
            "rejected_by_2d_score": int(rejected_by_2d),
            "rejected_by_rms": int(rejected_by_rms),
            "flip_risk_count": int(flip_risk_count),
            "truncated": bool(truncated),
            "best": self._subset_candidate_public(best),
            "second": self._subset_candidate_public(second),
            "best_score": best_score,
            "second_score": second_score,
            "best_combined_score": best_combined_score,
            "second_combined_score": second_combined_score,
            "margin": margin,
            "combined_margin": combined_margin,
            "generic_score": generic_score_value,
            "score_delta": float(best_score - generic_score_value),
            "subset_adoption_ready": subset_adoption_ready,
            "weighted_solve": weighted_solve,
        }

    def _build_subset_candidate(
        self,
        pattern: MarkerPattern,
        timestamp: int,
        *,
        source: str,
        observed: np.ndarray,
        marker_indices: Tuple[int, ...],
        weights: np.ndarray,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        coordinate_space: str,
        generic_pose: Optional[RigidBodyPose],
        generic_score: Dict[str, Any],
        observed_indices: Optional[Tuple[int, ...]] = None,
    ) -> Dict[str, Any]:
        reference = np.asarray(
            [pattern.marker_positions[index] for index in marker_indices],
            dtype=np.float64,
        )
        ambiguous = self._subset_is_ambiguous(pattern, tuple(sorted(marker_indices)))
        try:
            rotation, position, rms_error = KabschEstimator.estimate_weighted(
                reference,
                observed,
                weights,
            )
            quat_xyzw = Rotation.from_matrix(rotation).as_quat()
            quaternion = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=np.float64,
            )
            pose = RigidBodyPose(
                timestamp=timestamp,
                position=position,
                rotation=rotation,
                quaternion=quaternion,
                rms_error=float(rms_error),
                observed_markers=int(len(marker_indices)),
                valid=bool(float(rms_error) <= self.max_rms_error_m),
            )
            pose_payload = pose.to_dict()
        except Exception:
            return {
                "source": source,
                "valid": False,
                "rankable": False,
                "rejected_by_rms": True,
                "rejected_by_2d_score": True,
                "ambiguous": bool(ambiguous),
                "flip_risk": False,
                "marker_indices": [int(index) for index in marker_indices],
                "observed_indices": [int(index) for index in observed_indices or ()],
                "score": 0.0,
                "rms_error_m": float("inf"),
            }

        score = self._score_pose_reprojection(
            pose,
            pattern,
            camera_params,
            observations_by_camera,
            coordinate_space=coordinate_space,
        )
        score_value = float(score.get("score", 0.0))
        p95_error = float(score.get("p95_error_px", 0.0))
        rejected_by_rms = not pose.valid
        rejected_by_2d = (
            not score.get("scored", False)
            or score_value < self.subset_solve_config.min_score
            or p95_error > self.subset_solve_config.max_p95_error_px
        )
        rotation_delta_deg = (
            _quaternion_angle_deg(generic_pose.quaternion, pose.quaternion)
            if generic_pose is not None and generic_pose.valid and pose.valid
            else 0.0
        )
        position_delta_m = (
            float(np.linalg.norm(pose.position - generic_pose.position))
            if generic_pose is not None and generic_pose.valid and pose.valid
            else 0.0
        )
        flip_risk = bool(
            generic_pose is not None
            and generic_pose.valid
            and pose.valid
            and rotation_delta_deg > self.subset_solve_config.max_rotation_delta_deg
        )
        rankable = bool(
            pose.valid
            and score.get("scored", False)
            and not rejected_by_2d
            and not ambiguous
        )
        coverage = float(len(marker_indices) / max(1, pattern.num_markers))
        temporal_penalty = min(
            1.0,
            (position_delta_m / max(1e-6, self.subset_solve_config.prediction_gate_m))
            + (rotation_delta_deg / max(1e-6, self.subset_solve_config.max_rotation_delta_deg)),
        )
        source_bonus = (
            self.subset_solve_config.source_priority_bonus
            if source == "rigid_hint_subset"
            else 0.0
        )
        combined_score = float(
            np.clip(
                score_value
                + self.subset_solve_config.coverage_weight * coverage
                + source_bonus
                - self.subset_solve_config.temporal_penalty_weight * temporal_penalty
                - (self.subset_solve_config.flip_penalty if flip_risk else 0.0),
                0.0,
                1.5,
            )
        )
        return {
            "source": source,
            "valid": bool(pose.valid),
            "rankable": rankable,
            "ambiguous": bool(ambiguous),
            "rejected_by_rms": bool(rejected_by_rms),
            "rejected_by_2d_score": bool(rejected_by_2d),
            "flip_risk": flip_risk,
            "marker_indices": [int(index) for index in marker_indices],
            "observed_indices": [int(index) for index in observed_indices or ()],
            "observed_markers": int(len(marker_indices)),
            "rms_error_m": float(pose.rms_error),
            "score": score_value,
            "combined_score": combined_score,
            "coverage": coverage,
            "temporal_penalty": temporal_penalty,
            "source_bonus": float(source_bonus),
            "score_detail": dict(score),
            "generic_score": float((generic_score or {}).get("score", 0.0)),
            "score_delta": float(score_value - float((generic_score or {}).get("score", 0.0))),
            "mean_error_px": float(score.get("mean_error_px", 0.0)),
            "p95_error_px": p95_error,
            "matched_marker_views": int(score.get("matched_marker_views", 0)),
            "position_delta_m": position_delta_m,
            "rotation_delta_deg": rotation_delta_deg,
            "weights": [float(value) for value in np.asarray(weights, dtype=np.float64).reshape(-1)],
            "pose": pose_payload,
        }

    def _rigid_hint_markers_by_index(
        self,
        pattern: MarkerPattern,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
    ) -> Dict[int, Dict[str, Any]]:
        by_marker: Dict[int, Dict[str, Any]] = {}
        for payload in rigid_hint_triangulated_points or []:
            if not isinstance(payload, dict):
                continue
            if str(payload.get("rigid_name", pattern.name)) != pattern.name:
                continue
            if bool(payload.get("is_virtual", False)):
                continue
            try:
                marker_idx = int(payload["marker_idx"])
                point = np.asarray(payload["point"], dtype=np.float64).reshape(3)
            except (KeyError, TypeError, ValueError):
                continue
            if marker_idx < 0 or marker_idx >= pattern.num_markers or not np.isfinite(point).all():
                continue
            errors = payload.get("reprojection_errors_px", [])
            mean_error = (
                float(np.mean(np.asarray(errors, dtype=np.float64)))
                if isinstance(errors, list) and errors
                else 0.0
            )
            contributing_rays = int(payload.get("contributing_rays", 0))
            weight = max(0.1, float(contributing_rays)) / (1.0 + mean_error)
            previous = by_marker.get(marker_idx)
            previous_weight = float(previous.get("weight", -1.0)) if previous else -1.0
            if previous is None or weight > previous_weight:
                by_marker[marker_idx] = {
                    "point": point,
                    "weight": float(weight),
                }
        return by_marker

    def _subset_candidate_pruned(
        self,
        *,
        source: str,
        pattern: MarkerPattern,
        observed: np.ndarray,
        marker_indices: Tuple[int, ...],
        generic_pose: Optional[RigidBodyPose],
    ) -> bool:
        if source == "rigid_hint_subset":
            return False
        if generic_pose is None or not generic_pose.valid:
            return False
        predicted = (
            generic_pose.rotation @ np.asarray(
                [pattern.marker_positions[index] for index in marker_indices],
                dtype=np.float64,
            ).T
        ).T + generic_pose.position.reshape(1, 3)
        residuals = np.linalg.norm(np.asarray(observed, dtype=np.float64) - predicted, axis=1)
        return bool(float(np.mean(residuals)) > self.subset_solve_config.prediction_gate_m)

    def _prioritized_generic_observed_subsets(
        self,
        generic_points: np.ndarray,
        pattern: MarkerPattern,
        generic_pose: Optional[RigidBodyPose],
    ) -> List[Tuple[int, ...]]:
        all_subsets: List[Tuple[float, Tuple[int, ...]]] = []
        sizes = {int(size) for size in self.subset_solve_config.subset_sizes}
        point_indices = tuple(range(len(generic_points)))
        predicted_markers = None
        if generic_pose is not None and generic_pose.valid:
            predicted_markers = (
                generic_pose.rotation @ pattern.marker_positions.T
            ).T + generic_pose.position.reshape(1, 3)
        for size in sorted(sizes, reverse=True):
            if size < 3 or size > len(generic_points) or size > pattern.num_markers:
                continue
            for subset in combinations(point_indices, size):
                points = generic_points[list(subset)]
                if predicted_markers is None:
                    priority = 0.0
                else:
                    distances = [
                        float(np.min(np.linalg.norm(predicted_markers - point.reshape(1, 3), axis=1)))
                        for point in points
                    ]
                    priority = float(np.mean(distances))
                    if priority > self.subset_solve_config.prediction_gate_m:
                        continue
                all_subsets.append((priority, tuple(int(index) for index in subset)))
        all_subsets.sort(key=lambda item: (item[0], -len(item[1]), item[1]))
        limit = max(1, int(self.subset_solve_config.max_generic_observed_subsets))
        return [subset for _, subset in all_subsets[:limit]]

    def _subset_is_ambiguous(
        self,
        pattern: MarkerPattern,
        marker_indices: Tuple[int, ...],
    ) -> bool:
        if len(marker_indices) >= pattern.num_markers:
            return False
        reference = pattern.marker_positions
        profile = self._subset_distance_profile(reference[list(marker_indices)])
        for other in combinations(range(pattern.num_markers), len(marker_indices)):
            if tuple(other) == tuple(marker_indices):
                continue
            other_profile = self._subset_distance_profile(reference[list(other)])
            if len(profile) != len(other_profile):
                continue
            delta = float(np.linalg.norm(profile - other_profile))
            if delta <= self.subset_solve_config.ambiguous_subset_delta_m:
                return True
        return False

    @staticmethod
    def _subset_distance_profile(points: np.ndarray) -> np.ndarray:
        distances = []
        for idx_a, idx_b in combinations(range(len(points)), 2):
            distances.append(float(np.linalg.norm(points[idx_a] - points[idx_b])))
        return np.sort(np.asarray(distances, dtype=np.float64))

    @staticmethod
    def _subset_candidate_public(candidate: Dict[str, Any]) -> Dict[str, Any]:
        if not candidate:
            return {}
        return {
            "source": str(candidate.get("source", "")),
            "valid": bool(candidate.get("valid", False)),
            "rankable": bool(candidate.get("rankable", False)),
            "ambiguous": bool(candidate.get("ambiguous", False)),
            "flip_risk": bool(candidate.get("flip_risk", False)),
            "marker_indices": [int(index) for index in candidate.get("marker_indices", [])],
            "observed_indices": [int(index) for index in candidate.get("observed_indices", [])],
            "observed_markers": int(candidate.get("observed_markers", 0)),
            "score": float(candidate.get("score", 0.0)),
            "combined_score": float(candidate.get("combined_score", 0.0)),
            "coverage": float(candidate.get("coverage", 0.0)),
            "temporal_penalty": float(candidate.get("temporal_penalty", 0.0)),
            "source_bonus": float(candidate.get("source_bonus", 0.0)),
            "score_delta": float(candidate.get("score_delta", 0.0)),
            "rms_error_m": float(candidate.get("rms_error_m", 0.0)),
            "mean_error_px": float(candidate.get("mean_error_px", 0.0)),
            "p95_error_px": float(candidate.get("p95_error_px", 0.0)),
            "matched_marker_views": int(candidate.get("matched_marker_views", 0)),
            "position_delta_m": float(candidate.get("position_delta_m", 0.0)),
            "rotation_delta_deg": float(candidate.get("rotation_delta_deg", 0.0)),
            "pose": dict(candidate.get("pose", {})),
        }

    def _subset_weighted_solve_summary(self, candidate: Dict[str, Any]) -> Dict[str, Any]:
        if not candidate:
            return {"valid": False, "reason": "no_best_candidate"}
        return {
            "valid": bool(candidate.get("valid", False)),
            "source": str(candidate.get("source", "")),
            "observed_markers": int(candidate.get("observed_markers", 0)),
            "rms_error_m": float(candidate.get("rms_error_m", 0.0)),
            "weights": [float(value) for value in candidate.get("weights", [])],
            "marker_indices": [int(index) for index in candidate.get("marker_indices", [])],
        }

    @staticmethod
    def _invalid_pose(timestamp: int) -> RigidBodyPose:
        return RigidBodyPose(
            timestamp=timestamp,
            position=np.zeros(3, dtype=np.float64),
            rotation=np.eye(3, dtype=np.float64),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
            valid=False,
        )

    def _evaluate_reacquire_guard(
        self,
        tracker: RigidBodyTracker,
        pose: RigidBodyPose,
        score: Dict[str, Any],
    ) -> Dict[str, Any]:
        config = self.reacquire_guard_config
        thresholds = config.thresholds_dict()
        if not config.shadow_enabled and not config.enforced:
            return _empty_reacquire_guard(
                enabled=False,
                enforced=False,
                reason="disabled",
                thresholds=thresholds,
            )
        guarded_continue = tracker.should_guard_post_reacquire_continue(
            int(config.post_reacquire_continue_frames)
        )
        if tracker.mode != TrackMode.REACQUIRE and not guarded_continue:
            return _empty_reacquire_guard(
                enabled=config.shadow_enabled,
                enforced=config.enforced,
                reason="not_guarded_mode",
                thresholds=thresholds,
            )
        if not pose.valid:
            return _empty_reacquire_guard(
                enabled=config.shadow_enabled,
                enforced=config.enforced,
                reason="invalid_pose",
                thresholds=thresholds,
            )

        prediction = tracker.peek_prediction(pose.timestamp)
        position_innovation_m = (
            float(np.linalg.norm(pose.position - prediction.position))
            if prediction.valid
            else 0.0
        )
        rotation_innovation_deg = (
            _quaternion_angle_deg(prediction.quaternion, pose.quaternion)
            if prediction.valid
            else 0.0
        )

        reasons: List[str] = []
        if not score.get("scored", False):
            reasons.append(f"score_not_available:{score.get('reason', 'unknown')}")
        if int(score.get("matched_marker_views", 0)) < config.min_matched_marker_views:
            reasons.append("insufficient_matched_marker_views")
        if int(score.get("missing_marker_views", 0)) > config.max_missing_marker_views:
            reasons.append("too_many_missing_marker_views")
        if float(score.get("mean_error_px", 0.0)) > config.max_mean_reprojection_error_px:
            reasons.append("mean_reprojection_error_too_high")
        if float(score.get("p95_error_px", 0.0)) > config.max_p95_reprojection_error_px:
            reasons.append("p95_reprojection_error_too_high")
        if (
            not config.allow_duplicate_assignment
            and int(score.get("duplicate_assignment_count", 0)) > 0
        ):
            reasons.append("duplicate_assignment")
        if position_innovation_m > config.max_position_innovation_m:
            reasons.append("position_innovation_too_high")
        if rotation_innovation_deg > config.max_rotation_innovation_deg:
            reasons.append("rotation_innovation_too_high")

        passed = not reasons
        return {
            "enabled": bool(config.shadow_enabled or config.enforced),
            "enforced": bool(config.enforced),
            "evaluated": True,
            "passed": bool(passed),
            "would_reject": bool(not passed),
            "reason": "ok" if passed else ",".join(reasons),
            "thresholds": thresholds,
            "score": dict(score),
            "position_innovation_m": float(position_innovation_m),
            "rotation_innovation_deg": float(rotation_innovation_deg),
            "rejected_count": 0,
        }

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
            cost = np.asarray(
                [
                    [
                        float(
                            np.linalg.norm(
                                np.asarray(projected, dtype=np.float64)
                                - np.asarray(observed, dtype=np.float64)
                            )
                        )
                        for observed in observed_uvs
                    ]
                    for projected in projected_uvs
                ],
                dtype=np.float64,
            )
            invalid_cost = 1e9
            gated_cost = np.where(cost <= self.reprojection_match_gate_px, cost, invalid_cost)
            row_ind, col_ind = linear_sum_assignment(gated_cost)
            assigned_blob_indices = set()
            for row, col in zip(row_ind, col_ind):
                error = float(gated_cost[row, col])
                if error >= invalid_cost:
                    continue
                matched += 1
                residuals.append(error)
                assigned_blob_indices.add(int(col))

            unexpected_blobs += max(0, len(observed_uvs) - len(assigned_blob_indices))

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
