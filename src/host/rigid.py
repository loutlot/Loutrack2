"""
Rigid body tracking module for pose estimation from 3D point clouds.

Provides functionality to:
- Cluster 3D points using DBSCAN
- Estimate rigid body pose using Kabsch algorithm
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
    ambiguous_blob_min_separation_px: float = 0.60
    ambiguous_blob_diameter_overlap_ratio: float = 0.30
    ambiguous_marker_assignment_min_margin_px: float = 0.29
    body_level_2d_recovery: bool = False
    body_level_2d_max_offsets: int = 12
    body_level_2d_nearest_per_marker: int = 2
    body_level_2d_assignment_nearest_per_marker: int = 3
    body_level_2d_trigger_max_assignments: int = 3
    body_level_2d_max_distance_scale: float = 0.12
    body_level_2d_min_margin: float = 0.0
    body_level_2d_candidate_cache: bool = False
    body_level_2d_cache_max_bins: int = 4
    body_level_2d_cache_early_exit: bool = False
    body_level_2d_keep_nbest_diagnostics: bool = True
    skip_generic_search_when_object_gated: bool = False
    temporal_2d_ownership_recovery: bool = False
    temporal_2d_motion_prediction: bool = False

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "pixel_min": float(self.pixel_min),
            "pixel_max": float(self.pixel_max),
            "single_ray_confidence_min": float(self.single_ray_confidence_min),
            "ambiguous_blob_min_separation_px": float(
                self.ambiguous_blob_min_separation_px
            ),
            "ambiguous_blob_diameter_overlap_ratio": float(
                self.ambiguous_blob_diameter_overlap_ratio
            ),
            "ambiguous_marker_assignment_min_margin_px": float(
                self.ambiguous_marker_assignment_min_margin_px
            ),
            "body_level_2d_recovery": bool(self.body_level_2d_recovery),
            "body_level_2d_max_offsets": int(self.body_level_2d_max_offsets),
            "body_level_2d_nearest_per_marker": int(
                self.body_level_2d_nearest_per_marker
            ),
            "body_level_2d_assignment_nearest_per_marker": int(
                self.body_level_2d_assignment_nearest_per_marker
            ),
            "body_level_2d_trigger_max_assignments": int(
                self.body_level_2d_trigger_max_assignments
            ),
            "body_level_2d_max_distance_scale": float(
                self.body_level_2d_max_distance_scale
            ),
            "body_level_2d_min_margin": float(self.body_level_2d_min_margin),
            "body_level_2d_candidate_cache": bool(
                self.body_level_2d_candidate_cache
            ),
            "body_level_2d_cache_max_bins": int(self.body_level_2d_cache_max_bins),
            "body_level_2d_cache_early_exit": bool(
                self.body_level_2d_cache_early_exit
            ),
            "body_level_2d_keep_nbest_diagnostics": bool(
                self.body_level_2d_keep_nbest_diagnostics
            ),
            "skip_generic_search_when_object_gated": bool(
                self.skip_generic_search_when_object_gated
            ),
            "temporal_2d_ownership_recovery": bool(
                self.temporal_2d_ownership_recovery
            ),
            "temporal_2d_motion_prediction": bool(
                self.temporal_2d_motion_prediction
            ),
            "enforce": bool(self.enforce),
            "activation_mode": str(self.activation_mode),
            "min_enforced_markers": int(self.min_enforced_markers),
        }


@dataclass(frozen=True)
class PoseContinuityGuardConfig:
    """Temporal pose gate for low-marker occlusion candidates."""

    enabled: bool = False
    enforced: bool = False
    apply_to_occlusion_only: bool = True
    hold_prediction_on_reject: bool = True
    max_position_innovation_m: float = 0.08
    max_rotation_innovation_deg: float = 90.0
    max_angular_velocity_deg_s: float = 2500.0
    max_angular_accel_deg_s2: float = 200000.0

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "enforced": bool(self.enforced),
            "apply_to_occlusion_only": bool(self.apply_to_occlusion_only),
            "hold_prediction_on_reject": bool(self.hold_prediction_on_reject),
            "max_position_innovation_m": float(self.max_position_innovation_m),
            "max_rotation_innovation_deg": float(self.max_rotation_innovation_deg),
            "max_angular_velocity_deg_s": float(self.max_angular_velocity_deg_s),
            "max_angular_accel_deg_s2": float(self.max_angular_accel_deg_s2),
        }


@dataclass(frozen=True)
class PositionContinuityGuardConfig:
    """Acceleration-limited position guard for low-marker occlusion candidates."""

    enabled: bool = False
    enforced: bool = False
    apply_to_occlusion_only: bool = True
    max_accel_m_s2: float = 60.0
    max_velocity_m_s: float = 8.0

    def thresholds_dict(self) -> Dict[str, Any]:
        return {
            "enabled": bool(self.enabled),
            "enforced": bool(self.enforced),
            "apply_to_occlusion_only": bool(self.apply_to_occlusion_only),
            "max_accel_m_s2": float(self.max_accel_m_s2),
            "max_velocity_m_s": float(self.max_velocity_m_s),
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
        "ambiguous_assignment_count": 0,
        "marker_margin_assignment_count": 0,
        "markers_with_two_or_more_rays": 0,
        "markers_with_one_ray": 0,
        "single_ray_candidates": 0,
        "generic_fallback_blob_count": 0,
        "allow_single_ray": False,
        "thresholds": dict(thresholds or {}),
        "per_marker_ray_count": [],
        "per_camera": {},
    }


def compact_object_gating_diagnostics(
    gating: Dict[str, Any],
    *,
    include_detail: bool = False,
) -> Dict[str, Any]:
    """Return a lightweight object-gating payload for GUI/status diagnostics."""
    if not isinstance(gating, dict):
        return {}
    compact_keys = {
        "enabled",
        "enforced",
        "diagnostics_only",
        "evaluated",
        "reason",
        "mode",
        "prediction_valid",
        "confidence",
        "pixel_gate_px",
        "camera_count",
        "marker_count",
        "candidate_window_count",
        "assigned_marker_views",
        "unmatched_marker_views",
        "duplicate_assignment_count",
        "ambiguous_assignment_count",
        "marker_margin_assignment_count",
        "markers_with_two_or_more_rays",
        "markers_with_one_ray",
        "single_ray_candidates",
        "generic_fallback_blob_count",
        "allow_single_ray",
        "per_marker_ray_count",
        "body_assignment",
        "thresholds",
    }
    payload = {key: gating[key] for key in compact_keys if key in gating}
    per_camera = gating.get("per_camera")
    if isinstance(per_camera, dict):
        if include_detail:
            payload["per_camera"] = {
                str(camera_id): dict(camera_payload)
                for camera_id, camera_payload in per_camera.items()
                if isinstance(camera_payload, dict)
            }
        else:
            payload["per_camera"] = {
                str(camera_id): {
                    key: camera_payload.get(key)
                    for key in (
                        "blob_count",
                        "projected_marker_count",
                        "assigned_marker_views",
                        "unmatched_marker_views",
                        "ambiguous_assignment_count",
                        "marker_margin_assignment_count",
                        "body_shifted_assignment",
                        "body_level_2d_cache_hit",
                        "body_level_2d_cache_early_exit",
                        "body_shifted_mean_normalized_cost",
                        "body_level_2d_nbest_candidate_count",
                        "body_level_2d_nbest_margin",
                        "temporal_2d_ownership_assignment",
                        "temporal_2d_ownership_mean_cost",
                    )
                    if key in camera_payload
                }
                for camera_id, camera_payload in per_camera.items()
                if isinstance(camera_payload, dict)
            }
    payload["detail_sampled"] = bool(include_detail)
    return payload


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


def _empty_pose_continuity_guard(
    *,
    enabled: bool = False,
    enforced: bool = False,
    reason: str = "not_evaluated",
    thresholds: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "enabled": bool(enabled),
        "enforced": bool(enforced),
        "evaluated": False,
        "passed": True,
        "would_reject": False,
        "held_prediction": False,
        "held_rotation": False,
        "reason": str(reason),
        "thresholds": dict(thresholds or {}),
        "occluded": False,
        "observed_markers": 0,
        "expected_markers": 0,
        "matched_marker_views": 0,
        "expected_marker_views": 0,
        "missing_marker_views": 0,
        "position_innovation_m": 0.0,
        "rotation_innovation_deg": 0.0,
        "angular_velocity_deg_s": 0.0,
        "previous_angular_velocity_deg_s": 0.0,
        "angular_accel_deg_s2": 0.0,
        "evaluated_count": 0,
        "would_reject_count": 0,
        "held_count": 0,
    }


def _empty_position_continuity_guard(
    *,
    enabled: bool = False,
    enforced: bool = False,
    reason: str = "not_evaluated",
    thresholds: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "enabled": bool(enabled),
        "enforced": bool(enforced),
        "evaluated": False,
        "passed": True,
        "would_reject": False,
        "clamped_position": False,
        "reason": str(reason),
        "thresholds": dict(thresholds or {}),
        "occluded": False,
        "observed_markers": 0,
        "expected_markers": 0,
        "matched_marker_views": 0,
        "expected_marker_views": 0,
        "missing_marker_views": 0,
        "position_innovation_m": 0.0,
        "position_velocity_m_s": 0.0,
        "previous_velocity_m_s": 0.0,
        "position_accel_m_s2": 0.0,
        "evaluated_count": 0,
        "would_reject_count": 0,
        "clamped_count": 0,
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


def _quaternion_delta_rotvec(current: np.ndarray, previous: np.ndarray) -> np.ndarray:
    q_curr = _normalize_quaternion(current)
    q_prev = _normalize_quaternion(previous)
    w1, x1, y1, z1 = (float(value) for value in q_curr)
    w2, x2, y2, z2 = (float(q_prev[0]), -float(q_prev[1]), -float(q_prev[2]), -float(q_prev[3]))
    delta = np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )
    delta = _normalize_quaternion(delta)
    if float(delta[0]) < 0.0:
        delta = -delta
    vector = delta[1:4]
    vector_norm = float(np.linalg.norm(vector))
    if vector_norm <= 1e-12:
        return np.zeros(3, dtype=np.float64)
    angle = 2.0 * float(np.arctan2(vector_norm, float(delta[0])))
    return vector * (angle / vector_norm)


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

    _PAIR_INDEX_CACHE: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    
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
    def _ordered_pair_distances(points: np.ndarray) -> np.ndarray:
        arr = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        point_count = int(len(arr))
        if point_count < 2:
            return np.zeros(0, dtype=np.float64)
        pair_indices = KabschEstimator._PAIR_INDEX_CACHE.get(point_count)
        if pair_indices is None:
            pair_indices = np.triu_indices(point_count, k=1)
            KabschEstimator._PAIR_INDEX_CACHE[point_count] = pair_indices
        idx_a, idx_b = pair_indices
        deltas = arr[idx_a] - arr[idx_b]
        return np.sqrt(np.einsum("ij,ij->i", deltas, deltas))

    @staticmethod
    def _pair_distance_rms(
        reference_distances: np.ndarray,
        candidate_points: np.ndarray,
    ) -> float:
        candidate_distances = KabschEstimator._ordered_pair_distances(candidate_points)
        if len(candidate_distances) != len(reference_distances):
            return float("inf")
        delta = candidate_distances - reference_distances
        return float(np.sqrt(np.mean(delta * delta))) if len(delta) else 0.0

    @staticmethod
    def _ranked_kabsch_candidate_count(total_candidates: int) -> int:
        if total_candidates <= 0:
            return 0
        return min(int(total_candidates), 48)
    
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
            ranked: List[Tuple[float, Tuple[int, ...], Tuple[int, ...]]] = []
            observed_pair_distances = KabschEstimator._ordered_pair_distances(
                observed_points
            )
            for subset in combinations(range(n_ref), n_obs):
                ref_subset = reference_points[list(subset)]
                for perm in permutations(range(n_obs)):
                    ref_perm = ref_subset[list(perm)]
                    ranked.append(
                        (
                            KabschEstimator._pair_distance_rms(
                                observed_pair_distances,
                                ref_perm,
                            ),
                            tuple(int(index) for index in subset),
                            tuple(int(index) for index in perm),
                        )
                    )

            ranked.sort(key=lambda item: item[0])
            max_kabsch = KabschEstimator._ranked_kabsch_candidate_count(len(ranked))
            for _pair_error, subset, perm in ranked[:max_kabsch]:
                ref_subset = reference_points[list(subset)]
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
            best_error = float('inf')
            best_obs = None
            
            # For small datasets, rank by invariant pair-distance error first so
            # only the most plausible correspondences pay the SVD/Kabsch cost.
            if n_obs <= 10 and n_ref <= 5:
                ranked: List[Tuple[float, Tuple[int, ...], Tuple[int, ...]]] = []
                reference_pair_distances = KabschEstimator._ordered_pair_distances(
                    reference_points
                )
                for combo in combinations(range(n_obs), n_ref):
                    obs_subset = observed_points[list(combo)]
                    for perm in permutations(range(n_ref)):
                        obs_perm = obs_subset[list(perm)]
                        ranked.append(
                            (
                                KabschEstimator._pair_distance_rms(
                                    reference_pair_distances,
                                    obs_perm,
                                ),
                                tuple(int(index) for index in combo),
                                tuple(int(index) for index in perm),
                            )
                        )

                ranked.sort(key=lambda item: item[0])
                max_kabsch = KabschEstimator._ranked_kabsch_candidate_count(
                    len(ranked)
                )
                for _pair_error, combo, perm in ranked[:max_kabsch]:
                    obs_subset = observed_points[list(combo)]
                    obs_perm = obs_subset[list(perm)]
                    try:
                        _, _, error = KabschEstimator.estimate(obs_perm, reference_points)
                        if error < best_error:
                            best_error = float(error)
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
        self._angular_velocity = np.zeros(3)
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
        self._pose_continuity_guard_evaluated_count = 0
        self._pose_continuity_guard_would_reject_count = 0
        self._pose_continuity_guard_held_count = 0
        self._last_pose_continuity_guard: Dict[str, Any] = _empty_pose_continuity_guard()
        self._position_continuity_guard_evaluated_count = 0
        self._position_continuity_guard_would_reject_count = 0
        self._position_continuity_guard_clamped_count = 0
        self._last_position_continuity_guard: Dict[str, Any] = (
            _empty_position_continuity_guard()
        )
        self._prediction_hold_frames = 0
        self._last_pose_source = "init"
        self._prediction_cache_timestamp: Optional[int] = None
        self._prediction_cache: Optional[PredictedPose] = None

    def _invalidate_prediction_cache_locked(self) -> None:
        self._prediction_cache_timestamp = None
        self._prediction_cache = None
    
    def update(
        self,
        pose: RigidBodyPose,
        *,
        invalid_reason: Optional[str] = None,
        stage_reacquire: bool = True,
        prediction_hold: bool = False,
        pose_source: str = "",
    ) -> None:
        """Update tracker with new pose estimate, including velocity estimation."""
        with self._lock:
            self._invalidate_prediction_cache_locked()
            if stage_reacquire and self._mode == TrackMode.REACQUIRE and pose.valid:
                self._record_innovation_locked(pose)
                if self._measurement_matches_prediction_locked(pose):
                    self._mode_consecutive_accepts += 1
                    self._mode_consecutive_rejects = 0
                    reason = "reacquire_candidate_consistent"
                else:
                    self._mode_consecutive_accepts = 0
                    self._mode_consecutive_rejects += 1
                    reason = "reacquire_candidate_large_innovation"

                ready = (
                    self._mode_consecutive_accepts
                    >= self.mode_config.reacquire_consecutive_accepts
                )
                if not ready:
                    rejected_pose = RigidBodyPose(
                        timestamp=int(pose.timestamp),
                        position=np.zeros(3, dtype=np.float64),
                        rotation=np.eye(3, dtype=np.float64),
                        quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64),
                        valid=False,
                    )
                    previous_valid = self._pose_history[-1].valid if self._pose_history else False
                    self._pose_history.append(rejected_pose)
                    self.total_frames += 1
                    self.stats.record(
                        rejected_pose,
                        previous_valid=previous_valid,
                        invalid_reason=invalid_reason
                        or "reacquire_pending_confirmation:" + reason,
                    )
                    self.lost_frames += 1
                    if self._mode_consecutive_rejects >= self.mode_config.reacquire_lost_frames:
                        self._transition_mode_locked(
                            TrackMode.LOST,
                            pose.timestamp,
                            "reacquire_timeout",
                        )
                    else:
                        self._last_mode_reason = reason
                        self._mode_frame_count += 1
                    return

            previous_valid = self._pose_history[-1].valid if self._pose_history else False
            if (
                self._mode == TrackMode.REACQUIRE
                and not pose.valid
                and invalid_reason
                and not str(invalid_reason).startswith("reacquire_pending_confirmation")
            ):
                self._mode_consecutive_accepts = 0
            self._update_mode_locked(pose)
            self._pose_history.append(pose)
            self._last_pose_source = str(
                pose_source
                or ("prediction_hold" if prediction_hold else ("measurement" if pose.valid else "invalid"))
            )
            self.total_frames += 1
            self.stats.record(
                pose,
                previous_valid=previous_valid,
                invalid_reason=invalid_reason,
            )

            if pose.valid:
                if prediction_hold:
                    self._prediction_hold_frames += 1
                else:
                    self._prediction_hold_frames = 0
                if self.track_count > 0 and self._last_valid_timestamp > 0:
                    dt_s = (pose.timestamp - self._last_valid_timestamp) / 1_000_000.0
                    # Only update velocity if dt is sane (0–500 ms)
                    if prediction_hold:
                        pass
                    elif 0.0 < dt_s < 0.5:
                        self._velocity = (pose.position - self._position) / dt_s
                        self._angular_velocity = (
                            _quaternion_delta_rotvec(pose.quaternion, self._quaternion)
                            / dt_s
                        )
                    else:
                        self._velocity = np.zeros(3)
                        self._angular_velocity = np.zeros(3)
                self._position = pose.position.copy()
                self._quaternion = pose.quaternion.copy()
                self._last_valid_timestamp = pose.timestamp
                self.track_count += 1
                self.lost_frames = 0
            else:
                self._prediction_hold_frames = 0
                self.lost_frames += 1

    @property
    def prediction_hold_frames(self) -> int:
        with self._lock:
            return int(self._prediction_hold_frames)

    def prepare_reacquire_candidate(self, pose: RigidBodyPose) -> Tuple[bool, str]:
        """Evaluate a reacquire candidate without committing it to the pose state."""
        with self._lock:
            if self._mode != TrackMode.REACQUIRE:
                return True, "not_reacquire"
            if not pose.valid:
                self._mode_consecutive_rejects += 1
                self._mode_consecutive_accepts = 0
                reason = "reacquire_waiting"
            else:
                self._record_innovation_locked(pose)
                if self._measurement_matches_prediction_locked(pose):
                    self._mode_consecutive_accepts += 1
                    self._mode_consecutive_rejects = 0
                    reason = "reacquire_candidate_consistent"
                else:
                    self._mode_consecutive_accepts = 0
                    self._mode_consecutive_rejects += 1
                    reason = "reacquire_candidate_large_innovation"

            ready = bool(
                pose.valid
                and self._mode_consecutive_accepts
                >= self.mode_config.reacquire_consecutive_accepts
            )
            if not ready and self._mode_consecutive_rejects >= self.mode_config.reacquire_lost_frames:
                self._transition_mode_locked(TrackMode.LOST, pose.timestamp, "reacquire_timeout")
                reason = "reacquire_timeout"
            else:
                self._last_mode_reason = reason
                self._mode_frame_count += 1
            return ready, reason

    def _stage_reacquire_candidate(self, pose: RigidBodyPose) -> Tuple[bool, str]:
        """Compatibility wrapper for tests and internal staging terminology."""
        return self.prepare_reacquire_candidate(pose)

    def record_non_committed_frame(
        self,
        pose: RigidBodyPose,
        *,
        invalid_reason: Optional[str] = None,
        pose_source: str = "non_committed",
    ) -> None:
        """Record a frame without letting its pose update prediction state."""
        with self._lock:
            self._invalidate_prediction_cache_locked()
            previous_valid = self._pose_history[-1].valid if self._pose_history else False
            self._pose_history.append(pose)
            self._last_pose_source = str(pose_source or "non_committed")
            self.total_frames += 1
            self.stats.record(
                pose,
                previous_valid=previous_valid,
                invalid_reason=invalid_reason,
            )
            if pose.valid:
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
            current_rotation = Rotation.from_quat([
                self._quaternion[1], self._quaternion[2],
                self._quaternion[3], self._quaternion[0]
            ])
            predicted_rotation_obj = (
                Rotation.from_rotvec(self._angular_velocity * max(0.0, float(dt)))
                * current_rotation
            )
            predicted_rotation = predicted_rotation_obj.as_matrix()
            quat_xyzw = predicted_rotation_obj.as_quat()
            predicted_quaternion = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=np.float64,
            )
            
            return RigidBodyPose(
                timestamp=0,  # Will be set by caller
                position=predicted_position,
                rotation=predicted_rotation,
                quaternion=_normalize_quaternion(predicted_quaternion),
                rms_error=0.0,
                observed_markers=0,
                valid=False  # Mark as prediction
            )

    def peek_prediction(self, timestamp_us: int) -> PredictedPose:
        """Predict pose at timestamp_us without mutating tracker state."""
        with self._lock:
            timestamp_us = int(timestamp_us)
            if (
                self._prediction_cache_timestamp == timestamp_us
                and self._prediction_cache is not None
            ):
                return self._prediction_cache
            if self.track_count == 0 or self._last_valid_timestamp <= 0:
                quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
                prediction = PredictedPose(
                    timestamp=timestamp_us,
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
                self._prediction_cache_timestamp = timestamp_us
                self._prediction_cache = prediction
                return prediction

            dt_s = max(0.0, float(timestamp_us - self._last_valid_timestamp) / 1_000_000.0)
            velocity = self._velocity.copy()
            angular_velocity = self._angular_velocity.copy()
            if self._prediction_hold_frames > 0 and self.pattern.num_markers >= 5:
                # Long occlusion holds should coast, not keep amplifying the
                # last measured dance turn.
                hold_frames = float(self._prediction_hold_frames)
                velocity *= float(max(0.35, 0.97 ** hold_frames))
                angular_velocity *= float(max(0.10, 0.88 ** hold_frames))
            if dt_s > 0.5:
                # Long gaps make constant velocity less trustworthy; do not mutate
                # the stored velocity here because this is a pure peek.
                velocity = np.zeros(3, dtype=np.float64)
                angular_velocity = np.zeros(3, dtype=np.float64)
            position = self._position + velocity * min(dt_s, 0.5)
            current_rotation = Rotation.from_quat([
                self._quaternion[1],
                self._quaternion[2],
                self._quaternion[3],
                self._quaternion[0],
            ])
            rotation_obj = (
                Rotation.from_rotvec(angular_velocity * min(dt_s, 0.5))
                * current_rotation
            )
            quat_xyzw = rotation_obj.as_quat()
            quaternion = _normalize_quaternion(
                np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            )
            confidence = self._rolling_confidence_locked()
            prediction = PredictedPose(
                timestamp=timestamp_us,
                position=position.copy(),
                rotation=rotation_obj.as_matrix(),
                quaternion=quaternion.copy(),
                velocity=velocity.copy(),
                dt_s=float(dt_s),
                valid=True,
                confidence=confidence,
                position_sigma_m=self._position_sigma_m(dt_s, confidence),
                rotation_sigma_deg=self._rotation_sigma_deg(dt_s, confidence),
                lost_frames=int(self.lost_frames),
            )
            self._prediction_cache_timestamp = timestamp_us
            self._prediction_cache = prediction
            return prediction

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

    def latest_object_gating(self) -> Dict[str, Any]:
        """Return the latest object-gating payload without building full diagnostics."""
        with self._lock:
            return dict(self._last_object_gating)

    def latest_object_gating_counts(self) -> Tuple[bool, int, int, int]:
        """Return hot-path object-gating counters without copying diagnostics."""
        with self._lock:
            gating = self._last_object_gating
            return (
                bool(gating.get("evaluated")),
                int(gating.get("marker_count", 0) or 0),
                int(gating.get("camera_count", 0) or 0),
                int(gating.get("assigned_marker_views", 0) or 0),
            )

    def latest_object_gating_skip_counts(self) -> Tuple[int, int, int, int]:
        """Return structural evidence counters used to skip generic search."""
        with self._lock:
            gating = self._last_object_gating
            return (
                int(gating.get("markers_with_two_or_more_rays", 0) or 0),
                int(gating.get("ambiguous_assignment_count", 0) or 0),
                int(gating.get("marker_margin_assignment_count", 0) or 0),
                int(gating.get("duplicate_assignment_count", 0) or 0),
            )

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

    def record_pose_continuity_guard(self, guard: Dict[str, Any]) -> None:
        """Attach latest low-marker temporal guard diagnostics."""
        with self._lock:
            payload = dict(guard or _empty_pose_continuity_guard())
            if payload.get("evaluated"):
                self._pose_continuity_guard_evaluated_count += 1
            if payload.get("would_reject"):
                self._pose_continuity_guard_would_reject_count += 1
            if payload.get("held_prediction") or payload.get("held_rotation"):
                self._pose_continuity_guard_held_count += 1
            payload["evaluated_count"] = int(self._pose_continuity_guard_evaluated_count)
            payload["would_reject_count"] = int(self._pose_continuity_guard_would_reject_count)
            payload["held_count"] = int(self._pose_continuity_guard_held_count)
            self._last_pose_continuity_guard = payload

    def pose_continuity_metrics(self, pose: RigidBodyPose) -> Dict[str, Any]:
        """Return prediction and angular-jump metrics for a candidate pose."""
        with self._lock:
            if self.track_count == 0 or self._last_valid_timestamp <= 0:
                return {"prediction_valid": False}

            dt_s = max(0.0, float(pose.timestamp - self._last_valid_timestamp) / 1_000_000.0)
            velocity = self._velocity.copy() if dt_s <= 0.5 else np.zeros(3, dtype=np.float64)
            angular_velocity = (
                self._angular_velocity.copy() if dt_s <= 0.5 else np.zeros(3, dtype=np.float64)
            )
            predicted_position = self._position + velocity * min(dt_s, 0.5)
            current_rotation = Rotation.from_quat([
                self._quaternion[1],
                self._quaternion[2],
                self._quaternion[3],
                self._quaternion[0],
            ])
            predicted_rotation = (
                Rotation.from_rotvec(angular_velocity * min(dt_s, 0.5))
                * current_rotation
            )
            quat_xyzw = predicted_rotation.as_quat()
            prediction_quaternion = _normalize_quaternion(
                np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
            )
            position_innovation_m = float(np.linalg.norm(pose.position - predicted_position))
            rotation_innovation_deg = _quaternion_angle_deg(prediction_quaternion, pose.quaternion)

            valid_history = [item for item in self._pose_history if item.valid]
            angular_velocity_deg_s = 0.0
            previous_angular_velocity_deg_s = 0.0
            angular_accel_deg_s2 = 0.0
            if dt_s > 1e-6:
                angular_velocity_deg_s = rotation_innovation_deg / dt_s
            if len(valid_history) >= 2:
                prev = valid_history[-1]
                prev_prev = valid_history[-2]
                prev_dt_s = max(0.0, float(prev.timestamp - prev_prev.timestamp) / 1_000_000.0)
                if prev_dt_s > 1e-6:
                    previous_angular_velocity_deg_s = (
                        _quaternion_angle_deg(prev_prev.quaternion, prev.quaternion) / prev_dt_s
                    )
                if dt_s > 1e-6:
                    angular_accel_deg_s2 = abs(
                        angular_velocity_deg_s - previous_angular_velocity_deg_s
                    ) / dt_s

            return {
                "prediction_valid": True,
                "prediction": {
                    "timestamp": int(pose.timestamp),
                    "position": predicted_position.tolist(),
                    "quaternion": prediction_quaternion.tolist(),
                    "velocity": velocity.tolist(),
                    "dt_s": float(dt_s),
                },
                "position_innovation_m": position_innovation_m,
                "rotation_innovation_deg": rotation_innovation_deg,
                "angular_velocity_deg_s": float(angular_velocity_deg_s),
                "previous_angular_velocity_deg_s": float(previous_angular_velocity_deg_s),
                "angular_accel_deg_s2": float(angular_accel_deg_s2),
            }

    def record_position_continuity_guard(self, guard: Dict[str, Any]) -> None:
        """Attach latest low-marker position acceleration guard diagnostics."""
        with self._lock:
            payload = dict(guard or _empty_position_continuity_guard())
            if payload.get("evaluated"):
                self._position_continuity_guard_evaluated_count += 1
            if payload.get("would_reject"):
                self._position_continuity_guard_would_reject_count += 1
            if payload.get("clamped_position"):
                self._position_continuity_guard_clamped_count += 1
            payload["evaluated_count"] = int(self._position_continuity_guard_evaluated_count)
            payload["would_reject_count"] = int(
                self._position_continuity_guard_would_reject_count
            )
            payload["clamped_count"] = int(self._position_continuity_guard_clamped_count)
            self._last_position_continuity_guard = payload

    def position_continuity_metrics(self, pose: RigidBodyPose) -> Dict[str, Any]:
        """Return position velocity and acceleration metrics for a candidate pose."""
        with self._lock:
            if self.track_count == 0 or self._last_valid_timestamp <= 0:
                return {"prediction_valid": False}

            dt_s = max(0.0, float(pose.timestamp - self._last_valid_timestamp) / 1_000_000.0)
            if dt_s <= 1e-6 or dt_s > 0.5:
                return {"prediction_valid": False, "dt_s": float(dt_s)}

            previous_position = self._position.copy()
            previous_velocity = self._velocity.copy()
            candidate_velocity = (pose.position - previous_position) / dt_s
            delta_velocity = candidate_velocity - previous_velocity
            position_accel_m_s2 = float(np.linalg.norm(delta_velocity) / dt_s)
            position_velocity_m_s = float(np.linalg.norm(candidate_velocity))
            previous_velocity_m_s = float(np.linalg.norm(previous_velocity))
            predicted_position = previous_position + previous_velocity * dt_s
            position_innovation_m = float(np.linalg.norm(pose.position - predicted_position))

            return {
                "prediction_valid": True,
                "dt_s": float(dt_s),
                "previous_position": previous_position.tolist(),
                "previous_velocity": previous_velocity.tolist(),
                "candidate_velocity": candidate_velocity.tolist(),
                "position_innovation_m": position_innovation_m,
                "position_velocity_m_s": position_velocity_m_s,
                "previous_velocity_m_s": previous_velocity_m_s,
                "position_accel_m_s2": position_accel_m_s2,
            }

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
            "object_gating": compact_object_gating_diagnostics(
                self._last_object_gating,
                include_detail=(
                    self._mode != TrackMode.CONTINUE
                    or int(self.total_frames) % 12 == 0
                    or int(self._last_object_gating.get("ambiguous_assignment_count", 0) or 0) > 0
                    or int(self._last_object_gating.get("duplicate_assignment_count", 0) or 0) > 0
                ),
            ),
            "last_pose_source": str(self._last_pose_source),
            "prediction_hold_frames": int(self._prediction_hold_frames),
            "rigid_hint_pose": dict(self._last_rigid_hint_pose),
            "subset_hypothesis": dict(self._last_subset_hypothesis),
            "reacquire_guard": dict(self._last_reacquire_guard),
            "pose_continuity_guard": dict(self._last_pose_continuity_guard),
            "position_continuity_guard": dict(self._last_position_continuity_guard),
            "rms_error_m": latest_rms,
            "observed_markers": latest_observed,
            "real_ray_count": latest_observed,
            "virtual_marker_count": 0,
            **self.stats.snapshot(),
        }

    def get_event_diagnostics(self) -> Dict[str, Any]:
        """Return the small diagnostic subset used by per-frame event extraction."""
        latest = self._pose_history[-1] if self._pose_history else None
        latest_valid = bool(latest.valid) if latest is not None else False
        return {
            "is_tracking": self.is_tracking,
            "valid": latest_valid,
            "mode": self._mode.value,
            "reacquire_guard": self._last_reacquire_guard,
            "pose_continuity_guard": self._last_pose_continuity_guard,
            "position_continuity_guard": self._last_position_continuity_guard,
            "rigid_hint_pose": self._last_rigid_hint_pose,
            "subset_hypothesis": self._last_subset_hypothesis,
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
            if self._mode != TrackMode.REACQUIRE:
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
        angular_velocity = (
            self._angular_velocity if dt_s <= 0.5 else np.zeros(3, dtype=np.float64)
        )
        predicted_position = self._position + velocity * min(dt_s, 0.5)
        current_rotation = Rotation.from_quat([
            self._quaternion[1],
            self._quaternion[2],
            self._quaternion[3],
            self._quaternion[0],
        ])
        predicted_rotation = (
            Rotation.from_rotvec(angular_velocity * min(dt_s, 0.5))
            * current_rotation
        )
        quat_xyzw = predicted_rotation.as_quat()
        predicted_quaternion = _normalize_quaternion(
            np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
        )
        self._last_position_innovation_m = float(np.linalg.norm(pose.position - predicted_position))
        self._max_position_innovation_m = max(
            self._max_position_innovation_m,
            self._last_position_innovation_m,
        )
        self._last_rotation_innovation_deg = _quaternion_angle_deg(
            predicted_quaternion,
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
        dt_s = max(0.0, float(pose.timestamp - self._last_valid_timestamp) / 1_000_000.0)
        occlusion_position_slack_m = min(
            0.90,
            0.035 * float(self.lost_frames) + 0.12 * dt_s,
        )
        occlusion_rotation_slack_deg = min(
            60.0,
            4.0 * float(self.lost_frames) + 12.0 * dt_s,
        )
        position_gate_m = (
            self.mode_config.continue_position_gate_m + occlusion_position_slack_m
        )
        rotation_gate_deg = min(
            180.0,
            self.mode_config.continue_rotation_gate_deg + occlusion_rotation_slack_deg,
        )
        return (
            self._last_position_innovation_m <= position_gate_m
            and self._last_rotation_innovation_deg <= rotation_gate_deg
        )


@dataclass
class _PoseCandidate:
    rigid_name: str
    pattern: MarkerPattern
    tracker: RigidBodyTracker
    pose: RigidBodyPose
    score: Dict[str, Any]
    source: str
    hint_diagnostic: Dict[str, Any]
    subset_diagnostic: Dict[str, Any]
    position_guard: Dict[str, Any]
    continuity_guard: Dict[str, Any]
    reacquire_guard: Dict[str, Any]
    rank_score: float
    best_cluster_idx: int = -1
    reject_reasons: List[str] = field(default_factory=list)
    invalid_reason: str = ""


@dataclass(frozen=True)
class _PreparedCameraObservations:
    camera_id: str
    camera: Any
    uv: np.ndarray
    blob_indices: np.ndarray
    area_px2: np.ndarray
    diameter_px: np.ndarray
    uncertainty_px: np.ndarray

    @property
    def blob_count(self) -> int:
        return int(len(self.uv))


@dataclass(frozen=True)
class _PreparedObservationContext:
    coordinate_space: str
    by_camera: Dict[str, _PreparedCameraObservations]
    camera_count: int
    total_blob_count: int


class RigidBodyEstimator:
    """
    Complete rigid body estimation pipeline.
    
    Integrates clustering, pose estimation, and tracking.
    """

    _MULTI_PATTERN_BOOT_FULL_POINT_LIMIT = 40
    _MULTI_PATTERN_BOOT_TOP_CANDIDATES = 8
    _PARTIAL_OCCLUSION_HOLD_MAX_FRAMES = 90
    _UNSEEN_OCCLUSION_HOLD_MAX_FRAMES = 24
    _SINGLE_ANCHOR_HOLD_MAX_FRAMES = 6
    _MULTI_PATTERN_BOOT_MAX_MEAN_REPROJECTION_PX = 3.0
    _MULTI_PATTERN_BOOT_MAX_P95_REPROJECTION_PX = 4.0
    _MULTI_PATTERN_BOOT_MAX_MISSING_MARKER_VIEWS = 2
    
    def __init__(
        self,
        patterns: Optional[List[MarkerPattern]] = None,
        marker_diameter: float = 0.014,
        cluster_radius_m: float = 0.08,
        max_rms_error_m: float = 0.055,
        reprojection_match_gate_px: float = 12.0,
        reacquire_guard_config: ReacquireGuardConfig = ReacquireGuardConfig(),
        object_gating_config: ObjectGatingConfig = ObjectGatingConfig(),
        pose_continuity_guard_config: PoseContinuityGuardConfig = PoseContinuityGuardConfig(),
        position_continuity_guard_config: PositionContinuityGuardConfig = (
            PositionContinuityGuardConfig()
        ),
        subset_solve_config: SubsetSolveConfig = SubsetSolveConfig(),
        subset_diagnostics_mode: str = "full",
        subset_time_budget_ms: Optional[float] = None,
        subset_max_hypotheses: Optional[int] = None,
        rigid_candidate_separation_enabled: bool = False,
        temporal_body_nbest_enabled: bool = False,
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
        self.pose_continuity_guard_config = pose_continuity_guard_config
        self.position_continuity_guard_config = position_continuity_guard_config
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
        self.temporal_body_nbest_enabled = bool(temporal_body_nbest_enabled)
        self._stage_callback = stage_callback
        self._variant_metric_lock = threading.Lock()
        self._subset_frame_index = 0
        self._subset_sampled_count = 0
        self._subset_skipped_count = 0
        self._subset_budget_exceeded_count = 0
        self._rigid_candidate_separated_count = 0
        self._rigid_candidate_fallback_count = 0
        self._rigid_candidate_fallback_reason_counts: Counter[str] = Counter()
        self._mode_counts_by_rigid: Dict[str, Counter[str]] = {
            p.name: Counter() for p in self.patterns
        }
        self._candidate_counts_by_rigid_mode: Dict[str, Counter[str]] = {
            p.name: Counter() for p in self.patterns
        }
        self._candidate_rejection_reasons_by_mode: Dict[str, Counter[str]] = {
            mode.value: Counter() for mode in TrackMode
        }
        self._body_conflict_count = 0
        self._body_conflict_resolution_reasons: Counter[str] = Counter()
        self._cluster_call_count = 0
        self._cluster_input_point_total = 0
        self._cluster_output_count_total = 0
        self._cluster_output_count_max = 0
        self._estimate_pose_call_count = 0
        self._estimate_pose_point_total = 0
        self._body_level_2d_candidate_cache: Dict[Tuple[str, str], Dict[str, Any]] = {}
        self._body_level_2d_cache_query_count = 0
        self._body_level_2d_cache_hit_count = 0
        self._body_level_2d_cache_early_exit_count = 0
        self._body_level_2d_cache_bin_total = 0
        self._rigid_hint_marker_count_by_rigid: Dict[str, Counter[int]] = {
            p.name: Counter() for p in self.patterns
        }
        self._rigid_hint_view_count_by_rigid: Dict[str, Counter[int]] = {
            p.name: Counter() for p in self.patterns
        }
        self._last_rigid_hint_visibility_by_rigid: Dict[str, Dict[str, Any]] = {
            p.name: {
                "marker_count": 0,
                "marker_indices": [],
                "view_count": 0,
            }
            for p in self.patterns
        }
        self._last_confirmed_marker_views_by_rigid: Dict[
            str,
            Dict[int, Dict[str, Dict[str, Any]]],
        ] = {}
        self._last_confirmed_marker_points_by_rigid: Dict[
            str,
            Dict[int, List[Dict[str, Any]]],
        ] = {}
        self._subset_reference_cache: Dict[Tuple[str, int, Tuple[int, ...]], np.ndarray] = {}
        self._subset_ambiguity_cache: Dict[Tuple[str, int, Tuple[int, ...]], bool] = {}
        self.clusterer = PointClusterer(
            marker_diameter=marker_diameter,
            cluster_radius_m=self.cluster_radius_m,
        )
        
        # Create trackers for each pattern
        self.trackers: Dict[str, RigidBodyTracker] = {
            p.name: RigidBodyTracker(p) for p in self.patterns
        }

    def get_variant_metrics(self) -> Dict[str, Any]:
        with self._variant_metric_lock:
            metrics = {
                "subset_sampled_count": int(self._subset_sampled_count),
                "subset_skipped_count": int(self._subset_skipped_count),
                "subset_budget_exceeded_count": int(self._subset_budget_exceeded_count),
                "subset_diagnostics_mode": self.subset_diagnostics_mode,
                "subset_time_budget_ms": self.subset_time_budget_ms,
                "subset_max_hypotheses": self.subset_max_hypotheses,
                "rigid_candidate_separation_enabled": self.rigid_candidate_separation_enabled,
                "temporal_body_nbest_enabled": self.temporal_body_nbest_enabled,
                "rigid_candidate_separated_count": int(self._rigid_candidate_separated_count),
                "rigid_candidate_fallback_count": int(self._rigid_candidate_fallback_count),
                "rigid_candidate_fallback_reason_counts": dict(
                    self._rigid_candidate_fallback_reason_counts
                ),
                "cluster_radius_m": float(self.cluster_radius_m),
                "cluster_call_count": int(self._cluster_call_count),
                "cluster_input_point_total": int(self._cluster_input_point_total),
                "cluster_output_count_total": int(self._cluster_output_count_total),
                "cluster_output_count_max": int(self._cluster_output_count_max),
                "cluster_output_count_mean": (
                    float(self._cluster_output_count_total)
                    / float(self._cluster_call_count)
                    if self._cluster_call_count > 0
                    else 0.0
                ),
                "estimate_pose_call_count": int(self._estimate_pose_call_count),
                "estimate_pose_point_total": int(self._estimate_pose_point_total),
                "estimate_pose_points_mean": (
                    float(self._estimate_pose_point_total)
                    / float(self._estimate_pose_call_count)
                    if self._estimate_pose_call_count > 0
                    else 0.0
                ),
                "body_level_2d_cache_query_count": int(
                    self._body_level_2d_cache_query_count
                ),
                "body_level_2d_cache_hit_count": int(
                    self._body_level_2d_cache_hit_count
                ),
                "body_level_2d_cache_early_exit_count": int(
                    self._body_level_2d_cache_early_exit_count
                ),
                "body_level_2d_cache_bin_total": int(
                    self._body_level_2d_cache_bin_total
                ),
            }
        metrics["passive_stabilization_v2"] = self.get_passive_stabilization_metrics()
        return metrics

    def get_passive_stabilization_metrics(self) -> Dict[str, Any]:
        """Return diagnostics for passive 4-camera / 5-marker V2 PDCA entry."""
        with self._variant_metric_lock:
            candidate_counts = {
                rigid_name: dict(counter)
                for rigid_name, counter in sorted(
                    self._candidate_counts_by_rigid_mode.items()
                )
            }
            mode_counts = {
                rigid_name: dict(counter)
                for rigid_name, counter in sorted(self._mode_counts_by_rigid.items())
            }
            rejection_by_mode = {
                mode: dict(counter)
                for mode, counter in sorted(
                    self._candidate_rejection_reasons_by_mode.items()
                )
            }
            return {
                "mode_counts_by_rigid": mode_counts,
                "candidate_counts_by_rigid_mode": candidate_counts,
                "boot_candidate_count_by_rigid": {
                    rigid_name: int(counts.get("boot", 0))
                    for rigid_name, counts in candidate_counts.items()
                },
                "continue_candidate_count_by_rigid": {
                    rigid_name: int(counts.get("continue", 0))
                    for rigid_name, counts in candidate_counts.items()
                },
                "reacquire_candidate_count_by_rigid": {
                    rigid_name: int(counts.get("reacquire", 0))
                    for rigid_name, counts in candidate_counts.items()
                },
                "hold_prediction_count_by_rigid": {
                    rigid_name: int(counts.get("prediction_hold", 0))
                    for rigid_name, counts in candidate_counts.items()
                },
                "candidate_rejection_reasons_by_mode": rejection_by_mode,
                "body_conflict_count": int(self._body_conflict_count),
                "body_conflict_resolution_reasons": dict(
                    self._body_conflict_resolution_reasons
                ),
                "rigid_hint_marker_count_histogram_by_rigid": {
                    rigid_name: {
                        str(count): int(value)
                        for count, value in sorted(counter.items())
                    }
                    for rigid_name, counter in sorted(
                        self._rigid_hint_marker_count_by_rigid.items()
                    )
                },
                "rigid_hint_view_count_histogram_by_rigid": {
                    rigid_name: {
                        str(count): int(value)
                        for count, value in sorted(counter.items())
                    }
                    for rigid_name, counter in sorted(
                        self._rigid_hint_view_count_by_rigid.items()
                    )
                },
            }

    def get_last_rigid_hint_visibility(self) -> Dict[str, Dict[str, Any]]:
        """Return the latest marker-indexed 3D hint visibility per rigid."""
        with self._variant_metric_lock:
            return {
                rigid_name: dict(payload)
                for rigid_name, payload in self._last_rigid_hint_visibility_by_rigid.items()
            }

    @staticmethod
    def _pattern_subset_cache_key(
        pattern: MarkerPattern,
        marker_indices: Tuple[int, ...],
    ) -> Tuple[str, int, Tuple[int, ...]]:
        return (
            str(pattern.name),
            int(pattern.num_markers),
            tuple(int(index) for index in marker_indices),
        )

    def _reference_points_for_indices(
        self,
        pattern: MarkerPattern,
        marker_indices: Tuple[int, ...],
    ) -> np.ndarray:
        key = self._pattern_subset_cache_key(pattern, marker_indices)
        cached = self._subset_reference_cache.get(key)
        if cached is None:
            cached = np.asarray(
                [pattern.marker_positions[int(index)] for index in marker_indices],
                dtype=np.float64,
            )
            self._subset_reference_cache[key] = cached
        return cached

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

    def _should_skip_generic_search_for_object_gated_continue(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        *,
        object_gating_enforced: bool,
        hint_marker_count: int,
        gating_evaluated: bool,
        gating_camera_count: int,
        gating_assigned_marker_views: int,
        gating_markers_with_two_or_more_rays: int,
        gating_ambiguous_assignment_count: int,
        gating_marker_margin_assignment_count: int,
        gating_duplicate_assignment_count: int,
    ) -> bool:
        """Return true only when body-level 2D evidence makes generic search redundant."""
        marker_count = int(pattern.num_markers)
        if (
            not bool(self.object_gating_config.skip_generic_search_when_object_gated)
            or not bool(object_gating_enforced)
            or tracker.mode != TrackMode.CONTINUE
            or marker_count < 5
            or int(tracker.track_count) <= 0
            or not bool(gating_evaluated)
            or int(gating_camera_count) < 2
            or int(hint_marker_count) < max(3, marker_count - 2)
        ):
            return False
        if int(gating_assigned_marker_views) < marker_count * 2:
            return False
        if int(gating_markers_with_two_or_more_rays) < marker_count:
            return False
        if int(gating_ambiguous_assignment_count) > 0:
            return False
        if int(gating_marker_margin_assignment_count) > 0:
            return False
        if int(gating_duplicate_assignment_count) > 0:
            return False
        return True

    def _record_mode_frame(self, rigid_name: str, mode: TrackMode) -> None:
        with self._variant_metric_lock:
            self._mode_counts_by_rigid.setdefault(str(rigid_name), Counter())[
                mode.value
            ] += 1

    def _record_pose_candidate(self, candidate: _PoseCandidate) -> None:
        mode = candidate.tracker.mode.value
        source = str(candidate.source or "unknown")
        with self._variant_metric_lock:
            counter = self._candidate_counts_by_rigid_mode.setdefault(
                candidate.rigid_name,
                Counter(),
            )
            counter[mode] += 1
            if source in {"prediction_hold", "prediction_hold_2d_constrained"}:
                counter["prediction_hold"] += 1
        self._record_candidate_rejection(candidate, candidate.invalid_reason)

    def _record_candidate_rejection(
        self,
        candidate: _PoseCandidate,
        reason: str | None,
    ) -> None:
        reason_text = str(reason or "").strip()
        if not reason_text:
            for reject_reason in candidate.reject_reasons:
                reason_text = str(reject_reason or "").strip()
                if reason_text:
                    break
        if not reason_text:
            return
        mode = candidate.tracker.mode.value
        reason_key = reason_text.split(":", 1)[0]
        with self._variant_metric_lock:
            self._candidate_rejection_reasons_by_mode.setdefault(
                mode,
                Counter(),
            )[reason_key] += 1

    def _record_rigid_hint_visibility(
        self,
        hint_markers_by_rigid: Dict[str, Dict[int, Dict[str, Any]]],
    ) -> None:
        with self._variant_metric_lock:
            latest: Dict[str, Dict[str, Any]] = {}
            for pattern in self.patterns:
                markers = hint_markers_by_rigid.get(pattern.name, {})
                marker_indices = [int(index) for index in sorted(markers)]
                marker_count = int(len(marker_indices))
                view_count = int(
                    sum(
                        max(1, int(payload.get("contributing_rays", 0) or 0))
                        for payload in markers.values()
                    )
                )
                self._rigid_hint_marker_count_by_rigid.setdefault(
                    pattern.name,
                    Counter(),
                )[marker_count] += 1
                self._rigid_hint_view_count_by_rigid.setdefault(
                    pattern.name,
                    Counter(),
                )[view_count] += 1
                latest[pattern.name] = {
                    "marker_count": marker_count,
                    "marker_indices": marker_indices,
                    "view_count": view_count,
                }
            self._last_rigid_hint_visibility_by_rigid = latest

    def _should_hold_unseen_tracked_rigid(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        timestamp: int,
        hint_markers_by_rigid: Dict[str, Dict[int, Dict[str, Any]]],
    ) -> bool:
        """Keep an established rigid on prediction when only other rigids are observed."""
        if len(self.patterns) <= 1:
            return False
        if int(tracker.track_count) <= 0:
            return False
        if tracker.mode not in {TrackMode.CONTINUE, TrackMode.REACQUIRE, TrackMode.LOST}:
            return False
        if hint_markers_by_rigid.get(pattern.name):
            return False
        if not tracker.peek_prediction(timestamp).valid:
            return False
        gating = tracker.latest_object_gating()
        evidence_markers = int(gating.get("markers_with_two_or_more_rays", 0)) + int(
            gating.get("single_ray_candidates", 0)
        )
        active_anchor_evidence = self._active_anchor_observation_count(gating)
        has_current_evidence = (
            int(gating.get("assigned_marker_views", 0)) > 0
            or evidence_markers > 0
            or active_anchor_evidence > 0
        )
        if (
            int(pattern.num_markers) >= 5
            and
            active_anchor_evidence > 0
            and evidence_markers < 2
            and int(tracker.prediction_hold_frames) >= self._SINGLE_ANCHOR_HOLD_MAX_FRAMES
        ):
            return False
        max_hold_frames = (
            self._PARTIAL_OCCLUSION_HOLD_MAX_FRAMES
            if has_current_evidence
            else self._UNSEEN_OCCLUSION_HOLD_MAX_FRAMES
        )
        if int(tracker.prediction_hold_frames) >= max_hold_frames:
            return False
        for other in self.patterns:
            if other.name == pattern.name:
                continue
            required = max(3, int(other.num_markers) - 1)
            if len(hint_markers_by_rigid.get(other.name, {})) >= required:
                return True
        return False

    def _should_hold_partial_marker_tracked_rigid(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        timestamp: int,
        hint_markers_by_rigid: Dict[str, Dict[int, Dict[str, Any]]],
    ) -> bool:
        """Keep a tracked rigid on prediction when too few of its own markers are visible."""
        if not self.object_gating_config.enabled or not self.object_gating_config.enforce:
            return False
        if int(tracker.track_count) <= 0 or tracker.mode != TrackMode.CONTINUE:
            return False
        if int(tracker.prediction_hold_frames) >= self._PARTIAL_OCCLUSION_HOLD_MAX_FRAMES:
            return False
        visible_markers = len(hint_markers_by_rigid.get(pattern.name, {}))
        gating = tracker.latest_object_gating()
        active_anchor_evidence = self._active_anchor_observation_count(gating)
        evidence_markers = int(gating.get("markers_with_two_or_more_rays", 0)) + int(
            gating.get("single_ray_candidates", 0)
        )
        if (
            int(pattern.num_markers) >= 5
            and
            active_anchor_evidence > 0
            and evidence_markers < 2
            and int(tracker.prediction_hold_frames) >= self._SINGLE_ANCHOR_HOLD_MAX_FRAMES
        ):
            return False
        can_extend_hold_from_physical_id = (
            int(tracker.prediction_hold_frames) > 0 and active_anchor_evidence > 0
        )
        required_markers = max(3, int(pattern.num_markers) - 1)
        min_visible_markers = 1 if int(tracker.prediction_hold_frames) > 0 else 2
        if (
            visible_markers < min_visible_markers
            and not can_extend_hold_from_physical_id
        ) or visible_markers >= required_markers:
            return False
        return bool(tracker.peek_prediction(timestamp).valid)

    @staticmethod
    def _active_anchor_observation_count(gating: Dict[str, Any]) -> int:
        if not isinstance(gating, dict):
            return 0
        observations = gating.get("active_anchor_observations", [])
        if isinstance(observations, list) and observations:
            cameras: set[str] = set()
            for item in observations:
                if not isinstance(item, dict):
                    continue
                camera_id = str(item.get("camera_id", ""))
                if camera_id:
                    cameras.add(camera_id)
            return int(len(cameras))
        return int(gating.get("active_anchor_assignment_count", 0) or 0)

    def _rigid_hint_candidate_marker_points(
        self,
        pattern: MarkerPattern,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        hint_markers: Optional[Dict[int, Dict[str, Any]]] = None,
    ) -> Optional[Tuple[List[int], np.ndarray]]:
        if hint_markers is None:
            hint_markers = self._rigid_hint_markers_by_index(
                pattern,
                rigid_hint_triangulated_points,
            )
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
        tracker: RigidBodyTracker,
        timestamp: int,
        points_3d: np.ndarray,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        hint_markers: Optional[Dict[int, Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Optional[Tuple[RigidBodyPose, Dict[str, Any]]]:
        marker_points = self._rigid_hint_candidate_marker_points(
            pattern,
            rigid_hint_triangulated_points,
            hint_markers=hint_markers,
        )
        if marker_points is None:
            temporal_nbest = self._try_temporal_body_hint_subset_candidate(
                pattern,
                tracker,
                timestamp,
                rigid_hint_triangulated_points,
                hint_markers,
                camera_params,
                observations_by_camera,
                coordinate_space=coordinate_space,
            )
            if temporal_nbest is not None:
                return temporal_nbest
            self._record_rigid_candidate_fallback("insufficient_rigid_hint_points")
            return None
        marker_indices, observed = marker_points
        reference = self._reference_points_for_indices(
            pattern,
            tuple(int(index) for index in marker_indices),
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
        score = self._score_full_continue_hint_quality(
            pose,
            pattern,
            tracker,
            rigid_hint_triangulated_points,
            observations_by_camera,
            coordinate_space=coordinate_space,
        )
        if not score.get("scored", False):
            score = self._score_pose_reprojection_from_hint_observations(
                pose,
                pattern,
                rigid_hint_triangulated_points,
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
        score = dict(score)
        score["rigid_hint_marker_indices"] = [int(index) for index in marker_indices]
        score["rigid_hint_marker_points_3d"] = [
            {
                "marker_idx": int(index),
                "point": [float(value) for value in np.asarray(point, dtype=np.float64).reshape(3)],
            }
            for index, point in zip(marker_indices, observed)
        ]
        score["rigid_hint_real_ray_count"] = int(score.get("matched_marker_views", 0))
        allowed, subset_reason = self._mode_subset_allowed(
            pattern,
            tracker.mode,
            score,
            prediction_valid=tracker.peek_prediction(timestamp).valid,
        )
        if not allowed:
            self._record_rigid_candidate_fallback(f"rigid_candidate_subset_gate:{subset_reason}")
            return None
        self._record_rigid_candidate_separated()
        return pose, score

    def _try_temporal_body_hint_subset_candidate(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        timestamp: int,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        hint_markers: Optional[Dict[int, Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Optional[Tuple[RigidBodyPose, Dict[str, Any]]]:
        """Recover a continuing rigid from passive body-level marker assignment N-best."""
        if (
            not self.temporal_body_nbest_enabled
            or
            len(self.patterns) <= 1
            or int(pattern.num_markers) < 5
            or tracker.mode != TrackMode.CONTINUE
            or int(tracker.track_count) <= 0
        ):
            return None
        if not hint_markers or len(hint_markers) < 3:
            return None
        prediction = tracker.peek_prediction(timestamp)
        if not prediction.valid:
            return None
        prediction_pose = RigidBodyPose(
            timestamp=int(timestamp),
            position=prediction.position.copy(),
            rotation=prediction.rotation.copy(),
            quaternion=prediction.quaternion.copy(),
            rms_error=0.0,
            observed_markers=0,
            valid=True,
        )
        predicted_markers = (
            prediction.rotation @ pattern.marker_positions.T
        ).T + prediction.position.reshape(1, 3)
        observed_items: List[Dict[str, Any]] = []
        for assigned_marker_idx in sorted(hint_markers):
            payload = hint_markers[int(assigned_marker_idx)]
            point = np.asarray(payload["point"], dtype=np.float64).reshape(3)
            if not np.isfinite(point).all():
                continue
            distances = np.linalg.norm(
                predicted_markers - point.reshape(1, 3),
                axis=1,
            )
            ranked = [int(index) for index in np.argsort(distances)[:2]]
            if int(assigned_marker_idx) not in ranked:
                ranked.append(int(assigned_marker_idx))
            observed_items.append(
                {
                    "assigned_marker_idx": int(assigned_marker_idx),
                    "point": point,
                    "weight": float(payload.get("weight", 1.0) or 1.0),
                    "candidate_marker_indices": ranked[:3],
                }
            )
        if len(observed_items) < 3:
            return None
        candidates: List[Dict[str, Any]] = []
        subset_sizes = [3]
        for observed_subset_indices in (
            combo
            for subset_size in subset_sizes
            for combo in combinations(range(len(observed_items)), subset_size)
        ):
            selected_items = [observed_items[index] for index in observed_subset_indices]
            marker_options = [
                list(item["candidate_marker_indices"]) for item in selected_items
            ]
            for marker_assignment in self._small_unique_index_products(marker_options):
                if len(marker_assignment) != len(selected_items):
                    continue
                marker_indices = tuple(int(index) for index in marker_assignment)
                observed = np.asarray(
                    [item["point"] for item in selected_items],
                    dtype=np.float64,
                )
                weights = np.asarray(
                    [float(item["weight"]) for item in selected_items],
                    dtype=np.float64,
                )
                candidate = self._build_subset_candidate(
                    pattern,
                    timestamp,
                    source="temporal_body_nbest",
                    observed=observed,
                    marker_indices=marker_indices,
                    weights=weights,
                    camera_params=camera_params,
                    observations_by_camera=observations_by_camera,
                    coordinate_space=coordinate_space,
                    generic_pose=prediction_pose,
                    generic_score=_empty_reprojection_score("prediction_prior"),
                    observed_indices=tuple(
                        int(item["assigned_marker_idx"]) for item in selected_items
                    ),
                )
                if candidate.get("rankable"):
                    candidates.append(candidate)
        # Keep the older exact-index subset as a tie-breaker path when N-best is
        # enabled: it is usually the cheapest correct hypothesis, but no longer
        # the only one.
        for marker_subset in combinations(sorted(hint_markers), 3):
            observed = np.asarray(
                [hint_markers[index]["point"] for index in marker_subset],
                dtype=np.float64,
            )
            weights = np.asarray(
                [hint_markers[index]["weight"] for index in marker_subset],
                dtype=np.float64,
            )
            candidate = self._build_subset_candidate(
                pattern,
                timestamp,
                source="rigid_hint_subset",
                observed=observed,
                marker_indices=tuple(int(index) for index in marker_subset),
                weights=weights,
                camera_params=camera_params,
                observations_by_camera=observations_by_camera,
                coordinate_space=coordinate_space,
                generic_pose=prediction_pose,
                generic_score=_empty_reprojection_score("prediction_prior"),
            )
            if candidate.get("rankable"):
                candidates.append(candidate)
        if not candidates:
            self._record_rigid_candidate_fallback("temporal_body_nbest_not_rankable")
            return None
        candidates.sort(
            key=lambda item: (
                float(item.get("combined_score", 0.0)),
                float(item.get("score", 0.0)),
                int(item.get("matched_marker_views", 0)),
                -float(item.get("p95_error_px", 0.0)),
                -float(item.get("rms_error_m", 0.0)),
            ),
            reverse=True,
        )
        best = candidates[0] if candidates else {}
        second = candidates[1] if len(candidates) > 1 else {}
        if not best or not best.get("valid") or not best.get("rankable"):
            self._record_rigid_candidate_fallback("temporal_body_nbest_not_rankable")
            return None
        pose = self._pose_from_payload(best.get("pose"), fallback_timestamp=timestamp)
        if pose is None or not pose.valid:
            self._record_rigid_candidate_fallback("temporal_body_nbest_invalid_pose")
            return None
        score = dict(best.get("score_detail") or _empty_reprojection_score("temporal_body_nbest"))
        score["rigid_hint_marker_indices"] = [
            int(index) for index in best.get("marker_indices", [])
        ]
        score["rigid_hint_real_ray_count"] = int(score.get("matched_marker_views", 0))
        score["temporal_body_nbest"] = True
        score["temporal_body_nbest_candidates"] = int(len(candidates))
        score["temporal_body_nbest_combined_score"] = float(
            best.get("combined_score", 0.0)
        )
        score["temporal_body_nbest_margin"] = float(
            float(best.get("combined_score", 0.0))
            - float(second.get("combined_score", 0.0) if second else 0.0)
        )
        score["temporal_body_nbest_rotation_delta_deg"] = float(
            best.get("rotation_delta_deg", 0.0)
        )
        score["temporal_body_nbest_temporal_penalty"] = float(
            best.get("temporal_penalty", 0.0)
        )
        allowed, subset_reason = self._mode_subset_allowed(
            pattern,
            tracker.mode,
            score,
            prediction_valid=True,
        )
        if not allowed:
            self._record_rigid_candidate_fallback(
                f"temporal_body_subset_gate:{subset_reason}"
            )
            return None
        self._record_rigid_candidate_separated()
        return pose, score

    @staticmethod
    def _small_unique_index_products(options: List[List[int]]) -> List[Tuple[int, ...]]:
        products: List[Tuple[int, ...]] = [tuple()]
        for values in options:
            next_products: List[Tuple[int, ...]] = []
            for prefix in products:
                used = set(prefix)
                for value in values:
                    ivalue = int(value)
                    if ivalue in used:
                        continue
                    next_products.append(prefix + (ivalue,))
            products = next_products
            if not products:
                break
        return products

    def _try_multi_pattern_boot_candidate(
        self,
        pattern: MarkerPattern,
        points_3d: np.ndarray,
        timestamp: int,
        camera_params: Optional[Dict[str, Any]] = None,
        observations_by_camera: Optional[Dict[str, List[Any]]] = None,
        *,
        coordinate_space: str = "raw_pixel",
    ) -> Optional[RigidBodyPose]:
        """Bootstrap one rigid from a small mixed point cloud when clustering splits markers."""
        if len(self.patterns) <= 1:
            return None
        point_count = int(len(points_3d))
        if (
            point_count < pattern.num_markers
            or point_count > self._MULTI_PATTERN_BOOT_FULL_POINT_LIMIT
        ):
            return None
        reference_signature = self._distance_signature(pattern.marker_positions)
        distance_matrix = cdist(points_3d, points_3d)
        edge_indices = tuple(combinations(range(pattern.num_markers), 2))
        candidate_combos = self._multi_pattern_boot_combos(
            distance_matrix,
            pattern,
            reference_signature,
        )
        ranked_combos: List[Tuple[float, Tuple[int, ...]]] = []
        for combo in candidate_combos:
            signature = np.sort(
                np.asarray(
                    [distance_matrix[combo[i], combo[j]] for i, j in edge_indices],
                    dtype=np.float64,
                )
            )
            signature_error = float(
                np.sqrt(np.mean((signature - reference_signature) ** 2))
            )
            ranked_combos.append((signature_error, combo))
        ranked_combos.sort(key=lambda item: item[0])

        best_pose: Optional[RigidBodyPose] = None
        best_rank: Tuple[float, float, float] = (-float("inf"), -float("inf"), -float("inf"))
        use_2d_boot_rank = bool(
            int(pattern.num_markers) >= 5 and camera_params and observations_by_camera
        )
        max_ranked = (
            min(len(ranked_combos), self._MULTI_PATTERN_BOOT_TOP_CANDIDATES * 16)
            if use_2d_boot_rank
            else self._MULTI_PATTERN_BOOT_TOP_CANDIDATES
        )
        for _signature_error, combo in ranked_combos[:max_ranked]:
            pose = self.estimate_pose(points_3d[list(combo)], pattern, timestamp)
            if not pose.valid:
                continue
            rank = (-float(pose.rms_error), 0.0, 0.0)
            if use_2d_boot_rank:
                score = self._score_pose_reprojection(
                    pose,
                    pattern,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                )
                if score.get("scored", False):
                    rank = (
                        float(score.get("score", 0.0)),
                        float(score.get("matched_marker_views", 0)),
                        -float(score.get("p95_error_px", 0.0)),
                    )
            if rank > best_rank:
                best_pose = pose
                best_rank = rank
        if best_pose is not None and best_pose.rms_error <= self.max_rms_error_m:
            return best_pose
        three_marker_pose = self._try_multi_pattern_boot_partial_candidate(
            pattern,
            points_3d,
            timestamp,
        )
        if three_marker_pose is not None:
            return three_marker_pose
        return None

    def _try_multi_pattern_boot_partial_candidate(
        self,
        pattern: MarkerPattern,
        points_3d: np.ndarray,
        timestamp: int,
    ) -> Optional[RigidBodyPose]:
        """Bootstrap from a 3-of-4 marker subset when one triangulated point is missing."""
        point_count = int(len(points_3d))
        if (
            len(self.patterns) <= 1
            or pattern.num_markers != 4
            or point_count < 3
            or point_count > 16
        ):
            return None

        distance_matrix = cdist(points_3d, points_3d)
        candidates: List[Tuple[float, Tuple[int, int, int], Tuple[int, int, int]]] = []
        for marker_combo in combinations(range(pattern.num_markers), 3):
            reference_signature = self._distance_signature(
                pattern.marker_positions[list(marker_combo)]
            )
            for point_combo in combinations(range(point_count), 3):
                point_signature = np.sort(
                    np.asarray(
                        [
                            distance_matrix[point_combo[0], point_combo[1]],
                            distance_matrix[point_combo[0], point_combo[2]],
                            distance_matrix[point_combo[1], point_combo[2]],
                        ],
                        dtype=np.float64,
                    )
                )
                signature_error = float(
                    np.sqrt(np.mean((point_signature - reference_signature) ** 2))
                )
                candidates.append((signature_error, marker_combo, point_combo))
        candidates.sort(key=lambda item: item[0])

        best_pose: Optional[RigidBodyPose] = None
        best_error = float("inf")
        max_candidates = max(self._MULTI_PATTERN_BOOT_TOP_CANDIDATES * 2, 12)
        for _signature_error, marker_combo, point_combo in candidates[:max_candidates]:
            reference = np.asarray(
                [pattern.marker_positions[index] for index in marker_combo],
                dtype=np.float64,
            )
            observed = np.asarray(points_3d[list(point_combo)], dtype=np.float64)
            try:
                rotation, position, rms_error = KabschEstimator.estimate(reference, observed)
                quat_xyzw = Rotation.from_matrix(rotation).as_quat()
                quaternion = np.array(
                    [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                    dtype=np.float64,
                )
            except Exception:
                continue
            pose = RigidBodyPose(
                timestamp=timestamp,
                position=position,
                rotation=rotation,
                quaternion=quaternion,
                rms_error=float(rms_error),
                observed_markers=3,
                valid=bool(float(rms_error) <= self.max_rms_error_m),
            )
            if pose.valid and pose.rms_error < best_error:
                best_pose = pose
                best_error = float(pose.rms_error)
        return best_pose

    @staticmethod
    def _distance_signature(points: np.ndarray) -> np.ndarray:
        arr = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        distances = [
            float(np.linalg.norm(arr[i] - arr[j]))
            for i, j in combinations(range(len(arr)), 2)
        ]
        return np.sort(np.asarray(distances, dtype=np.float64))

    def _multi_pattern_boot_combos(
        self,
        distance_matrix: np.ndarray,
        pattern: MarkerPattern,
        reference_signature: np.ndarray,
    ) -> List[Tuple[int, ...]]:
        point_count = int(distance_matrix.shape[0])
        marker_count = int(pattern.num_markers)
        if point_count <= 12 or marker_count != 4:
            return list(combinations(range(point_count), marker_count))

        tolerance = max(0.02, min(0.06, float(self.max_rms_error_m)))
        min_edge = float(reference_signature[0] - tolerance)
        max_edge = float(reference_signature[-1] + tolerance)
        adjacency = [set() for _ in range(point_count)]
        for i in range(point_count):
            for j in range(i + 1, point_count):
                distance = float(distance_matrix[i, j])
                if min_edge <= distance <= max_edge:
                    adjacency[i].add(j)
                    adjacency[j].add(i)

        combos: List[Tuple[int, ...]] = []
        for a in range(point_count):
            neighbors_a = {item for item in adjacency[a] if item > a}
            for b in sorted(neighbors_a):
                common_ab = {item for item in neighbors_a.intersection(adjacency[b]) if item > b}
                for c in sorted(common_ab):
                    common_abc = {item for item in common_ab.intersection(adjacency[c]) if item > c}
                    for d in sorted(common_abc):
                        combos.append((a, b, c, d))
        return combos

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
        self._subset_reference_cache.clear()
        self._subset_ambiguity_cache.clear()

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
        blob_uvs_by_camera: Dict[str, np.ndarray] = {}
        blob_diameters_by_camera: Dict[str, np.ndarray] = {}
        blob_uncertainties_by_camera: Dict[str, np.ndarray] = {}
        blob_anchor_rigids_by_camera: Dict[str, List[str]] = {}
        blob_anchor_markers_by_camera: Dict[str, List[Optional[int]]] = {}
        if camera_params and frames_by_camera:
            space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"
            for camera_id, frame in frames_by_camera.items():
                camera = camera_params.get(camera_id)
                blobs = list(getattr(frame, "blobs", []) or [])
                if camera is None or not blobs:
                    continue
                blob_uvs_by_camera[str(camera_id)] = np.asarray(
                    [self._frame_blob_uv(camera, blob, space) for blob in blobs],
                    dtype=np.float64,
                ).reshape(-1, 2)
                blob_diameters = np.zeros(len(blobs), dtype=np.float64)
                anchor_rigids: List[str] = []
                anchor_markers: List[Optional[int]] = []
                for blob_idx, blob in enumerate(blobs):
                    try:
                        area = float(blob.get("area", 0.0) or 0.0)
                    except Exception:
                        area = 0.0
                    if area > 0.0:
                        blob_diameters[blob_idx] = float(np.sqrt(4.0 * area / np.pi))
                    anchor_rigids.append(str(blob.get("active_anchor_rigid", "") or ""))
                    try:
                        anchor_marker = int(blob.get("active_anchor_marker"))
                    except (TypeError, ValueError):
                        anchor_marker = None
                    anchor_markers.append(anchor_marker)
                blob_diameters_by_camera[str(camera_id)] = blob_diameters
                blob_uncertainties = np.ones(len(blobs), dtype=np.float64)
                valid_diameters = (
                    np.isfinite(blob_diameters)
                    & (blob_diameters > 0.0)
                )
                blob_uncertainties[valid_diameters] = np.maximum(
                    1.0,
                    4.0 / blob_diameters[valid_diameters],
                )
                blob_uncertainties_by_camera[str(camera_id)] = blob_uncertainties
                blob_anchor_rigids_by_camera[str(camera_id)] = anchor_rigids
                blob_anchor_markers_by_camera[str(camera_id)] = anchor_markers
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
                blob_uvs_by_camera=blob_uvs_by_camera,
                blob_diameters_by_camera=blob_diameters_by_camera,
                blob_uncertainties_by_camera=blob_uncertainties_by_camera,
                blob_anchor_rigids_by_camera=blob_anchor_rigids_by_camera,
                blob_anchor_markers_by_camera=blob_anchor_markers_by_camera,
                coordinate_space=coordinate_space,
            )
            tracker.record_object_gating(result)
            results[pattern.name] = result
        self._resolve_object_gating_assignment_conflicts(results)
        for pattern in self.patterns:
            tracker = self.trackers.get(pattern.name)
            result = results.get(pattern.name)
            if tracker is not None and isinstance(result, dict):
                tracker.record_object_gating(result)
        return results

    @staticmethod
    def _resolve_object_gating_assignment_conflicts(
        results: Dict[str, Dict[str, Any]],
    ) -> None:
        """Keep each camera blob owned by the strongest body-level 2D assignment."""
        ownership: Dict[Tuple[str, int], List[Tuple[str, Dict[str, Any], Dict[str, Any]]]] = {}
        for rigid_name, gating in results.items():
            if not isinstance(gating, dict):
                continue
            per_camera = gating.get("per_camera")
            if not isinstance(per_camera, dict):
                continue
            for camera_id, camera_payload in per_camera.items():
                if not isinstance(camera_payload, dict):
                    continue
                for assignment in camera_payload.get("assignments", []) or []:
                    if not isinstance(assignment, dict):
                        continue
                    try:
                        blob_index = int(assignment["blob_index"])
                    except (KeyError, TypeError, ValueError):
                        continue
                    ownership.setdefault((str(camera_id), blob_index), []).append(
                        (str(rigid_name), camera_payload, assignment)
                    )

        removed_by_rigid: Dict[str, int] = {}
        for owners in ownership.values():
            if len({item[0] for item in owners}) <= 1:
                continue
            ranked = sorted(
                owners,
                key=lambda item: (
                    1 if bool(item[2].get("temporal_2d_ownership")) else 0,
                    0 if bool(item[2].get("body_shifted")) else 1,
                    float(item[2].get("temporal_2d_confidence", 0.0) or 0.0),
                    -float(item[2].get("normalized_distance", 0.0) or 0.0),
                    -float(item[2].get("distance_px", 0.0) or 0.0),
                    item[0],
                ),
                reverse=True,
            )
            winner = ranked[0]
            for loser in ranked[1:]:
                if loser[0] == winner[0]:
                    continue
                loser[2]["_drop_body_conflict"] = True
                removed_by_rigid[loser[0]] = int(removed_by_rigid.get(loser[0], 0)) + 1

        for rigid_name, gating in results.items():
            if not isinstance(gating, dict):
                continue
            marker_count = int(gating.get("marker_count", 0) or 0)
            marker_ray_counts = [0 for _ in range(max(0, marker_count))]
            assigned_marker_views = 0
            active_anchor_assignment_count = 0
            ambiguous_assignment_count = 0
            marker_margin_assignment_count = 0
            per_camera = gating.get("per_camera")
            if not isinstance(per_camera, dict):
                continue
            for camera_payload in per_camera.values():
                if not isinstance(camera_payload, dict):
                    continue
                kept = [
                    assignment
                    for assignment in camera_payload.get("assignments", []) or []
                    if isinstance(assignment, dict)
                    and not bool(assignment.pop("_drop_body_conflict", False))
                ]
                camera_payload["assignments"] = kept
                camera_payload["assigned_marker_views"] = int(len(kept))
                camera_payload["unmatched_marker_views"] = int(
                    max(0, int(camera_payload.get("projected_marker_count", 0) or 0) - len(kept))
                )
                camera_payload["active_anchor_assignment_count"] = int(
                    sum(1 for assignment in kept if assignment.get("active_anchor"))
                )
                camera_payload["ambiguous_assignment_count"] = int(
                    sum(1 for assignment in kept if assignment.get("ambiguous"))
                )
                camera_payload["marker_margin_assignment_count"] = int(
                    sum(1 for assignment in kept if assignment.get("marker_margin_ambiguous"))
                )
                for assignment in kept:
                    marker_idx = int(assignment.get("marker_idx", -1))
                    if 0 <= marker_idx < len(marker_ray_counts):
                        marker_ray_counts[marker_idx] += 1
                    assigned_marker_views += 1
                    if assignment.get("active_anchor"):
                        active_anchor_assignment_count += 1
                    if assignment.get("ambiguous"):
                        ambiguous_assignment_count += 1
                    if assignment.get("marker_margin_ambiguous"):
                        marker_margin_assignment_count += 1
            gating["assigned_marker_views"] = int(assigned_marker_views)
            gating["unmatched_marker_views"] = int(
                sum(
                    int(payload.get("unmatched_marker_views", 0) or 0)
                    for payload in per_camera.values()
                    if isinstance(payload, dict)
                )
            )
            gating["duplicate_assignment_count"] = int(removed_by_rigid.get(str(rigid_name), 0))
            gating["ambiguous_assignment_count"] = int(ambiguous_assignment_count)
            gating["marker_margin_assignment_count"] = int(marker_margin_assignment_count)
            gating["active_anchor_assignment_count"] = int(active_anchor_assignment_count)
            gating["markers_with_two_or_more_rays"] = int(
                sum(1 for count in marker_ray_counts if count >= 2)
            )
            gating["markers_with_one_ray"] = int(
                sum(1 for count in marker_ray_counts if count == 1)
            )
            gating["per_marker_ray_count"] = [int(value) for value in marker_ray_counts]

    def _evaluate_object_gating_for_pattern(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        *,
        timestamp: int,
        camera_params: Optional[Dict[str, Any]],
        frames_by_camera: Optional[Dict[str, Any]],
        coordinate_space: str,
        blob_uvs_by_camera: Optional[Dict[str, np.ndarray]] = None,
        blob_diameters_by_camera: Optional[Dict[str, np.ndarray]] = None,
        blob_uncertainties_by_camera: Optional[Dict[str, np.ndarray]] = None,
        blob_anchor_rigids_by_camera: Optional[Dict[str, List[str]]] = None,
        blob_anchor_markers_by_camera: Optional[Dict[str, List[Optional[int]]]] = None,
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
        ambiguous_assignment_count = 0
        marker_margin_assignment_count = 0
        candidate_window_count = 0
        per_camera: Dict[str, Any] = {}
        total_blobs = 0
        total_assigned_blobs = 0
        body_assignment_cost = 0.0
        body_assignment_second_cost = 0.0
        body_assignment_margin = 0.0
        body_assignment_margin_count = 0
        active_anchor_assignment_count = 0
        active_anchor_blocked_count = 0
        active_anchor_observations: List[Dict[str, Any]] = []
        space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"

        for camera_id, frame in frames_by_camera.items():
            camera = camera_params.get(camera_id)
            blob_uvs = (
                blob_uvs_by_camera.get(str(camera_id))
                if isinstance(blob_uvs_by_camera, dict)
                else None
            )
            if camera is None or blob_uvs is None or len(blob_uvs) == 0:
                continue
            projected = self._project_markers_to_camera(markers_world, camera, space)
            if not projected:
                continue
            projected_uvs = np.asarray(projected, dtype=np.float64).reshape(-1, 2)

            total_blobs += len(blob_uvs)
            projected_count = int(len(projected))
            candidate_window_count += projected_count
            delta = projected_uvs[:, None, :] - blob_uvs[None, :, :]
            distances = np.sqrt(np.sum(delta * delta, axis=2))
            blob_diameters = (
                blob_diameters_by_camera.get(str(camera_id))
                if isinstance(blob_diameters_by_camera, dict)
                else None
            )
            if blob_diameters is None or len(blob_diameters) != len(blob_uvs):
                blob_diameters = np.zeros(len(blob_uvs), dtype=np.float64)
            blob_uncertainties = (
                blob_uncertainties_by_camera.get(str(camera_id))
                if isinstance(blob_uncertainties_by_camera, dict)
                else None
            )
            if blob_uncertainties is None or len(blob_uncertainties) != len(blob_uvs):
                blob_uncertainties = np.ones(len(blob_uvs), dtype=np.float64)
            anchor_rigids = (
                blob_anchor_rigids_by_camera.get(str(camera_id))
                if isinstance(blob_anchor_rigids_by_camera, dict)
                else None
            )
            anchor_markers = (
                blob_anchor_markers_by_camera.get(str(camera_id))
                if isinstance(blob_anchor_markers_by_camera, dict)
                else None
            )
            if anchor_rigids is None or len(anchor_rigids) != len(blob_uvs):
                anchor_rigids = [""] * len(blob_uvs)
            if anchor_markers is None or len(anchor_markers) != len(blob_uvs):
                anchor_markers = [None] * len(blob_uvs)
            blob_uncertainty_px = np.asarray(
                blob_uncertainties,
                dtype=np.float64,
            ).reshape(1, -1)
            effective_gate_px = pixel_gate_px * np.maximum(blob_uncertainty_px, 1.0)
            normalized_distances = distances / np.maximum(blob_uncertainty_px, 1e-6)
            cost = np.where(
                distances <= effective_gate_px,
                normalized_distances,
                1e9,
            )
            for blob_idx, anchor_rigid in enumerate(anchor_rigids):
                if not anchor_rigid:
                    continue
                marker_idx = anchor_markers[blob_idx]
                if str(anchor_rigid) != str(pattern.name):
                    active_anchor_blocked_count += int(cost.shape[0])
                    cost[:, blob_idx] = 1e9
                    continue
                if marker_idx is None or marker_idx < 0 or marker_idx >= marker_count:
                    continue
                active_anchor_observations.append(
                    {
                        "camera_id": str(camera_id),
                        "marker_idx": int(marker_idx),
                        "blob_index": int(blob_idx),
                        "observed_uv": [
                            float(blob_uvs[int(blob_idx)][0]),
                            float(blob_uvs[int(blob_idx)][1]),
                        ],
                        "uncertainty_px": float(
                            blob_uncertainty_px[0, int(blob_idx)]
                            if 0 <= int(blob_idx) < blob_uncertainty_px.shape[1]
                            else 1.0
                        ),
                    }
                )
                blocked_rows = [row for row in range(marker_count) if row != int(marker_idx)]
                if blocked_rows:
                    active_anchor_blocked_count += int(len(blocked_rows))
                    cost[blocked_rows, blob_idx] = 1e9

            raw_assignments: List[Tuple[int, int, float]] = []
            raw_assignment_costs: List[float] = []
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
                    raw_assignments.append((marker_idx, blob_idx, float(distances[row, col])))
                    raw_assignment_costs.append(float(cost[row, col]))
            temporal_assignment = None
            if (
                bool(config.temporal_2d_ownership_recovery)
                and len(self.patterns) > 1
                and int(pattern.num_markers) >= 5
                and int(tracker.track_count) > 0
                and int(len(raw_assignments)) < max(3, marker_count - 1)
            ):
                temporal_assignment = self._temporal_2d_ownership_assignment(
                    pattern,
                    timestamp,
                    str(camera_id),
                    camera,
                    coordinate_space,
                    bool(config.temporal_2d_motion_prediction),
                    projected_uvs,
                    blob_uvs,
                    blob_uncertainty_px.reshape(-1),
                    raw_assignments,
                )
                if temporal_assignment is not None:
                    raw_assignments = list(temporal_assignment["assignments"])
                    raw_assignment_costs = list(temporal_assignment["costs"])
            body_shifted = None
            shifted_used = False
            cached_offsets: List[np.ndarray] = []
            cache_key = (str(pattern.name), str(camera_id))
            cache_hit = False
            cache_allowed = bool(
                config.body_level_2d_candidate_cache
                and tracker.mode == TrackMode.REACQUIRE
            )
            if cache_allowed:
                self._body_level_2d_cache_query_count += 1
                cached = self._body_level_2d_candidate_cache.get(cache_key)
                if isinstance(cached, dict):
                    age_us = max(0, int(timestamp) - int(cached.get("timestamp", 0) or 0))
                    raw_cached_offsets = cached.get("offsets_px")
                    if not isinstance(raw_cached_offsets, list):
                        raw_cached_offsets = [cached.get("offset_px", [0.0, 0.0])]
                    for offset_payload in raw_cached_offsets[: max(1, int(config.body_level_2d_cache_max_bins))]:
                        try:
                            cached_offset = np.asarray(
                                offset_payload,
                                dtype=np.float64,
                            ).reshape(2)
                        except (TypeError, ValueError):
                            continue
                        if age_us <= 250_000 and np.isfinite(cached_offset).all():
                            cached_offsets.append(cached_offset)
                    if cached_offsets:
                        cache_hit = True
                        self._body_level_2d_cache_hit_count += 1
                        self._body_level_2d_cache_bin_total += int(len(cached_offsets))
            if (
                int(pattern.num_markers) >= 5
                and bool(config.body_level_2d_recovery)
                and int(tracker.track_count) > 0
                and int(len(raw_assignments))
                <= max(0, int(config.body_level_2d_trigger_max_assignments))
            ):
                body_shifted = self._translated_body_assignment(
                    projected_uvs,
                    blob_uvs,
                    blob_uncertainty_px.reshape(-1),
                    pixel_gate_px=float(pixel_gate_px),
                    max_offsets=int(config.body_level_2d_max_offsets),
                    nearest_per_marker=int(config.body_level_2d_nearest_per_marker),
                    assignment_nearest_per_marker=int(
                        config.body_level_2d_assignment_nearest_per_marker
                    ),
                    max_distance_scale=float(config.body_level_2d_max_distance_scale),
                    marker_count=marker_count,
                    seed_offsets=cached_offsets,
                    early_accept_coverage=(
                        max(4, marker_count - 1)
                        if bool(config.body_level_2d_cache_early_exit)
                        else 0
                    ),
                )
                if (
                    body_shifted is not None
                    and len(body_shifted.get("assignments", [])) >= max(3, len(raw_assignments))
                    and float(body_shifted.get("margin", 0.0) or 0.0)
                    >= float(config.body_level_2d_min_margin)
                ):
                    raw_assignments = list(body_shifted["assignments"])
                    raw_assignment_costs = list(body_shifted["costs"])
                    shifted_used = True
                    if body_shifted.get("early_accepted"):
                        self._body_level_2d_cache_early_exit_count += 1
                    if bool(config.body_level_2d_candidate_cache):
                        offset_bins: List[List[float]] = []
                        for item in body_shifted.get("nbest", []) or []:
                            offset_values = item.get("offset_px") if isinstance(item, dict) else None
                            if not isinstance(offset_values, list) or len(offset_values) < 2:
                                continue
                            offset_bins.append([float(offset_values[0]), float(offset_values[1])])
                            if len(offset_bins) >= max(1, int(config.body_level_2d_cache_max_bins)):
                                break
                        if not offset_bins:
                            offset_bins.append(
                                [
                                    float(value)
                                    for value in body_shifted.get("offset_px", [0.0, 0.0])[:2]
                                ]
                            )
                        self._body_level_2d_candidate_cache[cache_key] = {
                            "timestamp": int(timestamp),
                            "offset_px": list(offset_bins[0]),
                            "offsets_px": offset_bins,
                        }
            if (
                bool(config.body_level_2d_candidate_cache)
                and not shifted_used
                and len(raw_assignments) >= max(3, marker_count - 1)
            ):
                offsets: List[np.ndarray] = []
                for assignment in raw_assignments:
                    marker_idx = int(assignment[0])
                    blob_idx = int(assignment[1])
                    if (
                        0 <= marker_idx < len(projected_uvs)
                        and 0 <= blob_idx < len(blob_uvs)
                    ):
                        offsets.append(
                            np.asarray(blob_uvs[blob_idx] - projected_uvs[marker_idx], dtype=np.float64)
                        )
                if offsets:
                    offset_array = np.asarray(offsets, dtype=np.float64).reshape(-1, 2)
                    median_offset = np.median(offset_array, axis=0)
                    if np.isfinite(median_offset).all():
                        offset_bins = [[float(value) for value in median_offset.reshape(2)]]
                        if raw_assignment_costs:
                            ranked_indices = np.argsort(
                                np.asarray(raw_assignment_costs, dtype=np.float64)
                            )
                        else:
                            ranked_indices = np.arange(len(offset_array))
                        for offset_index in ranked_indices:
                            if len(offset_bins) >= max(1, int(config.body_level_2d_cache_max_bins)):
                                break
                            offset = offset_array[int(offset_index)].reshape(2)
                            if not np.isfinite(offset).all():
                                continue
                            key = (round(float(offset[0]), 2), round(float(offset[1]), 2))
                            if any(
                                (round(float(existing[0]), 2), round(float(existing[1]), 2)) == key
                                for existing in offset_bins
                            ):
                                continue
                            offset_bins.append([float(offset[0]), float(offset[1])])
                        self._body_level_2d_candidate_cache[cache_key] = {
                            "timestamp": int(timestamp),
                            "offset_px": list(offset_bins[0]),
                            "offsets_px": offset_bins,
                        }
            best_assignment_cost = float(sum(raw_assignment_costs))
            second_assignment_cost = float("inf")
            # The exact second-best body assignment required one Hungarian
            # solve per matched marker view. It is diagnostic-only and was a
            # major GUI hot-path cost, so keep the primary body assignment and
            # local marker/blob ambiguity signals without recomputing N alts.
            if raw_assignments:
                body_assignment_cost += best_assignment_cost
                body_assignment_margin_count += 1
            ambiguous_indices: set[int] = set()
            min_blob_separation = float(config.ambiguous_blob_min_separation_px)
            diameter_overlap_ratio = float(config.ambiguous_blob_diameter_overlap_ratio)
            min_marker_assignment_margin = float(
                config.ambiguous_marker_assignment_min_margin_px
            )
            marker_margin_indices: set[int] = set()
            if min_marker_assignment_margin > 0.0 and len(raw_assignments) >= 2:
                assignment_markers = np.asarray(
                    [int(assignment[0]) for assignment in raw_assignments],
                    dtype=np.int64,
                )
                assignment_blobs = np.asarray(
                    [int(assignment[1]) for assignment in raw_assignments],
                    dtype=np.int64,
                )
                valid_assignment_mask = (
                    (assignment_markers >= 0)
                    & (assignment_markers < distances.shape[0])
                    & (assignment_blobs >= 0)
                    & (assignment_blobs < distances.shape[1])
                )
                if np.any(valid_assignment_mask):
                    valid_indices = np.flatnonzero(valid_assignment_mask)
                    valid_markers = assignment_markers[valid_indices]
                    valid_blobs = assignment_blobs[valid_indices]
                    assigned_distances = distances[valid_markers, valid_blobs]
                    marker_distances = distances[:, valid_blobs].copy()
                    marker_distances[valid_markers, np.arange(len(valid_indices))] = np.inf
                    alternative_distances = np.min(marker_distances, axis=0)
                    low_margin = (
                        np.isfinite(alternative_distances)
                        & (
                            (alternative_distances - assigned_distances)
                            <= min_marker_assignment_margin
                        )
                    )
                    marker_margin_indices.update(int(index) for index in valid_indices[low_margin])
                ambiguous_indices.update(marker_margin_indices)
            if min_blob_separation > 0.0 and len(raw_assignments) >= 2:
                assigned_by_blob = {
                    int(assignment[1]): assignment_idx
                    for assignment_idx, assignment in enumerate(raw_assignments)
                }
                blob_uv_array = np.asarray(blob_uvs, dtype=np.float64).reshape(-1, 2)
                assignment_blob_indices = np.asarray(
                    [int(assignment[1]) for assignment in raw_assignments],
                    dtype=np.int64,
                )
                valid_blob_rows = (
                    (assignment_blob_indices >= 0)
                    & (assignment_blob_indices < len(blob_uv_array))
                )
                valid_assignment_rows = np.flatnonzero(valid_blob_rows)
                if valid_assignment_rows.size:
                    assigned_blob_indices_valid = assignment_blob_indices[valid_assignment_rows]
                    assigned_blob_uvs = blob_uv_array[assigned_blob_indices_valid]
                    blob_delta = assigned_blob_uvs[:, None, :] - blob_uv_array[None, :, :]
                    assigned_blob_distances_sq = np.sum(blob_delta * blob_delta, axis=2)
                    blob_index_array = np.arange(len(blob_uv_array))
                    for local_row, assignment_idx_value in enumerate(valid_assignment_rows):
                        assigned_blob = int(assigned_blob_indices_valid[local_row])
                        ambiguous_distance_px = np.full(
                            len(blob_uv_array),
                            min_blob_separation,
                            dtype=np.float64,
                        )
                        if diameter_overlap_ratio > 0.0:
                            average_diameters = 0.5 * (
                                blob_diameters + float(blob_diameters[assigned_blob])
                            )
                            ambiguous_distance_px = np.maximum(
                                ambiguous_distance_px,
                                diameter_overlap_ratio * average_diameters,
                            )
                        nearby_blobs = np.flatnonzero(
                            (
                                assigned_blob_distances_sq[local_row]
                                <= ambiguous_distance_px * ambiguous_distance_px
                            )
                            & (blob_index_array != assigned_blob)
                        )
                        if nearby_blobs.size <= 0:
                            continue
                        assignment_idx = int(assignment_idx_value)
                        ambiguous_indices.add(assignment_idx)
                        for other_blob_value in nearby_blobs:
                            other_assignment_idx = assigned_by_blob.get(int(other_blob_value))
                            if other_assignment_idx is not None:
                                ambiguous_indices.add(other_assignment_idx)
            assignments: List[Dict[str, Any]] = []
            for assignment_idx, assignment in enumerate(raw_assignments):
                assignment_is_ambiguous = assignment_idx in ambiguous_indices
                assignment_is_marker_margin = assignment_idx in marker_margin_indices
                if assignment_is_ambiguous:
                    ambiguous_assignment_count += 1
                if assignment_is_marker_margin:
                    marker_margin_assignment_count += 1
                marker_idx = int(assignment[0])
                blob_idx = int(assignment[1])
                assignment_anchor_rigid = str(anchor_rigids[blob_idx]) if 0 <= blob_idx < len(anchor_rigids) else ""
                assignment_anchor_marker = (
                    anchor_markers[blob_idx]
                    if 0 <= blob_idx < len(anchor_markers)
                    else None
                )
                assignment_is_active_anchor = bool(assignment_anchor_rigid)
                if assignment_is_active_anchor:
                    active_anchor_assignment_count += 1
                marker_ray_counts[marker_idx] += 1
                assigned_marker_views += 1
                assignments.append(
                    {
                        "marker_idx": marker_idx,
                        "blob_index": blob_idx,
                        "observed_uv": [
                            float(blob_uvs[int(blob_idx)][0]),
                            float(blob_uvs[int(blob_idx)][1]),
                        ],
                        "distance_px": float(assignment[2]),
                        "normalized_distance": float(
                            raw_assignment_costs[assignment_idx]
                            if assignment_idx < len(raw_assignment_costs)
                            else 0.0
                        ),
                        "uncertainty_px": float(
                            blob_uncertainty_px[0, int(assignment[1])]
                            if 0 <= int(assignment[1]) < blob_uncertainty_px.shape[1]
                            else 1.0
                        ),
                        "ambiguous": bool(assignment_is_ambiguous),
                        "marker_margin_ambiguous": bool(assignment_is_marker_margin),
                        "active_anchor": bool(assignment_is_active_anchor),
                        "active_anchor_rigid": assignment_anchor_rigid,
                        "active_anchor_marker": (
                            int(assignment_anchor_marker)
                            if assignment_anchor_marker is not None
                            else None
                        ),
                        "body_shifted": bool(shifted_used),
                        "temporal_2d_ownership": bool(
                            temporal_assignment is not None
                            and assignment_idx < len(raw_assignments)
                            and len(raw_assignments[assignment_idx]) >= 4
                        ),
                        "temporal_2d_confidence": float(
                            raw_assignments[assignment_idx][3]
                            if (
                                temporal_assignment is not None
                                and assignment_idx < len(raw_assignments)
                                and len(raw_assignments[assignment_idx]) >= 4
                            )
                            else 0.0
                        ),
                    }
                )

            unmatched_marker_views += max(0, projected_count - len(assignments))
            total_assigned_blobs += len(assignments)
            per_camera[str(camera_id)] = {
                "blob_count": int(len(blob_uvs)),
                "projected_marker_count": int(projected_count),
                "assigned_marker_views": int(len(assignments)),
                "unmatched_marker_views": int(max(0, projected_count - len(assignments))),
                "ambiguous_assignment_count": int(len(ambiguous_indices)),
                "marker_margin_assignment_count": int(len(marker_margin_indices)),
                "body_assignment_cost": float(best_assignment_cost),
                "body_assignment_second_cost": (
                    float(second_assignment_cost)
                    if np.isfinite(second_assignment_cost)
                    else 0.0
                ),
                "body_assignment_margin": (
                    float(max(0.0, second_assignment_cost - best_assignment_cost))
                    if np.isfinite(second_assignment_cost)
                    else 0.0
                ),
                "active_anchor_assignment_count": int(
                    sum(1 for assignment in assignments if assignment.get("active_anchor"))
                ),
                "body_shifted_assignment": bool(shifted_used),
                "body_level_2d_cache_hit": bool(cache_hit),
                "body_level_2d_cache_early_exit": bool(
                    isinstance(body_shifted, dict)
                    and bool(body_shifted.get("early_accepted", False))
                ),
                "body_shifted_offset_px": (
                    [float(value) for value in body_shifted.get("offset_px", [])]
                    if shifted_used and isinstance(body_shifted, dict)
                    else []
                ),
                "body_shifted_mean_normalized_cost": (
                    float(body_shifted.get("mean_normalized_cost", 0.0))
                    if shifted_used and isinstance(body_shifted, dict)
                    else 0.0
                ),
                "body_level_2d_nbest_candidate_count": (
                    int(body_shifted.get("candidate_count", 0))
                    if isinstance(body_shifted, dict)
                    else 0
                ),
                "body_level_2d_nbest_margin": (
                    float(body_shifted.get("margin", 0.0))
                    if isinstance(body_shifted, dict)
                    else 0.0
                ),
                "body_level_2d_nbest": (
                    body_shifted.get("nbest", [])
                    if (
                        bool(config.body_level_2d_keep_nbest_diagnostics)
                        and isinstance(body_shifted, dict)
                    )
                    else []
                ),
                "temporal_2d_ownership_assignment": bool(temporal_assignment is not None),
                "temporal_2d_ownership_mean_cost": (
                    float(temporal_assignment.get("mean_normalized_cost", 0.0))
                    if isinstance(temporal_assignment, dict)
                    else 0.0
                ),
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
            "ambiguous_assignment_count": int(ambiguous_assignment_count),
            "marker_margin_assignment_count": int(marker_margin_assignment_count),
            "active_anchor_assignment_count": int(active_anchor_assignment_count),
            "active_anchor_blocked_count": int(active_anchor_blocked_count),
            "active_anchor_observations": active_anchor_observations,
            "markers_with_two_or_more_rays": int(markers_with_two_or_more),
            "markers_with_one_ray": int(markers_with_one),
            "single_ray_candidates": int(markers_with_one if allow_single_ray else 0),
            "generic_fallback_blob_count": int(max(0, total_blobs - total_assigned_blobs)),
            "allow_single_ray": bool(allow_single_ray),
            "body_assignment": {
                "policy": "keep_body_assignment_mark_local_ambiguity",
                "cost": float(body_assignment_cost),
                "second_cost": float(body_assignment_second_cost),
                "margin": float(body_assignment_margin),
                "mean_margin": float(
                    body_assignment_margin / body_assignment_margin_count
                    if body_assignment_margin_count
                    else 0.0
                ),
                "camera_count": int(body_assignment_margin_count),
            },
            "thresholds": thresholds,
            "per_marker_ray_count": [int(value) for value in marker_ray_counts],
            "per_camera": per_camera,
        }

    def _temporal_2d_ownership_assignment(
        self,
        pattern: MarkerPattern,
        timestamp: int,
        camera_id: str,
        camera: Any,
        coordinate_space: str,
        use_motion_prediction: bool,
        projected_uvs: np.ndarray,
        blob_uvs: np.ndarray,
        blob_uncertainties: np.ndarray,
        existing_assignments: List[Tuple[int, int, float]],
    ) -> Optional[Dict[str, Any]]:
        """Recover marker-to-blob candidates from short-lived confirmed 3D ownership."""
        views_by_marker = self._last_confirmed_marker_views_by_rigid.get(pattern.name, {})
        if not views_by_marker:
            return None
        projected = np.asarray(projected_uvs, dtype=np.float64).reshape(-1, 2)
        blobs = np.asarray(blob_uvs, dtype=np.float64).reshape(-1, 2)
        uncertainties = np.asarray(blob_uncertainties, dtype=np.float64).reshape(-1)
        if len(projected) <= 0 or len(blobs) <= 0 or not np.isfinite(projected).all():
            return None
        if len(uncertainties) != len(blobs):
            uncertainties = np.ones(len(blobs), dtype=np.float64)
        uncertainties = np.maximum(uncertainties, 1e-6)
        used_markers = {int(item[0]) for item in existing_assignments}
        used_blobs = {int(item[1]) for item in existing_assignments}
        marker_rows: List[Dict[str, Any]] = []
        max_age_us = 250_000
        body_span_px = max(1.0, float(np.linalg.norm(np.ptp(projected, axis=0))))
        search_radius_px = max(18.0, body_span_px * 0.85)
        predicted_points = (
            self._predicted_marker_points_from_history(pattern.name, timestamp)
            if use_motion_prediction
            else {}
        )
        for marker_idx, views_by_camera in sorted(views_by_marker.items()):
            marker_idx = int(marker_idx)
            if marker_idx in used_markers or marker_idx < 0 or marker_idx >= len(projected):
                continue
            if not isinstance(views_by_camera, dict):
                continue
            view = views_by_camera.get(str(camera_id))
            if not isinstance(view, dict):
                continue
            age_us = max(0, int(timestamp) - int(view.get("timestamp", 0) or 0))
            if age_us > max_age_us:
                continue
            try:
                previous_uv = np.asarray(view.get("observed_uv"), dtype=np.float64).reshape(2)
            except (TypeError, ValueError):
                continue
            if not np.isfinite(previous_uv).all():
                continue
            motion_uv = None
            predicted_point = predicted_points.get(marker_idx)
            if predicted_point is not None:
                motion_uv = self._project_world_point_to_camera(
                    predicted_point,
                    camera,
                    coordinate_space,
                )
                if motion_uv is not None and not np.isfinite(motion_uv).all():
                    motion_uv = None
            age_confidence = max(0.0, 1.0 - float(age_us) / float(max_age_us))
            marker_rows.append(
                {
                    "marker_idx": marker_idx,
                    "previous_uv": previous_uv,
                    "projected_uv": projected[marker_idx],
                    "motion_uv": motion_uv,
                    "confidence": float(age_confidence),
                }
            )
        if not marker_rows:
            return None

        cost = np.zeros((len(marker_rows), len(blobs)), dtype=np.float64)
        distance_px = np.zeros_like(cost)
        confidence = np.zeros_like(cost)
        for row, marker in enumerate(marker_rows):
            projected_delta = np.linalg.norm(
                blobs - marker["projected_uv"].reshape(1, 2),
                axis=1,
            )
            previous_delta = np.linalg.norm(
                blobs - marker["previous_uv"].reshape(1, 2),
                axis=1,
            )
            combined_delta = np.minimum(projected_delta, previous_delta + 0.25 * body_span_px)
            motion_uv = marker.get("motion_uv")
            if motion_uv is not None:
                motion_to_pose = float(
                    np.linalg.norm(
                        np.asarray(motion_uv, dtype=np.float64).reshape(2)
                        - marker["projected_uv"].reshape(2)
                    )
                )
                motion_delta = np.linalg.norm(
                    blobs - np.asarray(motion_uv, dtype=np.float64).reshape(1, 2),
                    axis=1,
                )
                combined_delta = np.minimum(
                    combined_delta,
                    motion_delta + 0.50 * motion_to_pose,
                )
            distance_px[row, :] = combined_delta
            confidence[row, :] = float(marker["confidence"]) / (
                1.0 + combined_delta / search_radius_px
            )
            cost[row, :] = combined_delta / uncertainties + (1.0 - float(marker["confidence"])) * 2.0
        for blob_index in used_blobs:
            if 0 <= blob_index < cost.shape[1]:
                cost[:, blob_index] = 1e9
        row_ind, col_ind = linear_sum_assignment(cost)
        assignments: List[Tuple[int, ...]] = list(existing_assignments)
        costs: List[float] = []
        added = 0
        for row, col in zip(row_ind, col_ind):
            if float(cost[row, col]) >= 1e9:
                continue
            if float(distance_px[row, col]) > search_radius_px:
                continue
            marker_idx = int(marker_rows[int(row)]["marker_idx"])
            assignments.append(
                (
                    marker_idx,
                    int(col),
                    float(distance_px[row, col]),
                    float(confidence[row, col]),
                )
            )
            costs.append(float(cost[row, col]))
            added += 1
        if added <= 0:
            return None
        return {
            "assignments": assignments,
            "costs": [0.0 for _ in existing_assignments] + costs,
            "mean_normalized_cost": float(
                np.mean(np.asarray(costs, dtype=np.float64)) if costs else 0.0
            ),
        }

    def _predicted_marker_points_from_history(
        self,
        rigid_name: str,
        timestamp: int,
    ) -> Dict[int, np.ndarray]:
        history_by_marker = self._last_confirmed_marker_points_by_rigid.get(str(rigid_name), {})
        predictions: Dict[int, np.ndarray] = {}
        max_age_us = 250_000
        max_dt_s = 0.25
        max_speed_m_s = 8.0
        for marker_idx, history in history_by_marker.items():
            if not isinstance(history, list) or not history:
                continue
            latest = history[-1]
            try:
                latest_ts = int(latest.get("timestamp", 0) or 0)
                latest_point = np.asarray(latest.get("point"), dtype=np.float64).reshape(3)
            except (TypeError, ValueError):
                continue
            age_us = max(0, int(timestamp) - latest_ts)
            if age_us > max_age_us or not np.isfinite(latest_point).all():
                continue
            predicted = latest_point.copy()
            if len(history) >= 2:
                previous = history[-2]
                try:
                    previous_ts = int(previous.get("timestamp", 0) or 0)
                    previous_point = np.asarray(previous.get("point"), dtype=np.float64).reshape(3)
                except (TypeError, ValueError):
                    previous_ts = 0
                    previous_point = np.zeros(3, dtype=np.float64)
                dt_s = float(latest_ts - previous_ts) / 1_000_000.0
                if (
                    0.0 < dt_s <= max_dt_s
                    and np.isfinite(previous_point).all()
                ):
                    velocity = (latest_point - previous_point) / dt_s
                    speed = float(np.linalg.norm(velocity))
                    if np.isfinite(speed) and speed > max_speed_m_s:
                        velocity *= max_speed_m_s / max(speed, 1e-9)
                    predicted = latest_point + velocity * min(float(age_us) / 1_000_000.0, max_dt_s)
            if np.isfinite(predicted).all():
                predictions[int(marker_idx)] = predicted
        return predictions

    @staticmethod
    def _translated_body_assignment(
        projected_uvs: np.ndarray,
        blob_uvs: np.ndarray,
        blob_uncertainties: np.ndarray,
        *,
        pixel_gate_px: float,
        max_offsets: int,
        nearest_per_marker: int,
        assignment_nearest_per_marker: int,
        max_distance_scale: float,
        marker_count: int,
        seed_offsets: Optional[List[np.ndarray]] = None,
        early_accept_coverage: int = 0,
    ) -> Optional[Dict[str, Any]]:
        """Rank whole-body 2D marker assignments after candidate translations.

        This is a passive body-level evidence path: when the absolute predicted
        position has drifted, keep the predicted orientation/shape and let the
        2D body constellation choose among several translated hypotheses before
        triangulation.
        """
        projected = np.asarray(projected_uvs, dtype=np.float64).reshape(-1, 2)
        blobs = np.asarray(blob_uvs, dtype=np.float64).reshape(-1, 2)
        uncertainties = np.asarray(blob_uncertainties, dtype=np.float64).reshape(-1)
        if (
            len(projected) <= 0
            or len(blobs) < 2
            or int(marker_count) <= 0
            or not np.isfinite(projected).all()
            or not np.isfinite(blobs).all()
        ):
            return None
        if len(uncertainties) != len(blobs):
            uncertainties = np.ones(len(blobs), dtype=np.float64)
        uncertainties = np.maximum(uncertainties, 1e-6)
        base_deltas = projected[:, None, :] - blobs[None, :, :]
        base_distances = np.sqrt(np.sum(base_deltas * base_deltas, axis=2))
        base_normalized = base_distances / uncertainties.reshape(1, -1)

        offset_items: List[Tuple[float, str, np.ndarray]] = []
        for seed_offset in seed_offsets or []:
            try:
                seed = np.asarray(seed_offset, dtype=np.float64).reshape(2)
            except (TypeError, ValueError):
                continue
            if np.isfinite(seed).all():
                offset_items.append((-1.0, "cache", seed))
        offset_items.append((0.0, "zero", np.zeros(2, dtype=np.float64)))
        nearest_per_marker = min(max(1, int(nearest_per_marker)), len(blobs))
        for marker_idx in range(min(int(marker_count), len(projected))):
            ranked_blob_indices = np.argsort(base_normalized[marker_idx])[:nearest_per_marker]
            for blob_idx in ranked_blob_indices:
                offset_items.append(
                    (
                        float(base_normalized[marker_idx, int(blob_idx)]),
                        "nearest",
                        blobs[int(blob_idx)] - projected[marker_idx],
                    )
                )
        offset_items.sort(key=lambda item: item[0])
        candidate_offsets: List[Tuple[str, np.ndarray]] = []
        seen_offsets: set[Tuple[int, int]] = set()
        for _cost, source, offset in offset_items:
            key = (int(round(float(offset[0]) * 4.0)), int(round(float(offset[1]) * 4.0)))
            if key in seen_offsets:
                continue
            seen_offsets.add(key)
            candidate_offsets.append((source, np.asarray(offset, dtype=np.float64).reshape(2)))
            if len(candidate_offsets) >= max(1, int(max_offsets)):
                break
        if not candidate_offsets:
            return None

        scale = max(
            1.0,
            float(np.linalg.norm(np.ptp(projected, axis=0))),
        )
        max_distance_px = max(float(pixel_gate_px), float(max_distance_scale) * scale)
        candidates: List[Dict[str, Any]] = []
        for offset_source, offset in candidate_offsets:
            shifted = projected + offset.reshape(1, 2)
            deltas = shifted[:, None, :] - blobs[None, :, :]
            distances = np.sqrt(np.sum(deltas * deltas, axis=2))
            normalized = distances / uncertainties.reshape(1, -1)
            row_count = normalized.shape[0]
            candidate_cols: set[int] = set()
            per_row_limit = min(
                max(1, int(assignment_nearest_per_marker)),
                normalized.shape[1],
            )
            for row in range(row_count):
                candidate_cols.update(
                    int(col) for col in np.argsort(normalized[row])[:per_row_limit]
                )
            if len(candidate_cols) < min(3, int(marker_count)):
                continue
            col_lookup = np.asarray(sorted(candidate_cols), dtype=np.int64)
            reduced_normalized = normalized[:, col_lookup]
            row_ind, reduced_col_ind = linear_sum_assignment(reduced_normalized)
            col_ind = col_lookup[reduced_col_ind]
            assignments: List[Tuple[int, int, float]] = []
            costs: List[float] = []
            for row, col in zip(row_ind, col_ind):
                if float(distances[row, col]) > max_distance_px * float(uncertainties[col]):
                    continue
                assignments.append((int(row), int(col), float(distances[row, col])))
                costs.append(float(normalized[row, col]))
            if len(assignments) < min(3, int(marker_count)):
                continue
            mean_cost = float(np.mean(np.asarray(costs, dtype=np.float64)))
            p95_cost = float(np.percentile(np.asarray(costs, dtype=np.float64), 95))
            offset_norm = float(np.linalg.norm(offset) / scale)
            coverage = int(len({int(item[0]) for item in assignments}))
            score = (
                2.0 * float(coverage)
                - mean_cost
                - 0.25 * p95_cost
                - 0.15 * offset_norm
            )
            candidates.append(
                {
                    "score": float(score),
                    "coverage": int(coverage),
                    "offset_source": str(offset_source),
                    "assignments": assignments,
                    "costs": costs,
                    "offset_px": [float(offset[0]), float(offset[1])],
                    "offset_norm": float(offset_norm),
                    "mean_normalized_cost": mean_cost,
                    "p95_normalized_cost": p95_cost,
                }
            )
            if (
                str(offset_source) == "cache"
                and int(early_accept_coverage) > 0
                and int(coverage) >= int(early_accept_coverage)
            ):
                best = dict(candidates[-1])
                best["candidate_count"] = int(len(candidates))
                best["margin"] = float(best.get("score", 0.0))
                best["early_accepted"] = True
                best["nbest"] = [
                    {
                        "score": float(best.get("score", 0.0)),
                        "coverage": int(best.get("coverage", 0)),
                        "offset_px": [float(value) for value in best.get("offset_px", [])],
                        "offset_source": str(best.get("offset_source", "")),
                        "mean_normalized_cost": float(
                            best.get("mean_normalized_cost", 0.0)
                        ),
                        "p95_normalized_cost": float(best.get("p95_normalized_cost", 0.0)),
                    }
                ]
                return best
        if not candidates:
            return None
        candidates.sort(
            key=lambda item: (
                int(item.get("coverage", 0)),
                float(item.get("score", 0.0)),
                -float(item.get("mean_normalized_cost", 0.0)),
                -float(item.get("offset_norm", 0.0)),
            ),
            reverse=True,
        )
        best = dict(candidates[0])
        second = candidates[1] if len(candidates) > 1 else {}
        best["candidate_count"] = int(len(candidates))
        best["margin"] = float(
            float(best.get("score", 0.0))
            - float(second.get("score", 0.0) if second else 0.0)
        )
        best["nbest"] = [
            {
                "score": float(item.get("score", 0.0)),
                "coverage": int(item.get("coverage", 0)),
                "offset_px": [float(value) for value in item.get("offset_px", [])],
                "offset_source": str(item.get("offset_source", "")),
                "mean_normalized_cost": float(item.get("mean_normalized_cost", 0.0)),
                "p95_normalized_cost": float(item.get("p95_normalized_cost", 0.0)),
            }
            for item in candidates[:5]
        ]
        best["early_accepted"] = False
        return best

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
        self._estimate_pose_call_count += 1
        self._estimate_pose_point_total += int(len(points_3d))
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

    @staticmethod
    def _matched_blob_views(score: Dict[str, Any]) -> set[Tuple[str, int]]:
        matched = score.get("matched_observations", []) if isinstance(score, dict) else []
        views: set[Tuple[str, int]] = set()
        if not isinstance(matched, list):
            return views
        for item in matched:
            if not isinstance(item, dict):
                continue
            try:
                blob_index = int(item.get("blob_index", -1))
            except (TypeError, ValueError):
                continue
            if blob_index >= 0:
                views.add((str(item.get("camera_id", "")), blob_index))
        return views

    def _record_confirmed_2d_ownership(
        self,
        candidate: _PoseCandidate,
        timestamp: int,
    ) -> None:
        score = candidate.score if isinstance(candidate.score, dict) else {}
        if (
            not candidate.pose.valid
            or candidate.invalid_reason
            or candidate.source != "rigid_hint"
            or not score.get("rigid_hint_marker_indices")
            or candidate.source
            in {
                "prediction",
                "prediction_hold",
                "prediction_hold_2d_constrained",
                "pose_continuity_guard",
                "position_continuity_guard",
            }
        ):
            return
        if not score.get("scored", False):
            return
        if float(score.get("p95_error_px", 0.0) or 0.0) > self.reprojection_match_gate_px:
            return
        matched = score.get("matched_observations", [])
        if not isinstance(matched, list):
            return
        rigid_bucket = self._last_confirmed_marker_views_by_rigid.setdefault(
            candidate.rigid_name,
            {},
        )
        point_bucket = self._last_confirmed_marker_points_by_rigid.setdefault(
            candidate.rigid_name,
            {},
        )
        for payload in score.get("rigid_hint_marker_points_3d", []) or []:
            if not isinstance(payload, dict):
                continue
            try:
                marker_idx = int(payload.get("marker_idx", -1))
                point = np.asarray(payload.get("point"), dtype=np.float64).reshape(3)
            except (TypeError, ValueError):
                continue
            if (
                marker_idx < 0
                or marker_idx >= int(candidate.pattern.num_markers)
                or not np.isfinite(point).all()
            ):
                continue
            history = point_bucket.setdefault(marker_idx, [])
            if not history or int(history[-1].get("timestamp", -1)) != int(timestamp):
                history.append(
                    {
                        "timestamp": int(timestamp),
                        "point": [float(value) for value in point],
                    }
                )
                del history[:-2]
        for item in matched:
            if not isinstance(item, dict):
                continue
            try:
                marker_idx = int(item.get("marker_idx", -1))
                blob_index = int(item.get("blob_index", -1))
                observed_uv = np.asarray(item.get("observed_uv"), dtype=np.float64).reshape(2)
            except (TypeError, ValueError):
                continue
            if (
                marker_idx < 0
                or marker_idx >= int(candidate.pattern.num_markers)
                or blob_index < 0
                or not np.isfinite(observed_uv).all()
            ):
                continue
            camera_id = str(item.get("camera_id", ""))
            if not camera_id:
                continue
            rigid_bucket.setdefault(marker_idx, {})[camera_id] = {
                "timestamp": int(timestamp),
                "blob_index": int(blob_index),
                "observed_uv": [float(value) for value in observed_uv],
                "source": str(candidate.source or ""),
            }

    def _candidate_rank_score(
        self,
        *,
        pose: RigidBodyPose,
        score: Dict[str, Any],
        source: str,
        tracker: RigidBodyTracker,
    ) -> float:
        if not pose.valid:
            return -1.0
        score_value = float(score.get("score", 0.0) or 0.0) if isinstance(score, dict) else 0.0
        matched = int(score.get("matched_marker_views", 0) or 0) if isinstance(score, dict) else 0
        p95 = float(score.get("p95_error_px", 0.0) or 0.0) if isinstance(score, dict) else 0.0
        source_bonus = {
            "rigid_hint": 0.06,
            "generic": 0.0,
            "prediction_hold_2d_constrained": 0.0,
            "prediction_hold": -0.01,
            "position_continuity_guard": -0.02,
            "pose_continuity_guard": -0.08,
        }.get(str(source), 0.0)
        if len(self.patterns) <= 1:
            return float(score_value + 0.01 * matched + source_bonus - 0.01 * p95)
        prediction = tracker.peek_prediction(pose.timestamp)
        temporal_penalty = 0.0
        if prediction.valid:
            position_delta = float(np.linalg.norm(pose.position - prediction.position))
            rotation_delta = _quaternion_angle_deg(prediction.quaternion, pose.quaternion)
            rotation_cost = (rotation_delta / 45.0) ** 2
            position_cost = position_delta / 0.20
            temporal_penalty = min(1.0, 0.18 * rotation_cost + 0.12 * position_cost)
        return float(score_value + 0.01 * matched + source_bonus - 0.01 * p95 - temporal_penalty)

    def _resolve_frame_local_candidate_ownership(self, candidates: List[_PoseCandidate]) -> None:
        if len(self.patterns) <= 1:
            return
        ownership: Dict[Tuple[str, int], List[_PoseCandidate]] = {}
        for candidate in candidates:
            if candidate.invalid_reason or not candidate.pose.valid:
                continue
            for view in self._matched_blob_views(candidate.score):
                ownership.setdefault(view, []).append(candidate)

        contested: Dict[str, List[str]] = {}
        for view, owners in ownership.items():
            rigid_names = {owner.rigid_name for owner in owners}
            if len(rigid_names) <= 1:
                continue
            with self._variant_metric_lock:
                self._body_conflict_count += 1
                self._body_conflict_resolution_reasons["rank_score"] += 1
            ranked = sorted(
                owners,
                key=lambda item: (
                    float(item.rank_score),
                    int(item.pose.observed_markers),
                    int(item.score.get("matched_marker_views", 0) or 0),
                    -float(item.score.get("p95_error_px", 0.0) or 0.0),
                    item.rigid_name,
                ),
                reverse=True,
            )
            winner = ranked[0]
            for loser in ranked[1:]:
                if loser.rigid_name == winner.rigid_name:
                    continue
                contested.setdefault(loser.rigid_name, []).append(
                    f"{view[0]}:{view[1]}->{winner.rigid_name}"
                )

        for candidate in candidates:
            conflicts = contested.get(candidate.rigid_name, [])
            if conflicts:
                candidate.reject_reasons.append("frame_blob_ownership_conflict")
                candidate.invalid_reason = (
                    "frame_local_blob_ownership_conflict:" + ",".join(sorted(set(conflicts))[:4])
                )

    def _strict_reacquire_reject_reasons(
        self,
        candidate: _PoseCandidate,
    ) -> List[str]:
        tracker = candidate.tracker
        guarded_recovery = (
            tracker.mode == TrackMode.REACQUIRE
            or (
                tracker.mode in {TrackMode.LOST, TrackMode.BOOT}
                and int(tracker.track_count) > 0
                and int(tracker.lost_frames) > 0
            )
        )
        if not guarded_recovery or not candidate.pose.valid:
            return []
        if not bool(self.reacquire_guard_config.enforced):
            return []
        score = candidate.score if isinstance(candidate.score, dict) else {}
        min_markers = max(3, int(candidate.pattern.num_markers) - 1)
        min_views = max(
            int(self.reacquire_guard_config.min_matched_marker_views),
            min_markers * 2,
        )
        reasons: List[str] = []
        if candidate.pose.observed_markers < min_markers:
            reasons.append("insufficient_reacquire_markers")
        if not score.get("scored", False):
            reasons.append(f"score_not_available:{score.get('reason', 'unknown')}")
        if int(score.get("matched_marker_views", 0) or 0) < min_views:
            reasons.append("insufficient_reacquire_matched_views")
        if int(score.get("missing_marker_views", 0) or 0) > self.reacquire_guard_config.max_missing_marker_views:
            reasons.append("too_many_missing_marker_views")
        if float(score.get("mean_error_px", 0.0) or 0.0) > self.reacquire_guard_config.max_mean_reprojection_error_px:
            reasons.append("mean_reprojection_error_too_high")
        if float(score.get("p95_error_px", 0.0) or 0.0) > self.reacquire_guard_config.max_p95_reprojection_error_px:
            reasons.append("p95_reprojection_error_too_high")
        if int(score.get("duplicate_assignment_count", 0) or 0) > 0:
            reasons.append("duplicate_assignment")
        if candidate.source == "pose_continuity_guard":
            reasons.append("held_pose_cannot_reacquire")
        allowed, subset_reason = self._mode_subset_allowed(
            candidate.pattern,
            tracker.mode,
            score,
            prediction_valid=tracker.peek_prediction(candidate.pose.timestamp).valid,
        )
        if not allowed:
            reasons.append(f"reacquire_subset_not_allowed:{subset_reason}")
        return reasons

    def _apply_candidate_update(
        self,
        candidate: _PoseCandidate,
        timestamp: int,
    ) -> RigidBodyPose:
        tracker = candidate.tracker
        tracker.record_reprojection_score(candidate.score)
        tracker.record_rigid_hint_pose(candidate.hint_diagnostic)
        tracker.record_subset_hypothesis(candidate.subset_diagnostic)
        tracker.record_position_continuity_guard(candidate.position_guard)
        tracker.record_pose_continuity_guard(candidate.continuity_guard)
        tracker.record_reacquire_guard(candidate.reacquire_guard)

        if candidate.invalid_reason:
            conflict_hold = self._pose_from_ownership_conflict_prediction_hold(
                candidate,
                timestamp,
            )
            if conflict_hold is not None:
                tracker.update(
                    conflict_hold,
                    invalid_reason=candidate.invalid_reason,
                    prediction_hold=True,
                    pose_source="ownership_conflict_prediction_hold",
                )
                return conflict_hold
            rejected_pose = self._invalid_pose(timestamp)
            tracker.update(
                rejected_pose,
                invalid_reason=candidate.invalid_reason,
                pose_source="invalid",
            )
            return rejected_pose
        if not candidate.pose.valid:
            tracker.update(candidate.pose, pose_source=candidate.source or "invalid")
            return candidate.pose

        if candidate.source == "pose_continuity_guard" and int(candidate.pattern.num_markers) >= 5:
            rejected_pose = self._invalid_pose(timestamp)
            tracker.update(
                rejected_pose,
                invalid_reason="pose_continuity_hold_without_full_physical_observability",
                pose_source="invalid",
            )
            return rejected_pose

        if (
            candidate.source == "position_continuity_guard"
            and int(candidate.pattern.num_markers) >= 5
            and len(self._score_marker_indices(candidate.score)) < 3
        ):
            rejected_pose = self._invalid_pose(timestamp)
            tracker.update(
                rejected_pose,
                invalid_reason="insufficient_physical_evidence_for_guard_hold",
                pose_source="invalid",
            )
            return rejected_pose

        if candidate.source in {"prediction_hold", "prediction_hold_2d_constrained"}:
            tracker.update(
                candidate.pose,
                stage_reacquire=False,
                prediction_hold=True,
                pose_source=candidate.source,
            )
            return candidate.pose

        strict_reacquire_reasons = self._strict_reacquire_reject_reasons(candidate)
        if strict_reacquire_reasons:
            rejected_pose = self._invalid_pose(timestamp)
            self._record_candidate_rejection(
                candidate,
                "strict_reacquire_rejected:" + ",".join(strict_reacquire_reasons),
            )
            tracker.update(
                rejected_pose,
                invalid_reason="strict_reacquire_rejected:" + ",".join(strict_reacquire_reasons),
                pose_source="strict_reacquire_rejected",
            )
            return rejected_pose

        if tracker.mode == TrackMode.REACQUIRE:
            brief_reacquire_gap = int(tracker.lost_frames) <= 1
            ready, reason = tracker.prepare_reacquire_candidate(candidate.pose)
            if not ready and not (
                brief_reacquire_gap and reason == "reacquire_candidate_consistent"
            ):
                rejected_pose = self._invalid_pose(timestamp)
                self._record_candidate_rejection(
                    candidate,
                    "reacquire_pending_confirmation:" + reason,
                )
                tracker.record_non_committed_frame(
                    rejected_pose,
                    invalid_reason="reacquire_pending_confirmation:" + reason,
                    pose_source="reacquire_pending_confirmation",
                )
                return rejected_pose

        tracker.update(
            candidate.pose,
            stage_reacquire=False,
            prediction_hold=candidate.source in {
                "prediction_hold",
                "prediction_hold_2d_constrained",
                "pose_continuity_guard",
            },
            pose_source=candidate.source,
        )
        self._record_confirmed_2d_ownership(candidate, timestamp)
        return candidate.pose

    @staticmethod
    def _pose_from_ownership_conflict_prediction_hold(
        candidate: _PoseCandidate,
        timestamp: int,
    ) -> Optional[RigidBodyPose]:
        """Hold prediction instead of committing a pose that reused another rigid's blobs."""
        if not str(candidate.invalid_reason or "").startswith(
            "frame_local_blob_ownership_conflict:"
        ):
            return None
        tracker = candidate.tracker
        if int(tracker.track_count) <= 0:
            return None
        if (
            int(candidate.pattern.num_markers) >= 5
            and int(tracker.prediction_hold_frames)
            >= RigidBodyEstimator._PARTIAL_OCCLUSION_HOLD_MAX_FRAMES
        ):
            return None
        prediction = tracker.peek_prediction(timestamp)
        if not prediction.valid:
            return None
        with tracker._lock:
            held_quaternion = tracker._quaternion.copy()
        held_rotation = Rotation.from_quat(
            [
                held_quaternion[1],
                held_quaternion[2],
                held_quaternion[3],
                held_quaternion[0],
            ]
        ).as_matrix()
        return RigidBodyPose(
            timestamp=int(timestamp),
            position=prediction.position.copy(),
            rotation=held_rotation,
            quaternion=held_quaternion,
            rms_error=0.0,
            observed_markers=0,
            valid=True,
        )

    @staticmethod
    def _wxyz_from_rotation(rotation: np.ndarray) -> np.ndarray:
        quat_xyzw = Rotation.from_matrix(np.asarray(rotation, dtype=np.float64)).as_quat()
        return _normalize_quaternion(
            np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]], dtype=np.float64)
        )

    @staticmethod
    def _project_world_point_to_camera(
        point_world: np.ndarray,
        camera: Any,
        coordinate_space: str,
    ) -> Optional[np.ndarray]:
        point_world = np.asarray(point_world, dtype=np.float64).reshape(3)
        rotation = np.asarray(camera.rotation, dtype=np.float64).reshape(3, 3)
        translation = np.asarray(camera.translation, dtype=np.float64).reshape(3)
        point_cam = rotation @ point_world + translation
        if float(point_cam[2]) <= 1e-9:
            return None
        distortion = np.asarray(
            getattr(camera, "distortion_coeffs", np.zeros(5, dtype=np.float64)),
            dtype=np.float64,
        ).reshape(-1)
        if coordinate_space == "undistorted_pixel" or not np.any(np.abs(distortion) > 1e-12):
            fx = float(camera.intrinsic_matrix[0, 0])
            fy = float(camera.intrinsic_matrix[1, 1])
            cx = float(camera.intrinsic_matrix[0, 2])
            cy = float(camera.intrinsic_matrix[1, 2])
            return np.array(
                [
                    fx * float(point_cam[0]) / float(point_cam[2]) + cx,
                    fy * float(point_cam[1]) / float(point_cam[2]) + cy,
                ],
                dtype=np.float64,
            )
        rvec, _ = cv.Rodrigues(rotation)
        projected, _ = cv.projectPoints(
            point_world.reshape(1, 3).astype(np.float32),
            rvec,
            translation,
            camera.intrinsic_matrix,
            distortion,
        )
        return np.asarray(projected, dtype=np.float64).reshape(2)

    @staticmethod
    def _triangulate_same_marker_observations(
        observations: List[Dict[str, Any]],
        coordinate_space: str,
    ) -> Optional[np.ndarray]:
        centers: List[np.ndarray] = []
        directions: List[np.ndarray] = []
        for observation in observations:
            try:
                camera = observation["camera"]
                uv = np.asarray(observation["observed_uv"], dtype=np.float64).reshape(2)
                rotation = np.asarray(camera.rotation, dtype=np.float64).reshape(3, 3)
                translation = np.asarray(camera.translation, dtype=np.float64).reshape(3)
                intrinsic = np.asarray(camera.intrinsic_matrix, dtype=np.float64).reshape(3, 3)
                distortion = np.asarray(
                    getattr(camera, "distortion_coeffs", np.zeros(5)),
                    dtype=np.float64,
                ).reshape(-1)
            except (KeyError, TypeError, ValueError):
                continue
            if not np.isfinite(uv).all():
                continue
            if coordinate_space == "undistorted_pixel" or not np.any(np.abs(distortion) > 1e-12):
                fx = float(intrinsic[0, 0])
                fy = float(intrinsic[1, 1])
                cx = float(intrinsic[0, 2])
                cy = float(intrinsic[1, 2])
                if abs(fx) <= 1e-9 or abs(fy) <= 1e-9:
                    continue
                normalized = np.array([(uv[0] - cx) / fx, (uv[1] - cy) / fy], dtype=np.float64)
            else:
                normalized_points = cv.undistortPoints(
                    uv.reshape(1, 1, 2).astype(np.float64),
                    intrinsic,
                    distortion,
                )
                normalized = np.asarray(normalized_points, dtype=np.float64).reshape(2)
            direction_cam = np.array([normalized[0], normalized[1], 1.0], dtype=np.float64)
            norm = float(np.linalg.norm(direction_cam))
            if norm <= 1e-12:
                continue
            direction_cam /= norm
            center_world = -rotation.T @ translation
            direction_world = rotation.T @ direction_cam
            direction_world_norm = float(np.linalg.norm(direction_world))
            if direction_world_norm <= 1e-12:
                continue
            centers.append(center_world)
            directions.append(direction_world / direction_world_norm)
        if len(centers) < 2:
            return None
        lhs = np.zeros((3, 3), dtype=np.float64)
        rhs = np.zeros(3, dtype=np.float64)
        identity = np.eye(3, dtype=np.float64)
        for center, direction in zip(centers, directions):
            projector = identity - np.outer(direction, direction)
            lhs += projector
            rhs += projector @ center
        try:
            point = np.linalg.solve(lhs + np.eye(3, dtype=np.float64) * 1e-9, rhs)
        except np.linalg.LinAlgError:
            return None
        if not np.isfinite(point).all():
            return None
        return point

    @staticmethod
    def _active_anchor_observations_for_pattern(
        pattern: MarkerPattern,
        object_gating: Dict[str, Any],
        camera_params: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        if not object_gating.get("evaluated"):
            return []
        observations: List[Dict[str, Any]] = []
        for item in object_gating.get("active_anchor_observations", []):
            if not isinstance(item, dict):
                continue
            try:
                camera_id = str(item.get("camera_id", ""))
                camera = camera_params.get(camera_id) or camera_params.get(item.get("camera_id"))
                marker_idx = int(item.get("marker_idx", -1))
                blob_index = int(item.get("blob_index", -1))
                observed_uv = np.asarray(item.get("observed_uv"), dtype=np.float64).reshape(2)
                uncertainty = float(item.get("uncertainty_px", 1.0) or 1.0)
            except (TypeError, ValueError):
                continue
            if camera is None or marker_idx < 0 or marker_idx >= pattern.num_markers:
                continue
            if not np.isfinite(observed_uv).all():
                continue
            observations.append(
                {
                    "camera_id": camera_id,
                    "camera": camera,
                    "marker_idx": marker_idx,
                    "blob_index": blob_index,
                    "observed_uv": observed_uv,
                    "uncertainty_px": max(1e-6, uncertainty),
                }
            )
        if observations:
            return observations

        per_camera = object_gating.get("per_camera")
        if not isinstance(per_camera, dict):
            return []
        for camera_id, camera_payload in per_camera.items():
            camera = camera_params.get(str(camera_id)) or camera_params.get(camera_id)
            if camera is None or not isinstance(camera_payload, dict):
                continue
            for assignment in camera_payload.get("assignments", []):
                if not isinstance(assignment, dict):
                    continue
                try:
                    marker_idx = int(assignment.get("marker_idx", -1))
                    blob_index = int(assignment.get("blob_index", -1))
                    observed_uv = np.asarray(assignment.get("observed_uv"), dtype=np.float64).reshape(2)
                    uncertainty = float(assignment.get("uncertainty_px", 1.0) or 1.0)
                except (TypeError, ValueError):
                    continue
                if marker_idx < 0 or marker_idx >= pattern.num_markers:
                    continue
                if not np.isfinite(observed_uv).all():
                    continue
                observations.append(
                    {
                        "camera_id": str(camera_id),
                        "camera": camera,
                        "marker_idx": marker_idx,
                        "blob_index": blob_index,
                        "observed_uv": observed_uv,
                        "uncertainty_px": max(1e-6, uncertainty),
                    }
                )
        return observations

    def _pose_from_2d_evidence_prediction_hold(
        self,
        pattern: MarkerPattern,
        prediction: PredictedPose,
        object_gating: Dict[str, Any],
        camera_params: Optional[Dict[str, Any]],
        coordinate_space: str,
        timestamp: int,
    ) -> Optional[Tuple[RigidBodyPose, Dict[str, Any]]]:
        """Constrain a prediction hold by assigned 2D evidence without creating a new boot."""
        if (
            len(self.patterns) <= 1
            or int(pattern.num_markers) < 5
            or not prediction.valid
            or not camera_params
            or not object_gating.get("evaluated")
        ):
            return None
        per_camera = object_gating.get("per_camera")
        if not isinstance(per_camera, dict):
            return None

        observations = self._active_anchor_observations_for_pattern(
            pattern,
            object_gating,
            camera_params,
        )
        if not observations:
            return None

        base_position = prediction.position.copy()
        base_rotation = prediction.rotation.copy()
        position_sigma = max(
            float(prediction.position_sigma_m or 0.0),
            float(pattern.marker_diameter) * max(1.0, np.sqrt(len(observations))),
        )

        def residuals(translation_delta: np.ndarray) -> np.ndarray:
            position = base_position + np.asarray(translation_delta, dtype=np.float64).reshape(3)
            values: List[float] = []
            for observation in observations:
                marker_world = (
                    base_rotation @ pattern.marker_positions[int(observation["marker_idx"])]
                    + position
                )
                projected_uv = self._project_world_point_to_camera(
                    marker_world,
                    observation["camera"],
                    coordinate_space,
                )
                if projected_uv is None or not np.isfinite(projected_uv).all():
                    values.extend([1e3, 1e3])
                    continue
                values.extend(
                    (
                        (projected_uv - observation["observed_uv"])
                        / float(observation["uncertainty_px"])
                    ).tolist()
                )
            values.extend(
                (np.asarray(translation_delta, dtype=np.float64).reshape(3) / position_sigma).tolist()
            )
            return np.asarray(values, dtype=np.float64)

        baseline_delta = np.zeros(3, dtype=np.float64)
        baseline_residual = residuals(baseline_delta)
        baseline_cost = float(np.dot(baseline_residual, baseline_residual))
        solved_delta = baseline_delta.copy()
        by_marker: Dict[int, List[Dict[str, Any]]] = {}
        for observation in observations:
            by_marker.setdefault(int(observation["marker_idx"]), []).append(observation)
        anchored_delta: Optional[np.ndarray] = None
        anchored_marker_idx: Optional[int] = None
        for marker_idx, marker_observations in sorted(
            by_marker.items(),
            key=lambda item: len(item[1]),
            reverse=True,
        ):
            if len({str(item["camera_id"]) for item in marker_observations}) < 2:
                continue
            anchor_point = self._triangulate_same_marker_observations(
                marker_observations,
                coordinate_space,
            )
            if anchor_point is None:
                continue
            marker_offset = base_rotation @ pattern.marker_positions[int(marker_idx)]
            anchored_delta = anchor_point - (base_position + marker_offset)
            anchored_marker_idx = int(marker_idx)
            break
        if anchored_delta is not None and np.isfinite(anchored_delta).all():
            solved_delta = anchored_delta.reshape(3)
        else:
            step = max(1e-5, float(pattern.marker_diameter) * 0.01)
            for _ in range(1):
                current_residual = residuals(solved_delta)
                jacobian = np.zeros((len(current_residual), 3), dtype=np.float64)
                for axis in range(3):
                    probe = solved_delta.copy()
                    probe[axis] += step
                    jacobian[:, axis] = (residuals(probe) - current_residual) / step
                lhs = jacobian.T @ jacobian + np.eye(3, dtype=np.float64) * 1e-6
                rhs = -(jacobian.T @ current_residual)
                try:
                    update = np.linalg.solve(lhs, rhs)
                except np.linalg.LinAlgError:
                    break
                if not np.isfinite(update).all():
                    break
                solved_delta = solved_delta + update
                if float(np.linalg.norm(update)) <= step:
                    break
        solved_residual = residuals(solved_delta)
        solved_cost = float(np.dot(solved_residual, solved_residual))
        if not np.isfinite(solved_cost) or solved_cost >= baseline_cost:
            return None

        position = base_position + solved_delta
        rotation = base_rotation.copy()
        if not np.isfinite(position).all():
            return None
        observed_markers = sorted({int(item["marker_idx"]) for item in observations})
        reprojection_errors: List[float] = []
        normalized_errors: List[float] = []
        matched_observations: List[Dict[str, Any]] = []
        for observation in observations:
            marker_world = rotation @ pattern.marker_positions[int(observation["marker_idx"])] + position
            projected_uv = self._project_world_point_to_camera(
                marker_world,
                observation["camera"],
                coordinate_space,
            )
            if projected_uv is None:
                continue
            error = float(np.linalg.norm(projected_uv - observation["observed_uv"]))
            normalized_error = float(error / float(observation["uncertainty_px"]))
            reprojection_errors.append(error)
            normalized_errors.append(normalized_error)
            matched_observations.append(
                {
                    "camera_id": str(observation["camera_id"]),
                    "marker_idx": int(observation["marker_idx"]),
                    "blob_index": int(observation["blob_index"]),
                    "error_px": error,
                    "normalized_error": normalized_error,
                    "uncertainty_px": float(observation["uncertainty_px"]),
                }
            )
        mean_error, p95_error, max_error = self._residual_error_stats(reprojection_errors)
        mean_normalized, p95_normalized, max_normalized = self._residual_error_stats(normalized_errors)
        matched = int(len(matched_observations))
        camera_count = int(len({item["camera_id"] for item in matched_observations}))
        expected = int(pattern.num_markers) * max(1, int(len(camera_params)))
        coverage = float(matched / expected) if expected > 0 else 0.0
        residual_score = (
            1.0 / (1.0 + mean_normalized / 5.0)
            if normalized_errors
            else 0.0
        )
        score = {
            "scored": True,
            "reason": "partial_2d_prediction_hold",
            "coordinate_space": coordinate_space,
            "score": float(np.clip(0.70 * residual_score + 0.30 * coverage, 0.0, 1.0)),
            "mean_error_px": float(mean_error),
            "p95_error_px": float(p95_error),
            "max_error_px": float(max_error),
            "mean_normalized_error": float(mean_normalized),
            "p95_normalized_error": float(p95_normalized),
            "max_normalized_error": float(max_normalized),
            "matched_marker_views": matched,
            "expected_marker_views": int(expected),
            "missing_marker_views": int(max(0, expected - matched)),
            "duplicate_assignment_count": 0,
            "unexpected_blob_count": 0,
            "camera_count": camera_count,
            "matched_observations": matched_observations,
            "rigid_hint_marker_indices": [int(index) for index in observed_markers],
            "rigid_hint_real_ray_count": matched,
            "prediction_hold_2d_constrained": True,
            "prediction_hold_2d_cost_before": float(baseline_cost),
            "prediction_hold_2d_cost_after": float(solved_cost),
            "prediction_hold_2d_observations": matched,
            "prediction_hold_anchor_triangulated": bool(anchored_delta is not None),
            "prediction_hold_anchor_marker_idx": anchored_marker_idx,
        }
        pose = RigidBodyPose(
            timestamp=int(timestamp),
            position=position,
            rotation=rotation,
            quaternion=self._wxyz_from_rotation(rotation),
            rms_error=0.0,
            observed_markers=int(len(observed_markers)),
            valid=True,
        )
        return pose, score

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
            self._record_rigid_hint_visibility({})
            for tracker in self.trackers.values():
                self._record_mode_frame(tracker.pattern.name, tracker.mode)
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

        clusters: Optional[List[np.ndarray]] = None

        def get_clusters() -> List[np.ndarray]:
            nonlocal clusters
            if clusters is None:
                clusters = self.clusterer.cluster(points_3d)
                self._cluster_call_count += 1
                self._cluster_input_point_total += int(len(points_3d))
                self._cluster_output_count_total += int(len(clusters))
                self._cluster_output_count_max = max(
                    int(self._cluster_output_count_max),
                    int(len(clusters)),
                )
                if len(self.patterns) == 1 and len(points_3d) >= 3:
                    min_points = max(3, self.patterns[0].num_markers - 1)
                    if not any(len(cluster) >= min_points for cluster in clusters):
                        clusters.append(points_3d)
            return clusters

        candidates: List[_PoseCandidate] = []
        used_clusters: set[int] = set()
        hint_markers_by_rigid: Dict[str, Dict[int, Dict[str, Any]]] = {}
        if has_rigid_hints:
            hint_markers_by_rigid = self._rigid_hint_markers_by_rigid(
                rigid_hint_triangulated_points,
            )
        self._record_rigid_hint_visibility(hint_markers_by_rigid)

        for pattern in self.patterns:
            best_pose = None
            best_error = float('inf')
            best_cluster_idx = -1
            best_score_from_separated: Optional[Dict[str, Any]] = None
            tracker = self.trackers[pattern.name]
            self._record_mode_frame(pattern.name, tracker.mode)
            skip_unowned_boot_generic = False
            skip_unseen_tracked_generic = False
            prediction_reason = "no_valid_pose"

            if self.rigid_candidate_separation_enabled and has_rigid_hints:
                separated = self._try_rigid_separated_candidate(
                    pattern,
                    tracker,
                    timestamp,
                    points_3d,
                    rigid_hint_triangulated_points,
                    hint_markers_by_rigid.get(pattern.name),
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                )
                if separated is not None:
                    best_pose, best_score_from_separated = separated
                    best_error = float(best_pose.rms_error)
                    best_cluster_idx = -1
                elif (
                    object_gating_enforced
                    and tracker.mode == TrackMode.BOOT
                    and int(tracker.track_count) == 0
                    and any(
                        other is not tracker and int(other.track_count) > 0
                        for other in self.trackers.values()
                    )
                    and len(hint_markers_by_rigid.get(pattern.name, {}))
                    < max(3, pattern.num_markers - 1)
                ):
                    skip_unowned_boot_generic = True

            if self._should_hold_unseen_tracked_rigid(
                pattern,
                tracker,
                timestamp,
                hint_markers_by_rigid,
            ):
                skip_unseen_tracked_generic = True
                prediction_reason = "tracked_rigid_unseen_hold_prediction"
                self._record_rigid_candidate_fallback(prediction_reason)

            if self._should_hold_partial_marker_tracked_rigid(
                pattern,
                tracker,
                timestamp,
                hint_markers_by_rigid,
            ):
                skip_unseen_tracked_generic = True
                prediction_reason = "tracked_rigid_partial_marker_hold_prediction"
                self._record_rigid_candidate_fallback(prediction_reason)

            gating = tracker.latest_object_gating()
            (
                gating_evaluated,
                _gating_marker_count,
                gating_camera_count,
                gating_assigned_marker_views,
            ) = tracker.latest_object_gating_counts()
            (
                gating_markers_with_two_or_more_rays,
                gating_ambiguous_assignment_count,
                gating_marker_margin_assignment_count,
                gating_duplicate_assignment_count,
            ) = tracker.latest_object_gating_skip_counts()
            if (
                not skip_unseen_tracked_generic
                and object_gating_enforced
                and int(pattern.num_markers) >= 5
                and int(tracker.track_count) > 0
                and len(hint_markers_by_rigid.get(pattern.name, {})) < 3
                and self._active_anchor_observation_count(gating) > 0
            ):
                skip_unseen_tracked_generic = True
                prediction_reason = "anchor_only_no_6dof_recovery"
                self._record_rigid_candidate_fallback(prediction_reason)
            if (
                not skip_unseen_tracked_generic
                and best_pose is None
                and self._should_skip_generic_search_for_object_gated_continue(
                    pattern,
                    tracker,
                    object_gating_enforced=object_gating_enforced,
                    hint_marker_count=len(hint_markers_by_rigid.get(pattern.name, {})),
                    gating_evaluated=gating_evaluated,
                    gating_camera_count=gating_camera_count,
                    gating_assigned_marker_views=gating_assigned_marker_views,
                    gating_markers_with_two_or_more_rays=(
                        gating_markers_with_two_or_more_rays
                    ),
                    gating_ambiguous_assignment_count=(
                        gating_ambiguous_assignment_count
                    ),
                    gating_marker_margin_assignment_count=(
                        gating_marker_margin_assignment_count
                    ),
                    gating_duplicate_assignment_count=gating_duplicate_assignment_count,
                )
            ):
                skip_unseen_tracked_generic = True
                prediction_reason = "object_gated_continue_skip_generic"
                self._record_rigid_candidate_fallback(prediction_reason)

            if (
                best_pose is None
                and not skip_unowned_boot_generic
                and not skip_unseen_tracked_generic
            ):
                for idx, cluster in enumerate(get_clusters()):
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

            if (
                best_pose is None
                and not skip_unowned_boot_generic
                and not skip_unseen_tracked_generic
            ):
                boot_pose = self._try_multi_pattern_boot_candidate(
                    pattern,
                    points_3d,
                    timestamp,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                )
                if boot_pose is not None:
                    best_pose = boot_pose
                    best_error = float(boot_pose.rms_error)
                    best_cluster_idx = -1

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
                selected_pose = best_pose
                selected_score = score
                selected_source = "generic"
                if (
                    best_score_from_separated is not None
                    and object_gating_enforced
                    and self.subset_diagnostics_mode == "off"
                ):
                    selected_source = "rigid_hint"
                    hint_diagnostic = {
                        "evaluated": True,
                        "reason": "ok",
                        "valid": bool(best_pose.valid),
                        "diagnostics_only": False,
                        "enforced": True,
                        "selected_for_pose": True,
                        "selection_reason": "object_gating_enforced",
                        "candidate_points": int(best_pose.observed_markers),
                        "observed_markers": int(best_pose.observed_markers),
                        "marker_indices": [
                            int(index)
                            for index in score.get("rigid_hint_marker_indices", [])
                        ],
                        "real_ray_count": int(score.get("rigid_hint_real_ray_count", 0)),
                        "rms_error_m": float(best_pose.rms_error),
                        "generic_rms_error_m": float(best_pose.rms_error),
                        "generic_valid": bool(best_pose.valid),
                        "generic_score": dict(score),
                        "score": dict(score),
                        "score_delta": 0.0,
                        "position_delta_m": 0.0,
                        "rotation_delta_deg": 0.0,
                        "virtual_marker_count": 0,
                        "invalid_points": 0,
                        "would_improve_score": False,
                        "pose": best_pose.to_dict(),
                    }
                else:
                    hint_diagnostic = self._evaluate_rigid_hint_pose(
                        pattern,
                        timestamp,
                        rigid_hint_triangulated_points,
                        camera_params,
                        observations_by_camera,
                        coordinate_space=coordinate_space,
                        generic_pose=best_pose,
                        generic_score=score,
                        generic_is_rigid_hint=best_score_from_separated is not None,
                    )
                    enforce_hint = self._should_select_rigid_hint_pose(
                        tracker,
                        hint_diagnostic,
                    )
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
                position_guard = self._evaluate_position_continuity_guard(
                    tracker,
                    pattern,
                    selected_pose,
                    selected_score,
                )
                if (
                    position_guard.get("enforced")
                    and position_guard.get("would_reject")
                ):
                    clamped_pose = self._pose_with_position_accel_limit(
                        position_guard,
                        source_pose=selected_pose,
                    )
                    if clamped_pose is not None:
                        selected_pose = clamped_pose
                        selected_source = "position_continuity_guard"
                        position_guard["clamped_position"] = True

                continuity_guard = self._evaluate_pose_continuity_guard(
                    tracker,
                    pattern,
                    selected_pose,
                    selected_score,
                )
                if (
                    continuity_guard.get("enforced")
                    and continuity_guard.get("would_reject")
                    and self.pose_continuity_guard_config.hold_prediction_on_reject
                ):
                    held_pose = self._pose_with_continuity_rotation_hold(
                        continuity_guard.get("prediction", {}),
                        source_pose=selected_pose,
                    )
                    if held_pose is not None:
                        selected_pose = held_pose
                        selected_source = "pose_continuity_guard"
                        continuity_guard["held_prediction"] = True
                        continuity_guard["held_rotation"] = True

                guard = self._evaluate_reacquire_guard(tracker, selected_pose, selected_score)

                invalid_reason = ""
                boot_guard_reasons = self._multi_rigid_boot_reject_reasons(
                    pattern,
                    tracker,
                    selected_source=selected_source,
                    selected_score=selected_score,
                )
                generic_guard_reasons = self._generic_continue_reject_reasons(
                    tracker,
                    selected_source=selected_source,
                    selected_score=selected_score,
                    hint_diagnostic=hint_diagnostic,
                )
                if boot_guard_reasons:
                    invalid_reason = (
                        "multi_rigid_boot_reprojection_guard_rejected:"
                        + ",".join(boot_guard_reasons)
                    )
                elif generic_guard_reasons:
                    invalid_reason = (
                        "generic_continue_reprojection_guard_rejected:"
                        + ",".join(generic_guard_reasons)
                    )
                elif (
                    position_guard.get("enforced")
                    and position_guard.get("would_reject")
                    and not position_guard.get("clamped_position")
                ):
                    invalid_reason = "position_continuity_guard_rejected"
                elif (
                    continuity_guard.get("enforced")
                    and continuity_guard.get("would_reject")
                    and not continuity_guard.get("held_prediction")
                ):
                    invalid_reason = "pose_continuity_guard_rejected"
                elif guard.get("enforced") and guard.get("would_reject"):
                    invalid_reason = "reprojection_guard_rejected"

                candidate = _PoseCandidate(
                    rigid_name=pattern.name,
                    pattern=pattern,
                    tracker=tracker,
                    pose=selected_pose,
                    score=selected_score,
                    source=selected_source,
                    hint_diagnostic=hint_diagnostic,
                    subset_diagnostic=subset_diagnostic,
                    position_guard=position_guard,
                    continuity_guard=continuity_guard,
                    reacquire_guard=guard,
                    rank_score=self._candidate_rank_score(
                        pose=selected_pose,
                        score=selected_score,
                        source=selected_source,
                        tracker=tracker,
                    ),
                    best_cluster_idx=best_cluster_idx,
                    invalid_reason=invalid_reason,
                )
                candidates.append(candidate)
                if selected_source == "generic" and best_cluster_idx >= 0:
                    used_clusters.add(best_cluster_idx)
            else:
                prediction = tracker.peek_prediction(timestamp)
                predicted = (
                    prediction.to_pose()
                    if prediction.valid
                    else tracker.predict()
                )
                predicted.timestamp = timestamp
                score = _empty_reprojection_score(prediction_reason)
                partial_hold_reasons = {
                    "tracked_rigid_partial_marker_hold_prediction",
                    "tracked_rigid_unseen_hold_prediction",
                    "object_gated_continue_skip_generic",
                }
                if prediction_reason in partial_hold_reasons and prediction.valid:
                    predicted = RigidBodyPose(
                        timestamp=int(timestamp),
                        position=prediction.position.copy(),
                        rotation=prediction.rotation.copy(),
                        quaternion=prediction.quaternion.copy(),
                        rms_error=0.0,
                        observed_markers=int(len(hint_markers_by_rigid.get(pattern.name, {}))),
                        valid=True,
                    )
                    camera_count = max(
                        1,
                        int(len(camera_params) if isinstance(camera_params, dict) else 0),
                    )
                    expected_marker_views = int(pattern.num_markers) * camera_count
                    hint_views = int(
                        sum(
                            max(1, int(marker.get("contributing_rays", 0) or 0))
                            for marker in hint_markers_by_rigid.get(pattern.name, {}).values()
                        )
                    )
                    score.update(
                        {
                            "matched_marker_views": hint_views,
                            "expected_marker_views": expected_marker_views,
                            "missing_marker_views": max(0, expected_marker_views - hint_views),
                        }
                    )
                    if prediction_reason in partial_hold_reasons:
                        constrained_hold = self._pose_from_2d_evidence_prediction_hold(
                            pattern,
                            prediction,
                            tracker.latest_object_gating(),
                            camera_params,
                            coordinate_space,
                            timestamp,
                        )
                        if constrained_hold is not None:
                            predicted, constrained_score = constrained_hold
                            score.update(constrained_score)
                hint_diagnostic = self._evaluate_rigid_hint_pose(
                    pattern,
                    timestamp,
                    rigid_hint_triangulated_points,
                    camera_params,
                    observations_by_camera,
                    coordinate_space=coordinate_space,
                    generic_pose=None,
                    generic_score=score,
                    generic_is_rigid_hint=False,
                )
                selected_pose = predicted
                selected_source = (
                    "prediction_hold_2d_constrained"
                    if selected_pose.valid and score.get("prediction_hold_2d_constrained")
                    else "prediction_hold"
                    if selected_pose.valid and prediction_reason in partial_hold_reasons
                    else "prediction"
                )
                if (
                    selected_source == "prediction"
                    and prediction_reason == "no_valid_pose"
                    and object_gating_enforced
                    and len(self.patterns) > 1
                    and int(tracker.track_count) > 0
                ):
                    prediction_reason = "no_visible_rigid_evidence"
                    selected_pose = self._invalid_pose(timestamp)
                    score = _empty_reprojection_score(prediction_reason)
                    selected_source = "invalid"
                if self._should_select_rigid_hint_pose(tracker, hint_diagnostic):
                    hint_pose = self._pose_from_payload(
                        hint_diagnostic.get("pose"),
                        fallback_timestamp=timestamp,
                    )
                    if hint_pose is not None and hint_pose.valid:
                        selected_pose = hint_pose
                        score = dict(hint_diagnostic.get("score") or _empty_reprojection_score())
                        selected_source = "rigid_hint"
                        hint_diagnostic["selected_for_pose"] = True
                        hint_diagnostic["selection_reason"] = "object_gating_enforced"
                    else:
                        hint_diagnostic["selected_for_pose"] = False
                        hint_diagnostic["selection_reason"] = "invalid_pose_payload"
                elif object_gating_enforced:
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
                    generic_pose=selected_pose if selected_pose.valid else None,
                    generic_score=score,
                )
                if selected_pose.valid:
                    position_guard = self._evaluate_position_continuity_guard(
                        tracker,
                        pattern,
                        selected_pose,
                        score,
                    )
                    if (
                        position_guard.get("enforced")
                        and position_guard.get("would_reject")
                    ):
                        clamped_pose = self._pose_with_position_accel_limit(
                            position_guard,
                            source_pose=selected_pose,
                        )
                        if clamped_pose is not None:
                            selected_pose = clamped_pose
                            selected_source = "position_continuity_guard"
                            position_guard["clamped_position"] = True

                    continuity_guard = self._evaluate_pose_continuity_guard(
                        tracker,
                        pattern,
                        selected_pose,
                        score,
                    )
                    if (
                        continuity_guard.get("enforced")
                        and continuity_guard.get("would_reject")
                        and self.pose_continuity_guard_config.hold_prediction_on_reject
                    ):
                        held_pose = self._pose_with_continuity_rotation_hold(
                            continuity_guard.get("prediction", {}),
                            source_pose=selected_pose,
                        )
                        if held_pose is not None:
                            selected_pose = held_pose
                            selected_source = "pose_continuity_guard"
                            continuity_guard["held_prediction"] = True
                            continuity_guard["held_rotation"] = True

                    guard = self._evaluate_reacquire_guard(tracker, selected_pose, score)
                else:
                    continuity_guard = _empty_pose_continuity_guard(
                        enabled=self.pose_continuity_guard_config.enabled,
                        enforced=self.pose_continuity_guard_config.enforced,
                        reason=prediction_reason,
                        thresholds=self.pose_continuity_guard_config.thresholds_dict(),
                    )
                    position_guard = _empty_position_continuity_guard(
                        enabled=self.position_continuity_guard_config.enabled,
                        enforced=self.position_continuity_guard_config.enforced,
                        reason=prediction_reason,
                        thresholds=self.position_continuity_guard_config.thresholds_dict(),
                    )
                    guard = _empty_reacquire_guard(
                        enabled=self.reacquire_guard_config.shadow_enabled,
                        enforced=self.reacquire_guard_config.enforced,
                        reason=prediction_reason,
                        thresholds=self.reacquire_guard_config.thresholds_dict(),
                    )

                invalid_reason = ""
                if (
                    position_guard.get("enforced")
                    and position_guard.get("would_reject")
                    and not position_guard.get("clamped_position")
                ):
                    invalid_reason = "position_continuity_guard_rejected"
                elif (
                    continuity_guard.get("enforced")
                    and continuity_guard.get("would_reject")
                    and not continuity_guard.get("held_prediction")
                ):
                    invalid_reason = "pose_continuity_guard_rejected"
                elif guard.get("enforced") and guard.get("would_reject"):
                    invalid_reason = "reprojection_guard_rejected"
                elif not selected_pose.valid and prediction_reason != "no_valid_pose":
                    invalid_reason = prediction_reason

                candidates.append(
                    _PoseCandidate(
                        rigid_name=pattern.name,
                        pattern=pattern,
                        tracker=tracker,
                        pose=selected_pose,
                        score=score,
                        source=selected_source,
                        hint_diagnostic=hint_diagnostic,
                        subset_diagnostic=subset_diagnostic,
                        position_guard=position_guard,
                        continuity_guard=continuity_guard,
                        reacquire_guard=guard,
                        rank_score=self._candidate_rank_score(
                            pose=selected_pose,
                            score=score,
                            source=selected_source,
                            tracker=tracker,
                        ),
                        invalid_reason=invalid_reason,
                    )
                )

        self._resolve_frame_local_candidate_ownership(candidates)

        poses: Dict[str, RigidBodyPose] = {}
        for candidate in candidates:
            self._record_pose_candidate(candidate)
            poses[candidate.rigid_name] = self._apply_candidate_update(candidate, timestamp)

        return poses

    def _generic_continue_reject_reasons(
        self,
        tracker: RigidBodyTracker,
        *,
        selected_source: str,
        selected_score: Dict[str, Any],
        hint_diagnostic: Dict[str, Any],
    ) -> List[str]:
        """Return reasons to reject a generic continuation pose that object gating cannot confirm."""
        if (
            not self.object_gating_config.enabled
            or not self.object_gating_config.enforce
            or tracker.mode != TrackMode.CONTINUE
            or selected_source not in {
                "generic",
                "position_continuity_guard",
                "pose_continuity_guard",
            }
        ):
            return []
        if bool(hint_diagnostic.get("selected_for_pose", False)):
            return []

        score = selected_score if isinstance(selected_score, dict) else {}
        config = self.reacquire_guard_config
        reasons: List[str] = []
        if self._other_object_gate_match_count(tracker, score) > 0:
            reasons.append("matched_other_rigid_gate_blobs")
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
        allowed, subset_reason = self._mode_subset_allowed(
            tracker.pattern,
            tracker.mode,
            score,
            prediction_valid=tracker.peek_prediction(
                int(score.get("timestamp", 0) or 0)
            ).valid,
        )
        if not allowed:
            reasons.append(f"subset_not_allowed:{subset_reason}")
        return reasons

    def _multi_rigid_boot_reject_reasons(
        self,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        *,
        selected_source: str,
        selected_score: Dict[str, Any],
    ) -> List[str]:
        """Reject multi-rigid boot poses that fit in 3D but do not explain the camera blobs."""
        if (
            len(self.patterns) <= 1
            or not self.object_gating_config.enabled
            or not self.object_gating_config.enforce
            or tracker.mode != TrackMode.BOOT
            or int(tracker.track_count) > 0
            or selected_source != "generic"
        ):
            return []
        score = selected_score if isinstance(selected_score, dict) else {}
        reasons: List[str] = []
        min_views = max(6, int(pattern.num_markers) * 2)
        if not score.get("scored", False):
            reasons.append(f"score_not_available:{score.get('reason', 'unknown')}")
            return reasons
        if int(score.get("matched_marker_views", 0) or 0) < min_views:
            reasons.append("insufficient_boot_matched_views")
        marker_coverage = len(self._score_marker_indices(score))
        if (
            int(score.get("missing_marker_views", 0) or 0)
            > self._MULTI_PATTERN_BOOT_MAX_MISSING_MARKER_VIEWS
            and marker_coverage < int(pattern.num_markers)
        ):
            reasons.append("too_many_boot_missing_marker_views")
        if (
            float(score.get("mean_error_px", 0.0) or 0.0)
            > self._MULTI_PATTERN_BOOT_MAX_MEAN_REPROJECTION_PX
        ):
            reasons.append("boot_mean_reprojection_error_too_high")
        if (
            float(score.get("p95_error_px", 0.0) or 0.0)
            > self._MULTI_PATTERN_BOOT_MAX_P95_REPROJECTION_PX
        ):
            reasons.append("boot_p95_reprojection_error_too_high")
        if int(score.get("duplicate_assignment_count", 0) or 0) > 0:
            reasons.append("boot_duplicate_assignment")
        allowed, subset_reason = self._mode_subset_allowed(
            pattern,
            TrackMode.BOOT,
            score,
            prediction_valid=False,
        )
        if not allowed:
            reasons.append(f"boot_subset_not_allowed:{subset_reason}")
        return reasons

    @staticmethod
    def _tracking_policy(pattern: MarkerPattern) -> Dict[str, Any]:
        policy = (pattern.metadata or {}).get("tracking_policy", {})
        return dict(policy) if isinstance(policy, dict) else {}

    @staticmethod
    def _normalized_subset_set(values: Any) -> set[Tuple[int, ...]]:
        result: set[Tuple[int, ...]] = set()
        if not isinstance(values, list):
            return result
        for item in values:
            try:
                subset = tuple(sorted(int(value) for value in item))
            except (TypeError, ValueError):
                continue
            if subset:
                result.add(subset)
        return result

    @staticmethod
    def _score_marker_indices(score: Dict[str, Any]) -> Tuple[int, ...]:
        if not isinstance(score, dict):
            return tuple()
        explicit = score.get("rigid_hint_marker_indices")
        if isinstance(explicit, list) and explicit:
            try:
                return tuple(sorted({int(value) for value in explicit}))
            except (TypeError, ValueError):
                pass
        counts: Counter[int] = Counter()
        for item in score.get("matched_observations", []) or []:
            if not isinstance(item, dict):
                continue
            try:
                counts[int(item.get("marker_idx"))] += 1
            except (TypeError, ValueError):
                continue
        return tuple(sorted(index for index, count in counts.items() if int(count) >= 2))

    def _mode_subset_allowed(
        self,
        pattern: MarkerPattern,
        mode: TrackMode,
        score: Dict[str, Any],
        *,
        prediction_valid: bool,
    ) -> Tuple[bool, str]:
        policy = self._tracking_policy(pattern)
        if not policy or int(pattern.num_markers) < 5:
            return True, "no_policy"
        marker_indices = self._score_marker_indices(score)
        visible_count = len(marker_indices)
        if visible_count >= int(pattern.num_markers):
            return True, "full_pattern"

        strong4 = self._normalized_subset_set(policy.get("strong_4_subsets"))
        strong3 = self._normalized_subset_set(policy.get("strong_3_subsets"))
        subset = tuple(sorted(marker_indices))
        if mode in {TrackMode.BOOT, TrackMode.REACQUIRE, TrackMode.LOST}:
            min_markers = int(
                policy.get(
                    "reacquire_min_markers" if mode != TrackMode.BOOT else "boot_min_markers",
                    4,
                )
                or 4
            )
            if visible_count < min_markers:
                return False, f"visible_{visible_count}_below_min_{min_markers}"
            if visible_count == 4 and strong4 and subset not in strong4:
                return False, "four_subset_not_whitelisted"
            return True, "strong_recovery_subset"

        if mode == TrackMode.CONTINUE:
            min_markers = int(policy.get("continue_min_markers", 3) or 3)
            if visible_count < min_markers:
                return False, f"visible_{visible_count}_below_min_{min_markers}"
            if visible_count == 4:
                if strong4 and subset not in strong4:
                    return False, "continue_four_subset_not_whitelisted"
                return True, "strong_continue_four_subset"
            if visible_count == 3:
                if not prediction_valid:
                    return False, "three_subset_without_prediction"
                if strong3 and subset not in strong3:
                    return False, "continue_three_subset_not_whitelisted"
                return True, "strong_continue_three_subset"
        return False, f"unsupported_subset_size_{visible_count}"

    def _other_object_gate_match_count(
        self,
        tracker: RigidBodyTracker,
        score: Dict[str, Any],
    ) -> int:
        """Count pose matches that reuse blob views already owned by another rigid gate."""
        matched = score.get("matched_observations", []) if isinstance(score, dict) else []
        if not isinstance(matched, list) or not matched:
            return 0
        matched_views: set[Tuple[str, int]] = set()
        for item in matched:
            if not isinstance(item, dict):
                continue
            try:
                blob_index = int(item.get("blob_index", -1))
            except (TypeError, ValueError):
                continue
            if blob_index >= 0:
                matched_views.add((str(item.get("camera_id", "")), blob_index))
        if not matched_views:
            return 0

        owned_elsewhere: set[Tuple[str, int]] = set()
        current_name = str(tracker.pattern.name)
        for name, other_tracker in self.trackers.items():
            if str(name) == current_name:
                continue
            gating = other_tracker.latest_object_gating()
            if not isinstance(gating, dict) or not gating.get("evaluated"):
                continue
            per_camera = gating.get("per_camera", {})
            if not isinstance(per_camera, dict):
                continue
            for camera_id, camera_payload in per_camera.items():
                if not isinstance(camera_payload, dict):
                    continue
                for assignment in camera_payload.get("assignments", []) or []:
                    if not isinstance(assignment, dict):
                        continue
                    try:
                        blob_index = int(assignment.get("blob_index", -1))
                    except (TypeError, ValueError):
                        continue
                    if blob_index >= 0:
                        owned_elsewhere.add((str(camera_id), blob_index))
        return len(matched_views & owned_elsewhere)

    def _should_select_rigid_hint_pose(
        self,
        tracker: RigidBodyTracker,
        diagnostic: Dict[str, Any],
    ) -> bool:
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
        if self._other_object_gate_match_count(tracker, score) > 0:
            diagnostic["selection_reason"] = "matched_other_rigid_gate_blobs"
            return False
        allowed, subset_reason = self._mode_subset_allowed(
            tracker.pattern,
            tracker.mode,
            score,
            prediction_valid=tracker.peek_prediction(
                int(diagnostic.get("timestamp", 0) or 0)
            ).valid,
        )
        if not allowed:
            diagnostic["selection_reason"] = "subset_not_allowed:" + subset_reason
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

    @staticmethod
    def _pose_with_continuity_rotation_hold(
        prediction: Dict[str, Any],
        *,
        source_pose: RigidBodyPose,
    ) -> Optional[RigidBodyPose]:
        try:
            quaternion = _normalize_quaternion(
                np.asarray(prediction["quaternion"], dtype=np.float64).reshape(4)
            )
        except (KeyError, TypeError, ValueError):
            return None
        if not np.isfinite(source_pose.position).all() or not np.isfinite(quaternion).all():
            return None
        return RigidBodyPose(
            timestamp=int(source_pose.timestamp),
            position=source_pose.position.copy(),
            rotation=_rotation_from_wxyz(quaternion),
            quaternion=quaternion,
            rms_error=float(source_pose.rms_error),
            observed_markers=int(source_pose.observed_markers),
            valid=True,
        )

    def _pose_with_position_accel_limit(
        self,
        guard: Dict[str, Any],
        *,
        source_pose: RigidBodyPose,
    ) -> Optional[RigidBodyPose]:
        try:
            limited_position = np.asarray(
                guard["limited_position"],
                dtype=np.float64,
            ).reshape(3)
        except (KeyError, TypeError, ValueError):
            return None
        if not np.isfinite(limited_position).all():
            return None
        return RigidBodyPose(
            timestamp=int(source_pose.timestamp),
            position=limited_position,
            rotation=source_pose.rotation.copy(),
            quaternion=source_pose.quaternion.copy(),
            rms_error=float(source_pose.rms_error),
            observed_markers=int(source_pose.observed_markers),
            valid=True,
        )

    def _evaluate_position_continuity_guard(
        self,
        tracker: RigidBodyTracker,
        pattern: MarkerPattern,
        pose: RigidBodyPose,
        score: Dict[str, Any],
    ) -> Dict[str, Any]:
        config = self.position_continuity_guard_config
        thresholds = config.thresholds_dict()
        if not config.enabled and not config.enforced:
            return _empty_position_continuity_guard(
                enabled=False,
                enforced=False,
                reason="disabled",
                thresholds=thresholds,
            )
        if not pose.valid:
            return _empty_position_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="invalid_pose",
                thresholds=thresholds,
            )

        score = score if isinstance(score, dict) else {}
        observed_markers = int(pose.observed_markers)
        expected_markers = int(pattern.num_markers)
        matched_views = int(score.get("matched_marker_views", 0) or 0)
        expected_views = int(score.get("expected_marker_views", expected_markers * 2) or 0)
        missing_views = int(score.get("missing_marker_views", 0) or 0)
        occluded = bool(observed_markers < expected_markers or missing_views > 0)
        if config.apply_to_occlusion_only and not occluded:
            return _empty_position_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="not_occluded",
                thresholds=thresholds,
            )

        metrics = tracker.position_continuity_metrics(pose)
        if not metrics.get("prediction_valid"):
            return _empty_position_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="no_prediction",
                thresholds=thresholds,
            )

        dt_s = float(metrics.get("dt_s", 0.0))
        previous_velocity = np.asarray(
            metrics.get("previous_velocity", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        ).reshape(3)
        candidate_velocity = np.asarray(
            metrics.get("candidate_velocity", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        ).reshape(3)
        previous_position = np.asarray(
            metrics.get("previous_position", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        ).reshape(3)
        delta_velocity = candidate_velocity - previous_velocity
        delta_velocity_norm = float(np.linalg.norm(delta_velocity))
        max_delta_velocity = max(0.0, float(config.max_accel_m_s2)) * dt_s
        limited_velocity = candidate_velocity.copy()
        if delta_velocity_norm > max_delta_velocity > 0.0:
            limited_velocity = previous_velocity + delta_velocity * (
                max_delta_velocity / delta_velocity_norm
            )

        limited_speed = float(np.linalg.norm(limited_velocity))
        max_velocity = max(0.0, float(config.max_velocity_m_s))
        if limited_speed > max_velocity > 0.0:
            limited_velocity = limited_velocity * (max_velocity / limited_speed)
            limited_speed = max_velocity
        limited_position = previous_position + limited_velocity * dt_s

        position_accel_m_s2 = float(metrics.get("position_accel_m_s2", 0.0))
        position_velocity_m_s = float(metrics.get("position_velocity_m_s", 0.0))
        previous_velocity_m_s = float(metrics.get("previous_velocity_m_s", 0.0))
        position_innovation_m = float(metrics.get("position_innovation_m", 0.0))
        reasons: List[str] = []
        if position_accel_m_s2 > config.max_accel_m_s2:
            reasons.append("position_accel_too_high")
        if position_velocity_m_s > config.max_velocity_m_s:
            reasons.append("position_velocity_too_high")

        passed = not reasons
        return {
            "enabled": bool(config.enabled or config.enforced),
            "enforced": bool(config.enforced),
            "evaluated": True,
            "passed": bool(passed),
            "would_reject": bool(not passed),
            "clamped_position": False,
            "reason": "ok" if passed else ",".join(reasons),
            "thresholds": thresholds,
            "occluded": occluded,
            "observed_markers": observed_markers,
            "expected_markers": expected_markers,
            "matched_marker_views": matched_views,
            "expected_marker_views": expected_views,
            "missing_marker_views": missing_views,
            "dt_s": dt_s,
            "position_innovation_m": position_innovation_m,
            "position_velocity_m_s": position_velocity_m_s,
            "previous_velocity_m_s": previous_velocity_m_s,
            "position_accel_m_s2": position_accel_m_s2,
            "limited_velocity_m_s": float(limited_speed),
            "limited_position": limited_position.tolist(),
            "evaluated_count": 0,
            "would_reject_count": 0,
            "clamped_count": 0,
        }

    def _evaluate_pose_continuity_guard(
        self,
        tracker: RigidBodyTracker,
        pattern: MarkerPattern,
        pose: RigidBodyPose,
        score: Dict[str, Any],
    ) -> Dict[str, Any]:
        config = self.pose_continuity_guard_config
        thresholds = config.thresholds_dict()
        if not config.enabled and not config.enforced:
            return _empty_pose_continuity_guard(
                enabled=False,
                enforced=False,
                reason="disabled",
                thresholds=thresholds,
            )
        if not pose.valid:
            return _empty_pose_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="invalid_pose",
                thresholds=thresholds,
            )

        score = score if isinstance(score, dict) else {}
        observed_markers = int(pose.observed_markers)
        expected_markers = int(pattern.num_markers)
        matched_views = int(score.get("matched_marker_views", 0) or 0)
        expected_views = int(score.get("expected_marker_views", expected_markers * 2) or 0)
        missing_views = int(score.get("missing_marker_views", 0) or 0)
        occluded = bool(observed_markers < expected_markers or missing_views > 0)
        if config.apply_to_occlusion_only and not occluded:
            return _empty_pose_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="not_occluded",
                thresholds=thresholds,
            )

        metrics = tracker.pose_continuity_metrics(pose)
        if not metrics.get("prediction_valid"):
            return _empty_pose_continuity_guard(
                enabled=config.enabled,
                enforced=config.enforced,
                reason="no_prediction",
                thresholds=thresholds,
            )

        position_innovation_m = float(metrics.get("position_innovation_m", 0.0))
        rotation_innovation_deg = float(metrics.get("rotation_innovation_deg", 0.0))
        angular_velocity_deg_s = float(metrics.get("angular_velocity_deg_s", 0.0))
        previous_angular_velocity_deg_s = float(
            metrics.get("previous_angular_velocity_deg_s", 0.0)
        )
        angular_accel_deg_s2 = float(metrics.get("angular_accel_deg_s2", 0.0))

        reasons: List[str] = []
        if rotation_innovation_deg > config.max_rotation_innovation_deg:
            reasons.append("rotation_innovation_too_high")
        if angular_velocity_deg_s > config.max_angular_velocity_deg_s:
            reasons.append("angular_velocity_too_high")
        if angular_accel_deg_s2 > config.max_angular_accel_deg_s2:
            reasons.append("angular_accel_too_high")

        passed = not reasons
        return {
            "enabled": bool(config.enabled or config.enforced),
            "enforced": bool(config.enforced),
            "evaluated": True,
            "passed": bool(passed),
            "would_reject": bool(not passed),
            "held_prediction": False,
            "held_rotation": False,
            "reason": "ok" if passed else ",".join(reasons),
            "thresholds": thresholds,
            "occluded": occluded,
            "observed_markers": observed_markers,
            "expected_markers": expected_markers,
            "matched_marker_views": matched_views,
            "expected_marker_views": expected_views,
            "missing_marker_views": missing_views,
            "position_innovation_m": position_innovation_m,
            "rotation_innovation_deg": rotation_innovation_deg,
            "angular_velocity_deg_s": angular_velocity_deg_s,
            "previous_angular_velocity_deg_s": previous_angular_velocity_deg_s,
            "angular_accel_deg_s2": angular_accel_deg_s2,
            "prediction": dict(metrics.get("prediction", {})),
            "evaluated_count": 0,
            "would_reject_count": 0,
            "held_count": 0,
        }

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
        generic_is_rigid_hint: bool = False,
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

        if generic_is_rigid_hint and generic_pose is not None and generic_pose.valid:
            pose = generic_pose
            score = dict(generic_score or _empty_reprojection_score())
        else:
            observed = np.asarray([by_marker[index]["point"] for index in marker_indices], dtype=np.float64)
            reference = self._reference_points_for_indices(
                pattern,
                tuple(int(index) for index in marker_indices),
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
        reference = self._reference_points_for_indices(pattern, marker_indices)
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
            if source in {
                "rigid_hint_subset",
                "temporal_body_nbest",
            }
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
                float(sum(float(value) for value in errors) / len(errors))
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
                    "contributing_rays": int(contributing_rays),
                    "mean_error_px": float(mean_error),
                }
        return by_marker

    def _rigid_hint_markers_by_rigid(
        self,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
    ) -> Dict[str, Dict[int, Dict[str, Any]]]:
        by_rigid: Dict[str, Dict[int, Dict[str, Any]]] = {}
        pattern_by_name = {pattern.name: pattern for pattern in self.patterns}

        for payload in rigid_hint_triangulated_points or []:
            if not isinstance(payload, dict) or bool(payload.get("is_virtual", False)):
                continue
            rigid_name = str(payload.get("rigid_name", ""))
            pattern = pattern_by_name.get(rigid_name)
            if pattern is None:
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
                float(sum(float(value) for value in errors) / len(errors))
                if isinstance(errors, list) and errors
                else 0.0
            )
            contributing_rays = int(payload.get("contributing_rays", 0))
            weight = max(0.1, float(contributing_rays)) / (1.0 + mean_error)
            markers = by_rigid.setdefault(rigid_name, {})
            previous = markers.get(marker_idx)
            previous_weight = float(previous.get("weight", -1.0)) if previous else -1.0
            if previous is None or weight > previous_weight:
                markers[marker_idx] = {
                    "point": point,
                    "weight": float(weight),
                    "contributing_rays": int(contributing_rays),
                    "mean_error_px": float(mean_error),
                }
        return by_rigid

    def _subset_candidate_pruned(
        self,
        *,
        source: str,
        pattern: MarkerPattern,
        observed: np.ndarray,
        marker_indices: Tuple[int, ...],
        generic_pose: Optional[RigidBodyPose],
    ) -> bool:
        if source in {"rigid_hint_subset", "temporal_body_nbest"}:
            return False
        if generic_pose is None or not generic_pose.valid:
            return False
        predicted = (
            generic_pose.rotation
            @ self._reference_points_for_indices(pattern, marker_indices).T
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
        normalized_indices = tuple(sorted(int(index) for index in marker_indices))
        if len(normalized_indices) >= pattern.num_markers:
            return False
        cache_key = self._pattern_subset_cache_key(pattern, normalized_indices)
        cached = self._subset_ambiguity_cache.get(cache_key)
        if cached is not None:
            return bool(cached)

        profile = self._subset_distance_profile(
            self._reference_points_for_indices(pattern, normalized_indices)
        )
        ambiguous = False
        for other in combinations(range(pattern.num_markers), len(normalized_indices)):
            if tuple(other) == normalized_indices:
                continue
            other_profile = self._subset_distance_profile(
                self._reference_points_for_indices(
                    pattern,
                    tuple(int(index) for index in other),
                )
            )
            if len(profile) != len(other_profile):
                continue
            delta = float(np.linalg.norm(profile - other_profile))
            if delta <= self.subset_solve_config.ambiguous_subset_delta_m:
                ambiguous = True
                break
        self._subset_ambiguity_cache[cache_key] = bool(ambiguous)
        return ambiguous

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

    @staticmethod
    def _residual_error_stats(residuals: List[float]) -> Tuple[float, float, float]:
        if not residuals:
            return 0.0, 0.0, 0.0
        values = np.sort(np.asarray(residuals, dtype=np.float64).reshape(-1))
        mean_error = float(values.mean())
        max_error = float(values[-1])
        if len(values) == 1:
            return mean_error, max_error, max_error
        rank = (len(values) - 1) * 0.95
        lower = int(np.floor(rank))
        upper = int(np.ceil(rank))
        if lower == upper:
            p95_error = float(values[lower])
        else:
            weight = float(rank - lower)
            p95_error = float(values[lower] * (1.0 - weight) + values[upper] * weight)
        return mean_error, p95_error, max_error

    @staticmethod
    def _blob_diameter_px_from_area(area: float) -> float:
        try:
            value = float(area)
        except (TypeError, ValueError):
            return 0.0
        if value <= 0.0 or not np.isfinite(value):
            return 0.0
        return float(np.sqrt(4.0 * value / np.pi))

    @staticmethod
    def _reprojection_uncertainty_px(blob_diameter_px: float) -> float:
        """Continuous centroid uncertainty from blob size; broader, not thresholdier."""
        diameter = float(blob_diameter_px)
        if not np.isfinite(diameter) or diameter <= 0.0:
            return 1.0
        return float(max(1.0, 4.0 / diameter))

    def _payload_reprojection_uncertainty_px(self, payload: Dict[str, Any]) -> Tuple[float, float]:
        diameter = self._blob_diameter_px_from_area(float(payload.get("area", 0.0) or 0.0))
        return diameter, self._reprojection_uncertainty_px(diameter)

    @staticmethod
    def _blob_diameters_and_uncertainties_from_areas(
        areas: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        area_arr = np.asarray(areas, dtype=np.float64).reshape(-1)
        diameters = np.zeros(len(area_arr), dtype=np.float64)
        valid_areas = np.isfinite(area_arr) & (area_arr > 0.0)
        diameters[valid_areas] = np.sqrt(4.0 * area_arr[valid_areas] / np.pi)
        uncertainties = np.ones(len(area_arr), dtype=np.float64)
        valid_diameters = np.isfinite(diameters) & (diameters > 0.0)
        uncertainties[valid_diameters] = np.maximum(
            1.0,
            4.0 / diameters[valid_diameters],
        )
        return diameters, uncertainties

    def _prepare_observation_context(
        self,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Optional[_PreparedObservationContext]:
        if not camera_params or not observations_by_camera:
            return None
        space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"
        prepared: Dict[str, _PreparedCameraObservations] = {}
        total_blob_count = 0
        for camera_id, observations in observations_by_camera.items():
            camera = camera_params.get(camera_id)
            if camera is None:
                continue
            payloads = self._extract_observation_payloads(observations, space)
            if not payloads:
                continue
            uvs = np.asarray(
                [payload["uv"] for payload in payloads],
                dtype=np.float64,
            ).reshape(-1, 2)
            blob_indices = np.asarray(
                [int(payload.get("blob_index", index)) for index, payload in enumerate(payloads)],
                dtype=np.int64,
            ).reshape(-1)
            areas = np.asarray(
                [float(payload.get("area", 0.0) or 0.0) for payload in payloads],
                dtype=np.float64,
            ).reshape(-1)
            diameters, uncertainties = self._blob_diameters_and_uncertainties_from_areas(
                areas
            )
            camera_key = str(camera_id)
            prepared[camera_key] = _PreparedCameraObservations(
                camera_id=camera_key,
                camera=camera,
                uv=uvs,
                blob_indices=blob_indices,
                area_px2=areas,
                diameter_px=diameters,
                uncertainty_px=uncertainties,
            )
            total_blob_count += int(len(uvs))
        if not prepared:
            return None
        return _PreparedObservationContext(
            coordinate_space=space,
            by_camera=prepared,
            camera_count=int(len(prepared)),
            total_blob_count=int(total_blob_count),
        )

    def _score_pose_reprojection(
        self,
        pose: RigidBodyPose,
        pattern: MarkerPattern,
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
        observation_context: Optional[_PreparedObservationContext] = None,
    ) -> Dict[str, Any]:
        if not pose.valid:
            return _empty_reprojection_score("invalid_pose")
        context = observation_context or self._prepare_observation_context(
            camera_params,
            observations_by_camera,
            coordinate_space=coordinate_space,
        )
        if context is None:
            return _empty_reprojection_score("no_2d_context")

        space = context.coordinate_space
        markers_world = (pose.rotation @ pattern.marker_positions.T).T + pose.position.reshape(1, 3)
        residuals: List[float] = []
        normalized_residuals: List[float] = []
        matched = 0
        expected = 0
        duplicate_assignments = 0
        unexpected_blobs = 0
        scored_cameras = 0
        matched_observations: List[Dict[str, Any]] = []

        for camera_obs in context.by_camera.values():
            observed_uvs = camera_obs.uv
            projected_uvs = self._project_markers_to_camera(
                markers_world,
                camera_obs.camera,
                space,
            )
            if not projected_uvs:
                continue
            projected_arr = np.asarray(projected_uvs, dtype=np.float64).reshape(-1, 2)

            scored_cameras += 1
            expected += len(projected_uvs)
            uncertainty_px = camera_obs.uncertainty_px.reshape(1, -1)
            deltas = projected_arr[:, None, :] - observed_uvs[None, :, :]
            cost_px = np.sqrt(np.sum(deltas * deltas, axis=2))
            cost = cost_px / np.maximum(uncertainty_px, 1e-6)
            effective_gate_px = self.reprojection_match_gate_px * np.maximum(
                uncertainty_px,
                1.0,
            )
            invalid_cost = 1e9
            gated_cost = np.where(cost_px <= effective_gate_px, cost, invalid_cost)
            row_ind, col_ind = linear_sum_assignment(gated_cost)
            assigned_blob_indices = set()
            for row, col in zip(row_ind, col_ind):
                normalized_error = float(gated_cost[row, col])
                if normalized_error >= invalid_cost:
                    continue
                error = float(cost_px[row, col])
                diameter_px = float(camera_obs.diameter_px[int(col)])
                uncertainty = float(camera_obs.uncertainty_px[int(col)])
                matched += 1
                residuals.append(error)
                normalized_residuals.append(normalized_error)
                assigned_blob_indices.add(int(col))
                matched_observations.append(
                    {
                        "camera_id": camera_obs.camera_id,
                        "marker_idx": int(row),
                        "blob_index": int(camera_obs.blob_indices[int(col)]),
                        "observed_uv": [
                            float(value)
                            for value in observed_uvs[int(col)].reshape(2)
                        ],
                        "error_px": error,
                        "normalized_error": normalized_error,
                        "blob_diameter_px": float(diameter_px),
                        "uncertainty_px": float(uncertainty),
                    }
                )

            unexpected_blobs += max(0, camera_obs.blob_count - len(assigned_blob_indices))

        if expected <= 0:
            return _empty_reprojection_score("no_projectable_markers")

        missing = max(0, expected - matched)
        mean_error, p95_error, max_error = self._residual_error_stats(residuals)
        mean_normalized_error, p95_normalized_error, max_normalized_error = (
            self._residual_error_stats(normalized_residuals)
        )
        coverage = float(matched / expected) if expected else 0.0
        residual_score = (
            1.0 / (1.0 + mean_normalized_error / 5.0)
            if normalized_residuals
            else 0.0
        )
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
            "mean_normalized_error": mean_normalized_error,
            "p95_normalized_error": p95_normalized_error,
            "max_normalized_error": max_normalized_error,
            "matched_marker_views": int(matched),
            "expected_marker_views": int(expected),
            "missing_marker_views": int(missing),
            "duplicate_assignment_count": int(duplicate_assignments),
            "unexpected_blob_count": int(unexpected_blobs),
            "camera_count": int(scored_cameras),
            "match_gate_px": float(self.reprojection_match_gate_px),
            "matched_observations": matched_observations,
        }

    def _score_full_continue_hint_quality(
        self,
        pose: RigidBodyPose,
        pattern: MarkerPattern,
        tracker: RigidBodyTracker,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Dict[str, Any]:
        if (
            not pose.valid
            or tracker.mode != TrackMode.CONTINUE
            or int(pose.observed_markers) < int(pattern.num_markers)
        ):
            return _empty_reprojection_score("not_full_continue_hint")

        residuals: List[float] = []
        normalized_residuals: List[float] = []
        matched_observations: List[Dict[str, Any]] = []
        marker_indices: set[int] = set()
        used_views: set[Tuple[str, int]] = set()
        duplicate_assignments = 0
        for payload in rigid_hint_triangulated_points or []:
            if not isinstance(payload, dict):
                continue
            if str(payload.get("rigid_name", pattern.name)) != pattern.name:
                continue
            try:
                marker_idx = int(payload["marker_idx"])
            except (KeyError, TypeError, ValueError):
                continue
            if marker_idx < 0 or marker_idx >= pattern.num_markers:
                continue
            observations = payload.get("observations", []) or []
            errors = payload.get("reprojection_errors_px", []) or []
            if not isinstance(observations, list) or not isinstance(errors, list):
                continue
            marker_indices.add(marker_idx)
            for observation, error_value in zip(observations, errors):
                if not isinstance(observation, dict):
                    continue
                try:
                    camera_id = str(observation.get("camera_id", ""))
                    blob_index = int(observation.get("blob_index", -1))
                    error = float(error_value)
                except (TypeError, ValueError):
                    continue
                diameter_px = self._blob_diameter_px_from_area(
                    float(observation.get("area", 0.0) or 0.0)
                )
                uncertainty = self._reprojection_uncertainty_px(diameter_px)
                if error > self.reprojection_match_gate_px * max(1.0, uncertainty):
                    continue
                view = (camera_id, blob_index)
                if blob_index >= 0 and view in used_views:
                    duplicate_assignments += 1
                    continue
                if blob_index >= 0:
                    used_views.add(view)
                residuals.append(error)
                normalized_error = float(error / max(uncertainty, 1e-6))
                normalized_residuals.append(normalized_error)
                matched_observations.append(
                    {
                        "camera_id": camera_id,
                        "marker_idx": int(marker_idx),
                        "blob_index": int(blob_index),
                        "observed_uv": [
                            float(value)
                            for value in np.asarray(
                                observation.get("undistorted_uv")
                                or observation.get("raw_uv")
                                or [observation.get("x", 0.0), observation.get("y", 0.0)],
                                dtype=np.float64,
                            ).reshape(2)
                        ],
                        "error_px": error,
                        "normalized_error": normalized_error,
                        "blob_diameter_px": float(diameter_px),
                        "uncertainty_px": float(uncertainty),
                    }
                )
        if len(marker_indices) < int(pattern.num_markers):
            return _empty_reprojection_score("not_all_hint_markers")

        camera_count = int(
            len(observations_by_camera)
            if isinstance(observations_by_camera, dict) and observations_by_camera
            else len({item["camera_id"] for item in matched_observations})
        )
        (
            object_gating_evaluated,
            object_gating_marker_count,
            object_gating_camera_count,
            object_gating_assigned_views,
        ) = tracker.latest_object_gating_counts()
        if object_gating_evaluated and object_gating_marker_count == int(pattern.num_markers):
            camera_count = max(camera_count, object_gating_camera_count)
        expected = int(pattern.num_markers) * max(0, camera_count)
        if expected <= 0:
            return _empty_reprojection_score("no_projectable_markers")

        matched = len(matched_observations)
        if object_gating_evaluated and object_gating_marker_count == int(pattern.num_markers):
            matched = max(
                matched,
                min(expected, object_gating_assigned_views),
            )
        missing = max(0, expected - matched)
        mean_error, p95_error, max_error = self._residual_error_stats(residuals)
        mean_normalized_error, p95_normalized_error, max_normalized_error = (
            self._residual_error_stats(normalized_residuals)
        )
        coverage = float(matched / expected) if expected else 0.0
        residual_score = (
            1.0 / (1.0 + mean_normalized_error / 5.0)
            if normalized_residuals
            else 0.0
        )
        duplicate_penalty = min(1.0, duplicate_assignments / max(1, matched))
        score = float(
            np.clip(
                0.70 * residual_score + 0.30 * coverage - 0.30 * duplicate_penalty,
                0.0,
                1.0,
            )
        )
        total_blobs = (
            sum(len(items or []) for items in observations_by_camera.values())
            if isinstance(observations_by_camera, dict)
            else matched
        )
        return {
            "scored": True,
            "reason": "ok",
            "coordinate_space": (
                "undistorted_pixel"
                if coordinate_space == "undistorted_pixel"
                else "raw_pixel"
            ),
            "score": score,
            "mean_error_px": mean_error,
            "p95_error_px": p95_error,
            "max_error_px": max_error,
            "mean_normalized_error": mean_normalized_error,
            "p95_normalized_error": p95_normalized_error,
            "max_normalized_error": max_normalized_error,
            "matched_marker_views": int(matched),
            "expected_marker_views": int(expected),
            "missing_marker_views": int(missing),
            "duplicate_assignment_count": int(duplicate_assignments),
            "unexpected_blob_count": int(max(0, total_blobs - matched)),
            "camera_count": int(camera_count),
            "match_gate_px": float(self.reprojection_match_gate_px),
            "matched_observations": matched_observations,
            "rigid_hint_marker_indices": [int(index) for index in sorted(marker_indices)],
            "rigid_hint_real_ray_count": int(matched),
            "score_source": "rigid_hint_quality",
        }

    def _score_pose_reprojection_from_hint_observations(
        self,
        pose: RigidBodyPose,
        pattern: MarkerPattern,
        rigid_hint_triangulated_points: Optional[List[Dict[str, Any]]],
        camera_params: Optional[Dict[str, Any]],
        observations_by_camera: Optional[Dict[str, List[Any]]],
        *,
        coordinate_space: str,
    ) -> Dict[str, Any]:
        if not pose.valid:
            return _empty_reprojection_score("invalid_pose")
        if not camera_params:
            return _empty_reprojection_score("no_camera_context")

        space = "undistorted_pixel" if coordinate_space == "undistorted_pixel" else "raw_pixel"
        markers_world = (
            pose.rotation @ pattern.marker_positions.T
        ).T + pose.position.reshape(1, 3)
        residuals: List[float] = []
        normalized_residuals: List[float] = []
        matched_observations: List[Dict[str, Any]] = []
        used_views: set[Tuple[str, int]] = set()
        duplicate_assignments = 0
        scored_cameras: set[str] = set()
        marker_indices: set[int] = set()
        camera_projection_cache: Dict[str, List[Tuple[float, float]]] = {}

        for payload in rigid_hint_triangulated_points or []:
            if not isinstance(payload, dict):
                continue
            if str(payload.get("rigid_name", pattern.name)) != pattern.name:
                continue
            try:
                marker_idx = int(payload["marker_idx"])
            except (KeyError, TypeError, ValueError):
                continue
            if marker_idx < 0 or marker_idx >= pattern.num_markers:
                continue
            for observation in payload.get("observations", []) or []:
                if not isinstance(observation, dict):
                    continue
                camera_id = str(observation.get("camera_id", ""))
                camera = camera_params.get(camera_id)
                if camera is None:
                    continue
                raw_uv = observation.get("undistorted_uv" if space == "undistorted_pixel" else "raw_uv")
                if raw_uv is None:
                    raw_uv = [observation.get("x", 0.0), observation.get("y", 0.0)]
                try:
                    observed_uv = np.asarray(raw_uv, dtype=np.float64).reshape(2)
                    blob_index = int(observation.get("blob_index", -1))
                except (TypeError, ValueError):
                    continue
                projected = camera_projection_cache.get(camera_id)
                if projected is None:
                    projected = self._project_markers_to_camera(markers_world, camera, space)
                    camera_projection_cache[camera_id] = projected
                if marker_idx >= len(projected):
                    continue
                error = float(
                    np.linalg.norm(
                        np.asarray(projected[marker_idx], dtype=np.float64) - observed_uv
                    )
                )
                diameter_px = self._blob_diameter_px_from_area(
                    float(observation.get("area", 0.0) or 0.0)
                )
                uncertainty = self._reprojection_uncertainty_px(diameter_px)
                if error > self.reprojection_match_gate_px * max(1.0, uncertainty):
                    continue
                view = (camera_id, blob_index)
                if blob_index >= 0 and view in used_views:
                    duplicate_assignments += 1
                    continue
                if blob_index >= 0:
                    used_views.add(view)
                scored_cameras.add(camera_id)
                marker_indices.add(marker_idx)
                residuals.append(error)
                normalized_error = float(error / max(uncertainty, 1e-6))
                normalized_residuals.append(normalized_error)
                matched_observations.append(
                    {
                        "camera_id": camera_id,
                        "marker_idx": int(marker_idx),
                        "blob_index": int(blob_index),
                        "observed_uv": [float(value) for value in observed_uv.reshape(2)],
                        "error_px": error,
                        "normalized_error": normalized_error,
                        "blob_diameter_px": float(diameter_px),
                        "uncertainty_px": float(uncertainty),
                    }
                )

        camera_count = int(
            len(observations_by_camera)
            if isinstance(observations_by_camera, dict) and observations_by_camera
            else len(scored_cameras)
        )
        expected = int(pattern.num_markers) * max(0, camera_count)
        if expected <= 0:
            return _empty_reprojection_score("no_projectable_markers")

        matched = len(matched_observations)
        missing = max(0, expected - matched)
        mean_error, p95_error, max_error = self._residual_error_stats(residuals)
        mean_normalized_error, p95_normalized_error, max_normalized_error = (
            self._residual_error_stats(normalized_residuals)
        )
        coverage = float(matched / expected) if expected else 0.0
        residual_score = (
            1.0 / (1.0 + mean_normalized_error / 5.0)
            if normalized_residuals
            else 0.0
        )
        duplicate_penalty = min(1.0, duplicate_assignments / max(1, matched))
        score = float(
            np.clip(
                0.70 * residual_score + 0.30 * coverage - 0.30 * duplicate_penalty,
                0.0,
                1.0,
            )
        )
        total_blobs = (
            sum(len(items or []) for items in observations_by_camera.values())
            if isinstance(observations_by_camera, dict)
            else matched
        )
        return {
            "scored": True,
            "reason": "ok",
            "coordinate_space": space,
            "score": score,
            "mean_error_px": mean_error,
            "p95_error_px": p95_error,
            "max_error_px": max_error,
            "mean_normalized_error": mean_normalized_error,
            "p95_normalized_error": p95_normalized_error,
            "max_normalized_error": max_normalized_error,
            "matched_marker_views": int(matched),
            "expected_marker_views": int(expected),
            "missing_marker_views": int(missing),
            "duplicate_assignment_count": int(duplicate_assignments),
            "unexpected_blob_count": int(max(0, total_blobs - matched)),
            "camera_count": int(camera_count),
            "match_gate_px": float(self.reprojection_match_gate_px),
            "matched_observations": matched_observations,
            "rigid_hint_marker_indices": [int(index) for index in sorted(marker_indices)],
            "rigid_hint_real_ray_count": int(matched),
        }

    @staticmethod
    def _extract_observation_payloads(
        observations: List[Any],
        coordinate_space: str,
    ) -> List[Dict[str, Any]]:
        key = "undistorted_uv" if coordinate_space == "undistorted_pixel" else "raw_uv"
        payloads: List[Dict[str, Any]] = []
        for index, observation in enumerate(observations):
            value = None
            blob_index = index
            area = 0.0
            if isinstance(observation, dict):
                value = observation.get(key)
                blob_index = observation.get("blob_index", index)
                area = observation.get("area", 0.0)
            else:
                value = getattr(observation, key, None)
                blob_index = getattr(observation, "blob_index", index)
                area = getattr(observation, "area", 0.0)
            if value is None or len(value) < 2:
                continue
            try:
                uv = (float(value[0]), float(value[1]))
                payloads.append(
                    {
                        "uv": uv,
                        "blob_index": int(blob_index),
                        "area": float(area or 0.0),
                    }
                )
            except (TypeError, ValueError, IndexError):
                continue
        return payloads

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
        distortion = np.asarray(
            getattr(camera, "distortion_coeffs", np.zeros(5, dtype=np.float64)),
            dtype=np.float64,
        ).reshape(-1)
        if coordinate_space == "undistorted_pixel" or not np.any(np.abs(distortion) > 1e-12):
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
            distortion,
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

    def get_tracking_event_status(self) -> Dict[str, Dict[str, Any]]:
        """Get lightweight tracking diagnostics for per-frame event recording."""
        return {
            name: tracker.get_event_diagnostics()
            for name, tracker in self.trackers.items()
        }
