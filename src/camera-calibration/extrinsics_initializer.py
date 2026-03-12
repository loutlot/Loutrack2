from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from extrinsics_samples import PoseCaptureSample


MIN_TRIANGULATION_ANGLE_DEG = 0.5


@dataclass(frozen=True)
class PairInitialization:
    camera_a: str
    camera_b: str
    rotation_ab: np.ndarray
    translation_ab: np.ndarray
    inlier_ratio: float
    usable_points: int
    triangulation_angle_deg_p50: float


def _normalize(camera: Any, points: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(
        points.reshape(-1, 1, 2).astype(np.float64),
        camera.intrinsic_matrix.astype(np.float64),
        camera.distortion_coeffs.astype(np.float64),
    ).reshape(-1, 2)


def _pair_correspondences(
    camera_a: str,
    camera_b: str,
    camera_params: Dict[str, Any],
    samples: Sequence[PoseCaptureSample],
) -> Tuple[np.ndarray, np.ndarray]:
    points_a: List[np.ndarray] = []
    points_b: List[np.ndarray] = []
    cam_a = camera_params[camera_a]
    cam_b = camera_params[camera_b]
    for sample in samples:
        if camera_a not in sample.image_points_by_camera or camera_b not in sample.image_points_by_camera:
            continue
        points_a.append(sample.image_points_by_camera[camera_a])
        points_b.append(sample.image_points_by_camera[camera_b])
    if not points_a:
        return np.empty((0, 2), dtype=np.float64), np.empty((0, 2), dtype=np.float64)
    a = _normalize(cam_a, np.asarray(points_a, dtype=np.float64))
    b = _normalize(cam_b, np.asarray(points_b, dtype=np.float64))
    return a, b


def _triangulation_angles_deg(
    normalized_a: np.ndarray,
    normalized_b: np.ndarray,
    rotation_ab: np.ndarray,
    translation_ab: np.ndarray,
) -> np.ndarray:
    if normalized_a.shape[0] == 0:
        return np.empty((0,), dtype=np.float64)
    pa = np.hstack([np.eye(3, dtype=np.float64), np.zeros((3, 1), dtype=np.float64)])
    pb = np.hstack([rotation_ab, translation_ab.reshape(3, 1)])
    points_4d = cv2.triangulatePoints(pa, pb, normalized_a.T, normalized_b.T)
    points_3d = (points_4d[:3] / points_4d[3]).T
    center_b = (-(rotation_ab.T @ translation_ab.reshape(3, 1))).reshape(3)
    angles: List[float] = []
    for point in points_3d:
        depth_a = float(point[2])
        depth_b = float((rotation_ab @ point.reshape(3, 1) + translation_ab.reshape(3, 1))[2, 0])
        if depth_a <= 0.0 or depth_b <= 0.0:
            continue
        ray_a = point / max(np.linalg.norm(point), 1e-12)
        ray_b_vec = point - center_b
        ray_b = ray_b_vec / max(np.linalg.norm(ray_b_vec), 1e-12)
        cos_angle = float(np.clip(np.dot(ray_a, ray_b), -1.0, 1.0))
        angles.append(float(np.degrees(np.arccos(cos_angle))))
    return np.asarray(angles, dtype=np.float64)


def estimate_pairwise_initialization(
    camera_params: Dict[str, Any],
    samples: Sequence[PoseCaptureSample],
    min_pairs: int,
) -> List[PairInitialization]:
    out: List[PairInitialization] = []
    for camera_a, camera_b in combinations(sorted(camera_params.keys()), 2):
        norm_a, norm_b = _pair_correspondences(camera_a, camera_b, camera_params, samples)
        if norm_a.shape[0] < max(8, min_pairs):
            continue
        essential, mask = cv2.findEssentialMat(
            norm_a,
            norm_b,
            focal=1.0,
            pp=(0.0, 0.0),
            method=cv2.RANSAC,
            prob=0.999,
            threshold=1e-3,
        )
        if essential is None or mask is None:
            continue
        essential_inliers = mask.ravel() > 0
        if int(np.count_nonzero(essential_inliers)) < max(8, min_pairs):
            continue
        inlier_a = norm_a[essential_inliers]
        inlier_b = norm_b[essential_inliers]
        _, rotation, translation, pose_mask = cv2.recoverPose(essential, inlier_a, inlier_b)
        positive_depth = pose_mask.ravel() > 0 if pose_mask is not None else np.ones(inlier_a.shape[0], dtype=bool)
        usable = int(np.count_nonzero(positive_depth))
        if usable < max(8, min_pairs):
            continue
        usable_a = inlier_a[positive_depth]
        usable_b = inlier_b[positive_depth]
        angles = _triangulation_angles_deg(usable_a, usable_b, rotation, translation.reshape(3))
        if angles.size == 0:
            continue
        angle_p50 = float(np.median(angles))
        if angle_p50 < MIN_TRIANGULATION_ANGLE_DEG:
            continue
        out.append(
            PairInitialization(
                camera_a=camera_a,
                camera_b=camera_b,
                rotation_ab=rotation,
                translation_ab=translation.reshape(3),
                inlier_ratio=float(np.mean(essential_inliers.astype(np.float64))),
                usable_points=usable,
                triangulation_angle_deg_p50=angle_p50,
            )
        )
    return out


def build_initial_camera_poses(
    reference_camera_id: str,
    camera_ids: Sequence[str],
    pair_rows: Sequence[PairInitialization],
) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], List[Dict[str, float]], Dict[str, str]]:
    edge_rows: List[Dict[str, float]] = []
    weighted_edges: List[Tuple[float, PairInitialization]] = []
    for row in pair_rows:
        score = float(row.usable_points) * float(row.inlier_ratio) * float(row.triangulation_angle_deg_p50)
        weighted_edges.append((score, row))
        edge_rows.append(
            {
                "camera_a": row.camera_a,
                "camera_b": row.camera_b,
                "usable_points": float(row.usable_points),
                "inlier_ratio": float(row.inlier_ratio),
                "triangulation_angle_deg_p50": float(row.triangulation_angle_deg_p50),
                "score": score,
            }
        )
    parents = {camera_id: camera_id for camera_id in camera_ids}

    def find(camera_id: str) -> str:
        root = camera_id
        while parents[root] != root:
            root = parents[root]
        while parents[camera_id] != camera_id:
            nxt = parents[camera_id]
            parents[camera_id] = root
            camera_id = nxt
        return root

    def union(camera_a: str, camera_b: str) -> bool:
        root_a = find(camera_a)
        root_b = find(camera_b)
        if root_a == root_b:
            return False
        parents[root_b] = root_a
        return True

    mst_graph: Dict[str, List[Tuple[str, PairInitialization, bool]]] = {camera_id: [] for camera_id in camera_ids}
    for _, row in sorted(weighted_edges, key=lambda item: item[0], reverse=True):
        if not union(row.camera_a, row.camera_b):
            continue
        mst_graph[row.camera_a].append((row.camera_b, row, False))
        mst_graph[row.camera_b].append((row.camera_a, row, True))

    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {
        reference_camera_id: (np.eye(3, dtype=np.float64), np.zeros(3, dtype=np.float64))
    }
    queue: deque[str] = deque([reference_camera_id])
    while queue:
        src = queue.popleft()
        src_r, src_t = poses[src]
        for dst, edge, reverse in mst_graph.get(src, []):
            if dst in poses:
                continue
            if reverse:
                rel_r = edge.rotation_ab.T
                rel_t = -(edge.rotation_ab.T @ edge.translation_ab.reshape(3, 1)).reshape(3)
            else:
                rel_r = edge.rotation_ab
                rel_t = edge.translation_ab
            dst_r = rel_r @ src_r
            dst_t = rel_r @ src_t + rel_t
            poses[dst] = (dst_r, dst_t)
            queue.append(dst)

    excluded_reasons = {
        camera_id: "no_connected_pairwise_path_from_reference"
        for camera_id in camera_ids
        if camera_id not in poses
    }
    return poses, edge_rows, excluded_reasons
