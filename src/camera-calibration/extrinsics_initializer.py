from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from itertools import combinations
from typing import Any, Dict, List, Sequence, Tuple

import cv2
import numpy as np

from extrinsics_samples import PoseCaptureSample


@dataclass(frozen=True)
class PairInitialization:
    camera_a: str
    camera_b: str
    rotation_ab: np.ndarray
    translation_ab: np.ndarray
    inlier_ratio: float
    usable_points: int
    angle_score: float


def _normalize(camera: Any, points: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(
        points.reshape(-1, 1, 2),
        camera.intrinsic_matrix,
        camera.distortion_coeffs,
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
        if essential is None:
            continue
        _, rotation, translation, pose_mask = cv2.recoverPose(essential, norm_a, norm_b)
        inliers = pose_mask.ravel() > 0 if pose_mask is not None else np.ones(norm_a.shape[0], dtype=bool)
        usable = int(np.count_nonzero(inliers))
        if usable < max(8, min_pairs):
            continue
        angle_proxy = float(np.clip(np.std(norm_a[inliers] - norm_b[inliers]) * 6.0, 0.1, 1.0))
        out.append(
            PairInitialization(
                camera_a=camera_a,
                camera_b=camera_b,
                rotation_ab=rotation,
                translation_ab=translation.reshape(3),
                inlier_ratio=float(np.mean(inliers.astype(np.float64))),
                usable_points=usable,
                angle_score=angle_proxy,
            )
        )
    return out


def build_initial_camera_poses(
    reference_camera_id: str,
    camera_ids: Sequence[str],
    pair_rows: Sequence[PairInitialization],
) -> Tuple[Dict[str, Tuple[np.ndarray, np.ndarray]], List[Dict[str, float]]]:
    edge_rows: List[Dict[str, float]] = []
    weighted_edges: List[Tuple[float, PairInitialization]] = []
    for row in pair_rows:
        score = float(row.usable_points) * float(row.inlier_ratio) * float(row.angle_score)
        weighted_edges.append((score, row))
        edge_rows.append(
            {
                "camera_a": row.camera_a,
                "camera_b": row.camera_b,
                "usable_points": float(row.usable_points),
                "inlier_ratio": float(row.inlier_ratio),
                "angle_score": float(row.angle_score),
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
    return poses, edge_rows
