from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from host.rigid import PointClusterer, RigidBodyEstimator, WAIST_PATTERN


def test_point_clusterer_defaults_to_80mm_radius() -> None:
    clusterer = PointClusterer()

    assert clusterer.eps == 0.08
    assert clusterer.cluster_radius_m == 0.08


def test_point_clusterer_groups_points_within_80mm_chain() -> None:
    clusterer = PointClusterer()
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.06, 0.0, 0.0],
            [0.12, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [0.56, 0.0, 0.0],
            [0.62, 0.0, 0.0],
        ],
        dtype=np.float64,
    )

    clusters = clusterer.cluster(points)

    assert sorted(len(cluster) for cluster in clusters) == [3, 3]


def test_rigid_body_estimator_passes_80mm_radius_to_clusterer() -> None:
    estimator = RigidBodyEstimator()

    assert estimator.cluster_radius_m == 0.08
    assert estimator.clusterer.eps == 0.08


def test_single_pattern_estimator_falls_back_to_full_points_when_80mm_clusters_split() -> None:
    estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN], cluster_radius_m=0.08)
    points = WAIST_PATTERN.marker_positions + np.array([1.0, 2.0, 3.0], dtype=np.float64)

    pose = estimator.process_points(points, timestamp=123)["waist"]

    assert pose.valid is True
    assert pose.observed_markers == WAIST_PATTERN.num_markers
