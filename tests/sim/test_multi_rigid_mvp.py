import inspect
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
assert SRC.exists(), f"Source path not found: {SRC}"
sys.path.insert(0, str(SRC))

from tools import sim  # type: ignore
from tools.sim.multi_rigid import (  # type: ignore
    DEFAULT_BODY_RIGIDS,
    DEFAULT_GENERATED_CAMERA_IDS,
    DESIGN_5MARKER_LAYOUT,
    MultiRigidFrameGenerator,
    MultiRigidScenarioConfig,
    load_camera_rig,
    load_mvp_patterns,
    run_multi_rigid_scenario,
)
from host.receiver import PairedFrames  # type: ignore


_BLOB_METADATA_KEYS = {
    "rigid_name",
    "gt_rigid_name",
    "marker_index",
    "gt_marker_index",
    "synthetic_blob_id",
}


def _make_mvp_generator():
    generator_cls = getattr(sim, "SyntheticFrameGenerator", None)
    assert generator_cls is not None, "tools.sim must expose SyntheticFrameGenerator for MVP tests"

    kwargs = {
        "camera_ids": ["pi-cam-01", "pi-cam-02"],
        "seed": 11,
        "fps": 118.0,
        "noise_px": 0.0,
        "marker_dropout": 0.0,
        "camera_dropout": 0.0,
        "trajectory": "static",
        "rigid_names": ("waist", "wand"),
        "marker_layout": "current_4marker",
        "camera_rig_source": "dummy",
        "rigids_path": "missing-test-rigids.json",
    }

    signature = inspect.signature(generator_cls)
    accepts_var_kwargs = any(
        parameter.kind == inspect.Parameter.VAR_KEYWORD
        for parameter in signature.parameters.values()
    )
    missing = [
        key
        for key in ("rigid_names", "marker_layout", "camera_rig_source")
        if key not in signature.parameters
    ]
    if missing and not accepts_var_kwargs:
        raise AssertionError(
            "SyntheticFrameGenerator must accept MVP scenario metadata: "
            + ", ".join(missing)
        )
    if not accepts_var_kwargs:
        kwargs = {key: value for key, value in kwargs.items() if key in signature.parameters}
    return generator_cls(**kwargs)


def _ledger_from(generator):
    for name in ("sidecar_ownership_ledger", "ownership_ledger", "sidecar_ledger"):
        if hasattr(generator, name):
            ledger = getattr(generator, name)
            return ledger() if callable(ledger) else ledger
    raise AssertionError("MVP simulator must expose a sidecar ownership ledger")


def _entry_value(entry, key):
    if isinstance(entry, dict):
        return entry[key]
    return getattr(entry, key)


def test_projection_emits_waist_and_wand_blobs_for_two_cameras():
    generator = _make_mvp_generator()

    paired = generator.next_paired()

    assert isinstance(paired, PairedFrames)
    assert set(paired.frames) == {"pi-cam-01", "pi-cam-02"}
    for frame in paired.frames.values():
        assert len(frame.blobs) == 8
        assert frame.timestamp == paired.timestamp
        for blob in frame.blobs:
            assert _BLOB_METADATA_KEYS.isdisjoint(blob)
            assert math.isfinite(float(blob["x"]))
            assert math.isfinite(float(blob["y"]))
            assert float(blob["area"]) > 0.0


def test_sidecar_ledger_tracks_blob_ownership_without_frame_blob_metadata():
    generator = _make_mvp_generator()
    paired = generator.next_paired()
    ledger = _ledger_from(generator)

    assert isinstance(ledger, dict)
    assert len(ledger) == 16

    for camera_id, frame in paired.frames.items():
        rigid_names = []
        marker_indices_by_rigid = {"waist": set(), "wand": set()}
        for emitted_blob_index, blob in enumerate(frame.blobs):
            assert _BLOB_METADATA_KEYS.isdisjoint(blob)
            key = (frame.timestamp, camera_id, emitted_blob_index)
            assert key in ledger
            entry = ledger[key]
            rigid_name = _entry_value(entry, "rigid_name")
            marker_index = int(_entry_value(entry, "marker_index"))
            synthetic_blob_id = _entry_value(entry, "synthetic_blob_id")

            assert rigid_name in {"waist", "wand"}
            assert marker_index in {0, 1, 2, 3}
            assert str(synthetic_blob_id)
            rigid_names.append(rigid_name)
            marker_indices_by_rigid[rigid_name].add(marker_index)

        assert rigid_names.count("waist") == 4
        assert rigid_names.count("wand") == 4
        assert marker_indices_by_rigid == {"waist": {0, 1, 2, 3}, "wand": {0, 1, 2, 3}}


def test_five_rigid_dance_occlusion_emits_four_camera_body_load():
    config = MultiRigidScenarioConfig(
        seed=23,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_dance_occlusion",
        trajectory_name="five_rigid_dance_occlusion",
        camera_rig_source="generated_4cam_from_1_2_intrinsics",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )
    patterns = load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout)
    camera_params = load_camera_rig(config)
    generator = MultiRigidFrameGenerator(
        config,
        patterns=patterns,
        camera_params=camera_params,
    )

    sample = generator.next_sample()

    assert sample is not None
    assert set(sample.paired.frames) == set(DEFAULT_GENERATED_CAMERA_IDS)
    assert set(sample.gt_poses) == set(DEFAULT_BODY_RIGIDS)
    assert len(sample.ownership_ledger) > 0
    owners = {entry.rigid_name for entry in sample.ownership_ledger.values()}
    assert owners == set(DEFAULT_BODY_RIGIDS)
    marker_indices_by_owner = {name: set() for name in DEFAULT_BODY_RIGIDS}
    for entry in sample.ownership_ledger.values():
        assert entry.marker_index in {0, 1, 2, 3, 4}
        marker_indices_by_owner[entry.rigid_name].add(entry.marker_index)
    assert marker_indices_by_owner == {name: {0, 1, 2, 3, 4} for name in DEFAULT_BODY_RIGIDS}
    for pattern in patterns:
        if pattern.name in DEFAULT_BODY_RIGIDS:
            assert pattern.num_markers == 5
            assert pattern.metadata.get("marker_layout") == DESIGN_5MARKER_LAYOUT
            policy = pattern.metadata.get("tracking_policy")
            assert isinstance(policy, dict)
            assert policy.get("boot_min_markers") == 4
            assert policy.get("continue_min_markers") == 3


def test_design_5marker_layout_allows_custom_body_override(tmp_path):
    rigids_path = tmp_path / "tracking_rigids.json"
    custom_head = [
        [0.000, 0.000, 0.000],
        [0.060, 0.000, 0.000],
        [0.000, 0.055, 0.000],
        [0.000, 0.000, 0.050],
        [0.045, 0.040, 0.030],
    ]
    rigids_path.write_text(
        json.dumps(
            {
                "custom_rigids": [
                    {
                        "name": "head",
                        "marker_positions": custom_head,
                        "marker_diameter_m": 0.014,
                        "tracking_policy": {"strong_4_subsets": [[0, 1, 2, 3]]},
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    patterns = load_mvp_patterns(rigids_path, marker_layout=DESIGN_5MARKER_LAYOUT)
    head = next(pattern for pattern in patterns if pattern.name == "head")

    assert head.metadata["source"] == "tracking_rigids"
    assert head.metadata["tracking_policy"]["strong_4_subsets"] == [[0, 1, 2, 3]]
    assert head.metadata["cad_marker_min_z_m"] > 0.0
    assert np.allclose(np.mean(head.marker_positions, axis=0), np.zeros(3))


def test_five_rigid_hard_occlusion_has_two_camera_two_marker_blackout():
    config = MultiRigidScenarioConfig(
        seed=29,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=360,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_dance_hard_occlusion_v1",
        camera_rig_source="generated_4cam_from_1_2_intrinsics",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )
    patterns = load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout)
    camera_params = load_camera_rig(config)
    generator = MultiRigidFrameGenerator(
        config,
        patterns=patterns,
        camera_params=camera_params,
    )

    target_frame = int(round((0.25 + 0.45) * config.fps))
    sample = None
    for _ in range(target_frame + 1):
        sample = generator.next_sample()

    assert sample is not None
    waist_markers_by_camera = {
        camera_id: {
            int(entry.marker_index)
            for entry in sample.ownership_ledger.values()
            if entry.camera_id == camera_id and entry.rigid_name == "waist"
        }
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    assert waist_markers_by_camera["pi-cam-01"] == {0, 2, 3}
    assert waist_markers_by_camera["pi-cam-02"] == {0, 2, 3}
    assert waist_markers_by_camera["pi-cam-03"] == {0, 1, 2, 3, 4}
    assert waist_markers_by_camera["pi-cam-04"] == {0, 1, 2, 3, 4}


def test_five_rigid_hard_occlusion_adds_phase_false_blob_burst():
    config = MultiRigidScenarioConfig(
        seed=31,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=240,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_dance_hard_occlusion_v1",
        camera_rig_source="generated_4cam_from_1_2_intrinsics",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        false_blobs_per_camera=2,
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    target_frame = int(round((0.25 + 0.90) * config.fps))
    sample = None
    for _ in range(target_frame + 1):
        sample = generator.next_sample()

    assert sample is not None
    false_counts = {
        camera_id: sum(
            1
            for entry in sample.ownership_ledger.values()
            if entry.camera_id == camera_id and entry.rigid_name == "__false__"
        )
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    assert false_counts == {camera_id: 8 for camera_id in DEFAULT_GENERATED_CAMERA_IDS}


def test_five_rigid_body_occlusion_models_body_mount_without_false_blobs():
    config = MultiRigidScenarioConfig(
        seed=35,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=260,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="generated_4cam_from_1_2_intrinsics",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        false_blobs_per_camera=4,
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    waist_frame = int(round((0.25 + 0.95) * config.fps))
    foot_frame = int(round((0.25 + 1.24) * config.fps))
    samples = {}
    sample = None
    for frame_index in range(foot_frame + 1):
        sample = generator.next_sample()
        if frame_index in {waist_frame, foot_frame}:
            samples[frame_index] = sample

    assert sample is not None
    assert set(samples) == {waist_frame, foot_frame}
    for checked_sample in samples.values():
        owners = {entry.rigid_name for entry in checked_sample.ownership_ledger.values()}
        assert owners <= set(DEFAULT_BODY_RIGIDS)
        assert "__false__" not in owners
        areas_by_camera = {
            camera_id: [
                float(blob["area"])
                for blob in checked_sample.paired.frames[camera_id].blobs
            ]
            for camera_id in DEFAULT_GENERATED_CAMERA_IDS
        }
        assert all(min(values) > 0.0 for values in areas_by_camera.values())
        assert max(areas_by_camera["pi-cam-02"]) > min(areas_by_camera["pi-cam-02"])
        assert max(areas_by_camera["pi-cam-04"]) > min(areas_by_camera["pi-cam-04"])
        assert sum(areas_by_camera["pi-cam-01"]) / len(areas_by_camera["pi-cam-01"]) > 26.0
        assert sum(areas_by_camera["pi-cam-03"]) / len(areas_by_camera["pi-cam-03"]) > 26.0

    waist_markers_by_camera = {
        camera_id: {
            int(marker_index)
            for entry in samples[waist_frame].ownership_ledger.values()
            if entry.camera_id == camera_id and entry.rigid_name == "waist"
            for marker_index in (
                [entry.marker_index]
                if entry.marker_index is not None
                else [
                    merged_marker
                    for merged_rigid, merged_marker in entry.merged_owners
                    if merged_rigid == "waist"
                ]
            )
        }
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    waist_counts = [len(markers) for markers in waist_markers_by_camera.values()]
    assert min(waist_counts) <= 3
    assert max(waist_counts) >= 4

    foot_counts_by_camera = {
        camera_id: {
            rigid_name: sum(
                1
                for entry in samples[foot_frame].ownership_ledger.values()
                if entry.camera_id == camera_id
                and (
                    entry.rigid_name == rigid_name
                    or any(
                        merged_rigid == rigid_name
                        for merged_rigid, _ in entry.merged_owners
                    )
                )
            )
            for rigid_name in ("left_foot", "right_foot")
        }
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    assert any(counts["left_foot"] < 5 for counts in foot_counts_by_camera.values())
    assert any(counts["right_foot"] < 5 for counts in foot_counts_by_camera.values())
    assert all(
        counts["left_foot"] >= 3 or counts["right_foot"] >= 3
        for counts in foot_counts_by_camera.values()
    )


def test_relaxed_body_occlusion_keeps_foot_markers_visible_in_more_cameras():
    hard_config = MultiRigidScenarioConfig(
        seed=35,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=260,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )
    relaxed_config = MultiRigidScenarioConfig(
        seed=35,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=260,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_relaxed_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )

    def foot_visible_count(config):
        generator = MultiRigidFrameGenerator(
            config,
            patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
            camera_params=load_camera_rig(config),
        )
        target_frame = int(round((0.25 + 1.24) * config.fps))
        sample = None
        for _ in range(target_frame + 1):
            sample = generator.next_sample()
        assert sample is not None
        return sum(
            1
            for entry in sample.ownership_ledger.values()
            if entry.rigid_name in {"left_foot", "right_foot"}
        )

    assert foot_visible_count(relaxed_config) > foot_visible_count(hard_config)


def test_cube_top_camera_rig_aims_four_corners_at_center():
    config = MultiRigidScenarioConfig(
        seed=36,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )

    camera_params = load_camera_rig(config)
    target = [0.0, 0.0, 1.2]

    assert set(camera_params) == set(DEFAULT_GENERATED_CAMERA_IDS)
    for camera in camera_params.values():
        center = -camera.rotation.T @ camera.translation
        assert math.isclose(float(center[2]), 2.4, abs_tol=1e-9)
        assert math.isclose(abs(float(center[0])), 1.2, abs_tol=1e-9)
        assert math.isclose(abs(float(center[1])), 1.2, abs_tol=1e-9)
        target_camera = camera.rotation @ target + camera.translation
        assert math.isclose(float(target_camera[0]), 0.0, abs_tol=1e-9)
        assert math.isclose(float(target_camera[1]), 0.0, abs_tol=1e-9)
        assert float(target_camera[2]) > 0.0


def test_multi_rigid_summary_reports_cluster_radius_override():
    config = MultiRigidScenarioConfig(
        seed=37,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        rigid_names=("waist",),
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        rigid_stabilization_profile="gui_live",
        cluster_radius_m=0.06,
        rigid_stabilization_overrides={"cluster_radius_m": 0.06},
    )

    summary = run_multi_rigid_scenario(config)

    assert summary["cluster_radius_m"] == 0.06
    assert summary["variant_metrics"]["cluster_radius_m"] == 0.06


def test_body_occlusion_blob_area_gently_scales_with_camera_depth():
    config = MultiRigidScenarioConfig(
        seed=38,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
    )
    camera_params = load_camera_rig(config)
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=camera_params,
    )

    camera_id = "pi-cam-02"
    reference_depth = generator._blob_reference_depth_by_camera[camera_id]
    camera = camera_params[camera_id]

    def world_point_at_depth(depth):
        point_camera = np.array([0.0, 0.0, depth], dtype=np.float64)
        return camera.rotation.T @ (point_camera - camera.translation)

    near_area = generator._depth_adjusted_blob_area(
        camera_id,
        11.0,
        world_point_at_depth(reference_depth * 0.70),
    )
    far_area = generator._depth_adjusted_blob_area(
        camera_id,
        11.0,
        world_point_at_depth(reference_depth * 1.30),
    )

    assert near_area > 11.0
    assert far_area < 11.0
    assert near_area > far_area
    assert near_area <= math.pi * (6.4 * 0.5) ** 2
    assert far_area >= math.pi * (2.2 * 0.5) ** 2


def test_pi_snr_marker_detection_model_preserves_high_snr_visible_blobs():
    base_config = dict(
        seed=49,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        marker_dropout_prob=1.0,
    )

    iid_generator = MultiRigidFrameGenerator(
        MultiRigidScenarioConfig(
            **base_config,
            marker_detection_model="iid_dropout",
        ),
        patterns=load_mvp_patterns(base_config["rigids_path"], marker_layout=DESIGN_5MARKER_LAYOUT),
        camera_params=load_camera_rig(
            MultiRigidScenarioConfig(
                **base_config,
                marker_detection_model="iid_dropout",
            )
        ),
    )
    pi_snr_config = MultiRigidScenarioConfig(
        **base_config,
        marker_detection_model="pi_snr",
    )
    pi_snr_generator = MultiRigidFrameGenerator(
        pi_snr_config,
        patterns=load_mvp_patterns(pi_snr_config.rigids_path, marker_layout=pi_snr_config.marker_layout),
        camera_params=load_camera_rig(pi_snr_config),
    )

    iid_sample = iid_generator.next_sample()
    pi_snr_sample = pi_snr_generator.next_sample()

    assert iid_sample is not None
    assert pi_snr_sample is not None
    assert len(iid_sample.ownership_ledger) == 0
    assert len(pi_snr_sample.ownership_ledger) > 0
    assert pi_snr_generator.marker_detection_summary()["model"] == "pi_snr"


def test_body_occlusion_merges_close_projected_blobs():
    config = MultiRigidScenarioConfig(
        seed=47,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=20,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        noise_px=0.05,
        body_mount_blob_merge_factor=0.25,
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    merged_entries = []
    for _ in range(config.frames):
        sample = generator.next_sample()
        assert sample is not None
        merged_entries.extend(
            entry
            for entry in sample.ownership_ledger.values()
            if len(entry.merged_owners) > 1
        )

    assert merged_entries
    assert any(":merged:" in entry.synthetic_blob_id for entry in merged_entries)


def test_mesh_lite_occlusion_requires_body_mount_scenario():
    config = MultiRigidScenarioConfig(
        seed=55,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=80,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_dance_occlusion",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    with pytest.raises(ValueError, match="body-mount scenario"):
        generator = MultiRigidFrameGenerator(
            config,
            patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
            camera_params=load_camera_rig(config),
        )
        generator.next_sample()


def test_mesh_lite_occlusion_adds_body_volume_visibility_loss():
    def emitted_entries(mesh_lite_occlusion: str) -> int:
        config = MultiRigidScenarioConfig(
            seed=55,
            camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
            frames=80,
            fps=118.0,
            rigid_names=DEFAULT_BODY_RIGIDS,
            scenario="five_rigid_body_occlusion_relaxed_v1",
            camera_rig_source="cube_top_2_4m_aim_center",
            marker_layout=DESIGN_5MARKER_LAYOUT,
            rigids_path="missing-test-rigids.json",
            mesh_lite_occlusion=mesh_lite_occlusion,
        )
        generator = MultiRigidFrameGenerator(
            config,
            patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
            camera_params=load_camera_rig(config),
        )
        count = 0
        for _ in range(config.frames):
            sample = generator.next_sample()
            assert sample is not None
            count += len(sample.ownership_ledger)
        return count

    assert emitted_entries("body_capsules") < emitted_entries("off")


def test_mesh_lite_geometry_keeps_each_marker_visible_in_two_cameras():
    config = MultiRigidScenarioConfig(
        seed=61,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=120,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_mesh_lite_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    for frame_index in range(config.frames):
        _poses, marker_world_by_rigid, occluders = generator._marker_positions_world_for_frame(
            frame_index
        )
        for rigid_name in DEFAULT_BODY_RIGIDS:
            for marker_index in range(5):
                visible_cameras = [
                    camera_id
                    for camera_id in DEFAULT_GENERATED_CAMERA_IDS
                    if not generator._is_marker_mesh_lite_occluded(
                        rigid_name,
                        camera_id,
                        marker_world_by_rigid[rigid_name][marker_index],
                        occluders,
                    )
                ]
                assert len(visible_cameras) >= 2


def test_mesh_lite_body_mount_scenario_uses_body_motion():
    config = MultiRigidScenarioConfig(
        seed=62,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=120,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_mesh_lite_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    first = generator._mesh_lite_body_pose_components(0)
    later = generator._mesh_lite_body_pose_components(80)

    moved = [
        float(np.linalg.norm(later[rigid_name][1] - first[rigid_name][1]))
        for rigid_name in DEFAULT_BODY_RIGIDS
    ]
    assert max(moved) > 0.05


def test_mesh_lite_mounts_rigids_on_body_surface_with_z_normal():
    config = MultiRigidScenarioConfig(
        seed=57,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_relaxed_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    body_poses = generator._mesh_lite_body_pose_components(0)
    mounts, occluders = generator._mesh_lite_surface_model(body_poses)
    occluder_by_name = {occluder.name: occluder for occluder in occluders}

    assert set(DEFAULT_BODY_RIGIDS).issubset(set(mounts))
    for mount in mounts.values():
        assert np.isclose(float(np.linalg.norm(mount.normal)), 1.0)
        assert np.allclose(mount.rotation[:, 2], mount.normal)

    head = occluder_by_name["head"]
    assert head.center is not None
    assert np.isclose(
        float(np.linalg.norm(mounts["head"].surface_position - head.center)),
        head.radius_m,
    )

    torso = occluder_by_name["torso"]
    assert torso.a is not None
    assert torso.b is not None
    assert np.isclose(
        float(np.linalg.norm(mounts["chest"].surface_position - torso.a)),
        torso.radius_m,
    )
    assert np.isclose(
        float(np.linalg.norm(mounts["waist"].surface_position - torso.b)),
        torso.radius_m,
    )

    left_leg = occluder_by_name["left_leg"]
    right_leg = occluder_by_name["right_leg"]
    assert left_leg.b is not None
    assert right_leg.b is not None
    assert np.isclose(
        float(np.linalg.norm(mounts["left_foot"].surface_position - left_leg.b)),
        left_leg.radius_m,
    )
    assert left_leg.a is not None
    assert np.allclose(
        left_leg.a - left_leg.b,
        -mounts["left_foot"].normal * float(np.linalg.norm(left_leg.a - left_leg.b)),
    )
    assert np.isclose(
        float(np.linalg.norm(mounts["right_foot"].surface_position - right_leg.b)),
        right_leg.radius_m,
    )
    assert right_leg.a is not None
    assert np.allclose(
        right_leg.a - right_leg.b,
        -mounts["right_foot"].normal * float(np.linalg.norm(right_leg.a - right_leg.b)),
    )


def test_body_mount_pose_is_consistent_with_or_without_mesh_lite_occlusion():
    configs = [
        MultiRigidScenarioConfig(
            seed=57,
            camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
            frames=1,
            fps=118.0,
            rigid_names=DEFAULT_BODY_RIGIDS,
            scenario="five_rigid_body_occlusion_relaxed_v1",
            camera_rig_source="cube_top_2_4m_aim_center",
            marker_layout=DESIGN_5MARKER_LAYOUT,
            rigids_path="missing-test-rigids.json",
            mesh_lite_occlusion=mesh_lite_occlusion,
        )
        for mesh_lite_occlusion in ("off", "body_capsules")
    ]
    marker_world_by_mode = {}
    for config in configs:
        generator = MultiRigidFrameGenerator(
            config,
            patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
            camera_params=load_camera_rig(config),
        )
        _poses, marker_world, _occluders = generator._marker_positions_world_for_frame(0)
        marker_world_by_mode[config.mesh_lite_occlusion] = marker_world

    for rigid_name in DEFAULT_BODY_RIGIDS:
        assert np.allclose(
            marker_world_by_mode["off"][rigid_name],
            marker_world_by_mode["body_capsules"][rigid_name],
        )


def test_body_mount_reference_pose_matches_1_7m_person_scale():
    config = MultiRigidScenarioConfig(
        seed=58,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_relaxed_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    body_poses = generator._mesh_lite_body_pose_components(0)
    positions = {
        rigid_name: np.asarray(position, dtype=np.float64)
        for rigid_name, (_rotation, position) in body_poses.items()
    }

    assert 1.48 <= positions["head"][2] <= 1.62
    assert 1.08 <= positions["chest"][2] <= 1.22
    assert 0.78 <= positions["waist"][2] <= 0.92
    assert 0.05 <= positions["left_foot"][2] <= 0.16
    assert 0.05 <= positions["right_foot"][2] <= 0.16
    foot_separation_m = float(
        np.linalg.norm(positions["left_foot"][:2] - positions["right_foot"][:2])
    )
    assert 0.16 <= foot_separation_m <= 0.32
    assert 0.25 <= positions["chest"][2] - positions["waist"][2] <= 0.40
    assert 0.65 <= positions["waist"][2] - positions["left_foot"][2] <= 0.85


def test_design_5marker_pattern_keeps_cad_fixture_height_metadata():
    patterns = load_mvp_patterns("missing-test-rigids.json", marker_layout=DESIGN_5MARKER_LAYOUT)
    for pattern in patterns:
        if pattern.name not in DEFAULT_BODY_RIGIDS:
            continue
        centroid_z_m = float(pattern.metadata["cad_marker_centroid_z_m"])
        min_z_m = float(pattern.metadata["cad_marker_min_z_m"])
        reconstructed_cad_z = np.asarray(pattern.marker_positions, dtype=np.float64)[:, 2] + centroid_z_m

        assert min_z_m > 0.0
        assert np.isclose(float(np.min(reconstructed_cad_z)), min_z_m)
        assert np.all(reconstructed_cad_z > 0.0)


def test_mesh_lite_markers_remain_outside_own_occluder_surface():
    config = MultiRigidScenarioConfig(
        seed=59,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=1,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_body_occlusion_relaxed_v1",
        camera_rig_source="cube_top_2_4m_aim_center",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        mesh_lite_occlusion="body_capsules",
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    _poses, marker_world_by_rigid, occluders = generator._marker_positions_world_for_frame(0)
    occluder_by_name = {occluder.name: occluder for occluder in occluders}
    own_occluder_by_rigid = {
        "head": occluder_by_name["head"],
        "chest": occluder_by_name["torso"],
        "waist": occluder_by_name["torso"],
        "left_foot": occluder_by_name["left_leg"],
        "right_foot": occluder_by_name["right_leg"],
    }

    def point_segment_distance(point, a, b):
        point = np.asarray(point, dtype=np.float64)
        a = np.asarray(a, dtype=np.float64)
        b = np.asarray(b, dtype=np.float64)
        ab = b - a
        denom = float(np.dot(ab, ab))
        t = 0.0 if denom <= 1e-12 else float(np.clip(np.dot(point - a, ab) / denom, 0.0, 1.0))
        nearest = a + ab * t
        return float(np.linalg.norm(point - nearest))

    min_clearance_m = 0.003
    for rigid_name, marker_points in marker_world_by_rigid.items():
        occluder = own_occluder_by_rigid[rigid_name]
        for point in marker_points:
            if occluder.kind == "sphere":
                assert occluder.center is not None
                distance = float(np.linalg.norm(point - occluder.center))
            else:
                assert occluder.a is not None
                assert occluder.b is not None
                distance = point_segment_distance(point, occluder.a, occluder.b)
            assert distance - float(occluder.radius_m) >= min_clearance_m


def test_five_rigid_swap_red_adds_cross_rigid_alias_blobs():
    config = MultiRigidScenarioConfig(
        seed=37,
        camera_ids=DEFAULT_GENERATED_CAMERA_IDS,
        frames=240,
        fps=118.0,
        rigid_names=DEFAULT_BODY_RIGIDS,
        scenario="five_rigid_dance_swap_red_v1",
        camera_rig_source="generated_4cam_from_1_2_intrinsics",
        marker_layout=DESIGN_5MARKER_LAYOUT,
        rigids_path="missing-test-rigids.json",
        false_blobs_per_camera=4,
    )
    generator = MultiRigidFrameGenerator(
        config,
        patterns=load_mvp_patterns(config.rigids_path, marker_layout=config.marker_layout),
        camera_params=load_camera_rig(config),
    )

    target_frame = int(round((0.25 + 0.90) * config.fps))
    sample = None
    for _ in range(target_frame + 1):
        sample = generator.next_sample()

    assert sample is not None
    alias_entries = [
        entry
        for entry in sample.ownership_ledger.values()
        if ":alias:" in entry.synthetic_blob_id
    ]
    false_entries = [
        entry
        for entry in sample.ownership_ledger.values()
        if entry.rigid_name == "__false__"
    ]
    assert alias_entries
    assert not false_entries
    assert {entry.rigid_name for entry in alias_entries} >= {"left_foot", "right_foot"}
    per_camera_alias_count = {
        camera_id: sum(1 for entry in alias_entries if entry.camera_id == camera_id)
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    assert all(count >= 1 for count in per_camera_alias_count.values())
