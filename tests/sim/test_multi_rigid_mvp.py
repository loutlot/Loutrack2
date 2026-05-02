import inspect
import math
import sys
from pathlib import Path

import numpy as np

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
