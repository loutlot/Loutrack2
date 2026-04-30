import inspect
import math
import sys
from pathlib import Path

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
        false_blobs_per_camera=1,
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
    assert alias_entries
    assert {entry.rigid_name for entry in alias_entries} >= {"left_foot", "right_foot"}
    per_camera_alias_count = {
        camera_id: sum(1 for entry in alias_entries if entry.camera_id == camera_id)
        for camera_id in DEFAULT_GENERATED_CAMERA_IDS
    }
    assert per_camera_alias_count == {camera_id: 3 for camera_id in DEFAULT_GENERATED_CAMERA_IDS}
