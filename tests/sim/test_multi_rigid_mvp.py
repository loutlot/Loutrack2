import inspect
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
assert SRC.exists(), f"Source path not found: {SRC}"
sys.path.insert(0, str(SRC))

from tools import sim  # type: ignore
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
