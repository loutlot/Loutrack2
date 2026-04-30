"""Synthetic tracking simulator development tools."""

from .multi_rigid import (  # noqa: F401
    DESIGN_5MARKER_LAYOUT,
    MultiRigidFrameGenerator,
    MultiRigidScenarioConfig,
    SyntheticFrameGenerator,
    assert_metrics,
    load_camera_rig,
    load_mvp_patterns,
    run_closed_loop,
    run_multi_rigid_scenario,
    verify_frame_log,
)
