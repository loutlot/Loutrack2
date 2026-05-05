# hot path estimate

## Scope

Estimate remaining optimization room for:

- raw scene visualization separation
- Kabsch / candidate-scoring small-array optimization
- per-body parallelization

Primary strict reference:

- Scenario: `five_rigid_body_occlusion_v1`
- Seed: `40`
- Frames: `240` for stage metrics, `160` for `cProfile`
- Profile: `gui_live`
- Rigids: `calibration/tracking_rigids.json`
- Marker layout: `design_5marker_seed`
- Camera rig: `cube_top_2_4m_aim_center`
- Marker model: `pi_snr`

## Current Stage Shape

From the current formal path on `body_occlusion seed40`:

| Metric | Mean ms | P95 ms |
|---|---:|---:|
| `pipeline_pair_ms` | 10.84 | 14.45 |
| `rigid_ms` | 4.70 | 8.26 |
| `geometry_ms` | 1.64 | 2.20 |

`cProfile` on the same style of run shows the largest cumulative buckets:

| Function bucket | Cum time over 160 frames | Read |
|---|---:|---|
| `RigidBodyEstimator._process_points` | 3.32s | main rigid tracking work |
| `RigidBodyEstimator._try_multi_pattern_boot_candidate` | 1.47s | rare/cold boot candidate spike |
| `RigidBodyEstimator.evaluate_object_conditioned_gating` | 1.18s | per-body 2D body assignment/gating |
| `RigidBodyEstimator.estimate_pose` | 1.13s | Kabsch + correspondence wrapper |
| `KabschEstimator.estimate` | 1.09s | many tiny rigid solves, heavily affected by candidate enumeration |
| `RigidBodyEstimator._score_pose_reprojection` | 0.70s | camera projection + dict/list scoring payload |
| `GeometryPipeline.process_paired_frames` | 0.40s | current fast/hint geometry path |

## Estimate

### Raw Scene Separation

The current simulator does not start the raw-scene worker, so stage p95 does not include that GUI visualization lane. A direct baseline raw-scene triangulation benchmark on the same frames averaged:

| Raw-scene frequency | CPU cost estimate |
|---|---:|
| every frame | 14.15ms/frame |
| every 4 frames, current 118/4 = 29.5fps | 3.54ms amortized per pose frame |
| every 8 frames, 14.75fps | 1.77ms amortized per pose frame |
| every 12 frames, 9.8fps | 1.18ms amortized per pose frame |

This is mostly separate-worker CPU, not direct main-thread tracking latency. If the display really needs 20-30fps, `raw_scene_interval_frames=4` is already the correct range. The remaining win is to ensure GUI/status serialization never sends full raw-scene data at 118fps.

Expected gain:

- Main tracking p95: likely `<0.2ms` unless CPU contention is currently visible.
- Total CPU budget: `1-2.4ms` amortized per pose frame if raw scene is reduced from 29.5fps to 10-15fps.
- Best next check: measure real GUI polling/send payload rate, not only sim.

### Kabsch / Candidate Scoring

The hot part is not one Kabsch solve. It is many tiny Kabsch/correspondence/scoring calls plus dict/list payload construction.

Important split:

- `KabschEstimator.estimate` is visible, but much of its cost comes from candidate enumeration.
- `_score_pose_reprojection` and object-gating projection/scoring are comparable targets.
- `_try_multi_pattern_boot_candidate` is a rare but very large spike. It matters for boot/reacquire spikes more than steady-state 118fps.

Expected gain:

- Conservative steady-state: `0.5-1.5ms` off rigid p95 if scoring/projection payloads are array-backed and diagnostics stay compact.
- Cold boot/reacquire spikes: potentially much larger if boot candidate enumeration is capped or ranked earlier.
- Optimizing only the tiny SVD/Kabsch kernel is probably small: likely `<0.3ms` p95 by itself.

Best next check:

- Add built-in timers for `object_gating`, `reprojection_score`, `estimate_pose`, and `boot_candidate` in `variant_metrics`, then optimize only the largest bucket.

### Per-Body Parallelization

The apparent parallelizable surface is the per-rigid work inside object gating and pose candidate generation. The theoretical ceiling is attractive because `_process_points` dominates the strict profile.

Practical caveat:

- The work uses many Python dict/list loops and shared tracker/candidate ownership state.
- Threading may not help much under the GIL.
- Process pools would copy too much small state unless the data model is first converted to compact arrays.

Expected gain:

- Python threads as-is: `0-10%`, with real risk of getting slower.
- After array/Rust-backed candidate scoring: `10-25%` may become realistic.
- Theoretical perfect 5-body split is higher, but not reachable without changing the data model and conflict-resolution boundary.

## Recommendation

Order of attack:

1. Keep raw scene at 20-30fps, but verify GUI/status is not serializing raw scene at 118fps.
2. Add fine-grained timers for candidate scoring and object-gating sub-buckets.
3. Optimize `_score_pose_reprojection` and object-gating payload/projection reuse before touching Kabsch internals.
4. Treat per-body parallelization as a later move, after the hot path is array-shaped enough to benefit.

Near-term expected practical win without risky architecture work: about `0.5-1.5ms` rigid p95 plus reduced GUI/CPU load from raw-scene transport discipline.
