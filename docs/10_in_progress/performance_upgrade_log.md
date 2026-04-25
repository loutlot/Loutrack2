# Tracking Performance Upgrade Log

## 2026-04-25 fast_ABCD official path

### Goal

- Reduce host-side realtime tracking latency as blob counts, rigid bodies, and cameras increase.
- Keep the previous path available as `baseline` while validating interchangeable fast variants.
- Preserve tracking quality: valid/reacquire/jump/flip/reprojection metrics must not regress.

### Implemented pipeline variants

- `baseline`: previous behavior, retained for replay comparison and rollback.
- `fast_A`: host geometry acceleration.
  - Batch undistort per camera.
  - Vectorized symmetric epipolar cost matrix.
- `fast_AB`: adds stage timing details.
  - `undistort_ms`, `epipolar_match_ms`, `generic_triangulation_ms`, `object_gating_ms`, `rigid_hint_triangulation_ms`, `rigid_pose_ms`, `subset_ms`, `fallback_ms`.
- `fast_ABC`: subset diagnostics sampled instead of every hot-path pair.
  - Keeps committed pose unchanged.
  - Evaluates every 30 subset frames plus invalid/reacquire/risk frames.
- `fast_ABCD`: object gating filters generic triangulation input when safe, with same-frame full generic fallback.
  - Tracker state is updated only once after fast/fallback selection.
  - Reports `fast_path_used_count`, `fallback_count`, `fallback_reason_counts`, `filtered_blob_count`, `full_blob_count`.
- `fast_ABCDE`: latest-only/backpressure simulation for replay/live-ready experimentation.
  - Not selected as official path because simulated drops occurred in replay.

### Official path decision

- The default `TrackingPipeline()` and normal replay path now use `fast_ABCD`.
- `subset_diagnostics_mode` defaults to `sampled` for `fast_ABC` and later.
- `baseline` remains selectable with `--pipeline-variant baseline`.
- `fast_ABCDE` remains experimental because strict replacement requires zero dropped pairs.

### Full replay AB results

#### Current log: `logs/tracking_gui.jsonl`

- Input: 48,800 frames / 24,400 pairs.
- Valid poses: 19,770 -> 19,770.
- `pipeline_pair_ms.mean`: 6.217 -> 3.611 ms, 41.9% faster.
- `pipeline_pair_ms.p95`: 7.830 -> 4.431 ms, 43.4% faster.
- `rigid_ms.mean`: 3.810 -> 1.414 ms, 62.9% faster.
- `rigid_ms.p95`: 5.291 -> 1.684 ms, 68.2% faster.
- `triangulation_ms.mean`: 1.964 -> 1.768 ms, 10.0% faster.
- `triangulation_ms.p95`: 2.154 -> 2.180 ms, 1.2% slower.
- Quality: reacquire count, pose jumps, mode transitions, max flip, max jump, and reprojection p95 were unchanged.
- `fast_path_used_count`: 3,650 / 24,400 pairs, 15.0%.
- `fallback_count`: 20,750.
- Main fallback reasons:
  - `insufficient_object_gating`: 20,584.
  - `filtered_hint_quality_gate`: 166.

#### Archive log: `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`

- Input: 30,452 frames / 15,226 pairs.
- Valid poses: 13,644 -> 13,644.
- `pipeline_pair_ms.mean`: 25.451 -> 4.327 ms, 83.0% faster.
- `pipeline_pair_ms.p95`: 26.591 -> 4.025 ms, 84.9% faster.
- `rigid_ms.mean`: 22.233 -> 1.281 ms, 94.2% faster.
- `rigid_ms.p95`: 23.033 -> 0.762 ms, 96.7% faster.
- `triangulation_ms.mean`: 2.732 -> 2.625 ms, 3.9% faster.
- `triangulation_ms.p95`: 2.994 -> 2.876 ms, 4.0% faster.
- Quality: no regression.
- `fast_path_used_count`: 13,230 / 15,226 pairs, 86.9%.

#### Archive log: `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`

- Input: 3,272 frames / 1,636 pairs.
- Valid poses: 1,469 -> 1,469.
- `pipeline_pair_ms.mean`: 23.988 -> 4.243 ms, 82.3% faster.
- `pipeline_pair_ms.p95`: 25.072 -> 4.073 ms, 83.8% faster.
- `rigid_ms.mean`: 20.814 -> 1.225 ms, 94.1% faster.
- `rigid_ms.p95`: 21.835 -> 1.206 ms, 94.5% faster.
- `triangulation_ms.mean`: 2.727 -> 2.618 ms, 4.0% faster.
- `triangulation_ms.p95`: 3.050 -> 2.911 ms, 4.6% faster.
- Quality: no regression.
- `fast_path_used_count`: 1,315 / 1,636 pairs, 80.4%.

### Outlier investigation

- `fast_ABCD` improves mean/p95 latency, but current log replay had occasional max-latency outliers.
- Trace replay showed three sources:
  - Python/OS scheduling gaps: `pipeline_pair_ms` jumped while internal stage total stayed low.
  - Sampled subset diagnostics spikes: e.g. `subset_ms` around 20-30 ms on risk/interval frames.
  - Non-repeatable triangulation spikes likely caused by runtime scheduling rather than deterministic log content.
- Tracking quality did not change during these spikes.
- If worst-case latency becomes critical, the next target is subset diagnostics budgeting or a `fast_ABCD_no_subset` comparison path.

### Verification commands

- `PYTHONPATH=src .venv/bin/python -m pytest tests/test_geo_blob_assignment.py tests/test_rigid_reprojection_scoring.py tests/test_tracking_pipeline_diagnostics.py tests/test_tracking_replay_harness.py -q`
  - Result: 50 passed.
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --log logs/tracking_gui.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --pipeline-variant baseline --out logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline.json --quiet`
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --log logs/tracking_gui.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --pipeline-variant fast_ABCD --subset-diagnostics-mode sampled --out logs/archive/fast_pipeline_ab_20260425_full_abcd/fast_ABCD.json --quiet`

### Generated reports

- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/fast_ABCD.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline_archive_baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline_archive_fast_ABCD.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/new_archive_baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/new_archive_fast_ABCD.json`
