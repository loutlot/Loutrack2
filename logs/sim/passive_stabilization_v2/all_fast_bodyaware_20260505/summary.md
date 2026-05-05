# all_fast body-aware fallback check

## Setup

- Frames: 240
- FPS: 118
- Seeds: 40, 41, 42
- Rigids: `calibration/tracking_rigids.json`
- Marker layout: `design_5marker_seed`
- Profile: `gui_live`
- Camera rig: `cube_top_2_4m_aim_center`
- Marker model: `pi_snr`
- Centroid noise: `diameter_scaled`, `0.15px` at `4.0px`
- Body-mount blob merge factor: `0.75`

## Aggregate Results

| Scenario | Avg min valid | Confusion sum | Avg pair p95 ms | Avg rigid p95 ms | Avg estimate calls | Generic skip sum | Body-aware fallback used |
|---|---:|---:|---:|---:|---:|---:|---:|
| `body_occlusion` | 0.7903 | 37 | 13.81 | 7.95 | 470.3 | 30 | 0 |
| `dance_hard` | 0.9514 | 38 | 10.56 | 4.26 | 306.0 | 0 | 0 |

## Read

The controlled `all_fast` route remains safe on the strict body-occlusion check: confusion stays at the safe profile from `all_fast_on_no_skip`, while rigid p95 stays below the old full fallback route.

The new body-aware 3D fallback path did not fire in these two strict scenarios. That is acceptable for this validation: the common path is already covered by full 4-of-5 rigid hints or controlled generic skip. The unit test covers the intended 3-of-5 CONTINUE case where body-level 2D hints can produce a pose before global cluster search.

GUI/status object-gating diagnostics are now compact by default and keep detailed per-camera assignment lists only for sampled, non-CONTINUE, or ambiguous frames.
