# all_fast strict scenario comparison

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

`all_fast_off` disables:

- `body_level_2d_candidate_cache`
- `body_level_2d_cache_early_exit`
- `skip_generic_search_when_object_gated`
- `raw_scene_interval_frames=1`

`all_fast_on` keeps the current GUI fast route.

## Aggregate Results

| Scenario | Variant | Avg min valid | Confusion sum | Avg pair p95 ms | Avg rigid p95 ms | Avg estimate calls |
|---|---:|---:|---:|---:|---:|---:|
| `body_occlusion` | `all_fast_off` | 0.7903 | 37 | 15.33 | 9.07 | 508.3 |
| `body_occlusion` | `all_fast_on` | 0.8861 | 232 | 15.02 | 6.82 | 386.3 |
| `body_occlusion` | `all_fast_on_no_skip` | 0.7903 | 37 | 14.19 | 8.31 | 508.3 |
| `body_occlusion` | `all_fast_on_skip_control` | 0.7903 | 37 | 13.78 | 7.92 | 470.3 |
| `dance_hard` | `all_fast_off` | 0.9514 | 42 | 10.89 | 5.46 | 309.3 |
| `dance_hard` | `all_fast_on` | 0.9722 | 24 | 9.63 | 3.71 | 287.3 |
| `dance_hard` | `all_fast_on_skip_control` | 0.9514 | 38 | 14.00 | 5.22 | 306.0 |

## Read

`dance_hard` is green for `all_fast_on`: validity improves, confusion drops, and rigid p95 improves by about 32%.

`body_occlusion` is not green for the full `all_fast_on` route. It raises min-valid substantially, but marker-source confusion jumps from 37 to 232 across the three seeds. The component probe shows that turning only `skip_generic_search_when_object_gated` back off restores the old confusion profile exactly while keeping a small pair-frame speed gain from the non-risky fast-path pieces.

The revised `all_fast_on_skip_control` route keeps `skip_generic_search_when_object_gated` enabled in the GUI path, but only fires it when every marker has two-camera body-level evidence and the body assignment has no ambiguous, marker-margin, or duplicate assignment. On `body_occlusion`, that restores the safe confusion profile while still reducing rigid p95 from `9.07ms` to `7.92ms`.

## Decision

Adopt the controlled `all_fast` route as the formal GUI path. The GUI still keeps `skip_generic_search_when_object_gated` enabled, but the skip only fires on complete, unambiguous whole-body 2D evidence. The candidate-bin cache, reduced diagnostics, raw-scene throttling, and the controlled generic-search skip are the accepted route.

Do not revive the old aggressive skip condition. It was the unsafe part of the earlier full `all_fast_on` route.

The strict failure mode was not a threshold issue; it was an ownership fallback issue during hard body occlusion.
