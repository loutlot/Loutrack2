# body-aware fallback deep dive

## Question

Check whether the latest optimization actually matters beyond being structurally plausible:

- Limit generic fallback to bodies that need it.
- Feed body-level 2D assignment into 3D fallback.
- Sample or compact GUI diagnostics.

## Scenario Sweep

Common setup:

- Frames: 240
- FPS: 118
- Seeds: 40, 41, 42
- Rigids: `calibration/tracking_rigids.json`
- Marker layout: `design_5marker_seed`
- Profile: `gui_live`
- Camera rig: `cube_top_2_4m_aim_center`
- Marker model: `pi_snr`
- Centroid noise: `diameter_scaled`, `0.15px` at `4.0px`

| Scenario | Variant | Avg min valid | Confusion sum | Avg pair p95 ms | Avg rigid p95 ms | Avg estimate calls | Generic skip sum | Body-aware fallback used |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| `body_occlusion` | `current_gui` | 0.7903 | 37 | 13.86 | 8.01 | 470.3 | 30 | 0 |
| `body_occlusion` | `temporal_off` | 0.7903 | 37 | 13.67 | 7.94 | 470.3 | 30 | 0 |
| `body_occlusion` | `temporal_off_no_skip` | 0.7903 | 37 | 13.98 | 8.12 | 508.3 | 0 | 0 |
| `dance_hard` | `current_gui` | 0.9514 | 38 | 10.93 | 4.39 | 306.0 | 0 | 0 |
| `dance_hard` | `temporal_off` | 0.9458 | 34 | 8.88 | 2.85 | 303.7 | 0 | 1 |
| `dance_hard` | `temporal_off_no_skip` | 0.9458 | 34 | 9.13 | 2.91 | 303.7 | 0 | 1 |

## Read

### Generic fallback limiting

This is meaningful. On `body_occlusion`, controlled generic skip fires 30 times across three seeds and reduces average estimate calls from `508.3` to `470.3` versus `temporal_off_no_skip`, without increasing confusion. This is the strongest confirmed optimization in this group.

### Body-aware hint fallback

This is not yet meaningful in the current GUI route. It fired `0` times with `current_gui` across these strict scenarios. When `temporal_body_nbest` is disabled, it fired once in `dance_hard seed41`, which proves the fallback can work, but also shows it is mostly shadowed by the stronger temporal body N-best path.

Current interpretation: body-aware hint fallback is a narrow 3-of-5 CONTINUE insurance path, not a proven production-speed improvement. It is not part of the formal GUI route.

### Diagnostics compacting

This is meaningful for GUI/status serialization. A representative 5-rigid object-gating payload went from about `33.7KB` to `9.3KB`, and JSON serialization for 2,000 snapshots dropped from about `428ms` to `82ms`. The compacting function itself is not faster than a shallow dict copy, so the benefit is specifically payload size and serialization/UI transport, not pure Python object copying.

## Decision

Keep controlled generic skip and compact diagnostics as formal optimizations.

Do not adopt body-aware hint fallback into the production route. If this path is revisited later, it should live behind an explicit experimental flag and be justified by real-log hit rate.
