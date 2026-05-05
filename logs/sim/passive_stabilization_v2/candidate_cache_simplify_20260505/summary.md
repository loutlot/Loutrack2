# candidate cache and simplification probe, 2026-05-05

## Goal

Try the simpler performance route proposed after the clustering probe:

- keep current 2D body-level N-best
- avoid changing DBSCAN
- cache body-level candidate bins per body/camera
- reuse cached bins during reacquire
- early-exit when cached evidence reaches 4-of-5
- probe whether local ambiguity guards can be removed without accuracy loss

## Implementation

`ObjectGatingConfig` now has:

- `body_level_2d_candidate_cache`
- `body_level_2d_cache_early_exit`

The cache stores the median 2D translation bin from strong body assignments. During `REACQUIRE`, the cached bin is tried before the normal translated-offset search. If the cached bin reaches 4-of-5, the search can return early.

## Single-seed result

Scenario: `five_rigid_body_mesh_lite_v1`, seed `40`, `240` frames, formal GUI route.

| Variant | Min valid | Confusion | Pair p95 ms | Rigid p95 ms | Cache hit | Early exit | Right foot rot p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cache off | `0.9083` | `0` | `15.37` | `8.98` | `0` | `0` | `0.79deg` |
| cache on | `0.9083` | `0` | `14.11` | `8.36` | `16` | `0` | `0.79deg` |
| no close-blob guard | `0.9083` | `0` | `16.95` | `11.23` | `16` | `0` | `0.79deg` |

The cache helped in this run without changing quality. The early-exit condition did not fire, so the useful part is candidate-bin ordering/reuse rather than hard search termination.

## Multi-seed read

Shorter `160` frame runs over seeds `40-42` showed identical validity and confusion for cache on/off, but timing was noisy:

- cache off: average pair p95 `41.35ms`, rigid p95 `33.49ms`
- cache on: average pair p95 `49.35ms`, rigid p95 `37.47ms`
- no marker-margin guard: average pair p95 `48.28ms`, rigid p95 `36.97ms`
- no close-blob guard: average pair p95 `56.02ms`, rigid p95 `41.76ms`

Quality did not change, but removing local ambiguity guards did not produce a reliable speed win. The close-blob guard should stay.

## Final 360-frame check

Seed `40`, `360` frames:

| Variant | Right foot valid | Confusion | Pair p95 ms | Rigid p95 ms | Cache hit | Right foot rot p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| cache off | `0.9389` | `0` | `16.25` | `8.16` | `0` | `0.79deg` |
| cache on | `0.9389` | `0` | `13.36` | `8.09` | `16` | `0.79deg` |

## Decision

Keep body-level 2D N-best and enable candidate-bin cache in the GUI route. Do not remove local ambiguity guards yet. The cache is simple and quality-neutral in these tests; the guard removals are not clearly worth it.

Next simplification target: make the body-level 2D cache early-exit useful by caching a small ranked bin set, not just one median offset.
