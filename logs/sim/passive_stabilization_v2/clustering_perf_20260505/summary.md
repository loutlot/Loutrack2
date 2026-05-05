# layered clustering performance comparison, 2026-05-05

## Goal

Compare whether layered clustering can reduce the passive `candidate_00534` GUI-route load without losing generality or creating threshold-like overfit.

## Common scenario

- Scenario: `five_rigid_body_mesh_lite_v1`
- Frames: `240`
- Seed: `40`
- Camera rig: `cube_top_2_4m_aim_center`
- Rigid shape source: `calibration/tracking_rigids.json`
- Marker layout: `design_5marker_seed`
- Stabilization: GUI live formal route
- Stress: mesh-lite body capsules, body blob merge factor `0.75`, Pi SNR detection, diameter-scaled centroid noise at `0.15px`

## Variants

| Variant | Meaning |
| --- | --- |
| `formal_baseline` | Current GUI route: body-level 2D N-best plus 3D DBSCAN radius `0.08m`. |
| `no_2d_body_cluster` | Disable body-level 2D recovery. |
| `2d_tight_nearest1` | Keep body-level 2D but allow only one nearest candidate per marker. |
| `2d_wide_cols3` | Keep body-level 2D and widen assignment candidates to three nearest blobs. |
| `2d_offsets6_cols2` | Keep body-level 2D but reduce translated offsets from `8` to `6`. |
| `3d_tight_radius006` | Keep 2D route and shrink generic 3D DBSCAN radius to `0.06m`. |
| `3d_wide_radius010` | Keep 2D route and widen generic 3D DBSCAN radius to `0.10m`. |
| `3d_wider_radius014` | Keep 2D route and widen generic 3D DBSCAN radius to `0.14m`. |
| `3d_global_radius100` | Approximate no useful 3D clustering by putting all generic points in one cluster. |

## Results

| Variant | Min valid | Right foot valid | Confusion | Pair p95 ms | Rigid p95 ms | Mean clusters | Kabsch-like calls | Right foot rot p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| `formal_baseline` | `0.9083` | `0.9083` | `0` | `14.79` | `8.59` | `4.84` | `528` | `0.79deg` |
| `no_2d_body_cluster` | `0.7750` | `0.7750` | `1` | `12.23` | `7.96` | `4.13` | `574` | `146.15deg` |
| `2d_tight_nearest1` | `0.7042` | `0.7042` | `20` | `12.86` | `7.88` | `4.20` | `566` | `176.00deg` |
| `2d_wide_cols3` | `0.9083` | `0.9083` | `0` | `13.72` | `8.20` | `4.84` | `528` | `0.79deg` |
| `2d_offsets6_cols2` | `0.9083` | `0.9083` | `0` | `14.02` | `8.28` | `4.84` | `528` | `0.79deg` |
| `3d_tight_radius006` | `0.0000` | `0.9500` | `0` | `11.09` | `4.04` | `2.32` | `936` | `0.77deg` |
| `3d_wide_radius010` | `0.6583` | `0.6583` | `62` | `14.36` | `8.53` | `4.13` | `290` | `169.04deg` |
| `3d_wider_radius014` | `0.6542` | `0.6542` | `62` | `16.44` | `11.05` | `4.05` | `538` | `169.09deg` |
| `3d_global_radius100` | `0.5958` | `0.5958` | `233` | `340.12` | `335.02` | `1.00` | `7135` | `168.31deg` |

## Read

The only safe candidates were `formal_baseline`, `2d_wide_cols3`, and `2d_offsets6_cols2`. The speed differences among those safe routes are small. `2d_wide_cols3` was the fastest in this 240-frame run, but the previous 20-cycle PDCA showed columns `2` was better in the longer verification. So this run does not justify changing the GUI default.

The unsafe variants are informative:

- Removing body-level 2D recovery makes timing look slightly better, but right foot ownership/rotation collapses.
- Tightening 2D candidates to nearest `1` is too brittle; it creates marker-source confusion and large rotation flips.
- Shrinking 3D DBSCAN radius to `0.06m` cuts rigid p95, but head never boots. That is not acceptable.
- Widening 3D DBSCAN merges unrelated body evidence and causes right-foot flips.
- Treating the whole scene as one 3D cluster explodes Kabsch-like calls and is unusable.

## Decision

Do not change the formal GUI route from the current layered setting. The useful part is already present: 2D whole-body N-best before triangulation plus conservative 3D DBSCAN. More clustering by itself does not buy safe performance.

The next promising route is not another hard cluster radius. It is a body-local candidate cache/early-exit:

- keep current 2D body-level N-best
- keep 3D DBSCAN radius `0.08m`
- cache per-body candidate bins from predicted marker rays/3D hints
- early-exit once a body has enough phase-aware evidence for a valid 4-of-5 or stable 3-of-5 continuation

That is still a physical-layer search reduction, not a narrow guard.
