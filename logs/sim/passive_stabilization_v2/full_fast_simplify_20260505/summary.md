# full fast simplification probe, 2026-05-05

## Goal

Try the full simple-speedup set without moving to Rust, DBSCAN tuning, or broad threshold tweaks:

- ranked body-level 2D cache
- cached 4-of-5 early-exit support
- thin body-level N-best diagnostics on the GUI route
- raw-scene visualization throttling for live GUI
- skip generic cluster search when CONTINUE already has strong object-gated evidence

## Implemented GUI route

The GUI `rigid_stabilization` payload now enables:

- `body_level_2d_candidate_cache: true`
- `body_level_2d_cache_max_bins: 4`
- `body_level_2d_cache_early_exit: true`
- `body_level_2d_keep_nbest_diagnostics: false`
- `skip_generic_search_when_object_gated: true`
- `raw_scene_interval_frames: 4`

The generic-search skip is restricted to established `CONTINUE` bodies with object-gating enforced, at least two cameras, and at least 4-of-5 equivalent marker evidence. It does not affect BOOT.

## Main comparison

Scenario: `five_rigid_body_mesh_lite_v1`, seed `40`, `240` frames.

| Variant | Min valid | Confusion | Pair p95 ms | Rigid p95 ms | Kabsch-like calls | Right foot rot p95 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `old_formal` | `0.9083` | `0` | `15.96` | `9.09` | `528` | `0.79deg` |
| `ranked_cache_only` | `0.9083` | `0` | `14.76` | `8.37` | `528` | `0.79deg` |
| `diag_thin_raw4` | `0.9083` | `0` | `14.11` | `8.10` | `528` | `0.79deg` |
| `all_fast` | `0.9958` | `0` | `9.09` | `3.74` | `405` | `0.79deg` |

## Verification

Seed `40`, `360` frames:

| Variant | Min valid | Confusion | Pair p95 ms | Rigid p95 ms | Kabsch-like calls | Phase-aware passed |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| `old_formal` | `0.9389` | `0` | `12.64` | `8.04` | `538` | false |
| `all_fast` | `0.9972` | `0` | `9.16` | `3.78` | `405` | true |

Short multi-seed check, frames `180`, seeds `40-42`:

| Variant | Avg min valid | Confusion total | Avg pair p95 ms | Avg rigid p95 ms | Kabsch-like calls total |
| --- | ---: | ---: | ---: | ---: | ---: |
| `old_formal` | `0.9074` | `1` | `20.12` | `13.04` | `1286` |
| `all_fast` | `0.9796` | `0` | `9.17` | `3.71` | `998` |

## Read

The biggest win was not the cache itself; it was avoiding generic cluster/Kabsch search when the body already has strong object-gated CONTINUE evidence. This is still a simple algorithmic reduction rather than a new guard: when body-level 2D evidence is already enough, do not ask generic 3D clustering to re-explain the same body.

Ranked cache and diagnostic thinning are safe incremental wins. Cached early-exit still rarely fires in this scenario, but the ranked cache is ready for harder reacquire cases where the cached offset bins match 4-of-5 earlier.

Raw-scene throttling is a live-GUI optimization. The inprocess sim does not start the GUI raw-scene worker, so the sim timing above mainly measures rigid/geometry route changes.

## Decision

Adopt `all_fast` as the GUI route for the next live check. Keep local ambiguity guards; previous probes did not show a reliable speed win from removing them.
