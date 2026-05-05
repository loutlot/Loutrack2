# candidate_00534 performance PDCA, 2026-05-05

## Goal

Adopt `candidate_00534` as the formal body rigid shape and improve runtime performance without relying on narrow threshold tuning. The preferred route is passive physical-layer leverage from the 5-marker body geometry plus whole-body 2D N-best assignment.

## Scenario

- Rigid shape: `candidate_00534`, copied into `calibration/tracking_rigids.json`.
- Stress: 4 camera, 5 rigid body-mount mesh-lite occlusion, Pi SNR detection, diameter-scaled centroid noise, no active anchor route.
- Baseline route: GUI live stabilization, passive temporal body N-best, body-level 2D recovery.
- Main score: zero ownership confusion first, then valid-frame ratio, rotation/position error, and timing.
- Timing target: 118 fps implies about `8.47ms` per pair on average. P95 should be read as the remaining spike budget.

## Sequential cycles

| Cycle | Hypothesis | Result | Decision |
| --- | --- | --- | --- |
| 1 | Baseline body-level 2D N-best is the quality floor. | right_foot valid `0.9125`, confusion `0`, pair p95 `14.03ms`, rigid p95 `8.57ms`. | Keep as reference. |
| 2 | Fewer translated offsets may reduce cost. | max offsets `8` kept quality, pair p95 `13.32ms`. | Keep exploring smaller search. |
| 3 | max offsets `6` may preserve quality. | quality held, pair p95 `13.57ms`. | No clear win over cycle 2. |
| 4 | max offsets `4` may be enough. | right_foot valid fell to `0.8875`. | Reject. |
| 5 | nearest-per-marker `1` may shrink candidate work. | quality held, pair p95 worsened to `15.86ms`. | Reject. |
| 6 | trigger body recovery only at `<=2` assignments. | one marker-source confusion appeared. | Reject. |
| 7 | trigger only at `<=1` assignments. | right_foot valid `0.6417`, right_foot rot p95 `142.62deg`. | Reject hard. |
| 8 | lower distance scale `0.08` may avoid broad matching. | right_foot valid `0.8750`. | Reject. |
| 9 | distance scale `0.10` may be safer than `0.08`. | right_foot valid still `0.8750`. | Reject. |
| 10 | positive body-level margin may prevent marginal adoption. | confusion `3`, right_foot rot p95 `93.72deg`. | Reject. |
| 11 | Reduce assignment columns to nearest `3` per marker. | quality held, pair p95 `14.34ms`. | Not enough. |
| 12 | Reduce assignment columns to nearest `2` per marker. | quality held, pair p95 `13.82ms`, rigid p95 `8.36ms`. | Viable. |
| 13 | Combine offsets `6` and columns `2`. | quality held, pair p95 `13.64ms`, rigid p95 `8.36ms`. | Viable, but not best. |
| 14 | Combine offsets `4` and columns `2`. | right_foot valid fell to `0.8875`. | Reject. |
| 15 | Disable body-level 2D recovery to measure value. | confusion `1`, right_foot valid `0.7750`, rot p95 `146.15deg`. | Body-level 2D is required. |
| 16 | Disable temporal body N-best to isolate cost. | quality held, pair p95 `15.94ms`, rigid p95 `9.51ms`. | Temporal body N-best remains useful. |
| 17 | Remove both pose and position continuity guards. | quality held, pair p95 `13.25ms`, rigid p95 `8.18ms`. | Good speed, but less conservative. |
| 18 | Disable pose guard only; keep position guard. | quality held, pair p95 `13.77ms`, rigid p95 `8.12ms`. | Best balanced runtime path. |
| 19 | Disable position guard only; keep pose guard. | quality held, pair p95 `14.99ms`, rigid p95 `9.82ms`. | Reject. |
| 20 | Use pose-off route but widen assignment columns to `3`. | quality held, pair p95 `16.74ms`, rigid p95 `9.94ms`. | Reject. |

## Adopted route

Use `candidate_00534` with body-level 2D recovery enabled, translated offsets `8`, nearest-per-marker `2`, assignment nearest-per-marker `2`, trigger max assignments `3`, distance scale `0.12`, pose-continuity guard disabled, and position-continuity guard enabled.

Final 360-frame verification:

- valid ratio: head/chest/waist/left_foot `1.0`, right_foot `0.9389`
- ownership confusion: `0`
- pair timing: mean `7.21ms`, p95 `12.19ms`
- rigid timing: mean `2.33ms`, p95 `7.70ms`
- right_foot accuracy while valid: rotation p95 `0.79deg`, position p95 `0.62mm`
- phase-aware result: full 3D hint measurement passes; hidden-only prediction hold is not counted as recovery

## Read

The performance win did not come from narrower thresholds. It came from making body-level 2D recovery real enough to keep the passive geometry path alive, then removing pose-continuity guarding that had become redundant for this candidate. The remaining issue is not mean speed; the mean pair time is inside the 118 fps budget. The remaining issue is reacquire spikes, visible in pair p95 and max latency.

## Next

Optimize the reacquire hot path specifically: cache/reuse body-level N-best work across cameras and bodies, and add a bounded early-exit when the same body hypothesis already has enough phase-aware 2D evidence.
