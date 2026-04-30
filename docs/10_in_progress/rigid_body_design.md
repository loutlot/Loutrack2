# Rigid Body Design PDCA Log

## 2026-04-30 - Current Best Design / Handoff Notes

### Executive Summary

The current best direction is **5 markers per rigid body + mode-specific acceptance + subset whitelist**.

2026-04-30 implementation PDCA status:

- `tools/sim/` now maps `five_rigid_dance_occlusion` to the exploratory 5-marker seed by default via `design_5marker_seed`.
- Runtime pose acceptance now reads `tracking_policy` metadata and rejects non-whitelisted 5-marker subsets in BOOT / REACQUIRE / CONTINUE.
- `fast_ABCDHRF` still owns the GUI path, but the 5-rigid stress scenario is now a design-validation route, not a 4-marker stress placeholder.
- Accuracy remained stable across the 5-rigid stress: all five rigids held `valid_frame_ratio = 1.0`, `wrong_ownership_count = 0`, and `marker_source_confusion_count = 0`.
- Performance improved substantially from the first 5-rigid baseline, but production latency is not yet fully cleared because startup spikes still exceed the strict pair budget: best balanced 120-frame repeat is cycle 45 at pair mean `4.27ms`, pair p95 `4.18ms`, rigid p95 `1.49ms`, geometry p95 `1.44ms`.
- Cycles 31-40 accepted the simpler lightweight per-frame event-diagnostics path for GUI event recording. Cycles 41-50 then moved the remaining hot-path work to object-gating and rigid-hint lookup reuse, improving pair p95 while preserving accuracy.
- The current adoption decision is: accept the 5-marker subset-policy route and lightweight event-diagnostics cleanup into GUI/sim development, keep performance PDCA open before calling it production-ready for 118fps 5-rigid live tracking.

The physical design problem is not fully solved as a final manufacturable coordinate set, but the engineering direction is now clear enough for the next implementer:

- Do not continue with the existing built-in 4-marker patterns as-is. They are geometrically ambiguous.
- Do not switch to 5 markers and accept arbitrary `3-of-5` / `4-of-5` subsets. That creates weak partial-pattern matches.
- Use 5 markers as the canonical body definition, but only commit poses from precomputed strong subsets.
- Treat `3-of-5` as a **prediction confirmation signal**, not as an independent boot/reacquire pose source.

Recommended runtime policy:

```text
BOOT:
  require 4-of-5 visible markers
  accepted 4-subset must be in strong_4_subsets
  reject candidates without 2D reprojection support
  require an ambiguity margin over the second-best body/assignment

REACQUIRE:
  require 4-of-5 visible markers
  accepted 4-subset must be in strong_4_subsets
  require agreement with recent prediction if recently lost
  treat long-loss recovery as BOOT

CONTINUE:
  accept whitelisted 4-of-5 normally
  allow whitelisted 3-of-5 only under prediction + 2D reprojection gates
  weak or non-whitelisted subset -> hold predicted pose, increase uncertainty, decay confidence
```

### Sim / Performance PDCA

Sim を使った performance score、ログの残し方、cycle 1-50 の詳細は `docs/10_in_progress/sim_performance_upgrade_log.md` に移した。この文書では marker geometry、subset whitelist、runtime policy の設計判断を扱う。

Existing built-in 4-marker patterns:

| metric | built-in range / worst |
|---|---:|
| self symmetry RMS | `0.29-9.06mm` |
| worst cross-pattern RMS | `6.57mm` |
| closest 3-marker subset delta | as low as `0.34mm` |

Previous balanced 4-marker candidate:

| metric | value |
|---|---:|
| worst self symmetry RMS | `20.53mm` |
| worst intra 3-marker delta | `14.28mm` |
| worst cross-pattern RMS | `14.65mm` |
| worst cross 3-marker profile delta | `8.20mm` |

5-marker exploratory runs:

| metric | observed range / best intermediate |
|---|---:|
| full `5-vs-5` cross | typically `16-22mm` |
| arbitrary `4-of-5` cross | can fall to `6-8mm` |
| arbitrary `3-of-5` profile cross | can fall to `4-5mm` |
| best whitelisted `4-of-5` cross seen | `13.71mm` |
| best whitelisted `3-of-5` profile seen | `9.92mm` |
| best targeted-search marker spacing seen | `40.33mm` minimum |

Interpretation:

- Full 5-marker observations are stronger than the 4-marker baseline.
- Arbitrary subsets remain unsafe.
- Whitelisted subsets recover much of the benefit while keeping runtime behavior explainable.
- The current numbers are design-direction evidence, not a final physical coordinate release.

### Best Candidate Policy

Use `strong_4_subsets` and `strong_3_subsets` as pattern metadata.

Suggested metadata shape:

```json
{
  "name": "waist",
  "marker_positions": [[...], "..."],
  "marker_diameter_m": 0.014,
  "tracking_policy": {
    "boot_min_markers": 4,
    "reacquire_min_markers": 4,
    "continue_min_markers": 3,
    "strong_4_subsets": [[0, 1, 2, 4]],
    "strong_3_subsets": [[1, 2, 4]]
  }
}
```

Do not let the runtime infer that every subset of the same size is equivalent. The whitelist is part of the design.

### Coordinate Status

No final coordinate set should be frozen yet.

The best current coordinate work is exploratory. One 5-marker seed used for subset-whitelist analysis was:

```text
head:
[ 39.5, -48.1, 18.9]
[ 39.0,  26.9, 29.1]
[-34.5,  53.8, 12.0]
[ -8.4, -63.9,  8.3]
[ 64.0,   6.4,  9.2]

waist:
[18.0, -50.0, 37.5]
[ 0.6,  61.8, 20.2]
[-6.6, -13.0, 63.3]
[22.9, -24.1, 55.9]
[58.2, -28.6,  4.8]

chest:
[  3.4,  39.5, 24.7]
[ 17.6, -30.4, 54.7]
[ 24.4,  58.4, 14.7]
[ 63.9,  -1.0, 11.8]
[-23.3, -42.3, 32.6]

left_foot:
[-29.7, -52.0, 25.3]
[ 62.1, -10.7, 16.1]
[  0.7,  47.4, 44.4]
[-17.3, -17.8, 49.1]
[-63.6,  -3.5, 13.0]

right_foot:
[  7.2,  45.2, 46.1]
[-44.2,  34.2, 33.1]
[ 24.0, -22.4, 56.1]
[-38.0,  -5.0, 52.5]
[-59.5,  24.3,  9.8]
```

This seed is useful for evaluator development, but it is not good enough to manufacture directly:

| metric | seed result |
|---|---:|
| raw worst `4-of-5` cross | `6.66mm` |
| raw worst `3-of-5` profile | `4.48mm` |
| best-1 whitelisted `4-of-5` cross | `11.58mm` |
| best-1 whitelisted `3-of-5` profile | `7.46mm` |

### What To Build Next

The next implementer should make the evaluator subset-aware before doing more physical design iteration.

Recommended order:

1. Extend `src/host/pattern_evaluator.py`.
   - Report all `4-of-N` and `3-of-N` subset metrics.
   - Report `strong_4_subsets` and `strong_3_subsets`.
   - Report why a subset is weak: nearest cross-body neighbor, low volume, low area, repeated edges, near-isosceles triangle, or low ambiguity margin.
2. Add a JSON output mode that can be stored with custom rigid definitions.
3. Add a synthetic optimizer script or mode that optimizes directly against:
   - full `5-vs-5` cross margin.
   - whitelisted `4-of-5` cross margin.
   - whitelisted `3-of-5` profile margin.
   - coverage across likely missing-marker cases.
4. Only after that, run a longer coordinate search and freeze a candidate.
5. Feed whitelist metadata into runtime candidate scoring.

Target thresholds for the next search:

| metric | minimum target | stretch target |
|---|---:|---:|
| full `5-vs-5` cross | `20mm` | `25mm+` |
| whitelisted `4-of-5` cross | `14mm` | `18mm+` |
| whitelisted `3-of-5` profile | `8mm` | `10mm+` |
| minimum marker spacing | `35mm` | `40mm+` |
| weak subset accepted in boot/reacquire | `0` | `0` |

### Runtime Handoff Notes

This design depends on runtime behavior already being strict about ownership and 2D evidence.

Important runtime principles:

- A candidate pose that fits in 3D but fails 2D reprojection should not acquire or reacquire a body.
- A visible cluster owned by another rigid body should not be reused for a different body.
- BOOT and REACQUIRE should be stricter than CONTINUE.
- CONTINUE may hold prediction through short occlusions instead of committing a weak subset pose.
- The confidence model should decay while only weak evidence is available.

Testing should include:

- all one-marker occlusions for each body.
- all two-marker occlusions for each body.
- multi-rigid distractor scenes.
- recent-loss reacquire.
- long-loss reacquire.
- body crossing / near-overlap scenarios.
- 2-camera and 4-camera variants.

Metrics to record:

- false lock rate.
- identity swap rate.
- reacquire latency.
- pose snap count.
- max translation / rotation innovation.
- percent frames held on prediction.
- rejected-valid rate for real observations.
- whitelist coverage by missing marker.

### Do Not Repeat These Mistakes

- Do not evaluate only full-pattern RMS. Partial visibility is where swaps happen.
- Do not treat low Kabsch RMS as sufficient; use 2D reprojection and ownership checks.
- Do not allow `3-of-5` for cold boot or independent reacquire.
- Do not assume 5 markers automatically makes every subset safe.
- Do not freeze coordinates until `strong_4_subsets` / `strong_3_subsets` are part of the evaluator output.

## 2026-04-26 - 5-marker / subset whitelist PDCA

### Context

Design constraint:

- 5 rigid body trackers are needed: `head`, `waist`, `chest`, `left_foot`, `right_foot`.
- Marker centers must stay inside a radius `65mm` hemisphere.
- `z >= 0` is fixed because each tracker mounts against the body.
- The goal is to reduce false recognition, pose flips, and rigid-body identity swaps.

The previous 4-marker exploration showed that a carefully selected 4-marker set can improve the built-in baseline, but making five independent 4-marker bodies mutually unique inside the hemisphere remains tight. The strongest practical rule was:

- `boot` / `reacquire`: require all 4 markers.
- `continue`: allow 3 markers only under prediction and reprojection gates.

This round tested whether 5 markers improves the design, and whether it should be used alone or together with mode-specific tracking policy.

### Baseline Findings

The built-in 4-marker patterns are weak geometrically:

| metric | built-in range / worst |
|---|---:|
| self symmetry RMS | `0.29-9.06mm` |
| worst cross-pattern RMS | `6.57mm` |
| closest 3-marker subset delta | as low as `0.34mm` |

The previous balanced 4-marker candidate improved the floor:

| metric | previous 4-marker candidate |
|---|---:|
| worst self symmetry RMS | `20.53mm` |
| worst intra 3-marker delta | `14.28mm` |
| worst cross-pattern RMS | `14.65mm` |
| worst cross 3-marker profile delta | `8.20mm` |

### PDCA 1 - Try 5 markers with mode policy

Plan:

Test the policy:

- `boot` / `reacquire`: require `4-of-5`.
- `continue`: allow `3-of-5`.
- Compare against 4-marker strict acquisition.

Do:

Generated 5-marker candidates inside the hemisphere and evaluated:

- full `5-vs-5` Kabsch cross-pattern RMS.
- any `4-of-5` subset cross-pattern RMS.
- any `3-of-5` subset distance-profile cross delta.

Check:

5-marker full observations improved, but arbitrary subsets were weak:

| metric | observed floor |
|---|---:|
| full `5-vs-5` cross | roughly `16-22mm` in quick runs |
| arbitrary `4-of-5` cross | as low as `6-8mm` |
| arbitrary `3-of-5` profile cross | as low as `4-5mm` |

Act:

5 markers alone is not enough. It must be paired with runtime policy. In particular, `3-of-5` must not be allowed for boot or independent reacquire.

### PDCA 2 - Add subset whitelist

Plan:

Treat 5 markers as the canonical rigid body, but only trust strong subsets:

- `boot` / `reacquire`: whitelisted strong `4-of-5` subsets only.
- `continue`: whitelisted strong `3-of-5` triplets only.
- weak subsets hold prediction and decay confidence instead of committing a new pose.

Do:

Evaluated the 5-marker seed with subset whitelisting. For each body, subsets were ranked by marker spacing, edge uniqueness, triangle area, and 4-point volume.

Raw all-subset results:

| metric | all subsets allowed |
|---|---:|
| worst `4-of-5` cross | `6.66mm` |
| worst `3-of-5` profile | `4.48mm` |

Whitelisted subset results:

| policy | worst margin |
|---|---:|
| best 1 strong `4-of-5` subset per body | `11.58mm` |
| best 1 strong `3-of-5` triplet per body | `7.46mm` |

Check:

Whitelist improves the effective margin, but coverage drops. Allowing the top 2-3 subsets per body often reintroduces the same weak pair that made all-subset mode unsafe.

Act:

The next evaluator should report `strong_4_subsets` and `strong_3_subsets` explicitly rather than only reporting whole-pattern ambiguity. Runtime should reject non-whitelisted weak subsets even if they produce a low RMS pose.

### PDCA 3 - Optimize for trusted subsets instead of all subsets

Plan:

Retarget optimization toward the actual tracking policy:

- maximize full `5-vs-5` identity margin.
- maximize whitelisted `4-of-5` reacquire margin.
- maintain at least one strong `3-of-5` continuation triplet per body.
- do not try to make every possible subset equally valid.

Do:

Ran a targeted random/greedy search that scored each candidate by:

- full 5-marker self symmetry.
- selected strong 4-subset cross separation.
- selected strong 3-triplet distance-profile separation.
- selected 4-subset volume and marker spacing.

The best intermediate run before stopping was:

| metric | targeted whitelist search |
|---|---:|
| worst full self symmetry | `18.89mm` |
| worst whitelisted `4-of-5` cross | `13.71mm` |
| worst whitelisted `3-of-5` profile | `9.92mm` |
| worst full `5-vs-5` cross | `17.61mm` |
| minimum marker spacing | `40.33mm` |

Check:

Targeting whitelist-compatible subsets moved the useful `3-of-5` continuation margin above the previous `8.20mm` 4-marker cross-triplet floor and improved marker spacing. It did not make all `4-of-5` combinations safe.

Act:

Adopt the policy shape, but do not freeze physical coordinates yet. The next practical step is to add subset-aware reporting to the evaluator, then run a longer optimizer against that exact report.

### Current Recommendation

Use this design policy:

```text
BOOT:
  require 4-of-5
  accepted subset must be in strong_4_subsets
  require low residual
  require ambiguity margin over second-best body/assignment

REACQUIRE:
  require 4-of-5
  accepted subset must be in strong_4_subsets
  require prediction agreement if recently lost
  treat long-loss recovery as BOOT

CONTINUE:
  allow 4-of-5 if subset is strong_4_subsets
  allow 3-of-5 only if triplet is strong_3_subsets
  use 3-of-5 as prediction confirmation, not independent pose source
  weak subset -> hold predicted pose, increase uncertainty, decay confidence
```

### Implementation Follow-up

Add a subset-aware design/evaluator phase:

1. Extend `src/host/pattern_evaluator.py` to report:
   - `strong_4_subsets`
   - `strong_3_subsets`
   - all weak subset pair explanations
   - per-subset volume, min spacing, edge-gap, and cross-body nearest neighbor
2. Store whitelist metadata with custom rigid body definitions.
3. Use whitelist metadata during rigid candidate scoring:
   - `boot` / `reacquire`: reject non-whitelisted 4-subset candidates.
   - `continue`: allow whitelisted 3-subset candidates only inside prediction and 2D reprojection gates.
4. Replay with synthetic subset drops:
   - all 1-marker occlusions.
   - all 2-marker occlusions.
   - multi-body distractor points.
   - recent-loss vs long-loss reacquire.

### Decision

The best path is **5 markers + mode policy + subset whitelist**.

Do not use 5 markers with arbitrary subset acceptance. That makes reacquire and continuation look more robust on paper but creates weak partial-pattern identity matches.

The key discipline is:

> 3-of-5 is a prediction-confirmation signal, not a standalone pose source.
