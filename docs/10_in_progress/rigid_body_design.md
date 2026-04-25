# Rigid Body Design PDCA Log

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
