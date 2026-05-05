# Rigid Body Stabilization V2 Plan

## Purpose

This plan defines the next PDCA entry point for improving 5-rigid tracking within the current hardware roadmap.

Current roadmap constraints:

- `4` cameras.
- `5` blobs / markers per rigid body.
- Passive or unlabeled blob observations.
- No LED-driven active marker ID.
- No 6+ marker rigid bodies.
- No 5+ camera expansion.

The research in `references/gpt_research/deep-research-report (2).md` still changes the direction, but only the parts that fit these constraints should drive the next PDCA cycle.

The useful lessons are:

- Treat BOOT, CONTINUE, and REACQUIRE as different modes with different evidence thresholds.
- Move away from marker-local threshold fixes toward body-level model-based assignment.
- Treat 2D blob quality as a first-class measurement confidence input.
- Keep subset uniqueness and strict partial-subset policy as design constraints.
- Use controlled ablations instead of repeatedly tuning pixel thresholds against the same hard frames.

Active IDs, 6-7 markers, and 6-8 cameras remain important prior-art signals, but they are not part of this implementation plan.

## Current State

The current simulator and design work has reached a useful but suspicious state:

- `candidate_00534` improves identity safety and timing over the current seed.
- Candidate-specific `object_gating_ambiguous_marker_assignment_min_margin_px = 0.50` cleans the known 240-frame hard boundary.
- The 480-frame hard high-noise run still fails on chest pose continuity after near-full occlusion.
- Several accepted fixes are narrow thresholds around close projected blobs and marker assignment margins.

This is a warning sign. The next step should not be another local threshold search. It should test whether the same constraints can be handled with cleaner mode separation, body-level assignment, and explicit blob confidence.

## Current Scenario Review

### What Is Appropriate

`five_rigid_body_occlusion_v1` is still the right hard fixture.

- It uses the 4-camera `cube_top_2_4m_aim_center` rig instead of a flat parallel layout.
- It covers the known body-mounted blockers: torso occlusion, seated waist, foot crossing, and camera-specific blob sizes.
- It has exposed realistic failure classes: marker swaps, cross-rigid wrong owner, low-marker hold, and sustained pose jumps.
- It keeps random false blobs out of the body-mounted route, which avoids optimizing for synthetic clutter instead of ownership.

`five_rigid_body_occlusion_relaxed_v1` remains useful as the green-regression entry.

- It catches obvious regressions without forcing every PDCA step through the hardest occlusion fixture.
- It should be the first smoke after any behavior change.

### What Needs Review

The current hard scenario risks overfitting because the same fixed frames keep driving threshold choices. For V2:

- Keep hard frames as diagnostic fixtures.
- Require seed repeats before adoption.
- Require at least one longer sustained run for any candidate that claims to improve continuity.
- Add diagnostics that explain why a body failed, not only whether it failed.

The sim also needs richer measurement diagnostics before more tracker tuning:

- per-camera blob area / equivalent diameter by marker and rigid.
- close-blob and merge-risk summaries.
- marker visibility by mode: BOOT, CONTINUE, REACQUIRE, and hold.
- candidate rejection reasons grouped by body and mode.
- body-level conflict events when two rigid hypotheses want the same blob.

## Environment Setup

Run from the repo root with the existing virtual environment.

Baseline commands:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_relaxed_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.05 \
  --seed 45 \
  --false-blobs-per-camera 0 \
  --out logs/sim/passive_stabilization_v2/baseline_relaxed_seed45_noise005
```

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.10 \
  --seed 45 \
  --false-blobs-per-camera 0 \
  --out logs/sim/passive_stabilization_v2/baseline_hard_seed45_noise010
```

Use the current practical candidate package as the main baseline when testing structural changes:

- Candidate source: `logs/design/rigid_body_v2/candidates_pdca_cycle7/rank_04_candidate_00534.json`
- Sim rigids: `logs/design/rigid_body_v2/sim_pdca_cycle20_candidate00534_policy_expand2/candidate_00534_waist_expand2_tracking_rigids.json`
- Candidate-specific override: `object_gating_ambiguous_marker_assignment_min_margin_px = 0.50`

Also keep the current seed as a control. If a change only helps `candidate_00534`, it may still be useful, but it should not be called a general runtime fix.

## V2 Experiment Contract

Each experiment must report:

- `wrong_ownership_count`
- `marker_source_confusion_count`
- `valid_frame_ratio`
- `position_delta_top_events`
- `rotation_delta_top_events`
- `pipeline_pair_ms`
- `rigid_ms`
- `geometry_ms`
- `mode_counts_by_rigid`
- `boot_candidate_count_by_rigid`
- `continue_candidate_count_by_rigid`
- `reacquire_candidate_count_by_rigid`
- `hold_prediction_count_by_rigid`
- `candidate_rejection_reasons_by_mode`
- `body_conflict_count`
- `body_conflict_resolution_reasons`
- `blob_quality_summary_by_camera`
- `blob_quality_summary_by_rigid`
- local guard settings that were disabled, relaxed, or retained

Do not adopt any change that only improves a summary score while hiding the exact failure frames.

## Primary Strategy

### Strategy A - Mode Separation

The commercial-system lesson that fits the current roadmap is strict mode separation.

Desired mode policy:

| mode | evidence policy |
|---|---|
| BOOT | strictest; require whitelisted `4-of-5`, low 2D reprojection error, and clear body-level margin |
| CONTINUE | allow whitelisted `3-of-5` only as prediction confirmation, not independent identity evidence |
| SHORT-LOSS REACQUIRE | require prediction agreement and low innovation; do not scan generic blob clouds broadly |
| LONG-LOSS REACQUIRE | behave closer to BOOT; require whitelisted `4-of-5` and clear body-level margin |
| FULLY UNSEEN HOLD | hold only with bounded confidence decay; do not integrate stale angular evidence into a committed pose |

This should reduce the need for local close-blob rescue rules.

### Strategy B - Body-Level Assignment

The closest passive prior-art pattern is body-level model matching:

1. Use the predicted rigid pose to create per-marker 2D windows.
2. Build candidate marker-to-blob assignments per rigid body.
3. Score each body candidate with:
   - 2D reprojection residual.
   - 3D or projected distance signature consistency.
   - whitelisted subset strength.
   - temporal innovation.
   - blob quality weights.
   - margin over the second-best body candidate.
4. Resolve cross-body blob conflicts at the body-candidate level.
5. Commit the winning pose only if the body-level margin is strong enough for the current mode.

The goal is to replace scattered marker-local rejection thresholds with one explainable body-candidate score.

### Strategy C - Blob Quality Confidence

The current close-blob rules are proxy quality measures. V2 should make measurement confidence explicit.

Initial blob quality inputs:

- area
- equivalent diameter
- nearest-neighbor blob distance
- diameter-relative overlap risk
- temporal area change
- temporal centroid jump
- camera id
- expected marker projection distance

Future real-Pi inputs, if available:

- circularity
- bounding box width / height
- saturation or bloom proxy
- contour merge / intrusion flag

Policy:

- Bad blob quality should lower candidate confidence.
- Severe quality failure should produce a missing marker, not a wrong marker.
- Blob quality should be logged before it is used for hard rejection.

## Out Of Scope For This Plan

The following are useful research conclusions but not current roadmap items:

- LED-driven active markers.
- Active anchor IDs.
- 6+ marker rigid bodies.
- 5+ camera or 6-8 camera expansion.

Keep them as future branches. Do not let them drive V2 acceptance criteria.

## Initial PDCA Entry

PDCA must remain sequential.

### Baseline 0 - Current Candidate Package

Measure `candidate_00534` with the current runtime package.

Matrix:

| scenario | frames | seeds | noise |
|---|---:|---|---|
| `five_rigid_body_occlusion_relaxed_v1` | `240` | `45,46,47` | `0.05px` |
| `five_rigid_body_occlusion_v1` | `240` | `45,46,47` | `0.05px` |
| `five_rigid_body_occlusion_v1` | `240` | `45,46,47` | `0.10px` |
| `five_rigid_body_occlusion_v1` | `480` | `45` | `0.15px` |

Read:

- Identity status: wrong/confusion.
- Validity floor by rigid.
- Chest sustained jump frames.
- Timing p95 and over-budget runs.
- Mode transitions around the top delta events.
- Candidate rejection reasons around the top delta events.

Decision:

- If this no longer reproduces the known state, stop and restore baseline before changing behavior.

### Cycle 1 - Add Passive Diagnostics Only

Hypothesis:

- Current summaries are too coarse; richer diagnostics can identify whether failures are mode-policy, assignment, or blob-quality problems.

Change:

- Add mode counts, rejection reasons, body conflict counts, and blob quality summaries.
- Do not change tracking behavior.

Measure:

- Run Baseline 0 seed `45` relaxed and hard.

Decision:

- Keep only if behavior is exactly unchanged and diagnostics explain the known chest / waist / foot failures.

### Cycle 2 - Long-Loss REACQUIRE As BOOT

Hypothesis:

- The dangerous ownership decisions happen after longer loss windows, so long-loss reacquire should use BOOT-like evidence.

Change:

- After a configurable loss age, starting at `8` frames, require whitelisted `4-of-5`, 2D score, and body-level margin.
- Short-loss continuation remains unchanged.

Measure:

- Hard seed `45`, noise `0.10px`.
- If green, repeat seed `46`.

Decision:

- Keep if wrong/confusion do not increase, valid ratio stays at least equal to baseline, and top delta events do not worsen.

### Cycle 3 - Body-Level Candidate Margin Diagnostics

Hypothesis:

- Existing local guard failures can be explained by weak body-level candidate margin.

Change:

- Compute body-level candidate score and second-best margin in diagnostics.
- Do not enforce yet.

Measure:

- Hard seed `45`, noise `0.10px`.
- Hard seed `45`, `480` frames, noise `0.15px`.

Decision:

- Keep if diagnostics show whether known jumps are caused by low margin, low visibility, or bad measurement quality.

### Cycle 4 - Enforce Body-Level Margin In REACQUIRE Only

Hypothesis:

- Reacquire is the safest place to replace local thresholds with body-level candidate margin.

Change:

- Enforce body-level margin for REACQUIRE candidates only.
- Do not change CONTINUE.
- Do not change blob-quality rejection yet.

Measure:

- Relaxed seeds `45,46,47`, noise `0.05px`.
- Hard seeds `45,46,47`, noise `0.10px`.

Decision:

- Keep if wrong/confusion stay `0/0`, valid does not drop below baseline, and timing stays near baseline.

### Cycle 5 - Relax Candidate-Specific Marker Margin

Hypothesis:

- Body-level reacquire scoring can replace the candidate-specific `0.50px` marker assignment margin.

Change:

- Return marker margin to the current global default for this candidate.
- Keep body-level REACQUIRE margin from Cycle 4.

Measure:

- Hard seeds `45,46,47`, noise `0.10px`.

Decision:

- Keep if wrong/confusion remain `0/0`, valid remains at baseline, and timing improves or stays equal.

### Cycle 6 - Blob Quality Weighting Diagnostics

Hypothesis:

- Some remaining marker confusion and pose jumps are measurement-quality failures, not identity failures.

Change:

- Add blob-quality weights to candidate scoring in diagnostics only.
- Compare weighted and unweighted candidate ranks.

Measure:

- Hard seed `45`, noise `0.10px`.
- Hard seed `45`, `480` frames, noise `0.15px`.

Decision:

- Keep if blob quality explains known failure frames without changing behavior.

### Cycle 7 - Blob Quality Missing-Marker Policy

Hypothesis:

- Severe blob-quality failures should become missing observations instead of weak wrong observations.

Change:

- Enforce quality-based missing marker only in REACQUIRE or low-confidence CONTINUE.
- Do not apply it to all valid continuation frames.

Measure:

- Relaxed seeds `45,46,47`, noise `0.05px`.
- Hard seeds `45,46,47`, noise `0.10px`.
- If green, hard seed `45`, `480` frames, noise `0.15px`.

Decision:

- Keep if identity stays clean, valid ratio does not drop below baseline, and sustained chest jumps improve.

## Goal Conditions

## PDCA Run 2026-05-04

Scope:

- Scenario: `five_rigid_body_occlusion_v1`.
- Rigids: `logs/design/rigid_body_v2/sim_pdca_cycle20_candidate00534_policy_expand2/candidate_00534_waist_expand2_tracking_rigids.json`.
- Profile: `gui_live`.
- Constraint: passive, 4 cameras, 5 blobs per rigid, no active marker, no 6+ marker rigid, no 5+ camera branch.

Baseline:

- Hard seed `45`, `480` frames, noise `0.15px`, marker margin `0.50px`.
- wrong/confusion `0/0`, chest valid `0.96875`, max position delta `28.82mm`, max rotation delta `57.50deg`.
- V2 diagnostics: chest `prediction_hold=105`, body conflicts `32`, rigid fallback dominated by `insufficient_rigid_hint_points=114` and `tracked_rigid_unseen_hold_prediction=104`.
- Read: current blocker is not active identity loss. It is a chest visibility gap where passive marker evidence disappears long enough that prediction/reacquire policy cannot preserve pose continuity.

Sequential cycles:

| Cycle | Hypothesis | Change | Result | Decision |
| --- | --- | --- | --- | --- |
| 1 | Candidate-specific `0.50px` margin may be overfitted and slow. | Set marker margin to `0.29px` on seed `45`, noise `0.15px`. | Chest failure unchanged: max position `28.82mm`, max rotation `57.50deg`. Pair p95 improved to `6.27ms`. | Margin is not the chest root cause. Keep testing confusion risk separately. |
| 2 | Lower margin may fail repeat seeds. | Run `0.29px` on seeds `46` and `47`, `240` frames. | seed `46`: confusion `1`; seed `47`: confusion `2`; pose deltas stayed small. | `0.29px` is unsafe for identity. Do not relax to default. |
| 3 | Intermediate margin may keep performance and remove confusion. | Run `0.40px` on seed `46`. | confusion `1`. | Still unsafe. |
| 4 | `0.50px` is the minimum known safe margin for high-noise repeats. | Run `0.50px` on seed `46`. | confusion `0`, max rotation `3.27deg`. | Keep `0.50px` for this candidate package unless a non-margin identity fix lands. |
| 5 | Pose continuity guard may be causing the long chest hold. | Disable pose continuity guard on seed `45`. | Chest failure unchanged; hold count unchanged at `105`. | Pose guard is not the cause. |
| 6 | Position continuity guard may be causing the long chest hold. | Disable pose and position continuity guards. | Chest failure unchanged; timing worsened slightly. | Continuity guards are not the cause. |
| 7 | Partial-marker hold may be too long. | Temporarily reduce partial hold from `90` to `24` frames. | No improvement; chest valid dropped to `0.9625`. | Reject and revert. Partial hold was not dominant. |
| 8 | Unseen hold may be too long. | Temporarily reduce unseen hold from `24` to `6` frames. | Worse: chest valid `0.91667`, body conflicts `85`, max rotation `57.73deg`. | Reject and revert. Short hold alone creates more reacquire churn. |
| 9 | Body-level missing-marker recovery may help if 3 marker-indexed hints exist. | Temporarily allow 3-marker rigid-hint candidates. | No improvement. Only one 3-marker hint reached policy and was rejected by whitelist; most failures still had insufficient hints. | Reject and revert. The missing evidence is below useful 3-marker recovery frequency. |
| 10 | The hard entry may be beyond the current passive evidence boundary. | Reduce noise to `0.10px` on seed `45`, `480` frames. | Max position improved to `18.84mm`, max rotation to `39.98deg`, but confusion `5` and chest valid `0.95833`. | Lower noise helps amplitude but does not remove the visibility gap; current hard 480-frame scenario is a failure-boundary test, not a green entry gate. |

Readout:

- The useful breakthrough signal is diagnostic, not algorithmic yet: the blocker is sustained chest evidence starvation, not a bad local threshold inside the solver.
- Existing heavy processing is not the root. Removing guards or shortening hold did not fix the jump.
- The passive research direction that still matches the data is camera/blob-side observability: log or add camera-side quality, merge, and visibility evidence before relaxing guards.
- The next simple algorithm candidate should be body-level visibility confidence and dropout classification before reacquire, not more pose smoothing.

Next PDCA entry:

- Keep hard seed `45`, `480` frames, noise `0.15px` as the red stress case.
- Use a green entry set first: `240` frames, seeds `45,46,47`, noise `0.10px`, marker margin `0.50px`, expected wrong/confusion `0/0`.
- Add a diagnostics-only chest visibility contract: per-frame visible marker count, per-camera nearest blob distance, body conflict owners, and whether the chest has zero/one/two/three/four/five marker-indexed hints.
- Only after the visibility contract explains frames `395`, `429`, and `458`, test a small policy change such as "do not reacquire after long unseen gap unless visibility confidence recovers to a whitelisted 4-marker subset."

### Minimum Breakthrough

The passive V2 route counts as a useful breakthrough if all are true:

- It preserves relaxed and hard green identity: wrong/confusion `0/0`.
- It removes the need for candidate-specific `0.50px` marker margin in 240-frame hard noise `0.10px`.
- It does not reduce any rigid's valid ratio below the current candidate baseline.
- It does not increase `rigid_ms.p95` by more than `0.05ms`.
- It replaces at least one local threshold dependency with a body-level or mode-level decision.
- It explains the known top delta events by mode, candidate margin, or blob quality.

### Strong Breakthrough

The route counts as a strong breakthrough if all minimum conditions pass and:

- The 480-frame hard noise `0.15px` run has wrong/confusion `0/0`.
- Chest max rotation delta drops below the current candidate baseline without adding a new pose-continuity guard.
- Pair p95 remains `<= 6.0ms`.
- Rigid p95 remains near `<= 1.5ms`.
- Over-budget sustained runs stay `0`.

### Stop Conditions

Stop this PDCA route and reassess if:

- Failures are dominated by visibility limits that cannot be solved with mode policy.
- Body-level margin diagnostics do not distinguish good and bad frames.
- Blob quality diagnostics do not explain close-marker or jump events.
- Relaxing the `0.50px` marker margin causes repeatable marker confusion.
- The route requires all existing local guards plus new body-level logic.

## PDCA Run 2026-05-04 Continued

Scope:

- Same hard scenario and candidate file as the first run.
- Constraint remains passive, 4 cameras, 5 blobs per rigid, no active marker, no 6+ marker rigid, no 5+ camera branch.
- New question: can the stress be reduced by adding a small amount of physical-layer information, instead of more guard or parameter tuning?

Sequential cycles:

| Cycle | Hypothesis | Change | Result | Decision |
| --- | --- | --- | --- | --- |
| 11 | The chest markers may be physically absent in the hard frames. | Added emitted marker visibility diagnostics from the ownership ledger. | Chest emitted visibility stayed at 5 markers; failure frames still had all true blobs emitted. | Physical emission is not the failure; the loss happens after 2D blobs enter triangulation/assignment. |
| 12 | The estimator may be losing marker-indexed rigid hints even while blobs exist. | Added rigid-hint marker/view histograms and top-delta frame logs. | Chest hint histogram was `0:115, 3:1, 5:364`; top failure frames had `0` hint markers. | Root is marker-indexed 3D evidence disappearance, not pose-guard behavior. |
| 13 | Long unseen prediction hold may block generic recovery. | Temporarily allowed generic recovery after 8 unseen frames. | Chest valid worsened to `0.92708`; body conflicts rose to `58`. | Reject. Generic fallback without body-level assignment increases churn. |
| 14 | Distance-signature recovery from generic 3D points may reacquire chest. | Temporarily tried bounded distance-signature candidate search. | No pose improvement; rigid p95 rose to about `42.8ms`; no candidates dominated. | Reject and revert. Heavy generic search is the wrong direction. |
| 15 | Generic 3D points may exist but be ignored by rigid hints. | Added generic 3D coverage against GT markers. | Chest coverage fell to `0` on top failure frames, with nearest generic points `166-359mm` away. | Loss is upstream of rigid fitting; generic points are also gone. |
| 16 | Failure may be simple blob merging or near-overlap. | Added top-delta nearest-blob distance logs. | Chest nearest distances on top frames were mostly `5.57-11.71px`, not catastrophic merges. | Not primarily a close-blob merge failure. |
| 17 | The scenario geometry may be intrinsically impossible. | Ran hard seed `45` with noise `0.00px`. | All rigids valid `1.0`; chest hint `0:2, 5:478`; max position about `2.05mm`. | Geometry is clean without centroid noise. |
| 18 | Small centroid noise may still be safe. | Ran noise `0.05px`. | All rigids valid `1.0`; max rotation `2.66deg`; no holds. | Stable below the noise boundary. |
| 19 | Failure boundary may start at `0.10px`. | Ran noise `0.10px`. | Chest valid `0.95833`; confusion `5`; chest hint `0:110, 5:367`. | Boundary is between `0.05px` and `0.10px`. |
| 20 | Boundary may be near `0.075px`. | Ran noise `0.075px`. | All rigids valid `1.0`; chest hint `0:2, 5:478`. | `0.075px` is green. |
| 21 | Boundary may be near `0.0875px`. | Ran noise `0.0875px`. | Chest valid `0.96875`; confusion `6`; chest hint `0:106, 5:366`. | `0.0875px` is red. |
| 22 | Boundary may be near `0.08px`. | Ran noise `0.08px`. | All rigids valid `1.0`; chest hint `0:2, 5:478`. | Current fixed-noise tolerance is about `0.08px`. |
| 23 | Boundary may be near `0.085px`. | Ran noise `0.085px`. | Chest valid `0.96875`; confusion `7`; chest hint `0:105, 5:366`. | Fixed-noise model collapses between `0.08px` and `0.085px`. |
| 24 | Blob diameter/SNR may explain why fixed pixel noise is too pessimistic. | Added `diameter_scaled` centroid-noise model and reran `0.085px`. | All rigids valid `1.0`; chest hints stayed `5` for `478` frames. | Keep the physical noise model for PDCA. |
| 25 | The same physical model may rescue the prior `0.10px` red. | Ran `0.10px`, `diameter_scaled`, 14mm baseline diameter. | All rigids valid `1.0`; max rotation `4.13deg`. | Strong signal: blob quality removes the old red without guard changes. |
| 26 | The original hard `0.15px` may pass with quality-aware noise alone. | Ran `0.15px`, `diameter_scaled`, 14mm baseline diameter. | Chest valid `0.95833`; confusion `15`; chest hint `0:110`. | 14mm baseline is still short at `0.15px`. |
| 27 | Larger physical markers may rescue `0.15px`. | Tried `1.5x` marker diameter. | No change because the scale was not applied to the loaded sim patterns. | Treat as setup bug; fix before reading this result. |
| 28 | Larger physical markers plus quality-aware noise may rescue `0.15px`. | Fixed marker scale propagation and ran `1.5x`, `0.15px`. | All rigids valid `1.0`; chest max rotation `4.13deg`; chest p05 blob diameter `4.76px`. | Strong breakthrough candidate. |
| 29 | Improvement may come from marker diameter metadata rather than centroid quality. | Ran `1.5x` with constant noise. | Chest valid `0.95417`; max position `43.21mm`; hint `0:119`. | Diameter metadata alone does not help; the physical effect is centroid quality. |
| 30 | A smaller marker increase may be enough. | Ran `1.25x`, `diameter_scaled`, `0.15px`. | All rigids valid `1.0`; chest max rotation `4.80deg`; chest p05 blob diameter `3.97px`. | `1.25x` is enough in no-dropout hard stress. |
| 31 | Minimal marker increase may be enough. | Ran `1.1x`, `diameter_scaled`, `0.15px`. | All rigids valid `1.0`; chest max rotation `5.45deg`; chest p05 blob diameter `3.49px`. | `1.1x` is enough without dropout, but margin is thin. |
| 32 | The `1.1x` result may be overfit to no-dropout conditions. | Added marker dropout `0.02` with `1.1x`. | Chest valid fell to `0.95833`; max position `68.09mm`. | `1.1x` has insufficient dropout margin. |
| 33 | `1.25x` may tolerate light dropout. | Ran dropout `0.02` with `1.25x`. | Chest stayed good, but left foot valid fell to `0.51667`; max left-foot position `500.72mm`. | Larger blobs solve centroid noise, not marker absence/redundancy. |
| 34 | `1.5x` may tolerate light dropout across all bodies. | Ran dropout `0.02` with `1.5x`. | Chest stayed good, but left foot and waist still failed. | Dropout needs a separate redundancy solution; do not hide it with guards. |
| 35 | `1.25x` no-dropout result may be seed-specific. | Ran seed `44`, `1.25x`, `0.15px`. | All rigids valid `1.0`; worst max position `2.08mm`; worst max rotation `5.63deg`. | Not seed `45` specific. |
| 36 | Seed robustness should hold for another repeat. | Ran seed `46`, `1.25x`, `0.15px`. | All rigids valid `1.0`; worst max position `3.08mm`; worst max rotation `6.29deg`. | Promote `diameter_scaled + >=1.25x marker diameter` as the next physical-layer candidate. |

Readout:

- The fixed-noise sim was too blunt for physical-layer decisions: it made 14mm, 17.5mm, and 21mm markers equally noisy unless an explicit diameter/SNR model was added.
- The first real breakthrough is not a new rigid guard. It is making centroid noise depend on equivalent blob diameter, then checking that a modest marker-diameter increase can keep marker-indexed 3D evidence alive.
- In this model, the current hard fixed-noise boundary is about `0.08px`; with diameter-aware noise, 14mm survives `0.10px`, and `1.25x` marker diameter survives `0.15px` across seeds `44-46`.
- Marker dropout is a separate structural problem. Larger blobs kept the chest stable but did not save low-observation left-foot and waist runs under dropout `0.02`.

Next PDCA entry:

- Treat `--centroid-noise-model diameter_scaled` as the default physical-layer diagnostic when evaluating marker size or exposure changes.
- Use `--marker-diameter-scale 1.25` as the first candidate physical change for `0.15px` no-dropout hard stress.
- Keep a separate dropout/redundancy branch red; do not solve it with more pose guards.
- Add real camera-side blob quality export before trusting this fully on hardware: equivalent diameter, contour area, saturation/bloom flags, circularity, and nearest-neighbor distance per blob.

## Future Branches

These are not current roadmap items, but the research says they are likely high-leverage if the passive 4-camera / 5-marker route stalls:

- Add one active or coded marker per rigid body.
- Move to 6-7 markers per rigid body.
- Move to 6-8 cameras or a more dome-like camera volume.
- Add real camera-side blob quality fields from Pi contour extraction.

Do not mix these with the current PDCA until the passive route has a clear result.

## PDCA Run 2026-05-04 Active Anchor / Temporal Body Hypothesis

Scope:

- Same body-mounted simulator family under `tools/sim/`.
- Main stress: `five_rigid_body_occlusion_v1`, `480` frames, seed `45`, `0.15px`, `diameter_scaled`, marker diameter scale `1.25`.
- Constraint: no threshold tuning. Test whether body hypothesis continuity or one decoded active anchor per rigid reduces the need for local ambiguity/drop guards.
- New sim switch: `--active-anchor-markers true`, which emits marker `0` as a decoded physical-ID anchor and lets object-gating bind that blob only to the matching rigid/marker.

Sequential cycles:

| Cycle | Hypothesis | Change | Result | Decision |
| --- | --- | --- | --- | --- |
| 0 | Current temporal hold may preserve low-observation bodies under light random dropout. | No anchor, marker dropout `0.02`. | Wrong `0`, confusion `5`, worst valid waist `0.82917`, max rotation about `179.87deg`. | Temporal hold improves survival but still allows marker swaps and stale jumps. |
| 1 | One anchor ID may reduce marker confusion under the same dropout. | Added active anchor, initially still subject to random dropout. | Confusion fell `5 -> 3`, but worst valid remained waist `0.80833`. | Anchor helps ownership, but random missing markers are a separate redundancy problem. |
| 2 | Without random dropout, anchor should isolate pure marker-swap benefit. | Active anchor, no marker dropout. | Wrong/confusion `0/0`, all valid `1.0`, max rotation about `1.67deg`. | Active anchor works for the no-dropout ownership case. |
| 3 | Compare Cycle 2 against no-anchor. | No anchor, no marker dropout. | Wrong `0`, confusion `2`, all valid `1.0`. | The anchor removed same scenario marker confusion that passive assignment left behind. |
| 4 | Active anchor should be treated as higher-SNR and not random-detection dropped. | Made active marker `0` immune to random marker dropout, still occludable. | Confusion `2`, worst valid chest `0.85833`, hold counts exploded, max rotation about `179.86deg`. | Reject broad temporal hold behavior; holding stale pose is worse than losing cleanly. |
| 5 | Restrict 2D/anchor evidence to extending an existing hold, not starting one from zero 3D markers. | Changed temporal hold evidence role separation. | Same as Cycle 4. | Root is still marker absence/reacquire, not hold-start policy. |
| 6 | No-dropout active-anchor result may be seed-specific. | Active anchor, seed `44`, no dropout. | Wrong `0`, confusion `2`, worst valid waist `0.99792`; confusion was head marker `3/4`. | One anchor does not disambiguate all non-anchor marker swaps. |
| 7 | Check another seed. | Active anchor, seed `46`, no dropout. | Wrong/confusion `0/0`, all valid `1.0`. | Anchor is useful but not a complete N-best body assignment replacement. |
| 8 | Red alias stress should show the cross-rigid ownership failure mode. | `five_rigid_dance_swap_red_v1`, no anchor. | Wrong `132`, confusion `115`, left foot valid `0.0`. | Red fixture remains a hard ownership stress. |
| 9 | Active anchor may reduce red cross-rigid alias ownership. | Same red stress with active anchor. | Wrong rose to `186`, confusion fell slightly to `108`; left foot still `0.0`. | Reject active anchor as a red-alias solver; one ID does not solve adversarial false/alias blobs. |

Readout:

- One physical-ID anchor per rigid is valuable for ordinary no-dropout ownership: it removed seed `45` marker confusion from `2` to `0`.
- It is not a dropout/redundancy solution. If markers are absent, the system needs more observations, better temporal body candidate enumeration, or physical redundancy; an anchor cannot reconstruct missing non-anchor geometry.
- It is not a complete marker-level N-best solver. Seed `44` still had a head marker `3/4` swap because only marker `0` carried ID.
- The naive temporal route of holding pose from weak 2D body evidence is dangerous: it can preserve validity while accumulating stale pose error.
- Next useful algorithm branch is body-level N-best assignment among non-anchor markers, scored by full-body reprojection and temporal innovation, not more local drop/keep tuning.

## PDCA Run 2026-05-04 Phase-Aware Evaluation Fix

Problem:

- The sim was counting any `pose.valid` frame as valid recovery, even when the pose source was `prediction_hold`.
- This mixed four different states: hidden compensation, partial 2D evidence, partial 3D recovery, and full 3D realignment.
- As a result, a stale hold could look like successful tracking until the GT error summary exposed a large jump.

Change:

- Added `last_pose_source` to rigid tracker diagnostics.
- Added `phase_evaluation_by_rigid` to `tools/sim` summaries.
- The new evaluation separates:
  - physical visibility: `hidden`, `one_camera_visible`, `partial_visible`, `full_visible`.
  - solver evidence: `hidden_prediction_only`, `partial_2d_evidence`, `partial_3d_hint`, `full_3d_hint`, `visible_no_solver_evidence`.
  - pose source: `rigid_hint`, `generic`, `prediction_hold`, `ownership_conflict_prediction_hold`, rejected/invalid states.
- Added `phase_aware_go_no_go` so prediction-hold drift is judged separately from measurement recovery.

Check:

- Run: `logs/sim/passive_stabilization_v2/pdca_phase_eval_cycle00_active_anchor_dropout002`.
- Legacy valid ratios still show broad survival: chest `0.85833`, waist `0.95208`.
- Phase-aware readout shows the real issue:
  - chest `full_3d_hint` measurement valid ratio `0.96460` with max rotation error below `1deg`.
  - chest `partial_2d_evidence` had `60` prediction-hold valid frames with max rotation error about `126deg`.
  - chest `visible_no_solver_evidence` had `238` prediction-hold valid frames with max rotation error about `179deg`.
  - waist `partial_2d_evidence` prediction-hold max rotation error stayed around `7.17deg`, while `visible_no_solver_evidence` reached about `29.59deg`.

Decision:

- Keep the phase-aware evaluation.
- Do not treat full hidden or visible-no-solver prediction hold as recovery.
- The next algorithm experiment should only use 2D evidence to constrain prediction during `partial_2d_evidence`, and should force hard realignment when `full_3d_hint` returns.
- The intended structure is:
  - hidden: coast/compensate and score hold drift only.
  - partial 2D: constrain prediction by 2D evidence plus acceleration prediction.
  - partial 3D: recover cautiously with body-level scoring.
  - full 3D: hard realign and reset accumulated drift.

## PDCA Run 2026-05-04 Active Anchor Side-Channel / 2D-Constrained Hold

Hypothesis:

- A decoded physical-ID anchor should be usable as a small physical-layer hint even when the unlabeled body assignment is too weak.
- The anchor should not be folded into normal Hungarian body assignment outside the pixel gate, because that can distort full-body ownership and hard realignment.
- During partial 2D evidence, use the anchor's 2D observation to lightly correct predicted translation. Keep rotation on the motion prediction until enough body geometry returns.

Sequential cycles:

| Cycle | Hypothesis | Change | Result | Decision |
| --- | --- | --- | --- | --- |
| 10 | Direct nonlinear pose optimization against partial 2D observations may reduce hold drift. | Added a `prediction_hold_2d_constrained` source using 2D assignments plus motion prior. | Existing 2-camera regression still passed geometrically but failed rigid p95 timing because the optimizer was too heavy. | Reject heavy per-frame least-squares for this path. |
| 11 | Translation-only correction is enough for the 2D hint layer and should avoid overfitting rotation. | Replaced nonlinear 6DoF solve with a tiny translation-only projection correction. | Tests passed. On `pdca_phase_eval_cycle02_anchor_wide_2d_translation_dropout002`, chest partial position max improved about `0.821m -> 0.046m`, but rotation still failed and marker confusion rose `2 -> 7`. | Good signal, bad coupling: active anchor was being mixed into normal assignment too broadly. |
| 12 | Treat active anchor as a side-channel, not as normal body assignment override. | Added `active_anchor_observations` diagnostics and used them only for `prediction_hold_2d_constrained`; normal object-gating assignment remains gated. | On `pdca_phase_eval_cycle04_anchor_sidechannel_2d_translation_dropout002`, right foot valid improved `0.8625 -> 0.97083`, left foot slightly improved `0.87083 -> 0.88125`, confusion was `3`; chest did not change because its failing partial span had no usable anchor side-channel. | Keep side-channel separation. It is a targeted physical-ID improvement, not a global rotation/redundancy fix. |

Readout:

- This validates the architecture split: physical ID can safely add a small piece of information without broadening unlabeled blob acceptance.
- The useful effect is translation stabilization when the anchor is visible; it does not solve rotation during long single-anchor or no-solver spans.
- The remaining blocker is not a threshold. It is missing body geometry: one anchor gives ownership and a 2D ray, but not enough rotational observability.
- Next algorithm branch should be body-level temporal N-best for non-anchor markers, with full-body reprojection and conflict resolution, so partial non-anchor geometry can constrain rotation without local marker guard tuning.

## PDCA Run 2026-05-04 Body-Level Temporal Subset / 2D PnP Probe

Hypothesis:

- If 3-of-5 3D marker hints are available, a body-level subset candidate may recover pose where the current 4-of-5 separated candidate falls back to prediction hold.
- If only one camera has 3+ marker assignments, a body-level 2D PnP candidate may use the rigid geometry plus temporal prediction to recover rotation earlier than pure hold.

Sequential cycles:

| Cycle | Hypothesis | Change | Result | Decision |
| --- | --- | --- | --- | --- |
| 13 | A continuing 5-marker rigid can adopt a strong 3-marker temporal subset when full 4-of-5 is unavailable. | Added temporal body subset candidate from rigid-hint subsets, scored by existing full-body reprojection and temporal prediction. | `pdca_phase_eval_cycle05_temporal_body_subset_dropout002` was effectively unchanged from the side-channel baseline; partial 3D windows were too sparse or rejected by existing body/policy checks. | Keep as a safe entry, but it is not the current blocker. |
| 14 | One-camera 3+ marker assignments can produce a body-level 2D PnP pose for partial 2D recovery. | Added `body_2d_pnp` candidate from object-gating assignments, using prediction as initial pose where possible. | Existing 2-camera regression stayed geometrically correct, but performance failed because PnP ran in a scenario that did not need it. | Scope PnP to multi-rigid 5-marker bodies only. |
| 15 | PnP candidates that violate physical continuity should not be clamped into measurement poses. | Guarded PnP adoption: if existing position or pose continuity rejects it, fall back to prediction hold instead of committing a clamped measurement. | `pdca_phase_eval_cycle07_body_2d_pnp_guarded_dropout002` preserved the side-channel gains: right foot valid `0.97083`, wrong `0`, confusion `3`; chest remained unchanged and phase go/no-go still failed. | Reject 2D PnP as the next breakthrough for this stress. It is useful as a guarded candidate type, but the dominant chest failure has no stable 3+ one-camera body evidence to exploit. |
| 16 | PnP is too heavy for this loop and should not stay on the active path. | Removed the PnP execution path and kept the physical-layer path to active-anchor side-channel correction plus temporal body subsets. | Tests remain green; `pdca_phase_eval_cycle08_pnp_free_physical_anchor_dropout002` matches the side-channel result with right foot valid `0.97083`, wrong `0`, confusion `3`, and rigid p95 `5.795ms`. | Continue with physical-layer hints and conflict propagation, not camera-side PnP recovery. |

Readout:

- The phase-aware scorer is doing its job: it prevents a visually plausible PnP/hold from being counted as recovery unless GT error stays bounded.
- The current hard chest span is still evidence starvation, not a local assignment threshold problem.
- PnP was useful as a probe, but it is too expensive for the live partial-evidence path.
- The physical-layer information to keep exploiting is decoded active-anchor identity, anchor visibility by camera, and whether that anchor is enough to constrain translation without pretending it observes rotation.
- The next high-leverage branch should be frame-level conflict propagation across bodies: when a partial body loses ownership, preserve and compare N-best body hypotheses instead of immediately reducing the loser to hold/invalid.

## Production-Readiness Trial 2026-05-04

Question:

- Which current `src/host/rigid.py` route is closest to a real 118fps production path for 5 rigid bodies, 5 blobs each, and 4 cameras?
- Avoid threshold tuning. Prefer physical-layer evidence, body-level assignment, and honest invalidation over overfit guards.

Routes checked:

| Trial | Route | Stress | Result | Decision |
| --- | --- | --- | --- | --- |
| `production_trial_06_no_weak_5marker_holds_dropout002` | Reject 5-marker no-3D prediction holds. | hard body scenario, marker dropout `0.02`, active anchor. | Wrong `0`, confusion `2`, prediction-hold phase errors passed, but valid ratios fell to chest `0.24792`, right foot `0.41458`; rigid p95 about `27.65ms`. | Honest but too sparse for live pose output. Good safety baseline, not production UX. |
| `production_trial_09_anchor_ray_relock_dropout002` | Use decoded anchor observations as camera rays and relock translation only; no PnP. | same hard dropout stress. | Wrong `0`, confusion `2`; head/waist stayed high, but chest `0.15833`, feet around `0.33-0.35`; single-anchor hold rotation still reached about `11deg`. | Keep the PnP-free anchor ray idea, but do not count it as full recovery. One anchor is translation information, not rotation information. |
| `production_trial_10_anchor_reboot_hold_dropout002` | Allow past-track BOOT to continue from active-anchor hold. | same hard dropout stress. | Valid ratios rose, but partial 2D prediction-hold rotation reached about `180deg`. | Reject. A single anchor cannot safely advance BOOT/recovery state. |
| `production_trial_13_hold_no_mode_advance_dropout002` | Keep prediction-hold from advancing mode. | same hard dropout stress. | Valid ratios rose, but repeated anchor-only holds again reached about `180deg`. | Reject. Mode discipline alone is not enough; anchor-only hold needs hard observability limits. |
| `production_trial_12_anchor_ray_relock_hard_nodrop` | PnP-free active anchor side-channel with no synthetic dropout. | hard body scenario, no marker dropout. | All valid ratios `1.0`, wrong/confusion `0/0`, phase-aware go/no-go passed, rigid p95 about `1.79ms`, pair p95 about `6.81ms`. | This is the closest production route when the Pi SNR premise means no random marker dropout. Remaining work is performance p95 and real-log validation. |
| `production_trial_11_anchor_ray_relock_relaxed_dropout002` | Same as above on relaxed body stress with dropout. | relaxed body scenario, marker dropout `0.02`. | Chest `0.97292`, feet `1.0`, waist `0.95`, but head `0.93542`, wrong/confusion `1/2`, timing still over budget. | Useful green-regression fixture, but not enough as the hard production gate. |

Readout:

- The strongest production path right now is `gui_live` object-gating enforced, active-anchor side-channel, no PnP, and full 3D rigid-hint hard realignment.
- The no-dropout hard scenario shows the core geometry path is viable: all five bodies track at valid `1.0` with zero ownership errors.
- The hard dropout scenario is not solved by one physical-ID anchor. It mostly creates rotation-unobservable spans; accepting them as valid produces stale or flipped orientation.
- Therefore one active anchor can reduce ownership search and stabilize translation, but production-grade dropout recovery needs one of:
  - body-level N-best assignment over non-anchor markers that recovers 3+ true marker geometry without local guard tuning.
  - another physical orientation cue, such as a second coded marker or a non-camera orientation source.
  - a stricter output contract that marks anchor-only spans as compensated/invalid for full 6DoF consumers.
- Timing is still not production-ready at 118fps in Python for the hard multi-rigid route: no-drop hard rigid p95 is about `1.79ms` versus the current `1.5ms` target, and pair p95 is about `6.81ms` versus the `6ms` target.

## GUI Formal Route / Speed and N-Best Trial 2026-05-04

Formal GUI route:

- `src/host/tracking_runtime.py` keeps the formal GUI route passive-first and no longer exposes the rejected anchor-guided N-best option.
- The formal route is therefore:
  - `fast_ABCDHRF`
  - `subset_diagnostics_mode=off`
  - object-gating enforced
  - passive temporal body N-best
  - full 3D rigid-hint hard realignment
  - no anchor-only 6DoF recovery

Trials:

| Trial | Route | Result | Decision |
| --- | --- | --- | --- |
| `production_trial_14_anchor_guided_nbest_dropout002` | Active-anchor-guided non-anchor body N-best enabled by code path. | No valid-ratio improvement over the formal route; wrong `0`, confusion `2`, but rigid p95 about `28.04ms`. Most attempts were not rankable. | Rejected and removed from the formal code surface. |
| `production_trial_15_gui_formal_hard_nodrop` | Formal GUI route, no marker dropout. | All valid `1.0`, wrong/confusion `0/0`, phase go/no-go passed; rigid p95 about `1.87ms`, pair p95 about `7.09ms`. | Accuracy is production-like under the high-SNR/no-dropout premise; timing still needs work. |
| `production_trial_17_gui_formal_fast_marginless_hard_nodrop` | Removed exact second-best body-assignment margin recomputation from the GUI hot path. | Accuracy unchanged, but timing did not reliably improve in this run: rigid p95 about `1.92ms`, pair p95 about `7.15ms`. | Keep the simplification because the removed work was diagnostic-only, but it is not the main no-drop bottleneck. |
| `production_trial_19_anchor_only_fast_invalid_dropout002` | Short-circuit 5-marker anchor-only recovery to invalid instead of generic 3D search. | Valid ratios stayed intentionally sparse under dropout, wrong `0`, confusion `2`; rigid p95 improved from about `28.88ms` to about `2.50ms`, pair p95 from about `34.17ms` to about `8.31ms`. | Keep. This is a correct observability-based speed fix. |

Current readout:

- The formal GUI route is accurate when the physical premise is high SNR and no random false/dropout blobs.
- One active anchor plus non-anchor N-best does not yet recover hard dropout; it mostly spends time generating unrankable hypotheses.
- The largest hard-dropout speed win came from refusing unobservable anchor-only 6DoF recovery early.
- Remaining timing work should target the always-on full-hint path:
  - object-gating projection/assignment across 5 bodies and 4 cameras.
  - rigid full-hint Kabsch/score/update loop around `src/host/rigid.py`.
  - pair-level overhead outside rigid, since no-drop pair p95 remains above `6ms`.

## Sim Observation Realism Check 2026-05-04

Question:

- The previous `marker_dropout=0.02` stress used iid random non-anchor marker loss, but Pi logs are high SNR and false-blob-free.
- Does a more physical detection model change the conclusion for the formal GUI route?

Change:

- `tools/sim/multi_rigid.py` now exposes `--marker-detection-model iid_dropout|pi_snr`.
- `iid_dropout` is the old model: every non-anchor marker may disappear independently.
- `pi_snr` keeps the same `--marker-dropout` as a maximum stress amount, but scales detection loss by projected blob equivalent diameter relative to `--centroid-noise-reference-diameter-px`.
- Active anchors are still only lost by occlusion, not random non-anchor detection dropout.

Trials:

| Trial | Detection model | Result | Decision |
| --- | --- | --- | --- |
| `production_trial_22_iid_detection_dropout002` | `iid_dropout`, dropout `0.02` | Synthetic detector dropped `749` marker observations. Valid ratios collapsed on feet (`0.39583` / `0.33958`), wrong/confusion `1/2`, phase failed. | This is too pessimistic for the stated high-SNR Pi premise and should stay as a red stress, not the production-realistic baseline. |
| `production_trial_24_pi_snr_detection_dropout002_single` | `pi_snr`, dropout `0.02` | Synthetic detector dropped `1` marker observation. All valid ratios `1.0`, wrong/confusion `0/0`, phase passed. Pair p95 `6.37ms`, rigid p95 `1.76ms`. | This matches the current physical premise better: the core route is accuracy-ready, with timing still slightly over target. |
| `production_trial_23_pi_snr_nodrop` | `pi_snr`, dropout `0.0` | All valid ratios `1.0`, wrong/confusion `0/0`, phase passed. | Confirms the `pi_snr` model reduces to the no-drop high-SNR case when no detection stress is requested. |

Readout:

- The hard iid dropout was masking the important distinction between physical occlusion and random detector failure.
- Under high-SNR Pi-like detection, the formal GUI route no longer needs local threshold loosening to stay accurate.
- The remaining blocker becomes production timing, not rigid correctness.
- Keep iid dropout for red-team robustness, but use `pi_snr` as the main real-operation sim entrance until real logs show frequent true marker disappearance.

## GUI Hot-Path Optimization 2026-05-04

Goal:

- Keep the formal GUI route unchanged behaviorally.
- Optimize the realistic `pi_snr` production entrance without threshold tuning.
- Preserve all five rigid bodies at valid `1.0`, wrong ownership `0`, and marker confusion `0`.

Changes:

- Cached per-camera blob reprojection uncertainty once per frame instead of recomputing it for every rigid.
- Cached geometry observations by blob index during observation construction so rigid-hint triangulation does not rebuild the lookup.
- Switched two-view rigid-hint residual summaries to the small-sample summary path.
- Replaced tracker angular-velocity update through SciPy matrix conversion with a direct quaternion-delta rotvec.

Trials:

| Trial | Change | Result | Decision |
| --- | --- | --- | --- |
| `production_trial_27_pi_snr_cache_repeat` | Blob uncertainty cache plus geometry observation lookup cache. | Accuracy unchanged. Pair p95 about `6.33ms`, rigid p95 about `1.79ms`. | Keep the cache, but it is not enough by itself. |
| `production_trial_31_pi_snr_small_summary` | Small-sample rigid-hint residual summaries. | Accuracy unchanged. Pair p95 improved to about `5.82ms`; rigid p95 about `1.67ms`. | Keep. This removes repeated percentile work from the 2-view hint path. |
| `production_trial_34_pi_snr_quat_update` | Direct quaternion-delta angular velocity update. | Accuracy unchanged. Pair p95 about `5.39ms`, rigid p95 about `1.44ms`, geometry p95 about `1.27ms`. | Keep. This clears the p95 118fps target under the realistic sim entrance. |
| `production_trial_35_pi_snr_quat_update_repeat` | Repeat of the same route. | Accuracy unchanged. Pair p95 about `5.16ms`, rigid p95 about `1.34ms`, geometry p95 about `1.26ms`. | Confirms the p95 improvement, though rare pair max spikes remain. |

Readout:

- The realistic `pi_snr` entrance now shows the formal GUI route can hit the p95 budget without loosening assignment thresholds.
- The remaining failed production flag is rare max / consecutive-over-budget spikes, not p95 or accuracy.
- Next speed work should target allocation spikes and event/snapshot payload churn rather than marker-assignment policy.
