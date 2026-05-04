# Rigid Body Design V2 Plan

## Purpose

This plan defines an end-to-end route for finding manufacturable 5-marker rigid-body designs that improve identity stability and live performance in the current multi-rigid tracking pipeline.

The current direction from `docs/10_in_progress/rigid_body_design.md` remains valid:

- Use 5 markers per rigid body.
- Do not accept arbitrary `3-of-5` or `4-of-5` subsets.
- Store `strong_4_subsets` and `strong_3_subsets` as part of the rigid definition.
- Treat `3-of-5` as prediction confirmation, not as an independent BOOT or REACQUIRE pose source.

V2 does not replace that policy. It extends the design process so that candidate coordinates are scored against the real 4-camera projection, current object-gating behavior, body-mounted occlusion failures, and 118fps performance budget.

## Current Evidence

The design and sim logs point to the same failure mode:

- Full 5-marker observations are much stronger than the built-in 4-marker patterns.
- Arbitrary partial subsets are still unsafe.
- The hard body-mounted scenario still exposes marker identity ambiguity under high noise.
- Recent object-gating improvements reduce but do not eliminate the need for better marker geometry.
- Performance is now close to budget, so design changes should reduce ambiguity-triggered work rather than add more guard logic.

Relevant references:

- Design policy and first 5-marker subset search: `docs/10_in_progress/rigid_body_design.md`
- Body-mounted sim and object-gating PDCA: `docs/10_in_progress/sim_performance_upgrade_log_v2.md`
- Simulator usage: `docs/40_tools/multi_rigid_simulator.md`

## 2026-05-03 PDCA Status

Baseline measurement:

- `tests/test_pattern_evaluator.py` passed before edits.
- Current `design_5marker_seed` failed Gate 2: worst full cross `16.30mm`, no body reached a strong `4-of-5` subset under the V2 target set, and several bodies missed the `35mm` spacing floor.

Cycle log:

| cycle | hypothesis | change | result | decision | next action |
|---:|---|---|---|---|---|
| 1 | Subset-aware geometry JSON will make Gate 2 measurable before sim. | Extended `src/host/pattern_evaluator.py` with Gate 2 metrics, strong subset metadata, nearest-neighbor details, and weak-subset reasons. | Unit tests passed; current seed reported no Gate 2 pass. | Keep evaluator changes. | Add repeatable candidate search. |
| 2 | A simple mutation search can produce stored candidate JSON for later sim references. | Added `tools/rigid_body_design_search.py`. | Initial smoke found candidates but script import path failed when run directly. | Fix command portability. | Make script executable from repo root. |
| 3 | Direct script path setup will unblock search runs. | Added repo-root and `src` path setup inside the tool. | 100-iteration search wrote JSON, but scoring kept low-spacing baseline too high. | Search tool works; scoring needs physical-constraint weighting. | Penalize spacing failures and add random restarts. |
| 4 | Gate-target scoring plus random restarts will surface manufacturable layouts. | Reweighted score around full cross, subset floor, and marker spacing; rejected mutations below `35mm`. | 2000-iteration run improved spacing and full cross, but still failed Gate 2. | Continue; evaluator was over-rejecting high-margin edge-similar subsets. | Treat repeated-edge symmetry as a hard weak reason only when margin is thin. |
| 5 | Edge-similarity should not reject a subset whose ambiguity margin is already strong. | Relaxed repeated-edge / near-isosceles rejection to low-margin cases and lowered the 4-point volume floor. | 500-iteration run produced strong subsets for `waist` and `right_foot`; best candidate still failed due `head`, `chest`, `left_foot`, and full-cross margins. | Gate 2 is not achieved yet; implementation progress is usable, final design is not frozen. | Run a longer body-specific search that mutates the current worst body first, then add Gate 3 projection scoring. |

Current artifacts:

- Candidate search summaries: `logs/design/rigid_body_v2/candidates_pdca_cycle2/`, `logs/design/rigid_body_v2/candidates_pdca_cycle3/`, `logs/design/rigid_body_v2/candidates_pdca_cycle4/`, `logs/design/rigid_body_v2/candidates_pdca_cycle5/`
- Best Cycle 5 result: Gate 2 failed, worst full cross `15.36mm`, worst best-subset score `7.81mm`, min marker spacing `35.97mm`.

## 2026-05-04 PDCA Status

Strategy shift:

- Stop full-population random mutation once a partial win appears.
- Continue from the best stored JSON candidate.
- Mutate the weakest / failing bodies first, while preserving bodies that already pass.
- Treat projection screening as a separate gate before spending full sim runs.

Cycle log:

| cycle | hypothesis | change | result | decision | next action |
|---:|---|---|---|---|---|
| 6 | Body-specific mutation will preserve good bodies and lift the weak body floor. | Added `--seed-candidate` and `--focus-worst-rate` to `tools/rigid_body_design_search.py`. | Best candidate improved worst full cross to `19.41mm` and worst best-subset score to `10.44mm`, but still failed Gate 2. | Keep focused search; remaining failures are narrow. | Relax repeated-edge hard rejection when margin itself is above target. |
| 7 | Edge repetition should be diagnostic, not a hard fail, when subset margin already passes. | Adjusted evaluator rejection reasons and reran focused search. | `candidate_00389` passed Gate 2: worst full cross `20.06mm`, worst best-subset score `10.36mm`, min spacing `35.99mm`. | Gate 2 is achieved. | Add projection screen before sim. |
| 8 | A lightweight projection screen can catch 2D regressions before sim. | Added `tools/rigid_body_projection_screen.py`; compared Gate 2 best against current seed for 120 frames. | p05 separation improved by `3.64px`, subset area improved, but one worst pair was `0.003px` below baseline. | Need close-pair diagnostics and longer frame window. | Add worst-pair reporting and run 240-frame projection. |
| 9 | The strict projection fail may be a single close-pair artifact. | Added worst-pair metadata and reran 240 frames. | Worst pair improved, but `waist` strong-3 projected area collapsed below baseline in late frames. | Do not promote rank 0. | Sweep the Gate 2 top candidates through projection. |
| 10 | Another Gate 2 pass candidate may project better without more search. | Swept top 10 Cycle 7 candidates through 120 and 240-frame projection screens. | `candidate_00252` passed Gate 2 and 240-frame projection: p05 `+2.26px`, worst pair `+0.08px`, worst strong subset area `+0.25px²`. | Promote `candidate_00252` to Gate 4 entry candidate. | Export it as sim-readable custom rigids and run green smoke. |
| 11 | Sim can validate `candidate_00252` once custom body rigids override the built-in design seed. | Let `tools/sim` override body rigids from `custom_rigids` and exported `logs/design/rigid_body_v2/candidate_00252_tracking_rigids.json`. | Relaxed seed45/noise0.05 smoke had wrong/confusion `0/0`, but `left_foot` valid was `0.0`; default baseline on the same current tree also had poor `left_foot` validity. | Gate 4 is blocked by current sim/runtime baseline instability, not yet by candidate rejection. | First restore/diagnose baseline relaxed green validity, then rerun candidate sim matrix. |

Current promoted candidate:

- Candidate JSON: `logs/design/rigid_body_v2/candidates_pdca_cycle7/rank_02_candidate_00252.json`
- Sim custom rigids: `logs/design/rigid_body_v2/candidate_00252_tracking_rigids.json`
- Projection report: `logs/design/rigid_body_v2/projection_pdca_cycle10_sweep_240/rank_02/summary.json`

Gate 2 metrics for `candidate_00252`:

| metric | value |
|---|---:|
| worst full cross | `20.26mm` |
| worst best-subset score | `9.70mm` |
| min marker spacing | `35.99mm` |

Gate 3 lightweight metrics for `candidate_00252` over 240 frames:

| metric vs current seed | delta |
|---|---:|
| marker pair p05 | `+2.26px` |
| worst marker pair | `+0.08px` |
| worst strong subset area | `+0.25px²` |

## 2026-05-04 Continued PDCA Status

Strategy shift:

- Treat Gate 2/3 as necessary but not sufficient.
- Add a sim BOOT smoke before promoting a candidate to the full matrix.
- Fix runtime hold behavior when ownership safety rejects a candidate after a short valid run.
- Prefer candidate-specific runtime overrides over global guard changes when current seed shows side effects.

Cycle log:

| cycle | hypothesis | change | result | decision | next action |
|---:|---|---|---|---|---|
| 12 | The relaxed baseline failure is either missing observations or a runtime ownership/acquire issue. | Checked generated ownership and tracking summaries. | True `left_foot` blobs existed in all frames; failures were `frame_local_blob_ownership_conflict` or no valid BOOT candidate. | Runtime/acquire path is the blocker, not blob generation. | Handle ownership conflicts without committing wrong blobs. |
| 13 | If a tracked body loses a frame-local ownership contest, holding prediction is safer than invalidating it. | Added prediction hold for `frame_local_blob_ownership_conflict` when the tracker already has a valid prediction. | `left_foot` improved from near-zero valid to `0.22`, wrong/confusion stayed `0/0`. | Direction is useful but incomplete. | Let explicit prediction-hold candidates bypass reacquire measurement rejection. |
| 14 | `tracked_rigid_*_hold_prediction` should not be treated as a failed reacquire measurement. | Updated prediction-hold candidate application to commit as hold, not strict reacquire. | At correct `118fps`, relaxed baseline seed45/noise0.05 reached all-body valid `1.0`, wrong/confusion `0/0`, trimmed p95 `4.88/1.43ms`. | Baseline green restored; earlier `60fps` CLI readings were not comparable. | Re-test promoted candidates at `118fps`. |
| 15 | `candidate_00252` should now pass relaxed sim if Gate 2/3 were sufficient. | Ran relaxed seed45/noise0.05 with custom rigids. | `left_foot` valid stayed `0.0`; BOOT explained only 3 markers and failed guard. | Gate 2/3 pass is not enough. | Sweep Gate 2 candidates through sim BOOT smoke. |
| 16 | Another Gate 2 candidate may be more BOOT-compatible. | Swept Cycle 7 top 10 candidates through relaxed seed45/noise0.05 at `118fps`. | `candidate_00534` reached all-body valid `1.0`, wrong/confusion `0/0`, trimmed p95 `4.89/1.45ms`. | Promote `candidate_00534`. | Run relaxed green seeds. |
| 17 | `candidate_00534` should hold relaxed green across seeds. | Ran relaxed seeds `45/46/47`, 240 frames, noise `0.05px`. | All seeds all-body valid `1.0`, wrong/confusion `0/0`, trimmed p95 about `4.9/1.43ms`. | Relaxed green passes. | Run hard green. |
| 18 | Hard green will expose whether whitelist coverage is enough. | Ran hard seeds `45/46/47`, noise `0.05px`. | wrong/confusion `0/0`, but `waist` valid dropped to `0.90-0.92`. | Candidate geometry is viable but waist whitelist is too narrow for hard occlusion. | Inspect hard missing subsets. |
| 19 | Adding waist `[0,2,3,4]` may cover hard occlusion. | Added waist strong4 `[0,2,3,4]`. | No meaningful change. | Wrong missing subset. | Inspect frame-level invalid reasons. |
| 20 | Adding waist `[0,1,2,3]` covers the actual hard visible subset. | Added waist strong4 `[0,1,2,3]` despite geometry-only score being below Gate 2 target. | Hard noise `0.05px`, seeds `45/46/47`: all-body valid `1.0`, wrong/confusion `0/0`. | Runtime whitelist must include sim-observed safe subsets, not only top geometry subset. | Measure hard high-noise boundary. |
| 21 | Candidate high-noise hard boundary should improve or preserve current seed. | Ran hard noise `0.10/0.15px`, seeds `45/46/47`. | Noise `0.10px`: all clean. Noise `0.15px`: valid `>=0.996`, wrong `0`, marker confusion `0/1/2`. Current seed had no confusion but seed47 valid fell to `0.875` with much worse timing. | Candidate improves robustness but not stretch confusion target. | Try marker-assignment margin override. |
| 22 | Raising marker assignment margin to `0.35px` will remove high-noise swaps. | Ran candidate with `object_gating_ambiguous_marker_assignment_min_margin_px=0.35`. | Confusion remained at noise `0.15px`. | Not enough. | Try `0.50px`. |
| 23 | A stronger `0.50px` assignment margin will reject the remaining swaps. | Ran candidate hard noise `0.15px`, seeds `45/46/47`, margin `0.50px`. | wrong/confusion `0/0`, min valid `>=0.996`, trimmed p95 about `4.9/1.5ms`. | Candidate + margin override reaches hard boundary target. | Check green side effects. |
| 24 | Candidate + margin `0.50px` should preserve relaxed and hard green. | Ran relaxed noise `0.05px` and hard noise `0.05/0.10px`, seeds `45/46/47`. | All runs valid `1.0`, wrong/confusion `0/0`; trimmed p95 stayed near budget. | Candidate runtime package is practical. | Check whether margin can be global default. |
| 25 | Margin `0.50px` may be safe as a global default. | Ran current seed with margin `0.50px` across relaxed/hard matrix. | Current seed still had seed46 confusion at hard noise `0.15px` and seed47 valid/perf collapse. | Do not change global default from this evidence. Use margin `0.50px` as candidate-specific runtime override. | Next: CAD fixture check and longer sustained run for `candidate_00534`. |
| 26 | `candidate_00534` + waist expansion + margin `0.50px` can sustain high-noise 480-frame operation. | Ran hard seed45/noise `0.15px`, 480 frames. | wrong/confusion `0/0`, valid `>=0.966`, but chest had rotation delta max `57.5deg` and position delta max `28.8mm`; scenario failed. | Sustained high-noise failure is not identity ownership, but chest pose continuity during occlusion. | Add frame-level delta event logging. |
| 27 | The sustained failure may be current-seed behavior, not candidate-specific. | Ran current seed with the same 480-frame hard/noise `0.15px` and margin `0.50px`. | Current seed had marker confusion `1`, worse timing, and head/waist rotation jumps; candidate still wins on identity and timing. | Keep `candidate_00534` as the practical package, but keep sustained gate open. | Diagnose candidate chest jump. |
| 28 | Max-only summary is too coarse to drive the next PDCA step. | Added `position_delta_top_events` and `rotation_delta_top_events` to sim summaries. | Candidate chest jumps clustered at frames `395/429/458`; wrong/confusion stayed `0/0`. | Failure is a pose-continuity problem after full/near-full occlusion. | Try narrowing chest policy. |
| 29 | Requiring chest 4 markers in continue should remove risky 3-marker recovery. | Tried a chest policy with `continue_min_markers=4` and no strong 3-subsets. | chest valid fell to `0.958`, jumps remained. | 3-marker continue is not the root cause. | Check whether chest whitelist is too narrow. |
| 30 | Adding chest strong4 `[1,2,3,4]` should reduce prediction holds. | Added only that next-best 4-point subset. | Sustained result was effectively unchanged. | Whitelist coverage is not the active cause for this seed. | Inspect unseen prediction behavior. |
| 31 | Fully unseen holds should freeze instead of integrating stale angular velocity. | Tested a local freeze variant for fully unseen prediction holds. | chest position max improved to `12.6mm`, rotation max improved to `11.4deg`, but still missed the strict `5deg` sustained gate and timing p95 worsened. | Freeze reduces catastrophic jumps but is not production-ready. | Try bounded coast, then reject if identity side effects appear. |
| 32 | Bounded one-frame coast can reduce freeze snap without over-rotation. | Tested a local bounded coast variant. | noise `0.15px` still failed rotation max (`13.7deg`), though trimmed rigid p95 returned to `1.49ms`. | Bounded coast is not enough. | Check side effects at noise `0.10px`. |
| 33 | Bounded coast should preserve the previously green noise `0.10px` sustained/short behavior. | Ran noise `0.10px` with the bounded coast variant. | chest marker confusion appeared (`11` events). | Reject bounded coast and revert it. | Keep original prediction behavior; retain delta-event logging for the next root fix. |
| 34 | Reverting coast should preserve the established 240-frame green boundary. | Ran 240-frame hard seed45/noise `0.10px`, margin `0.50px`. | all-body valid `1.0`, wrong/confusion `0/0`, trimmed p95 `4.94/1.49ms`. | Practical short-run package remains good; sustained high-noise chest continuity is the next blocker. | Next: design a pose-continuity fix that smooths re-entry without moving marker assignment projections. |
| 35 | The next 10-cycle run should start by re-reading the current sustained failure on the latest tree. | Re-read Cycle 28/34 summaries and inspected current rigid/pipeline insertion points. | Latest tree includes passive-stabilization counters and a `prediction_hold_2d_constrained` path; Cycle 28's old baseline is no longer the active current-tree baseline. | Measure the current tree before adding new logic. | Try output-only smoothing, then compare to no-smoothing current baseline. |
| 36 | Output-only re-entry smoothing can reduce chest jump without changing assignment. | Added a local return-pose limiter after prediction hold (`12mm`, `4.5deg`) and ran 480-frame seed45/noise `0.15px`. | Same identity failure as the current tree: wrong/confusion `1/4`; chest valid `0.954`; chest rotation max `50.6deg`; p95 pair/rigid `6.72/2.12ms`. | Reject the smoothing experiment; it does not address the active current-tree failure. | Disable smoothing and establish current-tree baseline. |
| 37 | The current tree without output smoothing may already have the same identity failure. | Disabled the local smoothing and reran the same 480-frame condition. | wrong/confusion `1/4`, chest valid `0.954`, chest rotation max `50.6deg`, p95 pair/rigid `6.44/1.78ms`. | The active blocker is now identity/body-conflict rank resolution, not output smoothing. | Test whether stronger marker margin helps. |
| 38 | Raising candidate marker assignment margin to `0.75px` will reject the remaining close marker swaps. | Reran 480-frame seed45/noise `0.15px` with margin `0.75px`. | No material change: wrong/confusion `1/4`, same failure frames. | Marker-vs-marker margin is not the controlling guard. | Test close-blob diameter overlap guard. |
| 39 | Raising close-blob overlap guard to `0.50` will reject pi-cam-04 near-blob ownership mistakes. | Reran with `object_gating_ambiguous_blob_diameter_overlap_ratio=0.50`. | No material change: wrong/confusion `1/4`. | The failure is not controlled by the existing close-blob ambiguity gate. | Try active-anchor physical-ID assist. |
| 40 | Active anchor markers can anchor body identity and reduce rank conflicts. | Reran with `active_anchor_markers=true`. | wrong/confusion stayed `1/4`, but chest valid improved to `0.969` and chest rotation max improved to `37.1deg`. | Active anchors help pose continuity, not the ownership-ledger identity event. | Test physical noise assumptions. |
| 41 | Diameter-scaled centroid noise is a more realistic high-noise boundary and may reduce small-blob swaps. | Reran with `centroid_noise_model=diameter_scaled`. | Worse: wrong/confusion `1/15`, chest rotation max `81.0deg`, p95 pair/rigid `7.46/2.50ms`. | Reject diameter-scaled noise for this sustained acceptance run. | Check whether lower constant noise restores 480-frame identity. |
| 42 | Lowering constant noise to `0.10px` should restore 480-frame identity if noise amplitude is the main cause. | Reran current baseline at noise `0.10px`. | Worse than noise `0.15px`: wrong/confusion `1/14`, chest rotation max `60.6deg`. | The failure is geometry/projection conflict, not simply centroid noise amplitude. | Test smaller marker diameter as a physical design lever. |
| 43 | Smaller markers reduce projected overlap and should reduce pi-cam-04 conflicts. | Reran with `marker_diameter_scale=0.70`. | Improved but not green: wrong/confusion `1/8`, chest valid `0.969`; persistent pi-cam-04 waist/left_foot wrong owner remains. | Marker diameter helps marker swaps but not the body-pair projection conflict. | Try an even smaller marker diameter to see if the effect saturates. |
| 44 | `0.50` marker diameter scale should further reduce projected overlap if diameter is the remaining lever. | Reran with `marker_diameter_scale=0.50`. | No further gain over `0.70`: wrong/confusion `1/8`; p95 timing worsened. | Diameter effect saturates; do not solve this by shrinking markers alone. | Next: modify design geometry or body-conflict rank logic for waist/left_foot pi-cam-04 separation. |

Current practical candidate package:

- Candidate source: `logs/design/rigid_body_v2/candidates_pdca_cycle7/rank_04_candidate_00534.json`
- Sim rigids with expanded waist whitelist: `logs/design/rigid_body_v2/sim_pdca_cycle20_candidate00534_policy_expand2/candidate_00534_waist_expand2_tracking_rigids.json`
- Required runtime override for this candidate: `object_gating_ambiguous_marker_assignment_min_margin_px = 0.50`

Current validation status:

| gate | status |
|---|---|
| Gate 2 geometry | pass with `candidate_00534`; waist whitelist expanded from sim evidence |
| Gate 3 lightweight projection | not the top projection candidate, but sim BOOT smoke passes |
| Relaxed sim green | pass, seeds `45/46/47`, noise `0.05px` |
| Hard sim green | pass, seeds `45/46/47`, noise `0.05px` |
| Hard boundary | pass in the earlier 240-frame matrix with margin override `0.50px`; must be revalidated on the current passive-stabilization tree |
| Sustained 480-frame run | blocked on the current tree: seed45/noise `0.15px` has persistent pi-cam-04 waist/left_foot wrong owner plus chest re-entry jumps |
| CAD manufacturability | pending |

## Design Objective

Find five rigid-body marker coordinate sets for:

- `head`
- `waist`
- `chest`
- `left_foot`
- `right_foot`

Each body should fit within the physical fixture constraints while maximizing:

- 3D identity separation between bodies.
- Safe whitelisted `4-of-5` BOOT / REACQUIRE subsets.
- Safe whitelisted `3-of-5` CONTINUE confirmation subsets.
- 2D marker separation after projection into the current 4-camera rig.
- Assignment margin between competing markers and blobs.
- Low object-gating ambiguity count in body-mounted sim.
- Stable 118fps live-route timing.

## Non-Goals

- Do not optimize for arbitrary `3-of-5` or `4-of-5` acceptance.
- Do not freeze a coordinate set from geometry-only metrics.
- Do not make a design that requires weaker runtime ownership or reprojection guards.
- Do not chase one clean seed if nearby seeds or noise levels still fail.
- Do not treat performance wins as valid if they increase wrong ownership or marker confusion.

## Constraints

Physical constraints:

| item | requirement |
|---|---:|
| marker count | `5` per rigid body |
| body count | `5` |
| coordinate region | hemisphere |
| max radius | `65mm` |
| z floor | `z >= 0` |
| target marker spacing | minimum `35mm`, stretch `40mm+` |
| manufacturable status | final candidate must be checked by CAD fixture generation |

Runtime constraints:

| mode | accepted evidence |
|---|---|
| BOOT | whitelisted `4-of-5` only |
| REACQUIRE | whitelisted `4-of-5` only, plus prediction agreement for recent loss |
| CONTINUE | whitelisted `4-of-5`, or whitelisted `3-of-5` only as prediction confirmation |
| weak subset | hold prediction, decay confidence, do not commit independent pose |

Performance constraints:

| metric | minimum target | stretch target |
|---|---:|---:|
| pipeline pair p95 | `<= 6.0ms` | `<= 5.2ms` |
| rigid p95 | `<= 1.5ms` | `<= 1.3ms` |
| sustained pair budget overrun | `0` sustained runs | `0` total |
| wrong ownership | `0` | `0` |
| marker source confusion | `0` on green matrix | `0` on hard matrix |
| min valid frame ratio | `>= 0.95` | `>= 0.995` |

## End-To-End Pipeline

The V2 pipeline has four gates. A candidate must pass each gate before moving to the next one.

```text
candidate generation
  -> geometry screening
  -> projection screening
  -> sim validation
  -> CAD / runtime freeze
```

### Gate 1 - Candidate Generation

Generate candidates with a constrained evolutionary search:

1. Start from the current 5-marker exploratory seed.
2. Generate mutations by moving one or more markers within the hemisphere.
3. Reject candidates that violate physical constraints.
4. Keep a diverse population rather than only the current best score.
5. Preserve body-to-body diversity so two bodies do not converge to similar shapes.

Generation strategies to include:

- Local mutation around the current seed.
- Wider-spacing biased mutation.
- Asymmetry biased mutation.
- Projection-margin biased mutation after Gate 2 exists.
- Body-specific mutation when one body is repeatedly the worst case.

Candidate records should be stored as JSON so every later sim result can point back to exact coordinates and whitelist metadata.

Suggested output location:

```text
logs/design/rigid_body_v2/candidates/
```

### Gate 2 - Geometry Screening

Run fast coordinate-only evaluation before touching the simulator.

Required geometry metrics:

| metric | minimum target | stretch target |
|---|---:|---:|
| full `5-vs-5` cross margin | `20mm` | `25mm+` |
| whitelisted `4-of-5` cross margin | `14mm` | `18mm+` |
| whitelisted `3-of-5` profile margin | `8mm` | `10mm+` |
| minimum marker spacing | `35mm` | `40mm+` |
| weak subset accepted in BOOT / REACQUIRE | `0` | `0` |

Required explanations for rejected or weak subsets:

- nearest cross-body neighbor.
- low 4-point volume.
- low 3-point triangle area.
- repeated edge lengths.
- near-isosceles or near-symmetric profile.
- low margin against another body or subset.

Implementation target:

- Extend `src/host/pattern_evaluator.py`.
- Add JSON output with per-body `strong_4_subsets`, `strong_3_subsets`, and rejection reasons.
- Add a design-search command or script under `tools/` that can run this evaluator repeatedly.

Gate 2 pass condition:

- At least one strong `4-of-5` subset per body reaches target.
- At least one strong `3-of-5` subset per body reaches target.
- No non-whitelisted weak subset is required for BOOT or REACQUIRE coverage.
- Candidate beats the current exploratory seed on the worst whitelisted `4-of-5` or `3-of-5` score.

### Gate 3 - Projection Screening

Geometry metrics are necessary but not sufficient. Gate 3 projects each candidate into the current 4-camera rig and measures whether markers become ambiguous in image space.

Required projection metrics:

| metric | target |
|---|---:|
| per-camera marker-pair 2D separation p05 | higher than current seed |
| worst projected marker-pair separation | no new close-pair below current seed unless sim proves safe |
| whitelisted subset projected area | higher than current seed on worst camera |
| marker-vs-marker assignment margin | above current `0.29px` guard boundary where possible |
| close-blob diameter overlap risk | lower than current seed on hard frames |
| cross-body projected profile similarity | lower than current seed |

Projection screening should use:

- `cube_top_2_4m_aim_center` camera rig.
- Same coordinate space assumptions as `tools/sim`.
- Occlusion masks for all one-marker and two-marker missing cases.
- Body poses sampled from `five_rigid_body_occlusion_v1`.

Important current sim-derived failure references:

- `head` marker `3/4` can become a close-marker swap under seed `45` / noise `0.10px`.
- `chest` marker `0/4` can become low marker-assignment margin under seed `46` / noise `0.15px`.
- Close-blob ambiguity is now guarded by fixed `0.60px` plus diameter overlap ratio `0.30`.
- Marker assignment ambiguity is now guarded by margin `0.29px`.

Gate 3 pass condition:

- Candidate reduces the worst known projected ambiguity compared with the current seed.
- Candidate does not create a new body whose projected subset is weaker than the current worst body.
- Candidate has at least one strong `4-of-5` and one strong `3-of-5` subset that remain strong under projection.

### Gate 4 - Sim Validation

Only a small number of candidates should reach full sim validation.

Validation matrix:

| scenario | frames | seeds | noise |
|---|---:|---|---|
| `five_rigid_body_occlusion_relaxed_v1` | `240` | `45,46,47` | `0.05px` |
| `five_rigid_body_occlusion_v1` green | `240` | `45,46,47` | `0.05px` |
| `five_rigid_body_occlusion_v1` hard | `240` | `45,46,47` | `0.10px, 0.15px` |
| best final candidate sustained | `480` | best and worst seed | selected boundary noise |

Core sim metrics:

- `wrong_ownership_count`
- `marker_source_confusion_count`
- `valid_frame_ratio`
- `ownership_confusion_summary`
- `ambiguous_assignment_count`
- `marker_margin_assignment_count`
- object-gating `assigned_marker_views`
- pose jump count
- max position innovation
- max rotation innovation
- pipeline pair p95
- rigid p95
- geometry p95
- sustained over-budget count

Gate 4 pass condition:

- Green matrix has `wrong_ownership_count = 0`.
- Green matrix has `marker_source_confusion_count = 0`.
- Hard matrix improves or preserves current best failure count.
- No seed has min valid frame ratio below `0.95`.
- Pair p95 stays within `6.0ms`.
- Rigid p95 does not regress meaningfully from the current object-gating optimized path.

## PDCA Rules

The design search may generate many candidates internally, but adoption must still follow sequential PDCA.

For each adoption cycle:

1. Measure the current baseline candidate first.
2. State the hypothesis for the next candidate or scoring change.
3. Make exactly one design-search, scoring, or runtime-evaluation change.
4. Measure the result.
5. Read the failure or improvement.
6. Decide whether to keep, adjust, or reject.
7. Choose the next action from that result.

Do not predeclare a large AB batch as PDCA. A batch can be used inside candidate generation, but the logged PDCA decision must be sequential.

Suggested PDCA log location:

```text
logs/design/rigid_body_v2/pdca_YYYYMMDD_*/
```

Suggested design log section:

```text
docs/10_in_progress/rigid_body_design_v2.md
```

## Scoring Model

Use a ranked score for search, but use hard gates for adoption.

Suggested search score:

```text
score =
  + full_5_cross_margin
  + whitelisted_4_cross_margin * 1.4
  + whitelisted_3_profile_margin * 1.2
  + projected_marker_margin * 1.5
  + projected_subset_area * 0.5
  + min_marker_spacing * 0.4
  - close_blob_risk * 2.0
  - marker_assignment_ambiguity_risk * 2.0
  - weak_subset_required_penalty
  - manufacturability_penalty
```

This score is only for ranking candidates. Final adoption uses Gate 4 pass conditions.

## Candidate Metadata Contract

Each candidate should carry enough metadata to be replayed and frozen later.

```json
{
  "candidate_id": "rigid_v2_YYYYMMDD_NNN",
  "source": "mutation_of_design_5marker_seed",
  "constraints": {
    "max_radius_mm": 65,
    "z_min_mm": 0,
    "marker_count": 5
  },
  "rigids": [
    {
      "name": "head",
      "marker_positions_mm": [[0, 0, 0]],
      "marker_diameter_m": 0.014,
      "tracking_policy": {
        "boot_min_markers": 4,
        "reacquire_min_markers": 4,
        "continue_min_markers": 3,
        "strong_4_subsets": [[0, 1, 2, 4]],
        "strong_3_subsets": [[1, 2, 4]]
      }
    }
  ],
  "geometry_report": {},
  "projection_report": {},
  "sim_report": {}
}
```

## Implementation Plan

### Phase 1 - Evaluator Report

Deliverables:

- Extend `src/host/pattern_evaluator.py`.
- Emit geometry JSON reports.
- Include subset whitelist recommendations and weak-subset explanations.
- Add tests for subset metrics and whitelist stability.

Done when:

- Current exploratory seed reproduces the known weak all-subset and improved whitelist numbers.
- Reports identify nearest weak subset pairs.

### Phase 2 - Candidate Search

Deliverables:

- Add constrained candidate generation under `tools/`.
- Store candidates and reports under `logs/design/rigid_body_v2/`.
- Generate an initial population from the current seed.

Done when:

- At least `1000` candidates can be screened by geometry without running full sim.
- Top candidates are reproducible by seed.
- Candidate reports include exact coordinate sets and policy metadata.

### Phase 3 - Projection Report

Deliverables:

- Add projection scoring using the current sim camera rig.
- Report per-camera marker-pair separation and assignment-margin risk.
- Report worst body, camera, frame, and marker pair.

Done when:

- Current known `head 3/4` and `chest 0/4` risk can be explained by projection metrics.
- Projection score can reject candidates that geometry-only metrics would falsely accept.

### Phase 4 - Sim Candidate Injection

Deliverables:

- Add a way for `tools/sim` to run a candidate JSON rigid definition.
- Preserve `tracking_policy` metadata through sim setup.
- Save summaries by `candidate_id`.

Done when:

- Current exploratory seed and one generated candidate can run the same 240-frame matrix.
- Sim summary includes candidate ID and design metrics references.

### Phase 5 - Sequential Design PDCA

Deliverables:

- Run 10 sequential PDCA cycles.
- Each cycle changes one scoring or generation decision.
- Log hypothesis, result, decision, and next action.

Initial 10-cycle focus:

| cycle target | purpose |
|---|---|
| baseline current seed | establish current design result under the new report |
| geometry-weight change | see whether stronger whitelist score predicts sim improvement |
| projection-weight change | see whether 2D margin predicts marker confusion reduction |
| body-specific mutation | fix the worst body from the previous result |
| spacing-biased mutation | test if wider spacing lowers close-blob risk |
| asymmetry-biased mutation | test if less symmetric profiles improve cross-body margin |
| whitelist coverage change | test one vs two strong subsets per body |
| hard-noise check | verify high-noise boundary |
| sustained run | check 480-frame behavior |
| freeze candidate candidate | decide whether a manufacturable candidate exists |

### Phase 6 - CAD Freeze Check

Deliverables:

- Generate fixture CAD for the top candidate.
- Check stem clearance, base holes, marker accessibility, and print feasibility.
- Feed any CAD-driven coordinate constraints back into Gate 1.

Done when:

- CAD constraints do not reduce design metrics below Gate 2 / Gate 3 targets.
- Candidate can be exported with final marker coordinates and whitelist metadata.

## Final Adoption Criteria

A rigid-body design can be frozen only when all of the following are true:

- Geometry report passes Gate 2.
- Projection report passes Gate 3.
- Sim green matrix is clean.
- Hard matrix is better than or equal to current design on marker confusion and wrong ownership.
- 480-frame sustained run does not show new identity drift.
- Pair p95 remains `<= 6.0ms`.
- Rigid p95 remains near or below current optimized path.
- CAD fixture is manufacturable without moving markers outside accepted margins.
- Runtime metadata includes `strong_4_subsets` and `strong_3_subsets`.
- The candidate can be replayed from stored JSON and logs.

## First Next Step

Start with Phase 1, not with a long coordinate search.

The highest-leverage first task is to make the evaluator report the exact subset and projection weaknesses that the sim already exposed. Once the current seed's known `head 3/4` and `chest 0/4` risks are visible in the report, candidate search becomes much less random and full sim runs can be reserved for serious finalists.
