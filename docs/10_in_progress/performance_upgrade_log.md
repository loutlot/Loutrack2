# トラッキング性能改善ログ

## 2026-04-26 `fast_ABCDHRF` 実ログ replay AB

### 目的

- 32 blobs/cam synthetic stress で採択候補になった `fast_ABCDHRF` を、実ログ3本で `fast_ABCD` と比較する。
- 比較対象:
  - `fast_ABCD`: 現正式経路。
  - `fast_ABCDHRF`: hint-only geometry + marker-index direct Kabsch + geo hot-path optimization、subset sampled。
  - `fast_ABCDHRF` + `subset_diagnostics_mode=off`: live表示想定。
- strict gate:
  - valid frames 非悪化。
  - reacquire / pose jumps / mode transitions / max flip 非悪化。
  - reprojection p95 5%以内。
  - `pipeline_pair_ms.p95` と `rigid_ms.p95` 改善。

### current log: `logs/tracking_gui.jsonl`

- `fast_ABCD`
  - valid poses: 19,770。
  - `pipeline_pair_ms.p95`: 5.092 ms。
  - `pipeline_pair_ms.max`: 8.660 ms。
  - `rigid_ms.p95`: 1.780 ms。
  - `triangulation_ms.p95`: 2.739 ms。
  - fast path used: 3,650。
  - fallback: 20,750。
- `fast_ABCDHRF` sampled
  - valid poses: 19,773。
  - `pipeline_pair_ms.p95`: 3.005 ms、約 41.0%改善。
  - `pipeline_pair_ms.max`: 8.557 ms。
  - `rigid_ms.p95`: 1.216 ms。
  - `triangulation_ms.p95`: 1.350 ms。
  - fast path used: 19,316。
  - fallback: 5,084。
  - strict gate: pass。
- `fast_ABCDHRF` subset off
  - valid poses: 19,773。
  - `pipeline_pair_ms.p95`: 2.768 ms、約 45.6%改善。
  - `pipeline_pair_ms.max`: 2.977 ms。
  - `rigid_ms.p95`: 0.852 ms。
  - `triangulation_ms.p95`: 1.260 ms。
  - strict gate: pass。

### archive baseline log: `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`

- `fast_ABCD`
  - valid poses: 13,644。
  - `pipeline_pair_ms.p95`: 5.490 ms。
  - `pipeline_pair_ms.max`: 35.099 ms。
  - `rigid_ms.p95`: 1.081 ms。
  - `triangulation_ms.p95`: 3.843 ms。
  - fast path used: 13,230。
  - fallback: 1,996。
- `fast_ABCDHRF` sampled
  - valid poses: 13,649。
  - `pipeline_pair_ms.p95`: 3.128 ms、約 43.0%改善。
  - `pipeline_pair_ms.max`: 9.059 ms。
  - `rigid_ms.p95`: 1.106 ms。
  - fast path used: 13,487。
  - fallback: 1,739。
  - strict gate: fail。
    - `rigid_ms.p95` がわずかに悪化。
    - `max_pose_flip_deg` が 177.8056 -> 177.8137 と 0.008度だけ増加。
- `fast_ABCDHRF` subset off
  - valid poses: 13,649。
  - `pipeline_pair_ms.p95`: 2.814 ms、約 48.7%改善。
  - `pipeline_pair_ms.max`: 3.160 ms。
  - `rigid_ms.p95`: 0.826 ms。
  - strict gate: fail。
    - speed gate は pass。
    - `max_pose_flip_deg` の微小増加により strict quality gate は fail。

### archive new log: `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`

- `fast_ABCD`
  - valid poses: 1,469。
  - `pipeline_pair_ms.p95`: 5.472 ms。
  - `pipeline_pair_ms.max`: 32.141 ms。
  - `rigid_ms.p95`: 1.518 ms。
  - `triangulation_ms.p95`: 3.735 ms。
  - fast path used: 1,315。
  - fallback: 321。
- `fast_ABCDHRF` sampled
  - valid poses: 1,469。
  - `pipeline_pair_ms.p95`: 3.421 ms、約 37.5%改善。
  - `pipeline_pair_ms.max`: 10.028 ms。
  - `rigid_ms.p95`: 1.250 ms。
  - fast path used: 1,374。
  - fallback: 262。
  - strict gate: pass。
- `fast_ABCDHRF` subset off
  - valid poses: 1,469。
  - `pipeline_pair_ms.p95`: 3.382 ms、約 38.2%改善。
  - `pipeline_pair_ms.max`: 3.928 ms。
  - `rigid_ms.p95`: 0.972 ms。
  - strict gate: pass。

### 判断

- `fast_ABCDHRF` は実ログでも速度面の効果が大きい。
  - current: p95 約 41-46%改善。
  - archive baseline: p95 約 43-49%改善。
  - archive new: p95 約 37-38%改善。
- `subset_diagnostics_mode=off` は live表示向けに最大遅延を強く抑える。
  - current max: 8.660 -> 2.977 ms。
  - archive baseline max: 35.099 -> 3.160 ms。
  - archive new max: 32.141 -> 3.928 ms。
- ただし正式置換はまだ保留。
  - archive baseline で `max_pose_flip_deg` が 0.008度だけ増え、厳格な「非悪化」判定では fail。
  - 実用上は微小差だが、現在の採択基準では quality gate を全ログ通過していない。
- 次の推奨:
  - `fast_ABCDHRF + subset_diagnostics_mode=off` を live experimental flag として試す。
  - 置換判断では `max_pose_flip_deg` に数値誤差許容、たとえば 0.01度、を入れるかを明示的に決める。
  - fallback発生frameだけを抽出し、archive baseline の 0.008度差が実害のあるpose flipか、浮動小数点/候補順の微差かを確認する。

### 生成物

- `logs/archive/performance_upgrade_ab_20260426_real_current/fast_ABCD.json`
- `logs/archive/performance_upgrade_ab_20260426_real_current/fast_ABCDHRF_sampled.json`
- `logs/archive/performance_upgrade_ab_20260426_real_current/fast_ABCDHRF_off.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_baseline/fast_ABCD.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_baseline/fast_ABCDHRF_sampled.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_baseline/fast_ABCDHRF_off.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_new/fast_ABCD.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_new/fast_ABCDHRF_sampled.json`
- `logs/archive/performance_upgrade_ab_20260426_real_archive_new/fast_ABCDHRF_off.json`

## 2026-04-26 32 blobs/cam 118fps目標 PDCA 3周

### 目標

- 32 blobs/cam、4 cameras、1 rigid body の synthetic stress で 118fps 安定運用を狙う。
- 118fps は 1 pair あたり約 8.47 ms 以下。
- 実ログは p95 4 blobs/cam 程度なので、32 blobs/cam は synthetic stress で評価する。
- 採択候補は、品質を落とさず `pipeline_pair_ms.p95` と最大遅延を下げられる経路。

### subagentsからの知見

- geometry探索では、32 blobs/cam で既存 `fast_ABCDP` pruning は候補削減率が高くても、full matrix 計算後の Python set/dict/component 分割が重く、32では不利と判断。
- 低リスク候補として次が挙がった。
  - 32では pruning を避け、64/128で再評価する。
  - `_refine_candidate_observation()` の no-drop 経路で triangulation / reprojection を二重計算しない。
  - forward epipolar gate を通った候補だけ backward distance を計算する。
  - 2-view triangulation に `cv.triangulatePoints()` fast path を入れる。
- benchmark helper として `tests/perf/synthetic_32_blob_benchmark.py` を追加し、32 blobs/cam の同一入力ABを再実行しやすくした。

### PDCA 1: rigid hint-only geometry

- 仮説:
  - object gating が成立している frame では、generic epipolar triangulation と rigid-hint triangulation の二重計算が無駄。
  - rigid-hint triangulated points をそのまま本線 `points_3d` として使えば、generic epipolar matching を飛ばせる。
- 実装:
  - `fast_ABCDH`: object-gated filtered path で `use_rigid_hint_as_generic=True` を使う。
  - `src/host/geo.py` に observations だけを準備する `prepare_observations_for_paired_frames()` を追加。
  - hint-only時は `rigid_hint_triangulated_points` を `triangulated_points` / `points_3d` に流用する。
- 結果:
  - `fast_ABCD`: `pipeline_pair_ms.p95` 6.017 ms、max 35.241 ms。
  - `fast_ABCDH`: `pipeline_pair_ms.p95` 4.035 ms、max 9.782 ms。
  - p95 は約 33% 改善し、118fps p95目標を通過。
- 判断:
  - 採択候補。安定追跡中は generic epipolar を二重に回さない方針が有効。

### PDCA 2: geometry hot-path micro optimizations

- 仮説:
  - hint-only以外の fallback や generic path でも、triangulation/refinementの重複計算を削れば安定性が上がる。
- 実装:
  - `fast_ABCDF`: generic path 用 hot-path optimization。
  - `fast_ABCDHF`: hint-only + hot-path optimization。
  - `fast_ABCDHRF`: hint-only + rigid candidate separation + hot-path optimization。
  - `src/host/geo.py`
    - no-drop refine で計算済み `point_3d` と residuals を再利用。
    - 2-view triangulation で `cv.triangulatePoints()` を使う fast path を追加。
    - forward epipolar gate 後に必要候補だけ backward distance を計算する。
- 結果:
  - `fast_ABCDF`: `pipeline_pair_ms.p95` 4.919 ms、baseline比 約 17.8%改善。
  - `fast_ABCDHRF`: `pipeline_pair_ms.p95` 3.792 ms 前後、hint-onlyよりさらに改善。
- 判断:
  - hot-path optimization は fallback保険として有効。
  - 本命は `fast_ABCDHRF`。

### PDCA 3: marker-index direct rigid solve + live diagnostics off

- 仮説:
  - rigid hint points は marker index を持っているため、汎用 correspondence 探索をせず、marker index 直結Kabschで解ける。
  - subset diagnostics は committed pose に使っていないため、live表示では off にする方が最大遅延を安定して下げられる。
- 実装:
  - `src/host/rigid.py`
    - rigid candidate separation で `marker_idx` と点を保持。
    - marker index に対応する reference points と observed points で直接 Kabsch solve。
  - `fast_ABCDHRF` を combined experimental candidate として追加。
  - synthetic benchmark helper は `--subset-diagnostics-mode off` を受けられるようにした。
- 結果:
  - `fast_ABCDHRF` default sampled:
    - `pipeline_pair_ms.p95` 3.562 ms。
    - `pipeline_pair_ms.max` 9.641 ms。
    - p95は十分だが、sampled subset diagnostics により max は8.47msを少し超える。
  - `fast_ABCDHRF` + `subset_diagnostics_mode=off`:
    - `pipeline_pair_ms.mean` 3.118 ms。
    - `pipeline_pair_ms.p95` 3.356 ms。
    - `pipeline_pair_ms.max` 3.726 ms。
    - `rigid_ms.p95` 0.807 ms。
    - `triangulation_ms.p95` 1.329 ms。
    - `poses_estimated` は baseline と同じ。
    - `accepted_points` は baseline と同じ。
    - 118fpsの p95 / max 目標を両方通過。
- 判断:
  - 32 blobs/cam 118fps の採択候補は `fast_ABCDHRF` + live表示時 `subset_diagnostics_mode=off`。
  - diagnostics は replay / sampled / background に逃がし、committed tracking の hot path から外す方針がよい。

### 採択候補

- 第一候補:
  - `fast_ABCDHRF` + `subset_diagnostics_mode=off`。
  - object gating 成立中は rigid hint-only geometry。
  - marker index direct Kabsch。
  - geo hot-path optimizations。
  - full generic fallback は維持。
- 採択条件:
  - 実機liveで object gating 成立率が十分に高いこと。
  - fallback時の品質非悪化を replay で確認すること。
  - subset diagnostics は live表示経路では off、検証/replayでは sampled/full に分けること。

### 実行コマンド

- `PYTHONPATH=src .venv/bin/python -m pytest tests/test_geo_blob_assignment.py tests/test_rigid_reprojection_scoring.py tests/test_tracking_pipeline_diagnostics.py tests/test_tracking_runtime.py tests/test_tracking_replay_harness.py -q`
  - Result: 74 passed。
- `PYTHONPATH=src .venv/bin/python tests/perf/synthetic_32_blob_benchmark.py --pairs 300 --warmup-pairs 80 --variants fast_ABCD,fast_ABCDF,fast_ABCDH,fast_ABCDHR,fast_ABCDHRF --subset-diagnostics-mode off`

## 2026-04-26 blob数確認と32 blobs/cam stress調査

### 目的

- 実ログABで `fast_ABCDR` が object-gated fast path 使用率を大きく増やした一方、p95速度ゲートに届かなかった理由を確認する。
- 現在の実ログが高blob負荷ではない可能性があるため、実ログのblob数を集計し、32 blobs/cam の synthetic stress で挙動を見る。

### 実ログのblob数

- `logs/tracking_gui.jsonl`
  - `pi-cam-01`: 平均 2.85、p50 3、p95 4、max 5。
  - `pi-cam-02`: 平均 3.67、p50 4、p95 4、max 5。
  - ペア合計はおおむね 6-8 blobs/pair。
  - slow-pair上位例も 3 + 4 = 7 blobs。
- `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`
  - 両カメラとも平均約 3.7、p95 4、max 5。
- `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`
  - 両カメラとも平均約 3.8、p95 4、max 4。

### 解釈

- 今回の実ログABは、32/64/128 blobs/cam のような高blob負荷ではない。
- `fast_ABCDR` が実ログp95で勝てなかった理由は、blob数が少なく、削減メリットより candidate separation の追加コストが見えやすかったためと考えられる。
- 多blob、多カメラ、多剛体では、`fast_ABCDR` のように object-conditioned / rigid-specific に候補を分ける設計が効きやすくなる可能性が高い。

### 32 blobs/cam generic epipolar pruning stress

- 条件:
  - 4 cameras。
  - 32 blobs/cam。
  - 真のmarker 4個 + deterministic noise blobs。
  - `fast_ABCD` と `fast_ABCDP` を同一入力で比較。
- 結果:
  - `fast_ABCD`: p95 10.802 ms。
  - `fast_ABCDP`: p95 11.409 ms。
  - 品質: reconstructed points / matches は一致。
  - pruning は候補を 3,072 -> 30 に削減し、削減率は 99.0%。
- 判断:
  - 32 blobs/cam では候補削減そのものは効いているが、grid/component/Hungarian分割の追加コストがまだ勝っており、速度は少し悪化。
  - `fast_ABCDP` は32では正式候補にしない。64/128 blobs/cam で交差点を見る価値は残る。

### 32 blobs/cam pipeline stress

- 条件:
  - 4 cameras。
  - 32 blobs/cam。
  - 同一 240 pair 入力。
  - warmup後 200 pair 測定。
  - `fast_ABCD`, `fast_ABCDR`, `fast_ABCDX` を比較。
- 結果:
  - `fast_ABCD`: `pipeline_pair_ms.p95` 15.679 ms、`rigid_ms.p95` 3.085 ms、`triangulation_ms.p95` 9.413 ms、`pipeline_pair_ms.max` 71.057 ms。
  - `fast_ABCDR`: `pipeline_pair_ms.p95` 14.985 ms、`rigid_ms.p95` 3.361 ms、`triangulation_ms.p95` 9.609 ms、`pipeline_pair_ms.max` 19.451 ms。
  - `fast_ABCDX`: `pipeline_pair_ms.p95` 14.972 ms、`rigid_ms.p95` 3.231 ms、`triangulation_ms.p95` 9.839 ms、`pipeline_pair_ms.max` 53.326 ms。
- 判断:
  - `fast_ABCDR` は pipeline p95 を約 4.4% 改善した。
  - 一方で rigid p95 は約 8.9% 悪化した。
  - 最大遅延は `fast_ABCDR` で 71.057 ms -> 19.451 ms に下がり、realtime安定化の価値が見えた。
  - このstressでは object gating が全frame成立しており、fast path使用率改善の差は見えない。主に rigid candidate separation による尾部安定化を見た結果である。

### 現時点の判断

- `fast_ABCDR` は「今の低blob実ログでp95最速」ではない。
- ただし 32 blobs/cam では pipeline p95 と最大遅延に改善があり、スケール時の土台として残す価値が高い。
- 正式採用判断には、64/128 blobs/cam と複数剛体stressで、p95速度と最大遅延のどちらを優先ゲートにするかを分けて見る必要がある。

## 2026-04-25 段階的な性能改善variant

### 目的

- マルチカメラ化、複数剛体化の前に、次段の性能安全策を検証する。
- 公式の `fast_ABCD` は変更せず、段階的variantを比較する。
  - `fast_ABCDS`: subset diagnostics の時間予算と最大仮説数。
  - `fast_ABCDG`: `fast_ABCDS` に object-gating fallback 理由の詳細内訳と、緩和した安全prefilter試行を追加。
  - `fast_ABCDR`: `fast_ABCDG` に pose solve 用 rigid-hint candidate separation を追加。
  - `fast_ABCDX`: 実験的tracking pathの統合版。GUI scene の raw points を全体上限で制限。

### 実装

- `src/host/rigid.py`
  - subset diagnostics に `subset_time_budget_ms` と `subset_max_hypotheses` を追加。
  - 予算による打ち切りは診断専用で、commit済みpose updateは変更しない。
  - rigid-hint triangulated markers から rigid-body candidate separation を追加。
- `src/host/pipeline.py`
  - `fast_ABCDS`, `fast_ABCDG`, `fast_ABCDR`, `fast_ABCDX` を追加。
  - object-gating filter reason counters を追加。
  - timestamp、blob数、stage ms、fast/fallback判断を持つ軽量slow-pair ring bufferを追加。
- `src/host/tracking_runtime.py`
  - GUI scene の `raw_points` をデフォルト64個に制限。
  - diagnostics が総payload sizeを見られるように `raw_point_count` と `raw_points_truncated` を追加。
- `src/host/tracking_replay_harness.py`
  - `--compare-performance-upgrades` を追加。
  - デフォルトreport pathは `logs/archive/performance_upgrade_ab_<timestamp>/comparison.json`。

### full replay AB結果

厳密なreplay gateは `quality_ok && pipeline_pair_ms.p95 improved && rigid_ms.p95 improved`。

#### 現行log: `logs/tracking_gui.jsonl`

- baseline `fast_ABCD`: valid poses 19,770、`pipeline_pair_ms.p95` 4.534 ms、`rigid_ms.p95` 1.463 ms。
- `fast_ABCDS`: quality ok、speed gate failed。subset budget exceeded 206回。
- `fast_ABCDG`: quality ok、speed gate failed。fast attempts 3,880、fast used 3,650。
- `fast_ABCDR`: quality ok、speed gate failed。valid poses 19,773、fast used 19,289、fallback 5,111。
- `fast_ABCDX`: quality ok、speed gate failed。replacement candidateなし。

#### archive log: `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`

- baseline `fast_ABCD`: valid poses 13,644、`pipeline_pair_ms.p95` 4.236 ms、`rigid_ms.p95` 0.676 ms、`subset_ms.max` 34.393 ms。
- `fast_ABCDS/G/R/X`: すべてquality ok、すべてp95のspeed gate failed。
- subset budget はworst-case tailを削減した。`subset_ms.max` は約6.3 msへ、`pipeline_pair_ms.max` は27.944 msから約10.4-10.8 msへ低下。

#### archive log: `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`

- baseline `fast_ABCD`: valid poses 1,469、`pipeline_pair_ms.p95` 4.525 ms、`rigid_ms.p95` 1.399 ms、`subset_ms.max` 33.283 ms。
- `fast_ABCDS/G/R/X`: すべてquality ok、すべてp95のspeed gate failed。
- subset budget はworst-case tailを削減した。`subset_ms.max` は約6.3-6.5 msへ、`pipeline_pair_ms.max` は29.349 msから約10.7-11.8 msへ低下。

### 判断

- 厳密なp95 speed gateが全logで通らなかったため、公式 `fast_ABCD` はまだ段階的tracking variantへ置き換えない。
- `fast_ABCDS/G/R/X` は replay と live tuning 用の実験variantとして残す。
- subset budgeting は tracking quality の観点では安全で、archive logのworst-case diagnostics spikeを明確に抑える。worst-case latencyをp95より重視する場合、別のlive-stability gate候補として最有力。
- rigid candidate separation は現行logで object-gated fast-path 使用率を大きく上げるが、cross-log p95 speed gateは通らなかった。

### 検証コマンド

- `PYTHONPATH=src .venv/bin/python -m pytest tests/test_tracking_pipeline_diagnostics.py tests/test_rigid_reprojection_scoring.py tests/test_tracking_runtime.py tests/test_tracking_replay_harness.py -q`
  - 結果: 55 passed。
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --compare-performance-upgrades --log logs/tracking_gui.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --ab-report-out logs/archive/performance_upgrade_ab_20260425_current/comparison.json --quiet`
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --compare-performance-upgrades --log logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --ab-report-out logs/archive/performance_upgrade_ab_20260425_archive_baseline/comparison.json --quiet`
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --compare-performance-upgrades --log logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --ab-report-out logs/archive/performance_upgrade_ab_20260425_archive_new/comparison.json --quiet`

### 生成report

- `logs/archive/performance_upgrade_ab_20260425_current/comparison.json`
- `logs/archive/performance_upgrade_ab_20260425_archive_baseline/comparison.json`
- `logs/archive/performance_upgrade_ab_20260425_archive_new/comparison.json`

## 2026-04-25 fast_ABCD 公式path

### 目的

- blob数、剛体数、カメラ数が増える状況に備え、host側realtime tracking latencyを下げる。
- 旧pathを `baseline` として残しつつ、差し替え可能なfast variantを検証する。
- tracking qualityを維持する。valid/reacquire/jump/flip/reprojection metricsを悪化させない。

### 実装したpipeline variant

- `baseline`: 従来挙動。replay比較とrollback用に保持。
- `fast_A`: host geometry acceleration.
  - cameraごとのbatch undistort。
  - vectorized symmetric epipolar cost matrix。
- `fast_AB`: stage timing detailsを追加。
  - `undistort_ms`, `epipolar_match_ms`, `generic_triangulation_ms`, `object_gating_ms`, `rigid_hint_triangulation_ms`, `rigid_pose_ms`, `subset_ms`, `fallback_ms`.
- `fast_ABC`: subset diagnosticsをhot-path pairごとではなくsampledに変更。
  - commit済みposeは変更しない。
  - 30 subset frameごとに加え、invalid/reacquire/risk frameで評価する。
- `fast_ABCD`: 安全な場合に object gating で generic triangulation input を絞り、同一frame内でfull generic fallbackする。
  - tracker stateはfast/fallback選択後に一度だけ更新する。
  - `fast_path_used_count`, `fallback_count`, `fallback_reason_counts`, `filtered_blob_count`, `full_blob_count` をreportする。
- `fast_ABCDE`: replay/live-ready実験用のlatest-only/backpressure simulation。
  - replayでsimulated dropが発生したため、公式pathには選ばない。

### 公式path判断

- default `TrackingPipeline()` と通常replay pathは `fast_ABCD` を使用する。
- `fast_ABC` 以降では `subset_diagnostics_mode` のdefaultを `sampled` にする。
- `baseline` は `--pipeline-variant baseline` で選択可能なまま残す。
- 厳密な置き換えにはdropped pairゼロが必要なため、`fast_ABCDE` は実験扱いのまま残す。

### full replay AB結果

#### 現行log: `logs/tracking_gui.jsonl`

- 入力: 48,800 frames / 24,400 pairs。
- valid poses: 19,770 -> 19,770。
- `pipeline_pair_ms.mean`: 6.217 -> 3.611 ms、41.9%高速化。
- `pipeline_pair_ms.p95`: 7.830 -> 4.431 ms、43.4%高速化。
- `rigid_ms.mean`: 3.810 -> 1.414 ms、62.9%高速化。
- `rigid_ms.p95`: 5.291 -> 1.684 ms、68.2%高速化。
- `triangulation_ms.mean`: 1.964 -> 1.768 ms、10.0%高速化。
- `triangulation_ms.p95`: 2.154 -> 2.180 ms、1.2%低速化。
- quality: reacquire count、pose jumps、mode transitions、max flip、max jump、reprojection p95 は不変。
- `fast_path_used_count`: 3,650 / 24,400 pairs, 15.0%.
- `fallback_count`: 20,750.
- 主なfallback理由:
  - `insufficient_object_gating`: 20,584.
  - `filtered_hint_quality_gate`: 166.

#### archive log: `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`

- 入力: 30,452 frames / 15,226 pairs。
- valid poses: 13,644 -> 13,644。
- `pipeline_pair_ms.mean`: 25.451 -> 4.327 ms、83.0%高速化。
- `pipeline_pair_ms.p95`: 26.591 -> 4.025 ms、84.9%高速化。
- `rigid_ms.mean`: 22.233 -> 1.281 ms、94.2%高速化。
- `rigid_ms.p95`: 23.033 -> 0.762 ms、96.7%高速化。
- `triangulation_ms.mean`: 2.732 -> 2.625 ms、3.9%高速化。
- `triangulation_ms.p95`: 2.994 -> 2.876 ms、4.0%高速化。
- quality: regressionなし。
- `fast_path_used_count`: 13,230 / 15,226 pairs, 86.9%.

#### archive log: `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`

- 入力: 3,272 frames / 1,636 pairs。
- valid poses: 1,469 -> 1,469。
- `pipeline_pair_ms.mean`: 23.988 -> 4.243 ms、82.3%高速化。
- `pipeline_pair_ms.p95`: 25.072 -> 4.073 ms、83.8%高速化。
- `rigid_ms.mean`: 20.814 -> 1.225 ms、94.1%高速化。
- `rigid_ms.p95`: 21.835 -> 1.206 ms、94.5%高速化。
- `triangulation_ms.mean`: 2.727 -> 2.618 ms、4.0%高速化。
- `triangulation_ms.p95`: 3.050 -> 2.911 ms、4.6%高速化。
- quality: regressionなし。
- `fast_path_used_count`: 1,315 / 1,636 pairs, 80.4%.

### outlier調査

- `fast_ABCD` は mean/p95 latencyを改善するが、現行log replayではまれにmax-latency outlierが出た。
- trace replayでは3つの原因が見えた。
  - Python/OS scheduling gap: internal stage totalは低いまま、`pipeline_pair_ms` だけが跳ねる。
  - sampled subset diagnostics spike: risk/interval frameで `subset_ms` が20-30 ms前後になる例。
  - 非再現性のtriangulation spike。deterministic log contentではなくruntime schedulingが原因の可能性が高い。
- これらのspike中もtracking qualityは変わらなかった。
- worst-case latencyが重要になる場合、次の対象は subset diagnostics budgeting または `fast_ABCD_no_subset` 比較path。

### 検証コマンド

- `PYTHONPATH=src .venv/bin/python -m pytest tests/test_geo_blob_assignment.py tests/test_rigid_reprojection_scoring.py tests/test_tracking_pipeline_diagnostics.py tests/test_tracking_replay_harness.py -q`
  - 結果: 50 passed。
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --log logs/tracking_gui.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --pipeline-variant baseline --out logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline.json --quiet`
- `PYTHONPATH=src .venv/bin/python -m src.host.tracking_replay_harness --log logs/tracking_gui.jsonl --calibration calibration --rigids calibration/tracking_rigids.json --patterns waist --pipeline-variant fast_ABCD --subset-diagnostics-mode sampled --out logs/archive/fast_pipeline_ab_20260425_full_abcd/fast_ABCD.json --quiet`

### 生成report

- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/fast_ABCD.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline_archive_baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/baseline_archive_fast_ABCD.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/new_archive_baseline.json`
- `logs/archive/fast_pipeline_ab_20260425_full_abcd/new_archive_fast_ABCD.json`
