# Sim Performance Upgrade Log V2

この文書は、`tools/sim/` で body-mounted scenario の精度・性能最適化 PDCA をやり直すための実験前提と入口を定義する。過去の探索履歴は `docs/10_in_progress/sim_performance_upgrade_log.md` に残し、V2 では「実運用に近い前提で、どの順序で信じられる改善にしていくか」を正とする。

## V2 の目的

V2 では、sim を fixed gate tuning の場にしない。実ログに近い body-mounted 条件で、GUI 正式経路の精度を壊さずに、性能のボトルネックを小さく切り分ける。

採用候補にする条件:

- `wrong_ownership_count == 0`
- `marker_source_confusion_count == 0`
- pose jump / identity flip を増やさない
- rigid ごとの `valid_frame_ratio` を落とさない
- `pipeline_pair_ms` / `rigid_ms` / `geometry_ms` の改善が startup spike や短尺 run 偶然ではない
- GUI 正式経路へ説明可能な変更である

速度だけ良い変更は採用候補にしない。特に continuity guard を外す変更は、まず上限性能の観測として扱い、GUI default へ入れる候補にはしない。

## 実験前提

### Scenario

主 scenario は `five_rigid_body_occlusion_v1`。

この scenario は body-mounted hard regression として扱う。random false blob で gate を最適化するのではなく、体による片側遮蔽、座り気味 waist、足交差、camera ごとの true blob size 差を見る。

固定する前提:

- `--scenario five_rigid_body_occlusion_v1`
- `--frames 240` 以上
- `--fps 118`
- `--camera-rig-source auto`
- `--rigid-stabilization-profile gui_live`
- `--false-blobs-per-camera 0`
- `--noise-px 0.05` / `0.10` を green candidate
- `--noise-px 0.15` を seed repeat caution
- `--noise-px 0.20` / `0.25` を robustness red boundary

`camera_rig_source=auto` は body-mounted 系 scenario では `cube_top_2_4m_aim_center` に解決される。camera は 2.4m cube 上面4隅に配置し、cube center を aim する。これは平行4cameraより実装着の囲い込みに近いが、full body mesh や実カメラ姿勢の完全再現ではない。

### Blob size

`five_rigid_body_occlusion_v1` は実ログ寄りの camera-specific true blob area を出す。

- `pi-cam-01` / `pi-cam-03` 相当は大きい blob
- `pi-cam-02` / `pi-cam-04` 相当は等価 diameter 約 4px 前後の小さい blob
- body-mounted scenario では camera ごとの初期 body depth 中央値を基準に、近い marker ほど等価 diameter を緩やかに増やす
- random false blob は追加しない

このため、`object_gating_pixel_max_px` の sweep は「false blob を消す最適化」ではない。予測投影と true blob centroid の距離 tail をどこまで許容するかを見る robustness 実験として扱う。

### GUI 正式経路

V2 baseline は GUI 正式経路と同じ意味を持つ `gui_live` profile にする。

- `pipeline_variant`: `fast_ABCDHRF`
- `subset_diagnostics_mode`: `off`
- `object_conditioned_gating`: enabled
- `object_gating_enforced`: true
- `object_gating_pixel_max_px`: `4.5`
- `pose_continuity_guard`: enabled / enforced
- `position_continuity_guard`: enabled / enforced
- `reacquire_guard_enforced`: true
- `reacquire_guard_shadow_enabled`: false
- `reacquire_guard_event_logging`: false

読み方:

- `fast_ABCDHRF` は object-gated fast path、rigid hint geometry、rigid candidate separation、subset budget、geometry hotpath を束ねた GUI live variant。
- `subset_diagnostics_mode=off` は subset 仮説探索を回さず、object gating / rigid hint / continuity guard を主経路にする設定。
- `object_conditioned_gating` は予測 pose から marker を 2D 投影し、近い blob だけを候補にする。
- `object_gating_enforced` は gating 結果を診断ではなく pose 選択に使う。
- `pixel max 4.5px` は gate 上限であり、blob size filter ではない。
- `pose_continuity_guard` は遮蔽時の回転ジャンプや identity flip を抑える。
- `position_continuity_guard` は遮蔽時の位置ジャンプを速度・加速度制約で抑える。
- `reacquire_guard` は LOST / REACQUIRE 復帰候補の 2D score と prediction innovation を検査する。

## Baseline command

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 0 \
  --seed 45 \
  --out logs/sim/five_rigid_body_occlusion_v1/v2_baseline_gui_live
```

`summary.json` で最低限見る項目:

- `camera_rig_source` が `cube_top_2_4m_aim_center`
- `wrong_ownership_count`
- `marker_source_confusion_count`
- `valid_frame_ratio`
- `position_delta_error_m`
- `rotation_delta_error_deg`
- `pipeline_pair_ms`
- `rigid_ms`
- `geometry_ms`
- `blob_area_summary.by_camera`
- `variant_metrics`
- `tracking.*.object_gating`
- `tracking.*.pose_continuity_guard`
- `tracking.*.position_continuity_guard`
- `tracking.*.reacquire_guard`

## V2 の入口

### Entry 0: 測定系の整理

最初の PDCA はアルゴリズム変更ではなく、測定の信用度を上げる。

やること:

- warmup frame を除外した p95 を summary に追加する
- startup spike と steady-state cost を分ける
- `pipeline_pair_ms`、`rigid_ms`、`geometry_ms`、`object_gating_ms`、`rigid_hint_triangulation_ms`、`rigid_pose_ms` を同じ表で読む
- `blob_area_summary` と `object_gating.assigned distance` の分布を同じ run で確認する

採否:

- correctness が clean のまま、steady-state 指標が安定して読めるなら次へ進む
- 240 frame の p95 が startup に引っ張られる場合、短尺 run の p95 は最適化判断に使わない

### Entry 1: Green baseline repeat

同じ GUI baseline を seed 複数で repeat し、変動幅を測る。

推奨:

- seeds: `45`, `46`, `47`
- noise: `0.25`
- false blobs: `0`
- camera rig: `auto`

見ること:

- clean が seed 依存で崩れないか
- `pi-cam-02` / `pi-cam-04` の等価 diameter p95 が 4px 台にいるか
- object gate の assigned distance p95 が `4.5px` 付近に張り付いていないか
- performance p95 の改善幅が run-to-run noise より大きいか

### Entry 2: Robustness boundary

GUI baseline のまま noise を段階的に上げる。ここでは改善ではなく、崩れる境界を測る。

推奨:

- `noise-px 0.05`: green candidate
- `noise-px 0.10`: seed repeat caution
- `noise-px 0.15`: boundary
- `noise-px 0.20` / `0.25`: red boundary

見ること:

- wrong / confusion が最初に出る noise
- valid ratio が落ちる rigid
- object gate unmatched views
- pose continuity guard / position continuity guard が reject / hold / clamp した回数
- `reprojection_score.mean_error_px` / `p95_error_px`

ここで gate を変えて pass させることを目的にしない。まず GUI baseline の壊れ方を記録する。

### Entry 3: Gate は小さく触る

gate sweep は入口ではなく、boundary を見た後に限定して行う。

扱い:

- `4.5px` を GUI baseline
- `5.0px` は robustness 確認候補
- `4.0px` は tight gate の失敗確認

採用条件:

- `0.25px` noise で clean
- `0.35px` noise で baseline より悪化しない
- `0.5px` noise の壊れ方が baseline より説明可能に改善する
- performance 改善が repeat で再現する
- 実ログ replay で object-gating assigned distance tail を切りすぎない

### Entry 4: Guard toggle は採用候補ではなく ablation

`pose_continuity_guard` / `position_continuity_guard` off は、性能上限と guard cost の観測にだけ使う。

採用しない理由:

- GUI 正式経路の安全装置を削る
- occlusion 時の identity flip / pose jump に対する防御が落ちる
- sim の clean run だけでは実運用の安全性を保証できない

guard を最適化するなら、on/off ではなく「評価回数を減らす」「occlusion 判定を明確にする」「ログ集計を軽くする」のような実装改善を対象にする。

## PDCA 記録フォーマット

PDCA は完全に逐次で回す。次の cycle は前 cycle の結果を読んでから決める。

各 cycle に必ず残す:

- baseline または前 cycle との差分
- hypothesis
- command
- result
- decision
- next action

表の列:

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|

`correctness` には最低限 `wrong/confusion/jump/min valid` を入れる。
`performance` には warmup-trimmed 指標が入るまでは `pair p95 / rigid p95 / geometry p95` と startup spike 有無を併記する。

## V2 の最初の 5 cycle 案

これは事前AB計画ではなく、入口の順番の目安。実際には各 cycle の結果を読んで次を決める。

1. GUI baseline を cube-top body-mounted で測る。
2. noise を下げ、scenario geometry と centroid jitter の境界を切り分ける。
3. seed repeat で run-to-run variance を測る。
4. 近接投影の wrong / marker confusion frame を読む。
5. green candidate が seed repeat で成立してから、gate / assignment / pose selection のどれを触るか判断する。

## 採用しない入口

以下は V2 の最初の入口にしない。

- random false blob を増やして gate を最適化する
- `object_gating_pixel_max_px` だけを seed 固定で最小化する
- pose / position continuity guard off を GUI 改善として扱う
- `five_rigid_dance_swap_red_v1` の pass を目的にする
- 30 frame smoke の p95 を性能改善として採用する

## 次に必要な実装

最初に入れる価値が高い実装:

- sim summary に warmup-trimmed performance を追加する
- object gating assigned distance の summary を scenario summary に集約する
- PDCA row を自動で `json` / `md` に追記する薄い helper を用意する

この3つが揃うと、最適化対象を触る前に「改善に見えるノイズ」をかなり減らせる。

## V2 Restart PDCA 1-10

距離追従blob径を入れた状態で、PDCA を cycle 1 からやり直した。過去の V2 PDCA 表はこの節に置き換えた。

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `auto` resolved to `cube_top_2_4m_aim_center`
- camera ids: `pi-cam-01` / `pi-cam-02` / `pi-cam-03` / `pi-cam-04`
- rigid set: `head` / `waist` / `chest` / `left_foot` / `right_foot`
- runtime profile: `gui_live`
- random false blobs: `0`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_v2_restart_pdca10/`

共通 command:

```bash
PYTHONPATH=src .venv/bin/python -c 'from tools.sim.multi_rigid import DEFAULT_BODY_RIGIDS, DEFAULT_GENERATED_CAMERA_IDS, MultiRigidScenarioConfig, run_multi_rigid_scenario; ...'
```

Baseline: seed `45` / noise `0.25px` は clean ではなかった。wrong/confusion は `1/8`、min valid は `0.825`、left_foot valid は `0.825`。pair/rigid/geometry p95 は `4.75/1.94/1.74ms`。blob diameter p95 は `pi-cam-01 11.43px`、`pi-cam-02 4.91px`、`pi-cam-03 12.10px`、`pi-cam-04 4.55px`。failure は frame `72` left_foot 0/2、frame `114-115` waist 0/1、frame `199` left_foot/waist wrong owner、frame `215` right_foot 0/3。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | seed `45`, noise `0.15px` | `0.25px` red が centroid jitter由来なら、低noiseでは clean に戻る。 | wrong/confusion `0/0`、min valid `0.996`。 | pair/rigid/geometry p95 `4.49/1.70/1.69ms`、fallback `0`。 | `0.15px` は seed 45 では clean。 | `0.25px` はgreen baselineから外す。 | `0.20px` で red 境界を確認する。 |
| 2 | seed `45`, noise `0.20px` | `0.15` と `0.25` の間に marker confusion 境界がある。 | wrong/confusion `0/6`、min valid `1.000`。 | `4.51/1.72/1.69ms`、fallback `0`。 | valid は保つが frame `72`, `114`, `115` で marker入れ替わり。 | `0.20px` は red。 | 中点 `0.175px` を測る。 |
| 3 | seed `45`, noise `0.175px` | `0.20px` と同じ箇所が出るなら境界は `0.15` 近傍。 | wrong/confusion `0/6`、min valid `0.996`。 | `4.50/1.66/1.79ms`、fallback `0`。 | `0.20px` と同じ left_foot / waist confusion。 | `0.175px` も red。 | `0.1625px` を測る。 |
| 4 | seed `45`, noise `0.1625px` | `0.175px` から下げると waist側は消え、最小近接だけ残る可能性がある。 | wrong/confusion `0/2`、min valid `0.996`。 | `4.48/1.77/1.67ms`、fallback `0`。 | frame `72`、`pi-cam-04`、left_foot 0/2 だけ残る。 | 残る failure は単純なnoise量ではなく近接投影。 | `0.15625px` を測る。 |
| 5 | seed `45`, noise `0.15625px` | clean 上限が `0.15` より少し上にあるか確認する。 | wrong/confusion `0/2`、min valid `0.996`。 | `4.39/1.75/1.69ms`、fallback `0`。 | left_foot 0/2 がまだ残る。 | seed45のclean上限は `0.15` と `0.15625` の間。 | `0.153125px` を測る。 |
| 6 | seed `45`, noise `0.153125px` | 前回境界と同様に `0.153125px` は red か確認する。 | wrong/confusion `0/2`、min valid `0.996`。 | `4.40/1.77/1.67ms`、fallback `0`。 | `0.153125px` でも left_foot 0/2 confusion。 | seed45の暫定greenは `0.15px` まで。 | seed repeat で `0.15px` が信頼できるか確認する。 |
| 7 | seed `46`, noise `0.15px` | seed45だけの偶然cleanでなければ seed46 も clean になる。 | wrong/confusion `1/1`、min valid `0.829`。 | `4.49/1.82/1.66ms`、fallback `60`。 | frame `59` chest confusion と frame `199` left_foot/waist wrong owner。 | `0.15px` は seed repeat green ではない。 | seed46で noise を下げる。 |
| 8 | seed `46`, noise `0.10px` | `0.15px` が少し強すぎるだけなら `0.10px` で clean になる。 | wrong/confusion `1/0`、min valid `0.829`。 | `11.80/5.33/3.61ms`、fallback `59`。 | frame `199` の left_foot/waist wrong owner が残る。性能値はこのrunだけ外乱大。 | `0.10px` も seed46 ではまだ red。 | noiseなしでscenario幾何自体を確認する。 |
| 9 | seed `46`, noise `0.0px` | noiseなしでも壊れるなら scenario visibility が不成立。 | wrong/confusion `0/0`、min valid `1.000`。 | `4.40/1.70/1.77ms`、fallback `0`。 | geometry単体は clean。wrong owner はjitterで近接姿勢を踏むため。 | 観測ノイズと近接投影の相互作用が主因。 | seed46の green 下限として `0.05px` を測る。 |
| 10 | seed `46`, noise `0.05px` | `0.05px` なら seed46でも clean に戻る。 | wrong/confusion `0/0`、min valid `1.000`。 | `4.41/1.86/1.63ms`、fallback `0`。 | seed46では `0.05px` が clean。 | 現時点のseed repeat green candidate は `0.05px`。 | 次は seed `47` / `0.05px`、その後に close-marker observation model を設計する。 |

結論: 距離追従blob径を入れた後も、`0.25px` は red boundary のまま。seed `45` では `0.15px` が clean だが、seed `46` では `0.15px` も `0.10px` も frame `199` の left_foot/waist wrong owner で崩れる。noise `0.0px` と `0.05px` は seed `46` で clean なので、scenario geometry 単体ではなく、近接投影姿勢と centroid jitter の組み合わせが問題。次の入口は gate tuning ではなく、近接 marker が別々のclean blobとして残る現在の observation model を、実機の contour merge / circularity reject に寄せること。

## Real-log distance / merge observation model check

実ログ `logs/tracking_gui.jsonl` の matched observations から、blob area と camera depth の関係を確認した。tracking diagnostics の pose と matched blob index を使って近似したところ、採用済み観測では以下の傾向だった。

- `pi-cam-01`: depth p50 約 `1.57m`、area p50 約 `64.5px^2`、`area * depth^2` p50 約 `162`
- `pi-cam-02`: depth p50 約 `2.77m`、area p50 約 `12px^2`、`area * depth^2` p50 約 `94`

ただし、この matched observation 集計は `pi-cam-02` 側の採用サンプルが少なく、pose / timestamp / pattern 対応も推定を含む。ログ生成経路を辿ると、`area` は Pi 側 `src/pi/service/blob_detection.py` の `cv2.contourArea(contour)` であり、host の `FrameLogger` は値を変換せず `logs/tracking_gui.jsonl` に保存している。検出は raw frame に threshold、mask、external contour、diameter filter、circularity filter をかけ、centroid は contour moments で出す。

生blob全体の分布は以下だった。

- `pi-cam-01`: area p50 `65.5px^2`、area p95 `101.9px^2`、diameter p50 / p95 `9.13px / 11.39px`
- `pi-cam-02`: area p50 `11.0px^2`、area p95 `18.5px^2`、diameter p50 / p95 `3.74px / 4.85px`

このため、距離依存を入れるとしても、`area * depth^2` 係数をそのまま cube-top sim へ外挿するのは危ない。実際に first-pass として以下を試したが、採用しなかった。

- `five_rigid_body_occlusion_v1` の true blob area を `area ~= k / depth^2` に変更した。
- `k` は実ログ由来で、large camera side は `162`、small camera side は `94` を使う。
- 近接した投影markerは、2つのclean blobではなく merge blob として1つの `Frame.blobs` に出す。
- merge blob の GT は sidecar ledger の `merged_owners` に複数 owner として保持する。

First-pass validation:

- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_distance_merge_observation/`
- `cycle16_distance_merge_noise0153125`: 距離areaをそのままmerge径に使うと merge が `1345` 件発生し、valid が大きく崩れた。
- `cycle18_capped_merge_noise0153125`: merge threshold を `min(2.5px, 0.25 * (d1 + d2))` に抑えると merge は `85` 件まで減ったが、wrong/confusion `1/2`、waist valid `0.000`、pair/rigid/geometry p95 `10.19/2.02/6.32ms` で still fail。

読み: 実世界寄りの距離area + merge の方向性は正しいが、今回の係数とmerge判定は大きすぎる可能性が高い。このため first-pass の `area ~= k / depth^2` と active merge は採用しない。一方で、body-mounted では剛体stemの投影距離も近接時に伸びるため、blob径だけ完全固定にするのも不自然。現在の入口としては、実ログの camera-specific area 分布を土台に、各 camera の初期 body depth 中央値を基準にした弱い diameter scaling だけを採用する。次は実機blob detector の circularity / area reject 条件と contour area 分布を使って、merge blob を「残る」「弾かれる」「片方だけ残る」のどれにするかを実ログで合わせる必要がある。
