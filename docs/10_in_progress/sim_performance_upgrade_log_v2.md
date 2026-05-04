# Sim Performance Upgrade Log V2

この文書は、`tools/sim/` で body-mounted scenario の精度・性能最適化 PDCA をやり直すための実験前提と判断基準を定義する。過去の探索履歴は `docs/10_in_progress/sim_performance_upgrade_log.md` に残す。

## V2 の目的

V2 の大ゴールは2つだけにする。

- sim 上の rigid body 推定 pose が GT の実位置・実姿勢に近いこと
- その精度を保ったまま、118fps の処理予算に収まること

`wrong_ownership_count`、`marker_source_confusion_count`、valid ratio、pose jump は主目的ではなく guardrail として扱う。速度だけ良い変更は採用候補にしない。sim fidelity が怪しい場合は tracker を最適化せず、まず観測モデルを実運用へ寄せる。

## 実験前提

主 scenario は `five_rigid_body_occlusion_v1`。random false blob で gate を最適化するのではなく、体による片側遮蔽、座り気味 waist、足交差、camera ごとの true blob size 差を見る。

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

`five_rigid_body_occlusion_v1` は実ログ寄りの camera-specific true blob area を出す。

- `pi-cam-01` / `pi-cam-03` 相当は大きい blob
- `pi-cam-02` / `pi-cam-04` 相当は等価 diameter 約 4px 前後の小さい blob
- body-mounted scenario では camera ごとの初期 body depth 中央値を基準に、近い marker ほど等価 diameter を緩やかに増やす
- random false blob は追加しない

このため、`object_gating_pixel_max_px` の sweep は「false blob を消す最適化」ではない。予測投影と true blob centroid の距離 tail をどこまで許容するかを見る robustness 実験として扱う。

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

## 判断ルール

PDCA は既定順路を持たない。各 cycle の結果を読んで、次に解く blocker を選ぶ。

- pose error / identity safety が red なら、性能最適化へ進まない。
- sim の blob size、merge、reject、centroid shift が実運用とズレているなら、tracker tuning より sim fidelity を優先する。
- 同一 rigid 内 marker confusion は、pose error が小さい場合は即 fail ではなく fragility diagnostic として読む。ただし別 rigid wrong owner は強い red flag とする。
- performance は startup spike ではなく sustained 118fps budget で見る。
- continuity guard off は上限性能の ablation に限り、GUI default 採用候補にはしない。

主に見る指標:

- pose accuracy: `position_error_m`、`rotation_error_deg`、`position_delta_error_m`、`rotation_delta_error_deg`
- identity safety: `wrong_ownership_count`、`marker_source_confusion_count`、`valid_frame_ratio`
- sim fidelity: `blob_area_summary.by_camera`、近接 blob の merge / reject / centroid shift
- performance: `pipeline_pair_ms`、`rigid_ms`、`geometry_ms`、118fps over-budget run length

## 実験候補

候補は順番ではなく棚として扱う。

- seed repeat と noise boundary で、green candidate と red boundary を測る。
- close marker が実機で merge / circularity reject / centroid shift する条件を sim に入れる。
- object gating assigned distance と reprojection error の分布を summary に集約する。
- correctness が green になってから、warmup-trimmed performance と stage breakdown を見る。
- gate sweep は boundary の読み取り後に限定し、false blob 排除の自己最適化として扱わない。

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

## 補助実装

測定の信用度を上げるために入れる価値が高い実装:

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

結論: 距離追従blob径を入れた後も、`0.25px` は red boundary のまま。seed `45` では `0.15px` が clean だが、seed `46` では `0.15px` も `0.10px` も frame `199` の left_foot/waist wrong owner で崩れる。noise `0.0px` と `0.05px` は seed `46` で clean なので、scenario geometry 単体ではなく、近接投影姿勢と centroid jitter の組み合わせが問題。次の blocker は gate tuning ではなく、近接 marker が別々のclean blobとして残る現在の observation model を、実機の contour merge / circularity reject に寄せること。

## V2 Follow-up PDCA 11-20

cycle 10 の next action から完全逐次で続けた。採否判断に使ったログは `logs/sim/five_rigid_body_occlusion_v1/20260501_v2_pdca11_20/` に残す。

追加した測定補助:

- `summary.json` の `performance_budget.warmup_trimmed` に、先頭30 processed pairsを上限にした warmup-trimmed `pipeline_pair_ms` / `rigid_ms` / `geometry_ms` を追加した。
- `five_rigid_body_occlusion_v1` に optional な `--body-mount-blob-merge-factor` を追加した。既定は `0.0` で既存 scenario は変えない。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 11 | seed `47`, noise `0.05px` | seed repeat でも `0.05px` が clean なら green candidate に近い。 | wrong/confusion `0/0`、min valid `1.000`。 | pair/rigid/geometry p95 `4.22/1.58/1.57ms`、over max run `2`。 | correctness は clean だが production は rigid p95 と over run で落ちる。 | performance の読みが startup / 外乱混じり。 | warmup-trimmed summary を入れる。 |
| 12 | warmup-trimmed summary追加、seed `47`, noise `0.05px` repeat | sustained 指標なら startup spike と通常負荷を分けられる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed pair/rigid/geometry p95 `12.91/3.43/4.16ms`、trimmed over max run `9`。 | この run は warmup 後も外乱が大きく、採否判断に不適。 | 同条件 repeat で再現性を見る。 | seed `47` / `0.05px` を再測。 |
| 13 | seed `47`, noise `0.05px` repeat | cycle 12 が外乱なら通常値へ戻る。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `3.98/1.51/1.38ms`、trimmed over max run `0`。 | cycle 12 の大跳ねは再現せず。rigid p95 は僅かに閾値超え。 | correctness green、性能は境界上。 | gateを狭めて workload が減るか確認。 |
| 14 | seed `47`, noise `0.05px`, gate `4.0px` | 狭い gate で候補が減り rigid p95 が改善する。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `24.02/9.56/8.92ms`、trimmed over max run `106`。 | gate `4.0px` はこの run で sustained に悪化。 | 採用しない。gate定数最適化へ進まない。 | 近接 blob model を直接測る。 |
| 15 | merge factor `0.45`, seed `47`, noise `0.05px` | 近接 true blobs を merged contour にすると spurious marker confusion を減らせる。 | wrong/confusion `0/0`、min valid `0.825`、merge `150`。 | trimmed p95 `5.06/1.97/1.63ms`、trimmed over max run `1`。 | merge が強すぎ、left_foot と waist の valid を落とす。 | factor `0.45` は不採用。 | factor を下げる。 |
| 16 | merge factor `0.35`, seed `47`, noise `0.05px` | 明確な重なりだけに絞れば valid を戻せる。 | wrong/confusion `0/0`、min valid `0.825`、merge `77`。 | trimmed p95 `4.05/1.55/1.34ms`、trimmed over max run `0`。 | merge 件数は半減したが left_foot valid は戻らない。 | factor `0.35` も不採用。 | さらに factor を下げる。 |
| 17 | merge factor `0.25`, seed `47`, noise `0.05px` | ごく近い contour merge だけなら clean valid を維持できる。 | wrong/confusion `0/0`、min valid `0.829`、merge `30`。 | trimmed p95 `3.95/1.54/1.37ms`、trimmed over max run `1`。 | 少数 merge でも left_foot valid が落ちる。 | default merge 採用はしない。 | 以前 red だった seed46/noise0.10 で効用を見る。 |
| 18 | merge factor `0.25`, seed `46`, noise `0.10px` | merge が frame `199` の wrong owner を消せるなら診断価値はある。 | wrong/confusion `0/0`、min valid `0.829`、merge `27`。 | trimmed p95 `3.93/1.47/1.36ms`、trimmed over max run `1`。 | wrong owner は消えるが min valid は red のまま。 | merge は optional 診断 knob に留める。 | 既定を no-merge に戻して regression 確認。 |
| 19 | default no merge、seed `47`, noise `0.05px` | optional化後も既定 scenario は clean のまま。 | wrong/confusion `0/0`、min valid `1.000`、merge `0`。 | trimmed p95 `4.14/1.52/1.41ms`、trimmed over max run `0`。 | 既定 correctness は維持。rigid p95 はまだ境界上。 | merge既定採用なし。 | upper-bound ablation として position guard off。 |
| 20 | seed `47`, noise `0.05px`, position continuity guard off | guard cost が閾値超えの主因なら p95 が下がる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.05/1.48/1.38ms`、trimmed over max run `0`。 | trimmed rigid p95 は閾値内に入るが、raw rigid p95 は `1.51ms`。guard off は GUI default 採用候補ではない。 | 本体採用なし。測定補助と optional merge knob だけ残す。 | 次は merged/low-marker 区間の valid hold か、rigid stage breakdown をさらに分ける。 |

結論: 追加10周では、GUI default として採用できる精度・性能改善は見つからなかった。`0.05px` は seed `47` でも clean だが、rigid p95 は境界上。近接 blob merge は seed `46` / `0.10px` の wrong owner を消せる一方で、seed `47` / `0.05px` の left_foot valid を `0.83` 前後まで落とすため既定採用しない。残す改善は、warmup-trimmed performance summary と optional merge knob による診断性向上。次の blocker は tracker tuning ではなく、merged / low-marker 区間での valid hold と rigid stage breakdown の切り分け。

## Relaxed Body-Mounted PDCA 1-10

`five_rigid_body_occlusion_v1` は hard fixture として残し、少し緩い兄弟 scenario `five_rigid_body_occlusion_relaxed_v1` を追加した。camera rig、body motion、camera-specific blob area は同じ系統のまま、feet の遮蔽 camera 数と waist/head の遮蔽時間を少し減らす。採否判断に使ったログは `logs/sim/five_rigid_body_occlusion_relaxed_v1/20260502_relaxed_pdca10/` に残す。

Baseline: seed `47` / noise `0.05px` は correctness clean。wrong/confusion `0/0`、min valid `1.000`、trimmed pair/rigid/geometry p95 `4.44/1.69/1.61ms`。scenario は production rigid p95 で fail。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | seed `47`, noise `0.10px` | 少し緩めれば `0.10px` まで green に近づく。 | wrong/confusion `1/2`、min valid `0.829`、frames `68,199`。 | trimmed p95 `3.96/1.44/1.37ms`、over max run `2`。 | frame `199` wrong owner と frame `68` confusion が残る。 | `0.10px` はまだ red。 | 中間 `0.075px` を測る。 |
| 2 | seed `47`, noise `0.075px` | red 境界は `0.075` 付近に下がる。 | wrong/confusion `0/2`、min valid `1.000`、frame `68`。 | trimmed p95 `3.89/1.44/1.36ms`、over `0`。 | wrong と valid は戻るが marker confusion が残る。 | correctness green とは言わない。 | `0.0625px` を測る。 |
| 3 | seed `47`, noise `0.0625px` | `0.0625px` なら clean になる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `5.45/2.05/1.95ms`、over `1`。 | correctness は clean、性能は重い外乱の可能性。 | 同条件 repeat。 | `0.0625px` repeat。 |
| 4 | seed `47`, noise `0.0625px` repeat | cycle 3 の性能値が外乱なら戻る。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.46/1.74/1.53ms`、over `0`。 | correctness は再現。rigid p95 はまだ境界外。 | seed repeat へ進む。 | seed `46` / `0.0625px`。 |
| 5 | seed `46`, noise `0.0625px` | seed repeat でも `0.0625px` が clean なら relaxed green候補。 | wrong/confusion `0/2`、min valid `0.996`、frame `73`。 | trimmed p95 `4.19/1.61/1.47ms`、over `0`。 | seed46では marker confusion が出る。 | seed-repeat green は `0.0625px` では成立しない。 | seed46を `0.05px` へ戻す。 |
| 6 | seed `46`, noise `0.05px` | `0.05px` なら seed46も clean に戻る。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.35/1.68/1.64ms`、over `0`。 | correctness clean。 | `0.05px` を seed repeat候補に戻す。 | seed45 / `0.05px`。 |
| 7 | seed `45`, noise `0.05px` | 3 seed repeat で `0.05px` が clean になる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.42/1.59/1.57ms`、over `0`。 | seed45/46/47 は correctness clean。 | relaxed green correctness は `0.05px`。 | 性能 ablation を見る。 |
| 8 | seed `47`, noise `0.05px`, position guard off | position guard cost が rigid p95 超えの主因なら改善する。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.46/1.59/1.65ms`、over `0`。 | guard off でも大きく改善しない。 | safety guard off は採用しない。 | gate `5.0px` を確認。 |
| 9 | seed `47`, noise `0.05px`, gate `5.0px` | 少し広い gate で candidate churn が減る可能性がある。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `4.29/1.67/1.56ms`、over `0`。 | correctness は維持するが rigid p95 は改善しない。 | gate変更は採用しない。 | 長め repeat で sustained を確認。 |
| 10 | seed `47`, noise `0.05px`, `480` frames | green候補が長め run でも clean か確認する。 | wrong/confusion `0/0`、min valid `1.000`。 | raw p95 `4.17/1.47/1.45ms`、trimmed p95 `4.21/1.53/1.47ms`、over `0`。 | correctness は長めでも clean。performance は raw rigid p95 はpass、trimmed rigid p95は僅かに境界外。 | relaxed scenario は green-regression 入口として採用候補。 | 次は production判定を raw/trimmed/repeat で再定義するか、rigid stage breakdown を切る。 |

結論: scenario を若干緩めた効果はあった。`noise 0.05px` は seed `45/46/47` で wrong/confusion `0/0`、min valid `1.000` になり、hard scenario より green regression として扱いやすい。一方で `noise 0.0625px` は seed46で marker confusion、`0.075px` は seed47で marker confusion、`0.10px` は wrong owner と valid drop が出るため、green 条件にはしない。性能は relaxed scenario でも rigid p95 が境界上で、guard off / gate `5.0px` は採用できる改善ではなかった。

## Relaxed Algorithm PDCA 1-10

`five_rigid_body_occlusion_relaxed_v1` の boundary failure を使い、rigid / object-gating 側の最初の最適化に入った。採否判断に使ったログは `logs/sim/five_rigid_body_occlusion_relaxed_v1/20260502_algo_pdca10/` に残す。

狙った failure:

- frame `68`: seed `47`, noise `0.075px`, `right_foot` marker `0/1` が `pi-cam-04` で swap。
- frame `73`: seed `46`, noise `0.0625px`, `head` marker `3/4` が `pi-cam-01` で swap。
- frame `199`: seed `47`, noise `0.10px`, `waist:1` が `pi-cam-04` で `left_foot:0` を拾う。

変更の方向:

- object-gating の assignment 後、割当 blob が同一 camera 内の別 blob と sub-pixel 近接している場合、その assignment を曖昧として triangulation 入力から外す。
- 近接判定は default `0.60px`。`0.50px` は frame `199` を取り逃がし、`0.75px` は correctness は良いが余分に重く出やすかった。
- 最後に距離判定を vectorized にして、pair p95 の悪化を戻した。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | assigned-assigned 近接 drop `0.75px`, seed `47`, noise `0.075px` | 同一rigid marker swap は近接 assignment を落とせば消える。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `5.08/1.80/1.65ms`、over `1`。 | frame `68` の right_foot swap は消えた。 | 方向性は有効。 | seed46の head swap を測る。 |
| 2 | same change, seed `46`, noise `0.0625px` | head `3/4` swap も同じ近接条件で消える。 | wrong/confusion `0/0`、min valid `0.996`。 | trimmed p95 `4.64/1.67/1.56ms`、over `0`。 | marker confusion は消え、valid は既存と同等。 | 同一rigid swap対策は有効。 | red wrong owner に進む。 |
| 3 | same change, seed `47`, noise `0.10px` | assigned-assigned drop だけで cross-rigid wrong owner も減る可能性。 | wrong/confusion `1/0`、min valid `0.829`、frame `199`。 | trimmed p95 `5.56/2.00/1.68ms`、over `2`。 | marker confusion は消えるが waist/left_foot wrong owner は残る。 | cross-rigid 近接には不十分。 | assigned blob の近傍に unassigned blob がある場合も落とす。 |
| 4 | assigned-near-any drop `0.75px`, seed `47`, noise `0.10px` | assigned blob の隣に別 blob があるなら contour-level ambiguity として捨てる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `6.80/1.41/1.37ms`、over `0`。 | frame `199` wrong owner と valid drop が消えた。 | correctness は大幅改善。 | 同条件 repeat で性能を見る。 |
| 5 | same change repeat | cycle 4 の性能が安定するか確認。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `8.79/2.11/1.98ms`、over `6`。 | correctness は再現するが、`0.75px` は重く出やすい。 | threshold を絞る。 | `0.60px` を測る。 |
| 6 | assigned-near-any drop `0.60px`, seed `47`, noise `0.10px` | `0.60px` でも frame `199` を防ぎつつ余分なdropを減らせる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `6.91/1.50/1.37ms`、over `0`。 | correctness は維持、rigid p95 は改善。 | `0.60px` が候補。 | 下限 `0.50px` を確認。 |
| 7 | assigned-near-any drop `0.50px`, seed `47`, noise `0.10px` | さらに狭くしても同じ failure を防げるか確認。 | wrong/confusion `1/0`、min valid `0.829`、frame `199`。 | trimmed p95 `6.79/1.51/1.37ms`、over `2`。 | `0.50px` では cross-rigid近接を取り逃がす。 | 不採用。 | default を `0.60px` にする。 |
| 8 | default `0.60px`, seed `46`, noise `0.0625px` | default化後も seed46 boundary を守れる。 | wrong/confusion `0/0`、min valid `0.996`。 | trimmed p95 `7.14/1.65/1.63ms`、over `0`。 | correctness は維持。 | default `0.60px` は成立。 | seed47 red boundaryを測る。 |
| 9 | default `0.60px`, seed `47`, noise `0.10px` | overrideなしで red boundary が clean になる。 | wrong/confusion `0/0`、min valid `1.000`。 | trimmed p95 `7.82/1.71/1.58ms`、over `0`。 | correctness は改善したが pair p95 が重い。 | hotpath を軽くする。 | near-any距離判定をvectorize。 |
| 10 | vectorized near-any, default `0.60px`, seed `47`, noise `0.10px` | 同じ correctness のまま pair p95 を戻す。 | wrong/confusion `0/0`、min valid `1.000`。 | raw p95 `4.99/1.71/1.63ms`、trimmed p95 `4.91/1.71/1.62ms`、over `0`。 | correctness 改善は維持し、pair p95 は通常域に戻った。 | 採用候補。 | hard scenario と実ログ replay で過剰dropがないか確認する。 |

結論: rigid algorithm 側の最初の改善として、object-gating の sub-pixel ambiguous blob assignment drop は有効。relaxed boundary の seed `47` / noise `0.10px` は wrong/confusion `1/2`、min valid `0.829` から `0/0`、`1.000` まで改善した。seed `46` / noise `0.0625px` の head marker confusion も消えた。default threshold は `0.60px` がよく、`0.50px` は wrong owner を取り逃がし、`0.75px` は余分に重い。次の確認は hard scenario と実ログ replay で、近接 true blobs を落としすぎて valid を削らないかを見る。

## Ambiguous Blob Guard Adoption Check

PDCA 10 の採用候補を、hard scenario と実ログで追加確認した。sim logs は `logs/sim/five_rigid_body_occlusion_v1/20260502_ambiguous_guard_hard_check/`、実ログ replay は `logs/archive/ambiguous_guard_replay_20260502/` に残す。

Hard scenario matrix:

| noise | seed 45 | seed 46 | seed 47 | read |
|---|---|---|---|---|
| `0.05px` | wrong/confusion `0/0`, min valid `1.000`, pair/rigid/geometry p95 `4.61/1.49/1.40ms` | `0/0`, `1.000`, `4.56/1.50/1.38ms` | `0/0`, `1.000`, `4.43/1.49/1.34ms` | clean。 |
| `0.10px` | `0/2`, `1.000`, `4.44/1.49/1.36ms` | `0/0`, `0.996`, `4.58/1.55/1.36ms` | `0/0`, `1.000`, `4.44/1.48/1.35ms` | wrong owner は出ないが、seed45 head marker `3/4` confusion が1 frame残る。 |
| `0.15px` | `0/0`, `0.996`, `4.45/1.51/1.35ms` | `0/1`, `0.996`, `4.61/1.51/1.36ms` | `0/0`, `1.000`, `4.45/1.50/1.36ms` | high-noise boundaryでは chest marker confusion が1件残る。 |

Hard scenario の結論: guard は hard side でも wrong owner を増やさず、`0.05px` の green repeat は維持した。一方で `0.10-0.15px` の high-noise marker confusion を完全には消さない。これは「採用してよいが、hard noise boundary を完全解決したとは言わない」という扱いにする。

実ログ確認:

- `logs/tracking_gui.jsonl` は camera が `pi-cam-01` / `pi-cam-02` の waist中心ログで、埋め込み diagnostics は `108` events / `108.26s`。
- diagnostics summary では waist valid ratio `0.574`、failure は主に accepted points不足、pose jump/flip。これは今回の close-blob guard とは別軸の低観測数問題。
- 5 rigid 全部で replay すると、ログ内容にない head/chest/feet が常時 invalid になり、判断材料として不適だった。
- waist-only replay で guard OFF (`0.0px`) と default (`0.60px`) を比較した。

| replay | frames processed | poses estimated | reacquire | max jump / flip | object gating | pair/rigid p95 | read |
|---|---:|---:|---:|---|---|---|---|
| OFF `0.0px` | `12594` | `7794` | `145` | `0.618m` / `174.44deg` | assigned views `7/8`, evaluated `1` | `18.86/18.09ms` | baseline。 |
| default `0.60px` | `12594` | `7794` | `145` | `0.618m` / `174.44deg` | assigned views `7/8`, evaluated `1` | `17.99/17.26ms` | behavior悪化なし。 |

実ログの結論: この waist実ログでは close-blob ambiguity がほぼ発火せず、default `0.60px` は valid / reacquire / jump / flip を悪化させなかった。性能は同等から僅かに軽いが、実ログ replay の p95 自体は高く、今回の guard とは別に低観測数・reacquire 多発ログとして扱う。

採用判断: `ObjectGatingConfig.ambiguous_blob_min_separation_px = 0.60` を GUI/live default 候補として維持する。次はアルゴリズムをさらに触る前に、hard high-noise marker confusion の残件を「pose側で吸収するべきか、観測品質として棄却するべきか」に分ける。

## Residual Marker Confusion Split

hard high-noise に残った marker confusion を、観測側で棄却すべきものと pose側で扱うべきものに分けた。logs は `logs/sim/five_rigid_body_occlusion_v1/20260502_residual_confusion_pdca/` に残す。

観察:

- seed `45` / noise `0.10px` / frame `74`: `head` marker `3/4` が `pi-cam-01` で swap。blob center distance は `1.53px`、平均blob直径に対して `0.17x`。これは実 contour なら重なり扱いに近く、観測 assignment を落とすべき。
- seed `46` / noise `0.15px` / frame `59`: `chest` marker `0/4` が `pi-cam-02` で近接。distance は `1.74px`、平均blob直径に対して `0.43x`。観測側で落とせるが、閾値を広げると別の pose境界を誘発する。
- ratio `0.50` や固定 `1.8px` は、chest 残件を消す代わりに frame `85` の `head` marker `3/4` confusion を出した。この frame は distance `4.59px`、直径比 `1.31x` で close observation ではなく pose/temporal identity 側の問題。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | fixed threshold `1.8px`, seed45/noise0.10 | head `3/4` は fixed px を上げれば落とせる。 | wrong/confusion `0/0`, min valid `1.000`。 | pair/rigid/geometry p95 `4.99/1.70/1.61ms`。 | head frame74 は消える。 | 方向は有効だが広すぎる可能性。 | chest残件で確認。 |
| 2 | fixed threshold `1.8px`, seed46/noise0.15 | chest `0/4` も消える。 | wrong/confusion `0/2`, min valid `0.996`。 | `4.91/1.73/1.58ms`。 | chestは消えたが head frame85 が新規発生。 | fixed px 拡大は不採用。 | blob径比で close observation だけ落とす。 |
| 3 | diameter ratio default first pass `0.50`, seed45/noise0.10 | blob重なり比なら head frame74 を消せる。 | `0/0`, min valid `1.000`。 | `5.37/1.70/1.62ms`。 | headは消える。 | ratioの方向は有効。 | chest側を見る。 |
| 4 | ratio `0.50`, seed46/noise0.15 | chestも消せるか確認。 | `0/2`, min valid `0.996`。 | `5.42/1.76/1.56ms`。 | head frame85 が再発。 | `0.50` は強すぎ。 | ratioを下げる。 |
| 5 | ratio `0.30`, seed45/noise0.10 | 明確な重なりだけなら head frame74 を消せる。 | `0/0`, min valid `1.000`。 | `5.37/1.70/1.59ms`。 | headは消える。 | ratio `0.30` は候補。 | chest側の副作用を見る。 |
| 6 | ratio `0.30`, seed46/noise0.15 | chestは残っても head frame85 を出さないか確認。 | `0/1`, min valid `0.996`。 | `5.42/1.71/1.60ms`。 | chest frame59 は残るが新規headは出ない。 | chestは pose側/別課題扱い。 | ratio上限を探る。 |
| 7 | ratio `0.40`, seed45/noise0.10 | `0.40` でも headは消える。 | `0/0`, min valid `1.000`。 | `5.38/1.82/1.62ms`。 | headはclean。 | seed46を見る。 | `0.40` の副作用確認。 |
| 8 | ratio `0.40`, seed46/noise0.15 | `0.40` が chest近傍まで触るか確認。 | `0/3`, min valid `0.996`。 | `5.40/1.77/1.52ms`。 | chestに加えて head frame85 も出る。 | `0.40` 不採用。 | default `0.30` にする。 |
| 9 | default `0.30`, seed45/noise0.10 | default化後も head残件を消せる。 | `0/0`, min valid `1.000`。 | `5.40/1.69/1.62ms`。 | correctness OK。 | 採用候補。 | seed46残件を再確認。 |
| 10 | default `0.30`, seed46/noise0.15 | default化後、chestだけが残るか確認。 | `0/1`, min valid `0.996`。 | `5.57/1.84/1.71ms`。 | chest frame59 だけ残る。 | default `0.30` 採用候補、chestは次課題。 | green repeatで副作用確認。 |

Green repeat with default `0.30`:

| noise | seed | correctness | performance |
|---|---:|---|---|
| `0.05px` | `45` | wrong/confusion `0/0`, min valid `1.000` | pair/rigid/geometry p95 `5.47/1.65/1.58ms` |
| `0.05px` | `46` | `0/0`, `1.000` | `5.34/1.73/1.57ms` |
| `0.05px` | `47` | `0/0`, `1.000` | `5.22/1.68/1.56ms` |

結論: `object_gating_ambiguous_blob_diameter_overlap_ratio = 0.30` を追加し、固定 `0.60px` だけでは拾えない明確な blob重なりを観測側で落とす。これで hard seed45/noise0.10 の head close-marker swap は消える。seed46/noise0.15 の chest frame59 は残す。これを観測側で無理に消すと head frame85 の pose/temporal identity 失敗を誘発するため、次は pose側の marker identity continuity / temporal assignment margin を扱う。性能は pair p95 は 6ms 内だが rigid p95 はまだ `1.5ms` 境界を超えるため、精度改善と性能改善は分けて扱う。

## Temporal Assignment Margin PDCA 1-10

残った `chest` frame `59` は、blob中心同士の重なりではなく、同じ blob が別 marker projection にもほぼ同じ距離で説明できる marker identity ambiguity だった。pre-update 診断では、seed `46` / noise `0.15px` / frame `59` の `pi-cam-02` で、`marker 4 -> blob 9` の competing marker margin が `0.285px`、`marker 0 -> blob 12` は `-0.154px` だった。一方、強すぎる観測棄却で出た head frame `85` は正しい assignment margin が `3.7px` 以上あり、別問題だった。

変更:

- object-gating assignment 後、assigned blob が別 marker projection にも `object_gating_ambiguous_marker_assignment_min_margin_px` 以内で説明できる場合、その view を曖昧として落とす。
- default 候補は `0.29px`。`0.25/0.28px` は chest frame59 を取り逃がし、`0.30/0.35px` は correctness は良いが余分に重く出やすかった。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | margin `0.35px`, seed46/noise0.15 | chest frame59 は marker-vs-marker margin不足として消せる。 | wrong/confusion `0/0`, min valid `0.996`。 | pair/rigid/geometry p95 `6.30/1.85/1.67ms`。 | chest は消えるが少し重い。 | 方向は有効。 | head側副作用を見る。 |
| 2 | margin `0.35px`, seed45/noise0.10 | head close-marker改善を維持できる。 | `0/0`, min valid `1.000`。 | `6.15/1.79/1.69ms`。 | correctness OK、性能は境界。 | threshold を下げる。 | `0.25px` を測る。 |
| 3 | margin `0.25px`, seed46/noise0.15 | 低めでも chest を落とせるか確認。 | `0/1`, min valid `0.996`。 | `5.54/1.57/1.48ms`。 | chest frame59 を取り逃がす。 | 低すぎ。 | 中間 `0.30px`。 |
| 4 | margin `0.30px`, seed46/noise0.15 | `0.30px` なら chest を落とせる。 | `0/0`, min valid `0.996`。 | `6.44/1.77/1.67ms`。 | correctness OK、性能は重い。 | もう少し下げる。 | `0.28px`。 |
| 5 | margin `0.28px`, seed46/noise0.15 | chest margin境界の下限を確認。 | `0/1`, min valid `0.996`。 | `5.51/1.65/1.57ms`。 | chestを取り逃がす。 | 下限は `0.28` と `0.30` の間。 | `0.30px` head側。 |
| 6 | margin `0.30px`, seed45/noise0.10 | head側の副作用を確認。 | `0/0`, min valid `1.000`。 | `15.27/5.06/3.77ms`。 | correctness OKだが性能外乱が大きい。 | repeat。 | 同条件 repeat。 |
| 7 | margin `0.30px`, seed45/noise0.10 repeat | cycle6 が外乱なら戻る。 | `0/0`, min valid `1.000`。 | raw `7.26/2.43/1.89ms`, trimmed `7.02/2.25/1.88ms`。 | 外乱だけではなく少し重い傾向。 | threshold最小側へ。 | green repeatで副作用を見る。 |
| 8 | margin `0.30px`, seed45/46/47 noise0.05 | green repeat を壊さないか確認。 | all wrong/confusion `0/0`, min valid `1.000`。 | raw pair p95 `6.17/6.05/6.62ms`, raw rigid p95 `1.97/1.85/2.02ms`。 | correctnessは維持、性能は境界超え。 | 最小閾値を探す。 | `0.29px`。 |
| 9 | margin `0.29px`, seed46/noise0.15 | chestを消す最小側として成立するか確認。 | `0/0`, min valid `0.996`。 | raw `5.81/1.82/1.66ms`, trimmed `5.78/1.80/1.65ms`。 | chestは消え、pair p95は6ms内。 | `0.29px` が候補。 | head側確認。 |
| 10 | margin `0.29px`, seed45/noise0.10 | head側もcleanなら default候補。 | `0/0`, min valid `1.000`。 | raw `6.09/1.81/1.78ms`, trimmed `6.06/1.78/1.72ms`。 | correctness OK、pair p95はほぼ境界。 | default候補として採用。 | 実ログ replay / performance最適化は別課題。 |

結論: `object_gating_ambiguous_marker_assignment_min_margin_px = 0.29px` を追加する。これで、前節で残した seed46/noise0.15 の `chest` frame59 marker confusion は消える。seed45/noise0.10 の head close-marker改善も維持する。green repeat も correctness は維持した。一方、rigid p95 はまだ `1.5ms` 境界を超えるため、このPDCAは correctness改善として採用候補、性能改善は次フェーズで別に扱う。

## Object-Gating Lightweight PDCA 1-10

correctness guard を保ったまま、object-gating の追加コストを軽くするPDCAを回した。logs は `logs/sim/five_rigid_body_occlusion_v1/20260502_lightweight_pdca/` に残す。

baseline は seed `46` / noise `0.15px` / 240 frames。wrong/confusion は `0/0` だが、pair/rigid/geometry raw p95 は `16.03/4.91/4.36ms`、warmup-trimmed p95 は `15.37/4.83/4.22ms` で、118fps運用には重すぎた。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 1 | baseline, seed46/noise0.15 | まず現状の guard コストを測る。 | wrong/confusion `0/0`, min valid `0.996`。 | raw `16.03/4.91/4.36ms`, trimmed `15.37/4.83/4.22ms`, over `45`。 | correctness はcleanだが明確に重い。 | 改善が必要。 | marker margin計算をまとめる。 |
| 2 | marker margin vectorized, full blob-pair matrix | per-assignment loop を減らせば軽くなる。 | `0/0`, min valid `0.996`。 | raw `10.48/3.39/3.35ms`, trimmed `10.44/3.38/3.26ms`, over `23`。 | 改善したが、blob全組距離がまだ重い。 | 一部有効。 | assigned blob だけを見る。 |
| 3 | assigned-blob distance matrix | 使う blob 行だけなら pair p95 を戻せる。 | `0/0`, min valid `0.996`。 | raw `5.92/1.73/1.65ms`, trimmed `5.84/1.70/1.63ms`, over `2`。 | pair p95 は6ms内、rigidはまだ少し重い。 | 採用候補。 | sqrtを外す。 |
| 4 | squared distance compare, seed46/noise0.15 | sqrtを避ければさらに軽くなる。 | `0/0`, min valid `0.996`。 | raw `6.35/2.08/1.95ms`, trimmed `6.04/1.95/1.85ms`, over `3`。 | 理論上は軽いが実測は悪化。 | 単独効果は不確か。 | head側でも見る。 |
| 5 | squared distance compare, seed45/noise0.10 | head close-marker側の correctness と性能を見る。 | `0/0`, min valid `1.000`。 | raw `6.55/1.87/1.86ms`, trimmed `6.53/1.83/1.80ms`, over `2`。 | correctness は維持、性能は境界。 | 次はprofile。 | hotspotを確認。 |
| 6 | cProfile, 120 frames | 次の軽量化対象を実測で決める。 | `0/0`, min valid `0.992`。 | profile run raw `11.92/3.84/3.45ms`。 | `evaluate_object_conditioned_gating` が `0.509s/120 frames`、`linear_sum_assignment` は `0.004s`。 | assignment solverではなくgating周辺を削る。 | blob diameterの重複計算を消す。 |
| 7 | per-camera blob diameter precompute, seed46/noise0.15 | rigidごとの blob area/diameter 再計算をやめると軽くなる。 | `0/0`, min valid `0.996`。 | raw `5.70/1.76/1.65ms`, trimmed `5.62/1.74/1.64ms`, over `2`。 | pair p95 は良好、rigidは境界超え。 | 採用候補。 | head側で確認。 |
| 8 | precompute repeat, seed45/noise0.10 | head close-marker改善を維持する。 | `0/0`, min valid `1.000`。 | raw `5.93/1.86/1.63ms`, trimmed `5.87/1.78/1.63ms`, over `2`。 | correctness は維持、pair p95は6ms内。 | precomputeは維持。 | close-blob内側 loop をもう一段試す。 |
| 9 | full threshold matrix vectorization | thresholdも行列化すれば Python loop を減らせる。 | `0/0`, min valid `0.996`。 | raw `5.71/1.85/1.80ms`, trimmed `5.62/1.83/1.75ms`, over `7`。 | threshold行列の確保が重く、overも増えた。 | 不採用。 | 小さい行単位判定へ戻す。 |
| 10 | final: diameter precompute + valid assigned rows, seed46/noise0.15 | precomputeだけ残し、余分な行列化を避ける。 | `0/0`, min valid `0.996`。 | raw `5.66/1.74/1.62ms`, trimmed `5.56/1.70/1.54ms`, over `4`。 | baseline比で大幅改善、correctness維持。rigid p95 `1.5ms` は未達。 | 採用。 | 次は object-gating 評価頻度か projection/geometry 側の分離。 |

結論: 軽量化として、per-camera blob diameter を `evaluate_object_conditioned_gating` の入口で一度だけ作り、close-blob ambiguity は assigned blob 行だけの距離行列で判定する形を採用する。これで seed46/noise0.15 の pair raw p95 は `16.03ms` から `5.66ms`、trimmed p95 は `15.37ms` から `5.56ms` まで戻った。wrong ownership と marker confusion は `0/0` を維持した。一方、rigid raw p95 は `1.74ms`、trimmed p95 は `1.70ms` で、`1.5ms` 目標はまだ未達。次はアルゴリズムの正しさをさらに触るより、object-gating を毎rigid毎frame full評価する必要があるか、projection/geometry時間をどこまで分離できるかを測る。

## Object-Gating Lightweight PDCA 11-20

前節の最終候補から、rigid p95 をさらに `1.5ms` に近づけるPDCAを続けた。logs は `logs/sim/five_rigid_body_occlusion_v1/20260503_lightweight_pdca/` に残す。

baseline は seed `46` / noise `0.15px` / 240 frames。前節より測定外乱は低く、wrong/confusion `0/0`、pair/rigid/geometry raw p95 `5.35/1.66/1.55ms`、warmup-trimmed p95 `5.27/1.65/1.50ms` だった。つまり pair はよいが rigid p95 はまだ `1.5ms` を少し超えていた。

| cycle | change | hypothesis | correctness | performance | read | decision | next |
|---|---|---|---|---|---|---|---|
| 11 | baseline final candidate, seed46/noise0.15 | 前節採用候補を再測定する。 | wrong/confusion `0/0`, min valid `0.996`。 | raw `5.35/1.66/1.55ms`, trimmed `5.27/1.65/1.50ms`, over `2`。 | かなり近いが rigid p95 は未達。 | もう一段軽量化する。 | profileで対象を確認。 |
| 12 | cProfile baseline, 120 frames | 次の主犯が object-gating か確認する。 | `0/0`, min valid `0.992`。 | profile run raw `10.36/3.37/3.10ms`。 | `evaluate_object_conditioned_gating` `0.436s/120 frames`、`_process_points` `0.435s/120 frames`、`linear_sum_assignment` `0.004s`。 | solverではなくgating本体とpose処理が同格。 | assignment payload allocationを削る。 |
| 13 | raw assignments を tuple 化, seed46/noise0.15 | dictを最終survivorだけ作れば allocation が減る。 | `0/0`, min valid `0.996`。 | raw `5.19/1.58/1.43ms`, trimmed `5.16/1.56/1.42ms`, over `2`。 | rigid p95 が大きく近づいた。 | 採用候補。 | head側副作用を見る。 |
| 14 | tuple assignment, seed45/noise0.10 | head close-marker改善を維持する。 | `0/0`, min valid `1.000`。 | raw `5.76/1.71/1.51ms`, trimmed `5.60/1.62/1.47ms`, over `3`。 | correctnessは維持。head側は少し重い。 | 候補維持。 | projection fast pathを試す。 |
| 15 | object-gating projection ndarray fast path | list-of-tuplesからndarray戻しを消せば軽くなる。 | `0/0`, min valid `0.996`。 | raw `5.70/1.77/1.54ms`, trimmed `5.64/1.72/1.52ms`, over `4`。 | 期待と逆に悪化。 | 不採用。 | fast pathを戻す。 |
| 16 | projection fast path revert, seed46/noise0.15 | cycle15の悪化が戻るか確認。 | `0/0`, min valid `0.996`。 | raw `5.39/1.66/1.48ms`, trimmed `5.39/1.66/1.47ms`, over `2`。 | 戻ったがcycle13ほどではない。 | tupleのみ残す。 | blob index再生成を試す。 |
| 17 | blob index array precompute | `np.arange(blob_count)` をrigidごとに作らない。 | `0/0`, min valid `0.996`。 | raw `5.84/1.71/1.63ms`, trimmed `5.72/1.68/1.56ms`, over `3`。 | 辞書参照増の方が勝ち、悪化。 | 不採用。 | precomputeを戻して repeat。 |
| 18 | tuple-only repeat, seed46/noise0.15 | 採用候補を再測定する。 | `0/0`, min valid `0.996`。 | raw `5.33/1.62/1.53ms`, trimmed `5.25/1.60/1.47ms`, over `2`。 | tuple化は安定して小改善。 | 採用。 | profileを再確認。 |
| 19 | cProfile tuple candidate, 120 frames | object-gating profileが下がったか確認する。 | `0/0`, min valid `0.992`。 | profile run raw `10.36/3.42/2.98ms`。 | function calls は `4.24M` から `4.19M`、object-gating は `0.436s` から `0.427s/120 frames`。小改善。 | 採用範囲は小さい。 | green repeat。 |
| 20 | final green repeat, seeds 45/46/47 noise0.05 | clean条件を壊していないか確認する。 | all wrong/confusion `0/0`, min valid `1.000`。 | raw pair p95 `5.34/5.62/5.54ms`, raw rigid p95 `1.58/1.67/1.66ms`。 | correctness維持。rigid `1.5ms` は seedにより未達。 | tuple化採用、次は `_process_points` / geometry 側へ。 | pose処理とgeometryを分けて測る。 |

結論: object-gating内の raw assignment を最初から dict にせず、内部では compact tuple として扱い、geometryへ渡す final assignments だけ dict 化する。seed46/noise0.15 repeat では raw p95 が `5.35/1.66/1.55ms` から `5.33/1.62/1.53ms`、best run では `5.19/1.58/1.43ms` まで下がった。projection ndarray fast path と blob index precompute は実測悪化のため不採用。correctness は green repeat まで `0/0` を維持した。ただし、profileでは object-gating と `_process_points` がほぼ同格まで来ているため、次に大きく軽くするなら object-gatingだけでなく pose処理/geometry側の重複 work を分けて見る。
