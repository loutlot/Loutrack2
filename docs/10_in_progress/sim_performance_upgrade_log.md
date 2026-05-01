# Sim Performance Upgrade Log

この文書は、`tools/sim/` を使って tracking pipeline / rigid body runtime の性能改善を回すときの、見るべきスコアとログの残し方をまとめる。rigid body の物理 marker 配置や subset policy の設計判断は `docs/10_in_progress/rigid_body_design.md` に置き、sim での速度・精度・採否記録はこの文書を正とする。

## 目的

sim の性能改善では、速くなったかだけでは採用しない。以下を同時に見る。

- 精度を犠牲にしていないか。
- ownership / marker confusion / pose jump が増えていないか。
- 4カメラ5剛体の負荷で、GUI/live 経路に載せても説明しやすい実装か。
- startup spike や並列実行ノイズを、改善として誤認していないか。
- green regression と red stress を混同していないか。

基本方針は「accuracy clean を守ったまま、pair p95 / rigid p95 / geometry p95 を下げる」。wrong ownership や marker confusion が出た変更は、速度が良くても採用候補ではなく、原因分析用の red stress として扱う。

## シナリオの使い分け

| scenario | 位置づけ | 採否の読み方 |
|---|---|---|
| `five_rigid_dance_occlusion` | 標準の 4カメラ5剛体 dance / occlusion 回帰 | 速度改善の主比較。accuracy clean が必須。 |
| `five_rigid_dance_hard_occlusion_v1` | cycle45 経路を GT として厳しめにした hard regression | 次の PDCA 入口。2カメラ同時 blackout、足交差、weak reacquire を見る。 |
| `five_rigid_body_occlusion_v1` | 実装着に寄せた body-mounted hard regression | false blob ではなく、体による片側遮蔽、座り気味 waist、足交差で camera 別 visibility が変わるかを見る。 |
| `five_rigid_dance_swap_red_v1` | 他 rigid 混在、occlusion、移動、回転で wrong / confusion を意図的に起こす red stress | pass を期待しない。混同の検出力と悪化量を見る。 |
| waist / wand partial occlusion 系 | 小さな closed-loop 回帰 | GUI 経路の occlusion hold や jump guard の単体確認。 |

green regression では `wrong_ownership_count == 0`、`marker_source_confusion_count == 0`、`pose_jump_count == 0` を基本条件にする。red stress では wrong / confusion が出ること自体は想定内なので、`production_go_no_go.passed` ではなく、どの rigid / marker / camera 条件で崩れたかを読む。

### Real-log review for body-mounted sim

`logs/tracking_gui.jsonl` の frame payload と diagnostics を見直した。

- frame 数は `25188`、camera は `pi-cam-01` / `pi-cam-02`。
- blob count は平均 `3.82/frame`、p95 `4`、max `5`。大量の random false blob ではなく、4点前後の true blob と欠けが中心。
- area は全体 p50 `22.5px^2`、p95 `88.5px^2`。等価 diameter は p50 `5.35px`、p95 `10.62px`。
- camera 差が大きい。`pi-cam-01` は area p50 `65.5px^2`、等価 diameter p50 `9.13px`。`pi-cam-02` は area p50 `11.0px^2`、等価 diameter p50 `3.74px`、p95 `4.85px`。
- object-gating の assigned distance は p50 `0.82px`、p95 `4.46px`、p99 `13.42px`。`waist` だけだと p95 `5.11px`。

このため、body-mounted sim で fixed pixel gate を最適化する意義は薄い。gate は false blob 専用ではなく、予測投影と blob centroid の距離窓なので、true blob が小さい実ログでは狭い gate が true observation の tail を切り得る。`five_rigid_body_occlusion_v1` は random false blob を入れず、camera 別 true blob area を実ログ寄りにして、gate tuning ではなく body-mounted occlusion / workload regression として扱う。

`five_rigid_body_occlusion_v1` smoke:

- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_body_mount_v1/`
- 240 frames / 4 camera / 5 rigid / GUI live / noise `0.25px`
- wrong / confusion: `0 / 0`
- valid frame ratio: head/chest/waist/left_foot/right_foot all `1.0`
- max position delta error: `0.0071m`
- pair p95 / rigid p95 / geometry p95: `5.21ms / 2.04ms / 1.94ms`
- `scenario_go_no_go.passed` は false。理由は既存 dance 系と同じく `rotation_delta_error_deg.max > 5deg` と `rigid_p95 > 1.5ms` で、identity jump ではない。

Implementation note: current `five_rigid_body_occlusion_v1` ignores requested random false blobs and emits real-log-like true blob areas. Generated 4 camera runs map even-rank cameras to the large-blob side of the real log and odd-rank cameras to the small-blob side, so `pi-cam-02` / `pi-cam-04` are around 4px equivalent diameter. `summary.json` includes `blob_area_summary` so future runs can verify this assumption.

Validation run after the real-log-like payload change:

- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_real_log_like_240/`
- command still passed `--false-blobs-per-camera 4`, but this scenario emitted no random false owners.
- wrong / confusion: `0 / 0`
- min valid: `1.000`
- pair p95 / rigid p95 / geometry p95: `4.45ms / 1.82ms / 1.67ms`
- `pi-cam-02` equivalent diameter mean / p95: `3.71px / 4.50px`
- `scenario_go_no_go.passed` remains false due to strict rotation delta and rigid p95 budget, not identity failure.

### Body-mounted real-log-like PDCA 1-5

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- random false blobs: `0`
- seed: `45`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca5_real_log_like_current/`
- aggregate: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca5_real_log_like_current/pdca5_rows.json`

Baseline: noise `0.25px` の GUI 経路は wrong / confusion / jump `0 / 0 / 0`、min valid `1.000`、pair p95 / rigid p95 / geometry p95 `4.90ms / 1.80ms / 1.76ms`。`pi-cam-02` equivalent diameter p95 は `4.50px`。

| cycle | hypothesis | result | decision / next |
|---|---|---|---|
| 1 | position continuity guard off で guard cost が下がるか見る。 | clean、`4.64/1.80/1.61ms`。 | pair / geometry は改善、rigid は変わらない。主要ボトルネックではない。 |
| 2 | pose continuity guard off で rotation hold 評価コストを切る。 | clean、`4.38/1.64/1.66ms`。 | 最も速いが GUI safety を削るので採用せず、上限確認へ。 |
| 3 | 両 guard off で上限性能を見る。 | clean、`4.36/1.65/1.66ms`。 | Cycle 2 から上積みなし。pose guard 単体が主な差分。 |
| 4 | noise `0.5px` の GUI baseline で centroid jitter 感度を見る。 | wrong / confusion `34 / 197`、min valid `0.000`、jump `1`、`28.43/19.84/5.97ms`。 | hard fail。jitter に対して scenario / tracker がかなり敏感。 |
| 5 | noise `0.5px` + pose guard off で guard が守っているか見る。 | wrong / confusion `0 / 16`、min valid `0.000`、jump `0`、`13.64/2.20/4.94ms`。 | baseline より壊れ方は軽いが still fail。guard より前段の assignment / pose selection が課題。 |

結論: real-log-like payload では、noise `0.25px` なら pose continuity guard off が性能を改善するが、安全装置を外すだけなので採用しない。noise `0.5px` では GUI baseline 自体が大きく崩れ、fixed gate や guard toggle よりも、centroid jitter 下での assignment / ownership / pose selection の堅牢化を次の PDCA 対象にするべき。今後この scenario を使うなら、`noise-px` を段階 sweep して「clean から崩れる境界」を回帰指標にするほうが、gate 定数 tuning より意味がある。

### Body-mounted PDCA 10 current rerun

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- seed: `45`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca10_body_mount_current/`
- aggregate: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca10_body_mount_current/pdca10_rows.json`

Baseline: GUI 経路 `fast_ABCDHRF` + gate `4.5px` + pose / position continuity guards enabled。wrong / confusion / jump は `0 / 0 / 0`、min valid は `1.000`、pair p95 / rigid p95 / geometry p95 は `4.44ms / 1.80ms / 1.66ms`。精度は clean だが、rigid p95 が `1.5ms` budget を超え、startup spike 由来の sustained budget も未達。

| cycle | hypothesis | result | decision / next |
|---|---|---|---|
| 1 | gate `4.0px` で候補処理を減らす。 | clean、min valid `1.000`、pair/rigid/geometry p95 `4.25/1.73/1.54ms`。 | 改善あり。まだ rigid budget 未達なので gate 幅の反対側を見る。 |
| 2 | gate `5.0px` で body-mounted の姿勢揺れを受け、余計な処理を減らす。 | clean、`4.19/1.64/1.63ms`。 | GUI に入れやすい最有力。もう少し広げて限界を見る。 |
| 3 | gate `5.5px` でも速度が保てるか見る。 | clean、`4.25/1.70/1.65ms`。 | gate `5.0px` より弱い。guard コストの切り分けへ進む。 |
| 4 | gate `5.0px` + position guard off で位置 guard のコストを見る。 | clean、`4.24/1.64/1.62ms`。 | Cycle 2 から意味ある改善なし。pose guard 側を見る。 |
| 5 | gate `5.0px` + pose guard off で rotation hold 評価コストを見る。 | clean、`4.17/1.59/1.59ms`。 | 速いが GUI default の安全装置を外すため採用しない。両 guard off の上限を見る。 |
| 6 | gate `5.0px` + 両 guard off で上限性能を見る。 | clean、`4.09/1.58/1.61ms`。 | 上限でも rigid p95 は `1.5ms` 未達。安全を削る方向は止める。 |
| 7 | gate `5.0px` + object gate diagnostics-only で enforcement コストを見る。 | clean、`4.47/1.88/1.57ms`。 | 遅い。object-gating enforcement は戻す。 |
| 8 | object-conditioned gating off で GUI baseline の依存度を見る。 | min valid `0.533`、jump `2`、`328.68/325.35/4.02ms`。 | hard reject。object-conditioned gating は必須。 |
| 9 | `fast_ABCDHR` で `fast_ABCDHRF` の F hotpath を外す価値を見る。 | clean、`6.90/1.68/4.17ms`。 | reject。GUI baseline の `fast_ABCDHRF` を維持する。 |
| 10 | gate `5.0px` を repeat して Cycle 2 の再現性を見る。 | clean、`4.21/1.68/1.56ms`。 | gate `5.0px` は baseline より速いが rigid budget 未達。hard / red stress 確認前に GUI default へは入れない。 |

結論: body-mounted 単体では gate `5.0px` が GUI baseline より一貫して速く、accuracy clean も維持した。ただし rigid p95 は repeat でも `1.64-1.68ms` で `1.5ms` を切れず、`scenario_go_no_go.passed` は false のまま。continuity guards off は速度改善しても GUI default の安全性を削るため採用しない。今回の 10 周では、GUI 経路へ即採用できる性能・精度改善はなし。次は gate `5.0px` を `five_rigid_dance_hard_occlusion_v1` と `five_rigid_dance_swap_red_v1` へかけ、body-mounted 専用 profile として分ける価値があるかだけを見る。

### Body-mounted PDCA 11-20 current rerun

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca20_body_mount_current/`
- aggregate: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca20_body_mount_current/pdca20_rows.json`

追加方針: Cycle 10 までの最有力だった gate `5.0px` の周辺を詰め、その後 seed `46` / `47` で再現性を見る。採用条件は引き続き wrong / confusion / jump `0 / 0 / 0`、min valid `1.000` を守り、GUI baseline より pair p95 と rigid p95 の両方を改善すること。

| cycle | hypothesis | result | decision / next |
|---|---|---|---|
| 11 | seed `45` で gate `4.75px` が `5.0px` より安定するか見る。 | clean、pair/rigid/geometry p95 `4.24/1.70/1.63ms`。 | `5.0px` より弱い。広め側をもう少し見る。 |
| 12 | seed `45` で gate `5.125px` が pair / geometry を下げるか見る。 | clean、`4.19/1.69/1.54ms`。 | pair / geometry は良いが rigid が伸びない。さらに広げる最終確認へ。 |
| 13 | seed `45` で gate `5.25px` でも accuracy clean と速度が保てるか見る。 | clean、`4.25/1.68/1.57ms`。 | 決定打なし。seed robustness へ移る。 |
| 14 | seed `46` の GUI baseline を測る。 | clean、`4.48/1.73/1.66ms`。 | seed `46` でも rigid budget 未達。gate `5.0px` を確認する。 |
| 15 | seed `46` で gate `5.0px` が再現するか見る。 | clean、`4.67/1.84/1.86ms`。 | reject。gate `5.0px` は seed 依存がある。狭める側を見る。 |
| 16 | seed `46` で gate `4.0px` が安定改善するか見る。 | clean、`4.22/1.65/1.54ms`。 | seed `46` では改善。seed `47` で再確認する。 |
| 17 | seed `47` の GUI baseline を測る。 | clean、`4.22/1.62/1.59ms`。 | baseline 自体が速い。gate `4.0px` で勝てるか見る。 |
| 18 | seed `47` で gate `4.0px` を試す。 | clean、`4.26/1.68/1.53ms`。 | reject。rigid p95 が baseline より悪い。 |
| 19 | seed `47` で gate `4.25px` を試す。 | clean、`4.19/1.70/1.58ms`。 | pair は良いが rigid が悪い。広め候補を最後に見る。 |
| 20 | seed `47` で gate `5.0px` を試す。 | clean、`4.18/1.65/1.56ms`。 | pair は良いが rigid は baseline より悪い。pixel gate 変更は採用しない。 |

結論: 追加 10 周でも accuracy は全 run で clean だったが、performance 改善は seed 依存だった。seed `45` では gate `5.0px` が良く、seed `46` では gate `4.0px` が良く、seed `47` では GUI baseline gate `4.5px` が rigid p95 最良だった。したがって、body-mounted scenario 専用でも fixed pixel gate 変更は採用しない。改善余地は gate 定数ではなく、object-gating の per-frame workload / startup spike / guard evaluation の内訳を分けて、seed に左右されにくい hot path を削る方向に移す。

### Body-mounted PDCA 1-20

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca20_body_mount/`

| cycle | change / check | wrong | confusion | min valid | pair p95 | rigid p95 | decision |
|---|---|---:|---:|---:|---:|---:|---|
| 1 | baseline, gate `4.5px` | `0` | `0` | `1.000` | `4.63ms` | `1.71ms` | baseline |
| 2 | gate `4.0px` | `0` | `0` | `1.000` | `4.24ms` | `1.60ms` | body scenario では良いが red stress 既知リスクあり |
| 3 | gate `4.25px` | `0` | `0` | `1.000` | `4.30ms` | `1.65ms` | keep as observation |
| 4 | gate `5.0px` | `0` | `0` | `1.000` | `4.59ms` | `1.83ms` | reject; slower |
| 5 | gate `5.5px` | `0` | `0` | `1.000` | `5.63ms` | `1.92ms` | reject; slower |
| 6 | gate `6.0px` | `0` | `0` | `1.000` | `4.52ms` | `1.66ms` | no clear win |
| 7 | position continuity guard off | `0` | `0` | `1.000` | `4.31ms` | `1.66ms` | not enough |
| 8 | pose continuity guard off | `0` | `0` | `1.000` | `4.60ms` | `1.68ms` | not enough |
| 9 | both continuity guards off | `0` | `0` | `1.000` | `4.53ms` | `1.54ms` | fastest but unsafe for GUI default |
| 10 | object gate diagnostics only | `0` | `0` | `1.000` | `4.56ms` | `1.91ms` | reject; slower |
| 11 | object-conditioned gating off | `0` | `0` | `0.533` | `333.74ms` | `330.22ms` | reject hard |
| 12 | `fast_ABCDHF` | `0` | `0` | `1.000` | `78.14ms` | `75.00ms` | reject hard |
| 13 | `fast_ABCDHR` | `0` | `0` | `1.000` | `8.39ms` | `1.74ms` | reject; pair p95 worse |
| 14 | `fast_ABCDH` | `0` | `0` | `1.000` | `75.27ms` | `70.02ms` | reject hard |
| 15 | `fast_ABCDR` | `0` | `0` | `1.000` | `12.54ms` | `1.83ms` | reject |
| 16 | `fast_ABCDF` | `0` | `0` | `1.000` | `84.38ms` | `75.20ms` | reject hard |
| 17 | `fast_ABCD` | `0` | `0` | `1.000` | `82.41ms` | `73.00ms` | reject hard |
| 18 | gate `5.0px`, position guard off | `0` | `0` | `1.000` | `4.72ms` | `1.69ms` | no win |
| 19 | gate `5.0px`, both guards off | `0` | `0` | `1.000` | `4.42ms` | `1.59ms` | unsafe for GUI default |
| 20 | gate `5.0px`, seed `46` | `0` | `0` | `1.000` | `5.01ms` | `1.91ms` | no win |

結論: body-mounted 単体では gate `4.0px` と continuity guard off が速く見えるが、既存 red / hard stress で守ってきた「短時間 occlusion jump を持つ GUI live 経路」としては採用しない。`object_conditioned_gating` と `fast_ABCDHRF` は必須で、外すと pair / rigid p95 が大きく悪化する。今回の 20 周では精度 clean を保ったまま GUI 正式経路へ入れる新しい performance change は見つからなかった。

### Body-mounted PDCA 10 rerun with sim overrides

共通条件:

- scenario: `five_rigid_body_occlusion_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- seed: `45`
- logs: `logs/sim/five_rigid_body_occlusion_v1/20260501_pdca10_body_mount_codex/`

Baseline: GUI 経路 `fast_ABCDHRF` は wrong / confusion `0 / 0`、min valid `1.000` を維持した。pair p95 は `4.37ms` で GUI baseline より改善余地は小さく、rigid p95 `1.73ms` と startup spike 由来の max / sustained budget が主な未達。

| cycle | change / check | wrong | confusion | min valid | pair p95 | rigid p95 | decision |
|---|---|---:|---:|---:|---:|---:|---|
| 1 | gate `4.0px` | `0` | `0` | `1.000` | `4.18ms` | `1.57ms` | observe; faster but not under rigid p95 budget |
| 2 | gate `4.25px` | `0` | `0` | `1.000` | `4.25ms` | `1.59ms` | reject; weaker than gate `4.0px` |
| 3 | gate `5.0px` | `0` | `0` | `1.000` | `4.03ms` | `1.53ms` | observe; fastest gated variant with guards intact |
| 4 | position continuity guard off | `0` | `0` | `1.000` | `4.16ms` | `1.53ms` | reject for GUI default safety |
| 5 | pose continuity guard off | `0` | `0` | `1.000` | `4.21ms` | `1.59ms` | reject; no speed win |
| 6 | both continuity guards off | `0` | `0` | `1.000` | `3.91ms` | `1.49ms` | fastest but unsafe for GUI default |
| 7 | object gate diagnostics only | `0` | `0` | `1.000` | `4.42ms` | `1.79ms` | reject; slower |
| 8 | object-conditioned gating off | `0` | `0` | `0.533` | `333.90ms` | `330.39ms` | reject hard |
| 9 | `fast_ABCDHR` | `0` | `0` | `1.000` | `7.64ms` | `1.93ms` | reject; geometry p95 worse |
| 10 | `fast_ABCDR` | `0` | `0` | `1.000` | `12.92ms` | `1.97ms` | reject hard |

次の一手: GUI baseline を変えず、body-mounted 専用の採用候補は gate `5.0px` を hard / red stress へ再確認する。cycle 6 は rigid p95 `1.49ms` まで落ちるが、continuity guards を外すため GUI default には入れない。

## 見るべきスコア

### 1. Identity / ownership

最優先で見る。

| score | 意味 | 採否目安 |
|---|---|---|
| `wrong_ownership_count` | 別 rigid の blob を自 rigid として使った回数 | green regression は必ず `0`。 |
| `marker_source_confusion_count` | marker index または source marker の混同 | green regression は必ず `0`。 |
| `valid_frame_ratio` | rigid ごとの pose が valid だった割合 | hard occlusion では一時低下を許すが、主要 rigid の継続性を比較する。 |
| `tracking.<rigid>.status_counts` | BOOT / TRACKING / LOST 等の状態分布 | valid ratio だけでなく、どの状態で耐えたかを見る。 |

wrong / confusion が出た場合は、性能数字より先にその run を「accuracy regression」または「red stress observation」に分類する。速度改善表へ混ぜない。

### 2. Pose continuity

位置・姿勢ジャンプの検出。marker index 取り違えは wrong/confusion に出なくても、軽い jump として現れることがある。

| score | 意味 | 採否目安 |
|---|---|---|
| `pose_jump_count` | pose continuity guard が jump と判定した回数 | green regression は `0` を優先。 |
| `max_pose_jump_m` | 最大位置 jump | baseline より悪化させない。 |
| `max_pose_flip_deg` | 最大姿勢 flip | 高速回転シナリオでは delta 系と併読する。 |
| `position_delta_error_m` | GT とのフレーム間位置変化誤差 | 軽い位置 jump を拾う。 |
| `rotation_delta_error_deg` | GT とのフレーム間姿勢変化誤差 | 回転中の index swap / flip を拾う。 |
| `position_error_m` / `rotation_error_deg` | GT pose との絶対誤差 | valid frame の品質を見る。 |

`scenario_go_no_go.passed` が false でも、理由が `rotation_delta_error_deg.max` のみで `pose_jump_count == 0` の場合は、実装の採否ではなくシナリオ閾値の妥当性として別に判断する。

### 3. Performance

118fps 想定では、平均だけでなく p95 と sustained spike を見る。

| score | 意味 | 採否目安 |
|---|---|---|
| `pipeline_pair_ms.mean` | ペア処理全体の平均 | 改善傾向を見る。 |
| `pipeline_pair_ms.p95` | ペア処理全体の p95 | 主採用指標。 |
| `pipeline_pair_ms.max` | 最大 spike | startup spike か通常 spike かを分ける。 |
| `rigid_ms.p95` | rigid estimator の p95 | GUI/live の体感と直結。 |
| `geometry_ms.p95` | triangulation / geometry 側の p95 | object-gating / hint hotpath の改善確認。 |
| `production_go_no_go` | 118fps budget の機械判定 | 採用前の gate。ただし startup spike 由来かを見る。 |
| `performance_budget.pipeline_pair_over_8_475ms_count` | 118fps 1 frame budget 超え数 | sustained over と併読する。 |
| `pipeline_pair_no_sustained_over_8_475ms` | 連続 budget 超えがないか | startup だけでなく継続遅延を見る。 |

parallel contention がある run は速度比較に使わない。accuracy clean の確認には使えても、p95 改善量の採用根拠にはしない。

### 4. Variant / fallback diagnostics

速度が変わった理由を見るために、summary の raw timing だけでなく runtime diagnostics も残す。

| score / log | 見ること |
|---|---|
| `variant_metrics` | 採用 variant、fallback、hint hit などの内訳。 |
| object-gating diagnostics | gated blobs、assigned view counts、fallback 理由。 |
| rigid-hint diagnostics | hint pose が使われたか、prediction と合ったか。 |
| tracking event status | lightweight event path が hot path を汚していないか。 |
| slow pair traces | startup spike / sustained spike / false blob burst の切り分け。 |

実装の素直さを見るときは、「速いが特殊ケースだけを増やす」変更より、「同じ情報を一度だけ作って複数箇所で読む」変更を優先する。

## ログの残し方

比較用ログは repo 内の `logs/sim/` 以下に残す。短時間の作業用出力は一時領域でもよいが、PDCA の採否判断に使った run は必ず repo-relative path で追える形にする。

推奨 layout:

```text
logs/sim/<scenario>/<YYYYMMDD>_cycle<NN>_<label>/
  summary.json
  eval.json
  command.txt
  notes.md
```

`command.txt` には実行コマンドをそのまま残す。`notes.md` には、変更内容、採否、baseline との差分、気になった spike / failure を短く書く。`summary.json` と `eval.json` は sim の `--out` が出力するものをそのまま保持する。

最低限 `notes.md` に残す項目:

| item | 内容 |
|---|---|
| scenario | `five_rigid_dance_occlusion` など。 |
| frames / fps / seed | 長さ、fps、乱数 seed。 |
| camera count | 2カメラ / 4カメラ。 |
| rigid count / marker layout | 5剛体5marker など。 |
| runtime profile | `gui_live`、baseline、試験 variant。 |
| stress knobs | occlusion、他 rigid 混在、alias blob、dropout、足交差など。 |
| implementation change | どの hot path / guard / cache を変えたか。 |
| baseline | 比較元 cycle と主要 score。 |
| decision | accept / reject / profile-only / red-observation。 |

## PDCA 記録テンプレート

```markdown
### Cycle NN - <短い名前>

Plan:
- 何を速くするか、何を壊してはいけないか。

Do:
- 変更した path。
- 実行した scenario / frames / profile。

Check:
| metric | baseline | cycle NN | change |
|---|---:|---:|---:|
| pair mean | `...ms` | `...ms` | `...%` |
| pair p95 | `...ms` | `...ms` | `...%` |
| rigid p95 | `...ms` | `...ms` | `...%` |
| geometry p95 | `...ms` | `...ms` | `...%` |
| valid min | `...` | `...` | `...` |
| wrong/confusion/jump | `.../.../...` | `.../.../...` | `...` |

Act:
- accept / reject / repeat / red stressへ移す。
- 次に見る blocker。
```

採用前には同一条件で最低1回は sequential repeat する。profile-only cycle は table に残してよいが、性能改善率の分母・分子にしない。

## 採否ルール

accept の基本条件:

- green regression で `wrong_ownership_count == 0`。
- green regression で `marker_source_confusion_count == 0`。
- green regression で `pose_jump_count == 0`。
- `valid_frame_ratio` が baseline より意味のある悪化をしていない。
- `pipeline_pair_ms.p95` または `rigid_ms.p95` が改善、または実装が明らかに単純化して次の改善の足場になる。
- sequential repeat で同じ方向の結果が出る。

reject の典型:

- wrong/confusion が green regression に出た。
- position / rotation jump が増えた。
- p95 は良いが max spike が sustained に悪化した。
- startup-only 改善を通常 frame 改善として扱っている。
- 並列実行や他プロセス負荷で timing が汚れている。
- special-case が増えて、pipeline / rigid の説明可能性が下がる。

## 現在の既知 baseline

### 5-rigid standard PDCA cycle 1-50

Latest same-scenario PDCA on `five_rigid_dance_occlusion`:

| cycle | change / check | pair mean | pair p95 | rigid p95 | geometry p95 | accuracy / ownership |
|---|---|---:|---:|---:|---:|---|
| 1 | 5-marker seed + whitelist policy | `20.93ms` | `21.43ms` | `6.93ms` | `8.76ms` | valid `1.0`, wrong/confusion `0/0` |
| 2 | rigid-hint score reuses gated observations | `20.32ms` | `26.66ms` | `6.29ms` | `9.87ms` | valid `1.0`, wrong/confusion `0/0` |
| 3 | attempted projection cache in wrong scope | rejected | rejected | rejected | rejected | caught by sim `NameError`; patch fixed |
| 4 | camera projection cache in hint scoring | `18.41ms` | `19.04ms` | `3.80ms` | `8.27ms` | valid `1.0`, wrong/confusion `0/0` |
| 5 | lightweight metric summaries in geometry diagnostics | `13.85ms` | `14.63ms` | `3.44ms` | `4.19ms` | valid `1.0`, wrong/confusion `0/0` |
| 6 | vectorized object-gating distance matrix | `10.00ms` | `10.47ms` | `3.37ms` | `4.47ms` | valid `1.0`, wrong/confusion `0/0` |
| 7 | one tracking snapshot per pipeline pair for event recording | `8.96ms` | `9.19ms` | `3.84ms` | `4.31ms` | valid `1.0`, wrong/confusion `0/0` |
| 8 | formalized default `design_5marker_seed` for the five-rigid scenario | `9.76ms` | `16.32ms` | `5.87ms` | `4.77ms` | valid `1.0`, wrong/confusion `0/0` |
| 9 | 120-frame same-scenario validation | `8.35ms` | `8.59ms` | `3.20ms` | `4.13ms` | valid `1.0`, wrong/confusion `0/0` |
| 10 | docs/tests consolidation + 120-frame repeat | `8.73ms` | `10.23ms` | `3.68ms` | `4.65ms` | valid `1.0`, wrong/confusion `0/0` |
| 11 | per-timestamp prediction cache | `8.21ms` | `8.83ms` | `3.06ms` | `4.66ms` | valid `1.0`, wrong/confusion `0/0` |
| 12 | one-sort residual percentile summaries | `7.77ms` | `8.00ms` | `2.29ms` | `4.17ms` | valid `1.0`, wrong/confusion `0/0` |
| 13 | zero-distortion fast marker projection | `7.84ms` | `7.86ms` | `2.46ms` | `4.05ms` | valid `1.0`, wrong/confusion `0/0` |
| 14 | manual geo reprojection attempt | rejected | rejected | rejected | rejected | slower geometry path; reverted |
| 15 | reuse precomputed rigid-hint marker maps | `7.48ms` | `7.48ms` | `2.12ms` | `3.87ms` | valid `1.0`, wrong/confusion `0/0` |
| 16 | one-pass rigid-hint marker indexing | `7.46ms` | `7.41ms` | `2.06ms` | `3.80ms` | valid `1.0`, wrong/confusion `0/0` |
| 17 | wide-baseline rigid-hint fast refine trial | `7.43ms` | `7.52ms` | `2.06ms` | `3.83ms` | valid `1.0`, wrong/confusion `0/0` |
| 18 | fast full-CONTINUE hint quality scoring | `6.96ms` | `7.06ms` | `1.53ms` | `3.84ms` | valid `1.0`, wrong/confusion `0/0` |
| 19 | sim camera install now enables GUI geo hotpath | `5.00ms` | `4.97ms` | `1.72ms` | `1.64ms` | valid `1.0`, wrong/confusion `0/0` |
| 20 | cached wide-baseline camera pair per camera set | `4.83ms` | `4.79ms` | `1.64ms` | `1.47ms` | valid `1.0`, wrong/confusion `0/0` |
| 21 | single-candidate object-gating shortcut trial | rejected | rejected | rejected | rejected | slower than cycle 20; reverted |
| 22 | heap top-k boot candidate trial | rejected | rejected | rejected | rejected | slower startup path; reverted |
| 23 | generalized 5-marker clique boot filter trial | rejected | rejected | rejected | rejected | recursive overhead outweighed pruning; reverted |
| 24 | hot path reads latest object-gating directly instead of full diagnostics | `5.00ms` | `5.13ms` | `1.87ms` | `1.65ms` | valid `1.0`, wrong/confusion `0/0` |
| 25 | cKDTree object-gating nearest-neighbor trial | rejected | rejected | rejected | rejected | slower than dense assignment; reverted |
| 26 | reverted trials + object-gating diagnostic fast read | `4.96ms` | `4.94ms` | `1.95ms` | `1.67ms` | valid `1.0`, wrong/confusion `0/0` |
| 27 | all-camera residuals for wide-baseline hints | rejected | rejected | rejected | rejected | fixed coverage semantics but raised geometry p95 to `2.41ms`; reverted |
| 28 | reuse object-gating assigned-view counts for full-CONTINUE hint coverage | `4.82ms` | `4.93ms` | `1.79ms` | `1.79ms` | valid `1.0`, wrong/confusion `0/0` |
| 29 | object-gating coverage counters avoid copying diagnostic payloads | `4.59ms` | `4.44ms` | `1.46ms` | `1.46ms` | valid `1.0`, wrong/confusion `0/0` |
| 30 | 120-frame repeat of cycle 29 adoption candidate | `4.62ms` | `4.33ms` | `1.47ms` | `1.42ms` | valid `1.0`, wrong/confusion `0/0` |
| 31 | cProfile audit of cycle 30 hot paths | profile-only | profile-only | profile-only | profile-only | object gating, triangulation, and full diagnostics were the main remaining costs |
| 32 | lightweight per-frame tracking event status | `4.33ms` | `4.33ms` | `1.58ms` | `1.49ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 33 | cached object-gating ownership-set trial | rejected | rejected | rejected | rejected | slower (`5.11ms` pair mean); reverted |
| 34 | repeat after ownership-set revert | noisy | noisy | noisy | noisy | accuracy stayed clean; performance outlier not used for adoption |
| 35 | squared-distance object-gating assignment trial | rejected | rejected | rejected | rejected | slower and less direct than current dense assignment; reverted |
| 36 | reduced boot top-candidate count trial | rejected | rejected | rejected | rejected | did not remove startup spikes and reduced robustness margin; reverted |
| 37 | direct lightweight event-status call from pipeline | `4.46ms` | `4.45ms` | `1.69ms` | `1.68ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 38 | parallel contention repeat | not comparable | not comparable | not comparable | not comparable | accuracy stayed clean; timing contaminated by concurrent sim |
| 39 | parallel contention repeat | not comparable | not comparable | not comparable | not comparable | accuracy stayed clean; timing contaminated by concurrent sim |
| 40 | sequential repeat of accepted event-status cleanup | `4.50ms` | `4.51ms` | `1.73ms` | `1.80ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 41 | repeat of cycle 40 before new changes | `4.56ms` | `4.67ms` | `1.89ms` | `1.75ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 42 | event diagnostics read-only view trial | `4.56ms` | `4.68ms` | `1.78ms` | `1.77ms` | valid `1.0`, wrong/confusion/jump `0/0/0`; small effect |
| 43 | one-dict event status construction trial | `4.84ms` | `4.92ms` | `2.07ms` | `2.12ms` | valid `1.0`, wrong/confusion/jump `0/0/0`; timing outlier, kept only as cleanup |
| 44 | cProfile audit after event cleanup | profile-only | profile-only | profile-only | profile-only | event path was no longer material; object gating and rigid-hint triangulation remained hot |
| 45 | per-frame blob UV cache for object gating | `4.27ms` | `4.18ms` | `1.49ms` | `1.44ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 46 | per-frame rigid-hint blob lookup cache | `4.33ms` | `4.29ms` | `1.80ms` | `1.61ms` | valid `1.0`, wrong/confusion/jump `0/0/0`; p95 noisy but mean path simpler |
| 47 | small residual summary for fast rigid-hint refine | `4.27ms` | `4.14ms` | `1.72ms` | `1.51ms` | valid `1.0`, wrong/confusion/jump `0/0/0`; best pair p95 |
| 48 | repeat of accepted cache/summary path | `4.29ms` | `4.23ms` | `1.59ms` | `1.50ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |
| 49 | cProfile audit of accepted cache/summary path | profile-only | profile-only | profile-only | profile-only | object gating cumtime dropped about `0.284s -> 0.200s`; rigid-hint triangulation `0.297s -> 0.268s` |
| 50 | final sequential repeat of cycle 48 path | `4.36ms` | `4.44ms` | `1.74ms` | `1.65ms` | valid `1.0`, wrong/confusion/jump `0/0/0` |

### Red stress: `five_rigid_dance_swap_red_v1` PDCA 1-10

`five_rigid_dance_swap_red_v1` は pass を期待する green regression ではなく、別 rigid 由来の alias blob、同時 occlusion、移動、回転で ownership / marker confusion を意図的に起こす red stress として使う。今回の 10 周では、baseline の wrong ownership を大きく減らせるか、かつ GUI/live 経路として説明できる変更だけを残せるかを見た。

共通条件:

- scenario: `five_rigid_dance_swap_red_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- logs: `logs/sim/five_rigid_dance_swap_red_v1/`

| cycle | change / check | wrong | confusion | pair p95 | rigid p95 | decision |
|---|---|---:|---:|---:|---:|---|
| 0 | baseline | `770` | `30` | `21.98ms` | `16.52ms` | comparison only |
| 1 | reject small blobs under object gate (`min_blob_area=8`) | `54` | `72` | `156.89ms` | `151.13ms` | partial accept idea, perf/confusion too weak |
| 2 | missing-view gate for rigid-hint pose | `140` | `132` | `507.74ms` | `502.05ms` | reject |
| 3 | low-area contaminated prediction hold | `43` | `112` | `19.89ms` | `14.28ms` | reject; drift / jump risk |
| 4 | low-area contaminated invalid instead of hold | `62` | `92` | `89.30ms` | `83.49ms` | reject |
| 5 | pose position gate `4cm` | `62` | `92` | `86.09ms` | `80.42ms` | reject; no meaningful gain |
| 6 | prediction marker 3D gate | `62` | `92` | `106.76ms` | `100.79ms` | reject |
| 7 | suppress filtered fallback trial | `54` | `72` | `175.03ms` | `169.24ms` | reject |
| 8 | hint-only without rigid candidate separation | `2552` | `742` | `1745.00ms` | `1735.99ms` | reject |
| 9 | object gate max `10px` | `54` | `72` | `165.12ms` | `159.61ms` | reject; effectively same as cycle 1 |
| 10 | object gate max `6px` + `min_blob_area=8` | `46` | `29` | `89.06ms` | `81.89ms` | superseded by real-log review |

この時点の red stress は、alias / false blob を `area=4` として大量投入していたため、実ログよりも「面積で見分けられる偽物」に寄りすぎていた。実ログ確認後、rigid 側の `ObjectGatingConfig.min_blob_area` と GUI/live の `object_gating_min_blob_area` は採用しない判断に変更した。Pi 側に min-area filtering の経路があり、直近実ログでは本物らしい `pi-cam-02` blob に `area<8` が十分出ていたため、rigid runtime では面積を identity 判定に使わない。

採用して残すのは GUI/live の spatial object gate で、latest は `object_gating_pixel_max_px=4.5` とする。`five_rigid_dance_swap_red_v1` は real-log-like な camera 別 area 分布と、面積では見分けにくい cross-rigid alias blob に寄せ直した。さらに最新定義では、実障害に合わせて false blob を注入せず、CLI で false blob 数を指定されてもこの scenario では `0` にする。alias blob も source rigid の marker がその camera で見えている場合だけ追加する。次の本質的な PDCA 入口は面積閾値や false blob 除去ではなく、近接 rigid 同士の ownership arbitration / temporal identity scoring とする。

### Cross-rigid visible-source PDCA 1-10

共通条件:

- scenario: `five_rigid_dance_swap_red_v1`
- frames / fps: `240` / `118`
- camera rig: `generated_4cam_from_1_2_intrinsics`
- runtime profile: `gui_live`
- noise / requested false blobs: `0.25px` / `4 per camera`
- logs: `logs/sim/five_rigid_dance_swap_red_v1/20260501_pdca_crossrigid_ab/`

| cycle | change / check | wrong | confusion | pair p95 | rigid p95 | decision |
|---|---|---:|---:|---:|---:|---|
| 1 | current GUI/live, `fast_ABCDHRF`, gate `6px` | `336` | `48` | `30.08ms` | `25.44ms` | baseline |
| 2 | no rigid candidate separation, `fast_ABCDHF` | `222` | `20` | `2224.71ms` | `2221.68ms` | reject; too slow, jump `1` |
| 3 | separation without hint-only geometry, `fast_ABCDR` | `415` | `14` | `21.35ms` | `13.94ms` | reject; wrong increased |
| 4 | no hint-only / no separation, `fast_ABCDF` | `156` | `25` | `1696.36ms` | `1691.42ms` | reject; too slow, jump `1` |
| 5 | older `fast_ABCD` path | `367` | `29` | `2011.62ms` | `2000.27ms` | reject; too slow, jump `3` |
| 6 | gate `4.5px` with current path | `288` | `49` | `14.35ms` | `9.42ms` | accept candidate; no jump |
| 7 | gate `4.0px` | `282` | `29` | `19.84ms` | `14.56ms` | reject; jump `1`, waist valid drop |
| 8 | gate `4.25px` | `180` | `31` | `19.53ms` | `14.35ms` | reject; jump `1`, right-foot / waist valid drop |
| 9 | gate `4.5px` on `five_rigid_dance_hard_occlusion_v1` | `0` | `0` | `4.24ms` | `1.60ms` | green identity clean |
| 10 | gate `4.5px` on `five_rigid_dance_occlusion` | `0` | `0` | `5.06ms` | `2.05ms` | green identity clean |

結論: previous logic の AB では、candidate separation と hint-only geometry は残すほうが妥当だった。外すと wrong が一部下がって見える cycle はあるが、p95 が秒単位まで悪化し、jump / validity loss も出る。採用する変更は `object_gating_pixel_max_px=4.5` のみ。これは gate `6px` baseline から red stress の wrong ownership を `336 -> 288`、pair p95 を `30.08ms -> 14.35ms`、rigid p95 を `25.44ms -> 9.42ms` に下げ、green regression では wrong/confusion/jump を `0/0/0` に保った。

Cycle 1 から cycle 30 への変化:

| metric | cycle 1 | cycle 30 | change |
|---|---:|---:|---:|
| pair mean | `20.93ms` | `4.62ms` | `-77.9%` |
| pair p95 | `21.43ms` | `4.33ms` | `-79.8%` |
| rigid p95 | `6.93ms` | `1.47ms` | `-78.8%` |
| geometry p95 | `8.76ms` | `1.42ms` | `-83.8%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion | `0/0` | `0/0` | no regression |

Cycle 20 から cycle 30 への変化:

| metric | cycle 20 | cycle 30 | change |
|---|---:|---:|---:|
| pair mean | `4.83ms` | `4.62ms` | `-4.3%` |
| pair p95 | `4.79ms` | `4.33ms` | `-9.6%` |
| rigid p95 | `1.64ms` | `1.47ms` | `-10.5%` |
| geometry p95 | `1.47ms` | `1.42ms` | `-3.4%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion | `0/0` | `0/0` | no regression |

Cycle 30 から cycle 40 への変化:

| metric | cycle 30 | cycle 40 | change |
|---|---:|---:|---:|
| pair mean | `4.62ms` | `4.50ms` | `-2.7%` |
| pair p95 | `4.33ms` | `4.51ms` | `+4.2%` |
| rigid p95 | `1.47ms` | `1.73ms` | `+17.3%` |
| geometry p95 | `1.42ms` | `1.80ms` | `+27.1%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion/jump | `0/0/0` | `0/0/0` | no regression |

31-40 の best accepted cycle は pair timing では cycle 32。

| metric | cycle 30 | cycle 32 | change |
|---|---:|---:|---:|
| pair mean | `4.62ms` | `4.33ms` | `-6.4%` |
| pair p95 | `4.33ms` | `4.33ms` | roughly flat |
| rigid p95 | `1.47ms` | `1.58ms` | `+7.5%` |
| geometry p95 | `1.42ms` | `1.49ms` | `+4.7%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion/jump | `0/0/0` | `0/0/0` | no regression |

Cycle 40 から cycle 50 への変化:

| metric | cycle 40 | cycle 50 | change |
|---|---:|---:|---:|
| pair mean | `4.50ms` | `4.36ms` | `-3.1%` |
| pair p95 | `4.51ms` | `4.44ms` | `-1.6%` |
| rigid p95 | `1.73ms` | `1.74ms` | `+0.8%` |
| geometry p95 | `1.80ms` | `1.65ms` | `-8.6%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion/jump | `0/0/0` | `0/0/0` | no regression |

41-50 の best accepted cycle は balanced timing では cycle 45。

| metric | cycle 40 | cycle 45 | change |
|---|---:|---:|---:|
| pair mean | `4.50ms` | `4.27ms` | `-5.2%` |
| pair p95 | `4.51ms` | `4.18ms` | `-7.4%` |
| rigid p95 | `1.73ms` | `1.49ms` | `-13.5%` |
| geometry p95 | `1.80ms` | `1.44ms` | `-19.9%` |
| valid min | `1.0` | `1.0` | no loss |
| wrong/confusion/jump | `0/0/0` | `0/0/0` | no regression |

残 blocker:

- `production_go_no_go` は strict budget が startup spike も含むため false のまま残ることがある。cycle 45 は rigid p95 が `1.5ms` target 未満だが、repeat では揺れる。
- `scenario_go_no_go` は高速 dance stress の `rotation_delta_error_deg.max` が `5deg` を超えると false になる。ただし `pose_jump_count == 0` なら identity jump とは分けて読む。

### hard_v1_240 / GUI hold24 比較

`five_rigid_dance_hard_occlusion_v1` 240 frame では、baseline と cycle45 + hold 24f GUI 経路を分けて見る。

| run | valid min | head pos p95 | head rot p95 | head rot max | pair p95 | rigid p95 | wrong/confusion/jump |
|---|---:|---:|---:|---:|---:|---:|---|
| baseline | `0.792` | `0.0121m` | `72.0deg` | `143.4deg` | `4.222ms` | `1.645ms` | `0/0/0` |
| cycle45 + hold24 GUI | `0.838` | `0.0191m` | `65.5deg` | `76.0deg` | `4.873ms` | `1.717ms` | `0/0/0` |

この比較では、head の validity と rotation stability は改善した一方、pair / rigid timing はやや重くなった。GUI 採用では「軽い jump を抑える安定性」を優先した扱い。

### swap_red_v1_240

`five_rigid_dance_swap_red_v1` は pass 目的ではない。false blob ではなく、他 rigid 混在、occlusion、移動、回転で wrong/confusion を作れるか、検出できるかを見る。

| metric | value |
|---|---:|
| frames | `240` |
| pair p95 | `14.354ms` |
| rigid p95 | `9.415ms` |
| geometry p95 | `4.680ms` |
| valid min | `0.000` |
| wrong/confusion/jump | `288/49/0` |
| max pose jump | `0.096m` |
| max pose flip | `78.3deg` |

valid ratio:

| rigid | valid_frame_ratio |
|---|---:|
| chest | `0.7875` |
| head | `0.9708` |
| left_foot | `0.0000` |
| right_foot | `1.0000` |
| waist | `0.8542` |

この red stress は、green regression の採否表へ混ぜず、今後の confusion 対策の入口として扱う。
