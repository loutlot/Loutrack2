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
| `five_rigid_dance_swap_red_v1` | 他 rigid 混在、occlusion、移動、回転で wrong / confusion を意図的に起こす red stress | pass を期待しない。混同の検出力と悪化量を見る。 |
| waist / wand partial occlusion 系 | 小さな closed-loop 回帰 | GUI 経路の occlusion hold や jump guard の単体確認。 |

green regression では `wrong_ownership_count == 0`、`marker_source_confusion_count == 0`、`pose_jump_count == 0` を基本条件にする。red stress では wrong / confusion が出ること自体は想定内なので、`production_go_no_go.passed` ではなく、どの rigid / marker / camera 条件で崩れたかを読む。

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

採用して残すのは GUI/live の `object_gating_pixel_max_px=6` のみ。`five_rigid_dance_swap_red_v1` は real-log-like な camera 別 area 分布と、面積では見分けにくい cross-rigid alias blob に寄せ直した。さらに最新定義では、実障害に合わせて false blob を注入せず、CLI で false blob 数を指定されてもこの scenario では `0` にする。alias blob も source rigid の marker がその camera で見えている場合だけ追加する。この更新後の run は `logs/sim/five_rigid_dance_swap_red_v1/20260501_visible_source_mixing_gate6/` に残した。結果は wrong ownership `336`、marker confusion `48`、pair p95 `31.45ms`、rigid p95 `26.21ms` で、次の PDCA 入口は面積閾値や false blob 除去ではなく、近接 rigid 同士の ownership arbitration / temporal identity scoring とする。

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
| pair p95 | `31.452ms` |
| rigid p95 | `26.209ms` |
| geometry p95 | `5.018ms` |
| valid min | `0.000` |
| wrong/confusion/jump | `336/48/0` |
| max pose jump | `0.094m` |
| max pose flip | `75.0deg` |

valid ratio:

| rigid | valid_frame_ratio |
|---|---:|
| chest | `0.7208` |
| head | `0.9708` |
| left_foot | `0.0000` |
| right_foot | `1.0000` |
| waist | `0.9042` |

この red stress は、green regression の採否表へ混ぜず、今後の confusion 対策の入口として扱う。
