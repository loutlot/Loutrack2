# Multi-Rigid Simulator

`tools/sim/` は Loutrack runtime とは独立した開発用シミュレーター。
実機や GUI を起動せずに、合成 2D blob を `TrackingPipeline` へ流し、multi-rigid tracking の精度、所有権、復帰、処理時間を検証するために使う。

## 目的

- `waist` + `wand` の遮蔽、復帰、吸い込みを deterministic に再現する。
- GT pose と estimator pose を比較する。
- `(timestamp, camera_id, emitted_blob_index)` の sidecar ledger で blob ownership を検証する。
- `Frame.blobs` には GT metadata を混ぜず、実 payload と同じ shape を保つ。
- 118fps の production budget を summary の go/no-go として確認する。

## 配置

```text
tools/sim/
  __main__.py
  multi_rigid.py

tests/sim/
  test_multi_rigid_mvp.py
```

本アプリの host runtime からは外しているため、`src/host/sim.py` は使わない。
実行時は repo root から `PYTHONPATH=src` を付けて `tools.sim` を起動する。

## 基本コマンド

Clean な `waist` + `wand` scenario:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_wand_static_clean \
  --frames 120 \
  --fps 118 \
  --camera-rig-source dummy \
  --out logs/sim/waist_wand_static_clean
```

遮蔽復帰 scenario:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_wand_occlusion_reacquire \
  --frames 120 \
  --fps 118 \
  --trajectory linear \
  --camera-rig-source dummy \
  --out logs/sim/waist_wand_occlusion_reacquire
```

移動回転中の 1-2 blob 部分遮蔽 scenario:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_rotate_partial_occlusion \
  --frames 120 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --rigid-stabilization-profile gui_live \
  --out logs/sim/waist_rotate_partial_occlusion_gui
```

2 camera で同じ marker が同時に欠ける条件を確認する場合:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_rotate_partial_occlusion \
  --frames 120 \
  --fps 118 \
  --camera-rig-source dummy \
  --rigid-stabilization-profile gui_live \
  --out logs/sim/waist_rotate_partial_occlusion_2cam
```

`pi-cam-01` / `pi-cam-02` の intrinsics をコピーした generated 4 camera:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_wand_static_clean \
  --frames 120 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --out logs/sim/waist_wand_generated_4cam
```

2.4m cube 上面4隅に camera を置き、cube center を aim する body-mounted 寄りの generated 4 camera:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 0 \
  --out logs/sim/five_rigid_body_occlusion_v1_cube_top
```

`five_rigid_body_occlusion_v1` など body-mounted 系 scenario では、`--camera-rig-source auto` の既定値が `cube_top_2_4m_aim_center` に解決される。`cube_top_2_4m_aim_center` は camera center を `(+/-1.2m, +/-1.2m, 2.4m)` に置き、`(0, 0, 1.2m)` を camera 正面にする。intrinsics は `generated_4cam_from_1_2_intrinsics` と同じく `pi-cam-01` / `pi-cam-02` から交互にコピーする。平行4カメラより実装着の囲い込み配置に近いが、full body mesh や実カメラ姿勢の再現ではない。

## Summary の見方

`--out` を指定すると `summary.json` と `eval.json` が出る。
主に見る項目:

- `wrong_ownership_count`: 別 rigid の blob を採用した回数。まず `0` を維持する。
- `marker_source_confusion_count`: 同じ rigid 内で marker index が混ざった回数。
- `valid_frame_ratio`: rigid ごとの valid frame 比率。
- `position_error_m` / `rotation_error_deg`: GT pose との誤差 summary。
- `position_delta_error_m` / `rotation_delta_error_deg`: 連続 valid pose の移動量 / 回転量が GT とどれだけずれたか。軽い jump の検出に使う。
- `pipeline_pair_ms` / `rigid_ms` / `geometry_ms`: 処理時間 summary。
- `performance_budget.warmup_trimmed`: 先頭 warmup samples を除いた `pipeline_pair_ms` / `rigid_ms` / `geometry_ms`。startup spike と sustained 性能を分けて読む。
- `production_go_no_go`: 118fps budget の機械判定。
- `scenario_go_no_go`: ownership、marker index、valid ratio、jump、118fps budget を合わせた scenario 判定。

`production_go_no_go` の初期条件:

```text
pipeline_pair_ms.p95 <= 6.0
pipeline_pair_ms.max <= 8.475
rigid_ms.p95 <= 1.5
```

## 現在の基準

Clean scenario は `wrong_ownership_count == 0`、`waist` / `wand` とも `valid_frame_ratio == 1.0`、かつ `production_go_no_go.passed == true` を期待する。

遮蔽復帰 scenario は PDCA 用の負荷 scenario。
`wrong_ownership_count == 0` を維持しつつ、`rigid_ms.p95` と復帰品質を改善対象にする。

`waist_rotate_partial_occlusion` は GUI 正式経路候補の regression scenario。
`gui_live` profile では 4 camera と 2 camera の両方で `wrong_ownership_count == 0`、`marker_source_confusion_count == 0`、`valid_frame_ratio >= 0.95`、`scenario_go_no_go.passed == true` を期待する。
2 camera では同時に同じ 2 blobs が欠ける区間があり、`tracking.waist.reacquire_count == 0`、`tracking.waist.max_pose_jump_m` と `tracking.waist.max_pose_flip_deg` がほぼ 0 のままなら合格。
この `gui_live` profile は GUI runtime の正式既定値として採用済み。

4 camera / 5 rigid の将来負荷をかける場合:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_dance_occlusion \
  --frames 120 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 4 \
  --out logs/sim/five_rigid_dance_occlusion
```

`five_rigid_dance_occlusion` は `head` / `waist` / `chest` / `left_foot` / `right_foot` を同時に流し、ダンス相当の移動・回転、胴体遮蔽、脚交差、部分的な同時 marker loss を入れる stress scenario。
この scenario は既定で `marker_layout: design_5marker_seed` を使う。明示指定なしでも、`docs/10_in_progress/rigid_body_design.md` の 5-marker seed と subset whitelist policy が適用される。
現時点では production go/no-go ではなく、4 camera / 5 rigid の boot 識別性、subset policy、処理時間の限界を測る PDCA 用 scenario として扱う。cycle 45 の balanced repeat は pair mean `4.27ms`、pair p95 `4.18ms`、rigid p95 `1.49ms`、geometry p95 `1.44ms`、valid `1.0`、wrong/confusion/jump `0/0/0`。cycle 50 の final repeat は pair mean `4.36ms`、pair p95 `4.44ms`、rigid p95 `1.74ms`、geometry p95 `1.65ms`、valid `1.0`、wrong/confusion/jump `0/0/0` で、精度は維持しつつ cycle 40 より pair/geometry は改善した。startup spike と high-speed rotation delta の strict gate はまだ残っている。

実装着に寄せた body-mounted stress:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 0 \
  --out logs/sim/five_rigid_body_occlusion_v1
```

`five_rigid_body_occlusion_v1` は既存 5-rigid anatomical spacing を保ちつつ、false blob ではなく人体側の遮蔽で camera 別 visibility を変える。座り気味の waist、torso yaw による胸・腰の片側遮蔽、足交差時の left / right foot visibility 変化を deterministic に入れるが、alias blob 注入や full body mesh は使わない。`--false-blobs-per-camera` を渡してもこの scenario では random false blob を追加しない。実ログに合わせて true blob area は camera 別に偏らせ、`pi-cam-02` / `pi-cam-04` 相当は等価 diameter 約 4px 前後の小さい blob を出す。body-mounted scenario では各 camera の初期 body depth 中央値を基準にし、近い marker ほど等価 diameter を緩やかに大きくする。red stress より現実ログ寄りの hard regression 入口として扱う。
この scenario は pixel gate 定数の最適化には使わない。gate tuning は合成投影と seed に自己最適化しやすいため、実ログ replay を主評価にする。

`five_rigid_body_occlusion_relaxed_v1` は同じ camera rig、body motion、camera-specific blob area を使う少し緩い兄弟 scenario。feet の遮蔽 camera 数と waist/head の遮蔽時間を減らし、hard fixture ではなく green regression / PDCA 入口として使う。

body-mounted PDCA で GUI profile の一部だけを振る場合は、profile は `gui_live` のまま、sim 専用 override を付ける。

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_body_occlusion_v1 \
  --frames 240 \
  --fps 118 \
  --rigid-stabilization-profile gui_live \
  --object-gating-pixel-max-px 4.0 \
  --position-continuity-guard false \
  --noise-px 0.25 \
  --false-blobs-per-camera 0 \
  --out logs/sim/five_rigid_body_occlusion_v1/pdca_gate40_no_pos_guard
```

使える override は `--object-gating-pixel-max-px`、`--object-conditioned-gating`、`--object-gating-enforced`、`--pose-continuity-guard`、`--position-continuity-guard`。`summary.json` には `pipeline_variant`、`subset_diagnostics_mode`、`rigid_stabilization_overrides` が残る。

body-mounted observation model の診断では、任意で `--body-mount-blob-merge-factor` を指定できる。`0.0` が既定で、近接 true blobs を merge しない。`0.25` などを指定すると、projected distance が `factor * average equivalent diameter` 以下の true blobs を area-weighted centroid の merged blob として出し、ledger の `merged_owners` に元 marker を残す。これは tracker 採用候補ではなく、近接 contour merge が valid / ownership に与える影響を見る PDCA 用 knob として扱う。

GUI live の object gating は、同一 camera 内で近接している blob assignment を曖昧として落とす。既定の `object_gating_ambiguous_blob_min_separation_px` は `0.60px`、`object_gating_ambiguous_blob_diameter_overlap_ratio` は `0.30`。これは close marker / cross-rigid 近接時に marker swap や wrong owner を triangulation 入力へ入れないための guard で、`rigid_stabilization_overrides` から PDCA 用に変更できる。

実ログ replay でも同じ guard は `--object-gating-ambiguous-blob-min-separation-px` と `--object-gating-ambiguous-blob-diameter-overlap-ratio` で変更できる。min separation に `0` を指定すると close-blob ambiguity drop を無効にできるため、採用前確認では OFF/ON 比較に使う。

より厳しい deterministic stress をかける場合:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_dance_hard_occlusion_v1 \
  --frames 360 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 4 \
  --out logs/sim/five_rigid_dance_hard_occlusion_v1
```

`five_rigid_dance_hard_occlusion_v1` は同じ 5 rigid / 5-marker layout を使い、既存 dance scenario より高速な人間スケール運動、2 camera 同時の同一 2-marker blackout、脚交差中の false blob burst、短時間の weak reacquire 区間を deterministic に入れる。
長尺 run でも遮蔽時間は frame ratio ではなく約 `2.4s` cycle の秒数 phase で繰り返すため、`240` / `360` / `600` frames を段階的な regression / hard / soak として比較しやすい。

wrong ownership / marker confusion を再現する red stress:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario five_rigid_dance_swap_red_v1 \
  --frames 240 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --rigid-stabilization-profile gui_live \
  --noise-px 0.25 \
  --false-blobs-per-camera 4 \
  --out logs/sim/five_rigid_dance_swap_red_v1
```

`five_rigid_dance_swap_red_v1` は hard scenario を壊すための赤テスト。足交差と torso 近接時に別 rigid / marker 由来として ledger に残る alias blob を、遮蔽された marker 近傍へ deterministic に注入する。
この scenario は正式回帰の合格基準ではなく、`wrong_ownership_count` や `marker_source_confusion_count` が出る条件を作り、修正 PDCA の入口にするための stress fixture として扱う。

## テスト

Simulator だけを確認:

```bash
.venv/bin/python -m pytest tests/sim tests/test_sim_closed_loop.py -q
```

関連する tracking regression も含める:

```bash
.venv/bin/python -m pytest \
  tests/sim \
  tests/test_sim_closed_loop.py \
  tests/test_rigid_clusterer.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_runtime.py \
  -q
```

## 実装メモ

- `tools/sim/multi_rigid.py` は `TrackingPipeline._on_paired_frames()` に `PairedFrames` を直接流す。
- `camera_rig_source=auto` は body-mounted 系 scenario では `cube_top_2_4m_aim_center`、それ以外では `real_2cam` に解決される。
- `camera_rig_source=real_2cam` は `calibration/` の実 calibration を使う。
- `camera_rig_source=generated_4cam_from_1_2_intrinsics` は `pi-cam-01` / `pi-cam-02` の intrinsics を `pi-cam-03` / `pi-cam-04` へコピーし、deterministic extrinsics を使う。
- `camera_rig_source=cube_top_2_4m_aim_center` は 2.4m cube 上面4隅から中心を見る deterministic extrinsics を使う。
- `camera_rig_source=dummy` は軽い unit / CI 用。
- 5-marker / 5-rigid は `current_4marker` の MVP が安定してから `future_5marker` fixture として追加する。
