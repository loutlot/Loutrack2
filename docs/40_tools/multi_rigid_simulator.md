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

`pi-cam-01` / `pi-cam-02` の intrinsics をコピーした generated 4 camera:

```bash
PYTHONPATH=src .venv/bin/python -m tools.sim \
  --scenario waist_wand_static_clean \
  --frames 120 \
  --fps 118 \
  --camera-rig-source generated_4cam_from_1_2_intrinsics \
  --out logs/sim/waist_wand_generated_4cam
```

## Summary の見方

`--out` を指定すると `summary.json` と `eval.json` が出る。
主に見る項目:

- `wrong_ownership_count`: 別 rigid の blob を採用した回数。まず `0` を維持する。
- `marker_source_confusion_count`: 同じ rigid 内で marker index が混ざった回数。
- `valid_frame_ratio`: rigid ごとの valid frame 比率。
- `position_error_m` / `rotation_error_deg`: GT pose との誤差 summary。
- `pipeline_pair_ms` / `rigid_ms` / `geometry_ms`: 処理時間 summary。
- `production_go_no_go`: 118fps budget の機械判定。

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
- `camera_rig_source=real_2cam` は `calibration/` の実 calibration を使う。
- `camera_rig_source=generated_4cam_from_1_2_intrinsics` は `pi-cam-01` / `pi-cam-02` の intrinsics を `pi-cam-03` / `pi-cam-04` へコピーし、deterministic extrinsics を使う。
- `camera_rig_source=dummy` は軽い unit / CI 用。
- 5-marker / 5-rigid は `current_4marker` の MVP が安定してから `future_5marker` fixture として追加する。
