# Loutrack2

Loutrack2 は、複数 Raspberry Pi カメラで反射マーカーを観測し、Host 側で外部較正とトラッキングまで行うモーションキャプチャ実験リポジトリです。

## 現在の到達点

- Charuco ベースの intrinsics（内部較正）生成
- pose capture ログから `extrinsics_pose_v2.json` を生成
- Host GUI から blob/mask/capture/extrinsics/tracking を一貫操作
- tracking runtime の起動と scene snapshot JSON の確認

## 正規入口

- Host GUI: `host.loutrack_gui` / `src/host/loutrack_gui.py`
- Session orchestration: `host.wand_session` (`CalibrationSession`, `CalibrationSessionConfig`)
- Pi runtime: `pi.service.capture_runtime` / `src/pi/service/capture_runtime.py`
- Wand target model: `calibration.targets.wand`

API/プロトコルの契約として、`/api/*`、`schema/control.json`、`schema/messages.json` は維持対象です。

## リポジトリ構成

- [`src/pi`](src/pi): Pi 側キャプチャサービス
- [`src/host`](src/host): Host 側 GUI / runtime / orchestration
- [`src/camera-calibration`](src/camera-calibration): intrinsics / extrinsics CLI
- [`src/calibration`](src/calibration): 較正ドメイン型と target 定義
- [`calibration`](calibration): 較正 JSON 出力
- [`schema`](schema): 制御/メッセージ schema
- [`tests`](tests): 回帰テスト

## セットアップ

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

前提:

- Host 1 台 + Pi カメラ 2 台以上
- Host/Pi が同一ネットワークで疎通
- Pi は `linuxptp` 運用（`pi-cam-01` を Grandmaster、他を slave）

PTP の詳細は [`src/pi/README.md`](src/pi/README.md) を参照してください。

## 最短手順（tracking 表示まで）

### 1. Pi を起動

```bash
.venv/bin/python src/pi/service/capture_runtime.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000
```

別 Pi は `--camera-id pi-cam-02` のように変えて起動します。

### 2. intrinsics を作成

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata

python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm <実測値> \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

### 3. Host GUI を起動

```bash
python src/host/loutrack_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで `http://<HOST_IP>:8765/` を開きます。

### 4. GUI で extrinsics を生成

推奨フロー:

1. `Refresh`
2. カメラ選択
3. `Ping`
4. `Blob Detection Adjustment`
5. `Mask Adjustment` (`Build Mask`)
6. `Pose Capture`（開始/停止）
7. `Floor / Metric`（数秒）
8. `Generate Extrinsics`

### 5. Tracking を起動

GUI の tracking ページで `Start Tracking` を実行し、scene snapshot の更新を確認します。

## 主要出力ファイル

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `calibration/extrinsics_pose_v2.json`
- `logs/extrinsics_pose_capture.jsonl`
- `logs/extrinsics_wand_metric.jsonl`
- `logs/loutrack_gui_settings.json`

GUI 設定は `logs/loutrack_gui_settings.json` が唯一の保存先です。旧 `logs/wand_gui_settings.json` は起動時に 1 回だけ移行・削除されます。

## よく使う CLI

外部較正を CLI で実行:

```bash
.venv/bin/python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --wand-metric-log logs/extrinsics_wand_metric.jsonl \
  --output calibration/extrinsics_pose_v2.json
```

床置き wand 収録のみ実行:

```bash
.venv/bin/python src/host/auxiliary/capture_wand_floor.py \
  --output logs/extrinsics_wand_metric.jsonl \
  --duration-s 3.0
```

## テスト

最小 smoke:

```bash
PYTHONPATH=src .venv/bin/python -m pytest tests/test_refactor_compatibility.py
```

主要回帰:

```bash
PYTHONPATH=src .venv/bin/python -m pytest \
  tests/test_extrinsics_gui_v2.py \
  tests/test_tracking_runtime.py \
  tests/test_calibration_session.py \
  tests/test_pi_blob_detector.py \
  tests/test_pi_capture_controls.py \
  tests/test_pi_control_e2e.py \
  tests/test_pi_backend_unavailable.py \
  tests/test_pi_udp_emitter.py \
  tests/test_extrinsics_pose_v2.py \
  tests/test_extrinsics_samples.py \
  tests/test_refactor_compatibility.py
```

## 詳細ドキュメント

- Host 実装: [`src/host/README.md`](src/host/README.md)
- Pi 実装: [`src/pi/README.md`](src/pi/README.md)
- Camera calibration: [`src/camera-calibration/README.md`](src/camera-calibration/README.md)
- Deploy: [`src/deploy/README.md`](src/deploy/README.md)
- 運用 runbook: [`docs`](docs)

## 更新メモ（2026-03-17）

- README をゼロベースで再構成（導線を quickstart 中心へ整理）
- 旧互換入口の説明を除去し、新正規入口だけを掲載
