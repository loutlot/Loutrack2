# Loutrack2

Loutrack2 は、複数 Raspberry Pi カメラで反射マーカーを観測し、Host 側で外部較正とトラッキングまで行うモーションキャプチャ実験リポジトリです。

## 現在の到達点

- Charuco ベースの intrinsics（内部較正）生成
- pose capture ログから `extrinsics_pose_v2.json` を生成
- Host GUI から blob/mask/capture/extrinsics/tracking を一貫操作
- tracking runtime の起動と scene snapshot JSON の確認

## 直近の更新

- Pi runtime / control server の起動条件を修正し、MJPEG / preview render が OFF でも camera loop・blob diagnostics・timestamp/runtime status が止まらないよう改善
- `tests/test_pi_blob_detector.py` に、preview render OFF でも pipeline runtime が継続する回帰テストを追加
- Pi→Host の `intrinsics_get_corners` を chunk 取得対応に変更し、フレーム数が増えて NDJSON 64KB 上限に達しても `remote reports N` のまま Host 側 `got 0` にならないよう修正
- Intrinsics の `Discard Session` は `Stop` / `Clear Frames` と責務が重複していたため、混乱防止のため UI と API から削除
- Intrinsics `Clear Frames` が効いていないように見える問題を修正し、clear 後の `frames/rejected/grid` を GUI へ即反映するよう改善
- Intrinsics `Calibrate` 実行前に Pi 側の buffered corners を即時同期するよう修正し、GUI 表示では `Captured` が十分あるのに Host 側だけ `got 0` になる不整合を解消
- Intrinsics phase badge を状態別カラーにし、`capturing` はオレンジ系で見やすく表示するよう改善
- Intrinsics `Stop` API が `status` を返すようにし、GUI 側でも停止直後に `idle` へ即反映するよう修正
- Pi intrinsics の raw frame 処理例外を `last_error` へ反映するようにし、`Frame Collection=0` の原因が GUI status から追いやすいよう改善
- Intrinsics `Start Capture` 直後に GUI が無反応に見える問題に対応し、開始時に status poll を即時実行して `phase / rejected / grid` をすぐ反映するよう改善
- Host intrinsics poller で Pi 側 `phase` と `last_error` を同期し、`intrinsics_get_corners` 失敗時も `last_error` に反映されるよう改善
- Pi 側 intrinsics の `calibrate.py` 動的ロードで `sys.modules` 登録が抜けており、`Start Capture` 時に `intrinsics_start_failed: 'NoneType' object has no attribute '__dict__'` が出る不具合を修正
- 上記の回帰として、Pi intrinsics loader が dataclass を含む較正モジュールを正常ロードできるテストを追加
- Intrinsics `Start Capture` が `mjpeg_url` 未入力で `BAD_REQUEST` になる不具合を修正し、MJPEG 未設定でも Pi 側 `intrinsics_start` が実行されるよう改善
- `/api/intrinsics/start` 失敗時のデバッグログに `error type` と `repr` を追加し、`error=` が空でも原因切り分けできるよう改善
- `calibrate_extrinsics.py` の JSON 出力で、`pose_capture_log_path` / `wand_metric_log_path` に絶対パス（ローカルディレクトリ）が混入しないように修正
- 絶対パス入力時でも `extrinsics_pose_v2.json` 側は非絶対パスになることを `tests/test_extrinsics_pose_v2.py` で回帰テスト化
- GUI 初期ロードで Intrinsics の Camera Controls（focus/exposure/gain/fps）スライダーが保存済み `loutrack_gui_settings.json` と一致するよう復元処理を追加
- `loutrack_gui_settings.json` / 旧 `wand_gui_settings.json` のデフォルト参照を起動ディレクトリ依存からリポジトリルート基準へ修正し、起動場所が異なっても `focus` などの保存値が 5.215 に戻らないよう改善
- `loutrack_gui_settings.json` が部分的新形式（`ui` / `runtime_hints` 欠落など）でも legacy 扱いで `focus=5.215` / `square_length_mm=30` に戻らないよう、settings 正規化を後方互換対応に修正
- `logs/loutrack_gui_settings.json` を現行の完全スキーマ（`meta/calibration/intrinsics/extrinsics/ui/runtime_hints`）へ正規化し、既存値を保持したまま不足キーを補完
- GUI 設定保存を `draft/committed/ui` 方式へ再設計し、`/api/settings` / `POST /api/settings/draft` / `POST /api/settings/ui` を追加
- `loadState` は運用表示専用にし、フォーム値の3秒ポーリング上書きを廃止。全フォーム入力は `draft` として即時保存し、リロード時に復元
- 既存の入力上書き保護ロジックを削除し、Intrinsics/Extrinsics の実行系は `committed` を使用。無効入力は `draft` に残しつつ開始時に明示エラー
- ログパスは `manual wins until reset` を導入。手動入力でロックし、`Reset to latest` で runtime 最新値へ同期
- 数値/テキスト系フォームは `input`（短デバウンス）に加えて `change` でも即保存し、`Square Length` などの入力確定時に `draft` へ確実に反映
- ビュー遷移時の `set_preview` を用途別に分離し、`intrinsics` では `blob/mask` オーバーレイを無効化して `charuco` のみ表示、他ビューでは `blob` 表示・`charuco` 非表示に切替
- `intrinsics` を preview ON 対象へ追加し、タブ遷移した瞬間に `charuco only` 設定が必ず反映されるよう修正（入力イベント待ち不要）
- `selected_camera_ids` は GUI/サーバ双方で順序維持の重複排除を追加し、設定JSONに同一 camera id が蓄積しないよう修正
- Host `set_preview` 中継で `overlays/charuco` が落ちる不具合を修正し、GUI指定の `blob/charuco` 表示切替と Charuco パラメータが Pi 側へ正しく伝播することをテストで固定
- 全スライダー（`range` + `wandMetricSeconds`）を共通 state で一元管理するよう統合し、`focus/intFocus` など重複UIでも同一値・同一送信/保存経路（`/api/config` + `/api/settings/draft`）で動作するよう修正
- Intrinsics を Pi 主導へ移行し、`intrinsics_start|stop|clear|calibrate|status` を `capture_runtime.py` に追加。Host `/api/intrinsics/*` は互換パスのまま Pi コマンド中継へ切替
- Intrinsics 検出は Pi の raw frame 入力へ一本化し、Host 側 MJPEG 再検出経路を廃止。較正結果は Host `calibration/calibration_intrinsics_v1_<camera_id>.json` へ自動保存
- overlay 再検出防止の不変条件（`detect` は raw frame、overlay は表示専用）を処理経路コメントと `tests/test_pi_blob_detector.py` で固定し、Charuco overlay 描画閾値を 6 corners に統一

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
- `docs/10_in_progress/gui_runbook.md` を現行運用（`loutrack_gui` / `capture_runtime`）に合わせてゼロベース更新
