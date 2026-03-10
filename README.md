# Loutrack2

Loutrack2 は、複数 Raspberry Pi カメラで反射マーカーを観測し、Host 側で 3D 復元とトラッキング表示まで行うモーションキャプチャ実験リポジトリです。

現時点で到達している範囲:

- Charuco を使った各カメラの内部較正
- wand 収録ログからの外部較正生成
- Host GUI からの blob 調整 / mask 構築 / wand capture
- 外部較正後の tracking runtime 起動と JSON ベースの表示確認

この README は、初見の人が「何ができていて」「どの順で進めれば tracking 表示まで到達できるか」を把握するための入口です。

## 1. 全体像

Loutrack2 の流れは次の 4 段階です。

1. `src/pi/capture.py` を各 Raspberry Pi で起動して、反射マーカー blob を UDP 送信できる状態にする
2. `src/camera-calibration/calibrate.py` で各カメラの intrinsics を作る
3. `src/host/wand_gui.py` で wand を使って収録し、`calibration/calibration_extrinsics_v1.json` を作る
4. 同じ GUI から tracking を起動して、scene snapshot JSON で復元結果を確認する

ざっくり言うと:

- Pi は「2D blob を送る係」
- Host は「較正・同期・三角測量・姿勢推定・表示を担う係」

## 2. リポジトリ構成

- [`/src/pi`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi): Raspberry Pi 側の capture サービス
- [`/src/host`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host): Host 側の受信、GUI、tracking runtime
- [`/src/camera-calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration): Charuco 内部較正と wand 外部較正
- [`/calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/calibration): 較正 JSON とボード出力
- [`/docs`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs): 設計メモ、進行中 runbook、完了済み runbook
- [`/tests`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests): 回帰テスト

## 3. 前提

必要なもの:

- Host マシン 1 台
- Raspberry Pi カメラ 2 台以上
- 各 Pi で `src/pi/capture.py` が動くこと
- Host と Pi が同一ネットワークで通信できること
- Charuco ボード
- 4 点 wand（長辺の中点マーカーを含む）

Python 依存はルートの `requirements.txt` で入ります。

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

## 4. 最短で tracking 表示まで行く手順

### Step 1. 各 Pi で capture を起動

Pi ごとに `camera_id` を分けて起動します。

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview
```

別の Pi では `pi-cam-02` のように変えます。

ポイント:

- 制御ポートは既定で `8554`
- UDP は既定で `5000`
- `--debug-preview` は通常運用で ON にして、blob 調整や mask 構築の見え方を Pi 側画面で確認します

Pi 側の詳細は [`/src/pi/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/README.md) を参照してください。

### Step 2. 各カメラの内部較正を作る

まず Charuco ボードを準備し、各カメラごとに intrinsics を作ります。

ボード生成:

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata
```

内部較正:

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm <実測値> \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

最低限そろえるファイル:

- `calibration/calibration_intrinsics_v1_pi-cam-01.json`
- `calibration/calibration_intrinsics_v1_pi-cam-02.json`

内部較正の詳細は [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md) を参照してください。

### Step 3. Host GUI を起動

Host で次を実行します。

```bash
python src/host/wand_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで以下を開きます。

- `http://<HOST_IP>:8765/`
- `0.0.0.0` は Host 側の待受アドレスです。Pi 側 `--udp-dest` には `0.0.0.0:5000` ではなく、必ず Host の実 IP (`<HOST_IP>:5000`) を指定します。

GUI は calibration ページと tracking ページを持っています。

### Step 4. GUI で外部較正を作る

GUI 上では、基本的に次の順で進めます。

1. `Refresh` でカメラ一覧を更新
2. 対象カメラを選択
3. `Ping` で `ack=true` を確認
4. `Blob Detection Adjustment` で threshold / circularity / blob diameter / exposure などを調整
5. `Mask Adjustment` で `Build Mask`
6. `Wand Capture` で `Start Wand Capture`
7. 空間全体で wand を振ってログ収録
8. `Extrinsics Generation` で外部較正を生成

既定の重要パス:

- wand 収録ログ: `logs/wand_capture.jsonl`
- GUI 設定: `logs/wand_gui_settings.json`
- 外部較正出力: `calibration/calibration_extrinsics_v1.json`

外部較正を CLI で直接作る場合:

```bash
python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --log logs/wand_capture.jsonl \
  --output calibration/calibration_extrinsics_v1.json
```

### Step 5. tracking を起動して表示確認

外部較正が生成済みなら、GUI の tracking ページで `Start Tracking` を押します。

ここで確認できるもの:

- tracking status JSON
- scene snapshot JSON
- `frames_processed`
- `poses_estimated`
- カメラ情報と raw points
- 直感的に監視できる tracking confirmation ページ

Tracking タブには組み込みの `three.js` ビューアを使った 3D canvas があり、
カメラの frustum、rigid body（軸・マーカー・トレイル）と raw points をリアルタイムで描画します。
右側には per-camera health card（FPS / latency / blob 平均）と rigids サマリーを並べ、
`Start`/`Stop` ボタン越しにトラッキング品質を GUI 上だけで判断できます。
このビューアの静的アセットは repo 直下の `static/vendor/three.module.min.js` を HTTP 配信して読み込みます。

今の実装は「tracking runtime が立ち上がり、scene snapshot を返せる」段階です。初見向けにはまずこの JSON が更新されるところまで到達できれば十分です。

## 5. いま理解しておくべきファイル

最初に読む対象を絞るならこの 5 つで十分です。

- [`/src/pi/capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/capture.py): Pi 側の 2D blob 検出と送信
- [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py): 現在の運用入口
- [`/src/host/tracking_runtime.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/tracking_runtime.py): tracking の起動と scene snapshot
- [`/src/camera-calibration/calibrate.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate.py): intrinsics 作成
- [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py): wand ログから extrinsics 作成

## 6. 生成される成果物

実行すると主に次が増えます。

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `calibration/calibration_extrinsics_v1.json`
- `logs/wand_capture.jsonl`
- `logs/wand_gui_settings.json`

README を読む側は、まずこの 4 つが何者かを把握すると迷いにくいです。

## 7. 詰まりやすい点

- `Refresh` してもカメラが出ない
  - Pi 側の `capture.py` が起動しているか
  - `--udp-dest` やネットワーク経路が合っているか
  - GUI は `Refresh` 実行時に「現在のチェックなし (`camera_ids=[]`)」を受け取るとカメラ絞り込みを解除し、全発見カメラを再表示する（古い選択IDで空一覧に固定されない）
- `Ping` が通らない
  - Host から Pi の `8554/tcp` に届いているか
  - `camera_id` と実機の対応が崩れていないか
  - `src/deploy/hosts.ini` の IP と `camera_id` が、Pi 側 `capture.py --camera-id ...` の実起動値と一致しているか
  - `ping` が通っても GUI に反映されない場合は、まず `Refresh` を押してから対象カメラのチェック状態を確認する
  - `Refresh` / `Ping` を押したとき、GUI サーバーのターミナルに `[wand_gui] POST /api/command ...` が出るかを確認する（出ない場合はブラウザ側の実行エラーを疑う）
  - Host からの疎通確認例:
    - `ping 192.168.8.223`
    - `nc -vz 192.168.8.223 8554`
- ボタンを押しても無反応に見える
  - `Last Result` に `load_state: ...` / `command_ping: ...` 形式のエラーが出ていないか確認する
  - WebGL 非対応環境では tracking 3D viewer を自動で無効化し、calibration 操作は継続できる（`3D viewer unavailable...` 表示）
  - `three.module.min.js` の import に失敗しても、GUI は動作継続する（tracking viewerのみ無効化される）
- `set_exposure` / `set_focus` を送っても Pi preview が変わらない
  - Pi 側 `picamera2` backend は manual exposure 時に `AeEnable=false`、manual focus 時に `AfMode=Manual` が必要
  - このリポジトリではその manual 制御を有効化済みなので、古いプロセスが残っている場合は Pi 側 `capture.py` を再起動する
- extrinsics が生成できない
  - `calibration/` に対象カメラ分の intrinsics があるか
  - `logs/wand_capture.jsonl` に複数カメラの wand 観測が入っているか
- tracking が始まらない
  - `calibration/calibration_extrinsics_v1.json` ができているか

### 直近の実装アップデート（2026-03）

- `src/camera-calibration/calibrate_extrinsics.py` の wand 対応を強化し、4 点 wand（`[elbow, short, mid, long]`）の対応順抽出を追加
- ペアリング既定窓を `8000us` に変更し、`session_meta.pair_window_us` で既定値/上書き値を記録
- extrinsics 品質指標の `point_count` を固定値ではなく、実際に使用した対応点数で集計
- 4 点 wand 運用時は 3 点フレームを自動除外するガードを追加（3 点混入による index error を防止）
- `WAND_POINTS_MM` の順序を solver の canonical order（`elbow, short, mid, long`）へ統一
- `docs/10_in_progress/wand_4point_extrinsics_plan.md` を更新し、`references/jyjblrd` を踏まえた multiview BA ベースの extrinsics 再設計計画を追加
- `src/camera-calibration/calibrate_extrinsics.py` を multiview 方式へ更新し、ラベル付け・sample化・初期姿勢・joint BA を統合
- 新規 `src/camera-calibration/wand_label.py`（4点ラベリング）, `wand_samples.py`（multiview sample 構築）, `wand_bundle_adjustment.py`（joint BA）を追加
- `tests/test_wand_extrinsics.py` / `tests/test_wand_gui.py` の回帰を通る状態で、GUI の既存 API 契約（`pair_window_us`, `min_pairs`）は維持
- `min_pairs` を camera ごとの有効 sample 数にも適用し、不足 camera は `session_meta.excluded_camera_*` へ理由つきで除外して処理継続するよう修正
- joint BA で camera 並進も最適化対象へ戻しつつ、初期値からの変化量に境界を入れてスケール崩壊を防止
- `docs/10_in_progress/gui_runbook.md` を更新し、`Min Pairs` の適用範囲、除外 camera の確認手順、`session_meta` の運用確認項目を追加
- `Generate Extrinsics` の長時間停止を避けるため、BA 入力 sample を既定 `320` に均等間引きする `max_ba_samples` を solver に追加
- `wand_gui` で `intrinsics/log/output` の相対パスをプロジェクトルート基準の絶対パスへ正規化し、結果ファイルの出力先ずれを防止

## 8. 詳細ドキュメント

- Host モジュール概要: [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md)
- 内部較正: [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md)
- GUI operator runbook: [`/docs/10_in_progress/gui_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/gui_runbook.md)
- wand 4点 / referenceベース extrinsics 再設計計画: [`/docs/10_in_progress/wand_4point_extrinsics_plan.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/wand_4point_extrinsics_plan.md)
- wand 運用 runbook: [`/docs/20_completed/24_next_steps_wand_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/20_completed/24_next_steps_wand_runbook.md)

## 9. 現状の位置づけ

このリポジトリは「calibration から tracking 表示までの実働フロー」が通った段階です。いまの README は、実装の全詳細を説明するよりも、初見の人が次の 2 点を短時間で理解できるようにすることを目的にしています。

- 何をどの順で起動すればよいか
- どのファイルが入力で、どの JSON が出力なのか
