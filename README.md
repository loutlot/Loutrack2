# Loutrack2

Loutrack2 は、複数 Raspberry Pi カメラで反射マーカーを観測し、Host 側で 3D 復元とトラッキング表示まで行うモーションキャプチャ実験リポジトリです。

現時点で到達している範囲:

- Charuco を使った各カメラの内部較正
- pose capture からの reference ベース similarity extrinsics 生成
- Host GUI からの blob 調整 / mask 構築 / pose capture / Generate Extrinsics
- 外部較正後の tracking runtime 起動と JSON ベースの表示確認

この README は、初見の人が「何ができていて」「どの順で進めれば tracking 表示まで到達できるか」を把握するための入口です。

## 1. 全体像

Loutrack2 の流れは次の 4 段階です。

1. `src/pi/capture.py` を各 Raspberry Pi で起動して、反射マーカー blob を UDP 送信できる状態にする
2. `src/camera-calibration/calibrate.py` で各カメラの intrinsics を作る
3. `src/host/wand_gui.py` で pose capture を収録し、`calibration/extrinsics_pose_v2.json` を作る
4. 同じ GUI から tracking を起動して、scene snapshot JSON で復元結果を確認する

ざっくり言うと:

- Pi は「2D blob を送る係」
- Host は「較正・同期・三角測量・姿勢推定・表示を担う係」

## 2. リポジトリ構成

- [`/src/pi`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi): Raspberry Pi 側の capture サービス
- [`/src/host`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host): Host 側の受信、GUI、tracking runtime
- [`/src/camera-calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration): Charuco 内部較正と pose-capture 外部較正
- [`/calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/calibration): 較正 JSON とボード出力
- [`/docs`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs): 設計メモ、進行中 runbook、完了済み runbook
- [`/tests`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests): 回帰テスト

## 3. 前提

必要なもの:

- Host マシン 1 台
- Raspberry Pi カメラ 2 台以上
- 各 Pi で `src/pi/capture.py` が動くこと
- Host と Pi が同一ネットワークで通信できること
- 各 Pi に `linuxptp` が導入済みで、`pi-cam-01` を Grandmaster、他 Pi を client とする PTP が常時追従していること
- Charuco ボード

`linuxptp` はパッケージ導入だけでは不十分です。`ptp4l` / `phc2sys` を `pi-cam-01` 側 GM、他 Pi 側 client として常時動かし、`pmc -u -b 0 "GET TIME_STATUS_NP"` が読める状態を前提にします。

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

- PTP の初期設定は `pi-cam-01` で `sudo ./src/pi/setup_ptp.sh master`、他 Pi で `sudo ./src/pi/setup_ptp.sh slave` を使うと最短です。標準運用は `software` timestamping で、`hardware` は実験用です
- Loutrack が入れた PTP 設定を全部戻すときは `sudo ./src/pi/revert_ptp.sh` を使います
- `pi-cam-01` の wall clock が怪しいときは、session 前に `sudo ./src/pi/manual_ntp_sync.sh` で一度だけ手動 NTP 同期してから PTP に戻せます
- `Generate Extrinsics` の GUI 既定 `Pair Window (us)` は `2000` です
- 制御ポートは既定で `8554`
- UDP は既定で `5000`
- UDP frame には既存キーに加えて `timestamp_source` が入り、`timestamp` は露光寄りの時刻を優先して出力します
- `--debug-preview` は通常運用で ON にして、blob 調整や mask 構築の見え方を Pi 側画面で確認します
- `ping` diagnostics では `clock_sync` と `timestamping` を返し、PTP 健全性と timestamp source を確認できます

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
6. `Pose Capture` セクションで `Start Pose Capture`
7. 単一点ターゲットを空間全体で動かして pose log を収録
8. `Stop Pose Capture` で収録を止める
9. `Generate Extrinsics` で reference ベースの similarity extrinsics を生成

既定の重要パス:

- pose 収録ログ: `logs/extrinsics_pose_capture.jsonl`
- GUI 設定: `logs/wand_gui_settings.json`
- 外部較正出力: `calibration/extrinsics_pose_v2.json`

外部較正を CLI で直接作る場合:

```bash
python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --output calibration/extrinsics_pose_v2.json
```

いまの `Generate Extrinsics` は similarity pose だけを出します。`metric` / `world` は JSON 上では `unresolved` のまま残し、wand ベース固定は後続タスクで埋める前提です。

`pose.solve_summary` には reprojection 指標に加えて、非 anchor 観測の timestamp 差分統計
`matched_delta_us_p50` / `matched_delta_us_p90` / `matched_delta_us_max` も保存されます。

## 5. tracking を起動して表示確認

外部較正が生成済みなら、GUI の tracking ページで `Start Tracking` を押します。

ここで確認できるもの:

- tracking status JSON
- scene snapshot JSON
- `frames_processed`
- `poses_estimated`
- カメラ情報と raw points
- 直感的に監視できる tracking confirmation ページ

Tracking タブには組み込みの `three.js` ビューアを使った 3D canvas があり、
カメラの frustum、rigid body（軸・マーカー・トレイル）と raw points をリアルタイムで描画します。右側には per-camera health card（FPS / latency / blob 平均）と rigids サマリーを並べ、`Start`/`Stop` ボタン越しにトラッキング品質を GUI 上だけで判断できます。  
このビューアの静的アセットは repo 直下の `static/vendor/three.module.min.js` を HTTP 配信して読み込みます。

今の実装は「tracking runtime が立ち上がり、scene snapshot を返せる」段階です。初見向けにはまずこの JSON が更新されるところまで到達できれば十分です。

## 6. いま理解しておくべきファイル

最初に読む対象を絞るならこの 5 つで十分です。

- [`/src/pi/capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/capture.py): Pi 側の 2D blob 検出と送信
- [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py): 現在の運用入口
- [`/src/host/tracking_runtime.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/tracking_runtime.py): tracking の起動と scene snapshot
- [`/src/camera-calibration/calibrate.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate.py): intrinsics 作成
- [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py): pose capture ログから reference ベース extrinsics 作成

## 7. 生成される成果物

実行すると主に次が増えます。

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `calibration/extrinsics_pose_v2.json`
- `logs/extrinsics_pose_capture.jsonl`
- `logs/wand_gui_settings.json`

README を読む側は、まずこの 4 つが何者かを把握すると迷いにくいです。

## 8. 詰まりやすい点

- `Refresh` してもカメラが出ない
  - Pi 側の `capture.py` が起動しているか
  - `--udp-dest` やネットワーク経路が合っているか
  - GUI は `Refresh` 実行時に「現在のチェックなし (`camera_ids=[]`)」を受け取るとカメラ絞り込みを解除し、全発見カメラを再表示する（古い選択IDで空一覧に固定されない）
- `Ping` が通らない
  - Host から Pi の `8554/tcp` に届いているか
  - `camera_id` と実機の対応が崩れていないか
  - `src/deploy/hosts.ini` の IP と `camera_id` が、Pi 側 `capture.py --camera-id ...` の実起動値と一致しているか
  - `ping` の `clock_sync.status` が `unknown` / `degraded` のまま張り付いていないか
  - `ping` が通っても GUI に反映されない場合は、まず `Refresh` を押してから対象カメラのチェック状態を確認する
  - `Refresh` / `Ping` を押したとき、GUI サーバーのターミナルに `[wand_gui] POST /api/command ...` が出るかを確認する（出ない場合はブラウザ側の実行エラーを疑う）
- `sudo systemctl stop systemd-timesyncd` や `disable` が必要になるケースがある
- ボタンを押しても無反応に見える
  - `Last Result` に `load_state: ...` / `command_ping: ...` 形式のエラーが出ていないか確認する
- `set_exposure` / `set_focus` を送っても Pi preview が変わらない
  - Pi 側 `picamera2` backend は manual exposure 時に `AeEnable=false`、manual focus 時に `AfMode=Manual` が必要
  - このリポジトリではその manual 制御を有効化済みなので、古いプロセスが残っている場合は Pi 側 `capture.py` を再起動する
- extrinsics が生成できない
  - `calibration/` に対象カメラ分の intrinsics があるか
- `logs/extrinsics_pose_capture.jsonl` に複数カメラの単点観測が入っているか
- tracking が始まらない
  - `calibration/extrinsics_pose_v2.json` ができているか

### 直近の実装アップデート（2026-03）

- `src/pi/capture.py` の frame timestamp を露光寄りに寄せ、`sensor_metadata` がある場合はそれを優先、無い場合だけ `capture_dequeue` fallback にするよう更新
- UDP frame schema は互換を維持したまま `timestamp_source` を追加し、`ping` diagnostics には `clock_sync` / `timestamping` を追加
- `pi-cam-01` を固定 Grandmaster とする PTP 前提を導入し、Pi 側 OS prerequisite に `linuxptp` を追加
- `pose.solve_summary` に `matched_delta_us_p50` / `matched_delta_us_p90` / `matched_delta_us_max` を追加し、reprojection と timing quality を並べて確認できるよう更新
- `Generate Extrinsics` を clean-slate でリライトし、`references/jyjblrd` ベースの `pose_capture -> camera row 化 -> 隣接 pair F/E 初期化 -> BA` へ置換
- 正式出力を `calibration/extrinsics_pose_v2.json` へ変更し、top-level を `pose / metric / world` に分離
- `src/camera-calibration/calibrate_extrinsics.py` / `tests/...` など多数更新（省略）

## 9. 詳細ドキュメント

- Host モジュール概要: [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md)
- 内部較正: [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md)
- GUI operator runbook: [`/docs/10_in_progress/gui_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/gui_runbook.md)
- wand 4点 / referenceベース extrinsics 再設計計画: [`/docs/10_in_progress/wand_4point_extrinsics_plan.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/wand_4point_extrinsics_plan.md)
- jyjblrd ベース extrinsics migration 計画: [`/docs/10_in_progress/jyjblrd_extrinsics_migration_plan.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/jyjblrd_extrinsics_migration_plan.md)
- wand 運用 runbook: [`/docs/20_completed/24_next_steps_wand_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/20_completed/24_next_steps_wand_runbook.md)

## 10. 現状の位置づけ

このリポジトリは「calibration から tracking 表示までの実働フロー」が通った段階です。いまの README は、実装の全詳細を説明するよりも、初見の人が次の 2 点を短時間で理解できるようにすることを目的にしています。

- 何をどの順で起動すればよいか
- どのファイルが入力で、どの JSON が出力なのか
