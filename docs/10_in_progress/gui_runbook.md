# GUI Operator Runbook (Current)

この runbook は、2026-03-17 時点の現行実装に合わせた運用手順です。  
対象は `src/host/loutrack_gui.py` と `src/pi/service/capture_runtime.py` です。

## 0. 目的と完了条件

目的:

- intrinsics/extrinsics を揃えて tracking を開始できる状態にする
- 現場で「どこで失敗したか」を切り分けて復旧できるようにする

完了条件:

- `calibration/calibration_intrinsics_v1_<camera_id>.json`（全カメラ分）
- `calibration/extrinsics_pose_v2.json`
- GUI の Tracking タブで `frames_processed` が増加

## 1. 全体フロー

1. Pi 側の PTP と capture runtime を起動する
2. Host で Loutrack GUI を起動する
3. Intrinsics（必要な場合のみ）を作成する
4. Calibration タブで blob/mask/pose/floor metric を収録する
5. Generate Extrinsics を実行する
6. Tracking タブで動作確認する

## 2. Pi 側運用

### 2.1 初回のみ: PTP 設定

`pi-cam-01` を master、他カメラを slave にします。

```bash
# pi-cam-01
sudo ./src/pi/setup_ptp.sh master

# pi-cam-02 以降
sudo ./src/pi/setup_ptp.sh slave
```

確認:

```bash
pmc -u -b 0 "GET TIME_STATUS_NP"
```

### 2.2 capture runtime 起動

各 Pi で camera_id を変えて起動します。

```bash
.venv/bin/python src/pi/service/capture_runtime.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --mjpeg-port 8555 \
  --debug-preview
```

ポイント:

- Control は TCP `8554`
- UDP 送信先は Host の実 IP（`0.0.0.0` は不可）
- `--mjpeg-port 8555` は Intrinsics タブ運用で推奨

## 3. Host GUI 起動

```bash
python src/host/loutrack_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで `http://<HOST_IP>:8765/` を開きます。

## 4. Intrinsics（必要時のみ）

既に intrinsics が全カメラ分ある場合はスキップ可能です。

### 4.1 Charuco ボード準備

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata
```

### 4.2 GUI Intrinsics タブで収録

各カメラごとに次を実施:

1. Camera ID と MJPEG URL を設定
2. Start Capture
3. ボードを角度・位置を変えて撮影
4. Min Frames 到達後に Calibrate
5. `calibration/calibration_intrinsics_v1_<camera_id>.json` 出力確認

## 5. Extrinsics（Calibration タブ）

### 5.1 Camera 状態確認

1. `Refresh`
2. 対象カメラを選択
3. `Ping`

確認観点:

- `ack=true`
- `READY` へ遷移できる
- `clock_sync` が劣化し続けていない

### 5.2 Blob Detection Adjustment

単一点マーカーを見せながら、以下を調整:

- threshold
- circularity
- blob min/max diameter
- exposure/gain/fps/focus

### 5.3 Mask Adjustment

背景のみの状態で `Build Mask`。  
完了後に `READY` へ戻ることを確認。

### 5.4 Pose Capture

1. `Start Pose Capture`
2. 単一点マーカーを空間全体でゆっくり移動
3. `Stop Pose Capture`

出力:

- `logs/extrinsics_pose_capture.jsonl`

### 5.5 Floor / Metric

1. wand を床に静置
2. `Capture Floor / Metric`

出力:

- `logs/extrinsics_wand_metric.jsonl`

### 5.6 Generate Extrinsics

`Generate Extrinsics` を実行し、以下を確認:

- `calibration/extrinsics_pose_v2.json` 生成
- `pose.solve_summary` が埋まる
- wand metric ありなら `metric/world` が `resolved`

## 6. Tracking 確認

Tracking タブで `Start Tracking` を実行して確認:

- `frames_processed` が増える
- `poses_estimated` が増える
- 3D ビューが更新される

## 7. よくある失敗と復旧

### カメラが出ない

- Pi 側 process が起動しているか
- `--udp-dest` が Host 実 IP か
- Host から Pi:8554 へ疎通できるか
- `src/deploy/hosts.ini` の camera_id/IP が一致しているか

### Ping は通るが capture が始まらない

- `Build Mask` 後に `READY` へ戻っているか
- 背景ノイズで mask が破綻していないか

### Extrinsics が失敗する

- intrinsics が全カメラ分あるか
- pose log に複数カメラ同時観測が十分あるか
- camera_id 表記揺れがないか

### Tracking が不安定

- 1 台だけ latency が悪化していないか
- blob 誤検出が増えていないか（mask 再作成）
- カメラ固定後に再度動いていないか（extrinsics 再生成）

## 8. セッション後に残すファイル

- `logs/extrinsics_pose_capture.jsonl`
- `logs/extrinsics_wand_metric.jsonl`
- `logs/loutrack_gui_settings.json`
- `calibration/extrinsics_pose_v2.json`
- `calibration/calibration_intrinsics_v1_<camera_id>.json`

## 9. 参照

- [`README.md`](../../README.md)
- [`src/pi/README.md`](../../src/pi/README.md)
- [`src/host/README.md`](../../src/host/README.md)
- [`src/camera-calibration/README.md`](../../src/camera-calibration/README.md)
