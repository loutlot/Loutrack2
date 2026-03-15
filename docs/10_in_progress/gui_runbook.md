# GUI Operator Runbook

intrinsics 較正から extrinsics 較正完了までの手順書です。
現場で判断と復旧ができるレベルの情報に絞ります。

---

## 全体フロー概要

```
[Pi 側]
 1. PTP セットアップ (初回のみ)
 2. capture.py 起動 (カメラごとに)

[Host GUI]
 3. wand_gui.py 起動
 4. Intrinsics タブ → 各カメラの内部較正 (初回 or カメラ変更時)
 5. Calibration タブ → blob 調整 → mask → pose capture → floor/metric → Generate Extrinsics
 6. Tracking タブ → 動作確認
```

---

## 1. Pi 側セットアップ

### 1.1 PTP 初期設定 (初回のみ)

各 Pi で PTP 常駐が必要です。`pi-cam-01` を Grandmaster、他を slave として設定します。

```bash
# pi-cam-01 で実行
sudo ./src/pi/setup_ptp.sh master

# pi-cam-02, pi-cam-03 ... で実行
sudo ./src/pi/setup_ptp.sh slave
```

標準は `software` timestamping です。`hardware` は実験用です。

設定をリセットしたい場合:

```bash
sudo ./src/pi/revert_ptp.sh
```

PTP が追従しているかの確認:

```bash
pmc -u -b 0 "GET TIME_STATUS_NP"
```

slave 側で `master_offset` が読め、`gmPresent true` になれば OK です。
`unknown` / `degraded` が継続する場合は `capture.py` を起動する前に解消します。

### 1.2 Python 環境のセットアップ (初回のみ)

各 Pi で:

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

### 1.3 capture.py の起動

各 Pi で、それぞれの `camera_id` を変えて起動します。

**intrinsics 較正も行う場合（推奨）:**

```bash
# pi-cam-01
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview \
  --mjpeg-port 8555

# pi-cam-02
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-02 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview \
  --mjpeg-port 8555
```

`--mjpeg-port 8555` を付けると、GUI の Intrinsics タブから MJPEG 経由で Charuco 較正ができます。
`--debug-preview` は Pi 側画面で blob 描画を確認するために付けます（headless では自動無効）。

**extrinsics 較正のみ行う場合（intrinsics が既にある場合）:**

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview
```

### 1.4 起動後の確認

Pi 側 preview（または Pi 端末ログ）で次を確認します:

- 画像が出ている（headless の場合はログに `READY` が出る）
- blob 円が描画されている
- 露出が極端に破綻していない

---

## 2. Host GUI 起動

Host マシンで:

```bash
python src/host/wand_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで開きます:

```
http://<HOST_IP>:8765/
```

GUI のタブは 3 つあります:

| タブ | 用途 |
|---|---|
| **Intrinsics** | Charuco ボードによる内部較正 |
| **Calibration** | pose capture と extrinsics 生成 |
| **Tracking** | live tracking 確認 |

---

## 3. Intrinsics 較正 (GUI: Intrinsics タブ)

各カメラごとに繰り返します。既に `calibration/calibration_intrinsics_v1_<camera_id>.json` がある場合はスキップできます。

### 3.1 ボードの準備

Charuco ボードを印刷します（初回のみ）:

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata
```

印刷後、スクエアの実寸を mm で計測しておきます。

### 3.2 Intrinsics タブの設定

GUI の **Intrinsics** タブを開きます。

**Step 01 / Camera & Board Setup** に入力します:

| フィールド | 内容 |
|---|---|
| Camera ID | `pi-cam-01`（較正対象カメラ） |
| MJPEG URL | `http://<PI_IP>:8555/mjpeg` |
| Square Length (mm) | Charuco スクエアの実測値（例: `30`） |
| Marker Length (mm) | 空欄で auto（= Square × 0.75） |
| Squares X / Y | ボードの仕様（既定: `6` / `8`） |
| Min Frames | `25`（既定のまま） |
| Cooldown (s) | `1.5`（既定のまま） |

### 3.3 キャプチャ

1. **Start Capture** を押す
2. Charuco ボードをカメラ正面で様々な角度・位置に動かす
   - 画面の隅、中央、傾けた状態を均等に見せる
   - 右上の「3×3 Coverage Grid」が全セル埋まるよう動かす
3. `Captured: N / 25` が 25 以上になるまで続ける
4. **Stop** を押す（25 未満でも Stop できるが、精度が下がる）

Charuco Preview で角が検出されているフレームが表示されます。
`Rejected (no detect)` が多い場合は照明を改善するか、ボードの汚れ・しわを確認します。

### 3.4 較正実行

1. `Captured` が `Min Frames` 以上になったら **Calibrate** を押す
2. バックグラウンドで較正が走り、`Phase` が `calibrating` → `done` へ変わる
3. Calibration Output に RMS 誤差と出力パスが表示されれば完了
4. `calibration/calibration_intrinsics_v1_pi-cam-01.json` が生成される

**目安:** RMS < 1.0 px なら良好。> 2.0 px の場合は **Clear Frames** してやり直します。

### 3.5 他カメラへ繰り返し

Camera ID と MJPEG URL を `pi-cam-02` のものに変えて同じ手順を繰り返します。

全カメラの intrinsics が揃ったら、`calibration/` に次のファイルがあることを確認します:

```
calibration/calibration_intrinsics_v1_pi-cam-01.json
calibration/calibration_intrinsics_v1_pi-cam-02.json
...
```

---

## 4. Extrinsics 較正 (GUI: Calibration タブ)

### 4.1 カメラ確認

**Calibration** タブを開きます。

1. **Refresh** を押してカメラ一覧を更新する
2. 対象カメラが一覧に出ることを確認する
3. 対象カメラをチェックして選択する
4. **Ping** を押して `ack=true` 相当の応答を確認する

正常の確認ポイント:

- `State` が `READY` 以上（`IDLE` のままなら mask 未構築）
- `Clock` が `locked`（`unknown` が続く場合は PTP を確認）
- `Healthy` が崩れていない
- `Last Error` が空または古い

### 4.2 Step 01: Blob Detection Adjustment

単一点の反射マーカーをカメラに見せながら調整します。

調整項目:

| パラメータ | 意味 |
|---|---|
| threshold | blob 検出の輝度閾値 |
| circularity | 丸さフィルタ（0〜1） |
| blob min/max diameter | 検出する blob の直径範囲 |
| exposure | 露出時間 |
| gain | ゲイン |
| fps | フレームレート |
| focus | フォーカス（manual） |

判断基準:

- pose capture 用の単一点が安定して拾える
- 反射や背景ノイズが大量に拾われない
- カメラごとの差が大きすぎない

調整後は数秒待ち、blob diagnostics が落ち着くのを確認します。

### 4.3 Step 02: Mask Adjustment

背景だけが画角に見えている状態で実行します。

1. wand や人を画角から外す
2. **Build Mask** を押す
3. `State` が `READY` に戻るまで待つ（30 フレーム収録、約 1 秒）
4. wand を見せて、必要な点だけ残るか確認する

失敗の兆候:

- wand を見せても blob が消える → wand が mask に含まれた → やり直し
- 背景ノイズが増える → 動く物体が映り込んでいた → やり直し

`Build Mask` はやり直し可能です。READY に戻ったことを確認してから次へ進みます。

> **注意:** `READY` でないカメラが混ざっている状態では Pose Capture を開始できません。

### 4.4 Step 03: Pose Capture

単一点の反射マーカー（1点のみ）を使います。

1. **Start Pose Capture** を押す
2. マーカーを空間全体でゆっくり動かす
   - カメラ間で重なる視野を広く動かす
   - 空間の隅、高さの変化を網羅する
   - 最低でも 2 台のカメラが同時に観測できる区間を十分作る
3. 十分な量（目安: 数分〜5分程度）収録したら **Stop Pose Capture** を押す

ログは `logs/extrinsics_pose_capture.jsonl` に追記されます。

> **注意:** `pose_capture` の UDP ペイロードは full blobs を保持します。Pi 側で best blob に絞り込まれないため、multi-blob フレームの判定は Host 側 solver が行います。

### 4.5 Step 04: Floor / Metric Capture

wand（4点マーカー）を使います。

1. wand を床に静置する（水平に置く）
2. **Capture Floor / Metric** を押す
3. 自動的に数秒で停止し、`logs/extrinsics_wand_metric.jsonl` が更新される

この capture も static mask が必須です。wand は静止していれば OK です。

> この step で `metric.status=resolved` と `world.status=resolved` が得られます。wand log が無い場合は両方 `unresolved` のまま残ります。

### 4.6 Step 05: Generate Extrinsics

**Generate Extrinsics** を押します。

パラメータ（通常はデフォルトで OK）:

| パラメータ | デフォルト | 意味 |
|---|---|---|
| Pair Window (us) | 2000 | 同時観測と判定する timestamp 差の許容幅 |
| Wand Pair Window (us) | 8000 | wand metric 収録の同期許容幅 |
| Min Pairs | 8 | ref camera との同時観測の最低サンプル数 |
| Intrinsics Dir | calibration | intrinsics JSON の格納ディレクトリ |

完了の確認項目:

- エラーで止まらない
- `camera count` が期待台数に近い
- `calibration/extrinsics_pose_v2.json` が生成される
- `pose.solve_summary` に以下が入る:
  - `usable_rows` / `complete_rows`
  - `median_reproj_error_px` / `p90_reproj_error_px`
  - `matched_delta_us_p50` / `matched_delta_us_p90` / `matched_delta_us_max`
- wand log があれば `metric.status=resolved`, `world.status=resolved`

**目安:**

- `median_reproj_error_px` < 2.0 px: 良好
- `matched_delta_us_p90` < 3000 us: PTP 同期が良好
- extrinsics が生成されない場合は [§6 トラブルシューティング](#6-トラブルシューティング) へ

---

## 5. Tracking 動作確認 (GUI: Tracking タブ)

extrinsics が生成されたら **Tracking** タブへ移動します。

### 5.1 開始前確認

- `Extrinsics: calibration/extrinsics_pose_v2.json` が表示される
- status badge が `Ready`（`Waiting` のままなら extrinsics が未生成か読み込み失敗）
- 3D viewer が表示される

### 5.2 Tracking 開始

1. rigid body を撮影空間に入れる
2. **Start Tracking** を押す
3. 数秒後に確認:
   - status badge が `Tracking`
   - `frames_processed` / `poses_estimated` が増える
   - 3D viewer に raw points や rigid body が出る

### 5.3 品質確認ポイント

**3D viewer:**

- camera frustum の向きが壊れていない
- raw points が空間の変な遠方へ飛び続けない
- rigid body の軸・trail が滑らかに続く

**Camera Health:**

- FPS が極端に低いカメラがない
- latency が 1 台だけ大きく悪化していない
- `Blob avg` が 0 に張り付いていない

**Rigid Bodies:**

- `Valid` が維持される
- `Observed` が必要数を下回り続けない
- RMS が急に悪化し続けない

### 5.4 Tracking 停止

**Stop Tracking** を押します。status badge が `Ready` に戻れば完了です。

---

## 6. トラブルシューティング

### 6.1 Refresh してもカメラが出ない

1. Pi 側 `capture.py` が起動中か確認
2. `--udp-dest` に Host の実 IP が指定されているか（`0.0.0.0` は不可）
3. Host から Pi の TCP 8554 へ疎通があるか
4. `src/deploy/hosts.ini` の IP と `camera_id` が実起動値と一致しているか

### 6.2 Ping は通るが blob が出ない

1. lens cap やピントずれがないか
2. exposure / threshold が極端すぎないか
3. mask が wand を消していないか
4. wand の LED やマーカーに物理異常がないか
5. Pi 側 preview で blob 円が見えているか

### 6.3 Intrinsics 較正の精度が悪い

1. RMS > 2.0 px の場合は **Clear Frames** してやり直す
2. `Rejected (no detect)` が多い → 照明改善、ボードのしわ・汚れ確認
3. Coverage Grid のセルが偏っている → ボードの持ち方・移動範囲を変える
4. `Cooldown (s)` を下げると短時間で多くのフレームが取れるが、重複が増える

### 6.4 Extrinsics が生成できない

1. `logs/extrinsics_pose_capture.jsonl` に複数カメラの同時観測区間が十分あるか
2. `calibration/` に全カメラ分の intrinsics JSON があるか
3. `camera_id` の表記ゆれがないか（例: `pi-cam-01` vs `picam01`）
4. `Min Pairs` を満たすサンプルが各カメラにあるか（不足カメラは除外される）

台数が少ない場合:

- `session_meta.excluded_camera_ids` と `excluded_camera_reasons` を確認
- 除外されたカメラだけ pose capture を増やして再生成

### 6.5 Tracking が始まらない

1. `calibration/extrinsics_pose_v2.json` が存在するか
2. `world.status=resolved` になっているか（`unresolved` なら wand metric が必要）
3. Camera Health に入力が来ているか（UDP 5000 が届いているか）
4. rigid body pattern が視野内にあるか

### 6.6 Tracking は動くが品質が悪い

1. 1 台だけ latency が悪化していないか（PTP ずれの可能性）
2. blob 誤検出が増えていないか → mask を再構築
3. wand / extrinsics 作成時からカメラが動いていないか → extrinsics からやり直し
4. mask が古くなっていないか → **Build Mask** を再実行

---

## 7. セッション終了後に保存するもの

最低限以下を保存または確認します:

- `logs/extrinsics_pose_capture.jsonl`
- `logs/extrinsics_wand_metric.jsonl`
- `logs/wand_gui_settings.json`
- `calibration/extrinsics_pose_v2.json`
- `calibration/calibration_intrinsics_v1_<camera_id>.json`（全カメラ分）

記録しておくと良い運用メモ:

- 使用カメラ台数
- 除外カメラの有無と理由（`excluded_camera_reasons`）
- `pose.solve_summary` の主要指標（usable_rows / complete_rows / median_reproj_error_px / matched_delta_us_p90）
- tracking 品質の所感

---

## 8. コマンドリファレンス

### Pi 起動（フル構成）

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview \
  --mjpeg-port 8555
```

### Host GUI 起動

```bash
python src/host/wand_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

### Extrinsics CLI 生成（GUI の代替）

```bash
.venv/bin/python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --wand-metric-log logs/extrinsics_wand_metric.jsonl \
  --output calibration/extrinsics_pose_v2.json
```

### Extrinsics 3D 可視化

```bash
.venv/bin/python src/host/view_extrinsics_3d.py \
  --extrinsics calibration/extrinsics_pose_v2.json \
  --intrinsics-dir calibration \
  --save logs/extrinsics_pose_v2_view.png
```

### Pose log 品質分析

```bash
.venv/bin/python src/camera-calibration/analyze_pose_segments.py \
  --intrinsics calibration \
  --extrinsics calibration/extrinsics_pose_v2.json \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --pair-window-us 2000 \
  --output logs/extrinsics_pose_segments_analysis.json
```

### 悪い時間帯を除外して再 solve

```bash
# 悪いセグメントを除外した pose log を生成
.venv/bin/python src/camera-calibration/filter_pose_log_segments.py \
  --intrinsics calibration \
  --extrinsics calibration/extrinsics_pose_v2.json \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --output logs/extrinsics_pose_capture_filtered.jsonl \
  --pair-window-us 2000 \
  --median-ratio-threshold 1.5

# filtered log で再生成
.venv/bin/python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture_filtered.jsonl \
  --wand-metric-log logs/extrinsics_wand_metric.jsonl \
  --output calibration/extrinsics_pose_v2.json
```

---

## 9. 関連資料

- [`/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/README.md) — プロジェクト全体の概要と起動手順
- [`/src/pi/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/README.md) — Pi 側 capture 詳細
- [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md) — intrinsics 較正の詳細
- [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md) — Host モジュール概要
