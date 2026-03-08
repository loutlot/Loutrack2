# next_steps_wand_runbook

wand 外部較正を実行するための作業手順書（実装済みフロー準拠）。

## 1. 目的

- 複数 Pi の wand 収録を Host 主導で実行する。
- 収録ログから `calibration_extrinsics_v1.json` を生成する。
- Host 側で intrinsics + extrinsics を同時ロードして動作確認する。

## 2. 前提条件

以下を「収録前チェック」として上から順に実施する。

### 2.1 Host 環境の準備

1. 作業ディレクトリへ移動:

```bash
cd /Users/loutlot/Documents/cursor/MOCAP/Loutrack2
```

2. Python が使用可能であることを確認:

```bash
python3 --version
```

3. 依存パッケージを導入:

```bash
python3 -m pip install -r requirements.txt
```

4. 主要スクリプトが起動可能なことを確認:

```bash
python3 src/host/wand_gui.py --help
python3 src/camera-calibration/calibrate_extrinsics.py --help
```

### 2.2 Pi 側 capture サービスの確認

各 Pi で以下を満たすこと:

- `src/pi/capture.py` が `mask_start` / `mask_stop` / `start(mode=wand_capture)` 対応版で起動している
- `camera_id` が固有で重複しない
- control port `8554` で応答できる

Host から疎通確認（各 Pi 分実施）:

```bash
python3 -m src.host.control --ip <PI_IP> --port 8554 --camera-id <CAMERA_ID> ping
```

期待値:

- 返却 JSON に `"ack": true` が含まれる

### 2.3 inventory と camera_id の整合

1. inventory を確認:

```bash
cat src/deploy/hosts.ini
```

2. 確認ポイント:

- 1行につき 1 台
- `camera_id` 重複なし
- IP が最新

例（形式）:

```ini
pi-cam-01 192.168.1.101 pi-cam-01
pi-cam-02 192.168.1.102 pi-cam-02
```

### 2.4 ネットワーク前提（受動発見 + 制御）

- Host と全 Pi が同一 L2/L3 セグメントで通信可能
- Pi -> Host の UDP 送信（既定: 5000/udp）が通る
- Host -> Pi の TCP 制御（8554/tcp）が通る

確認の目安:

1. GUI 起動後 `Refresh` でカメラが列挙される（受動発見 + inventory merge）
2. `Ping` 実行で `last_ack=true` が返る

### 2.5 内部較正ファイルの確認

外部較正の前に、対象カメラ分の intrinsics が揃っていることを確認:

```bash
ls -1 calibration/calibration_intrinsics_v1_*.json
```

最低条件:

- wand 収録対象の全 camera_id についてファイルが存在する
- 各 JSON に `camera_id`, `camera_matrix`, `distortion_coefficients` が含まれる

簡易チェック（1ファイル例）:

```bash
python3 - <<'PY'
import json
path = "calibration/calibration_intrinsics_v1_pi-cam-01.json"
data = json.load(open(path, "r", encoding="utf-8"))
required = ["camera_id", "camera_matrix", "distortion_coefficients"]
missing = [k for k in required if k not in data]
print("missing:", missing)
PY
```

### 2.6 wand 物理条件の確認

- wand 定義: `WAND_POINTS_MM = [(0,0,0), (168,0,0), (0,243,0)]`
- 3 marker の center-to-center 実寸が定義値と一致
- マーカー径 14mm

注意:

- 実寸がズレると `baseline_m` とスケール推定が崩れる
- 反射材の汚れ・剥離がある場合は交換してから収録する

## 3. 事前確認

1. inventory を確認する:

```bash
cat src/deploy/hosts.ini
```

2. 受動発見を使う場合は Pi から UDP が流れていることを確認する。

3. 最低 2 台が疎通することを確認する（例）:

```bash
python3 -m src.host.control --ip 192.168.1.101 --port 8554 --camera-id pi-cam-01 ping
python3 -m src.host.control --ip 192.168.1.102 --port 8554 --camera-id pi-cam-02 ping
```

## 4. 収録手順（GUI 運用）

1. GUI を起動:

```bash
python3 src/host/wand_gui.py --host 127.0.0.1 --port 8765 --udp-port 5000
```

2. ブラウザで `http://127.0.0.1:8765/` を開く。

3. `Refresh` → 対象カメラを選択 → `Ping` 実行。

4. スライダーを設定して `Apply To Selected`:
   - `exposure_us`
   - `gain`
   - `fps`
   - `focus`（既定値 `5.215`）
   - `threshold`
   - `circularity_min`
   - `blob min/max diameter px`

5. `Mask Start` 実行。
   - `mask threshold` と `mask seconds` は GUI 側設定を使用する
   - `mask seconds * fps` で内部の `frames` が計算される

6. `Start` 実行後、wand を空間全体で 60 秒程度動かす（20 ポーズ以上）。

7. `Stop` 実行。

8. 収録後、GUI の `Generate Extrinsics` を実行:
   - `Intrinsics Dir`: 例 `calibration`
   - `Log Path`: 収録ログ JSONL のパス
   - `Output Path`: 例 `calibration/calibration_extrinsics_v1.json`

## 5. 収録手順（CLI 運用）

GUI を使わない場合は `WandSession` を直接実行する。

```python
from src.host.wand_session import SessionConfig, WandSession

session = WandSession()
result = session.run_session(
    SessionConfig(
        exposure_us=1200,
        gain=4.0,
        fps=80,
        duration_s=60.0,
    )
)
print(result["session_id"])
print(result["metadata_path"])
```

## 6. 外部較正の生成

収録ログ（JSONL）を指定して extrinsics を作成する。

```bash
python3 src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --log logs/<wand_capture_log>.jsonl \
  --output calibration/calibration_extrinsics_v1.json \
  --pair-window-us 5000 \
  --min-pairs 8
```

必要に応じて参照カメラを固定:

```bash
python3 src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --log logs/<wand_capture_log>.jsonl \
  --reference-camera pi-cam-01
```

## 7. 検証

1. 出力 JSON の生成確認:

```bash
ls -l calibration/calibration_extrinsics_v1.json
```

2. Host 幾何ロード確認（`geo.py` は intrinsics と extrinsics を同時ロード）:

```python
from src.host.geo import GeometryPipeline

geo = GeometryPipeline()
count = geo.load_calibration("calibration")
print("loaded cameras:", count)
print("camera ids:", geo.get_camera_ids())
```

3. 主要 quality 値確認:
   - `cameras[].quality.inlier_ratio`
   - `cameras[].quality.median_reproj_error_px`
   - `cameras[].quality.baseline_m`

## 8. トラブルシュート

- `mask_start` 失敗:
  - 光源反射が強すぎる場合、露光を下げる。
  - `mask_start` を再実行する。
- `Not enough paired observations`:
  - 収録時間を延長し、wand の姿勢バリエーションを増やす。
  - `--pair-window-us` を少し広げる（例: 7000）。
- `findEssentialMat failed`:
  - 対象カメラが同じ視野を十分に共有しているか確認する。
  - 2D blob の誤検出が多い場合は threshold / mask 条件を見直す。
- `baseline_m` が不自然:
  - wand の実寸（center-to-center）を再確認する。
  - `blobs` に wand 以外が混入していないかログ確認する。

## 9. 成果物

- 収録セッション metadata:
  - `logs/wand_sessions/<session_id>.json`
- 外部較正成果物:
  - `calibration/calibration_extrinsics_v1.json`
- 参照スキーマ:
  - `schema/calibration_extrinsics_v1.json`
