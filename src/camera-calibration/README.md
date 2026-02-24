# camera-calibration

Loutrack2 向けの Charuco ボードベース内部較正ツールです。

## 概要

このディレクトリには次の 2 つのスクリプトがあります。

- `charuco_board.py`: 印刷用 Charuco ボード（A4 PDF）を生成
- `calibrate.py`: Charuco 観測からカメラ内部パラメータを推定

`calibrate.py` は次のモードをサポートします。

- ライブ撮影モード（Pi Camera、`picamera2` 必須）
- オフラインモード（`--input-dir` で画像群を指定）
- ハードウェア不要の自己テスト（`--self-test`）

## ファイル

- `src/camera-calibration/charuco_board.py`
- `src/camera-calibration/calibrate.py`

## 前提条件

- Python 3.9+
- ArUco 対応 OpenCV（`opencv-python`）
- NumPy
- Pillow（`charuco_board.py` の PDF 出力で使用）
- （ライブモードのみ）Raspberry Pi 上の `picamera2`

リポジトリルートで依存をインストール:

```bash
pip install -r requirements.txt
```

## 1) Charuco ボード生成

基本コマンド:

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata
```

印刷時の重要ポイント:

- 100% スケールで印刷する
- fit-to-page を無効にする
- 印刷後に 1 マスの辺長を実測し、その値を較正に使う

主なオプション:

- `--squares-x` / `--squares-y`（既定値: `6` / `8`）
- `--square-length-mm`（既定値: `30.0`）
- `--marker-length-mm`（既定値: `20.0`）
- `--dictionary`（既定値: `DICT_6X6_250`）
- `--dpi`（既定値: `300`）
- `--write-metadata`（PDF 横にメタデータ JSON を出力）

## 2) カメラ内部較正の実行

`--square-length-mm` は全モードで必須です。

### A. ライブ撮影モード（Raspberry Pi）

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm <MEASURED_MM> \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

ライブ撮影時のキー操作:

- `SPACE`: フレームを取り込み（コーナー検出時のみ）
- `C`: 較正を実行（最小有効フレーム数を満たしたとき）
- `Q`: 終了

### B. オフラインモード（画像ディレクトリ）

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm <MEASURED_MM> \
  --input-dir "./calibration/images/pi-cam-01" \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

### C. 自己テストモード（合成データ）

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm 30.0 \
  --self-test \
  --self-test-views 30 \
  --seed 0
```

## 出力

較正結果はリポジトリのスキーマ形式 JSON で保存されます。

- `calibration/calibration_intrinsics_v1_<camera_id>.json`

主な出力項目:

- カメラ行列（`fx`, `fy`, `cx`, `cy`）
- 歪み係数（`k1`, `k2`, `p1`, `p2`, `k3`）
- `rms_error`
- ボード設定と品質指標（`per_view_errors`、有効フレーム数など）

## 品質目安

- RMS 誤差の目安: 理想 `< 0.5 px`、許容 `< 1.0 px`
- 有効フレームは最低 `25` 枚以上（`--min-frames`、既定 `25`）
- 位置・傾き・距離・向きを変えて多様な姿勢で撮影する
- ピントずれやブレを避ける

### 有効フレームの考え方

このツールでの「有効フレーム」は、Charuco コーナーが十分に検出できた画像です。
`calibrate.py` では、1 フレーム中に検出された Charuco コーナーが一定数未満の場合、そのフレームは較正に使われません。

撮影時の実践ポイント:

- 中央だけでなく、画面の四隅・端にもボードを配置する
- 正面だけでなく、角度・距離を変えて撮る（近距離/遠距離を混ぜる）
- 露出過多・反射・モーションブラーを避ける

### ボードが見切れるフレームについて

ボード全体が必ずしも写っている必要はありません。
一部が見切れていてもコーナーが十分検出できれば有効です。

ただし、見切れたフレームだけに偏ると推定が不安定になることがあります。
「全体が見えるフレーム」と「端で一部見切れるフレーム」を混ぜると、歪み推定の安定性が上がります。

`per_view_errors` に極端な値が出る場合は次を再確認してください。

- 印刷スケールが正確に 100% か
- `--square-length-mm` に実測値を正しく渡しているか
- 画像中のコーナー検出品質が十分か

## 実運用手順書（内部較正）

以下は、印刷ボード作成から較正 JSON 出力までの推奨手順です。

0. **Pi 側で実行する場合の準備**

   ライブ撮影モード（Pi Camera を直接使うモード）は Raspberry Pi 上で実行します。
   スクリプトだけでなく、依存とボードも必要です。

   - Pi に配置: `src/camera-calibration/calibrate.py`（必要なら `charuco_board.py` も）
   - 依存導入: `numpy`、ArUco 対応 OpenCV、（ライブ時）`picamera2`
   - ボード準備: 印刷済み Charuco ボード

   例（Pi 上で依存を入れる）:

   ```bash
   pip install -r requirements.txt
   ```

   実行後の JSON は Pi ローカルに保存されるため、ホストで使う場合は回収します。
   例:

   ```bash
   scp pi@<PI_IP>:/home/pi/loutrack2/calibration/calibration_intrinsics_v1_<camera_id>.json ./calibration/
   ```

1. **ボードを生成して印刷する**

   ```bash
   python src/camera-calibration/charuco_board.py \
     --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
     --write-metadata
   ```

   - 印刷は 100% スケール（fit-to-page 無効）
   - 印刷後に 1 マスの辺長を実測し、`--square-length-mm` に使う

2. **撮影モードを選ぶ**

   - Raspberry Pi カメラでその場で撮る: ライブ撮影モード
   - 既存画像から較正する: オフラインモード

3. **ライブ撮影でフレームを収集する（Pi Camera）**

   ```bash
   python src/camera-calibration/calibrate.py \
     --camera pi-cam-01 \
     --square-length-mm <MEASURED_MM> \
     --min-frames 25 \
     --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
   ```

   - `SPACE`: 有効に見えるフレームを取り込む
   - `C`: 較正実行（`--min-frames` 以上で有効）
   - `Q`: 終了
   - 中央/四隅、近距離/遠距離、角度違いを混ぜて収集する

4. **オフライン画像から較正する**

   ```bash
   python src/camera-calibration/calibrate.py \
     --camera pi-cam-01 \
     --square-length-mm <MEASURED_MM> \
     --input-dir "./calibration/images/pi-cam-01" \
     --min-frames 25 \
     --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
   ```

   - 画像枚数ではなく「有効フレーム数」が `--min-frames` 以上必要
   - 見切れ画像は、コーナーが十分検出されるなら使用可

5. **結果を確認する**

   - 出力 JSON: `calibration/calibration_intrinsics_v1_<camera_id>.json`
   - 目安: `rms_error < 1.0 px`（理想は `< 0.5 px`）
   - `per_view_errors` に極端な値がある場合は、ブレ・反射・印刷スケールを再確認する

6. **必要なら再撮影・再較正する**

   - 有効フレーム数を増やす（例: 25 -> 35）
   - 画面周辺でのフレーム比率を増やす
   - ピントや露出を調整してから再実行する

## 補足

- メインの追跡パイプラインはヘッドレス実行できますが、ライブ較正は OpenCV の表示ウィンドウが必要です。
- `--marker-length-mm` 未指定時は `square_length_mm * 20/30` が使われます。
- `calibrate.py` では OpenCV ArUco API のバージョン差分を吸収する互換処理を入れています。
