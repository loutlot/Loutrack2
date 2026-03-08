# Charuco較正: 実機ゼロベース手順

このドキュメントは、sim/self-test の結果を参照せず、実機で内部較正を最初から作り直すための手順書です。

## 前提

- 対象: Raspberry Pi Camera Module 3 Wide
- 実測値: 5マス = 150 mm
- 1マス換算: `150 / 5 = 30.0 mm`
- 本手順での指定値: `--square-length-mm 30.0`
- 実機メモ: `LensPosition=5.2` が現時点で最も安定
- Loutrack2 の現行 live capture 既定解像度: `2304x1296`

---

## Step 0: まっさらな Raspberry Pi の初期セットアップ

Raspberry Pi Connect で画面共有できている前提で、内部較正を実行可能な状態まで準備する。

### 0-1. OS 更新と基本ツール

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git python3-pip python3-venv
```

### 0-2. リポジトリ取得

```bash
cd ~
git clone https://github.com/<YOUR_ACCOUNT>/Loutrack2.git
cd Loutrack2
```

### 0-3. Python 仮想環境（重要）

`python3-picamera2` は apt でシステム側に入るため、仮想環境は `--system-site-packages` 付きで作成する。

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

### 0-4. Pi カメラ依存（ライブ較正用）

```bash
sudo apt install -y python3-picamera2
sudo reboot
```

再起動後:

```bash
cd ~/Loutrack2
source .venv/bin/activate
```

### 0-5. カメラ動作確認

OS によって `rpicam-hello` または `libcamera-hello` を使う。

```bash
rpicam-hello -t 3000
```

`rpicam-hello` が無い場合:

```bash
libcamera-hello -t 3000
```

加えて Python 側確認:

```bash
python3 -c "from picamera2 import Picamera2; print('picamera2 ok')"
```

### 0-6. トラブルシュート（`picamera2 is required` が出る場合）

`calibrate.py` で次のエラーが出る場合:

`ERROR: picamera2 is required for live capture mode.`

多くは venv から apt の `picamera2` が見えていないことが原因。以下で venv を作り直す。

```bash
cd ~/Loutrack2
deactivate 2>/dev/null || true
rm -rf .venv
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
python -c "from picamera2 import Picamera2; print('picamera2 ok')"
```

それでも import できない場合:

```bash
sudo apt install -y python3-picamera2
sudo reboot
```

---

## Step 1: `pi-cam-01` を実機で新規内部較正する

最初は `pi-cam-01` だけで実行し、手順と品質基準を固める。

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm 30.0 \
  --min-frames 25 \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

ライブ撮影時の操作:

- `SPACE`: 有効フレーム取り込み
- `C`: 較正実行
- `Q`: 終了

### Step 1 の具体的な進め方

結論としては「`SPACE` を連打して十分な有効フレームを集めてから `C`」でよい。
ただし、同じ構図を連打しても精度が上がりにくいので、毎回ボード位置と角度を変える。

1. 起動直後にプレビューでピントと露出を確認する。
2. ボードを中央で正面に置き、`SPACE` を 3-5 回取得する。
3. ボードを四隅（左上/右上/左下/右下）へ移動し、各位置で 3-5 回ずつ `SPACE`。
4. 距離を変える（近距離/中距離/遠距離）ごとに 3-5 回ずつ `SPACE`。
5. 角度を変える（左右に傾ける、上下に傾ける）ごとに 3-5 回ずつ `SPACE`。
6. 合計 25 枚に達したら、可能なら 30-40 枚まで追加する。
7. 取得後に `C` を押して較正を実行する。
8. 保存結果を確認し、必要なら `Q` で終了して再実行する。

### 押し方の注意

- `SPACE` は「検出が安定して見える瞬間」に押す。
- 手ブレ中に連打しない（近いフレームが重複しやすい）。
- 1秒に1回程度を目安に、構図を変えながら押す。
- `C` は最低25枚を満たしてから押す。
- `C` 実行後にエラーや品質不良なら、撮影分布を改善してやり直す。

撮影の実践ルール:

- 25枚以上、推奨30-40枚
- 中央だけでなく四隅・画面端を含める
- 近距離/遠距離、正面/斜めを混ぜる
- いろんな構図で撮る（位置・距離・角度を毎回変える）
- 構図が被らないようにする（同じ見え方の連続取得を避ける）
- フォーカスは可能なら `LensPosition=5.2` を基準値として試す
- ブレ、露出飽和、ピンぼけを避ける

---

## Step 2: `pi-cam-01` の結果を確認する

確認対象: `calibration/calibration_intrinsics_v1_pi-cam-01.json`

暫定の合格基準:

- `rms_error < 1.0`（理想 `< 0.5`）
- `quality.num_valid_frames >= 25`
- `per_view_errors` に極端な外れ値が連続しない

NG の場合は Step 1 を再実行し、以下を優先的に改善する。

- ボードの角度・距離バリエーションを増やす
- 画面周辺での取得枚数を増やす
- ピントと露出を調整する

---

## Step 3: `pi-cam-02` に同じ手順を適用する

`pi-cam-01` で手順が安定したら、同じ条件で `pi-cam-02` を実施する。

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-02 \
  --square-length-mm 30.0 \
  --min-frames 25 \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-02.json"
```

結果確認の観点は Step 2 と同じ。

---

## Step 4: 外部較正（wand 方式）

内部較正2台分が揃った後に、B5 L字 3点 wand（14mm blob）で外部較正を実装・実行する。

Host 主導の基本シーケンス:

1. `set_exposure` / `set_gain` / `set_fps` を配布
2. `mask_start` で 30 フレーム初期化（背景明点マスク）
3. `start` で wand 収録開始
4. 20+ ポーズ収録（推奨 60 秒）
5. `stop` で収録終了
6. Host で extrinsics solver を実行

- 実装予定: `src/camera-calibration/calibrate_extrinsics.py`
- 出力予定: `calibration/calibration_extrinsics_v1.json`
- 詳細計画: `docs/10_in_progress/wand_extrinsics_plan.md`

---

## Step 5: ワークフロー統合

最終的な流れ:

1. Charuco ボード印刷と実測
2. 各カメラで内部較正（intrinsics）
3. 複数カメラ同時撮影データ取得
4. wand 収録と外部較正（extrinsics）
5. Host 側で読み込み、三角測量検証

---

## 優先順位まとめ

| 優先度 | タスク | 依存 |
|--------|--------|------|
| P0 | `pi-cam-01` 実機内部較正（ゼロベース） | なし |
| P0 | `pi-cam-01` 結果レビューと再撮影判定 | P0 |
| P0 | `pi-cam-02` 実機内部較正 | `pi-cam-01` 手順確定 |
| P1 | 外部較正スクリプト実装 | P0完了 |
| P1 | 外部較正スキーマ定義 | なし |
| P2 | 較正ワークフロー統合 | P1完了 |

---

## 参考リンク

- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- OpenCV Stereo Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d

---

## 運用ルール

- sim/self-test の出力は本書の判定基準に使わない。
- 実測で進捗が出たら `README.md` の TODO と Phase 状態を同時更新する。
