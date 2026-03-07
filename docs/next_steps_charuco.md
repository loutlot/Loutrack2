# Charuco較正: 今後の手順

## 現状サマリー（2026-03 時点）

| 項目 | 状態 | 備考 |
|------|------|------|
| Charucoボード生成 | ✅ 完了 | `calibration/boards/charuco_6x8_30mm_a4.pdf` |
| 内部較正スクリプト | ✅ 完了 | `src/camera-calibration/calibrate.py` |
| pi-cam-01 内部較正 | ⚠️ 要実測確認 | 既存結果はダミー/暫定値が混在している可能性あり |
| pi-cam-02 内部較正 | ❌ 未実施 | hosts.ini に定義あり |
| 外部較正スクリプト | ❌ 未実装 | stereoCalibrate ベースが必要 |
| 外部較正スキーマ | ❌ 未定義 | calibration_extrinsics_v1.json 的なもの |

### 実測メモ

- Charuco 印刷ボードの実測値: 5マス = 138 mm
- 1マス換算値: `138 / 5 = 27.6 mm`
- 較正コマンドでは `--square-length-mm 27.6` を使用する

---

## Step 1: 既存較正データの棚卸しと品質確認

### 方針

現状はダミー/暫定データの混在可能性があるため、特定の数値を前提にせず、
まず「実測由来かどうか」を判定してから品質評価を行う。

- 既存 JSON の `captured_at` / `quality` / `board` を確認
- 実機採取ログや画像セットの有無を確認
- 実測由来でない場合は「参考値」に降格し、再較正を優先

### 主な原因候補

1. Charucoボード印刷時のスケール不一致
2. `--square-length-mm` と実際のマス幅が合っていない
3. 合成データ（self-test）起源の結果が混在している
4. キャリブレーション画像の画質・角度不足

### 対応

1. **実機で再較正**
   ```bash
   # Pi上で実行（要 picamera2）
   python src/camera-calibration/calibrate.py \
     --camera pi-cam-01 \
     --square-length-mm 27.6 \
     --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
   ```

2. **実測値を正確に取得**
   - 印刷したボードの1マスをノギスで実測
   - その値を `--square-length-mm` に指定

3. **撮影時の注意**
   - 最低25枚以上
   - ボードを様々な角度・位置で撮影
   - 画像全体にボードが大きく映るように
   - ピントが合っていることを確認

---

## Step 2: pi-cam-02 内部較正

`hosts.ini` に `pi-cam-02 192.168.1.102 pi-cam-02` が定義されているが、較正データがない。

### 手順

```bash
# Pi上で実行
python src/camera-calibration/calibrate.py \
  --camera pi-cam-02 \
  --square-length-mm 27.6 \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-02.json"
```

### 完了条件（暫定）

- `calibration/calibration_intrinsics_v1_pi-cam-02.json` が生成される
- RMS誤差 < 1.0px（理想 < 0.5px）
- `per_view_errors` が極端な外れ値を連発しない

---

## Step 3: 外部較正スクリプトの実装

### 概要

複数カメラ間の相対位置（R, t）を推定するスクリプトを作成する。

### API 設計案

```bash
# 実行例
python src/camera-calibration/calibrate_extrinsics.py \
  --cameras pi-cam-01 pi-cam-02 \
  --intrinsics-dir ./calibration/ \
  --output ./calibration/calibration_extrinsics_v1.json
```

### 機能要件

1. **入力**
   - 各カメラの内部較正ファイル（`calibration_intrinsics_v1_*.json`）
   - Charucoボードを同時に撮影した画像ペア
   - または、複数視点からの画像セット

2. **処理**
   - `cv2.stereoCalibrate()` を使用
   - 回転ベクトル R と並進ベクトル t を推定
   - 基本行列 F と本質行列 E も計算

3. **出力スキーマ案（ドラフト）**

```json
{
  "schema_version": "1.0",
  "calibration_type": "extrinsic_pairwise",
  "camera_pairs": [
    {
      "camera_1": "pi-cam-01",
      "camera_2": "pi-cam-02",
      "rotation_matrix": [[...], [...], [...]],
      "rotation_vector": [...],
      "translation_vector": [...],
      "essential_matrix": [[...]],
      "fundamental_matrix": [[...]],
      "rms_error": 0.xxx,
      "baseline_m": 0.xxx
    }
  ],
  "reference_camera": "pi-cam-01",
  "captured_at": "2026-02-23T..."
}
```

### 実装場所

`src/camera-calibration/calibrate_extrinsics.py`

### 依存関係

- Step 1, 2 の内部較正が完了していること
- OpenCV の `stereoCalibrate()` 関数

---

## Step 4: 較正ワークフローの統合

### 理想的なワークフロー

```
1. Charucoボード印刷 → 実測
2. 各カメラで内部較正（intrinsic）
3. 複数カメラで同時にボード撮影
4. 外部較正（extrinsic）
5. 較正データを Host の geo.py に読み込み
6. 三角測量のテスト
```

### 自動化の検討

- `calibrate_all.py` 的な統合スクリプト
- または、Makefile / justfile でワークフロー定義

---

## 優先順位まとめ

| 優先度 | タスク | 依存 |
|--------|--------|------|
| P0 | pi-cam-01 再較正（実測データで再生成） | なし |
| P0 | pi-cam-02 内部較正 | なし |
| P1 | 外部較正スクリプト実装 | P0完了 |
| P1 | 外部較正スキーマ定義 | なし |
| P2 | 較正ワークフロー統合 | P1完了 |

---

## 参考リンク

- OpenCV Camera Calibration: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html
- OpenCV Stereo Calibration: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga3207604e4b1a1758aa66acb6ed5aa65d
- 参考実装: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`

---

## メモ（運用ルール）

- 本書の数値は、実測で検証完了するまで「暫定値」として扱う。
- 進捗が出たら `README.md` の TODO と Phase 状態を必ず同時更新する。
