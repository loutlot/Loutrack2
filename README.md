# Loutrack2

Raspberry Pi Camera 3 Noir Wide を使ったアウトサイドイン型モーションキャプチャ実験プロジェクト。

## 開発状況（2026-03-09）

| 項目 | 状態 | 実体 |
|---|---|---|
| Pi での blob 検出 + UDP 送信 | ✅ 実装済み | `src/pi/capture.py` |
| Host 受信 + ペアリング + 三角測量 | ✅ 実装済み | `src/host/receiver.py`, `src/host/geo.py`, `src/host/pipeline.py` |
| wand 外部較正（収録/GUI/solver） | ✅ 実装済み | `src/host/wand_gui.py`, `src/host/wand_session.py`, `src/camera-calibration/calibrate_extrinsics.py` |
| 較正成果物 schema 運用 | ✅ 実装済み | `schema/calibration_intrinsics_v1.json`, `schema/calibration_extrinsics_v1.json` |
| 5剛体の常用 tracking UI | 🚧 進行中 | `docs/10_in_progress/tracking_gui_plan.md` |
| SteamVR 出力 | ⏳ 未着手 | （実装ファイルなし） |

## まずどこから手を付けるか（閲覧者向け）

1. **現行ワークフローを把握する**  
   `docs/20_completed/24_next_steps_wand_runbook.md`
2. **tracking 拡張の実装対象を確認する**  
   `docs/10_in_progress/tracking_gui_plan.md`
3. **コードの入口を読む**  
   - Pi 側: `src/pi/capture.py`  
   - Host 側: `src/host/wand_gui.py`, `src/host/pipeline.py`
4. **既存テストで壊れていない範囲を確認する**  
   `tests/test_wand_gui.py`, `tests/test_wand_extrinsics.py`, `tests/test_pi_control_e2e.py`

## 現在できること（実装ベース）

- Pi で検出した blob を UDP/JSON で Host に送信できる。
- Host で multi-camera フレームをペアリングし、三角測量と再投影誤差評価ができる。
- Host から Pi への `mask_start/start/stop` と露光・ゲイン等の制御ができる。
- wand 収録ログ（JSONL）から extrinsics を生成し、geo モジュールで読み込める。
- 同期評価・GUI 状態・control transport について pytest ベースの回帰テストがある。

## 未完了/課題（次に進めるべき点）

- 実機再計測を含む intrinsics/extrinsics の運用合格基準の確定。
- 受動発見（UDP）と `src/deploy/hosts.ini` 併用でのカメラ確定ロジックの仕上げ。
- tracking GUI（Page 2）実装と 5剛体運用の安定化。
- SteamVR 出力層の追加。

## 実装予定プラン（実体準拠）

### Phase A: 較正運用を固める
- `docs/20_completed/21_charuco_runbook.md` で intrinsics 再収録。
- `docs/20_completed/24_next_steps_wand_runbook.md` で extrinsics 再生成。
- 合格指標（pair 数 / reprojection error）を README と runbook に固定する。

### Phase B: カメラ確定ロジック完成
- `src/host/wand_session.py` と `src/host/wand_gui.py` で受動発見 + inventory マージを完成。
- 未発見/重複 camera_id/通信断の扱いを GUI とログに明示。

### Phase C: tracking GUI 実装
- `docs/10_in_progress/tracking_gui_plan.md` に沿って Page 2 を実装。
- 3D 表示、frustum 表示、health panel を段階導入。

### Phase D: 出力統合
- 5剛体 tracking の評価基準を固定。
- SteamVR 出力を追加。

## 実行コマンド（最小）

### 1. 環境構築

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Host GUI 起動

```bash
python src/host/wand_gui.py
```

### 3. Extrinsics 生成（CLI）

```bash
python src/camera-calibration/calibrate_extrinsics.py \
  --log-path logs/wand_capture.jsonl \
  --intrinsics-dir calibration \
  --output calibration/calibration_extrinsics_v1.json
```

### 4. テスト実行

```bash
.venv/bin/python -m pytest
```

## ディレクトリ構成（実体）

```text
loutrack2/
├── src/
│   ├── pi/                    # Pi capture service
│   ├── host/                  # Host pipeline / GUI / control
│   ├── camera-calibration/    # intrinsics / extrinsics scripts
│   └── deploy/                # deploy scripts + hosts.ini
├── schema/                    # JSON schema
├── tests/                     # pytest suites
├── calibration/               # calibration artifacts (json/pdf)
├── docs/
│   ├── 00_pre_implementation/ # 初期要件/設計
│   ├── 10_in_progress/        # 現在進行中（tracking GUI）
│   └── 20_completed/          # 実行済み runbook / 完了設計
├── context/                   # 要件起点 + archive
├── logs/                      # wand capture / GUI settings
└── hardware/                  # 3D print / KiCad
```

## ドキュメント案内

- 現在進行中: `docs/10_in_progress/tracking_gui_plan.md`
- 実行手順（完了版）: `docs/20_completed/21_charuco_runbook.md`, `docs/20_completed/24_next_steps_wand_runbook.md`
- 設計背景: `docs/00_pre_implementation/`
- schema 詳細: `schema/README.md`

## 参考プロジェクト

- [Low-Cost-Mocap](https://github.com/jyjblrd/Low-Cost-Mocap)
