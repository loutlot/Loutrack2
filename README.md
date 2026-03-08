# Loutrack2

Raspberry Pi Camera 3 Noir Wide を使用したアウトサイドイン・モーションキャプチャシステム。

## 概要

Loutrack2は、複数台のRaspberry Pi + カメラで反射マーカーを追跡し、3D位置と剛体姿勢をリアルタイムに推定するモーションキャプチャシステムです。

### 特徴

- **低コスト**: Raspberry Pi + Pi Camera 3 Noir Wide を使用
- **リアルタイム**: UDPベースの低遅延通信
- **高精度**: DLT三角測量 + Kabschアルゴリズムによる剛体推定
- **運用**: SSH/rsyncベースのデプロイ、ログ/リプレイ機能

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────┐
│                         Host PC (Python)                         │
│  ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────────┐ │
│  │ Receiver │ → │ Pairer   │ → │ Triang.  │ → │ Rigid Body   │ │
│  │ (UDP)    │   │ (ts+idx) │   │ (DLT)    │   │ (Kabsch)     │ │
│  └──────────┘   └──────────┘   └──────────┘   └──────────────┘ │
│       ↑                                              ↓          │
│  ┌──────────┐                               ┌──────────────┐   │
│  │ Logger   │                               │ Visualizer   │   │
│  │ (JSONL)  │                               │ (Console)    │   │
│  └──────────┘                               └──────────────┘   │
└─────────────────────────────────────────────────────────────────┘
        ↑ UDP/JSON            ↑ UDP/JSON            ↑ UDP/JSON
┌───────┴───────┐    ┌───────┴───────┐    ┌───────┴───────┐
│  Raspberry Pi │    │  Raspberry Pi │    │  Raspberry Pi │
│  + Camera     │    │  + Camera     │    │  + Camera     │
│  (pi-cam-01)  │    │  (pi-cam-02)  │    │  (pi-cam-03)  │
└───────────────┘    └───────────────┘    └───────────────┘
```

## ディレクトリ構成

```
loutrack2/
├── schema/                    # JSON Schema定義
│   ├── messages.json          # Pi→Host メッセージフォーマット
│   ├── control.json           # Host→Pi 制御メッセージ
│   └── calibration_intrinsics_v1.json  # 較正成果物
├── src/
│   ├── pi/                   # Pi側キャプチャサービス
│   │   ├── capture.py         # エントリポイント
│   │   └── README.md
│   ├── deploy/                # Piデプロイスクリプト
│   │   ├── hosts.ini          # Piインベントリ
│   │   ├── deploy.sh          # デプロイスクリプト
│   │   └── rollback.sh        # ロールバックスクリプト
│   └── host/                  # Host側Pythonモジュール
│       ├── logger.py          # フレームログ記録
│       ├── replay.py          # ログリプレイ
│       ├── metrics.py         # FPS/遅延/再投影誤差
│       ├── receiver.py        # UDP受信 + ペアリング
│       ├── geo.py             # 三角測量 + 再投影誤差
│       ├── rigid.py           # 剛体推定 (DBSCAN + Kabsch)
│       ├── pipeline.py        # 統合パイプライン
│       ├── sync_eval.py       # 同期評価 (許容窓/欠損/ドリフト)
│       └── visualize.py       # 可視化ユーティリティ
├── docs/                      # 設計ドキュメント
├── tests/                     # テストコード
└── requirements.txt           # Python依存パッケージ
```

## クイックスタート

### 1. 依存パッケージのインストール

```bash
pip install -r requirements.txt
```

### 2. カメラ較正

```bash
# 1. Charucoボード生成
python src/camera-calibration/charuco_board.py --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" --write-metadata

# 2. 100%印刷（fit-to-page禁止）
# （手動で印刷設定を調整）

# 3. 1マス実測 → その値を <MEASURED_MM> に入れる
# （例: 30.2）

# 4. カメラ較正（OpenCVウィンドウ表示可能なOSで実行）
# (1) メインパイプラインはヘッドレスで実行されます（GUI不要）
# (2) 出力は calibration/calibration_intrinsics_v1_{camera_id}.json に保存されます（camera_id は --camera から取得）
python src/camera-calibration/calibrate.py --camera pi-cam-01 --square-length-mm <MEASURED_MM> --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

### 3. Host側の起動

```python
from src.host import TrackingPipeline, TrackingVisualizer

# パイプライン作成
pipeline = TrackingPipeline(
    udp_port=5000,
    calibration_path="./calibration/",
    enable_logging=True
)

# 可視化設定
visualizer = TrackingVisualizer()
visualizer.start_session()

def on_pose(poses):
    visualizer.visualize_poses(poses, timestamp=0)

pipeline.set_pose_callback(on_pose)
pipeline.start()

# ... 実行中 ...

pipeline.stop()
```

### 4. Pi側のデプロイ

```bash
# SSH鍵の設定
ssh-keygen -t ed25519 -f ~/.ssh/loutrack_deploy_key
ssh-copy-id -i ~/.ssh/loutrack_deploy_key.pub pi@<PI_IP>

# デプロイ実行
./src/deploy/deploy.sh
```

## 依存関係

- Python 3.9+
- numpy >= 1.21.0
- scipy >= 1.7.0
- opencv-python >= 4.5.0

## 開発状況

| Phase | 内容 | 状態 |
|-------|------|------|
| Phase 0 | データ契約 + ログ/リプレイ + メトリクス | ✅ 完了 |
| Phase 1 | 幾何復元の最小縦切り | ✅ 完了 |
| Phase 2 | 同期と較正 | 🚧 進行中 |
| Phase 3 | 5剛体化 | ⏳ 未着手 |
| Phase 4 | SteamVR出力 | ⏳ 未着手 |
| Phase 5 | 運用/更新 (SCP/SSH配布) | ✅ 完了 |

## TODO (Current Sprint)

- [x] Charuco 印刷ボードの実測値を確定（5マス=150mm → 1マス=30.0mm）
- [x] Charuco 較正メモを追記（`LensPosition=5.2` が現時点ベスト）
- [x] `docs/next_steps_charuco.md` を実機ゼロベース手順へ更新（sim前提を除去）
- [x] `docs/next_steps_charuco.md` Step 1 に `SPACE`/`C` の詳細手順を追記
- [x] `docs/next_steps_charuco.md` に `picamera2 is required` エラー対処（venv再作成手順）を追記
- [x] `docs/next_steps_charuco.md` に「構図を被らせない撮影ルール」を追記
- [x] wand 外部較正の実装計画を追加（`docs/wand_extrinsics_plan.md`）
- [x] `docs/wand_extrinsics_plan.md` にコードロジック詳細（状態遷移、mask生成、solver手順）を追記
- [x] `docs/wand_extrinsics_plan.md` を更新（受動発見優先、映像なし同期スライダーGUI方針を明記）
- [x] Host wand control GUI/operation orchestration documented (`docs/wand_extrinsics_plan.md`)
- [x] `WAND_POINTS_MM` を確定（center-to-center: 168mm / 243mm, blob 14mm, 直交）
- [x] wand 次アクション手順書を追加（`docs/next_steps_wand_runbook.md`）
- [x] `docs/wand_extrinsics_plan.md` に進捗状況セクションを追記（2026-03-08）
- [x] 5剛体向け marker 形状の探索結果を整理（`docs/rigid_body_design_exploration.md`）
- [x] 剛体デザインを「中心(0,0,7)の半径65mm半球、z>7」制約で再探索し更新（`docs/rigid_body_design_exploration.md`）
- [x] `waist` は既存デザインを維持する方針で `src/host/rigid.py` と関連docを更新
- [x] 剛体 mount の STL 自動生成方針は撤回し、関連スクリプト/生成物/README記述を整理
- [ ] `pi-cam-01` の内部較正を実測値ベースで再実施する（まずこの1台で手順検証）
- [ ] `pi-cam-02` の内部較正を実施し、成果物 JSON を保存する
- [ ] 内部較正の品質判定基準を「暫定」から「運用基準」に確定する
- [x] Host 主導の wand 収録制御（`mask_start` / `start` / `stop`）を `src/host/wand_session.py` で実装
- [ ] 受動発見（`UDPReceiver`）+ `hosts.ini` 併用のカメラ確定ロジックを実装する
- [x] 映像なし Web GUI（同期スライダー: exposure/gain/fps）を実装する
- [x] `src/camera-calibration/calibrate_extrinsics.py` を wand 入力対応で実装する
- [x] Pi 側 capture サービスに `mask_start`/`mask_stop` + `start(mode=wand_capture)` を実装
- [x] Host 制御クライアントに `mask_start` / `mask_stop` API・CLIを追加
- [x] `tests/test_pi_control_e2e.py` に wand マスクフロー E2E を追加
- [x] 外部較正スキーマ（`calibration_extrinsics_v1`）を `schema/` に追加する
- [x] Host 側で intrinsics + extrinsics を読み込んだ三角測量検証を実施する
- [ ] 上記進捗に合わせて Phase 表とドキュメントを同期更新する

### 直近の実装更新（2026-03-08）

- `src/camera-calibration/calibrate_extrinsics.py` を追加し、wand JSONL ログ + intrinsics から `calibration/calibration_extrinsics_v1.json` を生成できるようにした
- `schema/calibration_extrinsics_v1.json` を追加し、`schema/README.md` に出力形式を追記した
- `src/host/geo.py` が calibration ディレクトリから extrinsics を自動ロードできるようにした
- `src/host/wand_session.py` が session metadata JSON を `logs/wand_sessions/` に永続化するようにした
- `src/host/wand_gui.py` を追加し、受動発見 + inventory マージ前提の wand Web UI を実装した
- `tests/test_wand_extrinsics.py` を追加し、synthetic wand 観測から solver と geo loader の接続を検証した
- `tests/test_wand_gui.py` を追加し、GUI 状態管理の同期操作を smoke test で検証した
- `docs/wand_extrinsics_plan.md` を現実装ベースの進捗・残課題へ更新した
- `docs/next_steps_wand_runbook.md` を現行実装フロー準拠の作業手順書として新規作成した
- `docs/next_steps_wand_runbook.md` の前提条件をコマンドベースの事前チェック手順として詳細化した
- Host から `set_focus` を追加し、`wand_gui` で focus スライダー（既定 `5.215`）を同期操作できるようにした
- `wand_gui` に「Generate Extrinsics」ボタンを追加し、GUI から `calibrate_extrinsics` を実行できるようにした
- blob 検出条件として `set_threshold` / `set_blob_diameter(min/max px)` を追加し、GUI から同期調整できるようにした
- `wand_gui` のスライダーを 200ms デバウンスの自動適用に変更し、未適用値が定期ポーリングで巻き戻る挙動を解消した
- `src/pi/capture.py` に `--debug-preview` を追加し、debug 時のみ Pi 上で OpenCV プレビューを描画できるようにした
- `docs/next_steps_wand_runbook.md` に Pi 側 `--debug-preview` 起動手順と確認できる内容を追記した
- `src/pi/capture.py` の debug preview を idle / mask 初期化 / wand 収録の各状態で継続表示するようにし、Pi デスクトップから blob 調整・mask 結果・wand 操作を遠隔確認しやすくした
- `src/pi/capture.py` が `DISPLAY` 未設定の SSH / headless 実行時には debug preview を起動前に無効化するようにし、Qt/xcb でプロセスが落ちる問題を避けるようにした
- `src/pi/capture.py` の既定解像度を `2304x1296` に更新し、`docs/next_steps_charuco.md` の live 較正前提と揃えた
- Pi capture / wand GUI / wand session の既定 FPS を `56` に揃えた
- wand GUI / wand session の既定 `exposure_us` / `gain` を暗すぎない初期値（`12000` / `8.0`）に更新した
- `src/pi/capture.py` が起動時と capture start/stop 時に backend 状態を標準出力へ出すようにした
- `src/pi/capture.py` の既定 backend を Raspberry Pi 上では `picamera2` にし、docs と実装のズレを解消した（非 Pi 環境では `dummy` を維持）
- `AGENTS.md` に、近い内容の重複テストや一時的なテスト残骸を同一変更内で整理・削除するルールを追加した
- `mask_start` を既存 mask の上書き更新として扱えるようにし、Pi preview の mask 領域も薄い赤オーバーレイ + 輪郭で見やすくした
- `mask_start` の host 側 timeout を 30 秒へ延長し、Pi 側でも preview 停止失敗時の早期エラーと mask 初期化 timeout 復旧を追加して `MASK_INIT` 張り付きが残らないようにした
- Pi 側 `mask_start` は debug preview 用 backend をそのまま handoff して再利用するようにし、preview stop 後の `picamera2` 再 open race を避けた
- Pi 側 `mask_start` の terminal debug log を増やし、preview handoff / backend source / frame capture / postprocess のどこで止まるかを切り分けやすくした
- `MASK_INIT` 中は preview 更新と blob 検出を行わず、閾値超え画素の蓄積だけにして、完了後に mask overlay を出す形へ簡略化した
- `mask_start` 完了直後の同期 preview 描画も外し、preview 再開は idle loop 側に一本化した
- Pi 側 debug preview は `mask_start` 前後でも同一 `DebugPreview` / 同一 HighGUI window を維持し、`destroyWindow` による close/reopen を shutdown 時以外やめた
- `MASK_INIT` 中は HighGUI/Qt を別スレッドから触らないため、debug preview window は開いたまま最後の描画フレームで freeze させ、再描画は mask 完了後の idle loop 再開まで行わないようにした
- `ping` の `debug_preview_active` は preview thread / emitter だけでなく、保持中の debug window 表示状態も含めて返すようにした
- `wand_gui` の Blob Detection Adjustment 設定を host ローカル（`logs/wand_gui_settings.json`）へ保存・再読込するようにした
- Pi idle preview は起動時に 1 回だけ立ち上げ、`ping` ごとの `picamera2` 再取得ループをやめるようにした
- `src/host/wand_gui.py` を `Blob Detection Adjustment -> Mask Adjustment -> Wand Capture -> Extrinsics Generation` の 4 セグメント UI に再編し、段階的に進めやすい workflow 表示へ更新した
- `src/host/wand_gui.py` の console パネルが長い JSON/パスで他要素に被らないよう、右カラム縮小・overflow・折り返しを調整した
- `src/host/wand_gui.py` で blob diameter の `min > max` を送らないようフロント側で自動正規化するようにした
- `docs/next_steps_wand_runbook.md` を venv 前提（`python3 -m venv .venv --system-site-packages`）へ更新し、Host 手順の `python` 実行を統一した
- `docs/next_steps_wand_runbook.md` に Pi 側 venv 準備・`hostnamectl --static` による `camera_id` 設定・capture サービス起動手順を追記した（port は既定 8554 前提）
- `src/pi/capture.py` の `camera_id` 既定値を device name 由来に変更し、`--camera-id` 未指定時は `hostnamectl --static`（失敗時は `socket.gethostname()`）を使用するようにした
- `docs/next_steps_wand_runbook.md` を更新し、Pi 側起動手順を `python src/pi/capture.py`（`camera_id` は device name 既定）に合わせた

## ドキュメントの読み順

1. `README.md`（現況 / TODO / 実行入口）
2. `docs/implementation_plan.md`（フェーズ計画と短期実装順）
3. `docs/requirements_def.md`（要件定義）
4. `docs/pi_control_transport.md`（Pi 制御通信仕様）
5. `schema/README.md`（JSON Schema の参照）
6. `docs/next_steps_charuco.md`（内部較正の実行手順。Step 0 にまっさらな Pi 初期セットアップを記載）
7. `docs/wand_extrinsics_plan.md`（wand 外部較正の実装計画）
8. `docs/next_steps_wand_runbook.md`（wand実装の実行手順）
9. `docs/rigid_body_design_exploration.md`（5剛体 marker 設計の探索結果）

補足: `docs/pre_doc/` は検討メモ・背景資料であり、現行仕様の正本ではありません。

### 完了したStep

- ✅ Step 1: Pi→Host メッセージスキーマ凍結
- ✅ Step 2: ログ/リプレイ/メトリクス基盤実装
- ✅ Step 3: 較正成果物 schema 固定
- ✅ Step 4: SCP/SSH 配布スケルトン作成
- ✅ Step 5: Host受信バッファ + ペアリング実装
- ✅ Step 6: 幾何復元モジュール実装
- ✅ Step 7: 1剛体推定と可視化
- ✅ Step 8: 同期評価ツール実装 (許容窓スイープ + 欠損挙動)

## データフロー

1. **Pi側**: カメラで反射マーカーを検出 → blob座標をUDP/JSONで送信
2. **Host側**: 
   - UDP受信 → timestamp/frame_indexでペアリング
   - 同期評価 → pair spread / missing gap / 推奨許容窓
   - DLT三角測量 → 3D点群
   - DBSCANクラスタリング → 剛体候補
   - Kabschアルゴリズム → 剛体姿勢

## ライセンス

MIT License

## 参考プロジェクト

- [Low-Cost-Mocap](https://github.com/jyjblrd/Low-Cost-Mocap) - jyjblrd氏の参考実装
