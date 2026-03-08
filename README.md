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
ssh-copy-id -i ~/.ssh/loutrack_deploy_key.pub pi@192.168.1.101

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
- [x] 5剛体向け marker 形状の探索結果を整理（`docs/rigid_body_design_exploration.md`）
- [x] 剛体デザインを「中心(0,0,7)の半径65mm半球、z>7」制約で再探索し更新（`docs/rigid_body_design_exploration.md`）
- [x] `waist` は既存デザインを維持する方針で `src/host/rigid.py` と関連docを更新
- [x] 剛体 mount の STL 自動生成方針は撤回し、関連スクリプト/生成物/README記述を整理
- [ ] `pi-cam-01` の内部較正を実測値ベースで再実施する（まずこの1台で手順検証）
- [ ] `pi-cam-02` の内部較正を実施し、成果物 JSON を保存する
- [ ] 内部較正の品質判定基準を「暫定」から「運用基準」に確定する
- [x] Host 主導の wand 収録制御（`mask_start` / `start` / `stop`）を `src/host/wand_session.py` で実装
- [ ] 受動発見（`UDPReceiver`）+ `hosts.ini` 併用のカメラ確定ロジックを実装する
- [ ] 映像なし Web GUI（同期スライダー: exposure/gain/fps）を実装する
- [ ] `src/camera-calibration/calibrate_extrinsics.py` を wand 入力対応で実装する
- [x] Pi 側 capture サービスに `mask_start`/`mask_stop` + `start(mode=wand_capture)` を実装
- [x] Host 制御クライアントに `mask_start` / `mask_stop` API・CLIを追加
- [x] `tests/test_pi_control_e2e.py` に wand マスクフロー E2E を追加
- [ ] 外部較正スキーマ（`calibration_extrinsics_v1`）を `schema/` に追加する
- [ ] Host 側で intrinsics + extrinsics を読み込んだ三角測量検証を実施する
- [ ] 上記進捗に合わせて Phase 表とドキュメントを同期更新する

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
