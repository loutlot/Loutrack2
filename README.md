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
├── deploy/                    # Piデプロイスクリプト
│   ├── hosts.ini              # Piインベントリ
│   ├── deploy.sh              # デプロイスクリプト
│   └── rollback.sh            # ロールバックスクリプト
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
# Charucoボードを使用して較正画像を撮影
# 較正スクリプトを実行（Step 9で実装予定）
python scripts/calibrate.py --camera pi-cam-01
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
./deploy/deploy.sh
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