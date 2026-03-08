# requirements_def

## 1. 目的

Raspberry Pi 5 + Raspberry Pi Camera 3 Noir Wide 複数台で、
頭・胸・腰・右足先・左足先の5剛体を 60fps 目標で追跡し、SteamVR にトラッカーとして入力する。

## 2. スコープ

### In Scope

- RasPi 側: Blob 検出と UDP JSON 送信
- Host 側: 受信、ペアリング、3D復元、剛体推定、SteamVR出力
- Host→RasPi: start/stop、カメラ設定、マスク、LED制御
- 較正(内部/外部)、同期、運用更新基盤

### Out of Scope (初期)

- 外部ハードトリガ前提の同期
- 大規模クラスタ(9台以上)の本番運用

## 3. 機能要件

### FR-1 RasPi観測

- 各 RasPi はフレーム毎に `camera_id`, `timestamp`, `frame_index`, `blobs[]` を送信する
- `timestamp` は `int64` の Unix epoch(us) を採用する
- `frame_index` は `uint32` の単調増加(カメラごと)を採用する
- `blobs` 要素は `{x, y, area}` を持つ

### FR-2 Host受信/ペアリング

- Host は UDP 受信し、camera_id 毎に時系列を管理
- `timestamp` を主、`frame_index` を副として multi-camera フレームをペアリング

### FR-3 幾何復元

- Host は較正値を用いて対応付け・三角測量を行い 3D 点群を生成
- 再投影誤差を計算し品質指標を出す

### FR-4 剛体推定

- 点群をクラスタリングして剛体候補を分離
- 各剛体について姿勢(quaternion)を推定

### FR-5 SteamVR出力

- 5トラッカー(head/chest/waist/left_foot/right_foot)を更新出力

### FR-6 Host→RasPi制御

- start/stop
- カメラ設定(露光、ゲイン、fps 等)
- マスク作成指示
- GPIO LED 制御

## 4. 非機能要件

### NFR-1 性能

- 目標更新レート: 60fps
- 同期目標: 2-5ms

### NFR-2 頑健性

- 一部カメラ欠損時でも推定継続(グレースフルデグレード)
- UDP パケット欠落時でも復帰可能

### NFR-3 保守性

- 2-8台の RasPi を更新/運用できること
- 較正データと設定をバージョン管理できること

## 5. インタフェース要件

### IR-1 RasPi→Host (UDP/JSON)

- 受信口: `0.0.0.0:5000` (想定)
- 文字コード: UTF-8

### IR-2 Host→RasPi (TCP/JSON)

- 制御系は到達保証を重視
- message schema は `request_id`, `camera_id`, `cmd`, `params`, `ack`, `error_code` を必須とする

### IR-3 Host→SteamVR

- OpenVR Driver を第一候補
- tracker role は `head`, `chest`, `waist`, `left_foot`, `right_foot` を固定する
- serial 命名規則は `loutrack-{role}-{index}` を採用する

## 6. 受け入れ基準(初版)

1. 2台以上のカメラ観測から 3D 点群が再現される
2. 5トラッカーの pose が連続出力される
3. 同期誤差評価で目標帯域(2-5ms)に近づくことを確認できる
4. start/stop と露光制御が Host から反映される

## 7. 推奨決定(暫定採用)

- `frame_index` は送る(障害時フォールバック用)
- `timestamp` は `int64` Unix epoch(us)、フレーム確定時刻を採用
- Host→Pi は TCP JSON-RPC 風 schema を採用
- SteamVR は OpenVR Driver、role/serial を固定
- 同期失敗時は「補間継続 → 連続失敗で予測のみ → 自動再同期」の2段階 degrade
- Charuco 較正成果物は `calibration_intrinsics_v1.json` を採用
- 外部較正は wand 運用手順(>=20ポーズ、誤差基準あり)を標準化
- DBSCAN 初期値は `eps=marker_diameter*0.8`, `min_samples=3`
- 姿勢推定は Kabsch 主体、PnP/3点PnP/Kalman を段階フォールバック
- 配布は SCP/SSH で開始し、台数>4 または更新頻度>週2で Ansible へ移行

## 8. なお未確定の要件

- 再投影誤差やフォールバック発火の閾値(実測で確定)
- 環境別(DBSCAN/同期窓)のチューニングテーブル
- OpenVR driver 実装時の予測/補間責務の最終配分
