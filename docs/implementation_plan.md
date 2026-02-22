# implementation_plan

## 0. 方針

最小の縦切りを先に完成させ、計測可能な状態で改善を回します。

## 0.1 着手前に採用する推奨決定

- `timestamp`: `int64` Unix epoch(us)、フレーム確定時刻
- `frame_index`: `uint32` 単調増加(カメラごと)
- Host→Pi 制御: TCP JSON-RPC 風 schema (`request_id`, `camera_id`, `cmd`, `params`, `ack`, `error_code`)
- SteamVR 出力: OpenVR Driver + 固定 role(`head/chest/waist/left_foot/right_foot`)
- role 用 serial: `loutrack-{role}-{index}`
- 同期 degrade: 補間継続 → 連続失敗で予測のみ → 自動再同期
- DBSCAN 初期値: `eps=marker_diameter*0.8`, `min_samples=3`

## 0.2 実行順の確定(2026-02-22)

- 最優先は Phase 0(データ契約 + ログ/リプレイ + メトリクス)とする
- 内部較正と配布は「前倒しするが薄く始める」を採用する
  - 内部較正: まず成果物 schema(`calibration_intrinsics_v1.json`)と Host 読み込み前提を固定
  - 配布: まず SCP/SSH(or rsync over SSH) の運用スケルトン(インベントリ、鍵、転送、再起動、ロールバック手順)を固定
- 本格的な較正実行スクリプトと運用配布自動化は、Phase 0 の観測基盤が揃った後に進める

## 1. フェーズ計画

### Phase 0: データ契約と検証基盤

- Pi→Host/Host→Pi/Host→SteamVR の message schema を固定
- ログ保存とリプレイ機能を作成
- 観測メトリクス(fps、遅延、再投影誤差)を可視化
- 受信ペアリングは `timestamp` 主、`frame_index` 副で実装
- カメラ/Pi 実機が無くても動く入力(ダミー/ログ再生)を先に整備
- 出力はまずローカル可視化/ログ出力で閉ループ確認し、SteamVRは後段で差し替える

**完了条件**
- 同一ログを再生して同一出力を再現できる

### Phase 1: 幾何復元の最小縦切り

- 対応付け + 三角測量 + 再投影誤差計算
- 1剛体(例: waist)で安定追跡

**完了条件**
- 1剛体の位置が安定して時系列出力される

### Phase 2: 同期と較正

- PTP導入と timestamp ペアリング最適化
- Charuco 内部較正 + 外部較正ワークフロー整備

**完了条件**
- 同期評価が 2-5ms 目標レンジに近づく
- 較正データが再利用可能な形式で保存される

### Phase 3: 剛体推定強化

- DBSCAN ベースの候補分離
- Kabsch 主体 + 条件付きフォールバック(PnP/3点PnP/Kalman)

**完了条件**
- 5剛体を識別し、姿勢が連続推定できる

### Phase 4: SteamVR 統合

- OpenVR Driver 連携
- role/serial の固定化
- 座標変換と原点合わせ

**完了条件**
- head/chest/waist/left_foot/right_foot の5トラッカーが SteamVR で認識される

### Phase 5: 運用/更新

- SCP/SSH ベース配布の初版
- 台数増加時の自動化移行判断(Ansible等)

**完了条件**
- 2-8台で更新作業が運用可能な手順に落ちる

## 2. 実装順序(短期タスク)

1. message schema 草案確定
2. ログ保存/リプレイ/メトリクス基盤を実装
3. 較正成果物 schema(`calibration_intrinsics_v1.json`)を固定
4. SCP/SSH ベース配布の運用スケルトンを作成(インベントリ、転送、再起動、ロールバック)
5. Host 受信バッファ + ペアリング実装
6. 幾何復元モジュール(対応付け/三角測量/誤差)
7. 1剛体推定と可視化
8. 同期評価ツール実装
9. 較正スクリプト実装(Charuco)
10. 5剛体化 + SteamVR 出力

## 2.1 参考実装の取り込み方

- 対応付け/三角測量/誤差評価: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`
- キャリブレーションUI/イベント構成: `references/jyjblrd/Low-Cost-Mocap/computer_code/src/App.tsx`
- Host 側イベント入口の設計: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`

## 3. リスクと先行対応

- **同期不達**: 許容窓と予測補間を導入
- **誤対応付け**: 再投影誤差閾値と外れ値除去を強化
- **遮蔽**: トラック管理 + フォールバックで継続
- **運用負荷**: 設定テンプレート化と更新手順の標準化

## 4. 未解決の決定(この計画で先に確定するもの)

- 再投影誤差の閾値
- フォールバック発火条件(観測カメラ数、可視点数、連続失敗回数)
- 同期許容窓(実測で環境別に最適化)
- DBSCAN の環境別補正テーブル
- 較正合格基準(RMS誤差と撮影ポーズ品質)

## 5. 直近マイルストーン

- M0: 推奨決定の採用と schema 凍結
- M1: データ契約 v0.1 確定
- M2: 1剛体の end-to-end 追跡成功
- M3: 5剛体 + SteamVR出力初版
- M4: 2-8台運用手順確立
