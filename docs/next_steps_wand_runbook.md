# next_steps_wand_runbook

この手順書は、現状の実装（Pi側 `mask_start`/`mask_stop` + Host側 `wand_session.py`）を前提に、
次に何をどう進めるかを運用順でまとめた実行ガイドです。

## 0. 前提（固定値）

- wand: `wand_l_b5_v1`
- blob径: 14mm
- center-to-center 定義（mm）
  - `P0=(0, 0, 0)`
  - `P1=(168, 0, 0)`
  - `P2=(0, 243, 0)`
- 制御シーケンス: `ping -> set_exposure/set_gain/set_fps -> mask_start -> start(mode=wand_capture) -> stop`

## 1. まずやること（最短）

1. `src/host/wand_session.py` に session metadata 保存を追加する
2. `src/host/wand_gui.py` を作成し、映像なしGUIを追加する
3. `src/camera-calibration/calibrate_extrinsics.py` の最小版を実装する

理由: 収録と再現性を先に固めることで、solverのデバッグが安定するため。

## 2. Session metadata 保存（必須）

`WandSession.run_session()` の返却内容を、そのまま JSON として保存する。

- 保存先例: `calibration/sessions/<session_id>.json`
- 最低限含めるキー
  - `session_id`
  - `wand`（`name`, `marker_diameter_mm`, `points_mm`）
  - `targets`
  - `ack_history`
  - `started_at`, `ended_at`
  - `config`（`exposure_us`, `gain`, `fps`, `duration_s`, `mask_params`）

チェック:

- `mask_start` の `mask_ratio` が異常に高くない（目安: 0.40以下）
- 1台でも `start` が失敗したら全台 `stop` 済みで終了する

## 3. GUI 実装（映像なし）

新規: `src/host/wand_gui.py`

最小機能:

- スライダー: `exposure_us`, `gain`, `fps`
- ボタン: `ping`, `mask_start`, `start`, `stop`
- テーブル: `camera_id`, `ip`, `last_ack`, `last_error`, `is_running`

実装ポイント:

- スライダー送信は 200ms デバウンス
- 最終値のみ送信（連打時）
- 送信結果を camera 単位で表示
- 受動発見 (`UDPReceiver`) + `hosts.ini` の併用を維持

## 4. wand収録の運用手順（現場向け）

1. 全Pi起動、HostでUDP受信開始
2. `ping` で対象カメラ健全性確認
3. `set_exposure` / `set_gain` / `set_fps` を全台へ同期配布
4. `mask_start` 実行（30フレーム）
5. `start(mode=wand_capture)`
6. wandを空間全体で20+ポーズ（推奨60秒）
7. `stop`
8. session metadata とログを保存

失敗時:

- `mask_start` 失敗: 1回だけ再試行、継続失敗なら中止
- `start` 失敗: 即 `stop` を全台へ送信して中止

## 5. Extrinsics solver 実装順

新規: `src/camera-calibration/calibrate_extrinsics.py`

実装順:

1. 入力ローダー（intrinsics JSON + session/log）
2. 同時刻ペアリング（`timestamp`主、`frame_index`副）
3. 2台最小版の `findEssentialMat` + `recoverPose`
4. wand辺長拘束でスケール決定
5. 再投影誤差を出して `calibration_extrinsics_v1.json` を保存

品質ゲート（初版）:

- `inlier_ratio`
- `median_reproj_error_px`
- `baseline_m`
- 有効フレーム採用率

## 6. 受け入れ判定（この手順書のDone条件）

- `wand_session` がメタデータ保存まで自動実行できる
- GUIから全操作（ping/mask/start/stop/slider同期）ができる
- 2台で extrinsics JSON 生成が再現する
- README と docs が実装内容に追従して更新済み
