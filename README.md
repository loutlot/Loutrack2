# Loutrack2

Loutrack2 は、複数 Raspberry Pi カメラで反射マーカーを観測し、Host 側で 3D 復元とトラッキング表示まで行うモーションキャプチャ実験リポジトリです。

現時点で到達している範囲:

- Charuco を使った各カメラの内部較正
- pose capture + wand metric capture からの外部較正生成
- Host GUI からの blob 調整 / mask 構築 / pose capture / wand metric capture
- 外部較正後の tracking runtime 起動と JSON ベースの表示確認

この README は、初見の人が「何ができていて」「どの順で進めれば tracking 表示まで到達できるか」を把握するための入口です。

## 1. 全体像

Loutrack2 の流れは次の 4 段階です。

1. `src/pi/capture.py` を各 Raspberry Pi で起動して、反射マーカー blob を UDP 送信できる状態にする
2. `src/camera-calibration/calibrate.py` で各カメラの intrinsics を作る
3. `src/host/wand_gui.py` で pose capture と wand metric capture を収録し、`calibration/calibration_extrinsics_v1.json` を作る
4. 同じ GUI から tracking を起動して、scene snapshot JSON で復元結果を確認する

ざっくり言うと:

- Pi は「2D blob を送る係」
- Host は「較正・同期・三角測量・姿勢推定・表示を担う係」

## 2. リポジトリ構成

- [`/src/pi`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi): Raspberry Pi 側の capture サービス
- [`/src/host`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host): Host 側の受信、GUI、tracking runtime
- [`/src/camera-calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration): Charuco 内部較正と wand 外部較正
- [`/calibration`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/calibration): 較正 JSON とボード出力
- [`/docs`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs): 設計メモ、進行中 runbook、完了済み runbook
- [`/tests`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests): 回帰テスト

## 3. 前提

必要なもの:

- Host マシン 1 台
- Raspberry Pi カメラ 2 台以上
- 各 Pi で `src/pi/capture.py` が動くこと
- Host と Pi が同一ネットワークで通信できること
- Charuco ボード
- 4 点 wand（長辺の中点マーカーを含む）

Python 依存はルートの `requirements.txt` で入ります。

```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

## 4. 最短で tracking 表示まで行く手順

### Step 1. 各 Pi で capture を起動

Pi ごとに `camera_id` を分けて起動します。

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest <HOST_IP>:5000 \
  --debug-preview
```

別の Pi では `pi-cam-02` のように変えます。

ポイント:

- 制御ポートは既定で `8554`
- UDP は既定で `5000`
- `--debug-preview` は通常運用で ON にして、blob 調整や mask 構築の見え方を Pi 側画面で確認します

Pi 側の詳細は [`/src/pi/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/README.md) を参照してください。

### Step 2. 各カメラの内部較正を作る

まず Charuco ボードを準備し、各カメラごとに intrinsics を作ります。

ボード生成:

```bash
python src/camera-calibration/charuco_board.py \
  --output "./calibration/boards/charuco_6x8_30mm_a4.pdf" \
  --write-metadata
```

内部較正:

```bash
python src/camera-calibration/calibrate.py \
  --camera pi-cam-01 \
  --square-length-mm <実測値> \
  --output "./calibration/calibration_intrinsics_v1_pi-cam-01.json"
```

最低限そろえるファイル:

- `calibration/calibration_intrinsics_v1_pi-cam-01.json`
- `calibration/calibration_intrinsics_v1_pi-cam-02.json`

内部較正の詳細は [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md) を参照してください。

### Step 3. Host GUI を起動

Host で次を実行します。

```bash
python src/host/wand_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで以下を開きます。

- `http://<HOST_IP>:8765/`
- `0.0.0.0` は Host 側の待受アドレスです。Pi 側 `--udp-dest` には `0.0.0.0:5000` ではなく、必ず Host の実 IP (`<HOST_IP>:5000`) を指定します。

GUI は calibration ページと tracking ページを持っています。

### Step 4. GUI で外部較正を作る

GUI 上では、基本的に次の順で進めます。

1. `Refresh` でカメラ一覧を更新
2. 対象カメラを選択
3. `Ping` で `ack=true` を確認
4. `Blob Detection Adjustment` で threshold / circularity / blob diameter / exposure などを調整
5. `Mask Adjustment` で `Build Mask`
6. `Wand Capture` セクションで `Start Pose Capture`
7. 単一点ターゲットを空間全体で動かして pose log を収録
8. 必要なら `Start Wand Metric Capture` で床置き wand の短時間ログを収録
9. `Generate Extrinsics` で similarity extrinsics を生成
10. `Apply Wand Scale/Floor` で床置き wand から scale / floor / world origin / yaw を適用

既定の重要パス:

- pose 収録ログ: `logs/extrinsics_pose_capture.jsonl`
- wand metric 収録ログ: `logs/extrinsics_wand_metric.jsonl`
- GUI 設定: `logs/wand_gui_settings.json`
- 外部較正出力: `calibration/calibration_extrinsics_v1.json`

外部較正を CLI で直接作る場合:

```bash
python src/camera-calibration/calibrate_extrinsics.py \
  --intrinsics calibration \
  --pose-log logs/extrinsics_pose_capture.jsonl \
  --wand-metric-log logs/extrinsics_wand_metric.jsonl \
  --output calibration/calibration_extrinsics_v1.json
```

`Generate Extrinsics` / CLI の similarity solve は、既定で BA 入力を `80` サンプルへ等間隔間引きします。長い pose capture でも GUI が固まりにくいようにするためです。`Apply Wand Scale/Floor` は既定で、時刻整合した wand multiview sample の中央 `1` 件だけを使います。

### Step 5. tracking を起動して表示確認

外部較正が生成済みなら、GUI の tracking ページで `Start Tracking` を押します。

ここで確認できるもの:

- tracking status JSON
- scene snapshot JSON
- `frames_processed`
- `poses_estimated`
- カメラ情報と raw points
- 直感的に監視できる tracking confirmation ページ

Tracking タブには組み込みの `three.js` ビューアを使った 3D canvas があり、
カメラの frustum、rigid body（軸・マーカー・トレイル）と raw points をリアルタイムで描画します。
右側には per-camera health card（FPS / latency / blob 平均）と rigids サマリーを並べ、
`Start`/`Stop` ボタン越しにトラッキング品質を GUI 上だけで判断できます。
このビューアの静的アセットは repo 直下の `static/vendor/three.module.min.js` を HTTP 配信して読み込みます。

今の実装は「tracking runtime が立ち上がり、scene snapshot を返せる」段階です。初見向けにはまずこの JSON が更新されるところまで到達できれば十分です。

## 5. いま理解しておくべきファイル

最初に読む対象を絞るならこの 5 つで十分です。

- [`/src/pi/capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/capture.py): Pi 側の 2D blob 検出と送信
- [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py): 現在の運用入口
- [`/src/host/tracking_runtime.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/tracking_runtime.py): tracking の起動と scene snapshot
- [`/src/camera-calibration/calibrate.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate.py): intrinsics 作成
- [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py): wand ログから extrinsics 作成

## 6. 生成される成果物

実行すると主に次が増えます。

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `calibration/calibration_extrinsics_v1.json`
- `logs/extrinsics_pose_capture.jsonl`
- `logs/extrinsics_wand_metric.jsonl`
- `logs/wand_gui_settings.json`

README を読む側は、まずこの 4 つが何者かを把握すると迷いにくいです。

## 7. 詰まりやすい点

- `Refresh` してもカメラが出ない
  - Pi 側の `capture.py` が起動しているか
  - `--udp-dest` やネットワーク経路が合っているか
  - GUI は `Refresh` 実行時に「現在のチェックなし (`camera_ids=[]`)」を受け取るとカメラ絞り込みを解除し、全発見カメラを再表示する（古い選択IDで空一覧に固定されない）
- `Ping` が通らない
  - Host から Pi の `8554/tcp` に届いているか
  - `camera_id` と実機の対応が崩れていないか
  - `src/deploy/hosts.ini` の IP と `camera_id` が、Pi 側 `capture.py --camera-id ...` の実起動値と一致しているか
  - `ping` が通っても GUI に反映されない場合は、まず `Refresh` を押してから対象カメラのチェック状態を確認する
  - `Refresh` / `Ping` を押したとき、GUI サーバーのターミナルに `[wand_gui] POST /api/command ...` が出るかを確認する（出ない場合はブラウザ側の実行エラーを疑う）
  - Host からの疎通確認例:
    - `ping 192.168.8.223`
    - `nc -vz 192.168.8.223 8554`
- ボタンを押しても無反応に見える
  - `Last Result` に `load_state: ...` / `command_ping: ...` 形式のエラーが出ていないか確認する
  - WebGL 非対応環境では tracking 3D viewer を自動で無効化し、calibration 操作は継続できる（`3D viewer unavailable...` 表示）
  - `three.module.min.js` の import に失敗しても、GUI は動作継続する（tracking viewerのみ無効化される）
- `set_exposure` / `set_focus` を送っても Pi preview が変わらない
  - Pi 側 `picamera2` backend は manual exposure 時に `AeEnable=false`、manual focus 時に `AfMode=Manual` が必要
  - このリポジトリではその manual 制御を有効化済みなので、古いプロセスが残っている場合は Pi 側 `capture.py` を再起動する
- extrinsics が生成できない
  - `calibration/` に対象カメラ分の intrinsics があるか
- `logs/extrinsics_pose_capture.jsonl` に複数カメラの単点観測が入っているか
- tracking が始まらない
  - `calibration/calibration_extrinsics_v1.json` ができているか

### 直近の実装アップデート（2026-03）

- `src/camera-calibration/calibrate_extrinsics.py` の wand 対応を強化し、4 点 wand（`[elbow, short, mid, long]`）の対応順抽出を追加
- ペアリング既定窓を `8000us` に変更し、`session_meta.pair_window_us` で既定値/上書き値を記録
- extrinsics 品質指標の `point_count` を固定値ではなく、実際に使用した対応点数で集計
- 4 点 wand 運用時は 3 点フレームを自動除外するガードを追加（3 点混入による index error を防止）
- `WAND_POINTS_MM` の順序を solver の canonical order（`elbow, short, mid, long`）へ統一
- `docs/10_in_progress/wand_4point_extrinsics_plan.md` を更新し、`references/jyjblrd` を踏まえた multiview BA ベースの extrinsics 再設計計画を追加
- `src/camera-calibration/calibrate_extrinsics.py` を multiview 方式へ更新し、ラベル付け・sample化・初期姿勢・joint BA を統合
- 新規 `src/camera-calibration/wand_label.py`（4点ラベリング）, `wand_samples.py`（multiview sample 構築）, `wand_bundle_adjustment.py`（joint BA）を追加
- `tests/test_wand_extrinsics.py` / `tests/test_wand_gui.py` の回帰を通る状態で、GUI の既存 API 契約（`pair_window_us`, `min_pairs`）は維持
- `min_pairs` を camera ごとの有効 sample 数にも適用し、不足 camera は `session_meta.excluded_camera_*` へ理由つきで除外して処理継続するよう修正
- joint BA で camera 並進も最適化対象へ戻しつつ、初期値からの変化量に境界を入れてスケール崩壊を防止
- `docs/10_in_progress/gui_runbook.md` を更新し、`Min Pairs` の適用範囲、除外 camera の確認手順、`session_meta` の運用確認項目を追加
- `Generate Extrinsics` の長時間停止を避けるため、BA 入力 sample を既定 `320` に均等間引きする `max_ba_samples` を solver に追加
- `wand_gui` で `intrinsics/log/output` の相対パスをプロジェクトルート基準の絶対パスへ正規化し、結果ファイルの出力先ずれを防止
- 4点対応付け確認のため、ログ1ペアを `logs/debug_pair_labeling.png` として可視化するデバッグ手順を実行確認
- 実測に合わせて 4点wand の幾何前提を更新し、wand 座標を `[elbow, short, mid, long] = [(0,0,0), (0,182,0), (132,0,0), (257,0,0)]` へ修正
- `short` は elbow から `+Y`、`elbow-mid-long` は `+X` 上に並ぶ前提へ統一し、solver / validation / test の wand 定義を揃えた
- `calibrate_extrinsics.py` のラベリング段に temporal consistency ゲート（前フレームからのラベル急変検出）を追加し、`session_meta.dropped_frame_counts_by_reason.temporal_jump` を記録
- `calibrate_extrinsics.py` に per-frame `confidence` スコア（linearity / midpoint / temporal）を追加し、`min_label_confidence` 未満を除外可能にした（`session_meta.label_confidence_stats` を出力）
- 時系列等間隔抽出（80サンプル）＋全ロジック（temporal jump / confidence gate 含む）で対応付けGIFを再生成し、`logs/debug_labeling_pair80_all_logic_uniform.gif` を更新
- 時刻ベースの等間隔抽出版として `logs/debug_labeling_pair80_all_logic_time_uniform.gif` も生成（現ログでは有効サンプルが約4.23秒に集中）
- `references/jyjblrd` ベースで extrinsics 校正を `pose_capture -> F/E 初期化 -> BA -> 床置き wand で scale/floor/validation` へ移す大規模 migration 計画を `docs/10_in_progress/jyjblrd_extrinsics_migration_plan.md` に追加
- `jyjblrd` migration 実装のレビュー指摘を修正し、BA seed triangulation の正規化、wand metric の時刻ベース multiview 対応、wand validation のフレーム再構成評価、`test_extrinsics_*` loader の collection failure を解消
- `src/camera-calibration/extrinsics_capture.py` / `extrinsics_samples.py` / `extrinsics_initializer.py` / `extrinsics_ba.py` / `extrinsics_scale.py` / `extrinsics_validate.py` を追加し、extrinsics solver を pose-capture 主体に移行
- `src/camera-calibration/calibrate_extrinsics.py` を二段化し、`solve_extrinsics()` を similarity solve 専用、`apply_wand_scale_floor()` を wand metric 適用専用に整理
- `src/camera-calibration/calibrate_extrinsics.py` の similarity solve に既定 `max_ba_samples=80` の等間隔 downsampling を追加し、長い pose capture で `Generate Extrinsics` が止まり続ける問題を回避
- `src/camera-calibration/calibrate_extrinsics.py` の wand metric 適用は既定 `max_wand_metric_samples=1` とし、床置き wand の中央 1 multiview sample だけで `Apply Wand Scale/Floor` を計算するよう変更
- `src/camera-calibration/extrinsics_initializer.py` を最大 spanning tree ベースの初期姿勢展開へ変更し、局所 BFS 依存を解消
- `src/camera-calibration/extrinsics_scale.py` で床法線合わせに加えて wand 長辺から yaw を決め、elbow を origin に置く world 座標系整列を追加
- `src/camera-calibration/extrinsics_scale.py` の scale 推定を wand 全点の similarity fit + 安定辺長比のハイブリッドへ変更し、外れ値除去を追加
- `src/camera-calibration/extrinsics_scale.py` の wand 三角測量を distortion-aware に修正し、回転不変でない scale 推定式を Umeyama 系 similarity fit に置き換え
- `wand_pnp` seed は既に metric pose なので、`apply_wand_metric_alignment()` では floor/origin/yaw のみを適用し、再スケーリングしないよう修正
- `Apply Wand Scale/Floor` は baseline 実測値に依存しない経路へ戻し、`expected_baseline_m` は solver ではなく validation 用メタデータとしてのみ扱うよう整理
- `wand_metric` の raw fallback は高信頼扱いしないようにし、multiview shape 整合で permutation を選び直すよう修正
- `wand` が成立していない candidate (`wand_metric_frames=0` や shape error 大) は `Apply Wand Scale/Floor` の成功候補として採らないよう修正
- `pose_capture` の camera shape を固定して global scale のみを known wand から合わせる scale-only BA を追加したが、現ログではまだ主候補には採用されていない
- `Apply Wand Scale/Floor` の既定 `max_wand_metric_samples` を `16` に引き上げ、GUI から wand metric のサンプル数を制御できるように更新
- `wand_metric` の 4点対応は current pose seed 上での multiview shape 整合を使って再ラベルするよう変更し、画像平面上ヒューリスティクスと measured baseline 依存を下げた
- `.venv` に `matplotlib` を追加し、[`src/host/view_extrinsics_3d.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/view_extrinsics_3d.py) で floor grid と camera frustum を対話回転できる 3D viewer を追加
- `src/camera-calibration/extrinsics_validate.py` の `floor_residual_mm` を world floor 軸に対する残差へ修正
- `src/pi/capture.py` の `pose_capture` quality から image center / temporal 距離の減点を外し、単点らしさと blob 品質だけで評価するよう整理
- `src/host/wand_gui.py` に `Apply Wand Scale/Floor` を追加し、GUI/API を `Generate Extrinsics -> Apply Wand Scale/Floor` の二段フローへ変更
- 新規テスト `tests/test_extrinsics_capture.py` / `tests/test_extrinsics_samples.py` / `tests/test_extrinsics_initializer.py` / `tests/test_extrinsics_ba.py` を追加し、既存 `tests/test_wand_extrinsics.py` / `tests/test_wand_gui.py` を新フローに合わせて整理
- `src/camera-calibration/extrinsics_samples.py` を greedy cursor 消費型から global span 最小の multiview sample builder へ差し替え、`parallax_proxy` を sample quality に追加
- `src/camera-calibration/extrinsics_initializer.py` で `findEssentialMat()` の inlier のみを `recoverPose()` に流し、edge score を `usable_points * inlier_ratio * triangulation_angle_deg_p50` に整理
- `src/camera-calibration/extrinsics_ba.py` の seed triangulation を all-view LS 化し、sample weight と seed 統計を BA に追加
- `src/camera-calibration/calibrate_extrinsics.py` で BA 前 sample 選別を coverage + information ベースへ変更し、raw `Generate Extrinsics` の validation を `pose_validation` として分離
- `src/camera-calibration/extrinsics_validate.py` を `validate_pose_capture_extrinsics()` / `validate_wand_metric_extrinsics()` に分割し、`session_meta.pose_validation` と `session_meta.wand_metric_validation` を出力するよう更新
- review 指摘を反映し、raw `pose_validation` は BA subset ではなく accepted 全 sample で再計算、`Apply Wand Scale/Floor` 後の camera quality / GUI summary は `wand_metric_validation` を正とし、legacy `session_meta.validation` は出力時に除去
- `src/camera-calibration/extrinsics_samples.py` の近傍 suppression を探索前 hard drop から soft penalty へ変更し、密な高情報サンプル帯を deterministic に捨てないよう修正

## 8. 詳細ドキュメント

- Host モジュール概要: [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md)
- 内部較正: [`/src/camera-calibration/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/README.md)
- GUI operator runbook: [`/docs/10_in_progress/gui_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/gui_runbook.md)
- wand 4点 / referenceベース extrinsics 再設計計画: [`/docs/10_in_progress/wand_4point_extrinsics_plan.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/wand_4point_extrinsics_plan.md)
- jyjblrd ベース extrinsics migration 計画: [`/docs/10_in_progress/jyjblrd_extrinsics_migration_plan.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/10_in_progress/jyjblrd_extrinsics_migration_plan.md)
- wand 運用 runbook: [`/docs/20_completed/24_next_steps_wand_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/20_completed/24_next_steps_wand_runbook.md)

## 9. 現状の位置づけ

このリポジトリは「calibration から tracking 表示までの実働フロー」が通った段階です。いまの README は、実装の全詳細を説明するよりも、初見の人が次の 2 点を短時間で理解できるようにすることを目的にしています。

- 何をどの順で起動すればよいか
- どのファイルが入力で、どの JSON が出力なのか
