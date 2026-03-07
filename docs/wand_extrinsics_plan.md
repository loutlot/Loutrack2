# wand_extrinsics_plan

## 1. 目的

外部較正を Charuco ペア方式ではなく、B5 サイズ L 字 wand（3端点、14mm blob）を使った運用に統一する。
Host 主導で Pi 群を同期制御し、時刻整合された観測ログから extrinsics を推定する。

## 2. 方針（結論）

- **Host を絡める**: 複数 Pi の開始時刻・設定配布・収録管理を一元化できるため。
- **Pi は lightweight**: 受信した制御に従い、観測送信と最小前処理のみ担当。
- **外部較正の正本は wand**: `docs/requirements_def.md` の推奨決定と整合。
- **収録モードを分離**: 通常トラッキングと wand 較正収録を mode で分ける。
- **Pi 発見は passive-first**: `UDPReceiver` の受動発見 + `hosts.ini` 補完を標準とし、active scan は任意機能とする。

## 3. 前提パラメータ（固定値）

- wand 名称: `wand_l_b5_v1`
- マーカー数: 3
- マーカー直径: 14mm
- 3D 座標定義（mm）: `WAND_POINTS_MM = [(0,0,0), (168,0,0), (0,243,0)]`
- 収録最小ポーズ数: 20
- 推奨収録時間: 60 秒
- mask 初期化フレーム数: 30
- ペアリング許容窓（初期）: 5000us

注: `WAND_POINTS_MM` は center-to-center 実測基準（0.1mm 以下精度）で固定済み。

## 4. 想定ワークフロー

1. Host が全 Pi に `set_exposure` / `set_gain` / `set_fps` を配布
2. Host が `mask_start` を送信し、Pi が 30 フレームで背景明点マスクを自動生成
3. Host が `start`（`mode=wand_capture`）を送信し wand 収録を開始
4. wand を空間全体で 20+ ポーズ（推奨 60 秒）動かして観測取得
5. Host が `stop` を送信して収録終了
6. Host で extrinsics solver を実行し `calibration_extrinsics_v1.json` を出力
7. Host 幾何パイプラインに intrinsics + extrinsics を読み込み検証

## 5. 実装タスク

### 5.1 Pi 側（`src/pi/capture.py`）

- `mask_start` 実装
  - 開始直後 30 フレームを取得
  - 各画素で閾値超過回数を集計し、静的マスクを生成
  - 本撮影ではマスク領域を除外して blob 検出
- `mask_stop` 実装（マスク解除/再初期化）
- `start` 時の mode 拡張（`capture`, `wand_capture`）
- 応答 `result` に mode / mask 状態を返せるようにする

### 5.2 Host 側制御

- 既存 `src/host/control.py` を使い、複数 Pi へ一括制御するオーケストレータを追加
- 映像なし GUI（スライダー中心）を追加し、複数 Pi の設定を同期操作できるようにする
- 収録セッション ID、開始/終了時刻、適用パラメータを記録
- コマンド順序を固定
  - `ping` -> `set_exposure`/`set_gain`/`set_fps` -> `mask_start` -> `start(mode=wand_capture)` -> `stop`

GUI の最小要件:

- スライダー: `exposure_us`, `gain`, `fps`（全 Pi 同期適用）
- ボタン: `ping`, `mask_start`, `start`, `stop`
- 一覧: `camera_id`, `ip`, `last_ack`, `last_error`, `is_running`
- 同期モード: 1 本のスライダー操作を対象 Pi 全台へ配信
- 実装方式: Host ローカルの映像なし Web UI（単一ページ）

### 5.3 収録データ形式

- Pi 送信データは既存 `schema/messages.json` のまま利用
- Host 側で session 単位の保存を追加（JSONL 推奨）
  - 最低限: `session_id`, `camera_id`, `timestamp`, `frame_index`, `blobs`
  - 推奨追加: `mask_ratio`, `exposure`, `gain`, `fps`, `host_received_at`

### 5.4 外部較正ソルバ

- `src/camera-calibration/calibrate_extrinsics.py` を新規実装
- 入力
  - `calibration_intrinsics_v1_*.json`
  - wand 収録ログ（複数カメラ観測）
  - wand 3点の既知 3D 配置（mm）
- 出力
  - `calibration/calibration_extrinsics_v1.json`

### 5.5 Host 幾何統合

- `src/host/geo.py` に extrinsics ロード機能を追加
- `load_calibration` で intrinsics + extrinsics を同時適用可能にする

### 5.6 既存コード接続点（現状）

- 制御送信クライアント: `src/host/control.py`
  - 既存: `ping`, `start`, `stop`, `set_exposure`, `set_gain`, `set_fps`
- 受動発見: `src/host/receiver.py`
  - `UDPReceiver.get_camera_addresses()` で `camera_id -> (ip, port)` を保持
- Pi 制御サーバ/送信本体: `src/pi/capture.py`
  - 既存: `set_exposure`/`set_gain`/`set_fps` の適用
  - 未実装: `mask_start`/`mask_stop`/`wand_capture` mode

## 6. コードロジック詳細（実装仕様）

### 6.1 Pi 側状態遷移

- 状態
  - `IDLE`: 待機
  - `MASK_INIT`: `mask_start` 実行中
  - `READY`: mask 準備完了、未収録
  - `RUNNING`: 収録中
- 遷移
  - `IDLE --mask_start--> MASK_INIT --done--> READY`
  - `IDLE/READY --start--> RUNNING --stop--> READY`
  - `READY --mask_stop--> IDLE`

### 6.2 `mask_start` アルゴリズム

1. 30 フレーム取得
2. 各フレームを grayscale 化
3. `gray > threshold` を二値化
4. `hit_count[y, x] += 1` を累積
5. `hit_count >= ceil(hit_ratio * N)` を mask 候補化（例: `hit_ratio=0.7`）
6. 小領域除去（`min_area_px`）
7. 膨張（`dilate_px`）して安全マージン確保
8. `self._static_mask` として保持

実装メモ:

- データ型は `uint16` 累積で十分（30 フレーム想定）
- mask 生成結果は `result` に `mask_pixels` / `mask_ratio` を返す
- mask 比率が異常（例: 40%以上）なら warning を返す

### 6.3 blob 検出への mask 適用

- `detect_blobs` 前に `gray[mask==1] = 0` を適用
- 既存しきい値処理は維持
- 既存 area フィルタを維持

### 6.4 Host オーケストレータ（新規）

推奨ファイル: `src/host/wand_session.py`

主処理:

1. inventory から対象カメラ一覧読み込み
2. 全カメラ `ping` 成功を確認
3. 設定配布（露光/ゲイン/FPS）
4. 全カメラ `mask_start` 実行
5. 収録セッション作成（`session_id` 発行）
6. 全カメラ `start(mode=wand_capture)` を短時間に送信
7. 指定時間収録
8. 全カメラ `stop`
9. session メタデータ保存

探索ロジック:

1. `hosts.ini` の静的 inventory を読み込み
2. `UDPReceiver.get_camera_addresses()` の受動発見結果をマージ
3. camera_id 優先で重複解決（受動発見IPを優先）
4. 各候補へ `ping` して最終対象を確定

失敗時ポリシー:

- 1台でも `start` 失敗なら全台 `stop` を送ってセッション中止
- `mask_start` 失敗台は再試行1回、失敗継続なら除外して中止判断

### 6.5 Host GUI 同期スライダー（新規）

推奨ファイル: `src/host/wand_gui.py`

内部ロジック:

1. GUI state に `selected_cameras` と `sync_enabled` を保持
2. スライダー操作時にデバウンス（例: 200ms）
3. デバウンス後に対象全台へ並列送信
4. `ack` 集約結果を UI に反映

同期適用擬似コード:

```python
def apply_slider_sync(param: str, value: float | int) -> None:
    targets = get_selected_cameras()
    fn = {
        "exposure_us": control.set_exposure,
        "gain": control.set_gain,
        "fps": control.set_fps,
    }[param]
    results = parallel_map(lambda cam: fn(cam.ip, cam.port, cam.camera_id, value), targets)
    update_ack_table(results)
```

注意点:

- 連続送信を避けるため、デバウンス + 最終値優先で送信
- 失敗台を UI で明示し、再送ボタンを提供

### 6.6 時刻整合とペアリング

- 既存方針どおり `timestamp` 主、`frame_index` 副
- wand solver では `pair_window_us` 内の観測だけ採用
- 2台のみなら 1:1 近傍、3台以上なら中心 timestamp でクラスタ化

### 6.7 wand 対応付け（初版）

- 前提: 各フレームで wand 由来の 3 blob が主要成分
- 各カメラフレームで最大 3 blob を候補化（area 上位 + 位置安定）
- フレーム間で nearest-neighbor 追跡し ID 継続
- ID 不確定フレームは破棄（初版は recall より precision 優先）

### 6.8 extrinsics 推定（初版）

1. 参照カメラを 1 台固定（`R=I, t=0`）
2. 参照カメラとの 2D 対応点集合を構築
3. `findEssentialMat` + `recoverPose` で `R, t_unit` を推定
4. 三角測量で wand 3点を復元し、既知辺長との誤差最小でスケール `s` を推定
5. `t = s * t_unit` を確定
6. 全フレームで再投影誤差を算出
7. 必要なら非線形最適化（BA）で `R,t` を微調整

### 6.9 品質ゲート

- `inlier_ratio`（Essential 推定）
- `median_reproj_error_px`
- `baseline_m`（現実的範囲）
- フレーム採用率（有効フレーム数 / 総フレーム数）

### 6.10 発見ロジックの運用ルール

- 標準: passive 発見（UDP受信） + `hosts.ini` の併用
- active scan は初期リリースでは実装しない
- active scan を追加する条件（任意）:
  - Pi が UDP を送る前に GUI で即制御したい
  - DHCP 変動が激しく inventory 保守負荷が高い
  - 受動発見率が運用上不足

## 7. スキーマ計画

- 新規: `schema/calibration_extrinsics_v1.json`
- 主要フィールド案
  - `schema_version`
  - `reference_camera`
  - `camera_pairs[]`
  - `rotation_matrix`, `translation_vector`, `essential_matrix`, `fundamental_matrix`
  - `rms_error`, `baseline_m`, `captured_at`
  - `session_meta`（session_id、収録条件、wand 定義ID）

## 8. 受け入れ条件（Phase 2 完了判定に追加）

- `mask_start` が 30 フレーム初期化を実行し、マスク状態が収録に反映される
- 2 台以上で wand 収録を実行し、extrinsics JSON を生成できる
- Host で intrinsics + extrinsics を読み込み、三角測量が継続動作する
- 再投影誤差・基線長が運用上不自然でない

## 9. リスクと対策

- **同期ズレ**: Host 主導 start/stop と timestamp 監視で補正
- **wand 対応付け誤り**: 3点幾何制約（辺長/角度）で外れ値除去
- **背景ノイズ残り**: `mask_start` のしきい値と 30 フレーム窓を調整可能にする
- **過剰マスク**: mask 適用率メトリクスを出し、異常時に `mask_stop` で解除
- **スケール不定**: 既知 wand 辺長を拘束に使い `t` スケールを決定
- **発見漏れ**: `hosts.ini` 併用と `ping` ヘルスチェックで補完

## 10. 段階導入

1. `mask_start` / `mask_stop` 実装
2. Host から複数 Pi へ制御一括実行（passive discovery + inventory）
3. 映像なし GUI（同期スライダー）実装
4. wand 収録ログ保存
5. extrinsics solver 最小実装（2 台）
6. `geo.py` で extrinsics ロード統合
7. 3 台以上へ拡張

## 11. 未確定事項（実装前に決める）

- `WAND_POINTS_MM` の現場再測（定期点検時のみ）
- `mask_start` パラメータ既定値
  - `threshold`, `hit_ratio`, `min_area_px`, `dilate_px`
- wand 収録時の最小 blob area / 最大 blob area
- quality gate の閾値

## 12. Host Wand Control GUI & Orchestration

### 12.1 `wand_session.py`: orchestration with control client + passive discovery

- `wand_session.py` becomes the Host-side coordinator that owns a `ControlClient`, an `UDPReceiver`, and the optional static inventory (`hosts.ini`).
- Discovery flow:
  1. `UDPReceiver.get_camera_addresses()` streams passive notices from Pis; merge these with `hosts.ini` entries by `camera_id` so that the most recent IP/port wins.
  2. `wand_session.WandSession.prepare_targets()` filters to healthy cameras by pinging each candidate once before the session starts.
- Control flow:
  1. Apply `set_exposure`, `set_gain`, `set_fps` with a shared payload before any capture commands.
  2. Issue `mask_start` and wait for each Pi to ack mask readiness; re-run once on failures, then abort if any camera still fails.
  3. Start the streaming window in `mode="wand_capture"`, track `session_id`, and keep a cancel token to stop the session if an error appears mid-run.
  4. After the desired duration or manual stop, send `stop` to all cameras and record the returned metadata.
- Pseudo code (simplified):

```python
class WandSession:
    def __init__(self, control_client, receiver, inventory):
        self.control = control_client
        self.receiver = receiver
        self.inventory = inventory

    async def run_session(self, config):
        targets = await self.prepare_targets(config.selection)
        await self.control.broadcast('set_exposure', targets, value=config.exposure_us)
        await self.control.broadcast('set_gain', targets, value=config.gain)
        await self.control.broadcast('set_fps', targets, value=config.fps)
        await self.control.broadcast('mask_start', targets, mask_params=config.mask)
        session = SessionMeta.create(config, targets)
        await self.control.broadcast('start', targets, mode='wand_capture', session_id=session.id)
        await wait(config.duration)
        await self.control.broadcast('stop', targets)
        session.finalize()
        await session.save()
```

### 12.2 Web UI architecture / slider sync & ack reporting

- The GUI is a lightweight, video-free single page powered by `fastapi` (or `quart`) that serves a simple HTML/JS bundle and exposes a WebSocket for status updates.
- State: `selected_cameras`, `sync_enabled`, `last_ack_map`, `errors`, `is_running` per camera.
- Slider sync logic (debounced, final value only):

```python
async def apply_slider_sync(param: str, value: float | int):
    targets = ui_state.get_selected_cameras()
    fn = {
        'exposure_us': control_client.set_exposure,
        'gain': control_client.set_gain,
        'fps': control_client.set_fps,
    }[param]

    futures = [asyncio.create_task(fn(cam.address, cam.camera_id, value)) for cam in targets]
    results = await asyncio.gather(*futures, return_exceptions=True)

    ack_payload = []
    for cam, result in zip(targets, results):
        if isinstance(result, Exception):
            ack_payload.append({
                'camera_id': cam.camera_id,
                'ack': False,
                'error': str(result)
            })
        else:
            ack_payload.append({
                'camera_id': cam.camera_id,
                'ack': result.ack,
                'error': result.error_message,
            })

    await ui_state.websocket.broadcast({
        'type': 'slider_ack',
        'param': param,
        'value': value,
        'results': ack_payload,
    })
```

- The JS front-end keeps a map of `camera_id -> ack/error` and highlights any Pi that reported `ack=false` or an error string; a “replay last slider” button can resubmit the stored value.
- Ack/error updates are also appended to an operation log area so hosts can audit whether a value change was successfully applied across the fleet.

### 12.3 Operation coordination (mask_start / start / stop)

- Sequence:
  1. `mask_start`: orchestration module flushes previous mask states, instructs each Pi to capture 30 frames, and waits for boolean completion. Each result stores `mask_ratio` and `mask_pixels` for later diagnostics.
  2. `start`: `mode=wand_capture` and `session_id` are sent together to mark the beginning of wand sensing. All targets must ack; a failure triggers a `stop` call to the already-running cameras and aborts the session.
  3. `stop`: invoked either after the planned duration or early stop command. Responses include `frames_received`, `blobs_detected`, `mask_state`, `session_id` to ensure the session metadata aligns.
- Ack collection/reporting: every control request returns a `ControlResponse` (`ack`, `error_code`, `error_message`). `wand_session` aggregates these into `session.ack_history` for the UI and logs.

### 12.4 Dependencies

- Host control stack: existing `src/host/control.py` + `UDPReceiver` from `src/host/receiver.py`.
- Web stack: `fastapi`, `uvicorn`, `jinja2` (or plain HTML templates), `python-socketio` or native WebSocket implementation for status pushes.
- Async support: `anyio` / `asyncio` for parallel command dispatch and UI updates.
- Front-end: vanilla JS or lightweight framework (e.g., Alpine.js) to tie sliders to API calls.

### 12.5 Implementation steps

1. Create `src/host/wand_session.py` that encapsulates discovery merging, ping health checks, session metadata recording, and sequential command dispatch.
2. Extend `src/host/control.py`’s client to expose helper methods for masked operations and to return structured `ControlResponse` objects.
3. Implement the passive-first discovery merge between `UDPReceiver` notices and `hosts.ini`, then add a `ping` gate in `wand_session` to lock the target set.
4. Build the FastAPI-powered Web UI (`src/host/wand_gui.py`) with endpoints for slider commands, mask/start/stop triggers, and WebSocket streams for ack logs.
5. Wire the UI slider handler to the pseudo code above so every slider change triggers `apply_slider_sync`, collects ack/error tuples, and broadcasts to connected clients.
6. Hook mask/start/stop buttons into `wand_session` operations so they record per-Pi responses and expose the aggregated status back to the UI.
7. Update documentation/README and add simple integration tests (mock control client) to verify ack aggregation and command ordering.
