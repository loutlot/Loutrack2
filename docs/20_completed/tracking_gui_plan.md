# tracking_gui_plan

## 0. 目的

外部較正 (`calibration_extrinsics_v1.json`) 完了後に、そのまま同じ運用導線で:

- live tracking を開始できる
- 剛体 pose を 3D で確認できる
- カメラ配置を四角錐 frustum として可視化できる
- tracking 品質を GUI 上で判断できる

状態を作る。

## 1. 前提

既存実装で既に揃っているもの:

- `src/host/pipeline.py`
  - UDP 受信
  - フレーム pairing
  - 三角測量
  - rigid body pose 推定
- `src/host/rigid.py`
  - 剛体 pattern 定義
  - pose 出力 (`RigidBodyPose`)
- `src/host/wand_gui.py`
  - 較正前後の運用 Web GUI
  - settings 保存
  - session 開始/停止 UI

不足しているもの:

- tracking 用の live scene API
- 3D scene viewer
- extrinsics 完了後の導線
- pose / reprojection / visibility をまとめて見る GUI

## 2. 推奨方針

`wand_gui` を捨てずに、同一アプリ内の 2 ページ構成にする。

- Page 1: Wand / Extrinsics Generation
- Page 2: Tracking Confirmation

理由:

- 既存の calibration 導線を壊さずに拡張できる
- 同じカメラ inventory / settings / log path を再利用できる
- operator は「較正用 GUI」と「tracking 用 GUI」を行き来せずに済む
- calibration 完了後に、tracking 確認へ明示的に遷移できる
- 1 ページに詰め込みすぎず、操作目的を分離できる

ただし backend は分離する。

- calibration orchestration: 既存 `WandGuiState`
- tracking runtime: 新規 `TrackingRuntime` 相当

この分離で、UI は一体でも runtime の責務を明確に保てる。

## 3. 目標 UI

### 3.1 Page 1: Wand / Extrinsics Generation

既存 `wand_gui` をベースに、次の役割へ限定する。

- Blob Detection Adjustment
- Mask Adjustment
- Wand Capture
- Extrinsics Generation
- 直近の extrinsics quality 表示
- `Open Tracking` / `Tracking Confirmation へ進む` 導線

Page 1 は calibration 完了までの運用を短距離で回すことを優先し、3D scene は持たせない。

### 3.2 Page 2: Tracking Confirmation

画面は次の 2 レイヤー構成が使いやすい。

#### 上段: Scene

- 3D canvas
- world axis
- floor grid
- camera frustum (四角錐)
- rigid body marker points
- rigid body local axis
- 軌跡 tail (直近 N 秒)

#### 下段: Control / Health

- Start Tracking / Stop Tracking
- calibration file path
- tracked rigid body 一覧
- per-camera health
  - fps
  - last frame age
  - blob count
  - paired ratio
- per-rigid-body health
  - valid
  - observed markers
  - rms error
  - lost duration

## 4. レンダリング方針

3D viewer は Web ベースで実装する。

推奨:

- `wand_gui.py` に router 相当の page state を追加
- Page 2 の HTML に tracking scene panel を追加
- frontend では local static asset として `three.js` を同梱して使う
- CDN 前提にはしない

描画対象:

1. Camera frustum
   - extrinsics から camera origin / rotation を算出
   - intrinsics + 仮想 near plane から image plane 四隅を計算
   - line segments で四角錐表示
2. Rigid body
   - `RigidBodyPose.position`
   - `RigidBodyPose.rotation`
   - pattern marker positions を world へ変換して点表示
   - body axes を RGB で表示
3. Debug layers
   - triangulated raw 3D points
   - reprojection / validity 色分け
   - trajectory trail

## 5. backend 構成案

新規に `TrackingRuntime` を追加し、`TrackingPipeline` を GUI 用に包む。

責務:

- `TrackingPipeline` の start/stop
- 最新 scene snapshot を保持
- frontend へ status と scene を供給
- logs/tracking_*.jsonl の記録

最小 API:

- `GET /api/tracking/status`
- `POST /api/tracking/start`
- `POST /api/tracking/stop`
- `GET /api/tracking/scene`

将来拡張:

- `GET /api/tracking/stream` (SSE)
- `POST /api/tracking/patterns`
- `POST /api/tracking/reset_origin`

## 6. scene snapshot schema

初版は polling で十分。

```json
{
  "tracking": {
    "running": true,
    "frames_processed": 1234,
    "poses_estimated": 1180
  },
  "cameras": [
    {
      "camera_id": "pi-cam-01",
      "position": [0.0, 1.2, 2.0],
      "rotation_matrix": [[1,0,0],[0,1,0],[0,0,1]],
      "frustum_near_corners": [[...], [...], [...], [...]],
      "healthy": true,
      "fps": 56.0,
      "last_frame_age_ms": 18.0
    }
  ],
  "rigid_bodies": [
    {
      "name": "waist",
      "valid": true,
      "position": [0.1, 0.9, 2.4],
      "quaternion": [1, 0, 0, 0],
      "rms_error": 0.003,
      "observed_markers": 4,
      "markers_world": [[...], [...], [...], [...]]
    }
  ],
  "raw_points": [[...], [...]],
  "timestamp_us": 1234567890
}
```

重要なのは frontend で再計算させすぎないこと。

- frustum corner 計算
- marker world 座標計算

は backend で済ませ、viewer は描画に集中させる。

## 7. 実装ステップ

### Step 1: Page 2 の静的 3D scene

- calibration directory を読み込み
- camera frustum と world axis だけ描画
- tracking なしでも Page 2 で camera layout を確認できる

完了条件:

- `calibration_extrinsics_v1.json` 読込後に 2 台以上の camera pyramid が見える

### Step 2: Page 1 / Page 2 の遷移導線

- header or tab で `Calibration` / `Tracking` を切り替えられる
- extrinsics 未生成時は Page 2 を read-only または empty state 表示にする
- `latest extrinsics path` を両ページで共有する

完了条件:

- calibration 完了後に operator が迷わず Page 2 へ移動できる

### Step 3: live tracking runtime を GUI に接続

- `TrackingRuntime` 追加
- `TrackingPipeline.set_pose_callback()` を scene snapshot 更新へ接続
- Page 2 の start/stop API を GUI ボタンへ接続

完了条件:

- Page 2 から tracking start/stop ができる
- status が更新される

### Step 4: rigid body 3D 描画

- pose を axis gizmo で描画
- marker 点群と rigid body local axes を表示
- valid / invalid 色分け

完了条件:

- 少なくとも `waist` 1 剛体の位置と向きが live 表示される

### Step 5: quality overlay

- per-camera health card
- pairing / reprojection / observed markers を表示
- tracking lost 時に scene と panel の両方で警告

完了条件:

- operator が再較正か再収録かを GUI だけで判断できる

### Step 6: multi-rigid-body / replay

- pattern 複数選択
- body ごとの表示 ON/OFF
- tracking log replay

完了条件:

- live と replay の両方で 5 剛体を確認できる

## 8. 使いやすさの要件

extrinsics 作成後の GUI は「次に何を押すか」が明確であるべき。

具体案:

- Page 1:
  1. Blob
  2. Mask
  3. Wand Capture
  4. Extrinsics
- Page 2:
  - Tracking scene
  - Tracking controls
  - health panel
- `Extrinsics Complete` になったら `Tracking Confirmation` への導線を強調表示する
- Page 2 は tracking 未開始でも静的 camera layout を見せる
- calibration path は自動入力
- last generated extrinsics の quality 指標を Page 1 / Page 2 の両方で参照できるようにする

## 9. 技術的リスク

- browser 側描画負荷
  - 対策: 初版は 10-15fps polling、tail 長も短くする
- scene 更新量が多い
  - 対策: 最新 snapshot のみ送る
- pattern が増えると UI が煩雑
  - 対策: 初版は `waist` 固定、その後 body selector を追加
- calibration file と live runtime の整合性
  - 対策: 起動時に loaded camera ids を明示表示

## 10. 推奨着手順

最初にやるべき順は以下。

1. Page 2 static camera frustum viewer
2. page navigation / empty state
3. tracking runtime wrapper + status API
4. 1 rigid body live render
5. health panel

この順なら、表示と runtime のどちらで壊れているかを毎段階で切り分けやすい。

## 11. コードベース対応表（どこをどう変えるか）

### 11.1 `src/host/wand_gui.py`

追加/変更対象:

- page state（`calibration` / `tracking`）を持つ UI state を追加
- `do_GET` に Page 2 用 route を追加
  - 例: `/tracking`（同一 HTML で表示切替でも可）
- `do_GET` に tracking API を追加
  - `/api/tracking/status`
  - `/api/tracking/scene`
- `do_POST` に tracking control API を追加
  - `/api/tracking/start`
  - `/api/tracking/stop`
- HTML/JS に page navigation と 3D viewer 初期化ロジックを追加

既存流用:

- `WandGuiState.get_state()` の workflow 情報は Page 1 で継続利用
- `generate_extrinsics()` の結果（output path, quality）を Page 2 の初期入力へ引き継ぐ

### 11.2 `src/host/pipeline.py`

変更は最小にする。

- `TrackingPipeline` 自体はほぼ据え置き
- GUI 用に必要な追加は accessor レベルで吸収
  - `last_points_3d`
  - `last_reprojection_errors`
  - `last_pair_timestamp_range_us`

※ 大きな責務追加は避け、GUI 向け集約は runtime wrapper 側で行う。

### 11.3 新規 `src/host/tracking_runtime.py`（追加）

役割:

- `TrackingPipeline` の起動停止管理
- thread-safe に最新 scene snapshot を構築/保持
- camera frustum 計算と rigid marker world 変換
- GUI API に返す JSON の整形

想定クラス:

- `TrackingRuntime`
  - `start(calibration_path: str, patterns: list[str])`
  - `stop()`
  - `status() -> dict`
  - `scene_snapshot() -> dict`

内部:

- `self._pipeline: TrackingPipeline`
- `self._latest_scene: dict`
- `self._lock: threading.Lock`
- `self._active_calibration_path: str`

### 11.4 `src/host/geo.py`

追加候補（必要なら）:

- camera center / orientation を安全に取り出す helper
  - `camera_center_world(camera: CameraParams) -> np.ndarray`
  - `camera_axes_world(camera: CameraParams) -> np.ndarray`

ただし可能なら `tracking_runtime.py` 側で完結し、`geo.py` には追加しない。

### 11.5 `tests/test_wand_gui.py`

追加:

- Page 切替の state 反映テスト
- tracking start/stop API の smoke
- extrinsics 未生成時に Page 2 が empty state になるテスト

既存テスト整理:

- Page 1 の workflow テストと重複する assert は統合する

### 11.6 新規 `tests/test_tracking_runtime.py`

追加:

- frustum geometry の計算テスト
- pose callback から scene snapshot が更新されるテスト
- runtime start/stop の状態遷移テスト

## 12. ロジック詳細

### 12.1 Camera frustum（四角錐）計算

入力:

- camera intrinsics: `fx, fy, cx, cy`, resolution `w, h`
- camera extrinsics: `R, t`（world -> camera）
- 描画用 near 距離 `z_near`（例: 0.25m）

手順:

1. image plane の 4 点を pixel で定義:
   - `(0,0), (w,0), (w,h), (0,h)`
2. pinhole 逆投影で camera 座標へ:
   - `x = (u - cx) / fx * z_near`
   - `y = (v - cy) / fy * z_near`
   - `z = z_near`
3. camera -> world 変換:
   - `Xw = R^T * (Xc - t)` または `Xw = C + R^T * Xc`（`C = -R^T t`）
4. `C` と 4 corner を line segments 化して返す

出力:

- `camera_origin_world`
- `frustum_near_corners_world[4]`
- `frustum_lines`（viewer がそのまま描画できる形）

### 12.2 Rigid body marker world 計算

入力:

- `RigidBodyPose.position`（world）
- `RigidBodyPose.rotation`（3x3）
- `MarkerPattern.marker_positions`（body local）

式:

- `marker_world_i = R_body * marker_local_i + t_body`

出力:

- `markers_world`
- `body_axes`（origin + xyz unit vectors）

### 12.3 Scene 更新戦略

更新源:

- `TrackingPipeline.set_pose_callback()`
- `TrackingPipeline.get_status()`

更新頻度:

- backend: pose callback 到達ごと
- frontend: polling 10-15Hz（初版）

戦略:

- 最新 snapshot のみ保持（履歴全送信しない）
- trail は backend で固定長 ring buffer を持つ
- lock 粒度は snapshot 入替時のみ

## 13. API 契約（初版）

### `GET /api/tracking/status`

```json
{
  "running": true,
  "calibration_loaded": true,
  "frames_processed": 12345,
  "poses_estimated": 12200,
  "receiver": {"frames_received": 13000, "cameras_discovered": 2},
  "sync": {"coverage_5000us": 0.97}
}
```

### `POST /api/tracking/start`

request:

```json
{
  "calibration_path": "calibration",
  "patterns": ["waist"]
}
```

response:

```json
{
  "ok": true,
  "running": true,
  "calibration_loaded": true
}
```

### `POST /api/tracking/stop`

response:

```json
{
  "ok": true,
  "summary": {
    "frames_processed": 12345,
    "poses_estimated": 12200,
    "duration_seconds": 20.1,
    "log_metadata": {"log_file": "logs/tracking_smoke.jsonl"}
  }
}
```

### `GET /api/tracking/scene`

`Section 6` の snapshot schema を返す。

## 14. UI 仕様（2 ページ）

### Page 1: Wand / Extrinsics Generation

- 現行機能を維持
- `Extrinsics Complete` 時に `Go to Tracking Confirmation` ボタンを有効化
- 直近 extrinsics quality を summary card に表示

### Page 2: Tracking Confirmation

必須 UI:

- 左: 3D scene
- 右上: tracking controls
- 右下: health panel
- 上部: `Back to Calibration` ナビ

empty state:

- extrinsics 未生成時は `Generate extrinsics first` を表示
- Start ボタンは disabled

## 15. テスト計画

### 15.1 単体テスト

- frustum corner 計算の数値整合
- marker world 変換の数値整合
- snapshot schema の key 存在性
- extrinsics 未設定時の empty state 応答

### 15.2 結合テスト

- `POST /api/tracking/start` -> `GET /api/tracking/status running=true`
- pose callback 模擬入力で `GET /api/tracking/scene` に body が出る
- `POST /api/tracking/stop` で log metadata が返る

### 15.3 実機スモーク

1. Page 1 で extrinsics を生成
2. Page 2 に遷移して camera frustum が表示される
3. tracking start で rigid body が live 更新される
4. stop 後に summary と log file が確認できる
