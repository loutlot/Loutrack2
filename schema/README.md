# schema - JSON Schema定義

Pi↔Host 間メッセージと較正成果物の参照用 JSON Schema。
現状はランタイムでこのディレクトリを直接読みませんが、現行 wire format / 出力形式に合わせて保守します。

## ファイル一覧

### messages.json - Pi→Host フレームメッセージ

PiからHostへ送信されるフレームデータのフォーマット。

```json
{
  "camera_id": "pi-cam-01",
  "timestamp": 1708594800000000,
  "timestamp_source": "sensor_metadata",
  "frame_index": 42,
  "capture_mode": "pose_capture",
  "blob_count": 2,
  "quality": 0.91,
  "blobs": [
    {"x": 100.5, "y": 200.3, "area": 50.0},
    {"x": 300.0, "y": 400.0, "area": 45.0}
  ]
}
```

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `camera_id` | string | ✅ | カメラ/Pi識別子 |
| `timestamp` | integer | ✅ | Unix時間 (マイクロ秒, int64) |
| `timestamp_source` | string | | `sensor_metadata` または `capture_dequeue` |
| `frame_index` | integer | ✅ | フレーム連番 (uint32) |
| `capture_mode` | string | | `capture` / `pose_capture` / `wand_metric_capture` |
| `blob_count` | integer | | `pose_capture` 時のみ付与される accepted blob 数 |
| `quality` | number | | `pose_capture` 時のみ付与される単点観測 quality |
| `blobs` | array | ✅ | 検出されたマーカーblob配列 |
| `blobs[].x` | number | ✅ | X座標 (ピクセル) |
| `blobs[].y` | number | ✅ | Y座標 (ピクセル) |
| `blobs[].area` | number | ✅ | blob面積 (ピクセル²) |

**ペアリングキー**:
- 主キー: `timestamp` (PTP同期時)
- 副キー: `frame_index` (フォールバック)

---

### control.json - Host→Pi 制御メッセージ

HostからPiへ送信する制御コマンド。TCP/JSON (JSON-RPC風)。

**リクエスト形式**:
```json
{
  "request_id": "req-001",
  "camera_id": "pi-cam-01",
  "cmd": "set_exposure",
  "params": {"value": 10000}
}
```

**レスポンス形式**:
```json
{
  "request_id": "req-001",
  "camera_id": "pi-cam-01",
  "ack": true
}
```

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `request_id` | string | ✅ | リクエストID (要求/応答マッチング用) |
| `camera_id` | string | ✅ | 対象カメラID (または "broadcast") |
| `cmd` | string | ✅ | コマンド名 |
| `params` | object | | コマンド固有パラメータ |
| `ack` | boolean | ✅ | 受理フラグ (レスポンス時) |
| `error_code` | integer | | エラーコード (ack=false時) |
| `error_message` | string | | エラーメッセージ |

**schema 上のコマンド集合**:
| コマンド | パラメータ | 説明 |
|----------|-----------|------|
| `start` | `mode` | キャプチャ開始 |
| `stop` | - | キャプチャ停止 |
| `set_exposure` | `value` (μs) | 露出時間設定 |
| `set_gain` | `value` | アナログゲイン設定 |
| `set_fps` | `value` | FPS設定 |
| `set_focus` | `value` | フォーカス設定（LensPosition） |
| `set_threshold` | `value` | blob 二値化しきい値（0-255） |
| `set_blob_diameter` | `min_px`, `max_px` | blob 直径pxフィルタ |
| `set_circularity_min` | `value` | blob circularity 下限（0-1） |
| `mask_start` | `threshold`, `seconds`, `hit_ratio?`, `min_area?`, `dilate?` | static mask 構築開始 |
| `mask_stop` | - | static mask 構築停止 |
| `set_preview` | `render_enabled`, `overlays`, `charuco` | MJPEG preview / overlay 制御 |
| `ping` | - | 生存確認 |

補足:
- `control.json` の enum は schema 上の集合で、実際に受理するコマンドは [capture_runtime.py](/src/pi/service/capture_runtime.py) の `SCHEMA_COMMANDS` / `MVP_SUPPORTED_COMMANDS` に従います。
- `led_on` / `led_off` / `set_resolution` は schema enum に残っていますが、現行 MVP では未サポートです。

---

### calibration_intrinsics_v1.json - カメラ内部較正

Charucoボードを使用した内部較正の成果物。

```json
{
  "schema_version": "1.0",
  "camera_id": "pi-cam-01",
  "camera_model": "pinhole",
  "resolution": {"width": 1280, "height": 960},
  "camera_matrix": {
    "fx": 500.0,
    "fy": 500.0,
    "cx": 640.0,
    "cy": 480.0,
    "matrix": [[500, 0, 640], [0, 500, 480], [0, 0, 1]]
  },
  "distortion_coefficients": {
    "k1": -0.01,
    "k2": 0.02,
    "p1": 0.0,
    "p2": 0.0,
    "k3": 0.0,
    "array": [-0.01, 0.02, 0.0, 0.0, 0.0]
  },
  "rms_error": 0.35,
  "captured_at": "2026-02-22T12:00:00",
  "quality": {
    "num_valid_frames": 30,
    "total_points": 1200,
    "pixel_aspect_ratio": 1.0
  }
}
```

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `schema_version` | string | ✅ | スキーマバージョン ("1.0") |
| `camera_id` | string | | カメラ識別子 |
| `camera_model` | string | ✅ | 投影モデル ("pinhole" / "fisheye") |
| `resolution` | object | ✅ | 画像解像度 |
| `camera_matrix` | object | ✅ | 内部パラメータ行列 |
| `distortion_coefficients` | object | ✅ | 歪み係数 |
| `rms_error` | number | ✅ | 再投影誤差RMS (目標: <1.0px) |
| `captured_at` | string | ✅ | 較正実施日時 (ISO 8601) |
| `quality` | object | | 品質メトリクス |
| `board` | object | | Charucoボード設定 |

**品質基準**:
| 指標 | 良好 | 許容 |
|------|------|------|
| RMS再投影誤差 | < 0.5 px | < 1.0 px |
| 有効フレーム数 | 40-50 | 25+ |
| Pixel Aspect比 | 0.99-1.01 | 0.95-1.05 |

---

### calibration_extrinsics_v2.json - カメラ外部較正

現行の本流出力は `reference_pose_capture` ベースの `v2` です。
`pose` / `metric` / `world` を top-level で分離して持ちます。

```json
{
  "schema_version": "2.0",
  "method": "reference_pose_capture",
  "created_at": "2026-03-14T20:25:10.710719+00:00",
  "pose_capture_log_path": "logs/extrinsics_pose_capture.jsonl",
  "camera_order": ["pi-cam-01", "pi-cam-02"],
  "pose": {
    "frame": "similarity_camera",
    "camera_poses": [
      {
        "camera_id": "pi-cam-01",
        "R": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        "t": [0, 0, 0],
        "focal_scale": 0.88,
        "median_reproj_error_px": 2.57
      }
    ],
    "solve_summary": {
      "usable_rows": 1277,
      "median_reproj_error_px": 2.52
    }
  },
  "metric": {
    "status": "resolved",
    "frame": "metric_camera",
    "scale_m_per_unit": 6.58,
    "source": "wand_floor_metric"
  },
  "world": {
    "status": "resolved",
    "frame": "world",
    "to_world_matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
    "floor_plane": {"normal": [0, 0, 1], "offset": 0, "axis": "Z"},
    "source": "wand_floor_metric"
  }
}
```

| フィールド | 型 | 必須 | 説明 |
|-----------|-----|------|------|
| `schema_version` | string | ✅ | スキーマバージョン (`"2.0"`) |
| `method` | string | ✅ | 現在は `reference_pose_capture` |
| `created_at` | string | ✅ | 推定実施日時 (ISO 8601) |
| `pose_capture_log_path` | string | ✅ | 単点 pose capture ログ |
| `camera_order` | array | ✅ | 出力 camera 順序 |
| `pose` | object | ✅ | similarity camera frame の解 |
| `metric` | object | ✅ | scale 解決後の metric frame 情報 |
| `world` | object | ✅ | floor / world 軸アライン後の情報 |
| `wand_metric_log_path` | string | | floor / metric で使った wand log |

### calibration_extrinsics_v1.json - 旧 schema

`v1` は旧 wand 直解き系の参照用として残していますが、現行の `Generate Extrinsics` 出力は `v2` を使います。

---

## バリデーション

```python
import json
from jsonschema import validate

# スキーマ読み込み
with open("schema/messages.json") as f:
    schema = json.load(f)

# メッセージ検証
message = {"camera_id": "pi-01", "timestamp": 123, "frame_index": 0, "blobs": []}
validate(instance=message, schema=schema)
```

## バージョニング

- スキーマ変更時は `$id` と `schema_version` を更新
- 後方互換性を維持 (フィールド追加のみ、削除は不可)
- 破壊的変更時は新規ファイル作成 (e.g., `messages_v2.json`)
