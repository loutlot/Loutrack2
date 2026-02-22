# schema - JSON Schema定義

Pi↔Host間のメッセージフォーマットと較正データのJSON Schema定義。

## ファイル一覧

### messages.json - Pi→Host フレームメッセージ

PiからHostへ送信されるフレームデータのフォーマット。

```json
{
  "camera_id": "pi-cam-01",
  "timestamp": 1708594800000000,
  "frame_index": 42,
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
| `frame_index` | integer | ✅ | フレーム連番 (uint32) |
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

**使用可能コマンド**:
| コマンド | パラメータ | 説明 |
|----------|-----------|------|
| `start` | `mode` | キャプチャ開始 |
| `stop` | - | キャプチャ停止 |
| `set_exposure` | `value` (μs) | 露出時間設定 |
| `set_gain` | `value` | アナログゲイン設定 |
| `set_fps` | `value` | FPS設定 |
| `led_on` / `led_off` | `ir`, `status_r/g/b` | LED制御 |
| `ping` | - | 生存確認 |

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
