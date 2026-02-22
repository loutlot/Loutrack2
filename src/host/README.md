# src/host - Host側モジュール

loutrack2のHost側Pythonモジュール。UDP受信から剛体推定までの全処理を担当。

## モジュール一覧

### logger.py - フレームログ記録

UDPで受信したフレームをJSONL形式で記録。

```python
from host import FrameLogger

logger = FrameLogger(log_dir="./logs")
logger.start_recording(session_name="test")
logger.log_frame({"camera_id": "pi-01", "timestamp": 12345, ...})
logger.stop_recording()
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `FrameLogger` | スレッドセーフなフレームロガー |

**主要関数**:
| 関数 | 説明 |
|------|------|
| `list_log_files()` | ログファイル一覧取得 |

---

### replay.py - ログリプレイ

記録したログを再生。

```python
from host import FrameReplay

replay = FrameReplay("logs/test.jsonl")
for frame in replay.replay(realtime=True, speed=2.0):
    process(frame)
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `FrameReplay` | ログファイルリプレイヤー |
| `FrameEntry` | フレームデータ |
| `EventEntry` | イベントデータ |

**主要関数**:
| 関数 | 説明 |
|------|------|
| `validate_log_integrity()` | ログ整合性検証 |

---

### metrics.py - パフォーマンス監視

FPS、遅延、再投影誤差を収集・エクスポート。

```python
from host import MetricsCollector

metrics = MetricsCollector()
metrics.record_frame("pi-01", timestamp, blob_count=4, frame_index=42)
metrics.record_triangulation(12, [0.5, 0.3, 0.8])
print(metrics.get_summary())
print(metrics.export_prometheus())
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `MetricsCollector` | メトリクス収集器 |
| `MetricsExporter` | ファイルエクスポート |

---

### receiver.py - UDP受信 + ペアリング

複数PiからのUDPメッセージを受信し、timestamp/frame_indexでペアリング。

```python
from host import FrameProcessor

processor = FrameProcessor(udp_port=5000)
processor.set_paired_callback(on_paired_frames)
processor.start()
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `Frame` | 受信フレームデータ |
| `PairedFrames` | ペアリング済みフレームセット |
| `FrameBuffer` | カメラごとのフレームバッファ |
| `FramePairer` | timestampベースのペアリング |
| `UDPReceiver` | UDPサーバー |
| `FrameProcessor` | 統合プロセッサ |

**ペアリングロジック**:
- 主キー: `timestamp` (±5ms許容)
- 副キー: `frame_index` (フォールバック)

---

### geo.py - 幾何復元

DLT三角測量と再投影誤差計算。

```python
from host import Triangulator, create_dummy_calibration

params = create_dummy_calibration(["pi-01", "pi-02"])
triangulator = Triangulator(params)

points_3d = triangulator.triangulate_point(
    image_points=[(100, 200), (150, 180)],
    camera_ids=["pi-01", "pi-02"]
)
error = triangulator.compute_reprojection_error(image_points, points_3d, camera_ids)
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `CameraParams` | カメラパラメータ (内外因) |
| `CalibrationLoader` | 較正ファイル読み込み |
| `Triangulator` | DLT三角測量 |
| `GeometryPipeline` | 幾何処理パイプライン |

**主要関数**:
| 関数 | 説明 |
|------|------|
| `create_dummy_calibration()` | テスト用ダミー較正生成 |
| `undistort_points()` | 歪み補正 |

---

### rigid.py - 剛体推定

DBSCANクラスタリング + Kabschアルゴリズムによる剛体姿勢推定。

```python
from host import RigidBodyEstimator, WAIST_PATTERN

estimator = RigidBodyEstimator(patterns=[WAIST_PATTERN])
poses = estimator.process_points(points_3d, timestamp)
print(poses["waist"].position)
print(poses["waist"].quaternion)
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `MarkerPattern` | 剛体マーカー配置定義 |
| `RigidBodyPose` | 推定された剛体姿勢 |
| `PointClusterer` | DBSCANクラスタリング |
| `KabschEstimator` | SVDベース剛体姿勢推定 |
| `PnPEstimator` | PnPフォールバック |
| `RigidBodyTracker` | 時間追跡・予測 |
| `RigidBodyEstimator` | 統合パイプライン |

**定義済みパターン**:
| パターン | マーカー数 | 説明 |
|----------|------------|------|
| `WAIST_PATTERN` | 4 | ウエスト（非対称キー付き） |
| `HEAD_PATTERN` | 4 | ヘッド（簡易版） |

---

### pipeline.py - 統合パイプライン

全モジュールを統合したエンドツーエンドパイプライン。

```python
from host import TrackingPipeline

pipeline = TrackingPipeline(
    udp_port=5000,
    calibration_path="./calibration/",
    enable_logging=True
)

def on_pose(poses):
    for name, pose in poses.items():
        if pose.valid:
            print(f"{name}: {pose.position}")

pipeline.set_pose_callback(on_pose)
pipeline.start()
# ... 実行中 ...
pipeline.stop()
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `TrackingPipeline` | UDP→三角測量→剛体推定パイプライン |
| `TrackingSession` | セッション管理・ポーズ履歴 |

---

### sync_eval.py - 同期評価

ペアリング結果から同期品質を評価し、許容窓の推奨値を算出。

```python
from host import SyncEvaluator

ev = SyncEvaluator(
    tolerance_windows_us=(1000, 2000, 5000, 10000),
    target_range_us=(2000, 5000),
)

ev.evaluate_pair(paired_frames)
status = ev.get_status()
print(status["recommendation"])
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `SyncEvaluator` | 同期品質評価（pair spread / gap / drift / window sweep） |

**出力の主要項目**:
| 項目 | 説明 |
|------|------|
| `pair_spread_us` | ペア内時刻差の統計（mean/median/p95/p99/max） |
| `window_sweep` | 許容窓ごとの通過率 |
| `camera_offsets` | カメラ別 offset/jitter/drift |
| `missing_behavior` | frame_index gapに基づく欠損挙動 |
| `recommendation` | 推奨許容窓と判定状態 |

---

### visualize.py - 可視化

コンソール出力とファイルエクスポート。

```python
from host import TrackingVisualizer, SimpleGraph

visualizer = TrackingVisualizer(output_dir="./output")
visualizer.start_session("test")
visualizer.visualize_poses(poses, timestamp, metrics)
visualizer.end_session()

# グラフ
graph = SimpleGraph(width=60, height=10)
for error in errors:
    graph.add_value(error)
print(graph.render("RMS Error"))
```

**主要クラス**:
| クラス | 説明 |
|--------|------|
| `TrackingVisualizer` | ポーズ可視化 |
| `SimpleGraph` | ASCII時系列グラフ |

---

## データフロー

```
UDP受信 (receiver.py)
    ↓ Frame
ペアリング (receiver.py: FramePairer)
    ↓ PairedFrames
同期評価 (sync_eval.py: SyncEvaluator)
    ↓ sync status
三角測量 (geo.py: Triangulator)
    ↓ points_3d
クラスタリング (rigid.py: PointClusterer)
    ↓ clusters
剛体推定 (rigid.py: KabschEstimator)
    ↓ RigidBodyPose
出力 (visualize.py, logger.py)
```

## スレッド安全性

以下のクラスはスレッドセーフ:
- `FrameLogger`
- `FrameBuffer`
- `MetricsCollector`
- `RigidBodyTracker`
- `TrackingSession`
