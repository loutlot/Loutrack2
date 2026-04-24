# Rigid Body Stabilization 実装ログ

## 2026-04-24 - Phase 0 Diagnostics / Same-path Replay

### 実装結果

- `src/host/rigid.py` に `TrackingStats` を追加し、rigid body ごとの valid/lost run、reacquire、short valid、pose jump、pose flip 候補、最新 RMS、observed marker 数を記録できるようにした。
- `src/host/pipeline.py` の `tracking_diagnostics` event と `get_status()` に tracking diagnostics を追加した。
- `src/host/tracking_replay_harness.py` を追加し、`logs/tracking_gui.jsonl` の frame entries を `FrameProcessor._on_frame_received()` に同期注入して、UDP 受信後段と同じ `FrameProcessor -> FramePairer -> TrackingPipeline._on_paired_frames -> GeometryPipeline -> RigidBodyEstimator` 経路で replay できるようにした。
- replay harness では過去ログの `received_at` が live stale-window で落ちないよう、replay 中だけ `FrameBuffer.max_age_seconds` を十分大きくしている。
- `tests/test_rigid_body_tracker_stats.py` と `tests/test_tracking_replay_harness.py` を追加し、既存 `tests/test_tracking_pipeline_diagnostics.py` も tracking diagnostics を検証するように更新した。

### 実ログ replay 結果

実行コマンド:

```bash
/tmp/loutrack-py313-test/bin/python -m src.host.tracking_replay_harness \
  --log logs/tracking_gui.jsonl \
  --calibration calibration \
  --rigids calibration/tracking_rigids.json \
  --patterns waist \
  --out /tmp/loutrack_stabilization_replay_summary.json
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- target pattern: `waist`
- final confidence: `0.9919517102615694`
- final RMS: `0.0015512782290946932 m`
- observed markers: `4`
- mean valid run: `123.25 frames`
- max valid run: `241 frames`
- current valid run: `241 frames`
- mean lost run: `1.3333333333333333 frames`
- reacquire count: `3`
- short valid ratio: `0.0`
- pose jump count: `0`
- max pose jump: `0.05091417487868144 m`
- max pose flip candidate: `138.43282502436344 deg`

### 読み取り

このログでは triangulation 自体はかなり安定している。`497` pair 中 `493` pose が valid で、pose jump は 10 cm 閾値を超えていない。一方で、reacquire が `3` 回あり、最大 pose flip candidate が約 `138 deg` 出ている。つまり次の改善対象は「点が作れないこと」より、**既知 rigid body の観測由来と対応付けを保持し、反転・再取得時の判定をより説明可能にすること**。

Phase 0 は「改善ロジックを入れず、観測できる状態にする」目的を満たした。次からは、replay summary の `pose_flip_deg`, `reacquire_count`, `valid_run` を Phase 間の比較指標として使える。

### 検証

OpenCV が入っている Python 3.13 環境で一時 venv を使って検証した。素の `python3` は Python 3.14 で `cv2` が無いため、このリポの tracking 系テストは依存入り Python で走らせる必要がある。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_tracking_replay_harness.py \
  tests/test_tracking_pipeline_diagnostics.py
```

結果: `6 passed`

### 次に期待すること

- Phase 0.5 で `BlobObservation2D` / `TriangulatedPoint` 相当の observation provenance を入れ、3D point がどの camera/blob/ray から来たかを rigid estimator へ渡せる状態にする。
- `points_3d` の既存互換出力は維持し、GUI scene や既存 tests は壊さない。
- replay summary に per-point provenance を後から追加できるようにし、次の Phase で `real_ray_count`, `virtual_marker_count`, `pose_flip_deg` をより正確に出す。
- Phase 0.5 では tracking 挙動を変えず、データ契約と diagnostics のみを追加する。

### 次フェーズの実装可否

次のフェーズは実装可能。推奨する次フェーズはロードマップ上の **Phase 0.5 - Observation Contract / Provenance**。

実装可能な理由:

- `src/host/geo.py` には既に `_BlobObservation` があり、raw/undistorted uv と camera id は内部で保持されている。
- `Triangulator.triangulate_paired_frames()` は accepted point ごとの contributing rays、reprojection error、epipolar error、triangulation angle を既に計算している。
- `GeometryPipeline.process_paired_frames()` は dict return なので、既存 key を保ったまま `observations_by_camera` と `triangulated_points` を追加できる。
- `TrackingPipeline` は `result` dict を受けて snapshot 化しているため、互換 key を残せば段階導入しやすい。

注意点:

- raw pixel と undistorted pixel を混ぜない。初期実装は既存 reprojection diagnostics と合わせて raw pixel を主 diagnostics にしつつ、undistorted uv も provenance として保持する。
- `TriangulatedPoint` を dataclass のまま GUI/API に流すと JSON 化で詰まるため、status/log 用には `to_dict` または plain dict に変換する。
- Phase 0.5 はあくまで観測契約の追加に留め、object-conditioned gating や single-ray continuation はまだ入れない。

## 2026-04-24 - Phase 0.5 Observation Contract / Provenance

### 実装結果

- `src/host/geo.py` に `BlobObservation2D` を追加し、各 blob の `camera_id`, `blob_index`, `raw_uv`, `undistorted_uv`, `area` を保持できるようにした。
- `src/host/geo.py` に `TriangulatedPoint` を追加し、3D point ごとに生成元の 2D observations、camera ids、blob indices、reprojection errors、epipolar errors、triangulation angles、source を保持できるようにした。
- `Triangulator.triangulate_paired_frames()` は既存の `points_3d` / `reprojection_errors` / `triangulation_quality` を維持しつつ、内部で `last_observations_by_camera` と `last_triangulated_points` を更新するようにした。
- `GeometryPipeline.process_paired_frames()` の返り値に `observations_by_camera` と `triangulated_points` を追加した。
- `TrackingPipeline` の latest triangulation snapshot に `observations_by_camera` と `triangulated_points` を保存し、`get_latest_triangulation_snapshot()` から plain dict/list として取得できるようにした。
- GUI/scene 向けの既存 `points_3d` はそのまま残したため、tracking 表示の入力互換性は維持している。

### データ契約

`observations_by_camera` は camera ごとの 2D 観測一覧。

```json
{
  "cam0": [
    {
      "camera_id": "cam0",
      "blob_index": 0,
      "raw_uv": [736.0, 432.0],
      "undistorted_uv": [736.0, 432.0],
      "area": 4.0
    }
  ]
}
```

`triangulated_points` は accepted 3D point ごとの provenance。

```json
{
  "point": [0.1, 0.0, 2.5],
  "observations": ["... BlobObservation2D dicts ..."],
  "camera_ids": ["cam0", "cam1"],
  "blob_indices": [0, 0],
  "contributing_rays": 2,
  "reprojection_errors_px": [0.1, 0.1],
  "epipolar_errors_px": [0.2],
  "triangulation_angles_deg": [12.0],
  "source": "generic",
  "rigid_name": null,
  "marker_idx": null,
  "is_virtual": false
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_tracking_replay_harness.py
```

結果: `18 passed`

実ログ replay も継続して通る。

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
).to_dict()
print({k: summary[k] for k in ['frame_count', 'pair_count', 'frames_processed', 'poses_estimated']})
print(summary['tracking']['waist'])
PY
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- reacquire count: `3`
- max pose flip candidate: `138.43282502436344 deg`

### 読み取り

Phase 0.5 では tracking ロジックを変えていないので、Phase 0 の replay 結果と同じ valid/reacquire/flip 指標が維持された。これで、次フェーズ以降で 2D reprojection score や object-conditioned gating を入れる際に、どの 3D point がどの camera/blob 由来なのかを追えるようになった。

特に次からは以下が可能になる。

- reacquire 直後の pose が、どの blob 組から作られたかを追跡できる。
- 表裏反転候補が出た frame で、marker 対応と 2D reprojection の整合性を検査できる。
- object-conditioned gating の preassignment 結果を `source = "rigid_hint"` や `is_virtual = true` として既存 point stream と区別できる。

### 次に期待すること

- 次フェーズは **Phase 1 - Pattern Ambiguity Evaluator** を推奨する。現在のログでも max pose flip candidate が大きいため、先に marker pattern 自体の自己対称性と subset ambiguity を数値化したい。
- `src/host/pattern_evaluator.py` を追加し、built-in pattern と `calibration/tracking_rigids.json` の custom rigids を同じ CLI で評価できるようにする。
- evaluator の出力を replay summary と並べることで、「アルゴリズムが悪い」のか「pattern が反転しやすい」のかを切り分ける。
- Phase 1 でも tracking runtime の挙動は変えず、warning / report / diagnostics だけを追加する。

### 次フェーズの実装可否

次フェーズも実装可能。Phase 1 は `src/host/rigid.py` の `MarkerPattern` と `KabschEstimator` だけで主要指標を計算できるため、`geo.py` や live tracking hot path に触らず低リスクに進められる。

注意点:

- 4 marker pattern は permutation が少なく、偶然かなり低 RMS で自己一致することがある。`min_self_symmetry_mm` だけでなく、該当 permutation と rotation angle も report する。
- 3 marker subset は平面/鏡像が曖昧になりやすいので、subset ambiguity は Phase 4/5 の hypothesis 採点に渡せる形式で保存する。
- GUI warning 連携は後回しでよい。まず CLI / tests / replay note で数値を出す。
