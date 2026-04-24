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

## 2026-04-24 - Phase 1 Pattern Ambiguity Evaluator

### 実装結果

- `src/host/pattern_evaluator.py` を追加し、rigid body marker pattern の trackability を live tracking hot path から独立して評価できるようにした。
- built-in pattern (`waist`, `head`, `chest`, `left_foot`, `right_foot`) と `calibration/tracking_rigids.json` の custom rigids を同じ evaluator で読めるようにした。
- `self_symmetry_score()` で identity 以外の marker permutation による最小 Kabsch RMS、best permutation、rotation angle、verdict を出すようにした。
- `subset_ambiguity_score()` で 3 marker subset の距離プロファイル衝突数と closest pair を出すようにした。
- `cross_pattern_match_distance()` で pattern 間の最小 RMS と subset 対応を出すようにした。
- CLI は markdown report と `--json` の両方をサポートする。
- `tests/test_pattern_evaluator.py` を追加し、対称 pattern が ambiguous になること、built-in pattern が評価できること、custom rigids JSON を読めること、CLI JSON が出ることを確認した。

### 評価コマンド

```bash
/tmp/loutrack-py313-test/bin/python -m src.host.pattern_evaluator \
  --rigids calibration/tracking_rigids.json \
  --json > /tmp/loutrack_pattern_eval.json
```

### Built-in pattern 評価結果

Self symmetry:

| pattern | min self RMS mm | best permutation | rotation deg | verdict |
|---|---:|---|---:|---|
| waist | `4.652` | `[2, 3, 0, 1]` | `180.0` | `ambiguous` |
| head | `0.292` | `[1, 0, 3, 2]` | `180.0` | `ambiguous` |
| chest | `5.732` | `[2, 3, 0, 1]` | `180.0` | `ambiguous` |
| left_foot | `9.057` | `[2, 3, 0, 1]` | `180.0` | `ambiguous` |
| right_foot | `4.057` | `[3, 2, 1, 0]` | `180.0` | `ambiguous` |

3-marker subset ambiguity:

| pattern | ambiguous pairs | closest delta mm | closest pair |
|---|---:|---:|---|
| waist | `0` | `5.549` | `{a: [0, 1, 2], b: [0, 2, 3]}` |
| head | `6` | `0.343` | `{a: [0, 1, 2], b: [0, 1, 3]}` |
| chest | `0` | `6.448` | `{a: [0, 1, 2], b: [1, 2, 3]}` |
| left_foot | `1` | `2.818` | `{a: [0, 1, 2], b: [0, 2, 3]}` |
| right_foot | `2` | `2.816` | `{a: [0, 1, 3], b: [0, 2, 3]}` |

Closest cross-pattern matches:

| pattern A | pattern B | min RMS mm | verdict |
|---|---|---:|---|
| left_foot | right_foot | `6.572` | `ambiguous` |
| head | right_foot | `9.987` | `ambiguous` |
| head | chest | `10.239` | `ambiguous` |
| head | left_foot | `11.959` | `ambiguous` |
| chest | left_foot | `12.798` | `ambiguous` |

### 読み取り

Built-in pattern は全て 180 度回転に近い permutation で `15mm` 未満の self symmetry RMS になった。これは Phase 0 replay の `max_pose_flip_deg = 138.43282502436344 deg` と整合する。特に `head` は self symmetry RMS が `0.292mm` とほぼ完全に近く、3-marker subset ambiguity も `6` pair あるため、tracking 側で反転や再取得の曖昧性が出やすい。

現時点での重要な結論は、**rigid body 認識の甘さはアルゴリズムだけでなく、pattern geometry 自体の曖昧さも強く関係している可能性が高い**ということ。次の tracking 改善では、pose predictor や 2D score に ambiguity penalty を入れるだけでなく、物理 marker 配置の見直しも検討対象にする。

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_tracking_replay_harness.py
```

結果: `24 passed`

### 次に期待すること

- 次フェーズは **Phase 2 - Pose Predictor と rolling confidence** を推奨する。
- predictor は tracking 挙動を即座に大きく変えるのではなく、まず `peek_prediction(timestamp_us)` と rolling confidence を追加して、Phase 3 以降の mode transition / 2D scoring / object-conditioned gating が参照できる state を作る。
- `predict()` が state を進める設計は避ける。candidate scoring で何度も呼ばれるため、Phase 2 では side-effect-free な prediction API を先に作る。
- confidence は現在の `track_count / total_frames` ではなく、直近 valid ratio、lost run、reacquire、pose jump、pose flip candidate、RMS を使った rolling 指標にする。
- Phase 1 の ambiguity report は Phase 4/5 の scoring に `ambiguity_penalty` として渡せるよう、後続で structured diagnostics に接続する。

### 次フェーズの実装可否

次フェーズも実装可能。ただし Phase 2 は tracker state API に触るため、Phase 0/0.5/1 より少し慎重に進める。

実装可能な理由:

- `RigidBodyTracker` は既に position / velocity / quaternion / last valid timestamp を保持している。
- Phase 0 で rolling stats が入っているため、confidence の材料は揃っている。
- `process_points()` の外部 API を変えずに `peek_prediction()` と `get_prediction_diagnostics()` を追加できる。

注意点:

- Phase 2 ではまだ object-conditioned gating を入れない。予測 pose と confidence を観測可能にするだけに留める。
- `RigidBodyTracker.predict(dt=...)` は既存互換として残し、新 API は `peek_prediction(timestamp_us)` にする。
- replay で `valid/reacquire/pose_flip` 指標が Phase 1 と変わらないことを受入条件にする。
