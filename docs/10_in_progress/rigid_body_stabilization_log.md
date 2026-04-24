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

## 2026-04-24 - Phase 2 Pose Predictor / Rolling Confidence

### 実装結果

- `src/host/rigid.py` に `PredictedPose` を追加し、予測 pose、velocity、dt、confidence、position sigma、rotation sigma を structured diagnostics として扱えるようにした。
- `RigidBodyTracker.peek_prediction(timestamp_us)` を追加し、tracker state を変更せずに指定 timestamp の pose を予測できるようにした。
- 既存 `RigidBodyTracker.predict(dt=...)` は互換 API として残した。Phase 2 では tracking の採択ロジックは変更していない。
- `RigidBodyTracker.rolling_confidence` を追加し、直近 valid ratio、median RMS、lost frames、pose jump、pose flip candidate から gating 用の confidence を計算できるようにした。
- `RigidBodyTracker.get_prediction_diagnostics()` と `get_diagnostics()` に prediction payload と rolling confidence を追加した。
- `tests/test_rigid_body_tracker_stats.py` に side-effect-free prediction と rolling confidence 低下のテストを追加した。

### データ契約

`tracking.<name>.prediction` は以下の形で出る。

```json
{
  "timestamp": 1776967897815340,
  "position": [-0.4760935342958236, 0.9328017370148517, 1.1295187986967496],
  "quaternion": [0.49762534881179205, 0.33433629307037327, -0.31294928395204247, 0.736648492179145],
  "velocity": [0.0961874724728649, -0.06710388339558249, -0.3273411039239402],
  "dt_s": 0.0,
  "valid": true,
  "confidence": 0.46102492276458507,
  "position_sigma_m": 0.013779501544708299,
  "rotation_sigma_deg": 11.7795015447083,
  "lost_frames": 0
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `26 passed`

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
tracking = summary['tracking']['waist']
print({k: summary[k] for k in ['frame_count', 'pair_count', 'frames_processed', 'poses_estimated']})
print({k: tracking[k] for k in ['valid', 'track_count', 'total_frames', 'reacquire_count', 'max_pose_flip_deg', 'rolling_confidence']})
print(tracking['prediction'])
PY
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- valid: `true`
- track count: `493`
- total frames: `497`
- reacquire count: `3`
- max pose flip candidate: `138.43282502436344 deg`
- rolling confidence: `0.46102492276458507`

### 読み取り

Phase 2 では採択ロジックを変えていないため、Phase 0/1 と同じ `493 / 497` valid、`reacquire_count = 3`、`max_pose_flip_deg = 138.43282502436344` が維持された。これは受入条件どおり。

一方で rolling confidence は `0.461` と高くない。直近 valid は安定しているが、Phase 0 で観測した大きな pose flip candidate を penalty として反映しているため。これは意図どおりで、Phase 3 以降ではこの confidence を使って「すぐ narrow gate に入らない」「reacquire 判定を慎重にする」方向へ進められる。

### 次に期待すること

- 次フェーズは **Phase 3 - Boot / Continue / Reacquire mode 分離** を推奨する。
- Phase 3 では `peek_prediction()` と `rolling_confidence` を使って、状態だけを `BOOT`, `CONTINUE`, `REACQUIRE`, `LOST` に分ける。
- 初期実装では candidate selection を大きく変えず、mode と diagnostics を追加して replay 指標が悪化しないことを確認する。
- その後、`CONTINUE` 時だけ prediction 近傍 candidate を優先するように段階的に進める。
- `rolling_confidence` が低い場合は narrow gate や single-ray continuation を許可しない方針にする。

### 次フェーズの実装可否

次フェーズも実装可能。Phase 2 で side-effect-free prediction と rolling confidence が入ったため、mode transition の材料は揃った。

注意点:

- Phase 3 の最初の PR では mode transition diagnostics までに留め、candidate pruning はまだ強く入れない方が安全。
- `predict()` は invalid pose を tracker に update する既存挙動で使われているため、置き換えるなら別フェーズで慎重にやる。
- replay の受入条件は `valid/reacquire/pose_flip` が Phase 2 から悪化しないこと。

## 2026-04-24 - Phase 3 Boot / Continue / Reacquire Mode Separation

### 実装結果

- `src/host/rigid.py` に `TrackMode` (`boot`, `continue`, `reacquire`, `lost`) と `TrackModeConfig` を追加した。
- `RigidBodyTracker.update()` が accepted / rejected measurement ごとに mode transition を更新するようにした。
- Phase 3 では pose acceptance はまだ変更していない。`RigidBodyEstimator` の candidate selection と RMS threshold は既存のまま。
- `boot -> continue` は 3 consecutive accepted、`continue -> reacquire` は 1 rejected、`reacquire -> continue` は prediction と整合する 2 consecutive accepted、`reacquire -> lost` は 5 consecutive rejected で遷移する。
- `lost -> boot` は strict shape-first candidate が再び accepted された時点で遷移する。現段階では shape-first acceptance は既存 Kabsch/RMS 判定を使う。
- diagnostics に `mode`, `mode_transition_count`, `last_mode_transition`, `mode_frame_count`, `mode_consecutive_accepts`, `mode_consecutive_rejects`, `last_mode_reason` を追加した。
- diagnostics に `last_position_innovation_m`, `max_position_innovation_m`, `last_rotation_innovation_deg`, `max_rotation_innovation_deg` を追加した。
- `tests/test_rigid_body_tracker_stats.py` に mode transition と reacquire innovation gate の unit test を追加した。
- `changelog.md` に Phase 3 の user-visible diagnostics 追加を記録した。

### Mode 遷移の現在仕様

```text
BOOT
  accepted x3 -> CONTINUE
  rejected    -> BOOT

CONTINUE
  accepted    -> CONTINUE
  rejected    -> REACQUIRE

REACQUIRE
  accepted and innovation within gate x2 -> CONTINUE
  accepted but large innovation          -> REACQUIRE
  rejected x5                            -> LOST

LOST
  accepted -> BOOT
  rejected -> LOST
```

現在の innovation gate は以下。

- position: `0.25 m`
- rotation: `120 deg`
- reacquire accepted count: `2`
- reacquire lost timeout: `5 frames`

この gate は Phase 3 では **mode transition 用** であり、candidate 自体を reject する gate ではない。既存挙動を壊さずに、Phase 4/5 で object-conditioned gating や 2D score を入れるための観測面を作った。

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `28 passed`

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
tracking = summary['tracking']['waist']
print({k: summary[k] for k in ['frame_count', 'pair_count', 'frames_processed', 'poses_estimated']})
print({k: tracking[k] for k in [
    'valid', 'mode', 'track_count', 'total_frames', 'reacquire_count',
    'max_pose_flip_deg', 'rolling_confidence', 'mode_transition_count',
    'last_mode_transition', 'last_mode_reason',
    'last_position_innovation_m', 'max_position_innovation_m',
]})
PY
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- valid: `true`
- final mode: `continue`
- track count: `493`
- total frames: `497`
- reacquire count: `3`
- mode transition count: `7`
- last mode transition: `reacquire->continue:reacquire_confirmed`
- last mode reason: `continue_measurement_accepted`
- max pose flip candidate: `138.43282502436344 deg`
- rolling confidence: `0.46102492276458507`
- last position innovation: `0.00034645534915246417 m`
- max position innovation: `0.0991694984463904 m`

### 読み取り

Phase 3 では採択ロジックを変えていないため、Phase 2 と同じ `493 / 497` valid、`reacquire_count = 3`、`max_pose_flip_deg = 138.43282502436344` が維持された。これは受入条件どおり。

一方で mode diagnostics は、ログ内で 7 回の transition が発生し、最後は `reacquire->continue` に戻っていることを示した。つまり、valid/lost の揺れは mode 層でも再現できており、次フェーズで「reacquire 中だけ候補採点を慎重にする」「continue 中だけ予測近傍を優先する」という制御を入れる準備ができた。

`max_position_innovation_m = 0.099m` は現在の `0.25m` gate 以内。位置 innovation だけでは今回の `138deg` class の flip を止めきれないため、次は 2D reprojection score と pattern ambiguity penalty を併用する必要がある。

### 次に期待すること

- 次フェーズは **Phase 4 - 2D Reprojection Scoring** を推奨する。
- Phase 4 では Phase 0.5 の `observations_by_camera` / `triangulated_points` を使い、Kabsch RMS だけでなく multi-camera 2D residual を pose hypothesis に付ける。
- まずは score を diagnostics と replay summary に出し、採択は変えない。
- 次に `REACQUIRE` mode の時だけ、3D RMS が良くても 2D residual / duplicate assignment / missing marker view が悪い候補を commit しないようにする。
- `CONTINUE` mode では prediction 近傍の candidate に temporal bonus を与えるが、`rolling_confidence` が低い時は narrow gate にしない。
- 今回の replay では位置 innovation が大きくないため、Phase 4 では rotation innovation と 2D reprojection が flip 抑制の主役になる見込み。

### 次フェーズの実装可否

次フェーズも実装可能。Phase 0.5 で 2D provenance、Phase 2 で prediction/confidence、Phase 3 で mode が揃ったので、Phase 4 の scoring 材料は揃っている。

注意点:

- まだ `process_context()` は導入していない。Phase 4 の最初は既存 `GeometryPipeline` output を `TrackingPipeline` から渡す薄い context wrapper を作るのが安全。
- 2D score は raw pixel と undistorted pixel を混ぜない。現状の triangulation provenance は raw/undistorted を両方持つので、score context に coordinate space を明示する。
- Phase 4 の初回は採択変更を入れず、score diagnostics の replay 差分だけを見る。reacquire 中の reject は次の小ステップに分けた方が戻しやすい。

## 2026-04-24 - Phase 4 2D Reprojection Scoring

### 実装結果

- `src/host/rigid.py` に 2D reprojection scoring diagnostics を追加した。
- `RigidBodyEstimator.process_context()` を追加し、既存 `process_points()` は 3D-only 互換 wrapper として残した。
- `TrackingPipeline` から `GeometryPipeline` の `camera_params` と `observations_by_camera` を estimator へ渡すようにした。fake estimator / 旧 estimator との互換のため、`process_context` がある場合だけ呼ぶ。
- 既存の pose acceptance は変更していない。Phase 4 初回では Kabsch RMS / cluster selection / valid 判定は Phase 3 と同じ。
- accepted pose から marker world positions を作り、各 camera の raw pixel blob に再投影して最近傍 residual を計算するようにした。
- `tracking.<name>.reprojection_score` に structured diagnostics を追加した。
- `tests/test_rigid_reprojection_scoring.py` を追加し、synthetic blobs で high score が出ること、旧 `process_points()` が 2D context なしで動くことを確認した。
- `changelog.md` に Phase 4 の diagnostics 追加を記録した。

### データ契約

`tracking.<name>.reprojection_score` は以下の形で出る。

```json
{
  "scored": true,
  "reason": "ok",
  "coordinate_space": "raw_pixel",
  "score": 0.8988986890582578,
  "mean_error_px": 0.8440602124269088,
  "p95_error_px": 1.1093540335311303,
  "max_error_px": 1.1127079208686501,
  "matched_marker_views": 8,
  "expected_marker_views": 8,
  "missing_marker_views": 0,
  "duplicate_assignment_count": 0,
  "unexpected_blob_count": 0,
  "camera_count": 2,
  "match_gate_px": 12.0
}
```

現在は `coordinate_space = raw_pixel` で scoring している。raw pixel projection は camera distortion も含めた `cv.projectPoints()` を使う。`undistorted_pixel` scoring 用の分岐も入れてあるが、pipeline 経由ではまだ raw pixel 固定。

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `30 passed`

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
tracking = summary['tracking']['waist']
print({k: summary[k] for k in ['frame_count', 'pair_count', 'frames_processed', 'poses_estimated']})
print({k: tracking[k] for k in [
    'valid', 'mode', 'track_count', 'total_frames', 'reacquire_count',
    'max_pose_flip_deg', 'rolling_confidence', 'mode_transition_count',
]})
print(tracking['reprojection_score'])
PY
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- valid: `true`
- final mode: `continue`
- track count: `493`
- total frames: `497`
- reacquire count: `3`
- mode transition count: `7`
- max pose flip candidate: `138.43282502436344 deg`
- rolling confidence: `0.46102492276458507`
- reprojection score: `0.8988986890582578`
- mean reprojection error: `0.8440602124269088 px`
- p95 reprojection error: `1.1093540335311303 px`
- matched marker views: `8 / 8`
- duplicate assignment count: `0`
- unexpected blob count: `0`

### 読み取り

Phase 4 でも採択ロジックを変えていないため、Phase 3 と同じ `493 / 497` valid、`reacquire_count = 3`、`max_pose_flip_deg = 138.43282502436344` が維持された。これは受入条件どおり。

最終 frame の 2D score は高く、`8 / 8` marker views が raw pixel residual 約 `0.84px` mean で一致している。つまり、この frame の pose は 2D 観測上はかなり整合している。一方で `max_pose_flip_deg` はまだ残っているので、flip は「最終 frame が悪い」というより、reacquire 近辺の一時的な hypothesis selection と pattern ambiguity の問題として扱うべき。

Phase 4 の重要な進捗は、これまで 3D RMS と valid/lost しか見えなかった rigid pose に対して、multi-camera 2D residual、missing views、duplicate blob assignment、unexpected blobs を同じ diagnostics 面で見られるようになったこと。次はこの score を frame ごと、特に `REACQUIRE` mode の accepted candidate に対して使う。

### 次に期待すること

- 次フェーズは **Phase 4.5 - Reacquire Candidate Guard** を挟むのが安全。
- いきなり全 mode で採択を変えず、`REACQUIRE` mode の accepted candidate だけに 2D score guard を入れる。
- guard の初期条件は緩めにし、例えば `matched_marker_views >= 6`, `mean_error_px <= 4.0`, `duplicate_assignment_count == 0`, `missing_marker_views <= 2` から始める。
- guard が reject した場合は invalid pose として tracker に流し、`invalid_reason = "reprojection_guard_rejected"` のように diagnostics で見えるようにする。
- Replay ではまず `valid/reacquire/pose_flip` が改善するか、悪化しないかを確認する。もし valid が落ちすぎる場合は guard を diagnostics-only に戻せる feature flag が必要。
- `score` の時系列が必要なので、次は replay summary に per-frame score dump または low-score event logging を追加すると原因分析がかなり楽になる。

### 次フェーズの実装可否

次フェーズも実装可能。Phase 4 で `process_context()` と `reprojection_score` が入ったため、`REACQUIRE` mode の candidate に対して score-based guard を入れる材料は揃った。

注意点:

- 現在の score は marker assignment を 2D 最近傍で見ているだけで、marker identity の permutation ambiguity までは解いていない。Phase 1 の ambiguity evaluator と組み合わせる必要がある。
- `reprojection_match_gate_px = 12px` は diagnostics 用に広め。採択 guard では mean/p95/duplicate/missing を併用し、単一 gate だけで落とさない。
- current replay の最終 frame は score が良いので、guard の効果を見るには reacquire 直後の accepted frames を per-frame でログ化した方がよい。

## 2026-04-24 - Phase 4.5 Reacquire Candidate Guard

### 実装結果

- `src/host/rigid.py` に `ReacquireGuardConfig` を追加した。
- `RigidBodyEstimator` が `REACQUIRE` mode の accepted candidate に対して、Phase 4 の `reprojection_score` と prediction innovation を使った guard 判定を行うようにした。
- デフォルトは shadow mode。`would_reject` を diagnostics に出すが、pose acceptance は変更しない。
- `ReacquireGuardConfig(enforced=True)` のときだけ、guard が fail した reacquire candidate を invalid pose として tracker に流す enforcement hook を追加した。
- enforcement reject 時は `invalid_reason = "reprojection_guard_rejected"` を残すようにした。
- `tracking.<name>.reacquire_guard` に latest guard result と累積 counter を追加した。
- `tests/test_rigid_reprojection_scoring.py` に shadow guard と enforcement guard の unit test を追加した。
- `changelog.md` に Phase 4.5 の shadow-first guard diagnostics 追加を記録した。

### Guard 条件

初期 guard は以下。single metric だけで落とさず、2D score と prediction innovation を合わせて評価する。

```python
guard_pass = (
    score.scored
    and score.matched_marker_views >= 6
    and score.missing_marker_views <= 2
    and score.mean_error_px <= 4.0
    and score.p95_error_px <= 8.0
    and score.duplicate_assignment_count == 0
    and position_innovation_m <= 0.25
    and rotation_innovation_deg <= 120.0
)
```

### データ契約

`tracking.<name>.reacquire_guard` は以下の形で出る。

```json
{
  "enabled": true,
  "enforced": false,
  "evaluated": false,
  "passed": true,
  "would_reject": false,
  "reason": "not_reacquire_mode",
  "thresholds": {
    "min_matched_marker_views": 6,
    "max_missing_marker_views": 2,
    "max_mean_reprojection_error_px": 4.0,
    "max_p95_reprojection_error_px": 8.0,
    "allow_duplicate_assignment": false,
    "max_position_innovation_m": 0.25,
    "max_rotation_innovation_deg": 120.0
  },
  "score": {},
  "position_innovation_m": 0.0,
  "rotation_innovation_deg": 0.0,
  "evaluated_count": 6,
  "would_reject_count": 5,
  "rejected_count": 0
}
```

最終 frame が `continue` の場合、latest result は `reason = "not_reacquire_mode"` になる。ただし累積 counter により、replay 中に `REACQUIRE` candidate が何回評価され、何回 shadow reject 相当だったかを確認できる。

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `32 passed`

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
tracking = summary['tracking']['waist']
print({k: summary[k] for k in ['frame_count', 'pair_count', 'frames_processed', 'poses_estimated']})
print({k: tracking[k] for k in [
    'valid', 'mode', 'track_count', 'total_frames', 'reacquire_count',
    'max_pose_flip_deg', 'rolling_confidence', 'mode_transition_count',
]})
print({k: tracking['reacquire_guard'][k] for k in [
    'enabled', 'enforced', 'evaluated_count',
    'would_reject_count', 'rejected_count', 'reason',
]})
PY
```

結果:

- input frames: `994`
- paired frames: `497`
- processed pairs: `497`
- estimated valid poses: `493`
- valid: `true`
- final mode: `continue`
- track count: `493`
- total frames: `497`
- reacquire count: `3`
- mode transition count: `7`
- max pose flip candidate: `138.43282502436344 deg`
- rolling confidence: `0.46102492276458507`
- guard enabled: `true`
- guard enforced: `false`
- guard evaluated count: `6`
- guard would reject count: `5`
- guard rejected count: `0`

### 読み取り

Phase 4.5 の default は shadow mode なので、Phase 4 と同じ `493 / 497` valid、`reacquire_count = 3`、`max_pose_flip_deg = 138.43282502436344` が維持された。これは受入条件どおり。

重要なのは、実ログ replay 中の `REACQUIRE` candidate が `6` 回評価され、そのうち `5` 回が guard 条件では `would_reject = true` になったこと。つまり、Phase 4 で見えていた 2D score / innovation は、少なくとも今回のログでは reacquire 中の怪しい candidate をかなり検出できている。

まだ enforcement は default off なので、実運用挙動は変えていない。だが synthetic unit test では enforcement on のとき、bad 2D candidate を invalid pose として流し、`invalid_reason = "reprojection_guard_rejected"` が出ることを確認した。

### 次に期待すること

- 次は **Phase 4.6 - Guard Enforcement Replay / Event Logging** を推奨する。
- まず replay harness か diagnostics event に per-frame `reacquire_guard` を残し、`would_reject_count = 5` の各 frame がどの score reason で落ちたかを確認する。
- 次に `ReacquireGuardConfig(enforced=True)` を replay harness から切り替えられるようにし、enforcement replay を Phase 4.5 shadow replay と比較する。
- enforcement replay の成功条件は、`pose_flip_deg` または `pose_jump_count` が改善し、`valid` frames が大きく減らないこと。
- 成功したら GUI/runtime config へ `reacquire_guard_enforced` を接続する。失敗したら guard 条件を reason ごとに緩め、特に `matched_marker_views` と `p95_error_px` を再調整する。

### 次フェーズの実装可否

次フェーズも実装可能。Phase 4.5 で shadow diagnostics と enforcement hook は入ったため、次は replay / config 接続の問題になる。

注意点:

- 現時点の `would_reject_count = 5` は有望だが、どの frame を reject するかの時系列がまだ見えていない。enforcement を本番 default にする前に per-frame guard event が必要。
- `rejected_count` は enforcement on のときだけ増える。shadow mode では `would_reject_count` を見る。
- enforcement は当面 `REACQUIRE` mode のみに限定する。`BOOT` や `CONTINUE` に広げるのは、stress log で安定性を確認してから。

## 2026-04-24 - Phase 4.5A-D Guard Replay / Event Logging / Go-No-Go

### 実装結果

- Phase 4.5A-D をまとめて実装した。独立した Phase 4.6 は作らず、Phase 4.5 の完了条件として扱う。
- `TrackingPipeline` に per-frame `reacquire_guard` event collection を追加した。
- `reacquire_guard_event_logging=True` のとき、live / replay logger に `event_type = "reacquire_guard"` を出せるようにした。
- `TrackingPipeline.get_reacquire_guard_events()` を追加し、replay summary が bounded event list を取り出せるようにした。
- `TrackingReplaySummary` に `reacquire_guard_events`, `reacquire_guard_summary`, `phase45_go_no_go` を追加した。
- `replay_tracking_log()` に `reacquire_guard_enforced`, `reacquire_guard_shadow_enabled`, `reacquire_guard_event_logging` を追加した。
- `compare_reacquire_guard_enforcement()` を追加し、同一ログで shadow replay と enforcement replay を連続実行して go/no-go 判定を返せるようにした。
- CLI に `--reacquire-guard-enforced`, `--disable-reacquire-guard-shadow`, `--reacquire-guard-event-logging`, `--compare-reacquire-guard-enforcement` を追加した。
- `tests/test_tracking_pipeline_diagnostics.py` に per-frame guard event logging の integration test を追加した。
- `tests/test_tracking_replay_harness.py` に Phase 4.5 summary と shadow/enforced comparison の test を追加した。
- `changelog.md` に Phase 4.5 replay / event logging / comparison の進捗を記録した。

### Replay summary contract

通常 replay summary には以下が追加される。

```json
{
  "reacquire_guard_events": [],
  "reacquire_guard_summary": {
    "event_count": 6,
    "would_reject_event_count": 5,
    "enforced_reject_event_count": 0,
    "reason_counts": {
      "insufficient_matched_marker_views": 4,
      "too_many_missing_marker_views": 4,
      "mean_reprojection_error_too_high": 4,
      "p95_reprojection_error_too_high": 4,
      "rotation_innovation_too_high": 3,
      "duplicate_assignment": 2,
      "ok": 1
    }
  },
  "phase45_go_no_go": {
    "decision": "pending_enforcement_replay"
  }
}
```

`--compare-reacquire-guard-enforcement` または `compare_reacquire_guard_enforcement()` では以下の形になる。

```json
{
  "shadow": {},
  "enforced": {},
  "phase45_go_no_go": {
    "decision": "no_go_adjust_thresholds",
    "valid_drop_ratio": 0.02231237322515213,
    "shadow_valid_frames": 493,
    "enforced_valid_frames": 482,
    "shadow_max_pose_flip_deg": 138.43282502436344,
    "enforced_max_pose_flip_deg": 132.52457336011963
  }
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `34 passed`

実ログで shadow / enforcement 比較も実行した。

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import compare_reacquire_guard_enforcement
comparison = compare_reacquire_guard_enforcement(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
print(comparison['phase45_go_no_go'])
PY
```

Shadow replay:

- frames processed: `497`
- valid poses: `493`
- reacquire count: `3`
- max pose flip candidate: `138.43282502436344 deg`
- pose jump count: `0`
- guard event count: `6`
- guard would reject event count: `5`
- enforced reject event count: `0`

Enforcement replay:

- frames processed: `497`
- valid poses: `482`
- reacquire count: `3`
- max pose flip candidate: `132.52457336011963 deg`
- pose jump count: `0`
- guard event count: `11`
- guard would reject event count: `11`
- enforced reject event count: `11`

Go / no-go:

- decision: `no_go_adjust_thresholds`
- valid drop frames: `11`
- valid drop ratio: `0.02231237322515213`
- max valid drop criterion: `0.02`
- pose flip not worse: `true`
- pose jump not worse: `true`

### 読み取り

Enforcement は max pose flip candidate を `138.43 deg` から `132.52 deg` へ少し改善した。一方、valid poses が `493` から `482` に落ち、低下率が `2.23%` になった。設定した暫定基準 `2%` を少し超えるため、現閾値のまま runtime default にするのはまだ早い。

ただし、guard が全く効いていないわけではない。shadow では `6` 件評価中 `5` 件が `would_reject`、enforcement では `11` 件 reject しており、reacquire 中の怪しい candidate を検出する力はある。次は落としすぎを避けるために、reason ごとの閾値調整が必要。

特に enforcement replay では `rotation_innovation_too_high` が `10` 件、`mean_reprojection_error_too_high` が `7` 件出ている。現 pattern は Phase 1 で自己対称性が強いと分かっているので、rotation innovation の単独 hard reject は少し強すぎる可能性がある。

### 次に期待すること

- 次は Phase 4.5 の閾値調整を小さく行う。新しい phase は作らず、Phase 4.5 の tuning として扱う。
- まず `rotation_innovation_too_high` を hard reject から warning / penalty に弱めるか、閾値を `120 deg` から `150-170 deg` へ緩める候補を replay 比較する。
- `matched_marker_views >= 6` と `missing_marker_views <= 2` は維持する。ここを緩めすぎると 2D score guard の意味が薄れる。
- `mean_error_px <= 4.0`, `p95_error_px <= 8.0` は、per-frame event を見て `5-6 px` / `10-12 px` 程度までの緩和候補を比較する。
- Go 条件は当面 `valid_drop_ratio <= 2%`, `pose_flip_deg not worse`, `pose_jump_count not worse` のままにする。
- 閾値調整後も go 判定が出ない場合、guard enforcement は default off / shadow-only のまま Phase 5 に進む。

### 次フェーズの実装可否

Phase 5 の実装には進める。ただし、Phase 4.5 enforcement を default on にする判断はまだ保留。

現状のおすすめ:

- Phase 4.5 guard は shadow/event logging を残す。
- Runtime default は shadow-only。
- Phase 5 に入る前に、1 回だけ threshold tuning replay を試す。
- tuning で `valid_drop_ratio <= 2%` に収まれば enforcement candidate として残す。
- 収まらなければ enforcement は off のまま、Phase 5 の object-conditioned gating で改善を狙う。

## 2026-04-24 - Phase 4.5 Guard Threshold Tuning

### 実装結果

- 実ログ `logs/tracking_gui.jsonl` に対して Phase 4.5 guard の閾値探索を行った。
- 探索対象は主に `max_rotation_innovation_deg`, `max_mean_reprojection_error_px`, `max_p95_reprojection_error_px`, `max_missing_marker_views`, `allow_duplicate_assignment`, `min_matched_marker_views`。
- 結果として、2D 側の閾値を緩めても valid drop はほぼ変わらず、flip 改善の有無は `max_rotation_innovation_deg` が支配的だった。
- `max_rotation_innovation_deg = 120 deg` は flip 改善が大きいが valid drop が `2.23%` で暫定基準 `2%` を超えた。
- `max_rotation_innovation_deg >= 139 deg` は valid drop が `1.42%` と軽いが、flip 改善が消えた。
- 最良バランスとして `max_rotation_innovation_deg = 136 deg` を採用した。
- `src/host/rigid.py` の `ReacquireGuardConfig.max_rotation_innovation_deg` default を `136.0` に更新した。
- `docs/10_in_progress/rigid_body_stabilization_plan.md` の Phase 4.5 guard 条件と feature flag 例も `136.0` に更新した。
- `changelog.md` に Phase 4.5 tuning の結果を記録した。

### 探索結果

Baseline shadow:

- valid poses: `493`
- max pose flip candidate: `138.43282502436344 deg`
- pose jump count: `0`

代表候補:

| max rotation innovation | valid poses | valid drop | valid drop ratio | max flip deg | flip improvement | rejected |
|---:|---:|---:|---:|---:|---:|---:|
| `120 deg` | `482` | `11` | `2.231%` | `132.52457336011963` | `5.908` | `11` |
| `135 deg` | `483` | `10` | `2.028%` | `134.0456763538021` | `4.387` | `10` |
| `136 deg` | `484` | `9` | `1.826%` | `135.53086180165727` | `2.902` | `9` |
| `137 deg` | `484` | `9` | `1.826%` | `135.53086180165727` | `2.902` | `9` |
| `138 deg` | `485` | `8` | `1.623%` | `137.00034352167532` | `1.432` | `8` |
| `139 deg` | `486` | `7` | `1.420%` | `138.43282502436344` | `0.000` | `7` |

`136 deg` と `137 deg` は同じ replay 結果だった。閾値として少し厳しい側の `136 deg` を採用する。

### 採用設定での Phase 4.5 comparison

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import compare_reacquire_guard_enforcement
c = compare_reacquire_guard_enforcement(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
print(c['phase45_go_no_go'])
PY
```

Shadow replay:

- valid poses: `493`
- max pose flip candidate: `138.43282502436344 deg`
- pose jump count: `0`
- guard would reject event count: `5`

Enforcement replay with `136 deg`:

- valid poses: `484`
- valid drop frames: `9`
- valid drop ratio: `0.018255578093306288`
- max pose flip candidate: `135.53086180165727 deg`
- pose jump count: `0`
- enforced reject count: `9`

Go / no-go:

- decision: `go_candidate`
- reason: `enforcement did not worsen replay metrics beyond configured thresholds`
- max valid drop criterion: `0.02`
- pose flip not worse: `true`
- pose jump not worse: `true`

### 読み取り

今回のログでは、`136 deg` が「valid を落としすぎず、flip も少し改善する」バランスだった。`120 deg` の方が flip 改善は大きいが、valid drop が基準を超える。`139 deg` 以上は安全だが flip 改善が出ない。

ただし、この結論は現在の短い実ログに対する最適化。runtime default で enforcement を有効化する前に、静止ログと stress log で false lost が増えないことを確認したい。

### 次に期待すること

- Phase 4.5 enforcement は `go_candidate` になったが、まだ本番 default は shadow-only が安全。
- 次に実機の静止ログ / stress log を取り、同じ `compare_reacquire_guard_enforcement()` で `136 deg` 設定を評価する。
- stress log でも `valid_drop_ratio <= 2%`, `pose_flip_deg not worse`, `pose_jump_count not worse` なら、runtime config から enforcement を明示的に有効化する候補にできる。
- Phase 5 には進める。Phase 5 では object-conditioned gating の default を diagnostics-only にし、Phase 4.5 enforcement と同時に強くしすぎないようにする。

## 2026-04-24 - Phase 5 Object-conditioned 2D Gating Diagnostics

### 実装結果

- `src/host/rigid.py` に `ObjectGatingConfig` と object-conditioned gating diagnostics を追加した。
- `RigidBodyEstimator.evaluate_object_conditioned_gating()` を追加し、tracker の `peek_prediction(timestamp)` から predicted marker world positions を作るようにした。
- 各 camera へ predicted marker を raw pixel projection し、marker x blob の cost matrix を作って Hungarian assignment で one-to-one に割り当てるようにした。
- Pixel gate は prediction uncertainty から `pixel_min=4px` と `pixel_max=16px` の範囲に clamp する。
- `single_ray_confidence_min=0.75` 未満では single-ray candidate を許可しない diagnostics にした。
- Phase 5 初回では generic triangulation の入力は変更していない。object-conditioned gating は diagnostics-only。
- `TrackingPipeline` が triangulation 前に object-conditioned gating を評価し、latest triangulation snapshot と diagnostics に `object_gating` を載せるようにした。
- `TrackingPipeline.get_object_gating_events()` を追加し、per-frame object gating event を replay summary へ出せるようにした。
- `TrackingReplaySummary` に `object_gating_events` と `object_gating_summary` を追加した。
- `tests/test_rigid_reprojection_scoring.py` に、2 camera x 4 marker の predicted windows が 8 marker-view assignment になる unit test を追加した。
- `changelog.md` に Phase 5 diagnostics の進捗を記録した。

### データ契約

`tracking.<name>.object_gating` は以下の形で出る。

```json
{
  "enabled": true,
  "enforced": false,
  "evaluated": true,
  "reason": "ok",
  "mode": "continue",
  "prediction_valid": true,
  "confidence": 0.4613978082521978,
  "pixel_gate_px": 6.123284450728988,
  "camera_count": 2,
  "marker_count": 4,
  "candidate_window_count": 8,
  "assigned_marker_views": 8,
  "unmatched_marker_views": 0,
  "duplicate_assignment_count": 0,
  "markers_with_two_or_more_rays": 4,
  "markers_with_one_ray": 0,
  "single_ray_candidates": 0,
  "generic_fallback_blob_count": 0,
  "allow_single_ray": false,
  "per_marker_ray_count": [2, 2, 2, 2]
}
```

Replay summary には `object_gating_summary` が追加される。

```json
{
  "event_totals": {
    "event_count": 496,
    "assigned_marker_views": 3773,
    "candidate_window_count": 3968,
    "markers_with_two_or_more_rays": 1861,
    "single_ray_candidates": 4,
    "generic_fallback_blob_count": 178
  }
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py
```

結果: `35 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
s = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
).to_dict()
print(s['object_gating_summary'])
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- max pose flip candidate: `138.43282502436344 deg`
- object gating events: `496`
- candidate windows: `3968`
- assigned marker views: `3773`
- markers with 2+ rays: `1861`
- single-ray candidates: `4`
- generic fallback blobs: `178`
- latest frame assigned marker views: `8 / 8`
- latest frame markers with 2+ rays: `4 / 4`
- latest frame fallback blobs: `0`

### 読み取り

Phase 5 初回は diagnostics-only なので、Phase 4.5 shadow と同じ `493 / 497` valid を維持した。これは意図どおり。

一方、実ログのほぼ全 frame で predicted marker windows が評価され、累計 `3773 / 3968` marker-view assignment が成立した。最終 frame では `8 / 8` assignment、全 marker が 2 ray 以上を持ち、fallback blob も `0` だった。これは object-conditioned gating が次の constrained triangulation へ進めるだけの材料をかなり持っていることを示す。

ただし single-ray candidates は累計 `4` と少ない。今回のログでは 2 camera 両方で見えている frame が多いため、single-ray continuation の効果はこのログだけでは評価しづらい。遮蔽 stress log が必要。

### 次に期待すること

- 次は Phase 5B として、object-conditioned gating から `source = "rigid_hint"` の constrained triangulated points を作る。ただし generic triangulation は fallback として残す。
- 最初は constrained points を commit には使わず、generic points と side-by-side で `rigid_hint_points`, `generic_points`, `fallback_blob_count` を replay summary に出す。
- `confidence < 0.75` のときは single-ray virtual marker を作らない方針を維持する。
- `single_ray_candidates` の評価には、片 camera 遮蔽を含む stress log を取る必要がある。
- Phase 4.5 enforcement と Phase 5 constrained triangulation を同時に強くしない。次の実装でも default は diagnostics-only にするのが安全。

## Phase 5B - Diagnostics-only rigid-hint triangulation

### 実装結果

- `src/host/geo.py` に `Triangulator.triangulate_rigid_hints()` を追加した。
- Phase 5A の `object_gating.per_camera.assignments` を、同フレームで生成済みの `BlobObservation2D` に引き直し、`marker_idx` ごとに 2 ray 以上あるものだけを `source = "rigid_hint"` の `TriangulatedPoint` として3D化するようにした。
- `GeometryPipeline.process_paired_frames()` は既存の `points_3d` / `triangulated_points` を変更せず、別キーとして `rigid_hint_points_3d`, `rigid_hint_triangulated_points`, `rigid_hint_reprojection_errors`, `rigid_hint_quality` を返す。
- `TrackingPipeline` は triangulation 時に `object_gating` を渡し、latest triangulation snapshot と diagnostics に `rigid_hint` の結果を残す。
- `TrackingPipeline.get_rigid_hint_events()` と replay summary の `rigid_hint_summary` を追加し、実ログ全体で hint lane の accepted / rejected を集計できるようにした。
- 採択ロジックはまだ変えていない。rigid pose estimation は従来どおり generic `points_3d` を使う。
- `tests/test_geo_blob_assignment.py` に、shuffled blob order でも object-gating assignment から marker-indexed rigid hint 3D点が4点出る test を追加した。
- `changelog.md` に Phase 5B の user-visible diagnostics 進捗を記録した。

### データ契約

`GeometryPipeline.process_paired_frames()` の返り値に以下が追加される。

```json
{
  "rigid_hint_points_3d": [[...]],
  "rigid_hint_triangulated_points": [
    {
      "source": "rigid_hint",
      "rigid_name": "waist",
      "marker_idx": 0,
      "contributing_rays": 2,
      "reprojection_errors_px": [...]
    }
  ],
  "rigid_hint_quality": {
    "reason": "ok",
    "rigid_count": 1,
    "candidate_markers": 4,
    "markers_with_two_or_more_rays": 4,
    "single_ray_candidates": 0,
    "accepted_points": 4,
    "rejected_markers": 0,
    "invalid_assignments": 0
  }
}
```

Replay summary には `rigid_hint_summary` が追加される。

```json
{
  "totals": {
    "event_count": 496,
    "candidate_markers": 1912,
    "markers_with_two_or_more_rays": 1861,
    "single_ray_candidates": 51,
    "accepted_points": 1859,
    "rejected_markers": 2,
    "invalid_assignments": 0
  }
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py
```

結果: `36 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
s = summary.to_dict()
print(s['object_gating_summary'])
print(s['rigid_hint_summary'])
print(s['geometry'].get('rigid_hint'))
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- object gating events: `496`
- object gating assigned marker views: `3773 / 3968`
- object gating markers with 2+ rays: `1861`
- object gating generic fallback blobs: `178`
- rigid hint events: `496`
- rigid hint candidate markers: `1912`
- rigid hint markers with 2+ rays: `1861`
- rigid hint accepted points: `1859`
- rigid hint rejected markers: `2`
- rigid hint invalid assignments: `0`
- rigid hint reprojection mean over events: `0.3226566969905932 px`
- rigid hint reprojection p95 over events: `0.5830684700602324 px`
- latest frame rigid hint accepted points: `4 / 4`
- latest frame rigid hint reprojection mean: `0.6255842174435337 px`

### 読み取り

Phase 5B は diagnostics-only なので、Phase 5A と同じ `493 / 497` valid を維持した。これは意図どおり。

重要なのは、2D gate で2 ray以上そろった `1861` marker のうち、`1859` 点が rigid hint triangulation まで通ったこと。rejected は `2` のみで、invalid assignment は `0`。このログでは object-conditioned matching の対応表が3D化に十分使える品質を持っている。

一方、`single_ray_candidates` は object-gating event 側では `4` だったが、rigid hint candidate marker 側では `51` になった。これは Phase 5A の `single_ray_candidates` が confidence threshold を通ったものだけを数えるのに対し、Phase 5B の rigid hint summary は「1 rayしか持たない marker」を素朴に数えているため。採択候補としての single-ray continuation へ進める前に、ここは naming を `one_ray_markers` と `single_ray_commit_candidates` に分けると誤読が減る。

### 次に期待すること

- 次は Phase 5C として、`rigid_hint_points_3d` を pose estimator へ side-by-side 入力し、generic pose と rigid-hint pose の両方を score する。ただし commit はまだ generic 優先にする。
- `single_ray_candidates` の名前を整理し、virtual marker を作れる候補と単なる1 ray観測を分ける。
- Phase 5C の go/no-go は、同ログで `rigid_hint_pose.valid >= generic_pose.valid`、flip candidate が増えないこと、reprojection score p95 が悪化しないことを条件にする。
- 片 camera 遮蔽 stress log が取れたら、single-ray virtual marker の効果をそこで初めて評価する。

## Phase 5C - Rigid-hint pose comparison / Phase 6 readiness

### 実装結果

- `RigidBodyEstimator.process_context()` に `rigid_hint_triangulated_points` を渡せるようにした。
- `source = "rigid_hint"` かつ marker index 付きの3D点から、marker correspondence 固定の Kabsch pose を diagnostics-only で解くようにした。
- 現行の generic pose 採択は変更していない。`rigid_hint_pose` は `TrackingStatus` に比較診断として載るだけ。
- `rigid_hint_pose` は generic pose と同じ `reprojection_score` で評価し、`score_delta`, `position_delta_m`, `rotation_delta_deg`, `would_improve_score` を出す。
- `TrackingPipeline.get_rigid_hint_pose_events()` を追加し、replay summary に `rigid_hint_pose_summary` を追加した。
- `rigid_hint_pose_summary.totals` では、`hint_pose_adoption_ready` と `phase6_ready` を分けた。
- `hint_pose_adoption_ready` は「hint poseをそのまま採択してよいか」の厳しめ判定。
- `phase6_ready` は「Phase 6 の subset RANSAC / weighted solve を実装・検証する入力がそろったか」の判定。
- `tests/test_rigid_reprojection_scoring.py` に、rigid hint pose が generic pose と side-by-side で評価され、採択を変えない test を追加した。
- `tests/test_tracking_replay_harness.py` に、`rigid_hint_pose_summary` の replay summary test を追加した。
- `changelog.md` に Phase 5C / Phase 6 readiness の進捗を記録した。

### データ契約

`tracking.<name>.rigid_hint_pose` は以下の形で出る。

```json
{
  "evaluated": true,
  "reason": "ok",
  "diagnostics_only": true,
  "valid": true,
  "generic_valid": true,
  "would_improve_score": false,
  "candidate_points": 4,
  "observed_markers": 4,
  "real_ray_count": 8,
  "virtual_marker_count": 0,
  "rms_error_m": 0.0015512782290946932,
  "generic_rms_error_m": 0.0015512782290946932,
  "score_delta": 0.0,
  "position_delta_m": 0.0,
  "rotation_delta_deg": 0.0,
  "marker_indices": [0, 1, 2, 3]
}
```

Replay summary には `rigid_hint_pose_summary` が追加される。

```json
{
  "totals": {
    "event_count": 472,
    "valid_count": 459,
    "generic_valid_count": 469,
    "would_improve_score_count": 6,
    "hint_pose_adoption_ready": false,
    "phase6_ready": true
  }
}
```

### 検証

OpenCV が入っている Python 3.13 の一時 venv で検証した。

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py
```

結果: `37 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
s = summary.to_dict()
print(s['rigid_hint_pose_summary']['totals'])
print(s['rigid_hint_pose_summary']['by_rigid'])
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- rigid hint pose events: `472`
- rigid hint pose valid: `459`
- generic pose valid on same events: `469`
- rigid hint pose would improve score: `6`
- rigid hint pose score delta mean: `0.004390109475119149`
- rigid hint pose p95 reprojection p95: `1.2200366323622638 px`
- max position delta: `0.023933041220510813 m`
- max rotation delta: `107.54560432521374 deg`
- reasons: `ok = 459`, `insufficient_rigid_hint_points = 13`
- `hint_pose_adoption_ready = false`
- `phase6_ready = true`

### 読み取り

Phase 5C の結果、Phase 6 は実装可能な状態になった。理由は、generic pose と rigid-hint pose を同じ replay 経路・同じ 2D score で横並び比較でき、Phase 6 の hypothesis selection に必要な candidate pose / score / delta / marker coverage がそろったため。

ただし、rigid hint pose をそのまま採択する段階ではない。`valid_count` は `459` で、同じ event 上の `generic_valid_count = 469` を下回る。また、最大 rotation delta が `107.5 deg` あるため、subset RANSAC / weighted solve なしで hint pose を commit すると flip を拾う危険が残る。

これは Phase 6 に進む理由としてはむしろ健全。Phase 6 では、複数 subset hypothesis を作り、Phase 4 の 2D score と Phase 5C の generic-vs-hint delta を使って、flip しそうな hypothesis を落とす。

### 次に期待すること

- Phase 6 は実装可能。
- Phase 6 の first step は、3-marker / 4-marker subset hypothesis を diagnostics-only で生成し、現行 generic pose と比較すること。
- 採択変更はまだしない。まず `subset_hypothesis_summary` に candidate 数、best score、2nd score、margin、rejected-by-ambiguity、rejected-by-2D-score を出す。
- `hint_pose_adoption_ready = false` のため、Phase 6 実装後も最初は diagnostics-only で進めるのが安全。

## Phase 6 - Subset RANSAC and weighted rigid solve diagnostics

### 実装結果

- `SubsetSolveConfig` を追加し、Phase 6 の subset hypothesis 生成を default diagnostics-only で制御できるようにした。
- 3-marker / 4-marker subset から pose hypothesis を作り、generic points と `rigid_hint` marker-indexed points の両方を候補源にした。
- 各 hypothesis を Phase 4 の 2D reprojection score で評価し、`best`, `second`, `margin`, `score_delta`, `flip_risk_count`, `rejected_by_2d_score`, `rejected_by_rms`, `rejected_by_ambiguity` を出すようにした。
- `KabschEstimator.estimate_weighted()` を追加し、best subset の weighted solve summary を `subset_hypothesis.weighted_solve` に出すようにした。
- Pattern 内の subset ambiguity は marker distance profile の近接で判定し、曖昧な subset を rankable candidate から外すようにした。
- `RigidBodyTracker` に `subset_hypothesis` diagnostics を保持させ、`TrackingPipeline.get_subset_hypothesis_events()` と replay summary の `subset_hypothesis_summary` を追加した。
- 採択ロジックはまだ変えていない。pose commit は従来の generic path を維持する。
- `tests/test_rigid_reprojection_scoring.py` に、正解4点 + noise6点でも subset hypothesis が高score candidateを出す test を追加した。
- `tests/test_tracking_replay_harness.py` に、`subset_hypothesis_summary.phase6_complete` の replay summary test を追加した。
- `changelog.md` に Phase 6 の user-visible diagnostics 進捗を記録した。

### データ契約

`tracking.<name>.subset_hypothesis` は以下の形で出る。

```json
{
  "evaluated": true,
  "reason": "ok",
  "diagnostics_only": true,
  "candidate_count": 125,
  "valid_candidate_count": 24,
  "rejected_by_ambiguity": 0,
  "rejected_by_2d_score": 101,
  "rejected_by_rms": 4,
  "flip_risk_count": 94,
  "truncated": false,
  "best_score": 0.8990512341102239,
  "second_score": 0.8988986890582578,
  "margin": 0.00015254505196615753,
  "generic_score": 0.8988986890582578,
  "score_delta": 0.00015254505196615753,
  "subset_adoption_ready": false,
  "best": {
    "source": "rigid_hint_subset",
    "marker_indices": [0, 1, 2, 3],
    "score": 0.8990512341102239,
    "p95_error_px": 1.1101003280511064,
    "rotation_delta_deg": 0.04732950064258966
  },
  "weighted_solve": {
    "valid": true,
    "source": "rigid_hint_subset",
    "observed_markers": 4,
    "rms_error_m": 0.0015591280938460513
  }
}
```

Replay summary には `subset_hypothesis_summary` が追加される。

```json
{
  "totals": {
    "event_count": 493,
    "candidate_count": 60111,
    "valid_candidate_count": 11241,
    "rejected_by_2d_score": 48870,
    "rejected_by_rms": 1886,
    "flip_risk_count": 45326,
    "subset_adoption_ready_count": 0,
    "truncated_count": 0,
    "phase6_complete": true
  }
}
```

### 検証

Phase 6 周辺の主テスト:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `38 passed`

周辺 runtime / GUI backend / clusterer test:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_clusterer.py -q
```

結果: `52 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
s = summary.to_dict()
print(s['pipeline_stage_ms'].get('rigid_ms'))
print(s['subset_hypothesis_summary']['totals'])
print(s['subset_hypothesis_summary']['by_rigid'])
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- rigid_ms mean: `30.909575 ms`
- rigid_ms p95: `32.266792 ms`
- rigid_ms max: `70.573209 ms`
- triangulation_ms mean: `2.686855025 ms`
- subset events: `493`
- subset candidate count: `60111`
- valid subset candidate count: `11241`
- rejected by 2D score: `48870`
- rejected by RMS: `1886`
- rejected by ambiguity: `0`
- flip risk count: `45326`
- truncated count: `0`
- best source counts: `generic_subset = 209`, `rigid_hint_subset = 284`
- margin mean: `0.0024109287281096044`
- margin p95: `0.004200195536867568`
- score delta mean: `0.017931867799784937`
- score delta p95: `0.08085246753707642`
- best p95 reprojection p95: `1.439467798673221 px`
- best rotation delta p95: `31.173432782515803 deg`
- subset adoption ready count: `0`
- `phase6_complete = true`

### 読み取り

Phase 6 は完了状態。subset hypothesis 生成、2D score による比較、flip-risk 診断、weighted solve summary、same-path replay summary まで通っており、実ログでも `phase6_complete = true`、`truncated_count = 0` になった。

一方、`subset_adoption_ready_count = 0` なので、採択ONはまだ早い。理由は明確で、best / second の margin がかなり薄い。latest frame でも best と second の差は `0.0001525` しかない。Phase 6 の診断は「候補を作れる」ことだけでなく、「採択判断には margin が足りない」ことも示している。

`flip_risk_count = 45326` は大きいが、これは候補全体の中に危険な permutation / subset が大量にあるという意味。2D score と margin gate がなければ flip を拾う危険が高い。Phase 6 の diagnostics-only default は妥当。

runtime cost は replay で `rigid_ms p95 = 32.27 ms`。候補生成が入ったぶん重くなっているため、採択ONの前に candidate pruning が必要。

### 次に期待すること

- 次は Phase 6.5 として、採択変更ではなく candidate pruning と margin 改善を行うのが安全。
- 具体的には、generic subset の全 permutation を減らし、`rigid_hint_subset` と prediction近傍 subset を優先する。
- `subset_adoption_ready` を増やすには、best / second margin を厚くする必要がある。2D score だけでなく、temporal innovation、marker coverage、source priority を combined score に入れる。
- `rigid_ms p95` を下げるため、candidate上限ではなく候補生成前の枝刈りを入れる。`truncated_count = 0` なので、現状は上限不足ではなく候補を作りすぎている。
- Phase 6 の採択ONはまだ行わない。`subset_adoption_ready_ratio` と `rigid_ms p95` が改善してから、shadow enforcement replay を挟む。

## Phase 6.5 - Candidate pruning and combined subset score

### 実装結果

- `SubsetSolveConfig` に `max_generic_observed_subsets`, `prediction_gate_m`, `source_priority_bonus`, `coverage_weight`, `temporal_penalty_weight`, `flip_penalty` を追加した。
- `generic_subset` は全 observed subset を展開せず、generic pose / prediction 近傍で説明できる observed subset を優先し、`max_generic_observed_subsets = 24` に絞るようにした。
- `generic_subset` の各 marker permutation は、generic pose から見て平均 `prediction_gate_m = 0.08m` を超える場合、pose solve 前に prune するようにした。
- `combined_score` を追加し、raw 2D score だけでなく marker coverage、`rigid_hint_subset` source bonus、temporal innovation penalty、flip penalty を加味して best / second を rank するようにした。
- `subset_hypothesis` に `pruned_candidate_count`, `best_combined_score`, `second_combined_score`, `combined_margin` を追加した。
- `TrackingPipeline` の `subset_hypothesis_events` と replay summary の `subset_hypothesis_summary` に pruning / combined margin metrics を追加した。
- 採択ロジックはまだ変更していない。Phase 6.5 も diagnostics-only。
- `changelog.md` に Phase 6.5 の user-visible diagnostics 進捗を記録した。

### データ契約

`tracking.<name>.subset_hypothesis` には以下が追加される。

```json
{
  "candidate_count": 77,
  "pruned_candidate_count": 48,
  "best_score": 0.8990512341102239,
  "second_score": 0.8937058535802875,
  "best_combined_score": 1.0188161721326305,
  "second_combined_score": 0.9887152621121309,
  "margin": 0.005345380529936383,
  "combined_margin": 0.030100910020499638,
  "subset_adoption_ready": true,
  "best": {
    "source": "rigid_hint_subset",
    "combined_score": 1.0188161721326305,
    "coverage": 1.0,
    "temporal_penalty": 0.0013058998755184183,
    "source_bonus": 0.04
  }
}
```

Replay summary の `subset_hypothesis_summary.totals` には以下が入る。

```json
{
  "candidate_count": 37714,
  "pruned_candidate_count": 22397,
  "valid_candidate_count": 6844,
  "subset_adoption_ready_count": 283,
  "subset_adoption_ready_ratio": 0.5740365111561866,
  "combined_margin_summary": {
    "mean": 0.028525272244270556,
    "p95": 0.03352911901724043
  }
}
```

### 検証

Phase 6 / 6.5 周辺の主テスト:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `38 passed`

周辺 runtime / GUI backend / clusterer test:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_clusterer.py -q
```

結果: `52 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
s = summary.to_dict()
print(s['pipeline_stage_ms'].get('rigid_ms'))
print(s['subset_hypothesis_summary']['totals'])
print(s['subset_hypothesis_summary']['by_rigid'])
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- rigid_ms mean: `21.752600879166668 ms`
- rigid_ms p95: `23.396334 ms`
- rigid_ms max: `74.928625 ms`
- subset events: `493`
- subset candidate count: `37714`
- pruned candidate count: `22397`
- valid subset candidate count: `6844`
- rejected by 2D score: `30870`
- rejected by RMS: `940`
- flip risk count: `23884`
- truncated count: `0`
- best source counts: `generic_subset = 27`, `rigid_hint_subset = 458`, empty best = `8`
- raw margin mean: `0.003803637575004811`
- raw margin p95: `0.00980447992408684`
- combined margin mean: `0.028525272244270556`
- combined margin p95: `0.03352911901724043`
- best p95 reprojection p95: `1.3162558093085044 px`
- best rotation delta p95: `0.2712228983038488 deg`
- subset adoption ready count: `283`
- subset adoption ready ratio: `0.5740365111561866`
- `phase6_complete = true`

Phase 6 からの改善:

- candidate count: `60111 -> 37714`
- valid candidate count: `11241 -> 6844`
- flip risk count: `45326 -> 23884`
- rigid_ms p95: `32.266792 ms -> 23.396334 ms`
- raw margin p95: `0.004200195536867568 -> 0.00980447992408684`
- combined margin p95: `0.03352911901724043`
- subset adoption ready count: `0 -> 283`

### 読み取り

Phase 6.5 は完了状態。候補数、flip-risk候補、runtime cost が下がり、combined score によって best / second margin が厚くなった。`subset_adoption_ready_count = 283` まで増えたので、Phase 6 の「採択ONはまだ全く無理」から、「一部 frame では採択候補として成立する」状態に進んだ。

ただし、採択ONはまだ default にしない。`subset_adoption_ready_ratio = 0.574` で、全 frame に対して十分ではない。また empty best が `8` frame 残っており、max rigid time も `74.9ms` とスパイクがある。shadow enforcement replay を挟まずに commit path を変えるのはまだ危険。

今回の combined score は `rigid_hint_subset` を強めに優先した。best source は `458 / 493` が `rigid_hint_subset` になったため、Phase 5 の object-conditioned gating がかなり効いている。一方で、generic subset が best の frame も `27` あり、fallback lane は残すべき。

### 次に期待すること

- 次は Phase 6.6 または Phase 7 前の guard として、`subset_adoption_ready` の shadow enforcement replay を入れるのが安全。
- default 採択ONの前に、`subset_adoption_ready` の frame だけ仮採択した場合の valid/lost、flip、rigid_ms を replay比較する。
- empty best `8` frame の理由を分解する。`reason_counts` や no-rankable frame の camera/blob状況を追加するとよい。
- runtime max `74.9ms` のスパイクを潰す。候補数だけでなく、per-frame candidate generation time / scoring time を分けて計測する。
- `subset_adoption_ready_ratio` が `0.9+`、かつ replay flip悪化なし、rigid_ms p95が目標内になってから採択ONを検討する。

## Phase 6.6 - Subset adoption shadow replay

### 実装結果

- `subset_hypothesis.best` に pose payload を残し、event 側に `generic_valid`, `best_position_delta_m`, `best_rotation_delta_deg` を追加した。
- `TrackingReplaySummary.subset_hypothesis_summary.totals.subset_adoption_shadow` を追加した。
- shadow replay は実際の commit path を変えず、`subset_adoption_ready == true` の frame だけ仮採択した場合の valid event delta、score worsening、flip worsening、jump worsening を集計する。
- `ready_for_enforcement_replay` は adoption coverage が `0.90+` かつ score/flip/jump 悪化が0の場合だけ true にする。
- `tests/test_tracking_replay_harness.py` に subset adoption shadow summary の test を追加した。
- `changelog.md` に subset adoption shadow replay の進捗を記録した。

### データ契約

`subset_hypothesis_summary.totals.subset_adoption_shadow` は以下の形で出る。

```json
{
  "decision": "keep_shadow_until_coverage_improves",
  "reason": "shadow adoption did not worsen candidate deltas, but coverage is still too low",
  "event_count": 493,
  "adopted_event_count": 283,
  "adoption_ratio": 0.5740365111561866,
  "baseline_valid_events": 493,
  "shadow_valid_events": 493,
  "valid_event_delta": 0,
  "score_worse_count": 0,
  "flip_worse_count": 0,
  "jump_worse_count": 0,
  "ready_for_enforcement_replay": false
}
```

### 検証

Phase 6 / 6.5 / 6.6 周辺の主テスト:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `38 passed`

実ログ replay:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
summary = replay_tracking_log(
    log_path='logs/tracking_gui.jsonl',
    calibration_path='calibration',
    patterns=['waist'],
    rigids_path='calibration/tracking_rigids.json',
)
s = summary.to_dict()
print(s['subset_hypothesis_summary']['totals']['subset_adoption_shadow'])
PY
```

結果:

- adopted event count: `283`
- adoption ratio: `0.5740365111561866`
- baseline valid events: `493`
- shadow valid events: `493`
- valid event delta: `0`
- score worse count: `0`
- flip worse count: `0`
- jump worse count: `0`
- ready for enforcement replay: `false`
- decision: `keep_shadow_until_coverage_improves`

### 読み取り

今の `logs/tracking_gui.jsonl` では、`subset_adoption_ready` frame だけ仮採択しても score / flip / jump の悪化は見えなかった。これは良いサイン。

ただし adoption ratio は `0.574` で、採択対象がまだ約57%に留まる。baseline valid event と shadow valid event も同じ `493` なので、このログでは valid/lost 改善はまだ示せていない。したがって default 採択ONや enforcement replay へはまだ進めず、shadow 継続が妥当。

この結果は「subset adoption が危険」というより、「subset adoption が効く frame は安全そうだが、coverage が足りず、今の短いログだけでは採択ONの価値がまだ弱い」という読み取り。

### 次に期待すること

- 次は coverage 改善。`subset_adoption_ready_ratio` を `0.9+` に近づける。
- empty best / no-rankable frame の理由を event に出し、残り frame がなぜ adoption_ready にならないか分解する。
- 片 camera 遮蔽、表裏flipが出やすい stress log で同じ shadow summary を確認する。
- adoption ratio が上がり、score/flip/jump worse が0のままなら、次に subset adoption enforcement replay を追加する。

## Phase 6.6 Review Fixes - Reacquire consistency / one-to-one scoring

### 実装結果

- `src/host/rigid.py` の `REACQUIRE` mode で、large innovation candidate を `reacquire_consecutive_accepts` に数えないようにした。`CONTINUE` へ戻るには、prediction と整合する candidate が連続で必要になった。
- `src/host/rigid.py` の partial-marker correspondence を、world/body 座標の直接最近傍ではなく、reference subset と permutation を Kabsch residual で選ぶ pose-invariant な探索にした。
- `src/host/rigid.py` の 2D reprojection scoring を、camera ごとの nearest blob 重複許容から Hungarian の one-to-one assignment に変えた。
- `src/host/tracking_replay_harness.py` の subset adoption shadow 集計で、`generic_valid` 欠損 event を baseline valid と誤カウントしないようにした。
- `tests/test_rigid_body_tracker_stats.py`, `tests/test_rigid_reprojection_scoring.py`, `tests/test_tracking_replay_harness.py` に regression test を追加した。
- `changelog.md` に review fix の user-visible outcome を記録した。

### 検証

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `42 passed`

周辺 runtime / GUI backend / clusterer test:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_clusterer.py -q
```

結果: `52 passed`

実ログ replay:

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
print(summary['tracking']['waist']['max_pose_flip_deg'])
print(summary['subset_hypothesis_summary']['totals']['subset_adoption_shadow'])
PY
```

結果:

- frames processed: `497`
- valid poses: `493`
- max pose flip candidate: `8.719878277400714 deg`
- max pose jump: `0.006559166867892473 m`
- subset adopted event count: `286`
- subset adoption ratio: `0.5801217038539553`
- score worse count: `0`
- flip worse count: `0`
- jump worse count: `0`
- decision: `keep_shadow_until_coverage_improves`

### 読み取り

今回の修正はリサーチ/計画の「reacquire は数フレームだけ multi-hypothesis を許して margin が付いたら commit」「2D scoring は camera ごとに one-to-one assignment」「partial subset は template geometry で評価する」という方針に沿っている。

特に `max_pose_flip_deg` が `138.43282502436344 deg` から `8.719878277400714 deg` まで下がり、`493 / 497` valid は維持された。これは、以前の large innovation candidate が reacquire confirmation に混ざっていた問題が replay 上でも flip 候補を増やしていたことを示す。

一方、subset adoption ratio はまだ `0.580` で、default 採択ONの条件 `0.9+` には届いていない。結論は Phase 6.6 と同じく、shadow 継続が妥当。

## Phase 6.7 - Object-gating enforcement / flag honesty

### 実装結果

- `ObjectGatingConfig(enforce=True)` を実挙動へ接続した。default `False` では従来どおり diagnostics-only。
- enforce on のときは、`rigid_hint` marker-indexed points から作った pose が valid で、`min_enforced_markers` と 2D matched marker views を満たす場合だけ committed pose として採択する。
- hint 品質が足りない場合は generic pose fallback を維持し、`selection_reason` に理由を残す。
- `tracking.<name>.rigid_hint_pose` と replay event に `enforced`, `diagnostics_only`, `selected_for_pose`, `selection_reason` を追加した。
- object gating event / summary に `enforced` と `diagnostics_only` を追加し、replay で shadow と enforcement の区別を追えるようにした。
- `replay_tracking_log(..., object_gating_enforced=True)` と CLI `--object-gating-enforced` を追加した。
- `TrackingPipeline`, `TrackingRuntime.start()`, GUI tracking start payload, `TrackingSession` に `rigid_stabilization` dict を通し、`reacquire_guard_*`, `object_conditioned_gating`, `object_gating_enforced`, `subset_ransac` を各 config へ写せるようにした。
- `SubsetSolveConfig(diagnostics_only=False)` はまだ採択経路がないため、明示的に `ValueError` にした。subset adoption は shadow summary が go 条件を満たすまで未実装扱いにする。
- `tests/test_rigid_reprojection_scoring.py`, `tests/test_tracking_pipeline_diagnostics.py`, `tests/test_tracking_replay_harness.py` に regression test を追加した。
- `changelog.md` に object-gating enforcement の user-visible outcome を記録した。

### 検証

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `46 passed`

GUI/runtime 周辺を含む追加検証:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `69 passed`

統合した rigid / runtime / GUI backend / clusterer 検証:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_geo_blob_assignment.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_rigid_body_tracker_stats.py \
  tests/test_pattern_evaluator.py \
  tests/test_tracking_replay_harness.py \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_clusterer.py -q
```

結果: `99 passed`

周辺 runtime / GUI backend / clusterer test:

```bash
/tmp/loutrack-py313-test/bin/python -m pytest \
  tests/test_tracking_runtime.py \
  tests/test_gui_backend_services.py \
  tests/test_rigid_clusterer.py -q
```

結果: `52 passed`

実ログ replay 比較:

```bash
/tmp/loutrack-py313-test/bin/python - <<'PY'
from src.host.tracking_replay_harness import replay_tracking_log
for enforced in (False, True):
    s = replay_tracking_log(
        log_path='logs/tracking_gui.jsonl',
        calibration_path='calibration',
        patterns=['waist'],
        rigids_path='calibration/tracking_rigids.json',
        object_gating_enforced=enforced,
    ).to_dict()
    waist = s['tracking']['waist']
    hint = s['rigid_hint_pose_summary']['totals']
    print(enforced, s['poses_estimated'], waist['max_pose_flip_deg'], waist['pose_jump_count'], hint['selected_for_pose_count'])
PY
```

結果:

| object gating enforced | valid poses | max pose flip deg | pose jumps | selected hint poses |
|---|---:|---:|---:|---:|
| `false` | `493` | `8.719878277400714` | `0` | `0` |
| `true` | `493` | `8.719878277400714` | `0` | `489` |

### 読み取り

`ObjectGatingConfig.enforce` は、これで「診断に表示されるだけ」ではなく、条件を満たす `rigid_hint` pose を実際の committed pose として使う意味になった。default は false のままなので、通常 runtime / replay の既存挙動は維持される。

今回の短い実ログでは、object-gating enforcement を有効にしても valid pose 数、pose flip、pose jump は悪化しなかった。一方で `hint_pose_adoption_ready` は false のままなので、まだ runtime default を on にする判断ではない。次は stress log / 静止 log でも `object_gating_enforced=True` replay を比較する。

Subset solve については、`diagnostics_only=False` が実採択を意味するように見える問題を、いったん明示的な unsupported error にした。subset adoption は `subset_adoption_shadow.ready_for_enforcement_replay` が true になるまで別フェーズで扱う。

## 2026-04-25 - Phase 6.8 Failure-log PDCA / object-gating enforcement replay

### 実装済みの範囲

- GUI start payload は live tracking 中の rigid stabilization enforcement を off にしている。
  - `object_conditioned_gating: false`
  - `object_gating_enforced: false`
  - `subset_ransac: false`
  - `reacquire_guard_enforced: false`
  - `reacquire_guard_event_logging: false`
- live tracking log には `tracking_diagnostics` event を残し、後から replay / failure segment 抽出に使える。
- `tracking_replay_harness` は `--diagnostics-summary` で live diagnostics の failure segment を抽出できる。
- replay は `calibration/extrinsics_pose_v2.json` を渡しても親の `calibration` directory に解決できる。
- replay は `--start-received-at` / `--end-received-at` / `--max-frames` で failure window だけを高速に切り出せる。
- `--compare-object-gating-enforcement` を追加し、diagnostics-only と `object_gating_enforced=True` のA/B比較、go/no-go判定を自動化した。
- object-gating go/no-go は valid frame drop、pose jump count、mode transition count、max flip tolerance、subset rotation mismatch を見る。
- max flip は完全非悪化ではなく、`max_flip_regression_deg = 1.0` 以内の微小差を許容する。旧ログ全体で出た `+0.008 deg` は数値ノイズ扱いで go にする。
- reacquire guard は全 `CONTINUE` frame へ広げるのではなく、必要な場合に `post_reacquire_continue_frames` で reacquire 直後だけ評価できる hook に留めた。
- generic pose がなく `rigid_hint` pose だけを採用する経路でも、valid pose なら reacquire guard を同じように評価する。

### 退避済みデータ

- 旧ログ / PDCA比較:
  - `logs/archive/rigid_stabilization_pdca_20260425_042805/tracking_gui_baseline.jsonl`
  - `logs/archive/rigid_stabilization_pdca_20260425_042805/full_compare_object_tolerant.json`
  - `logs/archive/rigid_stabilization_pdca_20260425_042805/pdca_segments/`
  - `logs/archive/rigid_stabilization_pdca_20260425_042805/pdca_post_guard/`
- 新ログ / 比較:
  - `logs/archive/rigid_stabilization_new_tracking_20260425_043548/tracking_gui_new.jsonl`
  - `logs/archive/rigid_stabilization_new_tracking_20260425_043548/diagnostics_summary.json`
  - `logs/archive/rigid_stabilization_new_tracking_20260425_043548/full_compare_object.json`
  - `logs/archive/rigid_stabilization_new_tracking_20260425_043548/failure_compare_object.json`

### 旧ログ failure window PDCA

旧 `logs/tracking_gui.jsonl` は `30452` frames、duration `129.76 s`。live diagnostics から `waist` の failure segment が4つ抽出された。

主要 failure window:

- `2026-04-25T02:46:15.064968` - `2026-04-25T02:46:32.478600`

baseline / object-gating enforced 比較:

| metric | diagnostics-only | object-gating enforced |
|---|---:|---:|
| poses | `1765` | `1768` |
| valid ratio | `0.874628` | `0.876115` |
| reacquire count | `68` | `66` |
| mode transitions | `145` | `141` |
| pose jumps | `12` | `12` |
| max pose jump m | `0.232616` | `0.232616` |
| max pose flip deg | `177.805626` | `177.805626` |
| max position innovation m | `0.556920` | `0.500697` |
| max rotation innovation deg | `179.984083` | `178.373793` |
| subset rotation max deg | `178.385586` | `2.295753` |

読み取り:

- object-gating enforcement は failure window で有効姿勢数、reacquire、mode transition、position innovation、subset rotation mismatch を改善した。
- max flip / pose jump は悪化しなかった。
- `subset rotation max` は subset RANSAC候補が通常経路 pose から最大何度ズレたかを見る内部診断で、最終採用 pose そのもののズレではない。ただし `178 deg -> 2.3 deg` は、marker対応や対称性由来の反転候補が大きく減った兆候として有用。

4つの failure segment 全体で見ると、object-gating enforcement は全区間で `subset rotation max` の180度級ズレを大幅に抑えた。seg3 では実際の `max_pose_flip_deg` も `146.058560 -> 48.306846` まで下がった。

一方、`object_gating_enforced + reacquire_guard_enforced` は seg1/seg2 で valid poses を減らした。

- seg1: `1391 -> 1386`
- seg2: `1768 -> 1766`

`post_reacquire_continue_frames = 1/2` も object-only より改善せず、seg1/seg2 では valid poses を削った。結論として、現ログでは **reacquire guard enforcement を足すより object-gating enforcement 単独が最良**。

### 旧ログ全体 replay

`--compare-object-gating-enforcement` を旧ログ全体へ適用した結果:

| metric | diagnostics-only | object-gating enforced |
|---|---:|---:|
| poses | `13644` | `13649` |
| valid ratio | `0.919593` | `0.919930` |
| reacquire count | `141` | `137` |
| mode transitions | `313` | `305` |
| pose jumps | `25` | `25` |
| max pose jump m | `0.437989` | `0.437989` |
| max pose flip deg | `177.805626` | `177.813651` |
| max pose flip delta deg | - | `+0.008025` |
| max position innovation m | `0.765045` | `0.500697` |
| max rotation innovation deg | `179.984083` | `178.894527` |
| subset rotation max deg | `167.793273` | `0.491559` |
| rigid p95 ms | `22.563500` | `29.926458` |
| pipeline pair p95 ms | `25.661958` | `33.142208` |

読み取り:

- 品質指標は概ね改善。valid ratio、reacquire、mode transition、position innovation、rotation innovation、subset rotation mismatch は良化した。
- `max_pose_flip_deg` は `+0.008 deg` 悪化したが、これは replay 数値ノイズ級のため `1.0 deg` tolerance 内として go。
- ただし performance cost は大きい。`pipeline_pair_ms.p95` が約 `+7.48 ms` 増えており、120fps 近辺の live tracking では無視しにくい。

### 新ログ replay

新 `logs/tracking_gui.jsonl` は `3272` frames、duration `14.03 s`。diagnostics event は `14`、failure segment は1つ。

failure window:

- `2026-04-25T04:32:57.196385` - `2026-04-25T04:33:08.365721`
- live diagnostics 上の最大 pose jump: `0.9483518087691913 m`
- live diagnostics 上の最大 pose flip: `177.85746626099103 deg`

新ログ全体の object-gating A/B:

| metric | diagnostics-only | object-gating enforced |
|---|---:|---:|
| poses | `1469` | `1469` |
| valid ratio | `0.915265` | `0.915265` |
| reacquire count | `44` | `44` |
| mode transitions | `89` | `89` |
| pose jumps | `6` | `6` |
| max pose jump m | `0.948352` | `0.952086` |
| max pose flip deg | `177.857466` | `176.670558` |
| max position innovation m | `1.232591` | `1.320411` |
| max rotation innovation deg | `179.936761` | `179.978517` |
| subset rotation max deg | `174.980230` | `63.969446` |
| rigid p95 ms | `23.251709` | `24.567416` |
| pipeline pair p95 ms | `26.558458` | `28.072958` |

新ログ failure window の object-gating A/B:

| metric | diagnostics-only | object-gating enforced |
|---|---:|---:|
| poses | `1132` | `1132` |
| valid ratio | `0.892744` | `0.892744` |
| reacquire count | `44` | `44` |
| mode transitions | `89` | `89` |
| pose jumps | `6` | `6` |
| max pose jump m | `0.948352` | `0.952086` |
| max pose flip deg | `177.857466` | `176.670558` |
| max position innovation m | `1.232591` | `1.320411` |
| max rotation innovation deg | `179.804892` | `179.978517` |
| subset rotation max deg | `174.980230` | `63.969446` |
| rigid p95 ms | `22.536750` | `22.425375` |
| pipeline pair p95 ms | `25.872833` | `25.758000` |

読み取り:

- 新ログでも go/no-go は `go_candidate`。
- max flip と subset rotation mismatch は改善。
- valid ratio、reacquire、mode transition、pose jump count は変化なし。
- ただし max pose jump は約 `+0.0037 m`、max position innovation は約 `+0.0878 m` 悪化した。
- 新ログの performance cost は旧ログより小さく、全体で `pipeline_pair_ms.p95 +1.51 ms` 程度、failure window ではむしろ微減した。

### 現時点の判断

object-gating enforcement は **offline replay / failure解析では採用候補**。

理由:

- 2本のログで go/no-go は `go_candidate`。
- 旧ログ failure segment では valid pose、reacquire、mode transition、position innovation が改善。
- 旧ログ全体でも valid ratio、reacquire、mode transition が改善。
- 新ログでは valid/reacquire/mode transition は維持しつつ max flip を改善。
- subset rotation max は両ログで大幅に減り、反転候補の探索空間がかなり締まった。

一方、**live GUI / real-time tracking の常時ONは保留**。

保留理由:

- 旧ログ全体で `pipeline_pair_ms.p95` が `25.66 ms -> 33.14 ms` へ増え、120fps 目標に対して重い。
- 新ログでは cost 増が小さいため、負荷はログ内容や failure 状態に依存して揺れる。
- 新ログでは max pose jump と max position innovation が少し悪化しており、現在の go/no-go は jump magnitude / position innovation の悪化を直接基準にしていない。
- live path では通常 `CONTINUE` 中の全フレームに enforcement をかけるより、`REACQUIRE` / `BOOT` / high-risk window に限定する方が性能面で現実的。

### 次の推奨作業

1. object-gating go/no-go に `max_pose_jump_m` と `max_position_innovation_m` の許容条件を追加する。
   - 今は `pose_jump_count` は見ているが、jump magnitude の微増は見ていない。
   - 新ログの `0.948352 -> 0.952086 m` は微小だが、`max_position_innovation_m 1.232591 -> 1.320411 m` は監視対象にしたい。
2. `object_gating_enforced` の限定発火モードを追加する。
   - 候補: `reacquire_only`, `boot_or_reacquire`, `risk_window`
   - 通常 `CONTINUE` では diagnostics-only を維持し、reacquire / lost recovery / flip-risk 近辺だけ enforcement する。
3. A/B/C比較を replay harness に追加する。
   - A: diagnostics-only
   - B: always object-gating enforced
   - C: limited object-gating enforced
4. 旧ログ全体、新ログ全体、旧 failure segments、新 failure window で同じ比較を回す。
5. live GUI でONにする判断は、limited enforcement が performance p95 と quality 指標の両方で always enforcement より良いことを確認してからにする。

### 実装ステータスまとめ

| 項目 | 状態 | 備考 |
|---|---|---|
| same-path replay | 実装済み | frame log を UDP後段と同じ経路へ注入 |
| diagnostics failure extraction | 実装済み | `--diagnostics-summary` |
| calibration file auto-resolve | 実装済み | `extrinsics_pose_v2.json` -> parent dir |
| replay window filter | 実装済み | received_at / timestamp / max_frames |
| object-gating diagnostics | 実装済み | live defaultはGUI payloadでoff |
| object-gating enforcement | 実装済み | replay / runtime flagあり、default off |
| object-gating A/B go/no-go | 実装済み | flip tolerance 1deg を含む |
| reacquire guard enforcement | 実装済みだが保留 | object-onlyよりvalidを削る傾向 |
| post-reacquire guard hook | 実装済みだが保留 | seg1/seg2で改善せず |
| subset RANSAC diagnostics | 実装済み | shadow summaryのみ |
| subset pose adoption | 未採用 | coverage / risk 条件未達 |
| live GUI enforcement ON | 保留 | performance p95 と innovation悪化を確認中 |
| limited object-gating enforcement | 未実装 | 次の有力候補 |

## 2026-04-25 - Phase 6.9 Limited object-gating enforcement exploration

### 実装結果

- `ObjectGatingConfig.activation_mode` を追加した。
- replay CLI に `--object-gating-activation-mode` を追加した。
  - `always`: 従来どおり全フレームで object gating を評価する。
  - `reacquire_only`: tracker mode が `REACQUIRE` の frame だけ object gating を評価する。
  - `boot_or_reacquire`: tracker mode が `BOOT` または `REACQUIRE` の frame だけ object gating を評価する。
- `TrackingPipeline` の `rigid_stabilization` payload から `object_gating_activation_mode` を `ObjectGatingConfig` へ渡せるようにした。
- `rigid_hint_pose` replay event に tracker `mode` を追加した。今後、always enforcement で採用された hint pose がどの mode で効いているかを追いやすくする。
- `tests/test_rigid_reprojection_scoring.py` に、`reacquire_only` が `CONTINUE` では inactive になり、`REACQUIRE` では active になる regression test を追加した。

### 退避済みデータ

- `logs/archive/rigid_stabilization_limited_object_gating_20260425_050441/new_full_reacquire_only.json`
- `logs/archive/rigid_stabilization_limited_object_gating_20260425_050441/new_full_boot_or_reacquire.json`
- `logs/archive/rigid_stabilization_limited_object_gating_20260425_050441/old_seg2_reacquire_only.json`
- `logs/archive/rigid_stabilization_limited_object_gating_20260425_050441/old_seg2_boot_or_reacquire.json`

### 比較結果

旧ログ seg2 failure window:

| mode | poses delta | reacquire delta | mode transition delta | selected hints | object events | subset rot max deg | rigid p95 ms | pair p95 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `always` | `+3` | `-2` | `-4` | `1680` | `2000` | `2.295753` | `22.433667` | `25.712750` |
| `reacquire_only` | `0` | `0` | `0` | `62` | `192` | `169.191443` | `21.871708` | `24.376459` |
| `boot_or_reacquire` | `0` | `0` | `0` | `91` | `242` | `169.191443` | `22.513333` | `25.394000` |

新ログ全体:

| mode | poses delta | reacquire delta | mode transition delta | selected hints | object events | max jump m | max flip deg | max pos innov m | subset rot max deg | rigid p95 ms | pair p95 ms |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `always` | `0` | `0` | `0` | `1374` | `1635` | `0.952086` | `176.670558` | `1.320411` | `63.969446` | `24.567416` | `28.072958` |
| `reacquire_only` | `0` | `0` | `0` | `37` | `122` | `0.948352` | `177.857466` | `1.232591` | `151.507740` | `28.196875` | `30.418792` |
| `boot_or_reacquire` | `0` | `0` | `0` | `47` | `150` | `0.948352` | `177.857466` | `1.232591` | `151.507740` | `28.094750` | `30.347625` |

### 読み取り

- 単純な mode 限定は object events / selected hints を大幅に減らす。
  - 旧 seg2: `2000 -> 192/242`
  - 新ログ: `1635 -> 122/150`
- ただし、品質改善もほぼ消える。
  - 旧 seg2 の `always` は poses `+3`, reacquire `-2`, mode transition `-4`, subset rot max `2.30 deg` まで改善。
  - `reacquire_only` / `boot_or_reacquire` は poses/reacquire/mode transition が baseline と同じで、subset rot max も `169 deg` 近辺に残った。
- 新ログでは限定モードが max jump / max position innovation を baseline に戻す一方、max flip 改善と subset rot max 改善は `always` より弱い。
- 新ログの p95 は限定モードの方が悪く見える。これは object gating を評価する frame が failure/reacquire 近辺に偏るため、単純な全体 p95 比較では「軽くなった」と出ない可能性がある。object events は減っているので、今後は object-gating stage の個別 timing も欲しい。

### 現時点の判断

`reacquire_only` / `boot_or_reacquire` は、**安全だが効きが弱い**。

live 常時ONの代替としてはまだ不十分。今回のログで object-gating enforcement が効いていた frame は、単純な `REACQUIRE` / `BOOT` だけでなく、`CONTINUE` 中の危険フレームにも多いと考えられる。

次の候補は **risk-window activation**。

条件案:

- tracker mode が `REACQUIRE` / `BOOT`
- または `CONTINUE` 中でも直近の innovation が大きい
  - `last_position_innovation_m`
  - `last_rotation_innovation_deg`
- または rolling confidence が低い
- または直近に invalid / reacquire transition があった post-reacquire window
- または subset diagnostics の `flip_risk_count` / large `best_rotation_delta_deg` が出た後の短い window

実装する場合は、`always` と単純 mode 限定の中間として `activation_mode = "risk_window"` を追加し、A/B/C/D比較を行う。

### 検証

```bash
PYTHONPATH=src .venv/bin/python -m pytest \
  tests/test_rigid_reprojection_scoring.py \
  tests/test_tracking_pipeline_diagnostics.py \
  tests/test_tracking_replay_harness.py -q
```

結果: `30 passed`
