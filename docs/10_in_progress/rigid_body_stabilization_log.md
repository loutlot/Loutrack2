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
