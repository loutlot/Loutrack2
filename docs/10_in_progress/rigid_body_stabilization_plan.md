# Rigid Body Tracking 安定化 実装ロードマップ

**参考文献**: [`references/gpt_research/rigid_body_stabilization.md`](../../references/gpt_research/rigid_body_stabilization.md)
**対象コード**: `src/host/rigid.py`, `src/host/pipeline.py`, `src/host/geo.py`, `src/host/tracking_runtime.py`, `src/host/replay.py`
**実ログ**: `logs/tracking_gui.jsonl`

---

## 0. レビュー結論

商用システム調査から見た最重要ポイントは、rigid body を「triangulation 後に 3D 点群へ当てる読取器」ではなく、**2D blob 対応付け、triangulation、3D pose 継続判定の全段を制約する主体**にすること。

現計画の方向性は正しいが、実装順と境界を修正する。特に Phase 3/4 で必要になる 2D 観測情報を後から足すと迷子になりやすいので、Phase 0 の直後に **Phase 0.5: Observation Contract / Provenance** を追加する。

### 0.1 実ログから見える現状

`logs/tracking_gui.jsonl` は約 5 秒、2 カメラ、diagnostics 5 件の短いログだった。各 diagnostics で `accepted_points = 4` が維持され、reprojection mean はおよそ `0.25-0.53 px`、epipolar mean はおよそ `0.62-1.29 px`、`rigid_ms` p95 はおよそ `0.56-1.65 ms` だった。

このログ単体では triangulation 品質は悪く見えない。むしろ、rigid body が valid / lost を繰り返す主因は、3D 点が作れないことより、**既知 rigid pattern が 2D 対応付けと継続判定を十分に制約していないこと**だと見る。

### 0.2 既存計画からの主な修正

1. `Phase 0` に same-path replay harness を追加し、実ログを `FrameReplay -> FrameProcessor._on_frame_received -> FramePairer -> TrackingPipeline._on_paired_frames -> GeometryPipeline -> RigidBodyEstimator` の経路で再生して rigid body 認識挙動を比較できるようにする。
2. `Phase 0.5` を新設し、2D blobs、triangulated points、rigid hints の provenance を先に固定する。
3. Pose filter は 13-state quaternion EKF の直接更新ではなく、nominal pose と small-angle covariance を分ける error-state filter を基本にする。
4. `predict(timestamp)` が state を進める API は避ける。hint 作成や candidate 採点で複数回呼ばれるため、`peek_prediction(timestamp)` と `advance_to(timestamp)` を分ける。
5. 2D scoring は raw pixel と undistorted pixel を混ぜない。raw pixel で scoring するか、undistorted pixel で scoring するかを context に明示する。
6. Single-ray continuation は実 3D 点として Kabsch に混ぜず、`virtual` または `weak` observation として別扱いにする。
7. Pattern evaluator を早める。手元の簡易計算では既定 pattern の最小自己対称 RMS は `waist 4.65mm`, `head 0.29mm`, `chest 5.73mm`, `left_foot 9.06mm`, `right_foot 4.06mm` 程度で、特に head は非常に曖昧。既存の「waist は unique のはず」という期待は置かない。

---

## 1. 目標アーキテクチャ

### 1.1 現状

```text
UDPRx -> FramePairer -> PairedFrames
                        |
                        v
                  GeometryPipeline
                  2D対応付け + DLT
                        |
                        v
                  points_3d: List[np.ndarray]
                  ここで 2D 情報をほぼ捨てる
                        |
                        v
                  RigidBodyEstimator.process_points()
                  DBSCAN cluster -> Kabsch -> valid/lost
                        |
                        v
                  RigidBodyTracker
                  位置線形外挿、姿勢は実質凍結
```

### 1.2 改善後

```text
FrameReplay/UDPRx -> FrameProcessor -> FramePairer -> PairedFrames
                                                 |
                                                 v
                                      ObservationContext
                                      per-camera 2D blobs
                                      raw + undistorted uv
                                      blob ids / camera ids
                                                 |
                +--------------------------------+-------------------------------+
                |                                                                |
                v                                                                v
        RigidBodyPredictor                                               Generic Triangulation
        pose + covariance + confidence                                   epipolar assignment
                |                                                                |
                v                                                                v
        RigidBodyHint                                             TriangulatedPoint provenance
        predicted marker windows                              point, rays, blob ids, quality
                |                                                                |
                +------------------------+---------------------------------------+
                                         v
                               RigidBodyEstimator
                               boot / continue / reacquire
                               3D fit + 2D score + temporal score
                                         |
                                         v
                               RigidBodyPose + TrackDiagnostics
```

この構成で、商用系に近い `shape first, temporal second` を再現する。boot は形状で厳しく、continue は履歴で助け、reacquire は短い multi-hypothesis と 2D score で誤復帰を抑える。

---

## 2. 共通設計原則

1. **Feature flag で段階導入する**: 各 phase は `rigid_stabilization.<feature>` で on/off 可能にする。off 時は既存挙動を保つ。
2. **同じ実ログを同じ経路で replay する**: 机上 KPI ではなく、`logs/tracking_gui.jsonl` と実機 stress log を pipeline 経路へ再投入して比較する。
3. **2D 観測を捨てない**: 3D 点だけでなく、どの camera/blob/ray から来た点かを rigid 側へ渡す。
4. **raw/undistorted 座標系を混ぜない**: scoring context に `coordinate_space = "raw_pixel"` または `"undistorted_pixel"` を持たせる。
5. **pose prediction は side effect free を基本にする**: candidate 生成中に filter state が進まないように、peek と update を分離する。
6. **single-ray は weak evidence**: 1 ray の marker は pose 継続の補助には使うが、実 triangulated point と同じ重みで Kabsch に入れない。
7. **lock-in を常に疑う**: confidence が高いときだけ narrow gate と single-ray を許可し、innovation / 2D residual / hypothesis margin が悪化したら即 reacquire へ落とす。

---

## 3. データ契約

Phase 0.5 で先にこの形を固める。以後の phase はこの context を参照するだけにする。

```python
@dataclass(frozen=True)
class BlobObservation2D:
    camera_id: str
    blob_index: int
    raw_uv: tuple[float, float]
    undistorted_uv: tuple[float, float]
    area: float = 0.0


@dataclass(frozen=True)
class TriangulatedPoint:
    point: np.ndarray
    observations: tuple[BlobObservation2D, ...]
    reprojection_errors_px: tuple[float, ...]
    epipolar_errors_px: tuple[float, ...]
    triangulation_angle_deg: float
    source: str = "generic"  # generic, rigid_hint, single_ray_virtual
    rigid_name: str | None = None
    marker_idx: int | None = None
    is_virtual: bool = False


@dataclass(frozen=True)
class RigidBodyHint:
    name: str
    marker_positions_world: np.ndarray
    predicted_raw_uv_by_camera: dict[str, np.ndarray]
    predicted_undistorted_uv_by_camera: dict[str, np.ndarray]
    pixel_gate_px: float
    confidence: float
    allow_single_ray: bool


@dataclass(frozen=True)
class RigidTrackingContext:
    timestamp_us: int
    camera_params: dict[str, CameraParams]
    observations_by_camera: dict[str, list[BlobObservation2D]]
    triangulated_points: list[TriangulatedPoint]
    coordinate_space: str  # raw_pixel or undistorted_pixel
```

`GeometryPipeline.process_paired_frames()` は既存 return shape を壊さず、追加 key として以下を返す。

```python
{
    "points_3d": list[np.ndarray],                 # 既存互換
    "triangulated_points": list[TriangulatedPoint], # 新規
    "observations_by_camera": dict[str, list[BlobObservation2D]],
    "triangulation_quality": dict,
    "assignment_diagnostics": dict,
}
```

---

## 4. 実装ロードマップ

### Phase 0 - Diagnostics と same-path replay harness

#### 目的

実機ログで rigid body 認識の valid/lost/swap を再現し、各 phase の効果を比較できるようにする。挙動は変えない。

#### 実装内容

1. `RigidBodyTracker` に rolling stats を追加する。
2. `TrackingPipeline._diagnostics_snapshot()` に rigid tracking status を含める。
3. `src/host/tracking_replay_harness.py` を追加し、JSONL の frame entries を `FrameProcessor._on_frame_received()` へ同期注入する。
4. replay harness は `TrackingPipeline.start()` を呼ばず、UDP receiver を開かない。代わりに `_running = True`、`_calibration_loaded = True`、`frame_processor.set_paired_callback(pipeline._on_paired_frames)` を設定し、ログ frame を `Frame` に戻して同じ pairer / pipeline 経路へ流す。
5. replay 結果として `TrackingReplaySummary` を出力する。

#### 追加 diagnostics

- `tracking.{name}.valid`
- `tracking.{name}.mode`
- `tracking.{name}.confidence`
- `tracking.{name}.rms_error_m`
- `tracking.{name}.observed_markers`
- `tracking.{name}.real_ray_count`
- `tracking.{name}.virtual_marker_count`
- `tracking.{name}.mean_valid_run_frames`
- `tracking.{name}.short_valid_ratio`
- `tracking.{name}.reacquire_count`
- `tracking.{name}.pose_jump_count`
- `tracking.{name}.pose_flip_deg`
- `tracking.{name}.invalid_reason`
- `tracking.{name}.hypothesis_margin`

#### Replay command

```bash
python -m src.host.tracking_replay_harness \
  --log logs/tracking_gui.jsonl \
  --calibration calibration \
  --rigids calibration/tracking_rigids.json \
  --out logs/stabilization_replay_summary.json
```

#### 受入基準

- `logs/tracking_gui.jsonl` を same-path replay できる。
- replay summary に frame count、pair count、valid run、lost run、pose jump、flip candidate が出る。
- `tests/test_tracking_replay_harness.py` で短い synthetic JSONL を replay し、FramePairer と TrackingPipeline が通る。

### Phase 0.5 - Observation Contract / Provenance

#### 目的

Phase 3/4 の前提として、2D blobs と 3D points の由来を保持する。ここで挙動は変えない。

#### 実装内容

1. `geo.py` の `_BlobObservation` を public に近い `BlobObservation2D` へ整理する。
2. `Triangulator.triangulate_paired_frames()` が `TriangulatedPoint` を内部生成し、既存 `points_3d` はそこから互換出力する。
3. `GeometryPipeline.process_paired_frames()` が `observations_by_camera` と `triangulated_points` を返す。
4. GUI scene は従来どおり `points_3d` を読む。新情報は diagnostics / rigid estimator context 用に使う。

#### 注意点

- raw pixel scoring では `cv.projectPoints(..., distortion_coeffs)` の出力と `raw_uv` を比較する。
- undistorted pixel scoring では distortion なしの projection と `undistorted_uv` を比較する。
- どちらを採用するかを `RigidTrackingContext.coordinate_space` に明記する。初期実装は raw pixel scoring を推奨する。

#### 受入基準

- 既存 `tests/test_tracking_pipeline_diagnostics.py` が通る。
- `triangulated_points[i].point` と `points_3d[i]` が一致する。
- diagnostics に per-point contributing cameras/blob ids を出せる。

### Phase 1 - Pattern Ambiguity Evaluator

#### 目的

アルゴリズムを複雑にする前に、rigid body pattern 自体が commercial 的に混同しやすい形状かを数値化する。

#### 実装内容

1. `src/host/pattern_evaluator.py` を追加する。
2. self-symmetry、cross-pattern ambiguity、3-marker subset ambiguity を計算する。
3. GUI で custom rigid body 保存時に warning を出す。
4. CLI で ambiguity table を出す。

#### 指標

- `min_self_symmetry_mm`: identity 以外の marker permutation で Kabsch した最小 RMS。
- `min_cross_pattern_mm`: 他 pattern と最も似る fit RMS。
- `ambiguous_triplet_count`: 3 marker subset が他の subset と距離的に近い数。
- `front_back_flip_risk`: 表裏反転候補の pose と RMS。

#### 初期仮説

既定 pattern は 4 点なので permutation が少なく、見た目より曖昧になりやすい。簡易 Kabsch では head が `0.29mm` 程度の自己対称 RMS になったため、head は高リスクとして扱う。

#### 受入基準

- 意図的に対称な test pattern が ambiguous 判定になる。
- 既定 5 pattern の ambiguity table が出る。
- `calibration/tracking_rigids.json` の custom rigids も評価できる。

### Phase 2 - Pose Predictor と rolling confidence

#### 目的

線形位置予測と姿勢凍結を、rigid body pose レベルの predictor に置き換える。後段 gating の信頼度を作る。

#### 方針

直接 quaternion を 13-state EKF として雑に更新しない。実装は以下のどちらかにする。

1. **初期版**: constant velocity + quaternion slerp/angular velocity estimate + covariance-like scalar confidence。
2. **安定版**: nominal state `[p, v, q, omega]` と error covariance `[dp, dv, dtheta, domega]` を持つ error-state filter。

初期版でも、API は安定版へ移行できる形にする。

#### 必須 API

```python
class RigidBodyTracker:
    def peek_prediction(self, timestamp_us: int) -> PredictedPose:
        """State を進めずに予測 pose を返す。"""

    def advance_to(self, timestamp_us: int) -> None:
        """フレーム処理ごとに一度だけ state を進める。"""

    def update_with_measurement(self, pose: RigidBodyPose, diagnostics: dict) -> None:
        """accepted measurement で更新する。"""
```

`peek_prediction()` は絶対に state を変更しない。Phase 3/4 で candidate 採点や hint 作成に何度も使うため。

#### Confidence

`track_count / total_frames` は lifetime 平均なので使わない。以下から rolling confidence を作る。

- 直近 `N=30` frames の valid ratio。
- 直近 innovation の median。
- 直近 2D reprojection score。
- `real_ray_count / expected_ray_count`。
- `virtual_marker_count` の比率。
- hypothesis margin。

#### 受入基準

- 合成 constant-velocity / constant-turn 軌道で position error と rotation error が bounded。
- `peek_prediction()` を複数回呼んでも state が変わらない。
- replay で Phase 0 baseline より `pose_jump_count` が増えない。

### Phase 3 - Boot / Continue / Reacquire mode 分離

#### 目的

取得時と継続時を同じ閾値で扱わない。boot は shape-first で厳しく、continue は予測近傍を優先し、reacquire は short-horizon hypotheses で慎重に commit する。

#### Mode

```python
class TrackMode(Enum):
    BOOT = "boot"
    CONTINUE = "continue"
    REACQUIRE = "reacquire"
    LOST = "lost"
```

#### 遷移

```text
BOOT -> N consecutive accepted -> CONTINUE
CONTINUE -> one rejected frame -> REACQUIRE
REACQUIRE -> M accepted frames with good innovation and margin -> CONTINUE
REACQUIRE -> K rejected frames -> LOST
LOST -> strict shape-first accepted -> BOOT
```

#### 候補評価

`RigidBodyEstimator.process_points(points_3d, timestamp)` だけでは不足するため、Phase 0.5 後は以下を主 API にする。

```python
def process_context(self, context: RigidTrackingContext) -> dict[str, RigidBodyPose]:
    ...
```

既存 `process_points()` は互換 wrapper として残す。

#### 受入基準

- mode transition の unit test を追加する。
- replay summary で valid/lost run が比較できる。
- `short_valid_ratio` が baseline より悪化しない。

### Phase 4 - 2D Reprojection Scoring

#### 目的

Kabsch の 3D RMS だけで pose を commit しない。全カメラの 2D blob と照合し、pose hypothesis の plausibility を採点する。

#### Score components

- `mean_reprojection_error_px`
- `matched_marker_view_count`
- `missing_marker_view_count`
- `duplicate_assignment_count`
- `unexpected_blob_count`
- `visibility_ratio`
- `hypothesis_margin`
- `temporal_innovation`
- `ambiguity_penalty`

#### 実装方針

1. 初期実装は raw pixel scoring に統一する。
2. 各 camera で projected marker と blobs の cost matrix を作り、Hungarian で one-to-one assignment する。
3. Gate 外の blob は invalid cost にする。
4. Score は mode ごとに重みを変える。

```python
score = (
    w_reproj * mean_reprojection_error_px
    + w_missing * missing_marker_view_count
    + w_duplicate * duplicate_assignment_count
    + w_unexpected * unexpected_blob_count
    + w_temporal * innovation_norm
    + w_ambiguity * pattern_ambiguity_penalty
)
```

#### 受入基準

- 合成シーンで正解 pose が mirror / swapped pose より低 score になる。
- replay で `pose_jump_count` と `pose_flip_deg` が増えない。
- `rigid_ms` p95 が 8ms 以下に収まる。現ログでは rigid p95 が 2ms 未満なので余裕はある。

### Phase 4.5 - Reacquire Candidate Guard

#### 目的

Phase 4 の 2D reprojection score を、いきなり全 tracking mode の採択へ使わない。まず `REACQUIRE` 中の accepted candidate だけを対象に、**shadow evaluation -> guarded enforcement** の順で進める。

狙いは、valid/lost の揺れから復帰する瞬間に、3D RMS は良いが 2D residual や blob assignment が怪しい hypothesis を commit しないこと。特に Phase 1 で見えた pattern ambiguity と、Phase 0/3 replay で見えた large pose flip candidate をここで抑える。

Phase 4.5 は Phase 5 前の安全ゲートとして扱う。`Phase 4.6` のような独立 phase は作らず、以下の sub-step をすべて Phase 4.5 の完了条件に含める。これにより進捗が細切れになりすぎることを避け、Phase 5 の object-conditioned gating へ入る前に「2D score を採択判断へ使ってよいか」を判断する。

#### なぜ Phase 5 の前に挟むか

Object-conditioned 2D gating は triangulation 前の blob 対応付けを変えるため効果が大きいが、lock-in risk も大きい。Phase 4.5 は triangulation 後の pose candidate に対する guard なので、失敗時の rollback が容易で、Phase 5 の前に「2D score を採択判断に使ってよいか」を検証できる。

#### 実装ステップ

Phase 4.5 は以下の 4 つの sub-step で完了扱いにする。

**4.5A - Shadow guard diagnostics**

1. `RigidBodyEstimator` に `reacquire_guard_shadow_enabled` と `reacquire_guard_enforced` 相当の flag / config を追加する。
2. 初期値は shadow only にする。pose acceptance は変えず、guard 判定だけ diagnostics に出す。
3. `RigidBodyTracker` の current mode が `reacquire` のときだけ guard を評価する。
4. Guard input は Phase 4 の `reprojection_score` と Phase 3 の innovation diagnostics を使う。
5. Shadow diagnostics として `reacquire_guard.would_reject`, `reason`, `thresholds`, `score`, `evaluated_count`, `would_reject_count` を出す。

**4.5B - Per-frame guard event logging**

1. Replay / live diagnostics event に per-frame `reacquire_guard` event を出す。
2. Event には `timestamp`, `rigid_name`, `mode`, `would_reject`, `reason`, `reprojection_score`, `position_innovation_m`, `rotation_innovation_deg` を含める。
3. `would_reject_count` だけでなく、どの frame がどの reason で落ちたかを追えるようにする。
4. Event logging は bounded にし、通常 frame では noisy にならないよう `evaluated == true` または `would_reject == true` の frame を優先する。

**4.5C - Enforcement replay**

1. Replay harness から `reacquire_guard_enforced` を切り替えられるようにする。
2. Enforcement は `REACQUIRE` mode の candidate のみに限定する。
3. Enforcement で reject した場合は accepted pose を commit せず、invalid pose として tracker に流す。
4. `invalid_reason = "reprojection_guard_rejected"` と `reacquire_guard.rejected_count` を残す。
5. Shadow replay と enforcement replay の差分を summary で比較する。

**4.5D - Go / no-go decision for Phase 5**

1. 実ログ replay で `pose_flip_deg` または `pose_jump_count` が改善するか確認する。
2. `valid` frames が大きく落ちないことを確認する。目安は baseline から `2%` 以内の低下、または lost run が長くならないこと。
3. Stress log でも false lost が増えないことを確認する。
4. 条件を満たす場合のみ runtime default の候補にする。満たさない場合は shadow-only のまま Phase 5 へ進む。

#### 初期 guard 条件

初回は緩めにする。single metric だけで落とさず、複数の悪化条件を組み合わせる。

```python
guard_pass = (
    reprojection_score.scored
    and reprojection_score.matched_marker_views >= 6
    and reprojection_score.mean_error_px <= 4.0
    and reprojection_score.p95_error_px <= 8.0
    and reprojection_score.duplicate_assignment_count == 0
    and reprojection_score.missing_marker_views <= 2
    and position_innovation_m <= 0.25
    and rotation_innovation_deg <= 120.0
)
```

ただし、Phase 4.5 の最初の実装ではこれを **shadow 判定のみ** にする。`guard_pass == False` でも pose は従来どおり commit し、replay summary で `would_reject` を見る。

#### 採択ロジックを変えるタイミング

今すぐ本番採択を変える必要はない。Phase 4 の実ログ replay では最終 frame の 2D score は高く、`493 / 497` valid、`reacquire_count = 3`、`max_pose_flip_deg = 138.43 deg` が Phase 3 と同じだった。つまり、まず必要なのは「どの frame の reacquire candidate が guard に引っかかるか」を時系列で見ること。

採択ロジックを変えてテストすべきタイミングは以下。

1. Shadow replay で、large pose flip 付近の accepted candidate が `would_reject = true` になる。
2. 通常の stable continue frame では `would_reject = false` が維持される。
3. Synthetic test で mirrored / swapped pose を reject し、correct pose を pass できる。
4. Enforcement on replay で `pose_flip_deg` または `pose_jump_count` が改善し、`valid` frames が大きく落ちない。

この 4 条件を満たしてから、`reacquire_guard_enforced = true` を試す。最初の enforcement 対象は `REACQUIRE` mode のみで、`BOOT` / `CONTINUE` には適用しない。

Phase 5 に進むために、enforcement を本番 default にする必要はない。Phase 4.5 の目的は、object-conditioned gating へ進む前に 2D score の採択利用リスクを把握すること。enforcement replay の結果が微妙な場合は、guard を shadow-only のまま残し、Phase 5 では preassignment の diagnostics と fallback 設計を優先する。

#### diagnostics

- `tracking.{name}.reacquire_guard.enabled`
- `tracking.{name}.reacquire_guard.enforced`
- `tracking.{name}.reacquire_guard.evaluated`
- `tracking.{name}.reacquire_guard.passed`
- `tracking.{name}.reacquire_guard.would_reject`
- `tracking.{name}.reacquire_guard.reason`
- `tracking.{name}.reacquire_guard.thresholds`
- `tracking.{name}.reacquire_guard.score`
- `tracking.{name}.reacquire_guard.rejected_count`
- `tracking.{name}.invalid_reason = "reprojection_guard_rejected"`

#### tests

- Unit: correct pose は guard pass、mirrored / swapped pose は shadow reject。
- Unit: `CONTINUE` mode では guard を評価しても enforcement しない。
- Unit: `REACQUIRE` mode で enforcement on のとき rejected candidate が tracker に invalid として流れる。
- Integration: per-frame guard event が replay / diagnostics に記録される。
- Replay smoke: shadow on で `valid/reacquire/pose_flip` が Phase 4 と同じ。
- Replay enforcement: enforcement on で `pose_flip_deg` / `pose_jump_count` が悪化せず、`valid` 減少が許容範囲内。

#### 受入基準

- Shadow mode で diagnostics が出る。
- Shadow mode では Phase 4 replay と `valid/reacquire/pose_flip` が一致する。
- Synthetic mirror/swap case で `would_reject = true` になる。
- Enforcement mode は feature flag で明示的に有効化でき、off で完全に Phase 4 挙動へ戻せる。
- Enforcement mode の採用判断は、実ログ replay と stress log replay の両方を見てから行う。
- Phase 5 に進む前に、Phase 4.5D の go / no-go を記録する。

### Phase 5 - Object-conditioned 2D Gating

#### 目的

最大効果の phase。generic triangulation の前に、tracked rigid body の predicted pose から marker ごとの 2D search window を作り、候補 blob を絞る。

#### 重要な設計修正

現行 triangulation は blob 数最大 camera を ref camera にする。object-conditioned gating は ref-camera 依存にしない。marker ごとに全 camera の candidate rays を集める `ray pool` として扱う。

#### 手順

1. `RigidBodyTracker.peek_prediction(timestamp)` から predicted world markers を作る。
2. 各 camera へ project し、marker ごとの 2D gate を作る。
3. camera ごとに marker x blob の Hungarian assignment を行う。
4. marker ごとに 2 ray 以上あれば constrained triangulation する。
5. 1 ray のみの場合は `single_ray_virtual` として weak observation を作る。
6. 残り blobs は従来 generic epipolar triangulation へ流す。

#### Single-ray rules

- `confidence > 0.75` のときだけ許可。
- 連続 single-ray-only frames は最大 `10` などで制限する。
- single-ray marker は `is_virtual=True` とし、Kabsch に実点と同じ重みでは入れない。
- real rays が戻った frame で必ず 2D score と innovation を sanity check する。

#### 受入基準

- 1 camera 遮蔽を含む synthetic replay で valid run が伸びる。
- 予測 pose が 10cm ずれた場合、preassignment が lock-in せず generic/reacquire へ fallback する。
- 実機 stress log で `reacquire_count` が Phase 4 より減る。

### Phase 6 - Subset RANSAC と weighted rigid solve

#### 目的

複数 rigid body や noise blob が混ざったとき、全 permutation に頼らず、局所 subset hypothesis と 2D score で正しい pose を選ぶ。

#### 実装内容

1. 3-marker / 4-marker subset から pose hypothesis を作る。
2. pattern evaluator の ambiguity score を使い、曖昧な subset は低優先にする。
3. 全 hypothesis を Phase 4 の 2D score で比較する。
4. final solve は weighted Kabsch にする。real triangulated points は重く、single-ray virtual は軽く、古い virtual はさらに軽くする。

#### 受入基準

- 正解 4 点 + noise 6 点の synthetic case で高確率に正解 pose を選ぶ。
- 複数 rigid body 同時 replay で `rigid_ms` が permutation 爆発しない。
- `virtual_marker_count > pattern.num_markers / 2` の pose は valid commit しない。

### Phase 7 - Pi-side 2D centroid tracking

#### 目的

Vicon 方式に近づける将来 phase。host 側の object-conditioned gating で効果を取ったあと、Pi 側で 2D centroid track id を持たせる。

#### 方針

- `src/pi/service/blob_detection.py` または `capture_runtime.py` に lightweight 2D tracker を追加する。
- payload に `blob_track_id` と short-lived confidence を載せる。
- host 側では triangulation の hard identity には使わず、2D assignment cost の補助にする。

これは今回の主ロードマップ外。Phase 0-6 の効果を見てから判断する。

---

## 5. Feature Flags

`logs/loutrack_gui_settings.json` に以下を追加する。

```jsonc
{
  "rigid_stabilization": {
    "diagnostics_enabled": true,
    "same_path_replay_enabled": true,
    "observation_provenance_enabled": false,
    "pattern_evaluator_warnings": true,
    "use_pose_predictor": false,
    "mode_transitions_enabled": false,
    "use_2d_reprojection_scoring": false,
    "reacquire_guard_shadow_enabled": false,
    "reacquire_guard_event_logging": false,
    "reacquire_guard_enforced": false,
    "object_conditioned_gating": false,
    "subset_ransac": false,
    "single_ray_virtual_markers": false,

    "mode": {
      "boot_min_markers": 4,
      "boot_max_rms_m": 0.010,
      "continue_max_rms_m": 0.025,
      "reacquire_max_rms_m": 0.020,
      "reacquire_max_frames": 30,
      "promote_continue_after": 3
    },
    "gate": {
      "pixel_min": 4.0,
      "pixel_max": 16.0,
      "single_ray_confidence_min": 0.75
    },
    "reacquire_guard": {
      "min_matched_marker_views": 6,
      "max_missing_marker_views": 2,
      "max_mean_reprojection_error_px": 4.0,
      "max_p95_reprojection_error_px": 8.0,
      "allow_duplicate_assignment": false,
      "max_position_innovation_m": 0.25,
      "max_rotation_innovation_deg": 120.0
    }
  }
}
```

本番 session 開始時に有効 flag を `stabilization_config` event として `logs/tracking_gui.jsonl` に出す。replay summary にも同じ config を保存する。

---

## 6. テスト戦略

| レベル | 目的 | 代表テスト |
|---|---|---|
| Unit | pattern ambiguity, predictor, scoring | `tests/test_pattern_evaluator.py`, `tests/test_pose_predictor.py`, `tests/test_rigid_scoring.py` |
| Geometry contract | provenance 互換性 | `tests/test_geometry_observation_context.py` |
| Integration | mode transition, reacquire guard, object gating, single-ray virtual | `tests/test_rigid_mode_transitions.py`, `tests/test_rigid_reacquire_guard.py`, `tests/test_object_conditioned_gating.py` |
| Replay | 実ログを同じ経路へ再投入 | `tests/test_tracking_replay_harness.py`, `tests/test_stabilization_replay_smoke.py` |
| Performance | p95 budget | existing diagnostics tests + replay summary assertions |

テスト追加時は、既存の類似 coverage と重複しないように `tests/test_rigid_clusterer.py`, `tests/test_tracking_pipeline_diagnostics.py`, `tests/test_geo_blob_assignment.py` を見直す。

---

## 7. 実機検証手順

Pi / GUI startup と shutdown は `docs/30_procedure/pi_gui_start_stop.md` に従う。

各 phase で以下の 3 種類のログを取る。

1. 静止ログ: rigid body を静止させ、jitter と false lost を見る。
2. 通常動作ログ: 腰や頭を通常速度で動かし、valid run と pose jump を見る。
3. Stress log: 片 camera 遮蔽、表裏が近い姿勢、複数 rigid body 同時、部分 occlusion を含める。

各ログを same-path replay し、phase 間で以下を比較する。

- `mean_valid_run_frames`
- `short_valid_ratio`
- `reacquire_count`
- `pose_jump_count`
- `pose_flip_deg`
- `real_ray_count`
- `virtual_marker_count`
- `rigid_ms.p95`
- `triangulation_ms.p95`

---

## 8. 終了ゲート

各 phase は次を満たすまで完了扱いにしない。

1. Feature flag off で既存テストが通る。
2. Feature flag on で該当 unit/integration tests が通る。
3. `logs/tracking_gui.jsonl` の same-path replay が完了する。
4. `pose_jump_count` と `pose_flip_deg` が前 phase より悪化しない。
5. `rigid_ms.p95` が 12ms 以下に収まる。
6. 新しい diagnostics が GUI/API/status/log のどこかで確認できる。

---

## 9. 今回は採用しないもの

- **JPDA / MHT**: 現段階では重すぎる。Phase 6 後も密集 multi-body で破綻する場合だけ検討する。
- **runtime AC-RANSAC**: online 常用はしない。offline threshold tuning には検討余地あり。
- **continuous auto-calibration**: 長時間運用で extrinsics drift が主要因だと確認できてから検討する。
- **marker 個別 Kalman を主軸にする設計**: passive marker identity が曖昧なため、pose-level predictor を主軸にする。

---

## 10. 次の実装順

1. Phase 0: `tracking_replay_harness` と rigid tracking diagnostics。
2. Phase 0.5: `BlobObservation2D` / `TriangulatedPoint` の provenance contract。
3. Phase 1: `pattern_evaluator`。既定 pattern と custom rigids の ambiguity table を出す。
4. Phase 2: side-effect-free pose predictor と rolling confidence。
5. Phase 3: mode transition。
6. Phase 4: 2D reprojection scoring。
7. Phase 4.5: reacquire candidate guard。4.5A shadow diagnostics、4.5B per-frame event logging、4.5C enforcement replay、4.5D go/no-go をまとめて完了させる。
8. Phase 5: object-conditioned 2D gating。
9. Phase 6: subset RANSAC と weighted solve。
