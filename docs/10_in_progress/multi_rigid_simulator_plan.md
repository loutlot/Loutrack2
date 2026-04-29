# Multi-Rigid Occlusion Simulator 実装計画

## 0. 結論

5 rigid body、各 5 blobs、4 camera、遮蔽、missing marker、false blob を扱うためのシミュレーターは過剰ではない。
むしろ、実ログだけでは再現しにくい「どの遮蔽条件で wrong ownership / pose flip / reacquire jump が起きるか」を固定条件で比較できる。
そのため、5 rigid 化の前に作る価値が高い。

ただし、最初から巨大な物理シミュレーターにはしない。
既存の `tests/perf/synthetic_32_blob_benchmark.py`、`tests/test_tracking_replay_harness.py`、
`src/host/receiver.py` の契約を土台にして、段階的に「合成 2D blob 生成器 + replay 可能な評価器」へ広げる。
`src/host/sim.py` は現行運用で使っていないため、互換維持よりも multi-rigid 前提での作り直しを優先してよい。

位置づけは **multi-rigid tracking の風洞**。
実機の代替ではなく、人間がやりえる高速動作、足組み、左右入れ替え、接近、遮蔽を再現して、アルゴリズム変更の精度と性能を高速に PDCA するための装置にする。

---

## 1. ゴール

### 1.0 目的

目的は、`docs/10_in_progress/rigid_body_design.md` の 5-marker / subset whitelist 方針を前提に、5 rigid bodies を multi camera で安定して追うアルゴリズム最適化の PDCA を高速化すること。

対象 rigid は以下。

- `head`
- `waist`
- `chest`
- `left_foot`
- `right_foot`

それぞれ 5 blobs を持つ前提で、実機で何度も再現するのが難しい動作を synthetic scenario として固定する。
特に重要なのは、性能だけを速くするのではなく、精度、所有権、reacquire、flip 耐性を同時に見ながら改善すること。

### 1.1 主要ゴール

1. 3D 上に 5 rigid bodies x 5 blobs の pattern と human-like trajectory を置く。
2. `calibration/extrinsics_pose_v2.json` と各 camera intrinsics を使い、4 camera の 2D blob 観測を生成する。
3. 高速移動、足組み、左右足の接近/入れ替え、身体部位同士の接近、occlusion、missing marker、pixel noise、false blob、camera dropout、timestamp jitter を注入する。
4. 既存の UDP payload 形式、または replay harness が受け取れる `Frame` / `PairedFrames` 形式へ流す。
5. ground truth pose と estimator 出力を比較し、pose error / flip / jump / wrong ownership / reacquire / fps を自動集計する。
6. 同じ seed と scenario で、tracking algorithm の差分を AB 比較できるようにする。
7. threshold と scenario pack を使い、精度改善と性能改善の PDCA を短い cycle で回せるようにする。

### 1.2 最初に欲しい成果物

- `python -m host.sim` で動く multi-rigid scenario runner。
- `tests/sim/` に置く予定の deterministic scenario fixtures。
- 1 command で `logs/sim/.../summary.json` のような比較結果を出す CLI。
- GUI や Pi 実機を起動せず、`TrackingPipeline` / `RigidBodyEstimator` の回帰を検出できるテスト。

`tests/sim/` は現時点では未作成の新規ディレクトリ候補。初回実装時に作成する。

---

## 2. 非ゴール

最初の版では以下をやらない。

- 反射材の画像生成、露光、ぼけ、レンズフレアなどの photorealistic rendering。
- 剛体同士の衝突、人体物理、布や装着揺れの物理シミュレーション。
- 自動最適化で tracking algorithm を直接書き換える仕組み。
- GUI 操作用の完全な可視化ツール。
- 実機ログを置き換える go/no-go 判定。

最初は「既知の 3D 点を camera に投影し、2D blob と欠落を作り、既存 pipeline に流し、GT と比較する」ことに集中する。

---

## 3. 既存資産との接続

### 3.1 使うもの

- `src/host/sim.py`
  - 既存実装は single rigid / 2 camera 中心のレガシー扱い。
  - 使える projection / UDP loopback の考え方は参考にするが、API 互換は必須にしない。
  - multi-rigid runner を同ファイルへ置き換える。
- `tests/test_sim_closed_loop.py`
  - `src/host/sim.py` を作り直すので、現行テストを新しい simulator 契約に合わせて置き換える。
  - stale な single-rigid 互換テストは残さない。
- `tests/perf/synthetic_32_blob_benchmark.py`
  - 4 camera / 32 blobs per camera の synthetic benchmark。
  - pipeline variant AB の JSON 集計がある。
- `tests/test_tracking_replay_harness.py`
  - replay harness の same-path 注入テスト。
- `src/host/receiver.py`
  - `Frame`
  - `PairedFrames`
- `src/pi/service/capture_runtime.py`
  - `pose_capture` UDP payload の現行 key。

### 3.2 役割分担

`src/host/sim.py` は使われていないため、互換維持用の薄い wrapper は必須ではない。
今後の simulation entrypoint は `src/host/sim.py` に固定する。

```text
src/host/sim.py
  multi-rigid scenario runner
  projection / occlusion / UDP loopback
  GT evaluation

tests/sim/
  deterministic scenario tests
  regression fixtures
```

古い single-rigid API を守るための余分な分岐は置かず、`src/host/sim.py` を multi-rigid 前提で作り直す。
ただし、`Frame` / `PairedFrames` と `pose_capture` UDP payload の外部契約は守る。

---

## 4. シミュレーター構成

### 4.1 データフロー

```text
ScenarioConfig
  |
  v
WorldModel
  5 rigid x 5 blob patterns
  trajectories
  marker GT poses
  |
  v
VirtualCameraRig
  intrinsics
  extrinsics
  projection
  |
  v
ObservationSynthesizer
  occlusion
  missing marker
  pixel noise
  false blob
  timestamp jitter
  |
  +--> Frame / PairedFrames in-process
  |
  +--> UDP pose_capture payload loopback
  |
  v
TrackingPipeline / RigidBodyEstimator
  |
  v
GroundTruthEvaluator
  pose error
  identity error
  jump / flip
  reacquire behavior
  timing
```

### 4.2 ScenarioConfig 案

```python
@dataclass(frozen=True)
class MultiRigidScenarioConfig:
    seed: int
    camera_ids: tuple[str, ...]
    frames: int
    fps: float
    rigid_names: tuple[str, ...]
    trajectory_name: str
    noise_px: float
    false_blobs_per_camera: int
    marker_dropout_prob: float
    camera_dropout_prob: float
    timestamp_jitter_us: int
    occlusion_profile: str
    motion_profile: str
    body_interaction_profile: str
    max_velocity_mps: float
    max_angular_velocity_deg_s: float
```

初期値は以下を想定する。

- `camera_ids`: `pi-cam-01` から `pi-cam-04`
- `rigid_names`: `head`, `waist`, `chest`, `left_foot`, `right_foot`
- `frames`: `600`
- `fps`: `118`
- `false_blobs_per_camera`: `0`, `16`, `32` のプリセット
- `motion_profile`: `walk`, `fast_step`, `leg_cross`, `feet_swap`, `close_approach`

### 4.3 Rigid pattern

最初は `calibration/tracking_rigids.json` を読む。
設計方針は `docs/10_in_progress/rigid_body_design.md` の **5 markers + mode policy + subset whitelist** に合わせる。
未定義の body がある場合だけ、built-in pattern または deterministic placeholder を使う。

重要なのは「5 rigid を同時に置く」こと。単体 tracking が正しくても、混在 blob cloud では以下が起きる。

- 近い rigid の marker を借りた boot。
- 遮蔽中の reacquire が別 body に吸われる。
- 3-of-5 continuation が weak subset で別 body を説明してしまう。
- false blob が低 RMS candidate を作る。
- `left_foot` と `right_foot` が接近または交差したときに identity が入れ替わる。
- 高速 movement の後に prediction gate が広がり、別 body へ吸い込まれる。

### 4.4 Camera model

projection は実 calibration を優先する。

- intrinsics: `calibration/calibration_intrinsics_v1_pi-cam-*.json` に一致する既存 camera、または dummy/generated camera
- extrinsics: `calibration/extrinsics_pose_v2.json`

calibration がない CI / unit test では、既存 `create_dummy_calibration()` 互換の deterministic camera rig を使う。

### 4.5 UDP payload

UDP loopback mode は `pose_capture` と同じ shape を出す。

```json
{
  "camera_id": "pi-cam-01",
  "timestamp": 1000000,
  "timestamp_source": "synthetic",
  "blobs": [{"x": 123.4, "y": 567.8, "area": 50.0}],
  "blob_count": 12,
  "quality": 0.08,
  "capture_mode": "pose_capture",
  "capture_to_process_ms": 0.0,
  "capture_to_send_ms": 0.0
}
```

in-process mode は `Frame` / `PairedFrames` を直接生成し、UDP receive や JSON parse の揺らぎを避ける。
アルゴリズム評価は in-process、receiver / payload 互換性評価は UDP loopback で分ける。

---

## 5. 評価指標

### 5.1 PDCA metrics

最重要の出力は、アルゴリズム変更ごとの「精度と性能の差分」を即座に見られる summary。
1 回の実行で以下を比較する。

- current branch / previous baseline
- scenario 別の failure mode
- pose 精度
- ownership 精度
- reacquire 安定性
- pipeline performance
- false positive / false recovery の増減

PDCA の cycle は以下にする。

```text
Plan:
  改善したい failure mode を scenario と threshold に落とす

Do:
  algorithm / parameter / gating policy を変更する

Check:
  scenario pack を実行し、accuracy と performance の差分を見る

Act:
  良い変更だけ残し、悪化した scenario を次の Plan に戻す
```

### 5.2 Pose metrics

- `position_error_m.mean`
- `position_error_m.p95`
- `rotation_error_deg.mean`
- `rotation_error_deg.p95`
- `valid_frame_ratio`
- `drop_rate`
- `reacquire_latency_frames`

### 5.3 Stability metrics

- `pose_jump_count`
- `pose_jump_max_m`
- `pose_flip_count`
- `pose_flip_max_deg`
- `short_valid_run_count`
- `lost_run_count`
- `reacquire_count`

### 5.4 Ownership metrics

- `wrong_ownership_count`
- `wrong_ownership_frames`
- `marker_source_confusion_count`
- `body_swap_count`
- `left_right_swap_count`
- `swap_recovery_latency_frames`

ownership 判定には、合成時に各 blob へ hidden metadata を持たせる。
pipeline へ流す payload には metadata を入れず、evaluator 側だけが `blob_id -> gt_rigid_name / marker_index` を保持する。

### 5.5 Performance metrics

- `pipeline_pair_ms.p95`
- `rigid_ms.p95`
- `geometry_ms.p95`
- `fps_effective`
- `frames_processed`
- `pairs_emitted`

既存の `TrackingPipeline.get_status()` と replay summary に寄せる。
`tests/perf/synthetic_32_blob_benchmark.py` の比較と見比べられる形にする。

---

## 6. シナリオセット

### 6.1 MVP scenarios

1. `single_static_clean`
   - 1 rigid、2 camera、noise なし。
   - simulator 基本動作の sanity check。
2. `five_static_clean`
   - 5 rigid x 5 blobs、4 camera、false blob なし。
   - multi-rigid の基本 ownership を確認。
3. `five_linear_32_false`
   - 5 rigid x 5 blobs、4 camera、32 blobs per camera。
   - `synthetic_32_blob_benchmark.py` に近い負荷条件。
4. `five_crossing_occlusion`
   - 2 body が近距離ですれ違い、一部 marker が遮蔽。
   - wrong ownership と reacquire を狙う。
5. `five_partial_visibility`
   - 3-of-5 / 4-of-5 が混在。
   - subset whitelist / mode policy の評価用。
6. `five_fast_human_motion`
   - 人間がやりえる高速な上半身/足の movement。
   - prediction gate、velocity limit、pose jump guard の評価用。
7. `feet_cross_left_right_swap`
   - `left_foot` と `right_foot` が足組みや交差で近接し、見た目の左右が入れ替わる。
   - left/right ownership と recovery latency を評価する。
8. `body_close_approach`
   - `waist`、`chest`、`head`、feet が短時間近づく。
   - cross-rigid attraction と wrong boot/reacquire を評価する。

### 6.2 拡張 scenarios

- camera 1 台 dropout。
- timestamp jitter と pair window 境界。
- rigid が camera 視野端を通る。
- body ごとに marker area を変える。
- false blob が rigid pattern に近い位置へ出る adversarial case。
- 足組み中に片足だけ 3-of-5 visibility へ落ちる case。
- 高速 movement 直後に self-occlusion が入る case。
- `left_foot` / `right_foot` の片方が完全 lost した直後、もう片方が近傍に残る case。

---

## 7. 実装フェーズ

### Phase 0 - 設計固定

目的:

- この文書を実装前の合意点にする。
- `src/host/sim.py` を作り直してよい方針を固定する。

受入基準:

- ゴール、非ゴール、MVP scenario、評価指標が文書化されている。

### Phase 1 - Multi-rigid in-process generator

目的:

- 5 rigid x 5 blobs / 4 camera の `PairedFrames` を deterministic に生成する。

実装:

- `src/host/sim.py` を multi-rigid runner として作り直す。
- `ScenarioConfig`、`WorldModel`、`VirtualCameraRig`、`ObservationSynthesizer` を追加。
- `calibration/tracking_rigids.json` と `calibration/extrinsics_pose_v2.json` を読む。
- calibration がない場合は dummy rig に fallback。

受入基準:

- 同じ seed の出力が完全に再現する。
- 1 frame に 5 rigid 分の projected blob が 4 camera へ出る。
- `tests/sim/` に clean projection の unit test がある。

### Phase 2 - GT evaluator

目的:

- estimator 出力と ground truth を比較し、summary を JSON で出す。

実装:

- pose error と ownership metrics を集計。
- hidden metadata を evaluator 側に保持。
- `TrackingPipeline` へは通常 payload だけを流す。

受入基準:

- clean scenario で position / rotation error が低い。
- intentional rigid swap fixture で `wrong_ownership_count` が増える。
- summary が `logs/sim/.../summary.json` に出る。

### Phase 3 - Human motion / occlusion / noise / false blob

目的:

- 人間がやりえる高速動作、足組み、左右入れ替え、遮蔽、吸い込みを合成条件で再現する。

実装:

- per-marker dropout。
- camera-specific occlusion window。
- gaussian pixel noise。
- uniform false blobs。
- adversarial false blobs。
- timestamp jitter。
- high-speed human motion。
- leg crossing / feet swap。
- close body-part approach。

受入基準:

- `five_crossing_occlusion` で wrong ownership / reacquire の差分が観測できる。
- seed を変えた regression pack を複数回しても summary schema が安定する。

### Phase 4 - UDP loopback compatibility

目的:

- Pi なしで host receiver 以降の経路も検証する。

実装:

- `pose_capture` payload を UDP で送る mode を追加。
- in-process mode と同じ scenario を UDP mode でも実行できるようにする。
- receiver timing metrics を summary に含める。

受入基準:

- UDP mode で `TrackingPipeline` が frame を受け、pairing される。
- payload key が `src/pi/service/capture_runtime.py` の現行 `pose_capture` と揃っている。

### Phase 5 - AB regression runner

目的:

- tracking algorithm の変更を synthetic scenario pack で比較し、精度と性能の PDCA を高速に回す。

実装:

- variant / flag matrix を受け取る CLI。
- scenario ごとの before/after summary。
- go/no-go threshold。

受入基準:

- `fast_ABCDHRF` など既存 variant を指定して比較できる。
- pose stability が改善しても fps が悪化しすぎた場合に failure にできる。
- performance が改善しても ownership / left-right swap が悪化した場合に failure にできる。

---

## 8. CLI 案

```bash
python -m host.sim \
  --scenario five_crossing_occlusion \
  --frames 600 \
  --fps 118 \
  --calibration calibration \
  --rigids calibration/tracking_rigids.json \
  --mode inprocess \
  --out logs/sim/five_crossing_occlusion
```

AB 比較:

```bash
python -m host.sim \
  --scenario-pack multi_rigid_occlusion_v1 \
  --variant baseline \
  --variant fast_ABCDHRF \
  --out logs/sim/ab_multi_rigid_occlusion_v1
```

UDP loopback:

```bash
python -m host.sim \
  --scenario five_linear_32_false \
  --mode udp \
  --udp-port 5000 \
  --out logs/sim/five_linear_32_false_udp
```

---

## 9. 回帰判定の初期 threshold

最初の threshold は厳しすぎない値から始める。

```text
clean scenarios:
  valid_frame_ratio >= 0.98
  position_error_m.p95 <= 0.02
  rotation_error_deg.p95 <= 10.0
  wrong_ownership_count == 0

occlusion scenarios:
  no body_swap_count increase from baseline
  pose_jump_count <= baseline
  wrong_ownership_frames <= baseline
  pipeline_pair_ms.p95 <= baseline * 1.25

human-motion scenarios:
  left_right_swap_count <= baseline
  swap_recovery_latency_frames <= baseline
  reacquire_latency_frames <= baseline
  pipeline_pair_ms.p95 <= baseline * 1.25
```

threshold は固定値だけでなく baseline comparison を使う。遮蔽 scenario は絶対値より「前より悪化していないか」を重視する。

---

## 10. 実装上の注意

1. simulation metadata を UDP payload や `Frame.blobs` に混ぜない。
   - 実 payload と違う shape になると、receiver / pipeline の検証価値が落ちる。
2. random は scenario seed から派生させる。
   - camera、rigid、false blob で seed stream を分ける。
3. stale な simulator API を温存しない。
   - `tests/test_sim_closed_loop.py` は新しい multi-rigid 契約に合わせて更新する。
   - 似た目的の旧テストと新テストを重複させず、必要なら統合する。
4. 実 calibration を使う mode と dummy calibration mode を分ける。
   - CI 的な軽いテストは dummy、実運用に近い探索は `calibration/` を使う。
5. 重い scenario pack は unit test に入れない。
   - unit は短く deterministic。
   - perf / regression は opt-in CLI にする。
6. optimizer は最後に置く。
   - まずは scenario、metrics、AB runner が安定してから parameter sweep を追加する。

---

## 11. 最小実装のタスク分解

1. `src/host/sim.py` に `ScenarioConfig` と 5 rigid projection を追加。
2. `tests/sim/` を作り、clean 1-frame projection test を追加。
3. in-process runner で `PairedFrames` を `TrackingPipeline._on_paired_frames()` に流す。
4. GT evaluator で pose error と valid/drop を集計。
5. hidden blob ownership metadata から wrong ownership を集計。
6. high-speed human motion、足組み、left/right swap、close approach preset を追加。
7. occlusion / false blob preset を追加。
8. UDP loopback mode を追加。
9. scenario pack AB runner を追加。

---

## 12. 判断

このシミュレーターは「過剰な研究設備」ではなく、5 rigid 化で必要になる安全装置に近い。

ただし、価値が出る順番は明確にする。

1. deterministic multi-rigid projection。
2. GT evaluator。
3. human-like motion scenario。
4. occlusion / false blob。
5. left/right swap and close-approach scenario。
6. UDP loopback。
7. AB regression。
8. parameter sweep / optimization。

この順で進めれば、初期実装は小さく、かつ現在の rigid stabilization 作業へすぐ効く。
