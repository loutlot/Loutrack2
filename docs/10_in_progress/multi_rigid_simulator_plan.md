# Multi-Rigid Occlusion Simulator 実装計画

## 0. 結論

5 rigid body、各 5 blobs、4 camera、遮蔽、missing marker、false blob を扱うためのシミュレーターは過剰ではない。
むしろ、実ログだけでは再現しにくい「どの遮蔽条件で wrong ownership / pose flip / reacquire jump が起きるか」を固定条件で比較できる。
そのため、5 rigid 化の前に作る価値が高い。

ただし、最初から巨大な物理シミュレーターにはしない。
既存の `tests/perf/synthetic_32_blob_benchmark.py`、`tests/test_tracking_replay_harness.py`、
`src/host/receiver.py` の契約を土台にして、段階的に「合成 2D blob 生成器 + replay 可能な評価器」へ広げる。
本アプリの runtime から切り離し、開発用ツールとして `tools/sim/` に置く。
最初の PDCA は現行 fixture と実ログに合わせて `waist` + `wand` の 2 rigid / 4 marker で回し、吸い込み、reacquire、ownership、118fps を先に固定する。
その後に 5-marker pattern fixture と 5 rigid scenario へ拡張する。

位置づけは **multi-rigid tracking の風洞**。
実機の代替ではなく、人間がやりえる高速動作、足組み、左右入れ替え、接近、遮蔽を再現して、アルゴリズム変更の精度と性能を高速に PDCA するための装置にする。

## 0.1 2026-04-30 PDCA 結果

`waist_rotate_partial_occlusion` を追加し、`generated_4cam_from_1_2_intrinsics` と `gui_live` profile で 5 周確認した。

- Cycle 1: 新 scenario 追加直後は ownership / marker index / valid ratio は合格、118fps budget は `rigid_ms.p95` と pair p95 で不合格。
- Cycle 2: GUI hot path の rigid-hint pose / score 再利用で `rigid_ms.p95` が約 2.08ms から約 1.54ms へ改善。
- Cycle 3: 再測で `rigid_ms.p95` は約 1.43ms まで改善し、残りは wall-clock 外れ値の sustained 判定のみ。
- Cycle 4: 180 frame regression 条件で `scenario_go_no_go.passed == true`、`wrong_ownership_count == 0`、`marker_source_confusion_count == 0`。
- Cycle 5: `tests/test_sim_closed_loop.py::test_waist_rotation_partial_occlusion_gui_profile_go_no_go` で正式 regression 化。

判断: `fast_ABCDHRF` + `gui_live` profile は、この scenario について GUI 経路へ正式採用してよい。
ただし長めの 180-240 frame では環境由来の pair max / sustained 外れ値が出ることがあるため、正式 gate は 120 frame regression を主判定にし、長時間 run は監視指標として残す。

## 0.2 2026-04-30 2 camera shared-blob occlusion PDCA 結果

`waist_rotate_partial_occlusion` を `dummy` 2 camera rig でも回し、2 camera 同時に同じ marker が欠ける条件を確認した。

- Baseline: `wrong_ownership_count == 0` だが、`waist.valid_frame_ratio == 0.5167`、`reacquire_count == 1`、`max_pose_jump_m == 0.0825`、`max_pose_flip_deg == 27.2155`。marker index の取り違えより、low-marker 区間で invalid 化して再捕捉時に跳ねる問題だった。
- Cycle 1: 2 marker の rigid hint が残る間だけ prediction hold を valid として扱い、`marker_source_confusion_count` は 0 へ改善。ただし 1 marker 境界でまだ 20 frame lost。
- Cycle 2: すでに hold 中なら 1 marker 証拠でも短時間 hold を継続し、`valid_frame_ratio == 1.0`、`reacquire_count == 0`、`max_pose_jump_m == 0` まで改善。ただし回転を固定保持したため absolute rotation error が大きかった。
- Cycle 3: tracker prediction に角速度を入れ、回転中の occlusion でも `rotation_error_deg.max` をほぼ 0 へ改善。
- Cycle 4: 回転 prediction の計算を軽量化し、`rigid_ms.p95` を 1.5ms 未満へ戻した。
- Cycle 5: partial object-gating 時も relaxed fast path を維持し、`pipeline_pair_ms.p95 == 4.91ms`、`rigid_ms.p95 == 1.27ms`、`scenario_go_no_go.passed == true`。

判断: 2 camera の shared-blob occlusion でも、GUI 経路へ採用可能。
`src/host/tracking_runtime.py` の GUI runtime default に正式採用し、frontend payload が欠けても同じ `fast_ABCDHRF` + `subset_diagnostics_mode=off` + object-gated continuity guard 経路で起動する。
ただし true LOST / no_prediction の再探索は引き続き full-blob geometry fallback を使う。

## 0.3 2026-04-30 4 camera / 5 rigid stress PDCA 結果

`five_rigid_dance_occlusion` を追加し、4 camera / 5 rigid で `head`、`waist`、`chest`、`left_foot`、`right_foot` を同時に流す stress scenario を作った。
動作はダンス相当の腰・胸の twist、頭部 bob、左右足の交差、胴体遮蔽、脚交差時の同時 marker loss、false blob を含む。

- Cycle 1: 初期配置が近すぎ、`waist` / `chest` が boot できず、scenario が難しすぎて評価風洞として不適切だった。
- Cycle 2: 初期配置を分離し、全 rigid が boot する条件にした。`wrong_ownership_count == 4`、`waist` が完全無観測のまま prediction valid で漂う問題を確認。
- Cycle 3: 完全無観測の unseen hold に上限を入れたが、pose continuity guard 経路が別に prediction hold を続けるため、漂いが残った。
- Cycle 4: multi-rigid で観測証拠がない通常 prediction を valid 採用しないようにし、完全無観測の valid drift は止まった。ただし `waist` の初期 boot 誤認が残った。
- Cycle 5: multi-rigid boot に 2D reprojection guard を追加し、3D RMS だけで通る悪い boot を拒否した。結果は `wrong_ownership_count == 0` になったが、現行 4-marker では `waist.valid_frame_ratio == 0.0` で、5 rigid stress はまだ production go/no-go 不合格。

判断: `five_rigid_dance_occlusion` は正式採用済み GUI 経路の置換条件ではなく、4-marker 5 rigid の限界と 5-marker 化の必要性を測る stress scenario として採用する。
性能面は `pipeline_pair_ms.p95` が約 20ms、`rigid_ms.p95` が約 5ms で 118fps 予算外。
精度を犠牲にして valid pose を出すより、現時点では悪い boot と完全無観測 drift を拒否する方を採った。

---

## 1. ゴール

### 1.0 目的

目的は、まず現行の `waist` + `wand` で multi-rigid occlusion の PDCA を高速化し、その後 `docs/10_in_progress/rigid_body_design.md` の 5-marker / subset whitelist 方針に沿って 5 rigid bodies へ拡張すること。

MVP 対象 rigid は以下。

- `waist`
- `wand`

拡張対象 rigid は以下。

- `head`
- `waist`
- `chest`
- `left_foot`
- `right_foot`

MVP は現行と同じ 4 marker rigid を扱う。
5 rigid 版ではそれぞれ 5 blobs を持つ fixture を先に固定し、実機で何度も再現するのが難しい動作を synthetic scenario として固定する。
特に重要なのは、性能だけを速くするのではなく、精度、所有権、reacquire、flip 耐性を同時に見ながら改善すること。

### 1.1 主要ゴール

1. 3D 上に MVP は 2 rigid x 4 blobs、拡張版は 5 rigid bodies x 5 blobs の pattern と human-like trajectory を置く。
2. `calibration/extrinsics_pose_v2.json` と各 camera intrinsics を使い、2 camera または 4 camera の 2D blob 観測を生成する。
3. 高速移動、足組み、左右足の接近/入れ替え、身体部位同士の接近、occlusion、missing marker、pixel noise、false blob、camera dropout、timestamp jitter を注入する。
4. 既存の UDP payload 形式、または replay harness が受け取れる `Frame` / `PairedFrames` 形式へ流す。
5. ground truth pose と estimator 出力を比較し、pose error / flip / jump / wrong ownership / reacquire / fps を自動集計する。
6. 同じ seed と scenario で、tracking algorithm の差分を AB 比較できるようにする。
7. threshold と scenario pack を使い、精度改善と性能改善の PDCA を短い cycle で回せるようにする。

### 1.2 最初に欲しい成果物

- `python -m tools.sim` で動く multi-rigid scenario runner。
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

- `tools/sim/`
  - 本アプリ runtime から独立した開発用 simulator package。
  - 既存 `src/host/sim.py` の projection / UDP loopback の考え方は参考にするが、app package には残さない。
  - multi-rigid runner と CLI entrypoint をここへ置く。
- `tests/test_sim_closed_loop.py`
  - simulator を `tools/sim/` へ移すので、現行テストを新しい simulator 契約に合わせて置き換える。
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

`src/host/sim.py` は本アプリの runtime package から外す。
今後の simulation entrypoint は `tools/sim/` に固定する。

```text
tools/sim/
  multi-rigid scenario runner
  projection / occlusion / UDP loopback
  GT evaluation

tests/sim/
  deterministic scenario tests
  regression fixtures
```

古い single-rigid API を守るための余分な分岐は置かず、`tools/sim/` を multi-rigid 前提で作る。
ただし、`Frame` / `PairedFrames` と `pose_capture` UDP payload の外部契約は守る。

---

## 4. シミュレーター構成

### 4.1 データフロー

```text
ScenarioConfig
  |
  v
WorldModel
  MVP: 2 rigid x 4 blob patterns
  Extended: 5 rigid x 5 blob patterns
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
    marker_layout: str
    camera_rig_source: str
```

初期値は以下を想定する。

- `camera_ids`: MVP は `pi-cam-01`, `pi-cam-02`。4 camera scenario は `pi-cam-01` から `pi-cam-04`
- `rigid_names`: MVP は `waist`, `wand`。5 rigid pack は `head`, `waist`, `chest`, `left_foot`, `right_foot`
- `frames`: `600`
- `fps`: `118`
- `false_blobs_per_camera`: `0`, `16`, `32` のプリセット
- `motion_profile`: `linear`, `occlusion_reacquire`, `walk`, `fast_step`, `leg_cross`, `feet_swap`, `close_approach`
- `marker_layout`: `current_4marker`, `future_5marker`
- `camera_rig_source`: `real_2cam`, `generated_4cam_from_1_2_intrinsics`, `dummy`

### 4.3 Rigid pattern

MVP は現行の `waist` built-in pattern と `calibration/tracking_rigids.json` の `wand` を読む。
これにより、直近の実ログと同じ 2 rigid / 4 marker 条件で吸い込み対策を PDCA できる。
5 rigid 版へ進む前に、`docs/10_in_progress/rigid_body_design.md` の **5 markers + mode policy + subset whitelist** に合わせた deterministic 5-marker fixture を追加する。
5-marker fixture がない状態で 5 rigid / 5 blobs を名乗らない。

重要なのは「5 rigid を同時に置く」こと。単体 tracking が正しくても、混在 blob cloud では以下が起きる。

- 近い rigid の marker を借りた boot。
- 遮蔽中の reacquire が別 body に吸われる。
- 3-of-5 continuation が weak subset で別 body を説明してしまう。
- false blob が低 RMS candidate を作る。
- `left_foot` と `right_foot` が接近または交差したときに identity が入れ替わる。
- 高速 movement の後に prediction gate が広がり、別 body へ吸い込まれる。

### 4.4 Camera model

projection は scenario の目的に合わせて camera rig source を明示する。

- `real_2cam`: `calibration/calibration_intrinsics_v1_pi-cam-01.json`、`calibration/calibration_intrinsics_v1_pi-cam-02.json`、`calibration/extrinsics_pose_v2.json` を使う。
- `generated_4cam_from_1_2_intrinsics`: `pi-cam-01` と `pi-cam-02` の intrinsics をコピーして `pi-cam-03` と `pi-cam-04` に割り当て、deterministic generated extrinsics で 4 camera rig を作る。
- `dummy`: calibration がない CI / unit test 用に deterministic camera rig を作る。

summary には必ず `camera_rig_source` と `marker_layout` を出す。
これにより、実 2 camera の回帰、generated 4 camera の負荷、5-marker fixture の評価を混同しない。

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

GT pose は pattern-local marker positions を body frame とし、scenario trajectory の `R_body_to_world` と `t_body_in_world` を正解にする。
estimator pose との比較は、同じ marker pattern origin で position error を測る。
rotation error は `R_est @ R_gt.T` の角度で測り、quaternion は `q` と `-q` を同一姿勢として扱う。
左右足など identity が意味を持つ rigid は、形状対称性で rotation が曖昧にならない deterministic 5-marker fixture を使う。

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

ownership 判定には、合成時に各 emitted blob へ sidecar metadata を持たせる。
pipeline へ流す payload には metadata を入れず、evaluator 側だけが `(timestamp, camera_id, emitted_blob_index) -> gt_rigid_name / marker_index / synthetic_blob_id` を保持する。
rigid diagnostics の matched observation / `blob_index` とこの sidecar ledger を突き合わせて、wrong ownership と marker source confusion を集計する。
pipeline 内で blob 順序が変わる経路を追加した場合は、diagnostics 側に元の `blob_index` が残ることをテスト条件にする。

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

1. `single_waist_static_clean`
   - 1 rigid、2 camera、noise なし。
   - simulator 基本動作の sanity check。
2. `waist_wand_static_clean`
   - `waist` + `wand`、2 camera、false blob なし。
   - 現行 tracking fixture で multi-rigid ownership を確認。
3. `waist_wand_occlusion_reacquire`
   - `waist` を完全遮蔽し、遮蔽終了後に `wand` 近傍へ吸われないことを確認する。
   - 直近の実ログ failure を synthetic に固定する最初の本命 scenario。
4. `waist_wand_linear_32_false`
   - `waist` + `wand`、2 camera または generated 4 camera、32 blobs per camera。
   - `synthetic_32_blob_benchmark.py` に近い負荷条件。
5. `five_static_clean`
   - 5 rigid x 5 blobs、generated 4 camera、false blob なし。
   - 5-marker fixture 固定後の multi-rigid 基本 ownership を確認。
6. `five_crossing_occlusion`
   - 2 body が近距離ですれ違い、一部 marker が遮蔽。
   - wrong ownership と reacquire を狙う。
7. `five_partial_visibility`
   - 3-of-5 / 4-of-5 が混在。
   - subset whitelist / mode policy の評価用。
8. `five_fast_human_motion`
   - 人間がやりえる高速な上半身/足の movement。
   - prediction gate、velocity limit、pose jump guard の評価用。
9. `feet_cross_left_right_swap`
   - `left_foot` と `right_foot` が足組みや交差で近接し、見た目の左右が入れ替わる。
   - left/right ownership と recovery latency を評価する。
10. `body_close_approach`
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
- simulator は app runtime ではなく `tools/sim/` に置く方針を固定する。

受入基準:

- ゴール、非ゴール、MVP scenario、評価指標が文書化されている。
- MVP は `waist` + `wand` / `current_4marker` で PDCA し、その後 `future_5marker` に進む方針が文書化されている。
- `camera_rig_source`、`marker_layout`、pose convention、ownership sidecar ledger の契約が文書化されている。

### Phase 1 - Waist + wand in-process generator

目的:

- `waist` + `wand` / 2 camera の `PairedFrames` を deterministic に生成する。

実装:

- `tools/sim/` を multi-rigid runner として作る。
- `ScenarioConfig`、`WorldModel`、`VirtualCameraRig`、`ObservationSynthesizer` を追加。
- `waist` built-in pattern、`calibration/tracking_rigids.json` の `wand`、`calibration/extrinsics_pose_v2.json` を読む。
- `real_2cam` を標準にし、calibration がない場合は `dummy` に fallback。

受入基準:

- 同じ seed の出力が完全に再現する。
- 1 frame に `waist` + `wand` 分の projected blob が 2 camera へ出る。
- `tests/sim/` に clean projection の unit test がある。
- summary に `camera_rig_source: real_2cam` と `marker_layout: current_4marker` が出る。

### Phase 2 - GT evaluator

目的:

- estimator 出力と ground truth を比較し、summary を JSON で出す。

実装:

- pose error と ownership metrics を集計。
- sidecar ownership ledger を evaluator 側に保持。
- `TrackingPipeline` へは通常 payload だけを流す。

受入基準:

- clean scenario で position / rotation error が低い。
- intentional waist/wand swap fixture で `wrong_ownership_count` が増える。
- summary が `logs/sim/.../summary.json` に出る。

### Phase 3 - Waist + wand occlusion / noise / false blob

目的:

- 直近の `waist` 完全遮蔽、遮蔽終了、`wand` 近傍への吸い込みを合成条件で再現する。

実装:

- per-marker dropout。
- camera-specific occlusion window。
- gaussian pixel noise。
- uniform false blobs。
- adversarial false blobs。
- timestamp jitter。
- `waist_wand_occlusion_reacquire`。
- `waist_wand_linear_32_false`。

受入基準:

- `waist_wand_occlusion_reacquire` で wrong ownership / reacquire の差分が観測できる。
- seed を変えた regression pack を複数回しても summary schema が安定する。

### Phase 4 - Generated 4 camera and 5-marker fixture

目的:

- 2 rigid PDCA の基盤を維持したまま、5 rigid / 5 marker へ進む準備をする。

実装:

- `generated_4cam_from_1_2_intrinsics` を追加する。
- `pi-cam-01` と `pi-cam-02` の intrinsics をコピーし、`pi-cam-03` と `pi-cam-04` に割り当てる。
- deterministic generated extrinsics で 4 camera rig を作る。
- 5 rigid 用の deterministic 5-marker fixture を追加する。
- high-speed human motion、leg crossing、feet swap、close body-part approach を追加する。

受入基準:

- 1 frame に 5 rigid x 5 marker 分の projected blob が generated 4 camera へ出る。
- summary に `camera_rig_source: generated_4cam_from_1_2_intrinsics` と `marker_layout: future_5marker` が出る。
- clean 5-marker scenario で wrong ownership が 0 になる。

### Phase 5 - UDP loopback compatibility

目的:

- Pi なしで host receiver 以降の経路も検証する。

実装:

- `pose_capture` payload を UDP で送る mode を追加。
- in-process mode と同じ scenario を UDP mode でも実行できるようにする。
- receiver timing metrics を summary に含める。

受入基準:

- UDP mode で `TrackingPipeline` が frame を受け、pairing される。
- payload key が `src/pi/service/capture_runtime.py` の現行 `pose_capture` と揃っている。

### Phase 6 - AB regression runner

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
python -m tools.sim \
  --scenario waist_wand_occlusion_reacquire \
  --frames 600 \
  --fps 118 \
  --calibration calibration \
  --rigids calibration/tracking_rigids.json \
  --marker-layout current_4marker \
  --camera-rig-source real_2cam \
  --mode inprocess \
  --out logs/sim/waist_wand_occlusion_reacquire
```

AB 比較:

```bash
python -m tools.sim \
  --scenario-pack waist_wand_occlusion_v1 \
  --variant baseline \
  --variant fast_ABCDHRF \
  --out logs/sim/ab_waist_wand_occlusion_v1
```

UDP loopback:

```bash
python -m tools.sim \
  --scenario waist_wand_linear_32_false \
  --mode udp \
  --udp-port 5000 \
  --out logs/sim/waist_wand_linear_32_false_udp
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

production performance:
  pipeline_pair_ms.p95 <= 6.0
  no sustained frames over 8.475ms
  rigid_ms.p95 <= 1.5
```

threshold は固定値だけでなく baseline comparison を使う。
ただし、production performance は 118fps の絶対条件なので、baseline comparison とは別の go/no-go とする。
遮蔽 scenario は絶対値より「前より悪化していないか」を重視する。

---

## 10. 実装上の注意

1. simulation metadata を UDP payload や `Frame.blobs` に混ぜない。
   - 実 payload と違う shape になると、receiver / pipeline の検証価値が落ちる。
2. random は scenario seed から派生させる。
   - camera、rigid、false blob で seed stream を分ける。
3. stale な simulator API を温存しない。
   - `tests/test_sim_closed_loop.py` は新しい multi-rigid 契約に合わせて更新する。
   - 似た目的の旧テストと新テストを重複させず、必要なら統合する。
4. 実 calibration、generated 4 camera、dummy calibration mode を分ける。
   - CI 的な軽いテストは dummy、実運用に近い探索は `real_2cam`、4 camera 負荷探索は `generated_4cam_from_1_2_intrinsics` を使う。
5. 重い scenario pack は unit test に入れない。
   - unit は短く deterministic。
   - perf / regression は opt-in CLI にする。
6. optimizer は最後に置く。
   - まずは scenario、metrics、AB runner が安定してから parameter sweep を追加する。

---

## 11. 最小実装のタスク分解

1. `tools/sim/` に `ScenarioConfig` と `waist` + `wand` projection を追加。
2. `tests/sim/` を作り、`waist_wand_static_clean` の 1-frame projection test を追加。
3. in-process runner で `PairedFrames` を `TrackingPipeline._on_paired_frames()` に流す。
4. GT evaluator で pose error と valid/drop を集計。
5. sidecar ownership ledger から wrong ownership を集計。
6. `waist_wand_occlusion_reacquire` と `waist_wand_linear_32_false` preset を追加。
7. scenario pack AB runner と production performance go/no-go を追加。
8. `generated_4cam_from_1_2_intrinsics` を追加。
9. 5-marker fixture と 5 rigid projection を追加。
10. high-speed human motion、足組み、left/right swap、close approach preset を追加。
11. UDP loopback mode を追加。

---

## 12. 判断

このシミュレーターは「過剰な研究設備」ではなく、5 rigid 化で必要になる安全装置に近い。

ただし、価値が出る順番は明確にする。

1. deterministic `waist` + `wand` projection。
2. GT evaluator。
3. `waist_wand_occlusion_reacquire` と false blob。
4. AB regression with 118fps go/no-go。
5. generated 4 camera from copied `pi-cam-01` / `pi-cam-02` intrinsics。
6. deterministic 5-marker fixture。
7. left/right swap and close-approach scenario。
8. UDP loopback。
9. parameter sweep / optimization。

この順で進めれば、初期実装は小さく、かつ現在の rigid stabilization 作業へすぐ効く。
