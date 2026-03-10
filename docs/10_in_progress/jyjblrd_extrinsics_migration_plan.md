# jyjblrd_extrinsics_migration_plan

## 0. 結論

現行の Loutrack2 の extrinsics 生成は、
[`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
で 4 点 wand を 1 フレームごとにラベル付けし、その対応を使って multiview BA まで持っていく構成です。

この構成は「既知剛体を使える」という理論上の強みはある一方で、今回の実データ条件では次の 2 点が支配的な失敗要因になっています。

- wand が小さく、2m 前後の距離では 4 点の画素配置差が小さい
- 失敗の大半が BA より前、つまり `同一フレーム内の 4 点対応付け` で起きている

そのため、[`/references/jyjblrd`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd)
の方式を参考にした次の移行が妥当です。

- extrinsics 校正では 4 点 wand を primary 観測として使わない
- 校正時は `対応が自明な少数点` を長時間ためる
- `F/E -> 初期姿勢 -> BA` のバッチ solver に寄せる
- 既知剛体は `スケール決定 + floor 検出 + 最終検証` に使う

この文書は、その migration を Loutrack2 の既存コードへ入れるための詳細計画です。

---

## 1. 参照実装の何を採るか

参照元:

- [`/references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py)
- [`/references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py)
- [`/references/jyjblrd/theory.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/theory.md)

採用する核は次です。

- 校正用観測を複数フレームためて一括推定する
- `findFundamentalMat` / `Essential` / `recoverPose or motionFromEssential` で初期姿勢を作る
- 最後は再投影誤差の BA で詰める

採用しないものは次です。

- intrinsics の再最適化
- live triangulation 用の匿名 blob 対応付けを、そのまま校正入力へ使うこと
- 「隣接カメラ順にだけ」姿勢をつなぐ逐次構成

Loutrack2 で必要なのは、参照実装のライブ側ではなく、`calculate-camera-pose` 側の
`複数フレーム蓄積 -> 初期化 -> BA` という思想です。

---

## 2. 現行コードとの差分

### 2.1 いまの主経路

現行の主経路は次です。

- GUI:
  - [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py)
- 収録:
  - [`/src/host/wand_session.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_session.py)
  - [`/src/pi/capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/pi/capture.py)
- solver:
  - [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
  - [`/src/camera-calibration/wand_label.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_label.py)
  - [`/src/camera-calibration/wand_samples.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_samples.py)
  - [`/src/camera-calibration/wand_bundle_adjustment.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_bundle_adjustment.py)

論理は以下です。

1. `wand_capture` で 4 blob を収録
2. 各フレームで `[elbow, short, mid, long]` にラベル
3. multiview sample を時刻近傍で構築
4. `E` で pair 初期化
5. 4 点 wand pose と camera extrinsics を joint BA

### 2.2 どこが壊れているか

実データから見ると、失敗の大半は solver 後段ではなく前段です。

- `temporal_jump` 落ちが多い
- baseline が壊れる
- GIF 上でラベル飛びが確認できた
- 有効サンプルが全ログ長に対して短い区間に集中する

これは、「4 点 wand の幾何制約が弱い」のではなく、
`小さく遠い 4 点を 2D だけで安定順序付けするのが難しい` ことを示しています。

したがって migration の本質は、
`既知剛体を活かす` ではなく `校正観測の組合せ問題を減らす` ことです。

---

## 3. 新アーキテクチャ

## 3.1 方針

extrinsics 校正を次の 3 レイヤへ分割します。

1. `capture`
   - 校正用観測を集める
2. `solve`
   - 対応済み 2D 点列から camera pose を解く
3. `metric/validate`
   - スケール、床、原点、wand 再投影で品質を確定する

これにより、`4 点 wand を見ながら solve する` のではなく、
`簡単な観測で extrinsics を解き、既知剛体はあとで使う` 構成へ変えます。

## 3.2 校正観測の前提

新しい校正観測は次を前提にします。

- 各フレームで各カメラは 1 個の校正点だけを見る
- その 1 点はカメラ間で同じ物理点を見ている
- これを数百フレーム以上ためる
- 観測点は空間内を十分に動く

この方式なら 4 点の intra-frame correspondence がなくなります。

## 3.3 既知剛体の役割

既知剛体は捨てません。役割を変えます。

- 主用途:
  - metric scale 決定
  - floor plane 検出
  - world origin / axis 決定
  - solver 結果の再投影検証
  - tracking 開始前の sanity check
- 非主用途:
  - extrinsics の primary initialization

この分離が、今回の migration の一番重要な設計判断です。

---

## 4. 実装対象と責務

## 4.1 新規モジュール

### A. `extrinsics_capture.py`

新規:

- [`/src/camera-calibration/extrinsics_capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_capture.py)

責務:

- 校正用 JSONL の schema 定義
- `single-point` 観測の reader / validator
- log summary の作成

データ型案:

```python
@dataclass(frozen=True)
class PoseCaptureObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_point: np.ndarray  # shape=(2,)
    blob_area: float
    blob_count: int
    quality: float

@dataclass(frozen=True)
class PoseCaptureSample:
    sample_id: int
    timestamps: dict[str, int]
    image_points_by_camera: dict[str, np.ndarray]  # camera_id -> (2,)
```

### B. `extrinsics_samples.py`

新規:

- [`/src/camera-calibration/extrinsics_samples.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_samples.py)

責務:

- camera ごとの単点観測から multiview sample を構築
- 時刻窓と単調性制約を適用
- sample quality を計算

### C. `extrinsics_initializer.py`

新規:

- [`/src/camera-calibration/extrinsics_initializer.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_initializer.py)

責務:

- camera pair ごとの `F/E` 初期化
- cheirality / inlier score による候補選定
- camera graph の edge scoring
- spanning tree による初期 pose 展開

### D. `extrinsics_ba.py`

新規:

- [`/src/camera-calibration/extrinsics_ba.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_ba.py)

責務:

- カメラ姿勢と未知 3D 点を同時最適化
- robust loss 付き reprojection BA
- 最適化統計を返す

最適化変数:

- reference 以外の camera の `rotvec + t`
- 各 sample の 3D point `X_k`

ここでは wand 剛体 pose ではなく、`sample ごとに 1 個の未知 3D 点` を置きます。
`jyjblrd` 方式に合わせると、この形が最も単純です。

### E. `extrinsics_scale.py`

新規:

- [`/src/camera-calibration/extrinsics_scale.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_scale.py)

責務:

- scale factor を既知距離から決める
- 床面、原点、軸方向を定義する
- extrinsics を world 座標系へ正規化する

scale / floor の候補:

- 4 点 wand の既知距離と床接地姿勢
- 2 点だけ見せる簡易スケール棒
- 既知カメラ間距離を使う手動 override

### F. `extrinsics_validate.py`

新規:

- [`/src/camera-calibration/extrinsics_validate.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_validate.py)

責務:

- wand 4 点を使った再投影 validation
- baseline / reprojection / cheirality / triangulation angle の診断
- floor plane 整合の診断
- GUI 向け summary 作成

## 4.2 既存ファイルへの変更

### `calibrate_extrinsics.py`

役割を全面的に変えます。

- 現行の `solve_wand_extrinsics()` は削除対象
- 新規 `solve_pose_capture_extrinsics()` を主 solver にする
- wand は solver 入力ではなく `scale/floor/validate` の別段として接続する

推奨構成:

```python
def solve_extrinsics(...) -> dict[str, Any]:
    similarity_result = solve_pose_capture_extrinsics(...)
    metric_result = apply_metric_alignment_from_wand(...)
    return validate_extrinsics(metric_result, ...)
```

### `wand_gui.py`

必要変更:

- `Generate Extrinsics` は `pose_capture + wand_metric` 前提の単一路線にする
- `pose_capture` 用の log path と summary を出す
- `wand floor/scale` の summary を出す
- validation summary を既存の quality 表示へ統合する

### `wand_session.py`

必要変更:

- `wand_capture` は calibration 用の主経路から外す
- `pose_capture` セッションを主経路にする
- `wand_metric_capture` セッションを追加し、床置き wand の数フレームを別保存する
- 1 点だけを収録する専用 API を持たせる
- 将来のため `capture kind` を明示する

### `pi/capture.py`

必要変更:

- `mode in ("capture", "pose_capture", "wand_metric_capture")` に再編する
- `pose_capture` 時は largest / brightest 1 blob を選んで stream
- `wand_metric_capture` 時は床上の 4 点 wand を従来どおり収録する
- `blob_count != 1` や低品質時は quality を落とすか sample 不採用にする

---

## 5. ロジック詳細

## 5.1 収録ロジック

`pose_capture` は次の動作にします。

1. operator が単一点ターゲットを空間全体へ動かす
2. 各 Pi は blob を検出
3. 各フレームで `best blob` を 1 個だけ選ぶ
4. Host は camera ごとに JSONL 保存
5. solver は収録終了後に一括実行

`best blob` の選び方:

- 第一優先: 最大面積
- 第二優先: 中心からの距離
- 第三優先: 前フレーム位置との距離

理由:

- 単点 capture なのに複数反射があるケースを減らすため

## 5.2 sample 構築

reference camera を 1 台決め、各 camera の時系列を単調に走査して sample を作ります。

条件:

- sample 内で最低 2 camera 観測
- `max(timestamp) - min(timestamp) <= pair_window_us`
- camera ごとに index は後戻りしない

品質指標:

- visible camera count
- sample span us
- camera ごとの timestamp delta

## 5.3 初期姿勢

pair `(i, j)` ごとに 2D 対応を集め、
`cv2.findFundamentalMat` または正規化済み点で `cv2.findEssentialMat` を行います。

pair quality:

- inlier ratio
- triangulation angle proxy
- usable point count

edge score:

```text
score = usable_points * inlier_ratio * angle_score
```

これで camera graph を張り、reference camera から spanning tree で world pose を展開します。

## 5.4 BA

コスト関数:

```text
r(i, k) = project(K_i, R_i, t_i, X_k) - u_ik
```

ここで `X_k` は sample `k` の未知 3D 点です。

最適化:

- `scipy.optimize.least_squares`
- loss は `huber` か `cauchy`
- reference camera は固定

この段階では scale は未定です。
つまり結果は similarity までしか決まりません。

## 5.5 scale 決定

scale は別段で決めます。

候補の優先順位:

1. 床に置いた 4 点 wand から `scale + floor plane + world axes` を同時決定
2. operator が 2 点既知距離を振る短い追加 capture
3. 手動 scale override

推奨は 1 です。
つまり extrinsics は `pose_capture` で similarity まで解き、最後に床置き wand で
metric scale と floor を一気に決めます。

wand を床に置けるなら、次の拘束を同時に使えます。

- `short`, `mid`, `long`, `elbow` の既知距離
- wand の一部または全点が同一床面上にあること
- wand の長辺方向を world 軸の 1 つとして採ること

推奨ロジック:

1. similarity extrinsics で wand 4 点を三角測量
2. 既知 wand モデルとの similarity alignment を解く
3. wand 点群から床面法線を推定する
4. 床法線を `+Z` または `+Y` に合わせる剛体変換を適用する
5. `elbow` または wand 中点を world origin に置く

## 5.6 validation

scale / floor 決定後、4 点 wand log を使って validation を行います。

見る指標:

- per-camera median reprojection error
- p90 reprojection error
- baseline range
- positive-depth ratio
- wand edge length consistency
- floor residual
- world-up consistency

これを `session_meta.validation` に保存します。

---

## 6. GUI / 運用フロー

新フローは次です。

1. intrinsics があることを確認
2. `Pose Capture Start`
3. 単一点ターゲットを空間内へ動かす
4. `Pose Capture Stop`
5. `Generate Extrinsics`
6. `Wand Metric Capture` で wand を床に置いた短い収録を取る
7. `Apply Wand Scale/Floor`
8. validation 結果確認

これにより、現行の
`4 点 wand を見せながら extrinsics を直接解く`
という UI から分離されます。

GUI 追加項目:

- pose capture log path
- wand metric log path
- floor / origin summary
- validation summary

---

## 7. 出力 schema

既存の
[`/calibration/calibration_extrinsics_v1.json`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/calibration/calibration_extrinsics_v1.json)
互換は維持します。

追加する `session_meta`:

```json
{
  "method": "pose_capture",
  "pose_capture_log_path": "logs/extrinsics_pose_capture.jsonl",
  "wand_metric_log_path": "logs/extrinsics_wand_metric.jsonl",
  "scale_source": "wand_floor_metric",
  "floor_source": "wand_floor_metric",
  "sample_count": 480,
  "accepted_sample_count": 312,
  "camera_graph_edges": [...],
  "optimizer": {
    "iterations": 23,
    "cost": 12.4
  },
  "validation": {
    "median_reproj_error_px": 1.6,
    "p90_reproj_error_px": 3.9,
    "positive_depth_ratio": 0.997,
    "floor_residual_mm": 2.1
  }
}
```

重要なのは downstream 互換です。
[`/src/host/geo.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/geo.py)
は camera ごとの `rotation_matrix`, `translation_m` を読めればよいので、
トップレベルの camera 配列は維持します。

---

## 8. 段階的 migration 計画

## Phase 1. 収録経路の置換

目的:

- extrinsics の入力を `4 点 wand direct solve` から `pose_capture + wand_metric_capture` へ置き換える

変更:

- `pose_capture` log writer を追加
- `wand_metric_capture` log writer を追加
- `wand_gui.py` の calibration UI を新フローへ置換

完了条件:

- GUI から `Pose Capture` と `Wand Metric Capture` の両方が取れる
- 旧 `Generate Extrinsics from wand_capture` 動線は消える

## Phase 2. similarity solver 実装

目的:

- `pose_capture` から similarity extrinsics を安定に得る

変更:

- `extrinsics_samples.py`
- `extrinsics_initializer.py`
- `extrinsics_ba.py`

完了条件:

- `pose_capture` のみで similarity pose が出る
- camera graph と optimizer stats が JSON に残る

## Phase 3. wand metric/floor 実装

目的:

- 床置き wand から metric scale と floor を確定する

変更:

- `extrinsics_scale.py`
- `extrinsics_validate.py`
- GUI の `Apply Wand Scale/Floor` 操作

完了条件:

- similarity 解から metric extrinsics を作れる
- floor / origin / axis が JSON に残る
- validation summary が GUI で見える

## Phase 4. 運用固め

目的:

- runbook / tests / tracking 接続までを新フローで閉じる

変更:

- runbook 更新
- GUI summary 更新
- tracking 側の読み込み確認

完了条件:

- 通常運用が `pose_capture -> wand_metric_capture -> generate/apply -> tracking` で完結する

---

## 9. テスト計画

### 単体テスト

追加:

- [`/tests/test_extrinsics_capture.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_extrinsics_capture.py)
- [`/tests/test_extrinsics_samples.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_extrinsics_samples.py)
- [`/tests/test_extrinsics_initializer.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_extrinsics_initializer.py)
- [`/tests/test_extrinsics_ba.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_extrinsics_ba.py)

観点:

- sample builder の単調性
- `F/E` 初期化の edge selection
- BA が既知 synthetic pose に収束すること
- scale solve が既知距離を再現すること

### 統合テスト

追加:

- [`/tests/test_wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_gui.py)
  - `pose_capture` と `wand_metric_capture` の payload を通す
- [`/tests/test_wand_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_extrinsics.py)
  - `pose_capture -> metric/floor` 出力が schema 互換であることを確認

### 実機確認

必要確認:

- GUI から `pose_capture` 収録が止まらないこと
- GUI から `wand_metric_capture` 収録が止まらないこと
- 出力 JSON が tracking で読めること
- wand validation の reprojection が許容範囲に入ること
- floor residual が許容範囲に入ること

---

## 10. リスクと対策

### リスク 1. 単点 capture でも誤検出が入る

対策:

- `blob_count == 1` を強く優先
- quality score を保存
- BA 前に low-quality sample を除外

### リスク 2. similarity 解の scale 決定が不安定

対策:

- wand を `scale + floor` 専用に使う
- scale/floor solve を独立ステップにする
- GUI に manual override を残す

### リスク 3. 既存運用と UI が大きく変わる

対策:

- UI を新フローへ一気に置き換える
- 旧フローを残さない前提で runbook も同時に更新する
- runbook を migration に合わせて更新

### リスク 4. world 座標の定義が曖昧になる

対策:

- scale, floor, origin を明示的な別工程にする
- `session_meta` に source を必ず残す

---

## 11. 実装順

実装順は次で固定します。

1. `pose_capture` の log schema と収録経路
2. `wand_metric_capture` の log schema と収録経路
3. sample builder
4. `F/E` 初期化
5. BA
6. wand による scale / floor solve
7. validation
8. GUI 切替
9. runbook / tests 更新

この順にする理由は、最初に `収録方式` を変えない限り、以降の solver 改善が実データで評価できないためです。

---

## 12. 最終判断

今回の migration は「solver を少し直す」規模ではありません。
主に変わるのは次です。

- 観測対象
- 収録モード
- solver の未知変数
- scale の扱い
- GUI の運用順

したがって、大規模 migration として扱うのが正しいです。

それでもやる価値はあります。
理由は、現在の失敗が `4 点 wand という既知剛体をどう最適化するか` より前の、
`フレーム内の対応付けが壊れる` ことに集中しているからです。

`jyjblrd` 寄りへ移す本質は、精巧な BA ではなく、
校正時に解くべき correspondence 問題の難易度を根本的に下げることです。

---

## 13. 実装反映ログ（2026-03-11）

この migration plan に基づき、以下を実装反映した。

### 13.1 新規追加モジュール

- `/src/camera-calibration/extrinsics_capture.py`
  - `PoseCaptureObservation` / `WandMetricObservation` を定義
  - `pose_capture` JSONL reader と validator を実装
  - `wand_metric_capture` JSONL reader を実装
  - pose log summary（quality, single_blob_ratio など）を出力
- `/src/camera-calibration/extrinsics_samples.py`
  - camera ごとの単点観測から multiview sample を構築
  - `pair_window_us` 制約、`>=2 camera` 制約、単調 cursor 走査を実装
  - sample quality（visible count / span / mean quality）を付与
- `/src/camera-calibration/extrinsics_initializer.py`
  - pair ごとの `findEssentialMat` / `recoverPose` 初期化を実装
  - edge score（usable_points * inlier_ratio * angle_score）を算出
  - graph から reference 起点で初期 pose 展開を実装
- `/src/camera-calibration/extrinsics_ba.py`
  - camera extrinsics + sampleごとの未知 3D 点を同時最適化
  - `scipy.optimize.least_squares`（robust loss）を実装
  - optimizer stats（iterations/cost/initial_cost）を返却
- `/src/camera-calibration/extrinsics_scale.py`
  - wand metric log から scale 候補を推定
  - floor normal 推定と world-up 整列を実装
  - similarity -> metric への camera pose 変換を実装
- `/src/camera-calibration/extrinsics_validate.py`
  - per-camera / global reprojection 指標を算出
  - positive-depth ratio と baseline range を算出
  - `session_meta.validation` へ保存可能な summary を返却

### 13.2 既存 solver の置換

- `/src/camera-calibration/calibrate_extrinsics.py` を刷新
  - 新しい主経路 `solve_extrinsics(...)` を追加
    - `pose_capture -> F/E 初期化 -> BA -> wand metric scale/floor -> validation`
  - `session_meta.method = "pose_capture"` など provenance を保存
  - `cameras[].rotation_matrix / translation_m` 互換を維持
  - 旧 API 互換として `solve_wand_extrinsics(...)` は wrapper として残置

### 13.3 Capture/GUI 経路の変更

- `/src/pi/capture.py`
  - start mode を `capture|pose_capture|wand_metric_capture` に拡張
  - `pose_capture` 時は best blob 1 点を送出し、`blob_count` / `quality` を付与
  - `wand_capture` 指定は内部で `wand_metric_capture` として扱う互換を追加
- `/src/host/wand_session.py`
  - `SessionConfig.capture_kind` を追加
  - `run_session()` が capture kind に応じた start mode を送信
- `/src/host/wand_gui.py`
  - pose/wand metric の 2 種 log path 管理を追加
  - command 経路に `start_pose_capture` / `start_wand_metric_capture` / 対応 stop を追加
  - `generate_extrinsics` が `pose_log_path` / `wand_metric_log_path` を solver に渡すよう変更
  - solver ローダを `solve_extrinsics` 優先に変更（旧関数 fallback あり）

### 13.4 Schema/Docs 更新

- `/schema/control.json`
  - `start.params.mode` enum を `capture|pose_capture|wand_metric_capture` へ更新
- `/README.md`
  - 運用フローと CLI 例を新経路（pose + wand metric）へ更新
- `/docs/10_in_progress/gui_runbook.md`
  - Pose Capture / Wand Metric Capture の実運用手順を追記

### 13.5 テスト更新

- 新規追加
  - `/tests/test_extrinsics_capture.py`
  - `/tests/test_extrinsics_samples.py`
  - `/tests/test_extrinsics_initializer.py`
  - `/tests/test_extrinsics_ba.py`
- 既存更新
  - `/tests/test_wand_extrinsics.py`
  - `/tests/test_wand_gui.py`

### 13.6 この反映時点の検証状況

- 実施済み
  - `python3 -m compileall src/camera-calibration src/host src/pi tests` 通過
- 未実施
  - `pytest` による全テスト実行（環境に pytest が未導入のため）

### 13.7 残タスク（運用前チェック）

- 実機で GUI 経路（pose / wand metric capture 開始停止）が安定するか確認
- `session_meta.validation` の運用閾値（reproj, floor residual など）を現場値へ確定
- runbook を新フローで一周して手順文言を最終調整
