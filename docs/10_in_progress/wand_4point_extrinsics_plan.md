# wand_4point_extrinsics_plan

## 0. 目的

現行の `Extrinsics Generation` は、
[`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
で「基準カメラ vs 各カメラ」の独立 pairwise 推定を行っています。

今回の変更目的は、[`/references/jyjblrd`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd)
の実装思想を取り込みつつ、Loutrack2 では既知剛体である 4 点 wand を最大限使う形に
Extrinsics solver を再設計することです。

狙いは次の 4 点です。

- カメラごとの独立解ではなく、全カメラ整合な extrinsics を出す
- 4 点 wand の既知形状を使い、スケールを後付けではなく最適化に直接入れる
- reprojection error を最終目的関数にした BA で姿勢を詰める
- 失敗時に「同期不足」「点対応不良」「初期姿勢不良」を切り分けられる診断を残す

---

## 1. 現行コードの整理

### 1.1 現在の solver フロー

対象:

- [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
- [`/src/host/wand_session.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_session.py)
- [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py)

現行フローは概ね次です。

1. `load_wand_log()` で camera ごとの `FrameObservation` を読む
2. `_ordered_wand_points()` で 4 点を `[elbow, short, mid, long]` に正規化する
3. `pair_observations()` で基準 camera と他 camera を timestamp 近傍で greedy に 1:1 ペアリングする
4. `estimate_pair_extrinsics()` で `findEssentialMat` + `recoverPose` を実行する
5. 単位並進で三角測量した wand 辺長から `scale_m_per_unit` を中央値で求める
6. camera ごとに quality を付けて `calibration_extrinsics_v1.json` を出す

### 1.2 現状の弱点

- pairwise 独立推定なので、`cam01-cam02` と `cam01-cam03` の整合は保証されない
- スケールが「三角測量後の辺長中央値」なので、初期姿勢が悪いと連鎖的に崩れる
- 最終段に全観測を使う BA がなく、`recoverPose` の初期解がそのまま出力に近い
- ペアリングが greedy なので、高速移動やフレーム落ちで不適切な対応を拾いやすい
- 出力 quality が per-camera 中心で、どの段階で失敗したかが見えにくい

---

## 2. 参照実装から取り込むもの

参照元:

- [`/references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py)
- [`/references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py)
- [`/references/jyjblrd/theory.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/references/jyjblrd/theory.md)

### 2.1 取り込む実装思想

- `calculate-camera-pose`
  - camera 間の相対姿勢を E 行列から初期化する
  - cheirality で候補姿勢を選ぶ
- `bundle_adjustment`
  - 最終的には reprojection residual を直接最小化する
  - robust loss を使って外れ値を抑える
- `triangulate_points` / `calculate_reprojection_errors`
  - 初期値評価と品質指標を一貫した reprojection 基準で扱う

### 2.2 そのままは採用しないもの

- 焦点距離まで同時最適化する設計
  - Loutrack2 では Charuco で intrinsics を既に作るので、extrinsics solver では固定でよい
- 未知 3D 点群を丸ごと最適化する設計
  - Loutrack2 は 4 点 wand の剛体モデルが既知なので、未知点にする必要がない
- camera を隣接順に逐次連結する前提
  - Loutrack2 は camera graph を「co-visibility が高い辺」から組んだ方が安定する

結論として、参照実装の核は
「E 行列ベースで初期化し、最後は BA で全体整合を取る」です。
ただし Loutrack2 では、未知点 BA より
「既知 wand モデル + 各時刻の wand pose + 全 camera extrinsics」
の joint optimization に寄せる方が筋が良いです。

---

## 3. 採用する理論

### 3.1 幾何の基本

camera `i` の内部行列を `K_i`、外部パラメータを `R_i, t_i` とし、
wand ローカル座標の 3D 点を `X_j`、時刻 `k` の wand pose を `R_wk, t_wk` とします。

world 座標での wand 点は:

```text
X_w(k, j) = R_wk X_j + t_wk
```

camera `i` への投影は:

```text
u_ijk ~ K_i (R_i X_w(k, j) + t_i)
```

最終的に最小化したいのは reprojection residual:

```text
r_ijk = project(K_i, R_i, t_i, R_wk, t_wk, X_j) - u_ijk
```

です。

### 3.2 なぜ 4 点 wand でスケールが決まるか

参照実装は未知点群を triangulate してから別段で scale を決めています。
一方 Loutrack2 では `WAND_POINTS_MM` が既知です。

つまり BA の中で使う 3D 点 `X_j` はすでに mm 単位で固定されているため、
camera extrinsics と wand pose を最適化した時点でスケールは metric に固定されます。

このため、新 solver の本筋では:

- `recoverPose` は方向付き並進の初期化に使う
- metric scale は BA の目的関数に内包する
- 現行の `scale_m_per_unit` は初期化用または診断用へ格下げする

という整理が自然です。

### 3.3 初期化が必要な理由

BA は非線形最適化なので、初期値が悪いと局所解に落ちます。
よって次の二段構えにします。

1. camera 間相対姿勢を E 行列で初期化
2. 各時刻の wand pose を PnP か triangulation + rigid fit で初期化

その後で joint BA に入ります。

### 3.4 外れ値への扱い

外れ値は主に次の 3 種です。

- 誤ラベリングされた 4 点
- 同期ずれした timestamp pair
- 遮蔽や誤検出で壊れた 2D blob

したがって solver は各段階で外れ除去を持つべきです。

- 2D段: wand ラベリング gate
- sample段: time delta / visible camera count gate
- 初期化段: essential inlier / cheirality gate
- 最適化段: Huber か Cauchy loss + residual trimming

---

## 4. 目標アーキテクチャ

### 4.1 データ単位を pair から sample へ変える

現行は `PairedObservation(ref, other)` が基本単位ですが、
新 solver は「同一時刻近傍で複数 camera が見た 4 点 wand」を 1 sample として扱います。

推奨データ構造:

```python
@dataclass(frozen=True)
class LabeledFrameObservation:
    camera_id: str
    timestamp: int
    frame_index: int
    image_points: np.ndarray   # shape=(4, 2), order=[elbow, short, mid, long]
    quality: dict[str, float]

@dataclass(frozen=True)
class MultiViewSample:
    sample_id: int
    camera_ids: list[str]
    timestamps: dict[str, int]
    image_points_by_camera: dict[str, np.ndarray]  # 4x2
```

この単位に変えると、
pairwise 初期化にも BA にも同じ入力を流せます。

### 4.2 solver の全体フロー

```text
wand log
  -> frame labeling
  -> monotonic multiview sample builder
  -> camera graph initializer
  -> per-sample wand pose initializer
  -> joint bundle adjustment
  -> diagnostics / json output
```

---

## 5. 実装計画

## Step 1. wand ラベリングを solver 本体から分離

変更対象:

- 新規 [`/src/camera-calibration/wand_label.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_label.py)
- 既存 [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
- [`/tests/test_wand_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_extrinsics.py)

内容:

- `_ordered_wand_points()` 相当を `canonicalize_wand_points()` として独立させる
- 戻り値に点列だけでなく `collinearity_error`, `midpoint_ratio_error` を含める
- 4 点が欠けたフレームはここで明示的に落とす

理由:

- 今後の sample builder / BA とは責務が違うため
- テストを correspondence 専用に分けやすくするため

## Step 2. greedy pairing を multiview sample builder へ置換

変更対象:

- 新規 [`/src/camera-calibration/wand_samples.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_samples.py)
- 既存 [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
- 新規または統合 [`/tests/test_wand_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_extrinsics.py)

内容:

- camera 別 `LabeledFrameObservation` を入力に、単調増加制約付き sample を組む
- コストは `max(timestamp) - min(timestamp)` を基本にする
- `pair_window_us` は sample 内の最大時差ゲートとして使う
- sample ごとに `visible_camera_count`, `delta_us_stats` を記録する

採用方針:

- 最初は DP ベースの monotonic matching で十分
- Hungarian までは不要。時系列単調性が強いので DP の方が実装が素直

## Step 3. camera graph 初期化を導入

変更対象:

- 新規 [`/src/camera-calibration/extrinsics_initializer.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/extrinsics_initializer.py)
- 既存 [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)

内容:

- sample 群から camera pair ごとの 2D-2D 対応を集める
- `findEssentialMat` + `recoverPose` で pairwise relative pose を推定する
- inlier 数と reprojection proxy を edge score にして camera graph を作る
- 基準 camera から最大 spanning tree で初期 world pose を展開する

参照実装との関係:

- `references` の `calculate-camera-pose` は隣接逐次だが、
  Loutrack2 では「見え方の良い辺から張る」方が妥当

出力:

- `camera_poses_init[camera_id]`
- `pair_metrics[(cam_a, cam_b)]`

## Step 4. sample ごとの wand pose 初期化を導入

変更対象:

- 新規 [`/src/camera-calibration/wand_pose_init.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_pose_init.py)
- 既存 [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)

内容:

- 初期 camera extrinsics を使って sample ごとの wand pose を初期化する
- 優先手順は次:
  1. 可視 1 camera でも 4 点揃っていれば `solvePnP`
  2. 2 camera 以上なら三角測量した 4 点へ Kabsch を当てて cross-check
  3. 両者が大きく食い違う sample は BA から除外

理由:

- 4 点 wand は既知 3D-2D 対応なので `solvePnP` が直接使える
- `references` が抱えていた「未知3D点をまず作る」必要が Loutrack2 では薄い

## Step 5. joint bundle adjustment を本体にする

変更対象:

- 新規 [`/src/camera-calibration/wand_bundle_adjustment.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/wand_bundle_adjustment.py)
- 既存 [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)

最適化変数:

- 全 non-reference camera の `rotvec(3) + t(3)`
- 全 sample の wand pose `rotvec(3) + t(3)`

固定値:

- 各 camera の intrinsics
- `WAND_POINTS_MM`
- reference camera pose

目的関数:

```text
sum robust_norm(project(camera_i, wand_pose_k, X_j) - u_ijk)
```

推奨実装:

- `scipy.optimize.least_squares`
- loss は `huber` か `cauchy`
- 初回は `trf` で十分

重要な設計判断:

- camera extrinsics と wand pose を同時最適化し、free 3D points は持たない
- これで scale は自動的に metric へ固定される

## Step 6. quality / diagnostics を段階別に拡張

変更対象:

- [`/src/camera-calibration/calibrate_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/camera-calibration/calibrate_extrinsics.py)
- [`/schema/calibration_extrinsics_v1.json`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/schema/calibration_extrinsics_v1.json)
- [`/schema/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/schema/README.md)

追加したい指標:

- `session_meta.sample_count`
- `session_meta.accepted_sample_count`
- `session_meta.dropped_counts_by_reason`
- `quality.init_median_reproj_error_px`
- `quality.final_median_reproj_error_px`
- `quality.final_p90_reproj_error_px`
- `quality.visible_camera_count_mean`
- `quality.optimizer_iterations`
- `quality.optimizer_cost`

方針:

- additive field だけなら `schema_version` は `1.0` 維持でよい
- 既存 consumer が壊れないことを優先する

## Step 7. CLI / GUI 互換を保つ

変更対象:

- [`/src/host/wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/wand_gui.py)
- [`/tests/test_wand_gui.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_gui.py)

内容:

- 既存 API:
  - `pair_window_us`
  - `min_pairs`
  - `intrinsics_path`
  - `log_path`
  - `output_path`
  を維持する
- 内部 solver は置き換えても GUI 契約は維持する
- 可能なら移行期間中は `solver_mode=legacy|multiview` を内部フラグで持たせる

理由:

- operator 導線はすでに `wand_gui` に集約されているため
- solver 差し替えで UI まで同時に壊す必要はないため

## Step 8. テストを段階ごとに整理

変更対象:

- [`/tests/test_wand_extrinsics.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_extrinsics.py)
- 必要に応じて新規:
  - [`/tests/test_wand_label.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_label.py)
  - [`/tests/test_wand_samples.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_samples.py)
  - [`/tests/test_wand_bundle_adjustment.py`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/tests/test_wand_bundle_adjustment.py)

最低限ほしいケース:

- 4 点ラベリングが順序入替と area 入替に耐える
- 同期ずれ付きログから sample builder が単調対応を作れる
- synthetic 2 camera / 3 camera データで BA 後に reprojection が改善する
- 出力 JSON が既存 GUI loader で読める

注意:

- 一時的な旧 solver テストと新 solver テストを重複放置しない
- `tests/test_wand_extrinsics.py` の synthetic fixture は共通化する

---

## 6. 実装順の推奨

最短で壊れにくい順は次です。

1. `wand_label.py` を分離して correspondence を固定化
2. `wand_samples.py` で multiview sample を作れるようにする
3. `extrinsics_initializer.py` で camera 初期姿勢を出す
4. `wand_pose_init.py` で sample pose 初期値を作る
5. `wand_bundle_adjustment.py` を導入する
6. `calibrate_extrinsics.py` を orchestration + CLI へ薄くする
7. schema / GUI summary を更新する

この順なら、途中段階でも
「sample 数は十分か」「初期姿勢は出ているか」「BA でどれだけ改善したか」
を段階別に観測できます。

---

## 7. 完了条件

次を満たしたら、現行方式から実運用を切り替えてよいです。

- 2 camera synthetic test で `final_median_reproj_error_px < init_median_reproj_error_px`
- 3 camera synthetic test で全 camera の平行移動誤差が許容範囲内
- 実ログで `accepted_sample_count >= min_pairs`
- 実ログで `final_median_reproj_error_px <= 2.0`
- 実測 baseline と出力 baseline の差が 20% 以内
- GUI の `generate_extrinsics` 結果 summary が後方互換を保つ

---

## 8. この計画の要点

今回の置換は、
`references` の「E 行列で初期化して BA で詰める」という主筋を採用しつつ、
Loutrack2 では 4 点 wand の既知剛体モデルを使って
scale と wand pose を最適化へ直接入れるのが本質です。

つまり、移行先は:

- 現行: `pairwise essential + post scale`
- 参照ベース単純移植: `pairwise/sequence essential + free-point BA + determine-scale`
- 採用案: `multiview essential init + known-wand joint BA`

です。

この形なら、参照実装より Loutrack2 の前提に合っていて、かつ 4 点 wand を使う意味が最も大きいです。
