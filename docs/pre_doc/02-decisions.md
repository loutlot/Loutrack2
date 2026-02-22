# 決めるべきことチェックリスト

`context/request.md` は全体像が明確な一方で、実装に入る前に決めないと作業が分岐して戻りが大きい項目がいくつかあります。

このドキュメントは「決定事項の一覧」と「迷ったときの判断軸」をまとめます。

## 0. 現時点の暫定回答と未解決の決定

`context/response_to_decisions.md` の内容を反映した、2026-02-22時点の整理です。

### 暫定的に方向が決まっている項目

- `timestamp` は PTP 同期時刻を使う方向
- `image_size` は送らない方向
- `exposure/gain` はフレームメッセージには送らない方向
- SteamVR 出力は OpenVR Driver を第一候補
- 同期目標は 2-5 ms
- クラスタリングは DBSCAN を第一候補
- 姿勢推定は Kabsch 主体 + フォールバック検討
- 更新はまず SCP/SSH ベースで開始

### 未解決の決定（実装前に確定が必要）

1. `frame_index` を送るか
   - **未解決理由**: timestamp 破綻時のフォールバックが未定義
   - **おすすめ**: 送る。`uint32` の単調増加カウンタ(カメラごと)として実装し、rollover は差分計算で吸収する。
2. `timestamp` の厳密仕様
   - **未解決理由**: 単位(ns/us/ms)、送信時刻の定義(露光開始/終了/処理後)が未定義
   - **おすすめ**: `int64` の Unix epoch `us` を採用。定義は「センサ取得完了時刻(フレーム確定時)」。
3. Host→Pi 制御プロトコルの確定
   - **未解決理由**: start/stop、マスク、露光、LED制御の message schema 未定義
   - **おすすめ**: TCP JSON-RPC 風に統一。全コマンドに `request_id`, `camera_id`, `cmd`, `params`, `ack`, `error_code` を持たせる。
4. OpenVR の tracker role / serial 命名規則
   - **未解決理由**: head/chest/waist/feet の固定割り当てルール未定
   - **おすすめ**: role 固定マップを先に決める。serial は `loutrack-{role}-{index}` 形式で永続化し、再起動後も不変にする。
5. 同期失敗時の degrade 戦略
   - **未解決理由**: 許容窓外フレームの扱い(破棄/補間/予測)未定
   - **おすすめ**: 2段階運用。許容窓外は(1)直近補間で継続、連続N回失敗で(2)予測のみへ移行、復帰後に自動再同期。
6. Charuco 内部較正の成果物フォーマット
   - **未解決理由**: 保存 JSON の schema とバージョン管理方針未定
   - **おすすめ**: `calibration_intrinsics_v1.json` を定義し、`schema_version`, `camera_model`, `resolution`, `K`, `dist`, `rms_error`, `captured_at` を必須化。
7. 外部較正ワークフロー
   - **未解決理由**: 治具(B5 3点 wand)の運用手順、評価指標、合格基準未定
   - **おすすめ**: 手順を固定化。撮影ポーズ数(>=20)、視野分布、合格基準(再投影誤差閾値)をチェックリスト化して運用する。
8. DBSCAN パラメータ
   - **未解決理由**: `eps`/`min_samples` と環境別チューニング方針未定
   - **おすすめ**: 初期値を `eps=marker_diameter*0.8`, `min_samples=3` で開始し、ログ再生で環境プロファイル別にテーブル化する。
9. 姿勢推定フォールバックの発火条件
   - **未解決理由**: reprojection error 閾値/観測カメラ数/可視点数の基準未定
   - **おすすめ**: 優先順を固定。Kabsch(3D点十分)→PnP(2D-3D対応あり)→3点PnP→Kalman予測。各段の閾値を設定ファイル化する。
10. 運用スケール時の配布方式
   - **未解決理由**: 2-8台は SCP で開始可能だが、将来的な Ansible/自動化移行条件未定
   - **おすすめ**: まず SCP/SSH で開始し、台数>4 または更新頻度>週2 を超えたら Ansible へ移行する運用ルールを先に定義する。

## 1. データ契約 (最優先)

### Pi → Host (検出結果)

`context/request.md` では UDP + JSON を想定しています。

- `camera_id`: 文字列
- `timestamp`: 何を入れるか(要決定)
- `frame_index`: 付けるか(要決定)
- `blobs`: `[{x,y,area}, ...]`
- `image_size`: 送るか(要決定)
- `exposure/gain`: 送るか(要決定)

現時点メモ:
- `timestamp`: PTP同期時刻を使う方向だが、形式は未確定
- `frame_index`: 未確定
- `image_size`: 送らない方向
- `exposure/gain`: フレームには送らない方向

判断軸:
- **ペアリング**のために、`timestamp` は「ホスト時刻に直せるもの」か「フレームカウンタ」かを明確にする
- 帯域と遅延の観点で、画像本体は送らない(最初から 2D blobs のみで良い)

### Host → Pi (制御)

`context/request.md` は TCP 推奨です。

- `start/stop`
- カメラ設定(解像度/fps/シャッター/露光/ゲイン/フォーカス)
- マスク作成の開始/終了
- GPIO LED制御

未解決:
- 制御メッセージ schema (JSON) と ACK/再送ポリシー
- Host が camera_id と RasPi IP をどう紐付け管理するか

判断軸:
- **到達保証が必要**なものはTCP、落ちてもいいテレメトリはUDP、など役割で分ける

### Host → (SteamVR)

SteamVRへ5トラッカー(頭/胸/腰/左右足先)を入れる方式を決める必要があります。

- 出力API/方式(未決)
- 座標系(右手/左手、単位、原点)
- レイテンシと補間(予測)の責務(Host側 vs SteamVR側)

現時点メモ:
- OpenVR Driver を第一候補
- ただし tracker role マッピングと serial 命名規則は未確定

現実解としては OpenVR Driver が中心になります。

- OpenVR Driver公式: https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md
- 参考実装(SlimeVR): https://github.com/SlimeVR/SlimeVR-OpenVR-Driver

詳細は `docs/05-steamvr-output.md`。

## 2. 同期 (最難所)

`context/request.md` の課題にある通り、ここは早期に「到達可能な精度」と「実装コスト」を見積もる必要があります。

- クロック同期: PTP / NTP / 独自
- 撮影開始の同期: どう揃えるか
- 長時間ドリフト: 許容と補正
- ホスト側ペアリング: timestamp と許容窓、欠損時の扱い

現時点メモ:
- 同期目標は 2-5 ms
- 外部トリガは現時点では過剰投資のため後回し
- ただし「2-5 ms 達成可否を測る検証手順」は未確定

PTP導入やlibcamera SyncMode等のメモは `docs/06-pi-sync.md` にまとめました。

## 3. キャリブレーション

- 内部パラメータ: 歪みモデル、保存形式
- 外部パラメータ: カメラの設置姿勢の推定・固定方法
- Rectification: Host側でやる前提 (詳細は `context/request.md`)

現時点メモ:
- 内部: Charuco 撮影スクリプトを用意する方向
- 外部: 先駆者を参考に、B5 + 14mm球3点 wand を活用する方向
- ただし手順・品質基準・成果物 schema は未確定

先駆者の例:
- カメラパラメータJSONの形: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/camera-params.json`
- 読み込みとundistort: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`

## 4. 対応付け・三角測量

- 対応付け: エピポーラ線 + 再投影誤差 最小化の方針
- 三角測量: DLT / 非線形最適化 / RANSAC
- メトリクス: 再投影誤差、観測数(何台見えているか)、復帰時間

先駆者の例:
- DLT三角測量: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`
- エピポーラ線を使う対応付け: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`

## 5. 剛体推定(クラスタリング + 姿勢)

`context/request.md` では k-means と PnP/Kabsch を想定しています。

- クラスタリング:
  - k-means: K(=剛体数)が固定か、点数が固定か
  - 代替: DBSCAN (外れ値/欠損に強い)
- 剛体ID: 5剛体をどう区別するか(マーカ配置パターン、距離拘束、履歴)
- 姿勢推定:
  - PnP: 2D-3D対応が必要(今回の3D点群からだと使い方に工夫が要る)
  - Kabsch: 3D-3D対応(既知配置との対応付け)が必要

現時点メモ:
- DBSCAN を第一候補
- ID推定にはマーカーパターン + 履歴(カルマン) + IK + Quest head姿勢を併用する方向
- 姿勢は Kabsch 主体で、条件付きで PnP / 3点PnP / Kalman にフォールバック
- ただし各フォールバック発火条件は未確定

## 6. 更新/運用

`context/request.md` の「複数Piへ一発でスクリプト更新」も、早めに運用設計として決めると後で楽になります。

※ 行番号込みの参照が必要なら `context/request.md` を検索して該当箇所を確認してください。

- Piのイメージ管理/初期セットアップ
- スクリプト配布方式(SSH/Ansible/自前アップデータ)
- 監視: Piが何台オンラインか、fps、温度、遅延

現時点メモ:
- 対象台数は 2-8台想定
- 初期は SCP/SSH で進める方向
- GUIはあれば便利だが必須ではない
- Ansible 等への移行条件は未確定
