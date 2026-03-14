# GUI Operator Runbook

Host GUI を使って、オペレーターが 1 セッションを安全に回すための手順書です。
対象は次の 2 つです。

- calibration ページで pose capture と外部較正を完了する
- tracking ページで live 状態を確認する

開発者向けの詳細説明は省き、現場で必要な判断と復旧に絞ります。

## 1. セッション開始前チェック

開始前に次を満たしていることを確認します。

- Host と各 Pi が同じネットワークで通信できる
- 各 Pi で `src/pi/capture.py` が起動済み
- 各 Pi に `linuxptp` が導入済みで、`pi-cam-01` が Grandmaster、他 Pi が client として常時追従している
- `src/deploy/hosts.ini` の IP と `camera_id` が最新
- 各カメラの intrinsics JSON が `calibration/` にある

`linuxptp` は install するだけでは足りません。標準運用は software timestamping の `ptp4l` 常駐です。最低限、次が整っている必要があります。

- `pi-cam-01` で Grandmaster 側の `ptp4l` が起動している
- 他 Pi で client 側の `ptp4l` が起動している
- `pmc GET TIME_STATUS_NP` が実行できる
- `capture.py` の `ping` で `clock_sync.status` が `locked` または短時間の `degraded` に収まる

初期セットアップは `sudo ./src/pi/setup_ptp.sh master` または `sudo ./src/pi/setup_ptp.sh slave` を使うと、`linuxptp` 導入、unit 配置、service 起動まで 1 コマンドで入ります。標準運用は `software` timestamping です。

以前の Loutrack PTP 設定を全部戻す必要がある場合は、先に `sudo ./src/pi/revert_ptp.sh` を実行してから再セットアップします。

最低限確認するファイル:

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `logs/wand_gui_settings.json`
- `calibration/extrinsics_pose_v2.json`（既存結果がある場合）

## 2. Pi 側の通常起動

通常運用では、各 Pi で `--debug-preview` を付けて起動します。
これを基準手順にします。

### 2.1 Pi で起動するコマンド

Pi のデスクトップ端末、または `DISPLAY` を通した SSH shell で次を実行します。

```bash
.venv/bin/python src/pi/capture.py --debug-preview
```

`--debug-preview` をONにするとプレビューがつきます。

### 2.1.1 PTP の最小確認

`capture.py` を起動する前に、各 Pi で PTP が常時追従状態に入っていることを確認します。

確認の要点:

- `pi-cam-01` は fixed Grandmaster として動かす
- 他 Pi は `pi-cam-01` に追従する client として動かす
- 運用中に手動の定期再同期はしない
- sanity check は約 60 秒ごとで十分

最低限の確認コマンド例:

```bash
pmc -u -b 0 "GET TIME_STATUS_NP"
```

この結果で slave 側 `master_offset` が読めない、または `gmPresent true` にならない場合は、`linuxptp` を入れただけで設定や常駐が未完了です。`capture.py` 側では自動設定しないので、先に PTP 側を直します。

### 2.2 起動直後に見ること

Pi 側 preview で次を確認します。

- 画面が出る
- blob 円表示が更新される
- 露出が極端に破綻していない
- preview が真っ黒やフリーズではない

headless 環境で `DISPLAY` が無い場合、サービス自体は動いても preview は自動で無効化されます。
通常運用では preview を見ながら進めたいので、Pi 側画面が出せる状態で起動するのを優先します。

## 3. Host GUI 起動

Host で GUI を起動します。

```bash
python src/host/wand_gui.py --host 0.0.0.0 --port 8765 --udp-port 5000
```

ブラウザで次を開きます。

```text
http://<HOST_IP>:8765/
```

画面は `Calibration` と `Tracking` の 2 タブです。通常の運用順は `Calibration` → `Tracking` です。

## 4. Calibration ページ運用

### 4.1 カメラ発見

1. `Refresh` を押す
2. 対象カメラが一覧に出ることを確認する
3. 対象カメラを選択する
4. `Ping` を押して `ack=true` 相当の応答を確認する

正常の目安:

- 対象カメラが expected 台数だけ見える
- `Healthy` が崩れていない
- `Clock` が `locked` に近く、`unknown` が継続しない
- `Last error` が空、または古いエラーで増えていない

### 4.2 Blob Detection Adjustment

単一点ターゲットをカメラに見せながら次を調整します。

- `threshold`
- `circularity`
- `blob min/max diameter`
- `exposure`
- `gain`
- `fps`
- `focus`

判断基準:

- pose capture 用の単一点が安定して拾える
- 反射や背景ノイズが大量に拾われない
- カメラごとの差が大きすぎない

調整後は数秒待ち、blob diagnostics が落ち着くことを確認します。

### 4.3 Mask Adjustment

背景だけが見えている状態で `Build Mask` を実行します。

運用ルール:

- wand や人を画角から外してから押す
- 動く物体が画面内にある状態で mask を作らない
- マスク作成後に再度 wand を見せ、必要な点だけ残るか確認する
- `READY` に戻ったことを確認してから `Start Pose Capture` を押す
- debug preview は `Build Mask` 中だけ一時的に止まることがあるが、それ以外では capture 中も同じ window を使い続ける

失敗の兆候:

- wand を見せても 4 点が消える
- 背景ノイズがむしろ増える

この場合は `Build Mask` をやり直します。

### 4.4 Pose Capture

1. `Start Pose Capture` で単一点ターゲットを空間全体で動かし、`logs/extrinsics_pose_capture.jsonl` を作る
2. `Stop Pose Capture` で収録を止める

`Generate Extrinsics` はこの pose log だけを入力にして similarity extrinsics を作ります。
metric / floor / world はこのフローではまだ解かず、出力 JSON では `unresolved` のまま残ります。
`Start Pose Capture` を含む capture 系 start はすべて static mask 必須なので、mask を消した直後や `READY` でないカメラが混ざっている状態では開始できません。

### 4.5 Extrinsics Generation

`Generate Extrinsics` を実行し、次を確認します。

- `Pair Window (us)` は既定 `2000`
- `Min Pairs` は既定 `8`
- `Min Pairs` は「全体サンプル数」だけでなく「各カメラが ref camera と同時観測したサンプル数」にも適用される

- エラーで止まらない
- `camera count` が期待台数に近い
- 最新出力が `calibration/extrinsics_pose_v2.json` に書かれる
- `pose.solve_summary` に `usable_rows` / `complete_rows` / `median_reproj_error_px` / `p90_reproj_error_px` / `matched_delta_us_p50` / `matched_delta_us_p90` / `matched_delta_us_max` が入る

raw similarity solve の確認項目:

- `pose.solve_summary.usable_rows`
- `pose.solve_summary.complete_rows`
- `pose.solve_summary.median_reproj_error_px`
- `pose.solve_summary.matched_delta_us_p90`
- `camera_order`

補足:

- GUI は `metric.status=unresolved`, `world.status=unresolved` を明示表示する
- 現行の `Generate Extrinsics` は camera pose のみを返す

ここで失敗した場合は、まず pose capture の同時観測不足または sample の parallax 不足を疑います。

## 5. Tracking ページ運用

外部較正ができたら `Tracking` タブへ移動します。

### 4.1 開始前確認

次が見えていることを確認します。

- `Extrinsics: calibration/extrinsics_pose_v2.json` が表示される
- status badge が `Waiting` ではなく `Ready` になる
- 3D viewer が表示される

`Waiting` のままなら、extrinsics が未生成か読み込み失敗です。

### 4.2 Tracking 開始

1. rigid body を撮影空間に入れる
2. `Start Tracking` を押す
3. 数秒待って status / scene が更新されるか確認する

正常の目安:

- status badge が `Tracking`
- `frames_processed` が増える
- `poses_estimated` が増える
- 3D viewer に raw points や rigid body が出る

### 4.3 Tracking 中の見る場所

3D viewer では次を見ます。

- camera frustum の向きが明らかに壊れていない
- raw points が空間の変な遠方へ飛び続けない
- rigid body の軸と trail が滑らかに続く

Camera Health では次を見ます。

- FPS が極端に低いカメラがない
- latency が 1 台だけ大きく悪化していない
- `Missing` が急増していない
- `Blob avg` が 0 に張り付いていない

Rigid Bodies では次を見ます。

- `Valid` が維持される
- `Observed` が必要数を割り続けない
- RMS が急に悪化し続けない

### 4.4 Tracking 停止

確認が終わったら `Stop Tracking` を押します。

停止後に見る点:

- status badge が `Ready` に戻る
- 停止 summary に frame 数が残る
- ブラウザ表示がフリーズではなく停止状態へ戻る

## 6. 異常時の切り分け

### 6.1 `Refresh` してもカメラが出ない

確認順:

1. Pi 側 `capture.py` が起動中か
2. `hosts.ini` の IP が正しいか
3. Host から Pi の制御ポートへ届くか
4. UDP 5000 が Host に届くか

### 6.2 `Ping` は通るが blob が出ない

確認順:

1. lens cap やピントずれがないか
2. exposure / threshold が極端すぎないか
3. mask が wand を消していないか
4. wand の LED やマーカーに物理異常がないか
5. Pi 側 preview で blob 円が見えているか

### 6.3 extrinsics が生成できない

確認順:

1. wand 収録ログが最新か
2. 2 台以上で同時観測された区間が十分あるか
3. 対象カメラの intrinsics JSON が揃っているか
4. `camera_id` の表記ゆれがないか
5. `Min Pairs` を満たすサンプルが各カメラにあるか（不足カメラは除外される）

`calibration/extrinsics_pose_v2.json` が出ているのに台数が少ない場合:

1. `session_meta.excluded_camera_ids` を確認する
2. `session_meta.excluded_camera_reasons` の内容を確認する
3. 除外されたカメラだけ wand 収録を増やして再生成する

### 6.4 tracking が始まらない

確認順:

1. `calibration/extrinsics_pose_v2.json` があるか
2. `Extrinsics:` 表示が期待パスか
3. Camera Health に入力が来ているか
4. rigid body pattern が視野内にあるか

### 6.5 tracking は動くが品質が悪い

確認順:

1. 1 台だけ latency が悪化していないか
2. blob 誤検出が増えていないか
3. mask が古くなっていないか
4. wand/extrinsics 作成時からカメラが動いていないか
5. Pi 側 preview で tracking 対象が安定して見えているか

カメラ位置が変わった疑いがある場合は、wand 収録からやり直します。

## 7. セッション終了後に残すもの

最低限、次を保存または確認します。

- `logs/extrinsics_pose_capture.jsonl`
- `logs/extrinsics_wand_metric.jsonl`
- `logs/wand_gui_settings.json`
- `calibration/extrinsics_pose_v2.json`

記録すべき運用メモ:

- 使用した camera 台数
- 除外された camera の有無（`excluded_camera_reasons` を記録）
- 再マスク実施の有無
- tracking 品質の所感
- `pose.solve_summary` の `usable_rows` / `complete_rows` / reprojection 指標 / `matched_delta_us_p90` の異常有無

## 8. 関連資料

- [`/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/README.md)
- [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md)
- [`/docs/20_completed/24_next_steps_wand_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/20_completed/24_next_steps_wand_runbook.md)
