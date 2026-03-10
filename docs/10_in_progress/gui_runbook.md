# GUI Operator Runbook

Host GUI を使って、オペレーターが 1 セッションを安全に回すための手順書です。
対象は次の 2 つです。

- calibration ページで wand 収録と外部較正を完了する
- tracking ページで live 状態を確認する

開発者向けの詳細説明は省き、現場で必要な判断と復旧に絞ります。

## 1. セッション開始前チェック

開始前に次を満たしていることを確認します。

- Host と各 Pi が同じネットワークで通信できる
- 各 Pi で `src/pi/capture.py` が起動済み
- `src/deploy/hosts.ini` の IP と `camera_id` が最新
- 各カメラの intrinsics JSON が `calibration/` にある
- wand の 3 点マーカーが破損していない

最低限確認するファイル:

- `calibration/calibration_intrinsics_v1_<camera_id>.json`
- `logs/wand_gui_settings.json`
- `calibration/calibration_extrinsics_v1.json`（既存結果がある場合）

## 2. Pi 側の通常起動

通常運用では、各 Pi で `--debug-preview` を付けて起動します。
これを基準手順にします。

### 2.1 Pi で起動するコマンド

Pi のデスクトップ端末、または `DISPLAY` を通した SSH shell で次を実行します。

```bash
.venv/bin/python src/pi/capture.py --debug-preview
```

`--debug-preview` をONにするとプレビューがつきます。

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
- `Last error` が空、または古いエラーで増えていない

### 4.2 Blob Detection Adjustment

wand をカメラに見せながら次を調整します。

- `threshold`
- `circularity`
- `blob min/max diameter`
- `exposure`
- `gain`
- `fps`
- `focus`

判断基準:

- wand の 3 点が安定して拾える
- 反射や背景ノイズが大量に拾われない
- カメラごとの差が大きすぎない

調整後は数秒待ち、blob diagnostics が落ち着くことを確認します。

### 4.3 Mask Adjustment

背景だけが見えている状態で `Build Mask` を実行します。

運用ルール:

- wand や人を画角から外してから押す
- 動く物体が画面内にある状態で mask を作らない
- マスク作成後に再度 wand を見せ、必要な点だけ残るか確認する

失敗の兆候:

- wand を見せても 3 点が消える
- 背景ノイズがむしろ増える

この場合は `Build Mask` をやり直します。

### 4.4 Wand Capture

1. `Start Wand Capture` を押す
2. 収録中に wand を空間全体へ動かす
3. 高さ、左右、前後、中央、端を一通り通す
4. 数秒以上、複数カメラから同時に見える状態を作る
5. `Stop` で収録を終了する

オペレーションのコツ:

- 速すぎる動きは避ける
- 同じ場所だけで振らない
- カメラ 1 台だけが見える姿勢を長く続けない
- 床面付近と頭上付近も通す

生成される主なログ:

- `logs/wand_capture.jsonl`

### 4.5 Extrinsics Generation

`Generate Extrinsics` を実行し、次を確認します。

- エラーで止まらない
- camera count が期待台数に近い
- 最新出力が `calibration/calibration_extrinsics_v1.json` に書かれる

ここで失敗した場合は、まず wand 収録不足を疑います。

## 5. Tracking ページ運用

外部較正ができたら `Tracking` タブへ移動します。

### 4.1 開始前確認

次が見えていることを確認します。

- `Extrinsics: calibration/calibration_extrinsics_v1.json` が表示される
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

### 6.4 tracking が始まらない

確認順:

1. `calibration/calibration_extrinsics_v1.json` があるか
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

- `logs/wand_capture.jsonl`
- `logs/wand_gui_settings.json`
- `calibration/calibration_extrinsics_v1.json`

記録すべき運用メモ:

- 使用した camera 台数
- 失敗したカメラの有無
- 再マスク実施の有無
- tracking 品質の所感

## 8. 関連資料

- [`/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/README.md)
- [`/src/host/README.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/src/host/README.md)
- [`/docs/20_completed/24_next_steps_wand_runbook.md`](/Users/loutlot/Documents/cursor/MOCAP/Loutrack2/docs/20_completed/24_next_steps_wand_runbook.md)
