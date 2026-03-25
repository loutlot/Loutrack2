# Pi キャプチャサービス

`src/pi/service/capture_runtime.py` が Raspberry Pi 側キャプチャサービスの正規入口です。

- TCP 制御 (NDJSON): `0.0.0.0:8554`
- UDP フレーム (1 データグラムにつき 1 つの JSON): デフォルト `255.255.255.255:5000`
- デフォルトバックエンド: Raspberry Pi では `picamera2`、それ以外では `dummy`
- デフォルトキャプチャ解像度: `2304x1296`
- 起動時のログはリスンアドレス、バックエンド、プレビュー状態、キャプチャ開始/停止ステータスを出力します
- UDP ペイロードは既存キーを保持し、`timestamp_source` を追加します

正規モジュール:

- `pi.service.capture_runtime`

## ローカルで実行（ダミーバックエンド）

macOS / Linux での開発用です。合成されたドットフレームを生成します。

```bash
.venv/bin/python src/pi/service/capture_runtime.py \
  --backend dummy \
  --camera-id pi-cam-01 \
  --udp-dest localhost:5000
```

## Raspberry Pi 上で実行（デフォルトで picamera2 バックエンド）

Raspberry Pi OS では `picamera2` がデフォルトのバックエンドです。`--backend picamera2` は省略可能です。

```bash
.venv/bin/python src/pi/service/capture_runtime.py \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000
```

`--mjpeg-port` は既定で `8555` です。`http://<PI_IP>:8555/mjpeg` で headless のまま確認できます。
無効化したい場合だけ `--mjpeg-port 0` を指定します。

Pi 本体でローカルに OpenCV デバッグプレビューを表示したい場合は、マスク設定中やワンドキャプチャ中、アイドル時に `--debug-preview` を付けます。

```bash
.venv/bin/python src/pi/service/capture_runtime.py \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000 \
  --debug-preview
```

SSH 経由でキャプチャを開始する場合は、事前にデスクトップ X ディスプレイをエクスポートしてください。

```bash
export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority
```

プレビューはデバッグ用のみです。サービスは single camera pipeline で動き、idle / mask build / capture のすべてが同じ backend から処理されます。OpenCV HighGUI は専用 preview thread だけが触り、preview は同じウィンドウを継続利用します。高負荷時は preview frame を drop して capture を優先します。`DISPLAY` が設定されていない場合、サービスはウィンドウを表示しないようにプレビューを無効化し、OpenCV HighGUI に触れずに実行を続行します。

MJPEG サーバー起動と描画負荷は分離されています。起動直後は MJPEG render が OFF で、`set_preview` API で ON/OFF を切り替えます。

```bash
PYTHONPATH=src .venv/bin/python -m host.control \
  --ip <PI_IP> \
  --camera-id pi-cam-01 \
  set_preview \
  --render-enabled true
```

描画 ON 時は OpenCV preview と同じ blob/mask/text overlay を MJPEG へ反映し、必要なら Charuco overlay も重畳できます。

`start` は `capture` / `pose_capture` / `wand_metric_capture` の全 mode で static mask を必須にします。つまり `Build Mask` 実行後に `READY` へ入っていることが開始条件です。mask は `mask_start` 中の生成処理を除き常時適用されます。

`pose_capture` は full detection を維持し、UDP payload でも full blobs をそのまま送ります。`blob_count` と `quality` は additive metadata として付与し、single-blob row の採否は host 側が `blob_count` と `len(blobs)` を見て判断します。

`picamera2` が利用できない環境でも、サービスは動作し続けます（例: `ping` は引き続き機能）。`start` は `ack:false` と `error_code=6` を返します。

## 1 コマンドでできる PTP 設定

Loutrack の固定トポロジー向けに、`src/pi/setup_ptp.sh` は `linuxptp` をインストールし、`systemd-timesyncd` を無効化し、ロール別の systemd ユニットを作成して即座に起動します。master では boot 時に一度だけ NTP で wall clock を寄せてから PTP を開始する bootstrap service も入ります。supported path は `software` timestamping です。

### Master (pi-cam-01) 設定

```bash
sudo ./src/pi/setup_ptp.sh master
```

### Slave（他の Pi カメラ）設定

```bash
sudo ./src/pi/setup_ptp.sh slave
```

第二引数を指定するとインターフェース名を上書きできます。第三引数で `software|hardware` を選べます。

```bash
sudo ./src/pi/setup_ptp.sh slave eth0 software
sudo ./src/pi/setup_ptp.sh slave eth0 hardware
```

`software` モードでは `loutrack-ptp4l.service` だけを起動します。`loutrack-phc2sys.service` は作成しません。
`hardware` は実験用オプションとして残していますが、現行の標準運用には含めません。

スクリプトが作成するファイル:

- `/etc/linuxptp/loutrack-ptp.conf`
- `/etc/systemd/system/loutrack-ptp4l.service`
- `/etc/systemd/system/loutrack-phc2sys.service`

標準の software 運用でのステータス確認例:

```bash
systemctl status loutrack-ptp4l.service
/usr/sbin/pmc -u -b 0 -s /var/run/ptp4lro -i /tmp/pmc.readme "GET TIME_STATUS_NP"
```

## PTP 設定の巻き戻し

Loutrack が作成した PTP 設定を全部戻すには、`src/pi/revert_ptp.sh` を実行します。

```bash
sudo ./src/pi/revert_ptp.sh
```

このスクリプトは:

- `loutrack-ptp4l.service` / `loutrack-phc2sys.service` を停止して disable
- `/etc/systemd/system/` の Loutrack unit を削除
- `/etc/linuxptp/loutrack-*` を削除
- `systemd-timesyncd` を再度 enable/start
- `timedatectl set-ntp true` で NTP を戻す

`linuxptp` パッケージ自体は削除しません。再セットアップや手動検証で再利用できるように残します。

## Master 用の手動 NTP 復旧

`setup_ptp.sh master` は boot 時に一度だけ NTP bootstrap を挟むため、通常の起動では手動介入なしで `pi-cam-01` の wall clock を寄せてから PTP が始まります。それでも絶対時刻が怪しい場合は、capture 前に `src/pi/manual_ntp_sync.sh` を手動で実行します。

```bash
sudo ./src/pi/manual_ntp_sync.sh
```

このスクリプトは:

- `loutrack-ptp4l.service` と `loutrack-phc2sys.service` を停止
- `systemd-timesyncd` を一時的に有効化して NTP 同期
- `NTPSynchronized=yes` を待機
- `systemd-timesyncd` を再度停止
- Loutrack の PTP service を存在するものだけ再起動

常用ではなく、長期停止後や wall clock が怪しいときの手動復旧専用です。

## タイムスタンプと PTP

`timestamp` は露光に合わせた最良の Unix エポック（マイクロ秒）として出力されます。

- `timestamp_source="sensor_metadata"`: `picamera2` のメタデータがセンサータイムスタンプを提供し、`capture_runtime.py` がエポック時間に変換します
- `timestamp_source="capture_dequeue"`: メタデータが無いまたは無効な場合、フレームキューからデキューした直後のクロック読み取りをフォールバックとして使用します

PTP は Raspberry Pi 実行時の OS レベルの前提条件です。

- `pi-cam-01` が固定 Grandmaster です
- それ以外の Pi カメラは PTP クライアントです
- `capture_runtime.py` は PTP サービスの起動/再起動を行いません
- `ping` は `pmc` を read-only UDS socket 経由で用いてベストエフォートで PTP 健全性を取得し、結果を 60 秒間キャッシュします

### 必要な Pi パッケージ（ハイレベル）

- `picamera2`（Python）
- `libcamera` スタック（システム）
- `linuxptp`（システム、`ptp4l` / `phc2sys` / `pmc` を提供）
- `numpy`, `opencv-python`（Python、リポジトリの requirements に合わせ）
