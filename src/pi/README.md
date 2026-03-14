# Pi キャプチャサービス

`src/pi/capture.py` は Raspberry Pi 側のキャプチャサービスです。

- TCP 制御 (NDJSON): `0.0.0.0:8554`
- UDP フレーム (1 データグラムにつき 1 つの JSON): デフォルト `255.255.255.255:5000`
- デフォルトバックエンド: Raspberry Pi では `picamera2`、それ以外では `dummy`
- デフォルトキャプチャ解像度: `2304x1296`
- 起動時のログはリスンアドレス、バックエンド、プレビュー状態、キャプチャ開始/停止ステータスを出力します
- UDP ペイロードは既存キーを保持し、`timestamp_source` を追加します

## ローカルで実行（ダミーバックエンド）

macOS / Linux での開発用です。合成されたドットフレームを生成します。

```bash
.venv/bin/python src/pi/capture.py \
  --backend dummy \
  --camera-id pi-cam-01 \
  --udp-dest localhost:5000
```

## Raspberry Pi 上で実行（デフォルトで picamera2 バックエンド）

Raspberry Pi OS では `picamera2` がデフォルトのバックエンドです。`--backend picamera2` は省略可能です。

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000
```

Pi 本体でローカルに OpenCV デバッグプレビューを表示したい場合は、マスク設定中やワンドキャプチャ中、アイドル時に `--debug-preview` を付けます。

```bash
.venv/bin/python src/pi/capture.py \
  --camera-id pi-cam-01 \
  --udp-dest 255.255.255.255:5000 \
  --debug-preview
```

SSH 経由でキャプチャを開始する場合は、事前にデスクトップ X ディスプレイをエクスポートしてください。

```bash
export DISPLAY=:0
export XAUTHORITY=/home/pi/.Xauthority
```

プレビューはデバッグ用のみです。サービスがアイドル状態、`mask_start` がマスクを構築中、フレームをストリーミング中に表示され、受理されたブロブ、アクティブなマスク、現在の検出パラメータがオーバーレイされます。`DISPLAY` が設定されていない場合、サービスはウィンドウを表示しないようにプレビューを無効化し、OpenCV HighGUI に触れずに実行を続行します。

`picamera2` が利用できない環境でも、サービスは動作し続けます（例: `ping` は引き続き機能）。`start` は `ack:false` と `error_code=6` を返します。

## 1 コマンドでできる PTP 設定

Loutrack の固定トポロジー向けに、`src/pi/setup_ptp.sh` は `linuxptp` をインストールし、`systemd-timesyncd` を無効化し、ロール別の systemd ユニットを作成して即座に起動します。

### Master (pi-cam-01) 設定

```bash
sudo ./src/pi/setup_ptp.sh master
```

### Slave（他の Pi カメラ）設定

```bash
sudo ./src/pi/setup_ptp.sh slave
```

第二引数を指定するとインターフェース名を上書きできます（例: `sudo ./src/pi/setup_ptp.sh slave eth0`）。

スクリプトが作成するファイル:

- `/etc/linuxptp/loutrack-ptp.conf`
- `/etc/systemd/system/loutrack-ptp4l.service`
- `/etc/systemd/system/loutrack-phc2sys.service`

ステータス確認例:

```bash
systemctl status loutrack-ptp4l.service
systemctl status loutrack-phc2sys.service
pmc -u -b 0 "GET TIME_STATUS_NP"
```

## Master 用の手動 NTP 復旧

`pi-cam-01` の絶対時刻が怪しい場合は、capture 前に `src/pi/manual_ntp_sync.sh` を手動で実行します。

```bash
sudo ./src/pi/manual_ntp_sync.sh
```

このスクリプトは:

- `loutrack-ptp4l.service` と `loutrack-phc2sys.service` を停止
- `systemd-timesyncd` を一時的に有効化して NTP 同期
- `NTPSynchronized=yes` を待機
- `systemd-timesyncd` を再度停止
- Loutrack の PTP service を再起動

常用ではなく、長期停止後や wall clock が怪しいときの手動復旧専用です。

## タイムスタンプと PTP

`timestamp` は露光に合わせた最良の Unix エポック（マイクロ秒）として出力されます。

- `timestamp_source="sensor_metadata"`: `picamera2` のメタデータがセンサータイムスタンプを提供し、`capture.py` がエポック時間に変換します
- `timestamp_source="capture_dequeue"`: メタデータが無いまたは無効な場合、フレームキューからデキューした直後のクロック読み取りをフォールバックとして使用します

PTP は Raspberry Pi 実行時の OS レベルの前提条件です。

- `pi-cam-01` が固定 Grandmaster です
- それ以外の Pi カメラは PTP クライアントです
- `capture.py` は PTP サービスの起動/再起動を行いません
- `ping` は `pmc` を用いてベストエフォートで PTP 健全性を取得し、結果を 60 秒間キャッシュします

### 必要な Pi パッケージ（ハイレベル）

- `picamera2`（Python）
- `libcamera` スタック（システム）
- `linuxptp`（システム、`ptp4l` / `phc2sys` / `pmc` を提供）
- `numpy`, `opencv-python`（Python、リポジトリの requirements に合わせ）
