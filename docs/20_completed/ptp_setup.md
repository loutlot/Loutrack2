# Raspberry Pi 5 と PTP

実は Raspberry Pi 5 は Gigabit Ethernet コントローラを通じて PTP 機能を利用できます。実際のタイムスタンプ精度やサポートレベルは、カーネルバージョンやドライバの実装に依存する場合があります。

## 環境確認

PTP が利用できるかどうかは、`ethtool -T eth0` コマンドで確認します。Raspberry Pi OS (Bookworm 以降) では `linuxptp` ツールスイートを利用して PTP 同期を設定できます。`PTP Hardware Clock: 0` とインデックス番号が表示されれば PTP ハードウェアクロックが利用可能です。

```sh
$ ethtool -T eth0
```

```text
Time stamping parameters for eth0:
Capabilities:
  hardware-transmit
  software-transmit
  hardware-receive
  software-receive
  software-system-clock
  hardware-raw-clock
PTP Hardware Clock: 0
Hardware Transmit Timestamp Modes:
  off
  on
  onestep-sync
Hardware Receive Filter Modes:
  none
  all
```

## linuxptp の概要

`linuxptp` は PTP (Precision Time Protocol) を Linux システムで利用するためのツール群です。通常は **ptp4l** と **phc2sys** の両方を組み合わせて使用し、高精度な時刻同期を実現します。

## システム情報

```sh
$ cat /etc/os-release
```

```text
PRETTY_NAME="Debian GNU/Linux 12 (bookworm)"
NAME="Debian GNU/Linux"
VERSION_ID="12"
VERSION="12 (bookworm)"
VERSION_CODENAME=bookworm
ID=debian
HOME_URL="https://www.debian.org/"
SUPPORT_URL="https://www.debian.org/support"
BUG_REPORT_URL="https://bugs.debian.org/"
```

```sh
$ uname -r
```

```text
6.12.25+rpt-rpi-2712
```

> **注**: 本記事は 2025 年 5 月 21 日時点の情報です。使用している Debian は **12 (bookworm)**、カーネルは **6.12.25+rpt‑rpi‑2712** です。

## 主要なツールと PTP の基本的な仕組み・役割

- **ptp4l** – NIC の PHC を PTP ネットワーク上のマスタークロックに同期させたり、逆にマスタークロックとして振る舞ったりするデーモン。
- **phc2sys** – NIC のハードウェアクロック (PHC) とシステムクロック (CLOCK_REALTIME) を相互に同期させるプログラム。
- **Grandmaster Clock** – ネットワーク全体の時刻基準となる最も正確なクロック源。
- **Master Clock** – Grandmaster から時刻情報を受け取り、下位のスレーブに配信するクロック。
- **Slave Clock** – マスターから時刻情報を受け取り、自身のクロックをマスターに同期させる。

PTP はこれらのデバイス間で時刻情報を交換し、伝搬遅延を計算・補正することで高精度な同期を実現します。大規模・高精度が要求されるシステムでは専用の Grandmaster が必要ですが、Raspberry Pi 5 でも小規模〜中規模ネットワークのマスター／スレーブとして十分機能します。

## Raspberry Pi 5 での PTP 実装手順概要

### 必要パッケージのインストール

```sh
sudo apt update
sudo apt install linuxptp
sudo apt install ufw
sudo ufw allow 319
sudo ufw allow 320
```

### PTP マスター側の設定

#### 1. 設定ファイル (`/etc/linuxptp/ptp-master.conf`)

```ini
[global]
time_stamping hardware
masterOnly 1
priority1 128
```

#### 2. PHC とシステムクロックの同期 (phc2sys)

```sh
sudo phc2sys -s CLOCK_REALTIME -c eth0 -O 0 --step_threshold 0.5 -m
```

実行例（ログ抜粋）:

```text
phc2sys[10753.997]: eth0 sys offset -1058242430566 s0 freq +0 delay 55
phc2sys[10754.997]: eth0 sys offset -1058242483590 s1 freq -53017 delay 37
phc2sys[10755.997]: eth0 sys offset -1262 s2 freq -54279 delay 37
phc2sys[10756.990]: eth0 sys offset 16 s2 freq -53380 delay 37
```

#### 3. ptp4l の起動 (マスター)

```sh
sudo ptp4l -f /etc/linuxptp/ptp-master.conf -i eth0 -m
```

実行結果の重要ポイントは `assuming the grand master role` というメッセージです。

#### 4. 簡易起動スクリプト例 (`/etc/linuxptp/ptp-master.sh`)

```sh
#!/bin/bash
sudo ptp4l -f /etc/linuxptp/ptp-master.conf -i eth0 -m > /var/log/ptp4l.log 2>&1 &
sudo phc2sys -s CLOCK_REALTIME -c eth0 -O 0 --step_threshold 0.5 -m > /var/log/phc2sys.log 2>&1 &
```

ログ確認例:

```sh
sudo tail -f /var/log/ptp4l.log
```

> **注意**: サービス化する前の一時的な確認手段です。最終的には systemd サービスとして登録します。

### PTP スレーブ側の設定

#### 1. ptp4l の起動（スレーブモード）

```sh
sudo ptp4l -i eth0 -s -m --summary_interval 1
```

ログ例:

```text
ptp4l[4518.538]: selected /dev/ptp0 as PTP clock
ptp4l[4518.538]: port 1: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[4518.538]: port 0: INITIALIZING to LISTENING on INIT_COMPLETE
ptp4l[4518.842]: port 1: new foreign master 2ccf67.fffe.290682-1
ptp4l[4522.842]: selected best master clock 2ccf67.fffe.290682
…
```

#### 2. PHC とシステムクロックの同期 (phc2sys)

```sh
sudo phc2sys -s eth0 -c CLOCK_REALTIME -O 0 --step_threshold 0.5 -m
```

### systemd によるサービス化

#### マスター側 service ファイル

- **ptp4l‑master.service**

```ini
[Unit]
Description=PTP4L Master Daemon
After=network-online.target
Wants=network-online.target
ExecStartPre=/bin/sleep 10

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp-master.conf -i eth0 -m
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

- **phc2sys‑master.service**

```ini
[Unit]
Description=PHC2SYS Master Daemon (System Clock to PHC)
After=ptp4l-master.service
Requires=ptp4l-master.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s CLOCK_REALTIME -c eth0 -O 0 --step_threshold 0.5 -w -m
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

#### スレーブ側 service ファイル

- **ptp4l‑slave.service**

```ini
[Unit]
Description=PTP4L Slave Daemon
After=network-online.target
Wants=network-online.target
ExecStartPre=/bin/sleep 10

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -i eth0 -s -m --summary_interval 1
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

- **phc2sys‑slave.service**

```ini
[Unit]
Description=PHC2SYS Slave Daemon (PHC to System Clock)
After=ptp4l-slave.service
Requires=ptp4l-slave.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s eth0 -c CLOCK_REALTIME -O 0 --step_threshold 0.5 -w -m
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

#### サービスの登録・起動・確認

```sh
sudo systemctl daemon-reload
sudo systemctl enable ptp4l-master.service
sudo systemctl enable phc2sys-master.service
sudo systemctl enable ptp4l-slave.service
sudo systemctl enable phc2sys-slave.service

sudo systemctl start ptp4l-master.service
sudo systemctl start phc2sys-master.service
sudo systemctl start ptp4l-slave.service
sudo systemctl start phc2sys-slave.service

# ステータス確認例
sudo systemctl status ptp4l-master.service
sudo systemctl status phc2sys-master.service
sudo systemctl status ptp4l-slave.service
sudo systemctl status phc2sys-slave.service
```

> **注意**: PTP を利用して高精度な時刻同期を行う際は、`systemd-timesyncd` など他の時刻同期サービスを停止してください。

```sh
sudo systemctl stop systemd-timesyncd
sudo systemctl disable systemd-timesyncd
```

## PTP 同期の確認方法

- **スレーブ側**: `ptp4l` のログで `master offset` や `delay` を確認
  ```sh
  sudo journalctl -u ptp4l-slave.service -f
  ```
- **マスター側**: `ptp4l` の `assuming the grand master role` が表示されているか確認
  ```sh
  sudo journalctl -u ptp4l-master.service -f
  ```
- **phc2sys** のログでもクロック同期状況を確認
  ```sh
  sudo journalctl -u phc2sys-master.service -f
  sudo journalctl -u phc2sys-slave.service -f
  ```

## 時間単位の換算表

- 1 秒 (s) = 1,000 ミリ秒 (ms)
- 1 ミリ秒 (ms) = 1,000 マイクロ秒 (µs)
- 1 マイクロ秒 (µs) = 1,000 ナノ秒 (ns)

以上が Raspberry Pi 5 を用いた PTP 同期の手順と確認方法です。実際に動作させる際は、ログを定期的に確認し、オフセットが数十ナノ秒レベルに収束していることを目安にしてください。