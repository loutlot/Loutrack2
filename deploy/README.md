# deploy - Raspberry Pi デプロイスクリプト

SSH/rsyncベースのRaspberry Pi配布・管理ツール。

## 概要

このディレクトリには、複数台のRaspberry Piへのアプリケーション配布、更新、ロールバックを行うスクリプトが含まれています。

## ファイル構成

```
deploy/
├── hosts.ini       # Piインベントリ (IP/camera_id)
├── deploy.sh       # デプロイスクリプト
├── rollback.sh     # ロールバックスクリプト
└── README.md       # このファイル
```

## 前提条件

### 1. SSH鍵の準備

```bash
# デプロイ用SSH鍵を生成
ssh-keygen -t ed25519 -f ~/.ssh/loutrack_deploy_key -N ""

# 各Piに公開鍵を登録
ssh-copy-id -i ~/.ssh/loutrack_deploy_key.pub pi@192.168.1.101
ssh-copy-id -i ~/.ssh/loutrack_deploy_key.pub pi@192.168.1.102
```

### 2. Pi側の準備

各Raspberry Piで以下を設定:

```bash
# ディレクトリ作成
sudo mkdir -p /opt/loutrack/releases
sudo chown pi:pi /opt/loutrack

# systemdサービス作成 (例)
sudo cat > /etc/systemd/system/loutrack.service <<EOF
[Unit]
Description=loutrack camera capture
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/opt/loutrack/current
ExecStart=/usr/bin/python3 capture.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable loutrack.service
```

### 3. sudo権限の設定

```bash
# piユーザーがパスワードなしでサービスを再起動できるように
sudo echo "pi ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart loutrack.service" >> /etc/sudoers.d/loutrack
```

## hosts.ini の書き方

```ini
# hostname    ip_address         camera_id
pi-cam-01     192.168.1.101      1
pi-cam-02     192.168.1.102      2
pi-cam-03     192.168.1.103      3
```

- `#` で始まる行はコメントとして無視
- 各フィールドはスペース/タブ区切り
- `hostname`: 識別用名前 (任意)
- `ip_address`: PiのIPアドレス
- `camera_id`: カメラ識別子 (メッセージで使用)

## デプロイ手順

### 基本的なデプロイ

```bash
# ./pi/ ディレクトリの内容を全Piに配布
./deploy/deploy.sh
```

### オプション

```bash
# ドライラン (実際には変更しない)
./deploy/deploy.sh --dry-run

# 順次実行 (デバッグ用)
./deploy/deploy.sh --no-parallel

# バージョンタグ指定
./deploy/deploy.sh --version v1.2.3
```

### デプロイの動作

1. `hosts.ini` を読み込み
2. 各Piに `/opt/loutrack/releases/<timestamp>/` を作成
3. `rsync --delete` で `./pi/` をミラーリング
4. `current` シンボリックリンクを新リリースに切り替え (atomic)
5. `loutrack.service` を再起動
6. 古いリリースを削除 (直近3世代のみ保持)

## ロールバック手順

```bash
# 前のバージョンに戻す
./deploy/rollback.sh
```

### ロールバックの動作

1. 各Piで現在の `current` リンク先を確認
2. 2番目に新しいリリースを特定
3. `current` リンクを切り替え
4. `loutrack.service` を再起動

## ディレクトリ構造 (Pi側)

```
/opt/loutrack/
├── current -> releases/20260222_120000/  # シンボリックリンク
└── releases/
    ├── 20260222_120000/  # 最新
    ├── 20260222_100000/  # 1つ前
    └── 20260221_180000/  # 2つ前
```

## トラブルシューティング

### SSH接続エラー

```bash
# 接続テスト
ssh -i ~/.ssh/loutrack_deploy_key pi@192.168.1.101

# known_hostsに追加
ssh-keyscan 192.168.1.101 >> ~/.ssh/known_hosts
```

### 権限エラー

```bash
# Pi側で所有権を確認
ls -la /opt/loutrack/

# 必要に応じて修正
sudo chown -R pi:pi /opt/loutrack/
```

### サービスが起動しない

```bash
# ログを確認
sudo journalctl -u loutrack.service -f

# ステータス確認
sudo systemctl status loutrack.service
```

### ロールバックできない

```bash
# リリース一覧を確認
ls -la /opt/loutrack/releases/

# 手動で切り替え
ln -sfn /opt/loutrack/releases/20260221_180000 /opt/loutrack/current
sudo systemctl restart loutrack.service
```

## Ansible移行の判断基準

現在のSCP/SSHベースで問題が発生した場合、Ansible移行を検討:

| 条件 | 推奨 |
|------|------|
| Pi台数 > 4台 | Ansible推奨 |
| 更新頻度 > 週2回 | Ansible推奨 |
| 設定変更が多い | Ansible推奨 |
| 台数 2-4台、更新は週1以下 | SCP/SSH継続 |

### Ansible移行時

```yaml
# playbook.yml (例)
- hosts: pi_cameras
  tasks:
    - name: Sync loutrack code
      synchronize:
        src: ../pi/
        dest: /opt/loutrack/current/
        delete: yes
    - name: Restart service
      systemd:
        name: loutrack.service
        state: restarted
```

## ログ

- `deploy/deploy.log` - デプロイログ
- `deploy/rollback.log` - ロールバックログ

```bash
# ログをリアルタイムで確認
tail -f deploy/deploy.log
```
