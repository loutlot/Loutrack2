# Pi Control Transport Spec (MVP)

このドキュメントは、Host(PC) -> Raspberry Pi の制御(TCP) と、Pi -> Host のフレーム送信(UDP) の通信規約を単一の仕様ソースとして固定する。

実装は後続タスクで行う。ここでは「wire format / ports / error codes / MVP cmd の扱い」を明文化する。

## 1. 用語

- Host: 受信・三角測量・可視化を行うPC側
- Pi: Raspberry Pi + Camera 側
- Control: Host -> Pi の制御コマンド
- Frame: Pi -> Host のフレーム(検出blob)メッセージ

## 2. TCP Control (NDJSON)

### 2.1 接続

- Pi は TCP server として待ち受ける
- bind: `0.0.0.0`
- port: `8554`

### 2.2 フレーミング

- フォーマット: `NDJSON`
- 1行 = 1 JSON object
- 行終端: `\n`
- max line: 65536 bytes
- read timeout: 2s/line timeout

注: 既存の JSON Lines パターンは `src/host/logger.py` / `src/host/replay.py` の「1行=1JSON」(jsonl) と同じ思想。

### 2.3 メッセージスキーマ

- `schema/control.json` 準拠
- request と response は同じ TCP stream 上で NDJSON として流れる

Request (Host -> Pi):

```json
{"request_id":"req-001","camera_id":"pi-cam-01","cmd":"set_exposure","params":{"value":10000}}
```

Response (Pi -> Host):

```json
{"request_id":"req-001","camera_id":"pi-cam-01","ack":true}
```

`ack:false` の場合は `error_code` と `error_message` を返す:

```json
{"request_id":"req-002","camera_id":"pi-cam-01","ack":false,"error_code":3,"error_message":"unknown_cmd: mask_start"}
```

## 3. UDP Frame Transport

### 3.1 送信単位

- 1 datagram = 1 JSON object
- `schema/messages.json` 準拠

### 3.2 送信先

- default dest: `255.255.255.255:5000`

備考:
- Host 側の既定受信は `src/host/receiver.py` の UDP port `5000`。

## 4. Error Codes

Control response の `error_code` は以下を固定する (1..7):

- 1: `invalid_json`
- 2: `invalid_request`
- 3: `unknown_cmd`
- 4: `not_running`
- 5: `already_running`
- 6: `backend_unavailable`
- 7: `internal_error`

`ack:true` のときは `error_code` は省略または 0 を許容する。

## 5. Command Support Policy (MVP)

### 5.1 MVP supported commands

この MVP フェーズで「機能としてサポートする」コマンドを以下に固定する:

- `ping`
- `start`
- `stop`
- `set_exposure`
- `set_gain`
- `set_fps`

### 5.2 Schema-valid but not supported (MVP)

`schema/control.json` 的に valid でも、MVP で未サポートのコマンドは以下の挙動を必須とする:

- response: `ack:false`
- `error_code=3` (unknown_cmd)

例: `mask_start` は schema 上 enum に含まれるが、MVP では未サポートのため以下を返す。

```json
{"request_id":"req-003","camera_id":"pi-cam-01","ack":false,"error_code":3,"error_message":"unknown_cmd: mask_start"}
```

## 6. 実装メモ (非規範)

- Host は各 Pi に TCP 接続し、Control request を NDJSON で送る。応答も同一接続で受け取る。
- Frame (UDP) はブロードキャストが基本だが、運用によりユニキャストに切り替えても schema/messages.json 互換である限り wire format は同じ。
