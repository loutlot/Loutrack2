# LED board BOM

KiCad 回路図 `hardware/LED board/kicad/LEDHat1.4_low_heat/LEDHat.kicad_sch` と添付画像を基準に、実装点数ベースで整理。

## 実装部品

| 回路図 | 部品 | 値 / 型番 | 実装数 | 現行購入候補 | URL | 購入単位 | 推奨購入数 | メモ |
|---|---|---|---:|---|---|---|---:|---|
| J1 | ピンソケット (メス) | 2x20 (40P) | 1 | 100085 | https://akizukidenshi.com/catalog/g/g100085/ | 1個 | 1 | Raspberry Pi GPIO 用 |
| D1-D24 | 赤外線LED | 940nm OSI5LAS1C1A | 24 | 106427 | https://akizukidenshi.com/catalog/g/g106427/ | 10個/パック | 3パック | 24個使用 |
| Q1 | Nch MOSFET | IRLML6344 | 1 | 106049 | https://akizukidenshi.com/catalog/g/g106049/ | 10個/パック | 1パック | メイン LED 群スイッチ |
| Q_R, Q_G, Q_B | Nch MOSFET | 2N7002E | 3 | 117477 | https://akizukidenshi.com/catalog/g/g117477/ | 5個/パック | 1パック | RGB LED 各色スイッチ |
| D31 | RGBフルカラーLED | 5mm 4本足 | 1 | 111852 | https://akizukidenshi.com/catalog/g/g111852/ | 1個 | 1 | 状態表示用 |
| R1-R8 | カーボン抵抗 | 1/4W 12Ω | 8 | 114271 | https://akizukidenshi.com/catalog/g/g114271/ | 100本/袋 | 1袋 | IR LED 4列分の直列抵抗 |
| R0 | カーボン抵抗 | 1/4W 470Ω | 1 | 125471 | https://akizukidenshi.com/catalog/g/g125471/ | 100本/袋 | 1袋 | RGB LED 用の直列抵抗 |
| J2 | 電源コネクタ | 1x2 Male | 1 | 112986 | https://akizukidenshi.com/catalog/g/g112986/ | 1個 | 1 | `1x10 L型ピンヘッダ` を 2 ピン分に切って使用 |

## 回路図から読める数量メモ

- 赤外線 LED は 6個 x 4列 = 24個。
- 12Ω 抵抗は 2本 x 4列 = 8本。
- RGB LED は 1個、制御 MOSFET は 3個。
- 電源側 MOSFET `Q1` は 1個。

## 補足

- `J2` は `1x10 L型ピンヘッダ` を 2 ピン分に切って使用する前提。
- 取り付け部材、配線材、スペーサー、ヒートシンク等の機構部品は回路図からは判断できないため未記載。
