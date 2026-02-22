# SteamVRへ5トラッカーを出す方式 (選定メモ)

`context/request.md` の最終目的は SteamVR に 5トラッカー(頭/胸/腰/右足先/左足先)を入力することです。
この部分は「最後にやる」ではなく、早めに方式を決めて **座標系・レイテンシ設計**を固定しないと戻りが大きくなります。

## 結論(推奨)

SteamVRへ汎用の外部6DoFトラッカーを追加するなら、現実解は **OpenVR Driver** です。

- OpenVR Driver公式ドキュメント: https://github.com/ValveSoftware/openvr/blob/master/docs/Driver_API_Documentation.md

## 選択肢

### Option A: OpenVR Driverを自作する(推奨)

特徴:
- SteamVRに「TrackedDevice」を登録できる(=複数トラッカーに向く)
- 実装の自由度が高い(座標変換、予測、補間、ロスト時の扱いを自前にできる)

実装の参考:
- SlimeVR OpenVR Driver (複数トラッカー実績): https://github.com/SlimeVR/SlimeVR-OpenVR-Driver
- Simple OpenVR Bridge Driver (学習テンプレ): https://github.com/ominitay/Simple-OpenVR-Bridge-Driver

通信設計(最小):
- Host側アプリが 5トラッカー分の pose を 60fpsでドライバへ送る
- ドライバは SteamVR からポーリングされるスレッドで最新 pose を取り出し、`TrackedDevicePoseUpdated` で反映する

レイテンシの目安(参考):
- SlimeVRの測定では 6-12ms 程度が報告されている(実装/環境依存)
- 出典: https://github.com/SlimeVR/SlimeVR-Server/issues/511

注意点:
- ドライバはC++での実装になることが多く、ビルド/配布/署名など運用面の作業が増える
- 5台分の device id / serial / role のマッピングを安定化させる必要がある

### Option B: 既存のSteamVR向けトラッカーエコシステムに“話しかける”

例えば SlimeVR のように、既にSteamVRへトラッカーを出す経路があるプロジェクトに対して、
自分の pose を送って「受け側のドライバ」に登録させるやり方。

メリット:
- ドライバ実装の重さを減らせる可能性

デメリット:
- 既存プロトコル/実装へ強く依存し、望む仕様(座標系、予測、再同定)に合わせづらい

参考:
- SlimeVR OpenVR Driver: https://github.com/SlimeVR/SlimeVR-OpenVR-Driver

### Option C: OpenXR API Layer / OSCブリッジ

非推奨です。

- OpenXRは現状、汎用の外部ボディトラッカーをSteamVRへ追加する標準的な拡張が薄く、実装コストが跳ねがち。
- OSCはアプリ依存(VRChat等)になりやすく、SteamVRの“トラッカー”としては一般解になりにくい。

参考:
- VRChat OSC Trackers (アプリ側機能): https://docs.vrchat.com/docs/osc-trackers

## loutrack2で決めるべき具体項目

- 5トラッカーのIDと役割の固定方法(機体/剛体IDと対応)
- 座標系(右手/左手、単位、原点、床)
- 出力の時間軸(サンプル時刻、予測、補間)
- ロスト/復帰時の扱い(“無効 pose”を出す vs 最終値保持)
