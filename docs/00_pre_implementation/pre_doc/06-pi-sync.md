# 複数Raspberry Piカメラの同期・露光制御 指針

`context/request.md` の課題の中でも「複数Piでのフレーム同期」は最難所です。
最初に “到達可能な精度” と “実装コスト” を把握し、どこまでを現実目標にするか決めます。

## 到達できる精度の目安(参考)

ネットワーク+ソフト同期でも、PTP構成で 100us 程度の同期が報告されています(条件依存)。

参考:
- PTP(Grandmaster/Client)導入の手順例: https://austinsnerdythings.com/2025/02/18/nanosecond-accurate-ptp-server-grandmaster-and-client-tutorial-for-raspberry-pi/
- PTPハードウェアタイムスタンプ(Raspberry Pi CM4): https://www.jeffgeerling.com/blog/2022/ptp-and-ieee-1588-hardware-timestamping-on-raspberry-pi-cm4/
- 複数カメラ同期の実験記事: https://medium.com/inatech/synchronize-multiple-cameras-to-capture-at-the-same-time-c285b520bd87

※ 上記は「理想的なクロック同期」の話で、実際の撮影タイミングは OSスケジューリングやローリングシャッター等で揺れます。

## 推奨アプローチ(段階的)

### Phase A: PTPで“時計”を揃える + timestampでホストがペアリング

1. Pi群を同一L2ネットワーク(できれば有線)に置く
2. 1台を Grandmaster として PTP を動かす
3. 各Piで system clock を PTP に追従させる
4. 各フレームに timestamp を付けて送信し、Hostが許容窓でペアリングする

実装のポイント:
- `context/request.md` の `timestamp` を「PTPで同期された wall clock 由来」にする
- Host側は “完全一致” を狙わず、許容窓(例: 数ms)で束ねる

### Phase B: 露光・ゲインの統一(ホスト一元管理)

`context/request.md` の方針通り、AE/AFなど自動系を切って固定値に寄せる。

- AEを無効にし、露光時間/アナログゲインを同一にする
- fps/フレーム期間も固定する

参考(概念/実装のヒント):
- Raspberry Piフォーラムの議論(ソフト同期/露光統一): https://forums.raspberrypi.com/viewtopic.php?t=332482

### Phase C: libcameraの同期機構(SyncMode)を検討

libcameraにはカメラ同期用のコントロールが提案・追加されています。

- libcamera-devel (2025-01) SyncModeのパッチ: https://lists.libcamera.org/pipermail/libcamera-devel/2025-January/048034.html

これは将来的に“同一Pi内/同一基盤”での同期改善に繋がる可能性がありますが、
複数Pi間の同期は結局ネットワーク時刻や外部トリガと組み合わせが必要になります。

### Phase D: 外部ハードトリガ(必要なら)

サブmsレベルを狙う、またはローリングシャッターの影響を抑えたい場合は外部トリガが最も強い。

- Arducam External Trigger Mode: https://docs.arducam.com/Raspberry-Pi-Camera/Multi-Camera-CamArray/external-trigger/

## loutrack2で決めるべき具体項目

- PTPを導入するか(ネットワーク/運用コスト vs 効果)
- timestampの定義(単位、基準、送信タイミング)
- Host側ペアリングの許容窓と欠損時の扱い
- 露光/ゲイン/フレーム期間の制御方針(AE/AFの扱い)
