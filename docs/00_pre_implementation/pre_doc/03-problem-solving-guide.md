# 課題解決の指針 (デバッグ/検証の進め方)

`context/request.md` の課題(同期・較正・遮蔽耐性・運用)は、実装してから一気に解けるタイプではなく、**観測→仮説→改善**を回す設計が必要です。

## 1. まず「観測できる」ようにする

- **ログ**: Pi→Hostの受信パケット(時刻/内容)を丸ごと保存
- **リプレイ**: 同じログを再生して、アルゴリズム変更の効果を比較
- **メトリクス**:
  - fps(受信/処理/出力)
  - エンドツーエンド遅延(可能なら)
  - 観測数(点が何台のカメラに見えているか)
  - 再投影誤差(対応付け/三角測量の健全性)

先駆者の例(メトリクスの出し方):
- fps emit: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`
- 画面側の操作/イベント: `references/jyjblrd/Low-Cost-Mocap/computer_code/src/App.tsx`

## 2. 座標系を最初に固定する

SteamVR連携まで見据えると、座標系が曖昧だと必ず破綻します。

- 各カメラ座標系
- World座標系(床/原点)
- SteamVR座標系

先駆者の例(床合わせ/原点):
- `acquire-floor`, `set-origin`: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`
- UIからのトリガ: `references/jyjblrd/Low-Cost-Mocap/computer_code/src/App.tsx`

## 3. 同期は「完全同期」を狙わない

現実的には、以下の3層で誤差を吸収します。

1. **クロック同期**: PTP/NTPで誤差を縮める
2. **ホスト側ペアリング**: timestamp窓で左右/複数カメラを束ねる
3. **フィルタ**: 欠損やジッタをKalmanで吸収

重要: 「同期の実装が進んでいないから三角測量は後回し」ではなく、
まずソフト側で動かして **許容窓がどれくらい必要か** を測り、PTP導入の価値を定量化する。

## 4. 遮蔽耐性は “トラック管理” の問題

`context/request.md` は点ごとのKalmanを想定していますが、実運用は以下がセットです。

- 予測: 次フレームに現れそうな位置(ゲーティング)
- 関連付け: 観測点をどのトラックに割り当てるか
- 再同定: 一時消失後の復帰
- 外れ値除去: 反射/ノイズ

先駆者の例(エピポーラ + 再投影誤差で外れを抑える):
- `find_point_correspondance_and_object_points`: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`

## 5. 失敗を切り分ける優先順位

デバッグの優先は以下です(上ほど先)。

1. データ契約の破綻(型/単位/座標)
2. 時刻の破綻(ペアリングが崩れている)
3. 較正の破綻(歪み/外部姿勢が狂っている)
4. 対応付けの破綻(点の組み合わせが間違う)
5. フィルタ/クラスタリング/剛体推定

この順にしないと「フィルタでごまかして悪化」が起きます。
