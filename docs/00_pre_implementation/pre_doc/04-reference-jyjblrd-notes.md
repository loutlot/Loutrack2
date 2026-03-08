# 参考: jyjblrd / Low-Cost-Mocap から学べること

`references/jyjblrd/Low-Cost-Mocap` は PS3 Eye を使った outside-in のモーキャプ実装です。
loutrack2 は Raspberry Pi + Pi camera で分散構成にする想定ですが、**アルゴリズムとデータ設計**の部分は強く参考になります。

外部リンク:
- GitHub: https://github.com/jyjblrd/Low-Cost-Mocap
- YouTube: https://youtu.be/0ql20JKrscQ
- 作者ブログ: https://joshuabird.com/blog/post/mocap-drones

## 1. 使える実装パーツ(アルゴリズム)

### 三角測量(DLT)

- 実装: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`
- 関数: `triangulate_point`, `triangulate_points`

### 対応付け(エピポーラ線 + 再投影誤差)

- 実装: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`
- 関数: `find_point_correspondance_and_object_points`

### バンドル調整(再投影誤差最小化)

- 実装: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/helpers.py`
- 関数: `bundle_adjustment`, `calculate_reprojection_errors`

### 床合わせ/原点

- 実装: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`
- イベント: `acquire-floor`, `set-origin`

## 2. 使える設計パーツ(プロトコル/UI)

### Host側イベント駆動(制御/状態)

- Host側(Socket.IO): `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`
- UI側: `references/jyjblrd/Low-Cost-Mocap/computer_code/src/App.tsx`

イベント例:
- `update-camera-settings`
- `capture-points`
- `calculate-camera-pose`
- `triangulate-points`
- `locate-objects`

loutrack2 でも、Pi→HostはUDPでも良いですが、**Host↔UI/制御**は同様のイベント駆動にすると「実験→改善」の速度が上がります。

## 3. 注意点(そのまま持ち込まない)

- OpenCV SFM依存(ビルドが重い)があり得る点: `references/jyjblrd/Low-Cost-Mocap/README.md`
- ハードウェア依存(PS3 Eye, ESP32, serial port)が前提の箇所: `references/jyjblrd/Low-Cost-Mocap/computer_code/api/index.py`
- `camera-params.json` は「形式」は使えるが、loutrack2 のカメラ( Pi camera 3 Noir Wide )に合わせた較正が別途必要
