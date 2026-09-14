# race_parameter_tui

`simple_pure_pursuit` / `reverse_pure_pursuit` の動的パラメータを、走行中に
端末から安全に調整するための小さな curses TUI です。ノード名は固定の
`/simple_pure_pursuit` ではなく、ROS graph から検出するため、車両 namespace
（例 `/vehicle_1/simple_pure_pursuit`）でも使えます。

## ビルドと起動

```bash
cd aichallenge/workspace
source /opt/ros/humble/setup.bash
colcon build --packages-select simple_pure_pursuit reverse_pure_pursuit race_parameter_tui
source install/setup.bash
ros2 run race_parameter_tui parameter_tui
```

pure pursuit ノードを launch などで起動した状態で TUI を別端末から起動します。
TUI は ROS graph 上のノードを自動検出するので、`/vehicle_1/simple_pure_pursuit`
のような namespace 付きノードも同じ手順で操作できます。

## 操作

- `↑` / `↓` または `j` / `k`: パラメータを選択
- `Tab` または `PageDown` (`]`): 次のノード、`PageUp` (`[`): 前のノード
- `Enter`: 数値/文字列を入力。bool は Enter で true/false を切り替え
- `r`: ROS graph と値を再読み込み
- `q` または `Esc`: 終了

更新要求は `AsyncParameterClient` で対象ノードへ送られ、成功/拒否理由を
画面下部へ表示します。ROS spin は別スレッドなので、UI操作中もサービス応答を
処理できます。項目数が端末の高さを超える場合は、`↑`/`↓` で選択項目が見える
ように自動スクロールし、画面2行目に現在の表示範囲が表示されます。

## 実際の変更例

1. TUI を起動し、`Tab` で対象の `simple_pure_pursuit`（namespace 付きならその
   ノード）を選びます。
2. `↑`/`↓` または `j`/`k` で `lookahead_gain` まで移動し、`Enter` を押します。
   入力欄へ `0.75` と入力して `Enter` を押すと、その場で更新要求が送られます。
3. 画面下部に `lookahead_gain: updated` と出ればノードが受理し、次の制御周期から
   新しい値が使われます。`rejected (...)` はノード側の検証で拒否された値と理由、
   `set failed (...)` はサービス通信などの失敗です。例えば負の `wheel_base` や
   非有限の gain は `rejected` になります。
4. `lookahead_scale_zones` を選び、`1:4:0.5,20:24:0.8` のように入力します。
   これは区間 `1`--`4` を倍率 `0.5`、区間 `20`--`24` を倍率 `0.8` として、形式を
   検証したうえで一括反映します。形式が壊れている場合は拒否され、既存値は変わり
   ません。
5. bool 項目は入力欄を使わず、`Enter` を押すたびに `true`/`false` が切り替わります。

これらの変更は実行中プロセスだけの一時変更です。ノードを再起動すると launch の
既定値へ戻るため、恒久的に使う値は launch やパラメータ YAML 側も更新してください。

## 動的反映対象と制約

pure pursuit は次の値を検証後に一括反映します: wheel base、lookahead 系、速度/加速度
gain、追い越し倍率、低速操舵制限、simple の飽和/壁ガード、reverse の曲率/低速制御と
`reverse_max_speed`。`lookahead_scale_zones` は `from:to:scale[,from:to:scale]`
形式（非負整数区間、有限の正の倍率）だけ受け付けます。距離・gain・速度・加速度・
倍率・stale time の不正な値、非有限値、型不一致は対象ノード側で拒否されます。

TUI は候補として `/simple_pure_pursuit`、`/reverse_pure_pursuit`、`/v2x_overtaker`、
`/stuck_recovery_controller` を探索します。表示対象はこのパッケージの allowlist と
実際に宣言されたパラメータの共通部分だけです。ノード側に動的 callback がない値は、
サービスが成功しても既存コードの保持値へ反映されない場合があるため、純粋な動的反映を
保証するのは pure pursuit の allowlist です。
