# 引き継ぎ
---

## 0. この資料の読み方

| 節 | 誰向け |
|---|---|
| 12 | 全員。何を作ったか、どう繋がっているか |
| 345 | コードを触る人。公式との差分と、現在の仕様 |

## 1. 全体像

公式サンプルは「レースラインを pure pursuit で追従するだけ」の1ノードです。本コードは他車がいる状況で順位を上げるため、次の3層を持ちます。

1. **追い越し・回避層**(`v2x_overtaker`、自作): V2X の他車位置から、抜ける場所・側・必要な車間を計算し、レースラインを横にずらした軌道と速度上限を作る。
2. **復帰層**(`stuck_recovery_controller`、自作): 壁や他車に詰まって動けないときに前進・後退で脱出する。最終的な制御指令の所有者。
3. **追従層**(`simple_pure_pursuit` = 公式の改造、`reverse_pure_pursuit` = 後退用の自作)

### ペナルティ

ペナルティは時間の加算ではなく **「その秒数だけ 5km/h に固定」**(AWSIM バイナリから実測)。Crash は10.0秒、Wall は5.0秒、Over は2.0秒、Block は20.0秒。壁や車に接触したまま留まるとペナルティは解除されず、5km/h 固定が続く。

### 追い越しレーン(公式ルール、運営 Slack 2026-09-06)

- 路面に水色の長方形で描画される。コード上の区間は **idx234〜21**。
- 車体全体がレーン内かつ 27km/h 以上の車が「アタッカー」。アタッカーがいる間、レーンに触れている 27km/h 以下の車は3秒以内に完全退出しないと違反。
- レーンに入らなければ制約はない。

### 順位ハンデ

速度上限は 1位 25km/h、2位以下 36km/h。**速度で上回れる相手は1位だけ**です。

## 2. ノード構成とデータの流れ

```
simple_trajectory_generator -> v2x_overtaker -> stuck_recovery_controller
                  |                    |
               trajectory_v2x       control_cmd
                        |
          simple_pure_pursuit / reverse_pure_pursuit
```

- `/control/command/control_cmd` に publish するのは `stuck_recovery_controller` だけ。
- **`longitudinal.speed` は AWSIM に使われない**。AWSIM は `acceleration` だけを使い、向きはギアが決める。
- `v2x_overtaker` は、観測、追い越し計画、速度要求の最小値による調停、横オフセット付き軌道の publish を1周期で行う。

## 3. 公式コードからの変更点(現在の仕様)

### 3.1 `simple_pure_pursuit`(公式ノードの改造)

公式実装の欠陥を修正した。加速度を上下ともクランプし、`tf2::getYaw()` を使い、閉ループ末尾では先頭へ回り込んで探索する。

追加機能:

| 機能 | 内容 |
|---|---|
| `lookahead_scale_zones` | 場所指定で lookahead を縮める(既定 `162:168:0.35`) |
| `lookahead_slow_*` / `lookahead_slow_min=2.8` | 低速時だけ lookahead を縮める |
| `max_acceleration=2.0` | AWSIM は ±1.37 に丸める |
| 出力先 | `/control/command/nominal_control_cmd` |

### 3.2 `v2x_overtaker`(新規)

抜く判断、追突防止、停止車回避、追い越しレーン管理、助走、車間調整、壁余裕を担当する。`zone_free` は既定 false。助走加速度は `runup_accel_mps2=3.2`、通常の壁余裕は `wall_margin=0.65m`、直線の追い越し中は `pass_wall_keep=0.25m`。

### 3.3 `stuck_recovery_controller`(新規)

`recovery_simple=true`(既定)では、レースライン上の前6m / 後4mを目標に、舵角候補を評価して前進・後退を切り替える。候補が全滅した場合は短距離探索を行い、壁に食い込んだまま改善しない前進は `wall_forward_ban` で禁止する。上限は30秒、最終手段 `desperate_enable` は既定 true。

### 3.4 `reverse_pure_pursuit`(新規)

後退区間を経路追従で走らせる。前軸基準で後方の目標点を追い、舵の符号を反転する。

### 3.5 起動設定・データ

- `reference.launch.xml` / `control/pure_pursuit.launch.xml`: 環境変数で既定値を切り替える。
- `docker-compose.yml`: 環境変数の既定値。
- `data/ten_map/raceline_ten_v2.csv`: 走行ラインと地点別の目標速度。
- `data/ten_map/corridor_ten.csv`: 中心線に対して横に使える範囲。
- 占有格子 `ten_occupancy_grid_map`: 壁の判定。

## 4. 実測で確定している車両の値

| 項目 | 値 | 備考 |
|---|---|---|
| ホイールベース(幾何) | 1.087 m | 公式。車体の形・当たり判定はこちら |
| ホイールベース(指令の式) | 2.14 m | `steering_tire_angle_gain 2.8` と一体で指令の単位変換を吸収 |
| 車体 前端 / 後端 / 半幅 | 1.554 / 0.510 / 0.725 m | 原点は後軸 |
| 最大舵角(公式) | 0.64 rad | |
| 最大実舵角(実測) | 0.31 rad (18度) | 指令を大きくしても飽和 |
| 最大速度 | 37.3 km/h | |
| 最大横加速度 | 23.2 m/s² | |
| 加速度指令の実効上限 | 1.37 | 公式表記は1.5 |
| 駆動時最大加速度(公式) | 3.2 m/s² | `docs/specifications/simulator.ja.md` |
| ブースト | +0.5 m/s²、10秒 | 同上 |

## 5. 効いていない設定の一覧

| 設定 | 実態 |
|---|---|
| `size_pad`(C++ 既定 0.25) | 実行時は 0.12(`race.sh`・compose・launch が上書き) |
| シェルで `RUNUP_ACCEL_MPS2` を渡す | `race.sh` が車ごとに上書きする。変更時は `RAM="1.6,1.6,1.6,1.6"` を使う |
| `runup_hold_min_room_m` / `runup_hold_ratio` | 宣言のみ。どこからも読まれない |
| `simple_dir_stable_n` | 宣言のみ。どこからも読まれない |
| `ot_lane_runup_look` のコメント「既定60m」 | 実際の値は45m |
| `zone_free` | 既定 false |

A/B で新しいフラグを足すときは、`docker-compose.yml` の `environment:`、`race.sh`、`race_repeat.sh` の3か所を通し、`ros2 param get` で値と型を確認する。

## 6. 提出手順(公式)

```bash
cd ~/aichallenge-racingkart
docker compose run --rm autoware-build
make eval
./create_submit_file.bash
make down
```
