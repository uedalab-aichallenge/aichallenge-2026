#!/bin/bash
# オンライン予選と同じ条件でローカル再現するための起動スクリプト。
#
# 公式アナウンス:
#   「自分のコード・近いランク帯の対戦相手・運営NPC の 3台同時レース を行い、
#     完走順にレートが増減します」
#
# したがって台数は 3。周回数・制限時間・handicap/ranking は parallel.sh(運営提供の
# 複数台設定)に合わせ、衝突判定だけは有効にする(parallel.sh は --collisions off で
# 車同士がすり抜けるため、回避の検証ができない)。
#
# 使い方: make simulator-ten-qual   または  ~/ten_aichallenge/local_script/race.sh ten-qual 3
#
# 第1引数で台数を変更可(既定 3)。
#
# ウィンドウ表示にする(-screen-fullscreen 0)。フルスクリーンは操作しづらいため。
# 補足: awsim.log は正常起動時も "Unable to load player prefs" で終わる(全34行)。
# ログでは起動の成否を判定できないので、/awsim/state が publish されているかで見る
# (race.sh の awsim_alive がやる)。

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
# 複数のシミュレータを同時に走らせるとき、スロットごとに ROS ドメインを丸ごとずらす。
# AWSIM の --ros2-base-domain N は「管理トピックを N、車両kを N+k」に載せる。
# 全コンテナが network_mode: host なので、分離はドメインIDだけが担う。
# DDS はドメインIDでポートを分けるため、ドメインが違えば探索も通信も交わらない。
#
# 既定(0)のときは追加引数を一切渡さない。こうすると従来の起動と argv が完全に同じになり、
# 通常の実験の条件は1バイトも変わらない。
# ROS2_BASE_DOMAIN は「車両1のドメイン」。既定(未設定)では AWSIM の既定値 1 が使われ、
# 車両が 1,2,3、管理トピックが 0 に載る。未設定なら追加引数を一切出さないので、
# 通常の実験の argv は従来と完全に同じになる。
export ROS_DOMAIN_ID=${AWSIM_ADMIN_DOMAIN:-0}

EXTRA_ARGS=()
if [ -n "${ROS2_BASE_DOMAIN:-}" ]; then
    EXTRA_ARGS+=(--ros2-base-domain "$ROS2_BASE_DOMAIN")
fi
# 2枚目以降は描画を止めて GPU 負荷を下げる。
# --camera off --lidar off なのでセンサ生成に GPU は要らない。
if [ "${AWSIM_HEADLESS:-0}" = "1" ]; then
    EXTRA_ARGS+=(-batchmode -nographics)
fi

vehicles="${1:-3}"
# 第2引数で NPC 台数。運営NPCを再現するために使う(既定 0)。
# 予選は「自分 + 対戦相手 + 運営NPC」の3台。
# ただしローカルの NPC は挙動が本番と異なるため使わず、
# 3台すべて Autoware(自分1台 + デフォルトMPC2台)で検証する。
npcs="${2:-${NPCS:-0}}"

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --camera off \
    --lidar off \
    --start-mode sync \
    --start-count-seconds 5 \
    --vehicles "${vehicles}" \
    --npcs "${npcs}" \
    --boosts 2 \
    --laps 6 \
    --timeout 600.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap on \
    --wall-recovery off \
    --ranking on \
    ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} \
    -screen-fullscreen 0 \

    -screen-width 1280 \
    -screen-height 720 \
    -screen-quality low
