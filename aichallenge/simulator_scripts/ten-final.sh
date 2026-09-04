#!/bin/bash
# SIM決勝(9/19)と同じ条件でローカル再現するための起動スクリプト。
#
# 一次情報: 本体 dev ブランチの s2r-final.sh(運営が Slack で「決勝の起動参考コマンド」
# として提示したもの)。台数・周回・timeout・各フラグはそれに一致させてある。
#   https://github.com/AutomotiveAIChallenge/aichallenge-racingkart/blob/dev/aichallenge/simulator_scripts/s2r-final.sh
#
# 予選(ten-qual.sh)との差:
#   --vehicles      3 -> 4
#   --timeout   600.0 -> 420.0
#   --overtaking-lane  (新規) on   ... 追い越しレーン。BLOCK ペナルティ 20秒
#   --start-random     (新規) off  ... グリッドは予選順位の順で固定
#   --start-count-seconds 5 -> 10
#
# s2r-final.sh からの差は「検証のための都合」だけに限る:
#   --sound off / ウィンドウ表示 / ROS2_BASE_DOMAIN / AWSIM_HEADLESS
#
# 使い方: make simulator-ten-final
#         ~/ten_aichallenge/local_script/race.sh ten-final 4

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
export ROS_DOMAIN_ID=${AWSIM_ADMIN_DOMAIN:-0}

EXTRA_ARGS=()
if [ -n "${ROS2_BASE_DOMAIN:-}" ]; then
    EXTRA_ARGS+=(--ros2-base-domain "$ROS2_BASE_DOMAIN")
fi
if [ "${AWSIM_HEADLESS:-0}" = "1" ]; then
    EXTRA_ARGS+=(-batchmode -nographics)
fi

# 第1引数で台数(既定 4 = 決勝と同じ)。第2引数で NPC 台数(既定 0)。
vehicles="${1:-4}"
npcs="${2:-${NPCS:-0}}"

exec $AWSIM_DIRECTORY/AWSIM.x86_64 \
    --venue citycircuit \
    --start-mode sync \
    --start-count-seconds 10 \
    --vehicles "${vehicles}" \
    --npcs "${npcs}" \
    --boosts 2 \
    --laps 6 \
    --timeout 420.0 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap on \
    --wall-recovery off \
    --overtaking-lane on \
    --start-random off \
    --ranking on \
    --camera off \
    --lidar off \
    ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} \
    -screen-fullscreen 0 \
    -screen-width 1280 \
    -screen-height 720 \
    -screen-quality low
