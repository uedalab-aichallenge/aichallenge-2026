#!/bin/bash

AWSIM_DIRECTORY=/aichallenge/simulator/AWSIM
# ten-qual.sh と同じスロット対応。既定(0)では追加引数を一切渡さないので、
# 通常の make eval の argv は従来と完全に同じになる。
# ROS2_BASE_DOMAIN は「車両1のドメイン」。既定(未設定)では AWSIM の既定値 1 が使われ、
# 車両が 1,2,3、管理トピックが 0 に載る。未設定なら追加引数を一切出さないので、
# 通常の実験の argv は従来と完全に同じになる。
export ROS_DOMAIN_ID=${AWSIM_ADMIN_DOMAIN:-0}

EXTRA_ARGS=()
if [ -n "${ROS2_BASE_DOMAIN:-}" ]; then
    EXTRA_ARGS+=(--ros2-base-domain "$ROS2_BASE_DOMAIN")
fi
if [ "${AWSIM_HEADLESS:-0}" = "1" ]; then
    EXTRA_ARGS+=(-batchmode -nographics)
fi

exec "$AWSIM_DIRECTORY/AWSIM.x86_64" \
    ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} \
    --venue citycircuit \
    --start-mode sync \
    --start-count-seconds 5 \
    --vehicles 1 \
    --npcs 0 \
    --boosts 2 \
    --laps 6 \
    --timeout 600 \
    --steer-source ackermann \
    --sound off \
    --collisions on \
    --handicap off \
    --wall-recovery off \
    --ranking off \
    --camera off \
    --lidar off

# Cameraを使う場合 : --camera cpu or gpu
# LiDARを使う場合 : --lidar cpu or gpu
# GPUがない場合 -headlessを末尾に追加
