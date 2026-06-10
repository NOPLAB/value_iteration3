#!/usr/bin/env bash
# 本家 value_iteration の vi_benchmark_node を津田沼 0.15m プールマップで走らせ、
# 並列スイープ (thread_num) の VI 時間を /results/tsudanuma_ros1.csv に出力する。
#
# mounts (docker run):
#   /src_value_iteration  : 本家 repo (ro)
#   /tmp/maps_processed   : pool_tsudanuma.py が出した 0.15m PGM+YAML
#   /results              : 出力 CSV
# arg1: build|run|all  (既定 all)。build は catkin_make のみ (重い VI は走らせない)。
set -e
MODE="${1:-all}"
THREAD_NUM="${THREAD_NUM:-16}"
GOAL_X="${GOAL_X:--0.475}"
GOAL_Y="${GOAL_Y:--0.325}"
GOAL_T="${GOAL_T:-0}"
DELTA="${DELTA:-0.1}"

source /opt/ros/noetic/setup.bash
mkdir -p /catkin_ws/src
ln -sfn /src_value_iteration /catkin_ws/src/value_iteration
cd /catkin_ws
catkin_make
source devel/setup.bash

if [ "$MODE" = "build" ]; then
  echo "[run_ros1_tsudanuma] catkin build done (mode=build); skipping VI run."
  exit 0
fi

roslaunch value_iteration benchmark_tsudanuma.launch \
  map_file:=/tmp/maps_processed/map_tsudanuma_015.yaml \
  thread_num:=${THREAD_NUM} \
  goal_x:=${GOAL_X} goal_y:=${GOAL_Y} goal_t:=${GOAL_T} \
  delta_threshold:=${DELTA} \
  output_csv:=/results/tsudanuma_ros1.csv \
  label:=tsudanuma_015_theta60_t${THREAD_NUM}
