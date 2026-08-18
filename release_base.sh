#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
NAME=fruit_movebase_mode_controller

# 这台底盘上电后需要一次“释放 -> 左右轮模式”。
ros2 param set /controller_manager "$NAME.type" \
  forward_command_controller/ForwardCommandController >/dev/null
ros2 param set /controller_manager "$NAME.params_file" \
  "$DIR/movebase_mode.yaml" >/dev/null

controllers=$(ros2 control list_controllers)
controller_line=$(grep "$NAME" <<<"$controllers" || true)
if [[ -z "$controller_line" ]]; then
  ros2 control load_controller "$NAME" --set-state active >/dev/null
elif [[ "$controller_line" == *inactive* ]]; then
  ros2 control set_controller_state "$NAME" active >/dev/null
fi

for mode in 5 0; do
  ros2 topic pub --once "/$NAME/commands" std_msgs/msg/Float64MultiArray \
    "{data: [$mode.0]}" >/dev/null
  sleep 0.3
done

echo "[底盘] 已发送软件释放；现场还要确认机身释放键或遥控器已经使能"
