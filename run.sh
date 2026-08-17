#!/usr/bin/env bash
set -euo pipefail
ulimit -c 0

DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$DIR"

if [[ " $* " == *" --fake "* ]]; then
  exec python3 "$DIR/main.py" "$@"
fi

set +u
source /opt/ros/humble/setup.bash
source /home/unix_ai/work/controller/install/setup.bash
set -u
export ROS_DOMAIN_ID=69
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/unix_ai/config/cyclonedds.xml
export PYTHONUNBUFFERED=1

if [[ ! -x "$DIR/ros_overlay/opt/ros/humble/lib/moveit_ros_move_group/move_group" ]]; then
  "$DIR/install_moveit_user.sh"
fi
source "$DIR/overlay_env.sh"

# 纯检查不需要释放底盘；真实任务和运动检查需要。
if [[ " $* " != *" --check "* ]] && \
   [[ " $* " != *" --command-check "* ]] && \
   [[ " $* " != *" --arm-motion-check "* ]] && \
   [[ " $* " != *" --pick-motion-check "* ]]; then
  "$DIR/release_base.sh"
fi

python3 "$DIR/prepare_moveit.py"
mkdir -p "$DIR/logs"

"$FRUITTEST_ROS/lib/moveit_ros_move_group/move_group" \
  --ros-args \
  -r __node:=fruit_move_group \
  -r /joint_states:=/fruit_test/joint_states \
  --params-file "$DIR/generated/moveit_params.yaml" \
  >"$DIR/logs/moveit.log" 2>&1 &
MOVEIT_PID=$!

stop_moveit() {
  # 这个 Humble 版本在 TERM 清理时会崩溃；它只是本脚本启动的规划子进程，
  # 直接结束最干净，也不会留下 core 文件或让 Ctrl+C 卡住。
  kill -KILL "$MOVEIT_PID" 2>/dev/null || true
  wait "$MOVEIT_PID" 2>/dev/null || true
}
trap stop_moveit EXIT INT TERM

python3 "$DIR/main.py" "$@"
