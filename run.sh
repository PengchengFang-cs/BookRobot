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

if [[ " $* " == *" --book-align "* || \
      " $* " == *" --book-pick "* || \
      " $* " == *" --book-place "* || \
      " $* " == *" --book-pick-place "* || \
      " $* " == *" --stage1-loop "* || \
      " $* " == *" --cart-perception "* || \
      " $* " == *" --cart-scan-navigation "* || \
      " $* " == *" --cart-approach-navigation "* ]]; then
  source "$DIR/scripts/book_vision_env.sh"
  RUN_ID="${FPC_EXPERIMENT_RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
  export FPC_EXPERIMENT_RECORD_DIR="$DIR/logs/record_rebot/$RUN_ID"
  mkdir -p "$FPC_EXPERIMENT_RECORD_DIR"
  printf 'run_id=%s\nstarted_at=%s\ncommand=%q' \
    "$RUN_ID" "$(date --iso-8601=seconds)" "$DIR/main.py" \
    >"$FPC_EXPERIMENT_RECORD_DIR/run.txt"
  printf ' %q' "$@" >>"$FPC_EXPERIMENT_RECORD_DIR/run.txt"
  printf '\n' >>"$FPC_EXPERIMENT_RECORD_DIR/run.txt"
  set +e
  python3 "$DIR/main.py" "$@" 2>&1 | \
    tee "$FPC_EXPERIMENT_RECORD_DIR/experiment.log"
  STATUS=${PIPESTATUS[0]}
  set -e
  printf 'finished_at=%s\nexit_code=%s\n' \
    "$(date --iso-8601=seconds)" "$STATUS" \
    >>"$FPC_EXPERIMENT_RECORD_DIR/run.txt"
  exit "$STATUS"
fi

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
