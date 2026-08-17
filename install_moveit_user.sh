#!/usr/bin/env bash
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
OVERLAY="$DIR/ros_overlay"
DEBS="$OVERLAY/debs"
mkdir -p "$DEBS"

set +u
source /opt/ros/humble/setup.bash
source /home/unix_ai/work/controller/install/setup.bash
set -u

# 只装本实验会用到的 MoveIt 规划、KDL 和 OMPL 组件。
PACKAGES=(
  libccd2
  libfcl0.7
  liboctomap1.9
  ros-humble-eigen-stl-containers
  ros-humble-geometric-shapes
  ros-humble-moveit-common
  ros-humble-moveit-core
  ros-humble-moveit-kinematics
  ros-humble-moveit-msgs
  ros-humble-moveit-planners-ompl
  ros-humble-moveit-ros-move-group
  ros-humble-moveit-ros-occupancy-map-monitor
  ros-humble-moveit-ros-planning
  ros-humble-moveit-simple-controller-manager
  ros-humble-object-recognition-msgs
  ros-humble-octomap
  ros-humble-octomap-msgs
  ros-humble-random-numbers
  ros-humble-ruckig
  ros-humble-srdfdom
  ros-humble-urdfdom-py
)

echo "[安装] 下载 MoveIt 到用户目录（不使用 sudo）"
cd "$DEBS"
for package in "${PACKAGES[@]}"; do
  if ! compgen -G "${package}_*.deb" >/dev/null; then
    apt-get download "$package"
  fi
done

for deb in ./*.deb; do
  dpkg-deb -x "$deb" "$OVERLAY"
done

source "$DIR/overlay_env.sh"
python3 -c 'from moveit_msgs.srv import GetMotionPlan'
ros2 pkg prefix moveit_ros_move_group >/dev/null

missing="$(ldd "$OVERLAY/opt/ros/humble/lib/moveit_ros_move_group/move_group" | grep 'not found' || true)"
if [[ -n "$missing" ]]; then
  echo "$missing"
  echo "MoveIt 还有动态库缺失，请看上面的名字。" >&2
  exit 1
fi
echo "[安装] MoveIt 用户目录安装完成"
