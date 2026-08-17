#!/usr/bin/env bash

# MoveIt 被解压在用户目录，不需要 sudo。
FRUITTEST_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FRUITTEST_OVERLAY="$FRUITTEST_DIR/ros_overlay"
FRUITTEST_ROS="$FRUITTEST_OVERLAY/opt/ros/humble"

export AMENT_PREFIX_PATH="$FRUITTEST_ROS${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export CMAKE_PREFIX_PATH="$FRUITTEST_ROS${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$FRUITTEST_ROS/lib:$FRUITTEST_ROS/lib/aarch64-linux-gnu:$FRUITTEST_OVERLAY/usr/lib/aarch64-linux-gnu:$FRUITTEST_OVERLAY/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export PYTHONPATH="$FRUITTEST_ROS/local/lib/python3.10/dist-packages:$FRUITTEST_ROS/lib/python3.10/site-packages:$FRUITTEST_ROS/lib/python3.10/dist-packages${PYTHONPATH:+:$PYTHONPATH}"
export PATH="$FRUITTEST_ROS/bin${PATH:+:$PATH}"
