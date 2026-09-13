#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/he/uav_vio_px4/ros2_ws/install/setup.bash
set -u
exec ros2 launch estimator_adapter phase2.launch.py \
  output_enabled:=false replay_mode:=true
