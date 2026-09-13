#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/he/uav_vio_px4/ros2_ws/install/setup.bash
set -u
exec ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav namespace:=ov_msckf rviz_enable:=false verbosity:=WARNING
