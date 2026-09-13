#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/he/uav_vio_px4/ros2_ws/install/setup.bash
set -u
exec ros2 bag play /home/he/datasets/euroc/V1_01_easy_db \
  --topics /imu0 /cam0/image_raw /cam1/image_raw
