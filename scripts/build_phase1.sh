#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
cd /home/he/uav_vio_px4/ros2_ws
colcon build --symlink-install
colcon test --packages-select estimator_adapter
colcon test-result --verbose
