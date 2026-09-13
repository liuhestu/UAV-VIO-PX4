#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
cd /home/he/uav_vio_px4/ros2_ws
colcon build --symlink-install
colcon test --packages-select estimator_adapter
colcon test-result --verbose
