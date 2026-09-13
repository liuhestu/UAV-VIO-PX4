#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
exec ros2 bag play /home/he/datasets/euroc/V1_01_easy_db \
  --topics /imu0 /cam0/image_raw /cam1/image_raw
