#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u
exec ros2 launch ov_msckf subscribe.launch.py \
  config:=euroc_mav namespace:=ov_msckf rviz_enable:=false verbosity:=WARNING
