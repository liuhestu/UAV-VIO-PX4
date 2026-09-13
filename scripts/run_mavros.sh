#!/usr/bin/env bash
set -eo pipefail

readonly FCU_DEVICE=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
test -e "${FCU_DEVICE}"
export GEOGRAPHICLIB_DATA="${WORKSPACE_ROOT}/.geographiclib"
source /opt/ros/humble/setup.bash
set -u
exec ros2 launch mavros px4.launch "fcu_url:=${FCU_DEVICE}:57600"
