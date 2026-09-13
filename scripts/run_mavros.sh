#!/usr/bin/env bash
set -euo pipefail

readonly FCU_DEVICE=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
test -e "${FCU_DEVICE}"
export GEOGRAPHICLIB_DATA=/home/he/uav_vio_px4/.geographiclib
source /opt/ros/humble/setup.bash
exec ros2 launch mavros px4.launch "fcu_url:=${FCU_DEVICE}:57600"
