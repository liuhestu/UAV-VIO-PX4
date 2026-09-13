#!/usr/bin/env bash
set -euo pipefail

readonly FCU_DEVICE=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
source /opt/ros/humble/setup.bash

ls -l "${FCU_DEVICE}"
ros2 topic echo /mavros/state --once
ros2 param get /mavros/param EKF2_EV_CTRL
ros2 topic info -v /mavros/odometry/out

printf '%s\n' '检查标准转换（每条命令会持续输出，请分别用 Ctrl+C 结束）：'
printf '%s\n' 'ros2 run tf2_ros tf2_echo odom_ned odom'
printf '%s\n' 'ros2 run tf2_ros tf2_echo base_link_frd base_link'
printf '%s\n' '只有确认 connected=true、armed=false、EKF2_EV_CTRL=0 后，才可启用测试输出。'
