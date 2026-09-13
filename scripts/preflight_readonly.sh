#!/usr/bin/env bash
set -eo pipefail

readonly FCU_DEVICE=/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
source /opt/ros/humble/setup.bash
set -u

ls -l "${FCU_DEVICE}"
ros2 topic echo /mavros/state --once
for parameter in \
  EKF2_EV_CTRL EKF2_HGT_REF EKF2_EV_NOISE_MD EKF2_EVP_GATE EKF2_EVP_NOISE \
  EKF2_EV_POS_X EKF2_EV_POS_Y EKF2_EV_POS_Z
do
  ros2 param get /mavros/param "${parameter}"
done
ros2 topic info -v /mavros/odometry/out

printf '%s\n' '检查标准转换（每条命令会持续输出，请分别用 Ctrl+C 结束）：'
printf '%s\n' 'ros2 run tf2_ros tf2_echo odom_ned odom'
printf '%s\n' 'ros2 run tf2_ros tf2_echo base_link_frd base_link'
printf '%s\n' '当前 Adapter 外参是未标定单位 SE(3)，只允许裸板 DISARM 测试，禁止飞行。'
printf '%s\n' '只有确认 connected=true、armed=false、EKF2_EV_CTRL=0 且 EV_POS_X/Y/Z=0 后，才可启用测试输出。'
printf '%s\n' 'Phase3D 临时 Gate 只能精确设为 3（XY+Z）；velocity/yaw 位必须保持关闭。'
