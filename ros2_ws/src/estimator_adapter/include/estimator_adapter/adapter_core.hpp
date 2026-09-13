#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>

#include <array>
#include <string>
#include <vector>

namespace estimator_adapter
{

struct AdapterConfig
{
  bool replay_mode{true};
  bool send_velocity{false};
  std::string frame_id{"odom"};
  std::string child_frame_id{"base_link"};
  // T_PV maps vector components from the OpenVINS/RealSense IMU frame V to
  // the Pixhawk IMU ROS-FLU frame P. Quaternion storage order is xyzw.
  std::array<double, 3> translation_pv_m{{0.0, 0.0, 0.0}};
  std::array<double, 4> rotation_pv_xyzw{{0.0, 0.0, 0.0, 1.0}};
  double minimum_quaternion_norm{1.0e-6};
};

bool set_and_validate_extrinsics(
  const std::vector<double> & translation_xyz_m,
  const std::vector<double> & rotation_xyzw,
  AdapterConfig * config,
  std::string * error);

bool finite_pose_and_covariance(const nav_msgs::msg::Odometry & input);

bool adapt_odometry(
  const nav_msgs::msg::Odometry & input,
  const rclcpp::Time & now,
  const AdapterConfig & config,
  rclcpp::Time * last_output_stamp,
  nav_msgs::msg::Odometry * output,
  std::string * error);

}  // namespace estimator_adapter
