#include "estimator_adapter/adapter_core.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace estimator_adapter
{
namespace
{
bool finite(double value) {return std::isfinite(value);}

constexpr double kExtrinsicQuaternionNormTolerance = 1.0e-3;

bool valid_linear_covariance(const std::array<double, 36> & covariance)
{
  constexpr double symmetry_tolerance = 1.0e-9;
  for (int row = 0; row < 3; ++row) {
    if (!finite(covariance[static_cast<std::size_t>(row * 6 + row)]) ||
      covariance[static_cast<std::size_t>(row * 6 + row)] < 0.0)
    {
      return false;
    }
    for (int col = 0; col < 3; ++col) {
      const double value = covariance[static_cast<std::size_t>(row * 6 + col)];
      const double transpose = covariance[static_cast<std::size_t>(col * 6 + row)];
      if (!finite(value) || !finite(transpose) ||
        std::abs(value - transpose) > symmetry_tolerance)
      {
        return false;
      }
    }
  }
  return true;
}

Eigen::Matrix3d read_linear_covariance(const std::array<double, 36> & covariance)
{
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      result(row, col) = covariance[static_cast<std::size_t>(row * 6 + col)];
    }
  }
  return result;
}

void write_linear_covariance(
  const Eigen::Matrix3d & value, std::array<double, 36> * covariance)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      (*covariance)[static_cast<std::size_t>(row * 6 + col)] = value(row, col);
    }
  }
}
}  // namespace

bool set_and_validate_extrinsics(
  const std::vector<double> & translation_xyz_m,
  const std::vector<double> & rotation_xyzw,
  AdapterConfig * config,
  std::string * error)
{
  if (config == nullptr) {
    if (error) {*error = "null config argument";}
    return false;
  }
  if (translation_xyz_m.size() != 3U) {
    if (error) {*error = "translation_xyz_m must contain exactly 3 values";}
    return false;
  }
  if (rotation_xyzw.size() != 4U) {
    if (error) {*error = "rotation_xyzw must contain exactly 4 values in xyzw order";}
    return false;
  }
  if (!std::all_of(translation_xyz_m.begin(), translation_xyz_m.end(), finite) ||
    !std::all_of(rotation_xyzw.begin(), rotation_xyzw.end(), finite))
  {
    if (error) {*error = "extrinsic translation and rotation must be finite";}
    return false;
  }

  Eigen::Quaterniond q_pv(
    rotation_xyzw[3], rotation_xyzw[0], rotation_xyzw[1], rotation_xyzw[2]);
  const double norm = q_pv.norm();
  if (!finite(norm) || norm < config->minimum_quaternion_norm) {
    if (error) {*error = "extrinsic quaternion norm is zero or too small";}
    return false;
  }
  if (std::abs(norm - 1.0) > kExtrinsicQuaternionNormTolerance) {
    if (error) {*error = "extrinsic quaternion norm differs from 1 by more than 1e-3";}
    return false;
  }
  q_pv.normalize();
  std::copy(translation_xyz_m.begin(), translation_xyz_m.end(), config->translation_pv_m.begin());
  config->rotation_pv_xyzw = {{q_pv.x(), q_pv.y(), q_pv.z(), q_pv.w()}};
  if (error) {error->clear();}
  return true;
}

bool finite_pose_and_covariance(const nav_msgs::msg::Odometry & input)
{
  const auto & p = input.pose.pose.position;
  const auto & q = input.pose.pose.orientation;
  if (!finite(p.x) || !finite(p.y) || !finite(p.z) ||
    !finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w))
  {
    return false;
  }
  return std::all_of(
    input.pose.covariance.begin(), input.pose.covariance.end(),
    [](double value) {return finite(value);});
}

bool adapt_odometry(
  const nav_msgs::msg::Odometry & input,
  const rclcpp::Time & now,
  const AdapterConfig & config,
  rclcpp::Time * last_output_stamp,
  nav_msgs::msg::Odometry * output,
  std::string * error)
{
  if (last_output_stamp == nullptr || output == nullptr) {
    if (error) {*error = "null output argument";}
    return false;
  }
  if (!finite_pose_and_covariance(input)) {
    if (error) {*error = "pose or pose covariance contains a non-finite value";}
    return false;
  }

  const auto & in_q = input.pose.pose.orientation;
  Eigen::Quaterniond q_wv(in_q.w, in_q.x, in_q.y, in_q.z);
  if (!finite(q_wv.norm()) || q_wv.norm() < config.minimum_quaternion_norm) {
    if (error) {*error = "quaternion norm is invalid";}
    return false;
  }
  q_wv.normalize();

  const auto & q_xyzw = config.rotation_pv_xyzw;
  if (!std::all_of(config.translation_pv_m.begin(), config.translation_pv_m.end(), finite) ||
    !std::all_of(q_xyzw.begin(), q_xyzw.end(), finite))
  {
    if (error) {*error = "configured extrinsic contains a non-finite value";}
    return false;
  }
  Eigen::Quaterniond q_pv(q_xyzw[3], q_xyzw[0], q_xyzw[1], q_xyzw[2]);
  if (!finite(q_pv.norm()) || q_pv.norm() < config.minimum_quaternion_norm) {
    if (error) {*error = "configured extrinsic quaternion norm is invalid";}
    return false;
  }
  if (std::abs(q_pv.norm() - 1.0) > kExtrinsicQuaternionNormTolerance) {
    if (error) {*error = "configured extrinsic quaternion norm is abnormal";}
    return false;
  }
  q_pv.normalize();
  Eigen::Quaterniond q_wp = q_wv * q_pv.conjugate();
  q_wp.normalize();

  const Eigen::Vector3d p_wv(
    input.pose.pose.position.x, input.pose.pose.position.y, input.pose.pose.position.z);
  const Eigen::Vector3d t_pv(
    config.translation_pv_m[0], config.translation_pv_m[1], config.translation_pv_m[2]);
  const Eigen::Vector3d p_wp = p_wv - q_wp * t_pv;

  *output = input;
  output->header.frame_id = config.frame_id;
  output->child_frame_id = config.child_frame_id;
  output->pose.pose.position.x = p_wp.x();
  output->pose.pose.position.y = p_wp.y();
  output->pose.pose.position.z = p_wp.z();
  output->pose.pose.orientation.x = q_wp.x();
  output->pose.pose.orientation.y = q_wp.y();
  output->pose.pose.orientation.z = q_wp.z();
  output->pose.pose.orientation.w = q_wp.w();

  rclcpp::Time stamp = config.replay_mode ? now : rclcpp::Time(input.header.stamp);
  if (last_output_stamp->nanoseconds() > 0 && stamp <= *last_output_stamp) {
    stamp = rclcpp::Time(last_output_stamp->nanoseconds() + 1, stamp.get_clock_type());
  }
  output->header.stamp = stamp;
  *last_output_stamp = stamp;

  const double nan = std::numeric_limits<double>::quiet_NaN();
  if (!config.send_velocity) {
    output->twist.twist.linear.x = nan;
    output->twist.twist.linear.y = nan;
    output->twist.twist.linear.z = nan;
    output->twist.twist.angular.x = nan;
    output->twist.twist.angular.y = nan;
    output->twist.twist.angular.z = nan;
    output->twist.covariance.fill(nan);
  } else {
    const auto & velocity = input.twist.twist.linear;
    if (!finite(velocity.x) || !finite(velocity.y) || !finite(velocity.z)) {
      if (error) {*error = "velocity requested but input velocity is non-finite";}
      return false;
    }
    if (!valid_linear_covariance(input.twist.covariance)) {
      if (error) {*error = "velocity requested but linear velocity covariance is invalid";}
      return false;
    }
    // OpenVINS odomimu linear velocity is expressed in its global frame. ROS
    // Odometry twist is defined in child_frame_id, so rotate it into the body.
    const Eigen::Vector3d v_global(velocity.x, velocity.y, velocity.z);
    const Eigen::Vector3d v_body = q_wp.conjugate() * v_global;
    output->twist.twist.linear.x = v_body.x();
    output->twist.twist.linear.y = v_body.y();
    output->twist.twist.linear.z = v_body.z();
    const Eigen::Matrix3d body_covariance =
      q_wp.conjugate().toRotationMatrix() * read_linear_covariance(input.twist.covariance) *
      q_wp.toRotationMatrix();
    write_linear_covariance(body_covariance, &output->twist.covariance);
  }

  if (error) {error->clear();}
  return true;
}

}  // namespace estimator_adapter
