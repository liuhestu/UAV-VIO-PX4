#include "estimator_adapter/adapter_core.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace
{
nav_msgs::msg::Odometry valid_input()
{
  nav_msgs::msg::Odometry message;
  message.header.stamp.sec = 10;
  message.header.frame_id = "global";
  message.child_frame_id = "imu";
  message.pose.pose.position.x = 1.0;
  message.pose.pose.position.y = 2.0;
  message.pose.pose.position.z = 3.0;
  message.pose.pose.orientation.w = 2.0;
  message.twist.twist.linear.x = 1.0;
  message.twist.covariance[0] = 1.0;
  message.twist.covariance[7] = 2.0;
  message.twist.covariance[14] = 3.0;
  return message;
}
}  // namespace

TEST(AdapterCore, RejectsNonFinitePose)
{
  auto input = valid_input();
  input.pose.pose.position.x = std::numeric_limits<double>::quiet_NaN();
  estimator_adapter::AdapterConfig config;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  EXPECT_FALSE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
}

TEST(AdapterCore, NormalizesQuaternionAndSuppressesVelocity)
{
  const auto input = valid_input();
  estimator_adapter::AdapterConfig config;
  config.send_velocity = false;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_EQ(output.header.frame_id, "odom");
  EXPECT_EQ(output.child_frame_id, "base_link");
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.w, 1.0);
  EXPECT_TRUE(std::isfinite(output.pose.pose.position.x));
  EXPECT_TRUE(std::isfinite(output.pose.pose.orientation.w));
  EXPECT_TRUE(std::isnan(output.twist.twist.linear.x));
  EXPECT_TRUE(std::isnan(output.twist.twist.linear.y));
  EXPECT_TRUE(std::isnan(output.twist.twist.linear.z));
  EXPECT_TRUE(std::isnan(output.twist.twist.angular.x));
  EXPECT_TRUE(std::isnan(output.twist.twist.angular.y));
  EXPECT_TRUE(std::isnan(output.twist.twist.angular.z));
  for (const double value : output.twist.covariance) {
    EXPECT_TRUE(std::isnan(value));
  }
}

TEST(AdapterCore, IdentityExtrinsicPreservesPoseAndCovariance)
{
  auto input = valid_input();
  input.pose.pose.orientation.w = 1.0;
  for (std::size_t i = 0; i < input.pose.covariance.size(); ++i) {
    input.pose.covariance[i] = static_cast<double>(i) * 0.01;
  }
  estimator_adapter::AdapterConfig config;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_DOUBLE_EQ(output.pose.pose.position.x, input.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.y, input.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(output.pose.pose.position.z, input.pose.pose.position.z);
  EXPECT_DOUBLE_EQ(output.pose.pose.orientation.w, 1.0);
  EXPECT_EQ(output.pose.covariance, input.pose.covariance);
}

TEST(AdapterCore, AppliesNinetyDegreeFixedRotation)
{
  auto input = valid_input();
  input.pose.pose.orientation.w = 1.0;
  estimator_adapter::AdapterConfig config;
  const double s = std::sqrt(0.5);
  ASSERT_TRUE(estimator_adapter::set_and_validate_extrinsics(
    {0.0, 0.0, 0.0}, {0.0, 0.0, s, s}, &config, nullptr));
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_NEAR(output.pose.pose.orientation.z, -s, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, s, 1.0e-12);
}

TEST(AdapterCore, AppliesPureTranslation)
{
  auto input = valid_input();
  input.pose.pose.orientation.w = 1.0;
  estimator_adapter::AdapterConfig config;
  ASSERT_TRUE(estimator_adapter::set_and_validate_extrinsics(
    {0.5, -1.0, 2.0}, {0.0, 0.0, 0.0, 1.0}, &config, nullptr));
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_NEAR(output.pose.pose.position.x, 0.5, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y, 3.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.z, 1.0, 1.0e-12);
}

TEST(AdapterCore, AppliesCombinedSe3Transform)
{
  auto input = valid_input();
  input.pose.pose.orientation.w = 1.0;
  estimator_adapter::AdapterConfig config;
  const double s = std::sqrt(0.5);
  ASSERT_TRUE(estimator_adapter::set_and_validate_extrinsics(
    {1.0, 0.0, 0.0}, {0.0, 0.0, s, s}, &config, nullptr));
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  // q_WP is -90 degrees around Z, so R_WP * [1,0,0] = [0,-1,0].
  EXPECT_NEAR(output.pose.pose.position.x, 1.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.y, 3.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.position.z, 3.0, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.orientation.z, -s, 1.0e-12);
  EXPECT_NEAR(output.pose.pose.orientation.w, s, 1.0e-12);
}

TEST(AdapterCore, RejectsInvalidExtrinsicArrays)
{
  estimator_adapter::AdapterConfig config;
  std::string error;
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}, &config, &error));
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, &config, &error));
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0}, &config, &error));
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {std::numeric_limits<double>::infinity(), 0.0, 0.0},
    {0.0, 0.0, 0.0, 1.0}, &config, &error));
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, &config, &error));
  EXPECT_FALSE(estimator_adapter::set_and_validate_extrinsics(
    {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 2.0}, &config, &error));
}

TEST(AdapterCore, EnforcesMonotonicReplayTimestamp)
{
  const auto input = valid_input();
  estimator_adapter::AdapterConfig config;
  rclcpp::Time last(20, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(19, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_EQ(rclcpp::Time(output.header.stamp).nanoseconds(), 20000000001LL);
}

TEST(AdapterCore, RotatesGlobalVelocityIntoChildFrame)
{
  auto input = valid_input();
  const double half_sqrt_two = std::sqrt(0.5);
  input.pose.pose.orientation.z = half_sqrt_two;
  input.pose.pose.orientation.w = half_sqrt_two;
  input.twist.twist.linear.x = 1.0;
  input.twist.twist.linear.y = 0.0;
  estimator_adapter::AdapterConfig config;
  config.send_velocity = true;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_NEAR(output.twist.twist.linear.x, 0.0, 1.0e-12);
  EXPECT_NEAR(output.twist.twist.linear.y, -1.0, 1.0e-12);
}

TEST(AdapterCore, RotatesLinearVelocityCovarianceIntoChildFrame)
{
  auto input = valid_input();
  const double half_sqrt_two = std::sqrt(0.5);
  input.pose.pose.orientation.z = half_sqrt_two;
  input.pose.pose.orientation.w = half_sqrt_two;
  estimator_adapter::AdapterConfig config;
  config.send_velocity = true;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  ASSERT_TRUE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
  EXPECT_NEAR(output.twist.covariance[0], 2.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[7], 1.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[14], 3.0, 1.0e-12);
  EXPECT_NEAR(output.twist.covariance[1], output.twist.covariance[6], 1.0e-12);
}

TEST(AdapterCore, RejectsInvalidVelocityCovarianceWhenVelocityIsEnabled)
{
  auto input = valid_input();
  input.twist.covariance[7] = -1.0;
  estimator_adapter::AdapterConfig config;
  config.send_velocity = true;
  rclcpp::Time last(0, 0, RCL_SYSTEM_TIME);
  nav_msgs::msg::Odometry output;
  std::string error;
  EXPECT_FALSE(estimator_adapter::adapt_odometry(
    input, rclcpp::Time(20, 0, RCL_SYSTEM_TIME), config, &last, &output, &error));
}
