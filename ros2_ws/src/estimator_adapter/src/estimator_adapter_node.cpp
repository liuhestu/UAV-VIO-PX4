#include "estimator_adapter/adapter_core.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace estimator_adapter
{

class EstimatorAdapterNode : public rclcpp::Node
{
public:
  EstimatorAdapterNode()
  : Node("estimator_adapter")
  {
    const auto input_topic = declare_parameter<std::string>("input_topic", "/ov_msckf/odomimu");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/mavros/odometry/out");
    output_enabled_ = declare_parameter<bool>("output_enabled", false);
    config_.replay_mode = declare_parameter<bool>("replay_mode", true);
    config_.send_velocity = declare_parameter<bool>("send_velocity", false);
    config_.frame_id = declare_parameter<std::string>("frame_id", "odom");
    config_.child_frame_id = declare_parameter<std::string>("child_frame_id", "base_link");
    const auto translation_xyz_m = declare_parameter<std::vector<double>>(
      "openvins_imu_to_pixhawk_imu.translation_xyz_m", {0.0, 0.0, 0.0});
    const auto rotation_xyzw = declare_parameter<std::vector<double>>(
      "openvins_imu_to_pixhawk_imu.rotation_xyzw", {0.0, 0.0, 0.0, 1.0});
    const double output_rate_hz = declare_parameter<double>("output_rate_hz", 30.0);
    max_input_age_seconds_ = declare_parameter<double>("max_input_age_seconds", 0.20);

    if (output_rate_hz <= 0.0 || max_input_age_seconds_ <= 0.0) {
      throw std::invalid_argument("output_rate_hz and max_input_age_seconds must be positive");
    }
    std::string extrinsic_error;
    if (!set_and_validate_extrinsics(
        translation_xyz_m, rotation_xyzw, &config_, &extrinsic_error))
    {
      throw std::invalid_argument("invalid openvins_imu_to_pixhawk_imu: " + extrinsic_error);
    }

    publisher_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, rclcpp::QoS(10));
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_ = message;
        latest_received_at_ = now();
      });

    const auto period = std::chrono::duration<double>(1.0 / output_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&EstimatorAdapterNode::on_timer, this));

    parameter_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "openvins_imu_to_pixhawk_imu.translation_xyz_m" ||
            parameter.get_name() == "openvins_imu_to_pixhawk_imu.rotation_xyzw")
          {
            result.successful = false;
            result.reason = "extrinsics are startup-only; restart the adapter to change them";
            return result;
          }
        }
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "output_enabled") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
              result.successful = false;
              result.reason = "output_enabled must be boolean";
              return result;
            }
            output_enabled_ = parameter.as_bool();
            RCLCPP_WARN(
              get_logger(), "TEST ONLY / NOT FOR FLIGHT: output_enabled changed to %s",
              output_enabled_ ? "true" : "false");
          }
        }
        return result;
      });

    RCLCPP_WARN(
      get_logger(),
      "TEST ONLY / NOT FOR FLIGHT: output=%s, enabled=%s, replay_mode=%s, velocity=%s, rate=%.1f Hz",
      output_topic.c_str(), output_enabled_ ? "true" : "false",
      config_.replay_mode ? "true" : "false", config_.send_velocity ? "true" : "false",
      output_rate_hz);
  }

private:
  void on_timer()
  {
    nav_msgs::msg::Odometry::ConstSharedPtr input;
    rclcpp::Time received_at(0, 0, get_clock()->get_clock_type());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      input = latest_;
      received_at = latest_received_at_;
    }
    if (!output_enabled_ || !input) {return;}

    const auto current_time = now();
    if ((current_time - received_at).seconds() > max_input_age_seconds_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Input is stale; suppressing odometry output");
      return;
    }

    nav_msgs::msg::Odometry output;
    std::string error;
    if (!adapt_odometry(*input, current_time, config_, &last_output_stamp_, &output, &error)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Rejected OpenVINS sample: %s", error.c_str());
      return;
    }
    publisher_->publish(output);
  }

  AdapterConfig config_;
  bool output_enabled_{false};
  double max_input_age_seconds_{0.2};
  std::mutex mutex_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_;
  rclcpp::Time latest_received_at_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Time last_output_stamp_{0, 0, RCL_SYSTEM_TIME};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace estimator_adapter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<estimator_adapter::EstimatorAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
