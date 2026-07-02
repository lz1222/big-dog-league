#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

class Go2SdkCmdVelBridge : public rclcpp::Node
{
public:
  Go2SdkCmdVelBridge()
  : Node("go2_sdk_cmd_vel_bridge"),
    sdk_initialized_(false),
    motion_active_(false),
    has_cmd_(false),
    timeout_stop_sent_(true),
    shutdown_stop_sent_(false)
  {
    declare_and_read_parameters();
    initialize_sdk();

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_,
      rclcpp::QoS(10),
      std::bind(&Go2SdkCmdVelBridge::on_cmd_vel, this, std::placeholders::_1));

    watchdog_timer_ = create_wall_timer(
      watchdog_period_from_timeout(timeout_sec_),
      std::bind(&Go2SdkCmdVelBridge::on_watchdog_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "go2_sdk_cmd_vel_bridge started: topic=%s, interface=%s, "
      "max_vx=%.3f, max_vy=%.3f, max_yaw=%.3f, deadband=%.3f, timeout=%.3f",
      cmd_vel_topic_.c_str(),
      network_interface_.c_str(),
      max_vx_,
      max_vy_,
      max_yaw_,
      deadband_,
      timeout_sec_);
  }

  ~Go2SdkCmdVelBridge()
  {
    stop_on_shutdown();
  }

  void stop_on_shutdown()
  {
    if (shutdown_stop_sent_ || !sdk_initialized_) {
      return;
    }

    shutdown_stop_sent_ = true;
    const int32_t ret = sport_client_.StopMove();
    RCLCPP_INFO(get_logger(), "StopMove ret=%d (shutdown)", ret);
  }

private:
  void declare_and_read_parameters()
  {
    declare_parameter<std::string>("cmd_vel_topic", "/navigation/cmd_vel");
    declare_parameter<std::string>("network_interface", "eth0");
    declare_parameter<double>("max_vx", 0.25);
    declare_parameter<double>("max_vy", 0.15);
    declare_parameter<double>("max_yaw", 0.6);
    declare_parameter<double>("deadband", 0.01);
    declare_parameter<double>("timeout_sec", 0.5);
    declare_parameter<bool>("balance_stand_on_start", true);

    get_parameter("cmd_vel_topic", cmd_vel_topic_);
    get_parameter("network_interface", network_interface_);
    get_parameter("max_vx", max_vx_);
    get_parameter("max_vy", max_vy_);
    get_parameter("max_yaw", max_yaw_);
    get_parameter("deadband", deadband_);
    get_parameter("timeout_sec", timeout_sec_);
    get_parameter("balance_stand_on_start", balance_stand_on_start_);

    if (cmd_vel_topic_.empty()) {
      throw std::runtime_error("cmd_vel_topic must not be empty");
    }
    if (network_interface_.empty()) {
      throw std::runtime_error("network_interface must not be empty");
    }
    if (!is_positive_finite(max_vx_)) {
      throw std::runtime_error("max_vx must be a finite positive number");
    }
    if (!is_positive_finite(max_vy_)) {
      throw std::runtime_error("max_vy must be a finite positive number");
    }
    if (!is_positive_finite(max_yaw_)) {
      throw std::runtime_error("max_yaw must be a finite positive number");
    }
    if (!std::isfinite(deadband_) || deadband_ < 0.0) {
      throw std::runtime_error("deadband must be a finite nonnegative number");
    }
    if (!is_positive_finite(timeout_sec_)) {
      throw std::runtime_error("timeout_sec must be a finite positive number");
    }
  }

  void initialize_sdk()
  {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface_);
    sport_client_.SetTimeout(10.0f);
    sport_client_.Init();
    sdk_initialized_ = true;

    int32_t ret = sport_client_.StopMove();
    RCLCPP_INFO(get_logger(), "StopMove ret=%d (startup)", ret);

    if (balance_stand_on_start_) {
      ret = sport_client_.BalanceStand();
      RCLCPP_INFO(get_logger(), "BalanceStand ret=%d (startup)", ret);
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = std::chrono::steady_clock::now();
    has_cmd_ = true;
    timeout_stop_sent_ = false;

    if (!is_finite_twist(*msg)) {
      motion_active_ = false;
      send_stop("invalid cmd_vel");
      return;
    }

    const float vx = static_cast<float>(
      apply_deadband_and_limit(msg->linear.x, max_vx_));
    const float vy = static_cast<float>(
      apply_deadband_and_limit(msg->linear.y, max_vy_));
    const float yaw = static_cast<float>(
      apply_deadband_and_limit(msg->angular.z, max_yaw_));

    if (vx == 0.0f && vy == 0.0f && yaw == 0.0f) {
      motion_active_ = false;
      send_stop("zero cmd_vel");
      return;
    }

    const int32_t ret = sport_client_.Move(vx, vy, yaw);
    motion_active_ = true;
    RCLCPP_DEBUG(
      get_logger(),
      "Move ret=%d vx=%.3f vy=%.3f yaw=%.3f",
      ret,
      vx,
      vy,
      yaw);
  }

  void on_watchdog_timer()
  {
    if (!has_cmd_ || timeout_stop_sent_) {
      return;
    }

    const auto elapsed = std::chrono::steady_clock::now() - last_cmd_time_;
    const double elapsed_sec =
      std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();

    if (elapsed_sec <= timeout_sec_) {
      return;
    }

    motion_active_ = false;
    timeout_stop_sent_ = true;
    send_stop("cmd_vel timeout");
  }

  void send_stop(const char * reason)
  {
    if (!sdk_initialized_) {
      return;
    }

    const int32_t ret = sport_client_.StopMove();
    RCLCPP_INFO(get_logger(), "StopMove ret=%d (%s)", ret, reason);
  }

  static bool is_positive_finite(const double value)
  {
    return std::isfinite(value) && value > 0.0;
  }

  static bool is_finite_twist(const geometry_msgs::msg::Twist & msg)
  {
    return std::isfinite(msg.linear.x) &&
      std::isfinite(msg.linear.y) &&
      std::isfinite(msg.angular.z);
  }

  double apply_deadband_and_limit(const double value, const double limit) const
  {
    if (std::fabs(value) <= deadband_) {
      return 0.0;
    }

    return std::max(-limit, std::min(limit, value));
  }

  static std::chrono::nanoseconds watchdog_period_from_timeout(
    const double timeout_sec)
  {
    const double period_sec = std::max(0.02, std::min(0.10, timeout_sec / 2.0));
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(period_sec));
  }

  unitree::robot::SportClient sport_client_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;

  std::string cmd_vel_topic_;
  std::string network_interface_;
  double max_vx_;
  double max_vy_;
  double max_yaw_;
  double deadband_;
  double timeout_sec_;
  bool balance_stand_on_start_;

  bool sdk_initialized_;
  bool motion_active_;
  bool has_cmd_;
  bool timeout_stop_sent_;
  bool shutdown_stop_sent_;
  std::chrono::steady_clock::time_point last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<Go2SdkCmdVelBridge> node;

  try {
    node = std::make_shared<Go2SdkCmdVelBridge>();
    rclcpp::spin(node);
  } catch (const std::exception & ex) {
    if (node) {
      RCLCPP_ERROR(node->get_logger(), "go2_sdk_cmd_vel_bridge error: %s", ex.what());
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("go2_sdk_cmd_vel_bridge"),
        "go2_sdk_cmd_vel_bridge startup error: %s",
        ex.what());
    }
  }

  if (node) {
    node->stop_on_shutdown();
    node.reset();
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
