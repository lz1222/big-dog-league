#include <rclcpp/rclcpp.hpp>

#include <array>
#include <chrono>
#include <memory>
#include <mutex>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"
#include "rk_arm/arm_safety_core.hpp"
#include "rk_arm/d1_feedback_parser.hpp"
#include "rk_arm/single_writer_guard.hpp"
#include "rk_interfaces/msg/arm_raw_state.hpp"
#include "rk_interfaces/srv/jog_arm_joint.hpp"
#include "rk_interfaces/srv/set_arm_gripper.hpp"
#include "rk_interfaces/srv/stop_arm.hpp"

namespace rk_arm {
namespace {
ArmLimits DevelopmentLimits() {
  // 未经真机验证时仅保留极小 App 显示值范围；默认手动控制关闭，数值不会下发。
  ArmLimits limits{};
  for (auto& axis : limits.axis) axis = {-1.0, 1.0, 0.1, 0.1};
  limits.axis[6] = {0.0, 1.0, 0.1, 0.1};
  limits.max_timeout_sec = 0.5;
  return limits;
}
}  // namespace

/** 唯一允许持有 D1 DDS command writer 的节点；当前默认不创建 writer。 */
class D1DdsDriverNode final : public rclcpp::Node {
 public:
  D1DdsDriverNode() : Node("d1_dds_driver_node"),
      manual_motion_enabled_(declare_parameter<bool>("manual_motion_enabled", false)),
      feedback_timeout_sec_(declare_parameter<double>("feedback_timeout_sec", 0.3)),
      safety_(DevelopmentLimits(), feedback_timeout_sec_, declare_parameter<double>("source_consistency_tolerance", 0.5), manual_motion_enabled_),
      writer_guard_(declare_parameter<std::string>("writer_lock_path", "/tmp/rk_d1_arm_writer.lock")) {
    const auto interface = declare_parameter<std::string>("network_interface", "eth0");
    feedback_topic_ = declare_parameter<std::string>("feedback_topic", "rt/arm_Feedback");
    servo_topic_ = declare_parameter<std::string>("servo_feedback_topic", "current_servo_angle");
    snapshot_.dds_ready = false;
    // 反馈 DDS 初始化仍会连接总线，但绝不创建 command topic writer。
    try {
      unitree::robot::ChannelFactory::Instance()->Init(0, interface);
      arm_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>(feedback_topic_);
      arm_subscriber_->InitChannel([this](const void* raw) { OnArmFeedback(raw); });
      servo_subscriber_ = std::make_unique<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>(servo_topic_);
      servo_subscriber_->InitChannel([this](const void* raw) { OnServoFeedback(raw); });
      snapshot_.dds_ready = true;
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "DDS feedback initialization failed; fail closed: %s", error.what());
    }
    if (manual_motion_enabled_) {
      // 本阶段仍不创建 writer：命令 ID 映射和停止协议未确认，必须保持关闭。
      RCLCPP_ERROR(get_logger(), "manual_motion_enabled was requested but command schema is unconfirmed; writer remains disabled");
    }
    raw_state_publisher_ = create_publisher<rk_interfaces::msg::ArmRawState>("arm/raw_state", rclcpp::QoS(10));
    // Foxy 的服务回调不支持泛型 lambda；明确类型也让接口边界更可审计。
    jog_service_ = create_service<rk_interfaces::srv::JogArmJoint>("arm/jog_joint",
        [this](std::shared_ptr<rk_interfaces::srv::JogArmJoint::Request> request,
               std::shared_ptr<rk_interfaces::srv::JogArmJoint::Response> response) { HandleJog(*request, *response); });
    gripper_service_ = create_service<rk_interfaces::srv::SetArmGripper>("arm/set_gripper",
        [this](std::shared_ptr<rk_interfaces::srv::SetArmGripper::Request> request,
               std::shared_ptr<rk_interfaces::srv::SetArmGripper::Response> response) { HandleGripper(*request, *response); });
    stop_service_ = create_service<rk_interfaces::srv::StopArm>("arm/stop",
        [this](std::shared_ptr<rk_interfaces::srv::StopArm::Request>,
               std::shared_ptr<rk_interfaces::srv::StopArm::Response> response) { HandleStop(*response); });
    state_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { PublishRawState(); });
    RCLCPP_WARN(get_logger(), "DEVELOPMENT DEFAULT / NOT HARDWARE VALIDATED; local lock cannot prove no remote DDS writer exists");
  }

  ~D1DdsDriverNode() override {
    // 停止协议尚未确认，析构时不伪造 DDS stop；没有 writer 时也不会产生命令。
    if (arm_subscriber_) arm_subscriber_->CloseChannel();
    if (servo_subscriber_) servo_subscriber_->CloseChannel();
  }

 private:
  void OnArmFeedback(const void* raw) {
    const auto* message = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw);
    D1FeedbackFrame frame;
    if (!ParseD1Feedback(message->data_(), &frame)) return;  // 坏 JSON 仅丢弃，不破坏已有有效状态。
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (frame.app_values) { snapshot_.app_values = *frame.app_values; snapshot_.angle_valid = true; snapshot_.latest_angle = now; }
    if (frame.enable_status) { snapshot_.enable_status = *frame.enable_status; snapshot_.power_status = *frame.power_status; snapshot_.error_status = *frame.error_status; }
  }
  void OnServoFeedback(const void* raw) {
    const auto* value = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.servo_values = {value->servo0_data_(), value->servo1_data_(), value->servo2_data_(), value->servo3_data_(), value->servo4_data_(), value->servo5_data_(), value->servo6_data_()};
    snapshot_.servo_valid = true; snapshot_.latest_servo = std::chrono::steady_clock::now();
  }
  FeedbackSnapshot Snapshot() const { std::lock_guard<std::mutex> lock(snapshot_mutex_); return snapshot_; }
  void PublishRawState() {
    const auto snapshot = Snapshot(); const auto now = std::chrono::steady_clock::now();
    auto message = rk_interfaces::msg::ArmRawState{};
    message.header.stamp = this->now(); message.feedback_valid = snapshot.angle_valid && snapshot.servo_valid;
    message.feedback_stale = safety_.CurrentState(snapshot, now) == ArmState::kStale;
    message.sources_consistent = safety_.SourcesConsistent(snapshot); message.app_values = snapshot.app_values;
    message.enable_status = snapshot.enable_status; message.power_status = snapshot.power_status; message.error_status = snapshot.error_status;
    message.feedback_age_sec = snapshot.angle_valid ? static_cast<float>(std::chrono::duration<double>(now - snapshot.latest_angle).count()) : -1.0F;
    message.value_unit = "app_display_unit"; message.reason = ToString(safety_.CurrentState(snapshot, now));
    raw_state_publisher_->publish(message);
  }
  void Reject(const SafetyDecision& decision, bool* accepted, int32_t* code, std::string* message) const {
    *accepted = decision.accepted; *code = static_cast<int32_t>(decision.error); *message = decision.reason;
  }
  void HandleJog(const rk_interfaces::srv::JogArmJoint::Request& request, rk_interfaces::srv::JogArmJoint::Response& response) {
    const auto current = Snapshot();
    if (request.joint_index < 0 || request.joint_index > 5 || (request.direction != -1.0 && request.direction != 1.0)) {
      response.accepted = false; response.error_code = static_cast<int32_t>(ArmError::kInvalidRequest); response.message = "joint_index must be 0..5 and direction must be -1 or 1"; return;
    }
    const MotionRequest motion{request.joint_index, current.app_values[request.joint_index] + request.direction * request.step, request.step, request.max_speed, request.timeout_sec};
    // 命令 ID 与 App 映射、停止 JSON 均尚未经官方协议确认，所以 schema=false，必定 fail closed。
    const auto decision = safety_.CheckMotion(current, motion, false, false, std::chrono::steady_clock::now());
    Reject(decision, &response.accepted, &response.error_code, &response.message);
  }
  void HandleGripper(const rk_interfaces::srv::SetArmGripper::Request& request, rk_interfaces::srv::SetArmGripper::Response& response) {
    const auto motion = MotionRequest{6, request.target, 0.0, request.max_speed, request.timeout_sec};
    const auto decision = safety_.CheckMotion(Snapshot(), motion, false, false, std::chrono::steady_clock::now());
    Reject(decision, &response.accepted, &response.error_code, &response.message);
  }
  void HandleStop(rk_interfaces::srv::StopArm::Response& response) {
    const auto decision = safety_.CheckStop(false, Snapshot().dds_ready, false);
    response.stopped = decision.accepted; response.error_code = static_cast<int32_t>(decision.error); response.message = decision.reason;
  }

  bool manual_motion_enabled_; double feedback_timeout_sec_; ArmSafetyCore safety_; SingleWriterGuard writer_guard_;
  std::string feedback_topic_, servo_topic_; mutable std::mutex snapshot_mutex_; FeedbackSnapshot snapshot_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_subscriber_;
  std::unique_ptr<unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> servo_subscriber_;
  rclcpp::Publisher<rk_interfaces::msg::ArmRawState>::SharedPtr raw_state_publisher_;
  rclcpp::Service<rk_interfaces::srv::JogArmJoint>::SharedPtr jog_service_;
  rclcpp::Service<rk_interfaces::srv::SetArmGripper>::SharedPtr gripper_service_;
  rclcpp::Service<rk_interfaces::srv::StopArm>::SharedPtr stop_service_; rclcpp::TimerBase::SharedPtr state_timer_;
};
}  // namespace rk_arm

int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<rk_arm::D1DdsDriverNode>()); rclcpp::shutdown(); return 0; }
