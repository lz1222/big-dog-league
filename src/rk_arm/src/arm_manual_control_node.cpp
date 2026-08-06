#include <rclcpp/rclcpp.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "rk_arm/arm_safety_core.hpp"
#include "rk_arm/feedback_bridge_protocol.hpp"
#include "rk_interfaces/msg/arm_raw_state.hpp"
#include "rk_interfaces/srv/jog_arm_joint.hpp"
#include "rk_interfaces/srv/set_arm_gripper.hpp"
#include "rk_interfaces/srv/stop_arm.hpp"

namespace rk_arm {
namespace { ArmLimits DevelopmentLimits() { ArmLimits limits{}; for (auto& axis : limits.axis) axis = {-1.0, 1.0, 0.1, 0.1}; limits.axis[6] = {0.0, 1.0, 0.1, 0.1}; limits.max_timeout_sec = 0.5; return limits; } }

/** ROS 状态与服务进程；只接收本机反馈帧，绝不链接或调用 Unitree DDS。 */
class ArmRawStateNode final : public rclcpp::Node {
 public:
  ArmRawStateNode() : Node("arm_manual_control_node"), manual_motion_enabled_(declare_parameter<bool>("manual_motion_enabled", false)), safety_(DevelopmentLimits(), declare_parameter<double>("feedback_timeout_sec", .3), declare_parameter<double>("source_consistency_tolerance", .5), manual_motion_enabled_), socket_path_(declare_parameter<std::string>("state_socket_path", "/tmp/rk_d1_arm_feedback.sock")) {
    SetupSocket(); raw_state_publisher_ = create_publisher<rk_interfaces::msg::ArmRawState>("arm/raw_state", 10);
    jog_service_ = create_service<rk_interfaces::srv::JogArmJoint>("arm/jog_joint", [this](std::shared_ptr<rk_interfaces::srv::JogArmJoint::Request> request, std::shared_ptr<rk_interfaces::srv::JogArmJoint::Response> response) { HandleJog(*request, *response); });
    gripper_service_ = create_service<rk_interfaces::srv::SetArmGripper>("arm/set_gripper", [this](std::shared_ptr<rk_interfaces::srv::SetArmGripper::Request> request, std::shared_ptr<rk_interfaces::srv::SetArmGripper::Response> response) { HandleGripper(*request, *response); });
    stop_service_ = create_service<rk_interfaces::srv::StopArm>("arm/stop", [this](std::shared_ptr<rk_interfaces::srv::StopArm::Request>, std::shared_ptr<rk_interfaces::srv::StopArm::Response> response) { const auto result = safety_.CheckStop(false, Snapshot().dds_ready, false); response->stopped = result.accepted; response->error_code = static_cast<int32_t>(result.error); response->message = result.reason; });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] { DrainPackets(); PublishState(); });
    RCLCPP_WARN(get_logger(), "DEVELOPMENT DEFAULT / NOT HARDWARE VALIDATED; manual motion remains disabled");
  }
  ~ArmRawStateNode() override { if (socket_fd_ >= 0) ::close(socket_fd_); std::error_code ignored; std::filesystem::remove(socket_path_, ignored); }
 private:
  void SetupSocket() { std::error_code ignored; std::filesystem::remove(socket_path_, ignored); socket_fd_ = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0); if (socket_fd_ < 0 || socket_path_.size() >= sizeof(sockaddr_un::sun_path)) throw std::runtime_error("cannot create feedback socket"); sockaddr_un address{}; address.sun_family = AF_UNIX; std::strncpy(address.sun_path, socket_path_.c_str(), sizeof(address.sun_path) - 1); if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0) throw std::runtime_error("cannot bind feedback socket"); }
  void DrainPackets() { FeedbackBridgePacket packet; while (::recv(socket_fd_, &packet, sizeof(packet), MSG_DONTWAIT) == sizeof(packet)) { if (packet.magic != FeedbackBridgePacket::kMagic) continue; snapshot_.dds_ready = true; snapshot_.angle_valid = packet.angle_valid; snapshot_.servo_valid = packet.servo_valid; snapshot_.app_values = packet.app_values; snapshot_.servo_values = packet.servo_values; snapshot_.enable_status = packet.enable_status; snapshot_.power_status = packet.power_status; snapshot_.error_status = packet.error_status; snapshot_.latest_angle = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(packet.angle_monotonic_ns)); snapshot_.latest_servo = std::chrono::steady_clock::time_point(std::chrono::nanoseconds(packet.servo_monotonic_ns)); } }
  FeedbackSnapshot Snapshot() const { return snapshot_; }
  void PublishState() { const auto snapshot = Snapshot(); const auto now = std::chrono::steady_clock::now(); auto message = rk_interfaces::msg::ArmRawState{}; message.header.stamp = this->now(); message.feedback_valid = snapshot.angle_valid && snapshot.servo_valid; message.feedback_stale = safety_.CurrentState(snapshot, now) == ArmState::kStale; message.sources_consistent = safety_.SourcesConsistent(snapshot); message.app_values = snapshot.app_values; message.enable_status = snapshot.enable_status; message.power_status = snapshot.power_status; message.error_status = snapshot.error_status; message.feedback_age_sec = snapshot.angle_valid ? static_cast<float>(std::chrono::duration<double>(now - snapshot.latest_angle).count()) : -1.0F; message.value_unit = "app_display_unit"; message.reason = ToString(safety_.CurrentState(snapshot, now)); raw_state_publisher_->publish(message); }
  void Reject(const SafetyDecision& decision, bool* accepted, int32_t* code, std::string* message) { *accepted = decision.accepted; *code = static_cast<int32_t>(decision.error); *message = decision.reason; }
  void HandleJog(const rk_interfaces::srv::JogArmJoint::Request& request, rk_interfaces::srv::JogArmJoint::Response& response) { const auto snapshot = Snapshot(); if (request.joint_index < 0 || request.joint_index > 5 || (request.direction != -1.0 && request.direction != 1.0)) { response.accepted = false; response.error_code = static_cast<int32_t>(ArmError::kInvalidRequest); response.message = "joint_index must be 0..5 and direction must be -1 or 1"; return; } Reject(safety_.CheckMotion(snapshot, {request.joint_index, snapshot.app_values[request.joint_index] + request.direction * request.step, request.step, request.max_speed, request.timeout_sec}, false, false, std::chrono::steady_clock::now()), &response.accepted, &response.error_code, &response.message); }
  void HandleGripper(const rk_interfaces::srv::SetArmGripper::Request& request, rk_interfaces::srv::SetArmGripper::Response& response) { Reject(safety_.CheckMotion(Snapshot(), {6, request.target, 0.0, request.max_speed, request.timeout_sec}, false, false, std::chrono::steady_clock::now()), &response.accepted, &response.error_code, &response.message); }
  bool manual_motion_enabled_; ArmSafetyCore safety_; std::string socket_path_; int socket_fd_{-1}; FeedbackSnapshot snapshot_{};
  rclcpp::Publisher<rk_interfaces::msg::ArmRawState>::SharedPtr raw_state_publisher_; rclcpp::Service<rk_interfaces::srv::JogArmJoint>::SharedPtr jog_service_; rclcpp::Service<rk_interfaces::srv::SetArmGripper>::SharedPtr gripper_service_; rclcpp::Service<rk_interfaces::srv::StopArm>::SharedPtr stop_service_; rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace rk_arm
int main(int argc, char** argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<rk_arm::ArmRawStateNode>()); rclcpp::shutdown(); return 0; }
