#include <rclcpp/rclcpp.hpp>

/**
 * 手动控制入口保留为 ROS 图内节点。
 * 它从不包含 Unitree DDS 头文件；所有请求必须经 d1_dds_driver_node 的单一安全门。
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("arm_manual_control_node");
  RCLCPP_WARN(node->get_logger(), "Manual controls are service-only; DEVELOPMENT DEFAULT / NOT HARDWARE VALIDATED");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
