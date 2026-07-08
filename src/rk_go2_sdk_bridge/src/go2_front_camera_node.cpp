#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/imgcodecs.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/video/video_client.hpp>

namespace
{

class Go2FrontCameraNode : public rclcpp::Node
{
public:
  Go2FrontCameraNode()
  : Node("go2_front_camera_node")
  {
    network_interface_ = declare_parameter<std::string>(
      "network_interface", "eth0");
    image_topic_ = declare_parameter<std::string>(
      "image_topic", "/go2/front/image_raw");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "go2_front_camera");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 8.0);
    sdk_timeout_sec_ = declare_parameter<double>("sdk_timeout_sec", 1.0);

    if (network_interface_.empty()) {
      throw std::runtime_error("network_interface must not be empty");
    }
    if (image_topic_.empty()) {
      throw std::runtime_error("image_topic must not be empty");
    }
    if (publish_rate_hz_ <= 0.0) {
      throw std::runtime_error("publish_rate_hz must be positive");
    }
    if (sdk_timeout_sec_ <= 0.0) {
      throw std::runtime_error("sdk_timeout_sec must be positive");
    }

    publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 3);

    RCLCPP_WARN(
      get_logger(),
      "Starting Go2 front camera via Unitree SDK2: interface=%s, topic=%s",
      network_interface_.c_str(),
      image_topic_.c_str());

    unitree::robot::ChannelFactory::Instance()->Init(
      0,
      network_interface_);
    video_client_ = std::make_unique<unitree::robot::go2::VideoClient>();
    video_client_->SetTimeout(static_cast<float>(sdk_timeout_sec_));
    video_client_->Init();

    const auto period =
      std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Go2FrontCameraNode::onTimer, this));
  }

private:
  void onTimer()
  {
    std::vector<uint8_t> jpeg_bytes;
    const int ret = video_client_->GetImageSample(jpeg_bytes);
    if (ret != 0 || jpeg_bytes.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "GetImageSample failed: ret=%d, bytes=%zu",
        ret,
        jpeg_bytes.size());
      return;
    }

    const cv::Mat encoded(1, static_cast<int>(jpeg_bytes.size()), CV_8UC1,
      jpeg_bytes.data());
    const cv::Mat image_bgr = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image_bgr.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to decode Go2 front camera JPEG, bytes=%zu",
        jpeg_bytes.size());
      return;
    }

    auto message = sensor_msgs::msg::Image();
    message.header.stamp = now();
    message.header.frame_id = frame_id_;
    message.height = static_cast<uint32_t>(image_bgr.rows);
    message.width = static_cast<uint32_t>(image_bgr.cols);
    message.encoding = "bgr8";
    message.is_bigendian = false;
    message.step = static_cast<uint32_t>(image_bgr.cols * image_bgr.elemSize());
    message.data.assign(
      image_bgr.datastart,
      image_bgr.dataend);
    publisher_->publish(message);
  }

  std::string network_interface_;
  std::string image_topic_;
  std::string frame_id_;
  double publish_rate_hz_;
  double sdk_timeout_sec_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<unitree::robot::go2::VideoClient> video_client_;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Go2FrontCameraNode>());
  } catch (const std::exception& error) {
    std::cerr << "go2_front_camera_node error: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
