// Go2 SportModeState 的完整只读 JSONL 记录器。
//
// 该工具只订阅 rt/sportmodestate，并逐帧输出 IDL 中的全部字段，供人工步态
// 签名 A/B 比较。它不创建 SportClient、RobotStateClient 或发布者，因此不会
// 获取运动租约、切换服务或向机器狗下发任何控制命令。
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

constexpr const char* kSportModeStateTopic = "rt/sportmodestate";

std::int64_t MonotonicNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

double ParsePositiveDuration(const char* raw)
{
  const double duration = std::stod(raw);
  if (!std::isfinite(duration) || duration <= 0.0) {
    throw std::runtime_error("duration_sec must be finite and positive");
  }
  return duration;
}

void PrintNumber(float value)
{
  // 非有限数在 JSON 中显式记录为 null，避免损坏完整采集文件的逐行解析。
  if (!std::isfinite(value)) {
    std::cout << "null";
    return;
  }
  std::cout << std::setprecision(9) << value;
}

template <typename T, std::size_t Size>
void PrintArray(const std::array<T, Size>& values)
{
  std::cout << '[';
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index != 0) std::cout << ',';
    if constexpr (std::is_floating_point<T>::value) {
      PrintNumber(values[index]);
    } else {
      std::cout << values[index];
    }
  }
  std::cout << ']';
}

void PrintPath(const std::array<unitree_go::msg::dds_::PathPoint_, 10>& path)
{
  std::cout << '[';
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (index != 0) std::cout << ',';
    const auto& point = path[index];
    std::cout << "{\"t_from_start\":";
    PrintNumber(point.t_from_start());
    std::cout << ",\"x\":";
    PrintNumber(point.x());
    std::cout << ",\"y\":";
    PrintNumber(point.y());
    std::cout << ",\"yaw\":";
    PrintNumber(point.yaw());
    std::cout << ",\"vx\":";
    PrintNumber(point.vx());
    std::cout << ",\"vy\":";
    PrintNumber(point.vy());
    std::cout << ",\"vyaw\":";
    PrintNumber(point.vyaw());
    std::cout << '}';
  }
  std::cout << ']';
}

class RawCapture
{
public:
  RawCapture()
  {
    subscriber_.reset(new unitree::robot::ChannelSubscriber<
        unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(&RawCapture::OnState, this, std::placeholders::_1), 1);
  }

  int frame_count() const { return frame_count_.load(); }

private:
  void OnState(const void* message)
  {
    if (message == nullptr) return;
    const auto& state = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    const int frame = frame_count_.fetch_add(1) + 1;
    // DDS 回调可与终端刷新并发；一行一锁保证每条 JSON 可独立解析。
    std::lock_guard<std::mutex> lock(output_mutex_);
    const auto& imu = state.imu_state();
    std::cout << "{\"frame\":" << frame
              << ",\"host_monotonic_ns\":" << MonotonicNanoseconds()
              << ",\"stamp\":{\"sec\":" << state.stamp().sec()
              << ",\"nanosec\":" << state.stamp().nanosec() << '}'
              << ",\"error_code\":" << state.error_code()
              << ",\"mode\":" << static_cast<unsigned int>(state.mode())
              << ",\"progress\":";
    PrintNumber(state.progress());
    std::cout << ",\"gait_type\":" << static_cast<unsigned int>(state.gait_type())
              << ",\"foot_raise_height\":";
    PrintNumber(state.foot_raise_height());
    std::cout << ",\"position\":";
    PrintArray(state.position());
    std::cout << ",\"body_height\":";
    PrintNumber(state.body_height());
    std::cout << ",\"velocity\":";
    PrintArray(state.velocity());
    std::cout << ",\"yaw_speed\":";
    PrintNumber(state.yaw_speed());
    std::cout << ",\"range_obstacle\":";
    PrintArray(state.range_obstacle());
    std::cout << ",\"foot_force\":";
    PrintArray(state.foot_force());
    std::cout << ",\"foot_position_body\":";
    PrintArray(state.foot_position_body());
    std::cout << ",\"foot_speed_body\":";
    PrintArray(state.foot_speed_body());
    std::cout << ",\"imu_state\":{\"quaternion\":";
    PrintArray(imu.quaternion());
    std::cout << ",\"gyroscope\":";
    PrintArray(imu.gyroscope());
    std::cout << ",\"accelerometer\":";
    PrintArray(imu.accelerometer());
    std::cout << ",\"rpy\":";
    PrintArray(imu.rpy());
    std::cout << ",\"temperature\":" << static_cast<unsigned int>(imu.temperature())
              << "},\"path_point\":";
    PrintPath(state.path_point());
    std::cout << "}" << std::endl;
  }

  std::atomic<int> frame_count_{0};
  std::mutex output_mutex_;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_>
      subscriber_;
};

int RunCapture(const std::string& network_interface, double duration_sec)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  std::cerr << "RAW_CAPTURE event=CHANNEL_FACTORY_INIT interface="
            << network_interface << " domain=0 duration_sec=" << duration_sec << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  RawCapture capture;
  std::this_thread::sleep_for(std::chrono::duration<double>(duration_sec));
  std::cerr << "RAW_CAPTURE event=COMPLETE frames=" << capture.frame_count() << std::endl;
  return capture.frame_count() > 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc != 3) {
      throw std::runtime_error(
          "Usage: go2_sdk_sport_state_raw_capture <network_interface> <duration_sec>");
    }
    return RunCapture(argv[1], ParsePositiveDuration(argv[2]));
  } catch (const std::exception& error) {
    std::cerr << "RAW_CAPTURE event=FATAL message=" << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
