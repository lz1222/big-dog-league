// MotionSwitcher CheckMode 的只读诊断工具。
//
// 只记录机器人返回的 form/name 原始字符串。它不调用任何选择、释放或切换
// 函数，因此不会改变当前 motion service 的模式。
#include <chrono>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include <unitree/robot/channel/channel_factory.hpp>

namespace
{

double ElapsedMilliseconds(const std::chrono::steady_clock::time_point& started)
{
  return std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
}

int ParsePositiveInt(const char* value, const char* parameter_name)
{
  const int parsed = std::stoi(value);
  if (parsed <= 0) {
    throw std::runtime_error(std::string(parameter_name) + " must be positive");
  }
  return parsed;
}

int RunProbe(
    const std::string& network_interface, int sample_count, int interval_ms,
    const std::string& stop_on_name)
{
  if (network_interface.empty()) {
    throw std::runtime_error("network interface must not be empty");
  }
  std::cout << "MOTION_SWITCHER_DIAG event=CHANNEL_FACTORY_INIT"
            << " interface=" << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::b2::MotionSwitcherClient client;
  client.SetTimeout(2.0F);
  client.Init();
  bool all_success = true;
  int final_sample = sample_count;
  bool transition_seen = false;
  auto next_sample = std::chrono::steady_clock::now();
  for (int sample = 1; sample <= final_sample; ++sample) {
    std::string form;
    std::string name;
    const auto started = std::chrono::steady_clock::now();
    const int32_t result = client.CheckMode(form, name);
    // 单进程定时采样避免反复 DDS 初始化扰动结果；仍只调用 CheckMode，
    // monotonic_ns 让 mcf 回归时刻可与任何 host 组件日志精确关联。
    std::cout << "MOTION_SWITCHER_DIAG event=CHECK_MODE"
              << " sample=" << sample
              << " monotonic_ns=" << std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch()).count()
              << " ret=" << result
              << " form=" << std::quoted(form)
              << " name=" << std::quoted(name)
              << " elapsed_ms=" << std::fixed << std::setprecision(3)
              << ElapsedMilliseconds(started) << std::endl;
    all_success = all_success && result == 0;
    if (!stop_on_name.empty() && !transition_seen && name == stop_on_name) {
      // 发现目标 mode 后继续 5 秒，区分真实回归与单帧 DDS/RPC 异常；
      // 期间不创建任何额外 client，也不改变机器人服务状态。
      transition_seen = true;
      final_sample = sample + 50;
      std::cout << "MOTION_SWITCHER_DIAG event=TARGET_NAME_DETECTED"
                << " target=" << std::quoted(stop_on_name)
                << " sample=" << sample
                << " confirmation_samples=50" << std::endl;
    }
    next_sample += std::chrono::milliseconds(interval_ms);
    if (sample < sample_count) {
      std::this_thread::sleep_until(next_sample);
    }
  }
  return all_success ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    if (argc < 2 || argc > 5) {
      throw std::runtime_error(
          "Usage: go2_sdk_motion_switcher_check_mode_probe <network_interface> "
          "[sample_count] [interval_ms] [stop_on_name]");
    }
    const int sample_count = argc >= 3 ? ParsePositiveInt(argv[2], "sample_count") : 1;
    const int interval_ms = argc >= 4 ? ParsePositiveInt(argv[3], "interval_ms") : 100;
    const std::string stop_on_name = argc >= 5 ? argv[4] : "";
    return RunProbe(argv[1], sample_count, interval_ms, stop_on_name);
  } catch (const std::exception& error) {
    std::cerr << "MOTION_SWITCHER_DIAG event=FATAL message="
              << std::quoted(error.what()) << std::endl;
    return 1;
  }
}
