// 只读订阅 Go2 高层运动状态。它既可供人工观察，也可作为冷启动控制面门禁；
// 全文件不创建 SportClient，因而不会向机器狗发送任何动作请求。
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <condition_variable>
#include <cerrno>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

namespace
{

constexpr const char* kSportModeStateTopic = "rt/sportmodestate";
constexpr double kDefaultDurationSec = 3.0;
constexpr double kDefaultPrintRateHz = 2.0;
volatile std::sig_atomic_t g_running = 1;

struct CalibrationConfig
{
  int max_valid_frames{0};
};

struct GateConfig
{
  double timeout_sec{0.0};
  int required_frames{0};
  int max_frame_gap_ms{0};
  std::string probe_instance_id;
  std::string pass_evidence_path;
  bool controlled_terminal_success{false};
};

struct MonitorStatistics
{
  int raw_frames{0};
  int valid_frames{0};
  int invalid_frames{0};
  int max_frame_gap_ms{0};
  std::int64_t first_valid_monotonic_ns{0};
  std::int64_t last_valid_monotonic_ns{0};
};

double ParsePositiveDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be finite and positive");
  }
  return value;
}

double MonotonicSeconds()
{
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t MonotonicNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::int64_t WallNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

void SignalHandler(int)
{
  // 只置位退出标志；监控进程不拥有 SportClient，因此退出不触发任何停车动作。
  g_running = 0;
}

const char* GaitTypeName(uint8_t gait_type)
{
  // 名称来自 Unitree SDK2 Go2 示例；未知值保留原始数字，避免错误解释固件扩展。
  switch (gait_type) {
    case 0:
      return "idle";
    case 1:
      return "trot";
    case 2:
      return "trot_running";
    case 3:
      return "climb_stair";
    case 4:
      return "trot_obstacle";
    default:
      return "unknown";
  }
}

class SportStateMonitor
{
public:
  explicit SportStateMonitor(bool emit_calibration_frames = false)
  : emit_calibration_frames_(emit_calibration_frames)
  {
    subscriber_.reset(
        new unitree::robot::ChannelSubscriber<
            unitree_go::msg::dds_::SportModeState_>(kSportModeStateTopic));
    subscriber_->InitChannel(
        std::bind(
            &SportStateMonitor::OnState, this, std::placeholders::_1),
        1);
  }

  bool Snapshot(unitree_go::msg::dds_::SportModeState_& state)
  {
    if (!received_.load()) {
      return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state = state_;
    return true;
  }

  /** 返回自启动以来收到的原始、有效状态帧和相邻有效帧最大间隔。
   * 时间戳使用本机单调时钟，只用于判断 DDS 数据连续性，绝不信任机器人墙钟。
   */
  MonitorStatistics GetStatistics() const
  {
    MonitorStatistics statistics;
    statistics.raw_frames = raw_frames_.load();
    statistics.valid_frames = valid_frames_.load();
    statistics.invalid_frames = invalid_frames_.load();
    statistics.max_frame_gap_ms = max_frame_gap_ms_.load();
    statistics.first_valid_monotonic_ns = first_valid_monotonic_ns_.load();
    statistics.last_valid_monotonic_ns = last_valid_monotonic_ns_.load();
    return statistics;
  }

  bool Shutdown()
  {
    // SDK reader 的回调运行于 DDS 线程。先拒绝新回调、断开 reader，再等待
    // 已进入的回调离开，避免 ChannelFactory::Release 时仍有 this 指针被持有。
    accepting_callbacks_.store(false);
    if (subscriber_) {
      subscriber_->CloseChannel();
      subscriber_.reset();
    }
    std::unique_lock<std::mutex> lock(callback_mutex_);
    return callback_cv_.wait_for(
        lock, std::chrono::seconds(1), [this]() {
          return active_callbacks_.load() == 0;
        });
  }

private:
  static bool IsFiniteState(const unitree_go::msg::dds_::SportModeState_& state)
  {
    // 这些数组来自固定 IDL；校验所有会被诊断输出的数值，避免坏帧伪装成就绪。
    const auto& position = state.position();
    const auto& velocity = state.velocity();
    const auto& rpy = state.imu_state().rpy();
    for (const auto value : position) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    for (const auto value : velocity) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    for (const auto value : rpy) {
      if (!std::isfinite(value)) {
        return false;
      }
    }
    return std::isfinite(state.progress()) &&
           std::isfinite(state.yaw_speed()) &&
           std::isfinite(state.body_height()) &&
           std::isfinite(state.foot_raise_height());
  }

  void OnState(const void* message)
  {
    if (!accepting_callbacks_.load() || message == nullptr) {
      return;
    }
    active_callbacks_.fetch_add(1);
    if (!accepting_callbacks_.load()) {
      FinishCallback();
      return;
    }
    const int raw_frame = raw_frames_.fetch_add(1) + 1;
    const std::int64_t monotonic_ns = MonotonicNanoseconds();
    const std::int64_t wall_ns = WallNanoseconds();
    if (emit_calibration_frames_ && raw_frame == 1) {
      // DDS 发现的首个可观察证据就是本订阅收到的第一份样本；不伪造独立发现事件。
      std::cout << "CALIBRATION_EVENT event=FIRST_DDS_DISCOVERY"
                << " wall_ns=" << wall_ns
                << " monotonic_ns=" << monotonic_ns << std::endl;
    }
    const auto& received = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(
        message);
    if (!IsFiniteState(received)) {
      const int invalid_frame = invalid_frames_.fetch_add(1) + 1;
      if (emit_calibration_frames_) {
        std::cout << "CALIBRATION_FRAME raw_index=" << raw_frame
                  << " valid=0 invalid_index=" << invalid_frame
                  << " reason=nonfinite_field wall_ns=" << wall_ns
                  << " monotonic_ns=" << monotonic_ns << std::endl;
      }
      FinishCallback();
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> statistics_lock(statistics_mutex_);
      if (has_valid_frame_) {
        const int gap_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - last_valid_frame_).count());
        int previous_max = max_frame_gap_ms_.load();
        while (gap_ms > previous_max &&
               !max_frame_gap_ms_.compare_exchange_weak(previous_max, gap_ms)) {
        }
      }
      last_valid_frame_ = now;
      has_valid_frame_ = true;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = received;
    received_.store(true);
    const int valid_frame = valid_frames_.fetch_add(1) + 1;
    if (valid_frame == 1) {
      first_valid_monotonic_ns_.store(monotonic_ns);
    }
    last_valid_monotonic_ns_.store(monotonic_ns);
    if (emit_calibration_frames_) {
      std::cout << "CALIBRATION_FRAME raw_index=" << raw_frame
                << " valid=1 valid_index=" << valid_frame
                << " reason=none wall_ns=" << wall_ns
                  << " monotonic_ns=" << monotonic_ns << std::endl;
    }
    FinishCallback();
  }

  void FinishCallback()
  {
    if (active_callbacks_.fetch_sub(1) == 1) {
      std::lock_guard<std::mutex> lock(callback_mutex_);
      callback_cv_.notify_all();
    }
  }

  std::mutex mutex_;
  std::mutex statistics_mutex_;
  std::atomic<bool> received_{false};
  std::atomic<int> raw_frames_{0};
  std::atomic<int> valid_frames_{0};
  std::atomic<int> invalid_frames_{0};
  std::atomic<int> max_frame_gap_ms_{0};
  std::atomic<std::int64_t> first_valid_monotonic_ns_{0};
  std::atomic<std::int64_t> last_valid_monotonic_ns_{0};
  std::atomic<bool> accepting_callbacks_{true};
  std::atomic<int> active_callbacks_{0};
  std::mutex callback_mutex_;
  std::condition_variable callback_cv_;
  bool has_valid_frame_{false};
  std::chrono::steady_clock::time_point last_valid_frame_;
  bool emit_calibration_frames_{false};
  unitree_go::msg::dds_::SportModeState_ state_;
  unitree::robot::ChannelSubscriberPtr<
      unitree_go::msg::dds_::SportModeState_> subscriber_;
};

void PrintState(const unitree_go::msg::dds_::SportModeState_& state)
{
  const auto& position = state.position();
  const auto& velocity = state.velocity();
  const auto& rpy = state.imu_state().rpy();

  std::cout
      << "SPORT_STATE"
      << " mode=" << static_cast<int>(state.mode())
      << " gait_type=" << static_cast<int>(state.gait_type())
      << " gait_name=" << GaitTypeName(state.gait_type())
      << " progress=" << state.progress()
      << " error_code=" << state.error_code()
      << " position=[" << position[0] << "," << position[1] << ","
      << position[2] << "]"
      << " velocity=[" << velocity[0] << "," << velocity[1] << ","
      << velocity[2] << "]"
      << " yaw_speed=" << state.yaw_speed()
      << " rpy=[" << rpy[0] << "," << rpy[1] << "," << rpy[2] << "]"
      << " body_height=" << state.body_height()
      << " foot_raise_height=" << state.foot_raise_height()
      << std::endl;
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " <network_interface> [duration_sec] [print_rate_hz]\n"
      << "  " << program
      << " <network_interface> --gate --timeout-sec SEC"
      << " --required-frames COUNT --max-frame-gap-ms MS"
      << " --probe-instance-id ID --pass-evidence PATH"
      << " [--controlled-terminal-success]\n\n"
      << "  " << program
      << " <network_interface> --calibration-stream"
      << " --max-valid-frames COUNT\n\n"
      << "This tool is READ_ONLY and subscribes to "
      << kSportModeStateTopic << ".\n";
}

CalibrationConfig ParseCalibrationArguments(int argc, char** argv)
{
  CalibrationConfig config;
  for (int index = 3; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--calibration-stream") {
      continue;
    }
    if (option != "--max-valid-frames" || index + 1 >= argc) {
      throw std::runtime_error(
          "--calibration-stream requires --max-valid-frames COUNT");
    }
    config.max_valid_frames = std::stoi(argv[++index]);
  }
  if (config.max_valid_frames <= 0) {
    throw std::runtime_error(
        "--calibration-stream requires positive --max-valid-frames");
  }
  return config;
}

int RunCalibrationStream(
    const std::string& network_interface, const CalibrationConfig& config)
{
  // 此模式只采集原始状态帧；max_valid_frames 是采集范围，不是生产就绪阈值。
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  std::cout << "CALIBRATION_EVENT event=CHANNEL_FACTORY_INIT_START"
            << " wall_ns=" << WallNanoseconds()
            << " monotonic_ns=" << MonotonicNanoseconds()
            << " interface=" << network_interface << " domain=0" << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  std::cout << "CALIBRATION_EVENT event=CHANNEL_FACTORY_INIT_COMPLETE"
            << " wall_ns=" << WallNanoseconds()
            << " monotonic_ns=" << MonotonicNanoseconds() << std::endl;
  SportStateMonitor monitor(true);
  while (g_running) {
    const MonitorStatistics statistics = monitor.GetStatistics();
    if (statistics.valid_frames >= config.max_valid_frames) {
      std::cout << "CALIBRATION_EVENT event=MONITOR_TARGET_REACHED"
                << " wall_ns=" << WallNanoseconds()
                << " monotonic_ns=" << MonotonicNanoseconds()
                << " raw_frames=" << statistics.raw_frames
                << " valid_frames=" << statistics.valid_frames
                << " invalid_frames=" << statistics.invalid_frames << std::endl;
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "CALIBRATION_EVENT event=MONITOR_PROCESS_EXIT"
            << " wall_ns=" << WallNanoseconds()
            << " monotonic_ns=" << MonotonicNanoseconds()
            << " reason=signal" << std::endl;
  return 0;
}

GateConfig ParseGateArguments(int argc, char** argv)
{
  GateConfig config;
  for (int index = 3; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--gate") {
      continue;
    }
    if (option == "--controlled-terminal-success") {
      config.controlled_terminal_success = true;
      continue;
    }
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value for " + option);
    }
    const std::string value = argv[++index];
    if (option == "--timeout-sec") {
      config.timeout_sec = ParsePositiveDouble(value.c_str(), "timeout_sec");
    } else if (option == "--required-frames") {
      config.required_frames = std::stoi(value);
    } else if (option == "--max-frame-gap-ms") {
      config.max_frame_gap_ms = std::stoi(value);
    } else if (option == "--probe-instance-id") {
      config.probe_instance_id = value;
    } else if (option == "--pass-evidence") {
      config.pass_evidence_path = value;
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }
  if (config.timeout_sec <= 0.0 || config.required_frames <= 0 ||
      config.max_frame_gap_ms <= 0) {
    throw std::runtime_error(
        "--gate requires positive --timeout-sec, --required-frames and "
        "--max-frame-gap-ms");
  }
  if (config.probe_instance_id.empty() || config.pass_evidence_path.empty()) {
    throw std::runtime_error(
        "--gate requires --probe-instance-id and --pass-evidence");
  }
  return config;
}

bool WriteAll(int file_descriptor, const std::string& content)
{
  std::size_t offset = 0;
  while (offset < content.size()) {
    const ssize_t written = ::write(
        file_descriptor, content.data() + offset, content.size() - offset);
    if (written <= 0) {
      return false;
    }
    offset += static_cast<std::size_t>(written);
  }
  return true;
}

bool WritePassEvidence(
    const GateConfig& config, const MonitorStatistics& statistics,
    std::string& error)
{
  // 使用 O_EXCL 防止旧 PASS 文件被本轮重用；完成 write/flush/fsync 后才允许
  // controlled _Exit，使父进程能够把 RC=0 与本次实例证据严格绑定。
  const int file_descriptor = ::open(
      config.pass_evidence_path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (file_descriptor < 0) {
    error = "evidence_open_errno=" + std::to_string(errno);
    return false;
  }
  std::ostringstream payload;
  payload << "{\n"
          << "  \"probe_instance_id\":\"" << config.probe_instance_id << "\",\n"
          << "  \"classification\":\"CONTROL_PLANE_OBSERVATION_PASS\",\n"
          << "  \"total_frames\":" << statistics.raw_frames << ",\n"
          << "  \"valid_frames\":" << statistics.valid_frames << ",\n"
          << "  \"invalid_frames\":" << statistics.invalid_frames << ",\n"
          << "  \"max_gap_ms\":" << statistics.max_frame_gap_ms << ",\n"
          << "  \"first_monotonic_ns\":" << statistics.first_valid_monotonic_ns << ",\n"
          << "  \"last_monotonic_ns\":" << statistics.last_valid_monotonic_ns << ",\n"
          << "  \"pass_monotonic_ns\":" << MonotonicNanoseconds() << ",\n"
          << "  \"motion_calls\":0,\n"
          << "  \"mutating_calls\":0\n"
          << "}\n";
  const std::string content = payload.str();
  const bool written = WriteAll(file_descriptor, content);
  const bool synced = written && ::fsync(file_descriptor) == 0;
  const int close_result = ::close(file_descriptor);
  if (!synced || close_result != 0) {
    error = "evidence_write_or_fsync_failed";
    return false;
  }
  return true;
}

int RunGate(const std::string& network_interface, const GateConfig& config)
{
  std::cout << "CONTROL_PLANE_DIAG event=PROBE_START interface="
            << network_interface << " topic=" << kSportModeStateTopic
            << " timeout_sec=" << config.timeout_sec
            << " required_frames=" << config.required_frames
            << " max_frame_gap_ms=" << config.max_frame_gap_ms << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  SportStateMonitor monitor;
  const auto start_time = std::chrono::steady_clock::now();
  bool first_dds_state_logged = false;
  while (std::chrono::steady_clock::now() - start_time <
         std::chrono::duration<double>(config.timeout_sec)) {
    const MonitorStatistics statistics = monitor.GetStatistics();
    if (statistics.raw_frames > 0 && !first_dds_state_logged) {
      std::cout << "CONTROL_PLANE_DIAG event=FIRST_DDS_STATE"
                << " monotonic_sec=" << MonotonicSeconds()
                << " raw_frames=" << statistics.raw_frames << std::endl;
      first_dds_state_logged = true;
    }
    if (statistics.valid_frames >= config.required_frames &&
        statistics.invalid_frames == 0 &&
        statistics.max_frame_gap_ms <= config.max_frame_gap_ms) {
      std::cout << "CONTROL_PLANE_DIAG event=FIRST_STABLE_DDS_STATE"
                << " monotonic_sec=" << MonotonicSeconds()
                << " valid_frames=" << statistics.valid_frames << std::endl;
      std::cout << "CONTROL_PLANE_DIAG classification=SUCCESS"
                << " raw_frames=" << statistics.raw_frames
                << " valid_frames=" << statistics.valid_frames
                << " max_frame_gap_ms=" << statistics.max_frame_gap_ms << std::endl;
      std::string evidence_error;
      if (!WritePassEvidence(config, statistics, evidence_error)) {
        std::cerr << "CONTROL_PLANE_DIAG classification=EVIDENCE_WRITE_FAILED"
                  << " detail=" << evidence_error << std::endl;
        return 1;
      }
      std::cout << "CONTROL_PLANE_DIAG event=PASS_EVIDENCE_FSYNCED"
                << " probe_instance_id=" << config.probe_instance_id
                << " path=" << config.pass_evidence_path << std::endl;
      if (config.controlled_terminal_success) {
        // 仅独立只读 probe 的成功路径允许绕过已证实不稳定的 SDK/DDS 静态
        // 析构。它已 fsync 本次证据，且没有 SportClient 或任何 mutation。
        std::cout << "CONTROL_PLANE_DIAG event=CONTROLLED_TERMINAL_SUCCESS"
                  << std::endl;
        std::cout.flush();
        std::_Exit(0);
      }
      // 优先正常关闭：reader 先断开并等待 callback，再释放 factory。若底层
      // 仍 abort，父门禁会看到非零 RC，即使 observation evidence 已存在也拒绝。
      if (!monitor.Shutdown()) {
        std::cerr << "CONTROL_PLANE_DIAG classification=TEARDOWN_CALLBACK_TIMEOUT"
                  << std::endl;
        return 1;
      }
      unitree::robot::ChannelFactory::Instance()->Release();
      std::cout << "CONTROL_PLANE_DIAG event=NORMAL_TEARDOWN_COMPLETE"
                << std::endl;
      return 0;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  const MonitorStatistics statistics = monitor.GetStatistics();
  std::string classification = "ROBOT_STATE_STREAM_NOT_FRESH";
  if (statistics.raw_frames == 0) {
    classification = "ROBOT_DDS_NOT_DISCOVERED";
  } else if (statistics.invalid_frames > 0) {
    classification = "ROBOT_STATE_STREAM_FORMAT_INVALID";
  }
  std::cerr << "CONTROL_PLANE_DIAG classification=" << classification
            << " raw_frames=" << statistics.raw_frames
            << " valid_frames=" << statistics.valid_frames
            << " invalid_frames=" << statistics.invalid_frames
            << " max_frame_gap_ms=" << statistics.max_frame_gap_ms << std::endl;
  monitor.Shutdown();
  unitree::robot::ChannelFactory::Instance()->Release();
  return 1;
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    PrintUsage(argv[0]);
    return 2;
  }

  try {
    const std::string network_interface = argv[1];
    if (argc >= 3 && std::string(argv[2]) == "--gate") {
      return RunGate(network_interface, ParseGateArguments(argc, argv));
    }
    if (argc >= 3 && std::string(argv[2]) == "--calibration-stream") {
      return RunCalibrationStream(
          network_interface, ParseCalibrationArguments(argc, argv));
    }
    const double duration_sec =
        argc >= 3 ? ParsePositiveDouble(argv[2], "duration_sec")
                  : kDefaultDurationSec;
    const double print_rate_hz =
        argc >= 4 ? ParsePositiveDouble(argv[3], "print_rate_hz")
                  : kDefaultPrintRateHz;

    std::cout << "READ_ONLY: subscribing to " << kSportModeStateTopic
              << " on interface " << network_interface << std::endl;
    unitree::robot::ChannelFactory::Instance()->Init(
        0, network_interface);
    SportStateMonitor monitor;

    const auto start_time = std::chrono::steady_clock::now();
    const auto end_time = start_time + std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(duration_sec));
    const auto print_period = std::chrono::duration_cast<
        std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / print_rate_hz));
    auto next_print = start_time;
    bool printed = false;

    while (std::chrono::steady_clock::now() < end_time) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= next_print) {
        unitree_go::msg::dds_::SportModeState_ state;
        if (monitor.Snapshot(state)) {
          PrintState(state);
          printed = true;
        }
        next_print = now + print_period;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!printed) {
      std::cerr << "No SportModeState received within "
                << duration_sec << "s" << std::endl;
      return 1;
    }
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
