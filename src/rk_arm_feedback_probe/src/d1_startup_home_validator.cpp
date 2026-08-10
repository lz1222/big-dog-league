#include "rk_arm/d1_feedback_parser.hpp"
#include "rk_arm_feedback_probe/startup_home_validation.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>

#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

namespace {

constexpr const char* kArmFeedbackTopic = "rt/arm_Feedback";
constexpr const char* kServoAngleTopic = "current_servo_angle";
std::atomic<bool> g_exit_requested{false};

/** 只请求正常关闭，让主线程析构 DDS reader；信号处理器不进行任何通信动作。 */
void HandleSignal(int) { g_exit_requested.store(true); }

struct Arguments {
  std::string interface_name{"eth0"};
  std::filesystem::path output_dir{"artifacts/startup_home_pose"};
  int duration_sec{60};
};

bool ParsePositiveInt(const char* text, int* output) {
  try {
    const int value = std::stoi(text);
    if (value <= 0) return false;
    *output = value;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseArguments(int argc, char** argv, Arguments* arguments) {
  for (int index = 1; index < argc; ++index) {
    const std::string option(argv[index]);
    if (option == "--interface" && index + 1 < argc) {
      arguments->interface_name = argv[++index];
    } else if (option == "--output-dir" && index + 1 < argc) {
      arguments->output_dir = argv[++index];
    } else if (option == "--duration-sec" && index + 1 < argc) {
      if (!ParsePositiveInt(argv[++index], &arguments->duration_sec)) return false;
    } else {
      return false;
    }
  }
  return arguments->duration_sec >= 30;
}

void PrintUsage(const char* executable) {
  std::cerr << "Usage: " << executable
            << " [--interface IFACE] [--output-dir DIR] [--duration-sec N]\n"
               "N must be at least 30. This program creates exactly two DDS readers: "
               "rt/arm_Feedback and current_servo_angle.\n";
}

std::int64_t MonotonicNs() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::array<double, 7> CopyServoValues(const unitree_arm::msg::dds_::PubServoInfo_& message) {
  return {message.servo0_data_(), message.servo1_data_(), message.servo2_data_(),
          message.servo3_data_(), message.servo4_data_(), message.servo5_data_(),
          message.servo6_data_()};
}

std::string Number(double value) {
  std::ostringstream output;
  output << std::fixed << std::setprecision(6) << value;
  return output.str();
}

std::string Statistic(const rk_arm_feedback_probe::ChannelStatistics& stats, double value) {
  // 零帧不能被显示成数值零，否则会把“未观测”误报为“姿态恰好为零”。
  return stats.frames == 0 ? "N/A" : Number(value);
}

void WriteValues(std::ostream& output,
                 const std::optional<std::array<double, 7>>& values) {
  for (std::size_t index = 0; index < 7; ++index) {
    output << ',';
    if (values && std::isfinite((*values)[index])) output << Number((*values)[index]);
  }
}

/** 本地 CSV 记录的是接收快照，不会将任一源值回写到 DDS。 */
class SampleWriter {
 public:
  explicit SampleWriter(const std::filesystem::path& output_dir) : output_dir_(output_dir) {}

  bool Open(std::string* error) {
    std::error_code code;
    std::filesystem::create_directories(output_dir_, code);
    if (code) { *error = "cannot create output directory: " + code.message(); return false; }
    frames_.open(output_dir_ / "startup_home_pose_frames.csv", std::ios::trunc);
    if (!frames_) { *error = "cannot open frames CSV"; return false; }
    frames_ << "host_monotonic_ns,trigger,angle0,angle1,angle2,angle3,angle4,angle5,angle6,"
               "servo0,servo1,servo2,servo3,servo4,servo5,servo6,enable_status,power_status,error_status\n";
    return true;
  }

  void Write(std::int64_t timestamp_ns, const char* trigger,
             const std::optional<std::array<double, 7>>& angles,
             const std::optional<std::array<double, 7>>& servos,
             const std::optional<std::array<int, 3>>& status) {
    frames_ << timestamp_ns << ',' << trigger;
    WriteValues(frames_, angles);
    WriteValues(frames_, servos);
    for (std::size_t index = 0; index < 3; ++index) {
      frames_ << ',';
      if (status) frames_ << (*status)[index];
    }
    frames_ << '\n';
    frames_.flush();
  }

 private:
  std::filesystem::path output_dir_;
  std::ofstream frames_;
};

void WriteYaml(const std::filesystem::path& path,
               const rk_arm_feedback_probe::StartupHomeSummary& summary) {
  std::ofstream output(path, std::ios::trunc);
  output << "# 由 d1_startup_home_validator 只读采样生成；未接入任何控制链路。\n"
            "startup_home_pose:\n  value_unit: app_display_unit\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    output << "  angle" << channel << ": "
           << Number(rk_arm_feedback_probe::kStartupHomeTargets[channel]) << '\n';
  }
  output << "\nhome_tolerance:\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    output << "  angle" << channel << ": ";
    if (summary.angle_stats[channel].frames == 0) {
      output << "null # 未收到 rt/arm_Feedback 角度帧，禁止作为到位参数使用\n";
    } else {
      output << Number(summary.home_tolerance[channel]);
      if (summary.needs_manual_check[channel]) output << " # 需要人工检查";
      output << '\n';
    }
  }
}

void WriteReport(const std::filesystem::path& path,
                 double elapsed_sec,
                 const rk_arm_feedback_probe::StartupHomeSummary& summary,
                 std::size_t angle_frames,
                 std::size_t servo_frames) {
  std::ofstream output(path, std::ios::trunc);
  const double feedback_hz = elapsed_sec > 0.0 ? angle_frames / elapsed_sec : 0.0;
  const double servo_hz = elapsed_sec > 0.0 ? servo_frames / elapsed_sec : 0.0;
  output << "# STARTUP_HOME_POSE_VALIDATION_REPORT\n\n"
            "只读采样完成。value_unit 始终为 `app_display_unit`，未执行单位转换，未创建 command writer。\n\n"
            "- 采样窗口: " << Number(elapsed_sec) << " s\n"
            "- `rt/arm_Feedback` 角度帧: " << angle_frames << "，频率: " << Number(feedback_hz) << " Hz\n"
            "- `current_servo_angle` 帧: " << servo_frames << "，频率: " << Number(servo_hz) << " Hz\n\n"
            "## 目标值\n\n"
            "| channel | target |\n|---|---:|\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    output << "| angle" << channel << " | "
           << Number(rk_arm_feedback_probe::kStartupHomeTargets[channel]) << " |\n";
  }
  output << "\n## 实际统计（rt/arm_Feedback angle）\n\n"
            "| channel | target | mean | median | min | max | peak_to_peak | max_jump | 双源最大差值 |\n"
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    const auto& stats = summary.angle_stats[channel];
    output << "| angle" << channel << " | "
           << Number(rk_arm_feedback_probe::kStartupHomeTargets[channel]) << " | "
           << Statistic(stats, stats.mean) << " | " << Statistic(stats, stats.median) << " | "
           << Statistic(stats, stats.min) << " | " << Statistic(stats, stats.max) << " | "
           << Statistic(stats, stats.peak_to_peak) << " | " << Statistic(stats, stats.max_jump) << " | "
           << (summary.paired_frames == 0 ? "N/A" : Number(summary.max_source_difference[channel])) << " |\n";
  }
  output << "\n## current_servo_angle 统计\n\n"
            "| channel | mean | median | min | max | peak_to_peak | max_jump |\n"
            "|---|---:|---:|---:|---:|---:|---:|\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    const auto& stats = summary.servo_stats[channel];
    output << "| servo" << channel << " | " << Statistic(stats, stats.mean) << " | "
           << Statistic(stats, stats.median) << " | " << Statistic(stats, stats.min) << " | "
           << Statistic(stats, stats.max) << " | " << Statistic(stats, stats.peak_to_peak) << " | "
           << Statistic(stats, stats.max_jump) << " |\n";
  }
  output << "\n## 双源一致性\n\n"
            "- 有效配对帧: " << summary.paired_frames << "（两源接收时间差不超过 250 ms）\n"
            "- 最大配对时间差: " << Number(summary.max_pair_age_ms) << " ms\n"
            "- 一致性阈值: 0.500000 app_display_unit\n"
            "- 结论: " << (summary.source_consistent ? "一致" : "不一致或无有效配对") << "\n\n"
            "## 状态帧\n\n"
            "- 帧数: " << summary.status.frames << "；正常 `(enable_status,power_status,error_status)=(1,0,0)` 帧: "
            << summary.status.normal_frames << "；异常帧: " << summary.status.abnormal_frames << '\n';
  if (summary.status.last) {
    output << "- 最后一帧: enable_status=" << (*summary.status.last)[0]
           << "，power_status=" << (*summary.status.last)[1]
           << "，error_status=" << (*summary.status.last)[2] << "\n";
  } else {
    output << "- 最后一帧: 未收到 funcode=3 状态帧\n";
  }
  output << "\n## 推荐到位判断参数\n\n"
            "规则：`tolerance = min(max(peak_to_peak * 2, 0.2), 0.5)`；未截断前大于 0.5 的通道需要人工检查。\n\n"
            "| channel | recommended home_tolerance | 人工检查 |\n|---|---:|---|\n";
  for (std::size_t channel = 0; channel < 7; ++channel) {
    output << "| angle" << channel << " | "
           << Statistic(summary.angle_stats[channel], summary.home_tolerance[channel]) << " | "
           << (summary.angle_stats[channel].frames == 0 ? "未采到数据" :
               (summary.needs_manual_check[channel] ? "需要" : "否")) << " |\n";
  }
  output << "\n- 推荐连续确认帧数量: " << summary.recommended_confirmation_frames
           << "（至少 10 帧，且不少于约 0.5 秒的 `rt/arm_Feedback` 帧）\n"
            "- `ARM_HOME_CONFIRMED`: " << (summary.arm_home_confirmed ? "是" : "否") << "\n\n"
            "判定要求：两源均有数据；所有已观测状态帧正常；双源最大差值不超过 0.5；无人工检查通道；最近推荐数量的 angle 帧全部在对应 target ± home_tolerance 内。\n";
}

/** 仅保存接收快照并计算统计；此对象不包含 publisher、command topic 或 SDK 控制调用。 */
class Collector {
 public:
  explicit Collector(SampleWriter* writer) : writer_(writer) {}

  void RecordFeedback(const std::string& payload, std::int64_t timestamp_ns) {
    rk_arm::D1FeedbackFrame frame;
    if (!rk_arm::ParseD1Feedback(payload, &frame)) return;
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame.app_values) {
      angles_ = *frame.app_values;
      validator_.RecordAngles(*angles_, timestamp_ns);
      ++angle_frames_;
      writer_->Write(timestamp_ns, kArmFeedbackTopic, angles_, servos_, status_);
    } else if (frame.enable_status) {
      status_ = std::array<int, 3>{*frame.enable_status, *frame.power_status, *frame.error_status};
      validator_.RecordStatus((*status_)[0], (*status_)[1], (*status_)[2]);
      writer_->Write(timestamp_ns, "rt/arm_Feedback:status", angles_, servos_, status_);
    }
  }

  void RecordServos(const std::array<double, 7>& values, std::int64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(mutex_);
    servos_ = values;
    validator_.RecordServos(*servos_, timestamp_ns);
    ++servo_frames_;
    writer_->Write(timestamp_ns, kServoAngleTopic, angles_, servos_, status_);
  }

  rk_arm_feedback_probe::StartupHomeSummary Summarize(double feedback_hz) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return validator_.Summarize(feedback_hz);
  }
  std::size_t angle_frames() const { std::lock_guard<std::mutex> lock(mutex_); return angle_frames_; }
  std::size_t servo_frames() const { std::lock_guard<std::mutex> lock(mutex_); return servo_frames_; }

 private:
  SampleWriter* writer_;
  mutable std::mutex mutex_;
  rk_arm_feedback_probe::StartupHomeValidator validator_;
  std::optional<std::array<double, 7>> angles_;
  std::optional<std::array<double, 7>> servos_;
  std::optional<std::array<int, 3>> status_;
  std::size_t angle_frames_{0};
  std::size_t servo_frames_{0};
};

}  // namespace

int main(int argc, char** argv) {
  Arguments arguments;
  if (!ParseArguments(argc, argv, &arguments)) {
    PrintUsage(argv[0]);
    return 2;
  }
  SampleWriter writer(arguments.output_dir);
  std::string error;
  if (!writer.Open(&error)) {
    std::cerr << "Output setup failed: " << error << '\n';
    return 3;
  }
  Collector collector(&writer);
  std::signal(SIGINT, HandleSignal);
  std::signal(SIGTERM, HandleSignal);
  const auto started = std::chrono::steady_clock::now();
  try {
    // 本次任务的唯一 DDS 对象均为白名单 topic 的 reader；没有控制客户端或命令通道。
    unitree::robot::ChannelFactory::Instance()->Init(0, arguments.interface_name);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> arm_feedback(kArmFeedbackTopic);
    unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_> servo_angles(kServoAngleTopic);
    arm_feedback.InitChannel([&collector](const void* raw) {
      const auto* message = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw);
      collector.RecordFeedback(message->data_(), MonotonicNs());
    });
    servo_angles.InitChannel([&collector](const void* raw) {
      const auto* message = static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw);
      collector.RecordServos(CopyServoValues(*message), MonotonicNs());
    });
    std::cout << "STARTUP_HOME_POSE read-only sampling started for " << arguments.duration_sec
              << " seconds on " << kArmFeedbackTopic << " and " << kServoAngleTopic << ".\n";
    const auto deadline = started + std::chrono::seconds(arguments.duration_sec);
    while (!g_exit_requested.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    arm_feedback.CloseChannel();
    servo_angles.CloseChannel();
    unitree::robot::ChannelFactory::Instance()->Release();
  } catch (const std::exception& exception) {
    std::cerr << "DDS reader failure: " << exception.what() << '\n';
    return 4;
  }
  const double elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  const std::size_t angle_frames = collector.angle_frames();
  const std::size_t servo_frames = collector.servo_frames();
  const double feedback_hz = elapsed_sec > 0.0 ? static_cast<double>(angle_frames) / elapsed_sec : 0.0;
  const auto summary = collector.Summarize(feedback_hz);
  WriteYaml(arguments.output_dir / "startup_home_pose.yaml", summary);
  WriteReport(arguments.output_dir / "STARTUP_HOME_POSE_VALIDATION_REPORT.md", elapsed_sec,
              summary, angle_frames, servo_frames);
  std::cout << "Read-only sampling complete: angle_frames=" << angle_frames
            << " servo_frames=" << servo_frames
            << " ARM_HOME_CONFIRMED=" << (summary.arm_home_confirmed ? "true" : "false") << '\n'
            << "Report: " << (arguments.output_dir / "STARTUP_HOME_POSE_VALIDATION_REPORT.md") << '\n';
  return 0;
}
