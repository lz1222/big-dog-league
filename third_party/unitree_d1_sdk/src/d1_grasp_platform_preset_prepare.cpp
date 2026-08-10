// 抓取平台预设抓取准备动作
// 执行命令：cd ~/rk_inspection_ws && build/unitree_d1_sdk/d1_grasp_platform_preset_prepare eth1 --execute --power-on
// 默认不连接硬件；只有 --execute --power-on 同时存在时才会发送 DDS 指令。
// 数据方向为 rt/arm_Feedback（兼容 arm_Feedback）-> 本状态机 -> rt/arm_Command。

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr int kJointCount = 7;
// 2° 小段配合 10Hz 目标保持，降低长臂多关节同时变位时的顿挫。
constexpr double kMaxSegmentDeg = 2.0;
// 到位容差小于分段量，并留出总线舵机约 1° 的稳态残差，避免已到姿态被误判失败。
constexpr double kArrivalToleranceDeg = 1.5;
// 官方说明：funcode=5 的 mode 范围为 0~80000；0 完全卸力、80000 完全锁死。
constexpr auto kFeedbackTimeout = std::chrono::seconds(2);
constexpr auto kEnableTimeout = std::chrono::seconds(5);
constexpr auto kCommandReplyTimeout = std::chrono::seconds(12);
constexpr auto kSegmentTimeout = std::chrono::seconds(15);
constexpr auto kHoldDuration = std::chrono::seconds(3);
// 官方 App 对 mode=0 以约 10Hz 连续下发目标；单次命令可能无法保持到位。
constexpr auto kTargetKeepalivePeriod = std::chrono::milliseconds(100);
constexpr char kCommandTopic[] = "rt/arm_Command";
constexpr char kFeedbackTopic[] = "rt/arm_Feedback";
// 现场旧服务仍可能发布不带 rt/ 前缀的话题；仅作反馈兼容，命令仍严格发送官方 rt/arm_Command。
constexpr char kCompatibilityFeedbackTopic[] = "arm_Feedback";

using Pose = std::array<double, kJointCount>;

// 用户指定的 App 关节 1~6 对应协议 angle0~angle5，爪夹对应 angle6，单位均为度。
// 抓取平台动作一：用户确认的 6 关节与爪夹预设，单位为 App 显示角度。
constexpr Pose kPoseOne{0.0, -90.0, 90.0, 8.0, 0.0, 0.0, 25.0};
constexpr Pose kPoseTwo{-86.0, 50.0, 41.0, 13.0, -79.0, -1.0, 40.0};
// 一号平台放置动作三：由用户确认的 6 关节与爪夹目标，单位为 App 显示角度。
constexpr Pose kPoseThree{-86.0, 44.0, 59.0, 16.0, -90.0, 4.0, 25.0};
// 一号平台动作四：保持机械臂姿态，仅将爪夹张开至 49°。
constexpr Pose kPoseFour{-86.0, 44.0, 59.0, 16.0, -90.0, 4.0, 49.0};
// 一号平台动作五：在爪夹保持张开的前提下抬升，为后续安全归位留出空间。
constexpr Pose kPoseFive{-86.0, 28.0, 59.0, 16.0, -90.0, 4.0, 49.0};
// 二号平台动作六：从动作一移动到二号平台的放置姿态。
constexpr Pose kPoseSix{88.0, 44.0, 57.0, 16.0, -90.0, -5.0, 25.0};
// 二号平台动作七：保持放置姿态，仅将爪夹张开至 49°。
constexpr Pose kPoseSeven{88.0, 44.0, 57.0, 16.0, -90.0, -5.0, 49.0};
// 二号平台动作八：在爪夹保持张开的前提下抬升，为后续安全归位留出空间。
constexpr Pose kPoseEight{88.0, 28.0, 57.0, 16.0, -90.0, -5.0, 49.0};
// 中转平台动作九：由动作一进入中转平台的放置姿态。
constexpr Pose kPoseNine{66.0, 49.0, 44.0, 0.0, -70.0, -6.0, 25.0};
// 中转平台动作十：保持放置姿态，仅将爪夹张开至 49°。
constexpr Pose kPoseTen{66.0, 49.0, 44.0, 0.0, -70.0, -6.0, 49.0};
// 中转平台动作十一：在爪夹保持张开的前提下抬升。
constexpr Pose kPoseEleven{66.0, 32.0, 44.0, 0.0, -70.0, -6.0, 49.0};
// 中转平台动作十二：抬升后向左移动至交接姿态，数值由用户确认。
constexpr Pose kPoseTwelve{57.0, -7.0, 61.0, 21.0, -24.0, -11.0, 49.0};
constexpr std::array<std::pair<double, double>, 6> kJointLimits{{
    {-90.0, 90.0}, {-90.0, 90.0}, {-135.0, 135.0},
    {-90.0, 90.0},
    // 现场确认 angle4 在约 -88.6° 已停止跟随，按 ±90° 设软件限位并留给轨迹残差处理。
    {-90.0, 90.0},
    {-40.0, 20.0},
}};

struct FeedbackState {
  std::optional<Pose> angles;
  std::optional<int> enable_status;
  std::optional<int> power_status;
  std::optional<int> error_status;
  std::optional<std::int64_t> reply_seq;
  std::optional<int> recv_status;
  std::optional<int> exec_status;
  std::chrono::steady_clock::time_point angle_time{};
  std::chrono::steady_clock::time_point status_time{};
  std::string last_raw;
};

std::mutex g_feedback_mutex;
FeedbackState g_feedback;
std::atomic<bool> g_stop_requested{false};
// 官方 App 使用递增 seq；其用于回执关联，因此不能复用示例中的固定值 4。
std::atomic<std::int64_t> g_next_sequence{
    static_cast<std::int64_t>(std::chrono::steady_clock::now().time_since_epoch().count() % 1000000000)};

void SignalHandler(int) { g_stop_requested.store(true); }

// 官方反馈为 JSON 字符串。此处只抽取已知数值字段，字段缺失即视为无效反馈，避免按零值运动。
std::optional<double> JsonNumber(const std::string& text, const std::string& key) {
  const std::regex pattern("\\\"" + key + "\\\"\\s*:\\s*(-?(?:[0-9]+(?:\\.[0-9]*)?|\\.[0-9]+)(?:[eE][+-]?[0-9]+)?)");
  std::smatch match;
  if (!std::regex_search(text, match, pattern)) return std::nullopt;
  try {
    return std::stod(match[1].str());
  } catch (...) {
    return std::nullopt;
  }
}

void FeedbackHandler(const void* raw_message) {
  const auto* message = static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw_message);
  const std::string text = message->data_();
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(g_feedback_mutex);
  g_feedback.last_raw = text;

  Pose angles{};
  bool all_angles_present = true;
  for (int index = 0; index < kJointCount; ++index) {
    const auto value = JsonNumber(text, "angle" + std::to_string(index));
    if (!value) {
      all_angles_present = false;
      break;
    }
    angles[index] = *value;
  }
  if (all_angles_present) {
    g_feedback.angles = angles;
    g_feedback.angle_time = now;
  }

  const auto enable = JsonNumber(text, "enable_status");
  const auto power = JsonNumber(text, "power_status");
  const auto error = JsonNumber(text, "error_status");
  if (enable || power || error) {
    if (enable) g_feedback.enable_status = static_cast<int>(*enable);
    if (power) g_feedback.power_status = static_cast<int>(*power);
    if (error) g_feedback.error_status = static_cast<int>(*error);
    g_feedback.status_time = now;
  }

  const auto recv = JsonNumber(text, "recv_status");
  const auto exec = JsonNumber(text, "exec_status");
  if (recv || exec) {
    const auto sequence = JsonNumber(text, "seq");
    if (sequence) g_feedback.reply_seq = static_cast<std::int64_t>(*sequence);
    if (recv) g_feedback.recv_status = static_cast<int>(*recv);
    if (exec) g_feedback.exec_status = static_cast<int>(*exec);
  }
}

FeedbackState SnapshotFeedback() {
  std::lock_guard<std::mutex> lock(g_feedback_mutex);
  return g_feedback;
}

bool IsFresh(const std::chrono::steady_clock::time_point& timestamp,
             std::chrono::seconds timeout) {
  return timestamp.time_since_epoch().count() != 0 &&
         std::chrono::steady_clock::now() - timestamp <= timeout;
}

void PrintPose(const std::string& label, const Pose& pose) {
  std::cout << label << " [";
  for (int index = 0; index < kJointCount; ++index) {
    if (index) std::cout << ", ";
    std::cout << std::fixed << std::setprecision(2) << pose[index];
  }
  std::cout << "] deg" << std::endl;
}

// 超时时输出最大偏差关节，便于区分真实机械阻挡与正常的舵机稳态残差。
void PrintPoseError(const Pose& actual, const Pose& target) {
  int worst_index = 0;
  double worst_error = 0.0;
  for (int index = 0; index < kJointCount; ++index) {
    const double error = std::abs(actual[index] - target[index]);
    if (error > worst_error) {
      worst_error = error;
      worst_index = index;
    }
  }
  PrintPose("超时实际反馈", actual);
  std::cerr << "最大偏差：angle" << worst_index << " = " << std::fixed
            << std::setprecision(2) << worst_error << " 度。" << std::endl;
}

bool ValidatePose(const Pose& pose, const std::string& name) {
  for (int index = 0; index < 6; ++index) {
    const auto [lower, upper] = kJointLimits[index];
    if (pose[index] < lower || pose[index] > upper) {
      std::cerr << name << " 的 angle" << index << " 超出关节限制。" << std::endl;
      return false;
    }
  }
  // 文档未给出爪夹 angle6 的范围，因此不臆设限位；实际值仍由每段 2° 限制。
  return true;
}

std::int64_t NextSequence() { return g_next_sequence.fetch_add(1); }

std::string BuildCommand(std::int64_t sequence, int funcode, const std::string& data) {
  std::ostringstream json;
  json << "{\"seq\":" << sequence << ",\"address\":1,\"funcode\":" << funcode
       << ",\"data\":" << data << "}";
  return json.str();
}

std::string EnableData(bool enable) {
  // 已抓取的官方 App 成功使用 mode=1 使能、mode=0 卸力；不用未现场验证的 80000。
  return std::string("{\"mode\":") + (enable ? "1" : "0") + "}";
}

std::string PowerData(bool power_on) {
  // 官方 funcode=6：power=1 上电，power=0 断电。本程序仅在明确授权时发送上电。
  return std::string("{\"power\":") + (power_on ? "1" : "0") + "}";
}

std::string PoseData(const Pose& pose) {
  std::ostringstream data;
  // 官方 App 的连续小步关节控制实际使用 mode=0；本程序额外把每一段各通道变化限为 2°。
  data << "{\"mode\":0";
  for (int index = 0; index < kJointCount; ++index) {
    data << ",\"angle" << index << "\":" << std::fixed << std::setprecision(3)
         << pose[index];
  }
  data << "}";
  return data.str();
}

std::int64_t PublishCommand(
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher,
    int funcode, const std::string& data, bool print_command = true) {
  const std::int64_t sequence = NextSequence();
  const auto payload = BuildCommand(sequence, funcode, data);
  if (print_command) std::cout << "SEND " << kCommandTopic << ": " << payload << std::endl;
  unitree_arm::msg::dds_::ArmString_ message{};
  message.data_() = payload;
  publisher.Write(message);
  return sequence;
}

bool WaitForAnyStatus() {
  const auto deadline = std::chrono::steady_clock::now() + kEnableTimeout;
  while (std::chrono::steady_clock::now() < deadline && !g_stop_requested.load()) {
    const auto feedback = SnapshotFeedback();
    if (IsFresh(feedback.status_time, kFeedbackTimeout)) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return false;
}

bool WaitForCommandSuccess(std::int64_t sequence, std::chrono::seconds timeout,
                           const std::string& label) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline && !g_stop_requested.load()) {
    const auto feedback = SnapshotFeedback();
    if (feedback.reply_seq && *feedback.reply_seq == sequence) {
      if (feedback.recv_status && *feedback.recv_status != 1) {
        std::cerr << label << " 被服务拒收。" << std::endl;
        return false;
      }
      if (feedback.exec_status && *feedback.exec_status != 1) {
        std::cerr << label << " 被服务拒绝执行。" << std::endl;
        return false;
      }
      if (feedback.recv_status && feedback.exec_status) {
        std::cout << label << " 已确认：recv_status=1, exec_status=1。" << std::endl;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  const auto feedback = SnapshotFeedback();
  std::cerr << label << " 未收到 recv_status=1 且 exec_status=1 的回执";
  if (!feedback.last_raw.empty()) std::cerr << "；最后反馈=" << feedback.last_raw;
  std::cerr << std::endl;
  return false;
}

std::optional<Pose> WaitForCurrentPose() {
  const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
  while (std::chrono::steady_clock::now() < deadline && !g_stop_requested.load()) {
    const auto feedback = SnapshotFeedback();
    if (feedback.angles && IsFresh(feedback.angle_time, kFeedbackTimeout)) {
      return feedback.angles;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return std::nullopt;
}

bool Reached(const Pose& actual, const Pose& target) {
  for (int index = 0; index < kJointCount; ++index) {
    if (std::abs(actual[index] - target[index]) > kArrivalToleranceDeg) return false;
  }
  return true;
}

std::vector<Pose> BuildSegments(const Pose& start, const Pose& target) {
  double largest_delta = 0.0;
  for (int index = 0; index < kJointCount; ++index) {
    largest_delta = std::max(largest_delta, std::abs(target[index] - start[index]));
  }
  const int count = std::max(1, static_cast<int>(std::ceil(largest_delta / kMaxSegmentDeg)));
  std::vector<Pose> result;
  result.reserve(count);
  for (int step = 1; step <= count; ++step) {
    const double ratio = static_cast<double>(step) / count;
    Pose point{};
    for (int index = 0; index < kJointCount; ++index) {
      point[index] = start[index] + (target[index] - start[index]) * ratio;
    }
    result.push_back(point);
  }
  return result;
}

bool MoveBySegments(unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>& publisher,
                    const Pose& start, const Pose& target, const std::string& phase) {
  const auto segments = BuildSegments(start, target);
  std::cout << phase << "：共 " << segments.size() << " 段，每通道最大变化 "
            << kMaxSegmentDeg << " 度。" << std::endl;

  // mode=0 是 App 的 10Hz 小步控制模式。必须持续发送逐步变化的目标，不能在每个
  // 中间点静止等待，否则控制器可能将重复目标视为无新的轨迹输入而不继续跟随。
  auto next_send_time = std::chrono::steady_clock::now();
  for (size_t index = 0; index < segments.size(); ++index) {
    // 使用绝对单调时间调度，避免上一帧的格式化/终端输出累积成控制周期抖动。
    std::this_thread::sleep_until(next_send_time);
    if (g_stop_requested.load()) return false;
    const auto feedback_before = SnapshotFeedback();
    // 官方 App 的实际记录中，mode=1 的回执成功后，enable_status 可能仍短暂为 0，
    // 直到第一条 funcode=2 才更新。因此安全判据改为新鲜的七路位置反馈和执行回执。
    if (!feedback_before.angles || !IsFresh(feedback_before.angle_time, kFeedbackTimeout)) {
      std::cerr << "运动前七路角度反馈超时；停止后卸力。" << std::endl;
      return false;
    }
    // 中段完整日志会阻塞终端刷新并破坏 10Hz 节奏，故只记录首段、每十段和末段。
    const bool report_progress = index == 0 || index + 1 == segments.size() ||
                                 (index + 1) % 10 == 0;
    if (report_progress) {
      PrintPose("第 " + std::to_string(index + 1) + "/" +
                    std::to_string(segments.size()) + " 段目标",
                segments[index]);
    }
    PublishCommand(publisher, 2, PoseData(segments[index]), report_progress);

    next_send_time += kTargetKeepalivePeriod;
    // 若系统调度曾明显落后，重置下一周期以防止连发多条命令造成新的顿挫。
    if (std::chrono::steady_clock::now() > next_send_time + kTargetKeepalivePeriod) {
      next_send_time = std::chrono::steady_clock::now() + kTargetKeepalivePeriod;
    }
  }

  // 轨迹末点保持发送，直到反馈进入容差范围；最终命令回执用于确认服务已接收末点。
  std::cout << phase << "：轨迹点已发送完毕，等待最终到位。" << std::endl;
  auto final_sequence = PublishCommand(publisher, 2, PoseData(target), false);
  auto next_keepalive = std::chrono::steady_clock::now() + kTargetKeepalivePeriod;
  const auto deadline = std::chrono::steady_clock::now() + kSegmentTimeout;
  while (std::chrono::steady_clock::now() < deadline && !g_stop_requested.load()) {
    const auto feedback = SnapshotFeedback();
    if (!feedback.angles || !IsFresh(feedback.angle_time, kFeedbackTimeout)) {
      std::cerr << "等待最终到位时七路角度反馈超时；停止后卸力。" << std::endl;
      return false;
    }
    if (Reached(*feedback.angles, target)) {
      return WaitForCommandSuccess(final_sequence, kCommandReplyTimeout, phase + " 最终目标");
    }
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_keepalive) {
      final_sequence = PublishCommand(publisher, 2, PoseData(target), false);
      next_keepalive = now + kTargetKeepalivePeriod;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  const auto final_feedback = SnapshotFeedback();
  if (final_feedback.angles) PrintPoseError(*final_feedback.angles, target);
  std::cerr << phase << " 在 " << kSegmentTimeout.count()
            << " 秒内未到最终目标；停止后卸力。" << std::endl;
  return false;
}

void PrintUsage(const char* program) {
  std::cout << "用法: " << program << " [network_interface] [--execute --power-on]\n"
            << "默认只打印安全计划，不会初始化 DDS 或控制机械臂。\n"
            << "真实执行示例: " << program << " eth1 --execute --power-on\n"
            << "真实执行必须显式同时给出 --execute --power-on；两者缺一不可。\n"
            << "--power-on 会发送 funcode=6/power=1；后续是否运动以 funcode=5/2 的官方回执为准。\n"
            << "执行中 Ctrl-C、反馈超时、失能或故障都会发送 funcode=5/mode=0 卸力。\n";
}

}  // namespace

int main(int argc, char** argv) {
  bool execute = false;
  bool power_on = false;
  std::string network_interface = "eth0";
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--execute") execute = true;
    else if (argument == "--power-on") power_on = true;
    else if (argument == "--help" || argument == "-h") {
      PrintUsage(argv[0]);
      return 0;
    } else if (argument.rfind("--", 0) == 0) {
      std::cerr << "未知参数: " << argument << std::endl;
      return 2;
    } else {
      network_interface = argument;
    }
  }

  if (!ValidatePose(kPoseOne, "位置一")) return 2;
  PrintPose("位置一", kPoseOne);
#ifdef D1_RETURN_HOME_ONLY
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段归位到位置一 -> 确认保持。"
            << std::endl;
#elif defined(D1_RESET_TO_POSE_ONE_ONLY)
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 从当前任意姿态 2° 分段置位到动作一 -> "
            << "确认保持。" << std::endl;
#elif defined(D1_PLATFORM1_PLACE_ONLY)
  if (!ValidatePose(kPoseThree, "动作三")) return 2;
  PrintPose("动作三（一号平台放置）", kPoseThree);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作一 -> "
            << "确认保持 -> 2° 分段放置到动作三 -> 确认保持。" << std::endl;
#elif defined(D1_PLATFORM1_GRIPPER_OPEN_ONLY)
  if (!ValidatePose(kPoseThree, "动作三") || !ValidatePose(kPoseFour, "动作四")) return 2;
  PrintPose("动作三（一号平台放置）", kPoseThree);
  PrintPose("动作四（一号平台夹爪张开）", kPoseFour);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作三 -> "
            << "确认保持 -> 2° 分段张开爪夹至动作四 -> 确认保持。" << std::endl;
#elif defined(D1_PLATFORM1_LIFT_ONLY)
  if (!ValidatePose(kPoseFour, "动作四") || !ValidatePose(kPoseFive, "动作五")) return 2;
  PrintPose("动作四（一号平台夹爪张开）", kPoseFour);
  PrintPose("动作五（一号平台抬升）", kPoseFive);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作四 -> "
            << "确认保持 -> 2° 分段抬升至动作五 -> 确认保持。" << std::endl;
#elif defined(D1_PLATFORM1_RETURN_HOME_ONLY)
  if (!ValidatePose(kPoseFive, "动作五")) return 2;
  PrintPose("动作五（一号平台抬升）", kPoseFive);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作五 -> "
            << "确认保持 -> 2° 分段归位至动作一 -> 确认保持。" << std::endl;
#elif defined(D1_PLATFORM1_COMPLETE_ACTION)
  if (!ValidatePose(kPoseThree, "动作三") || !ValidatePose(kPoseFour, "动作四") ||
      !ValidatePose(kPoseFive, "动作五")) {
    return 2;
  }
  PrintPose("动作三（一号平台放置）", kPoseThree);
  PrintPose("动作四（一号平台夹爪张开）", kPoseFour);
  PrintPose("动作五（一号平台抬升）", kPoseFive);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作一 -> "
            << "放置至动作三 -> 等待 2 秒 -> 张开至动作四 -> 等待 2 秒 -> 抬升至动作五 -> "
            << "归位至动作一；每段均反馈确认。" << std::endl;
#elif defined(D1_PLATFORM2_COMPLETE_ACTION)
  if (!ValidatePose(kPoseSix, "动作六") || !ValidatePose(kPoseSeven, "动作七") ||
      !ValidatePose(kPoseEight, "动作八")) {
    return 2;
  }
  PrintPose("动作六（二号平台放置）", kPoseSix);
  PrintPose("动作七（二号平台夹爪张开）", kPoseSeven);
  PrintPose("动作八（二号平台抬升）", kPoseEight);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作一 -> "
            << "放置至动作六 -> 等待 2 秒 -> 张开至动作七 -> 等待 2 秒 -> 抬升至动作八 -> "
            << "归位至动作一；每段均反馈确认。" << std::endl;
#elif defined(D1_TRANSFER_PLATFORM_COMPLETE_PLACE_ACTION)
  if (!ValidatePose(kPoseNine, "动作九") || !ValidatePose(kPoseTen, "动作十") ||
      !ValidatePose(kPoseEleven, "动作十一") || !ValidatePose(kPoseTwelve, "动作十二")) {
    return 2;
  }
  PrintPose("动作九（中转平台放置）", kPoseNine);
  PrintPose("动作十（中转平台夹爪张开）", kPoseTen);
  PrintPose("动作十一（中转平台抬升）", kPoseEleven);
  PrintPose("动作十二（中转平台左移）", kPoseTwelve);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段对齐动作一 -> "
            << "放置至动作九 -> 等待 2 秒 -> 张开至动作十 -> 等待 2 秒 -> 抬升至动作十一 -> "
            << "左移至动作十二；每段均反馈确认。" << std::endl;
#else
  if (!ValidatePose(kPoseTwo, "位置二")) return 2;
  PrintPose("位置二", kPoseTwo);
  std::cout << "流程：上电请求 -> 使能 -> 读取七路反馈 -> 2° 分段到位置一 -> 确认保持 -> "
            << "2° 分段到位置二 -> 确认保持。" << std::endl;
#endif
  if (!execute) {
    if (power_on) {
      std::cerr << "--power-on 必须与 --execute 同时使用；演练模式不会上电。" << std::endl;
      return 2;
    }
    std::cout << "DRY_RUN：未发送任何 DDS 命令；加入 --execute 后才会操作机械臂。" << std::endl;
    return 0;
  }
  if (!power_on) {
    std::cerr << "真实执行必须显式传入 --power-on，拒绝上电或移动。" << std::endl;
    return 2;
  }

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
  unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_> publisher(kCommandTopic);
  publisher.InitChannel();
  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> subscriber(kFeedbackTopic);
  subscriber.InitChannel(FeedbackHandler);
  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_> compatibility_subscriber(
      kCompatibilityFeedbackTopic);
  compatibility_subscriber.InitChannel(FeedbackHandler);

  // 给 10Hz 官方反馈留出首帧时间；未收到状态不能贸然发送任何命令。
  std::cout << "等待 " << kFeedbackTopic << " 的状态反馈..." << std::endl;
  if (!WaitForAnyStatus()) {
    std::cerr << "未收到状态反馈；未发送任何控制指令。" << std::endl;
    return 1;
  }

  const auto initial_status = SnapshotFeedback();
  const bool already_enabled = initial_status.enable_status && *initial_status.enable_status == 1;
  if (already_enabled) {
    // 已使能时重复上电/使能可能造成不必要的状态切换，直接使用当前保持姿态。
    std::cout << "检测到当前已上电使能；跳过上电与使能命令。" << std::endl;
  } else {
    // 历史官方 App 的成功运动记录中 power_status 仍为 0，故该字段不能作为
    // 控制前置条件；这里保留用户授权的 funcode=6 请求，但以命令回执为诊断依据。
    const auto power_sequence = PublishCommand(publisher, 6, PowerData(true));
    if (!WaitForCommandSuccess(power_sequence, std::chrono::seconds(2), "上电请求")) {
      std::cerr << "警告：上电请求没有回执；继续按官方 App 已验证的使能流程尝试。" << std::endl;
    }

    const auto enable_sequence = PublishCommand(publisher, 5, EnableData(true));
    if (!WaitForCommandSuccess(enable_sequence, kCommandReplyTimeout, "使能")) {
      std::cerr << "使能命令未获确认；发送卸力并退出。" << std::endl;
      PublishCommand(publisher, 5, EnableData(false));
      return 1;
    }
  }
  const auto current_pose = WaitForCurrentPose();
  if (!current_pose) {
    std::cerr << "使能后未收到七路新鲜反馈；发送卸力并退出。" << std::endl;
    PublishCommand(publisher, 5, EnableData(false));
    return 1;
  }
  PrintPose(already_enabled ? "当前已使能反馈" : "使能后当前反馈", *current_pose);

#ifdef D1_RETURN_HOME_ONLY
  // 归位的首动作点是动作二。仅在“已使能且偏离动作二”时先对齐动作二，
  // 再执行归位，保证当前任务的起点与后续轨迹一致。
  bool completed = true;
  Pose return_start = *current_pose;
  if (already_enabled && !Reached(return_start, kPoseTwo)) {
    completed = MoveBySegments(publisher, return_start, kPoseTwo,
                               "当前已使能，先对齐动作二");
    if (completed) return_start = kPoseTwo;
  }
  if (completed) {
    completed = MoveBySegments(publisher, return_start, kPoseOne,
                               "由动作二归位至位置一");
  }
#elif defined(D1_RESET_TO_POSE_ONE_ONLY)
  // 置位动作以最新反馈为轨迹起点，确保机械臂从任意已知姿态均回到动作一。
  bool completed = true;
  if (!Reached(*current_pose, kPoseOne)) {
    completed = MoveBySegments(publisher, *current_pose, kPoseOne,
                               "置位：由当前姿态移动至动作一");
  } else {
    std::cout << "当前已位于动作一，跳过置位轨迹。" << std::endl;
  }
#elif defined(D1_PLATFORM1_GRIPPER_OPEN_ONLY)
  // 夹爪张开的首动作点为动作三；避免从任意已使能姿态直接张开爪夹。
  bool completed = true;
  if (!Reached(*current_pose, kPoseThree)) {
    completed = MoveBySegments(publisher, *current_pose, kPoseThree,
                               "当前状态先对齐动作三");
  } else {
    std::cout << "当前已位于动作三，跳过首点对齐。" << std::endl;
  }
#elif defined(D1_PLATFORM1_LIFT_ONLY)
  // 抬升的首动作点为动作四；先完成张开姿态对齐，避免负载未释放时直接抬升。
  bool completed = true;
  if (!Reached(*current_pose, kPoseFour)) {
    completed = MoveBySegments(publisher, *current_pose, kPoseFour,
                               "当前状态先对齐动作四");
  } else {
    std::cout << "当前已位于动作四，跳过首点对齐。" << std::endl;
  }
#elif defined(D1_PLATFORM1_RETURN_HOME_ONLY)
  // 一号平台归位从动作五开始，先对齐抬升姿态再回动作一，避免低位横向回撤。
  bool completed = true;
  if (!Reached(*current_pose, kPoseFive)) {
    completed = MoveBySegments(publisher, *current_pose, kPoseFive,
                               "当前状态先对齐动作五");
  } else {
    std::cout << "当前已位于动作五，跳过首点对齐。" << std::endl;
  }
#else
  // 准备动作的首动作点是动作一。未使能时必先到动作一；已使能但偏离动作一
  // 时也先对齐，避免直接由任意保持姿态进入动作二。
  bool completed = true;
  if (!Reached(*current_pose, kPoseOne)) {
    const std::string pose_one_phase = already_enabled
        ? "当前已使能，先对齐动作一"
        : "上电使能后先移动至动作一";
    completed = MoveBySegments(publisher, *current_pose, kPoseOne, pose_one_phase);
  } else {
    std::cout << "当前已位于动作一，跳过首点对齐。" << std::endl;
  }
#endif
  if (completed) {
#if defined(D1_PLATFORM1_GRIPPER_OPEN_ONLY)
    std::cout << "动作三已到位，保持 " << kHoldDuration.count() << " 秒并复核反馈。" << std::endl;
    std::this_thread::sleep_for(kHoldDuration);
    const auto pose_three_feedback = WaitForCurrentPose();
    completed = pose_three_feedback && Reached(*pose_three_feedback, kPoseThree);
    if (!completed) std::cerr << "动作三保持复核失败。" << std::endl;
#elif defined(D1_PLATFORM1_LIFT_ONLY)
    std::cout << "动作四已到位，保持 " << kHoldDuration.count() << " 秒并复核反馈。" << std::endl;
    std::this_thread::sleep_for(kHoldDuration);
    const auto pose_four_feedback = WaitForCurrentPose();
    completed = pose_four_feedback && Reached(*pose_four_feedback, kPoseFour);
    if (!completed) std::cerr << "动作四保持复核失败。" << std::endl;
#elif defined(D1_PLATFORM1_RETURN_HOME_ONLY)
    std::cout << "动作五已到位，保持 " << kHoldDuration.count() << " 秒并复核反馈。" << std::endl;
    std::this_thread::sleep_for(kHoldDuration);
    const auto pose_five_feedback = WaitForCurrentPose();
    completed = pose_five_feedback && Reached(*pose_five_feedback, kPoseFive);
    if (!completed) std::cerr << "动作五保持复核失败。" << std::endl;
#else
    std::cout << "位置一已到位，保持 " << kHoldDuration.count() << " 秒并复核反馈。" << std::endl;
    std::this_thread::sleep_for(kHoldDuration);
    const auto pose_one_feedback = WaitForCurrentPose();
    completed = pose_one_feedback && Reached(*pose_one_feedback, kPoseOne);
    if (!completed) std::cerr << "位置一保持复核失败。" << std::endl;
#endif
  }
  if (completed) {
#ifdef D1_RETURN_HOME_ONLY
    std::cout << "位置一已确认归位：保持使能，不发送卸力命令。" << std::endl;
#elif defined(D1_RESET_TO_POSE_ONE_ONLY)
    std::cout << "置位动作已完成，动作一已确认：保持使能，不发送卸力命令。" << std::endl;
#elif defined(D1_PLATFORM1_GRIPPER_OPEN_ONLY)
    completed = MoveBySegments(publisher, kPoseThree, kPoseFour,
                               "由动作三张开爪夹至动作四（一号平台）");
    if (completed) {
      const auto pose_four_feedback = WaitForCurrentPose();
      completed = pose_four_feedback && Reached(*pose_four_feedback, kPoseFour);
    }
    if (completed) std::cout << "动作四已确认到位：保持使能，不发送卸力命令。" << std::endl;
#elif defined(D1_PLATFORM1_LIFT_ONLY)
    completed = MoveBySegments(publisher, kPoseFour, kPoseFive,
                               "由动作四抬升至动作五（一号平台）");
    if (completed) {
      const auto pose_five_feedback = WaitForCurrentPose();
      completed = pose_five_feedback && Reached(*pose_five_feedback, kPoseFive);
    }
    if (completed) std::cout << "动作五已确认到位：保持使能，不发送卸力命令。" << std::endl;
#elif defined(D1_PLATFORM1_RETURN_HOME_ONLY)
    completed = MoveBySegments(publisher, kPoseFive, kPoseOne,
                               "由动作五归位至动作一（一号平台）");
    if (completed) {
      const auto pose_one_feedback = WaitForCurrentPose();
      completed = pose_one_feedback && Reached(*pose_one_feedback, kPoseOne);
    }
    if (completed) std::cout << "动作一已确认归位：保持使能，不发送卸力命令。" << std::endl;
#elif defined(D1_PLATFORM1_COMPLETE_ACTION)
    // 完整动作严格串行：每段末端均读取反馈确认，避免上一段尚未稳定就执行下一段。
    completed = MoveBySegments(publisher, kPoseOne, kPoseThree,
                               "一号平台放置：由动作一移动至动作三");
    if (completed) {
      const auto pose_three_feedback = WaitForCurrentPose();
      completed = pose_three_feedback && Reached(*pose_three_feedback, kPoseThree);
      if (!completed) std::cerr << "动作三反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      // 放置到位后静置 2 秒，避免物体和爪夹仍接触时立刻执行张开。
      std::cout << "动作三已确认，等待 2 秒后执行一号平台夹爪张开动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseThree, kPoseFour,
                                 "一号平台夹爪张开：由动作三移动至动作四");
    }
    if (completed) {
      const auto pose_four_feedback = WaitForCurrentPose();
      completed = pose_four_feedback && Reached(*pose_four_feedback, kPoseFour);
      if (!completed) std::cerr << "动作四反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      // 用户指定夹爪张开完成后静置 2 秒，再抬升，给被放置物体脱离爪夹留出时间。
      std::cout << "动作四已确认，等待 2 秒后执行一号平台抬升动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseFour, kPoseFive,
                                 "一号平台抬升：由动作四移动至动作五");
    }
    if (completed) {
      const auto pose_five_feedback = WaitForCurrentPose();
      completed = pose_five_feedback && Reached(*pose_five_feedback, kPoseFive);
      if (!completed) std::cerr << "动作五反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseFive, kPoseOne,
                                 "一号平台归位：由动作五移动至动作一");
    }
    if (completed) {
      const auto pose_one_feedback = WaitForCurrentPose();
      completed = pose_one_feedback && Reached(*pose_one_feedback, kPoseOne);
      if (!completed) std::cerr << "动作一反馈确认失败。" << std::endl;
    }
    if (completed) {
      std::cout << "一号平台完整机械臂动作已完成：保持使能，不发送卸力命令。" << std::endl;
    }
#elif defined(D1_PLATFORM2_COMPLETE_ACTION)
    // 二号平台完整动作与一号平台相同：阶段确认后才进入下一段，异常时统一卸力。
    completed = MoveBySegments(publisher, kPoseOne, kPoseSix,
                               "二号平台放置：由动作一移动至动作六");
    if (completed) {
      const auto pose_six_feedback = WaitForCurrentPose();
      completed = pose_six_feedback && Reached(*pose_six_feedback, kPoseSix);
      if (!completed) std::cerr << "动作六反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      // 放置到位后静置 2 秒，避免物体和爪夹仍接触时立刻执行张开。
      std::cout << "动作六已确认，等待 2 秒后执行二号平台夹爪张开动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseSix, kPoseSeven,
                                 "二号平台夹爪张开：由动作六移动至动作七");
    }
    if (completed) {
      const auto pose_seven_feedback = WaitForCurrentPose();
      completed = pose_seven_feedback && Reached(*pose_seven_feedback, kPoseSeven);
      if (!completed) std::cerr << "动作七反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      // 张开后静置 2 秒，给被放置物体脱离爪夹留出时间，再执行抬升。
      std::cout << "动作七已确认，等待 2 秒后执行二号平台抬升动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseSeven, kPoseEight,
                                 "二号平台抬升：由动作七移动至动作八");
    }
    if (completed) {
      const auto pose_eight_feedback = WaitForCurrentPose();
      completed = pose_eight_feedback && Reached(*pose_eight_feedback, kPoseEight);
      if (!completed) std::cerr << "动作八反馈确认失败，停止完整动作。" << std::endl;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseEight, kPoseOne,
                                 "二号平台归位：由动作八移动至动作一");
    }
    if (completed) {
      const auto pose_one_feedback = WaitForCurrentPose();
      completed = pose_one_feedback && Reached(*pose_one_feedback, kPoseOne);
      if (!completed) std::cerr << "动作一反馈确认失败。" << std::endl;
    }
    if (completed) {
      std::cout << "二号平台完整机械臂动作已完成：保持使能，不发送卸力命令。" << std::endl;
    }
#elif defined(D1_TRANSFER_PLATFORM_COMPLETE_PLACE_ACTION)
    // 中转平台完整放置严格串行：放置、张开、抬升、左移均在前段反馈确认后才执行。
    completed = MoveBySegments(publisher, kPoseOne, kPoseNine,
                               "中转平台放置：由动作一移动至动作九");
    if (completed) {
      const auto pose_nine_feedback = WaitForCurrentPose();
      completed = pose_nine_feedback && Reached(*pose_nine_feedback, kPoseNine);
      if (!completed) std::cerr << "动作九反馈确认失败，停止完整放置动作。" << std::endl;
    }
    if (completed) {
      // 放置后静置 2 秒，避免物体和爪夹仍接触时立刻执行张开。
      std::cout << "动作九已确认，等待 2 秒后执行中转平台夹爪张开动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseNine, kPoseTen,
                                 "中转平台夹爪张开：由动作九移动至动作十");
    }
    if (completed) {
      const auto pose_ten_feedback = WaitForCurrentPose();
      completed = pose_ten_feedback && Reached(*pose_ten_feedback, kPoseTen);
      if (!completed) std::cerr << "动作十反馈确认失败，停止完整放置动作。" << std::endl;
    }
    if (completed) {
      // 张开后静置 2 秒，给被放置物体脱离爪夹留出时间，再执行抬升。
      std::cout << "动作十已确认，等待 2 秒后执行中转平台抬升动作。" << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(2));
      if (g_stop_requested.load()) completed = false;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseTen, kPoseEleven,
                                 "中转平台抬升：由动作十移动至动作十一");
    }
    if (completed) {
      const auto pose_eleven_feedback = WaitForCurrentPose();
      completed = pose_eleven_feedback && Reached(*pose_eleven_feedback, kPoseEleven);
      if (!completed) std::cerr << "动作十一反馈确认失败，停止完整放置动作。" << std::endl;
    }
    if (completed) {
      completed = MoveBySegments(publisher, kPoseEleven, kPoseTwelve,
                                 "中转平台左移：由动作十一移动至动作十二");
    }
    if (completed) {
      const auto pose_twelve_feedback = WaitForCurrentPose();
      completed = pose_twelve_feedback && Reached(*pose_twelve_feedback, kPoseTwelve);
      if (!completed) std::cerr << "动作十二反馈确认失败。" << std::endl;
    }
    if (completed) {
      std::cout << "中转平台完整放置动作已完成：保持使能，不发送卸力命令。" << std::endl;
    }
#elif defined(D1_PLATFORM1_PLACE_ONLY)
    completed = MoveBySegments(publisher, kPoseOne, kPoseThree,
                               "由动作一移动至动作三（一号平台放置）");
    if (completed) {
      const auto pose_three_feedback = WaitForCurrentPose();
      completed = pose_three_feedback && Reached(*pose_three_feedback, kPoseThree);
    }
    if (completed) std::cout << "动作三已确认到位：保持使能，不发送卸力命令。" << std::endl;
#else
    completed = MoveBySegments(publisher, kPoseOne, kPoseTwo, "移动至位置二");
    if (completed) {
      const auto pose_two_feedback = WaitForCurrentPose();
      completed = pose_two_feedback && Reached(*pose_two_feedback, kPoseTwo);
    }
    if (completed) std::cout << "位置二已确认到位：保持使能，不发送卸力命令。" << std::endl;
#endif
  }
  if (completed) {
    return 0;
  }

  std::cerr << "任务未完成或已中断；发送 funcode=5/mode=0 卸力。" << std::endl;
  PublishCommand(publisher, 5, EnableData(false));
  return 1;
}
