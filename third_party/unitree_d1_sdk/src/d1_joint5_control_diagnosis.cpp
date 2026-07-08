#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

namespace
{

constexpr int kServoCount = 7;

struct ServoStats
{
  bool initialized = false;
  double first = 0.0;
  double last = 0.0;
  double min = 0.0;
  double max = 0.0;
  double total_abs_delta = 0.0;
  double max_abs_step = 0.0;
};

std::mutex g_mutex;
std::array<double, kServoCount> g_latest{};
std::array<ServoStats, kServoCount> g_stats;
std::chrono::steady_clock::time_point g_start_time;
std::chrono::steady_clock::time_point g_last_sample_time;
std::string g_last_feedback;
std::string g_phase = "startup";
int g_sample_count = 0;
std::ofstream g_csv;

std::atomic<bool> g_stop_requested{false};

std::array<double, kServoCount> ExtractServoData(
    const unitree_arm::msg::dds_::PubServoInfo_* message)
{
  return {
    message->servo0_data_(),
    message->servo1_data_(),
    message->servo2_data_(),
    message->servo3_data_(),
    message->servo4_data_(),
    message->servo5_data_(),
    message->servo6_data_(),
  };
}

void ServoHandler(const void* raw_message)
{
  const auto* message =
      static_cast<const unitree_arm::msg::dds_::PubServoInfo_*>(raw_message);
  const auto values = ExtractServoData(message);
  const auto now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(g_mutex);
  if (g_sample_count == 0) {
    g_start_time = now;
  }
  const double elapsed =
      std::chrono::duration<double>(now - g_start_time).count();

  for (int index = 0; index < kServoCount; ++index) {
    auto& stat = g_stats[index];
    const double value = values[index];
    g_latest[index] = value;

    if (!stat.initialized) {
      stat.initialized = true;
      stat.first = value;
      stat.last = value;
      stat.min = value;
      stat.max = value;
      continue;
    }

    const double abs_delta = std::abs(value - stat.last);
    stat.last = value;
    stat.min = std::min(stat.min, value);
    stat.max = std::max(stat.max, value);
    stat.total_abs_delta += abs_delta;
    stat.max_abs_step = std::max(stat.max_abs_step, abs_delta);
  }

  if (g_csv.is_open()) {
    g_csv << std::fixed << std::setprecision(4) << elapsed << "," << g_phase;
    for (int index = 0; index < kServoCount; ++index) {
      g_csv << "," << values[index];
    }
    g_csv << "\n";
  }

  g_last_sample_time = now;
  g_sample_count += 1;
}

void FeedbackHandler(const void* raw_message)
{
  const auto* message =
      static_cast<const unitree_arm::msg::dds_::ArmString_*>(raw_message);
  std::lock_guard<std::mutex> lock(g_mutex);
  g_last_feedback = message->data_();
}

void SetCycloneInterfaceIfNeeded(const std::string& network_interface)
{
  if (network_interface.empty() || std::getenv("CYCLONEDDS_URI") != nullptr) {
    return;
  }

  std::ostringstream uri;
  uri << "<CycloneDDS><Domain><General><NetworkInterfaceAddress>"
      << network_interface
      << "</NetworkInterfaceAddress></General></Domain></CycloneDDS>";
  setenv("CYCLONEDDS_URI", uri.str().c_str(), 0);
  std::cerr << "CYCLONEDDS_URI was not set; using network interface "
            << network_interface << std::endl;
}

double ParseDouble(const char* raw, double fallback)
{
  try {
    return std::stod(raw);
  } catch (...) {
    return fallback;
  }
}

int ParseInt(const char* raw, int fallback)
{
  try {
    return std::stoi(raw);
  } catch (...) {
    return fallback;
  }
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " [network_interface] [joint_id] [monitor_servo_index] [delta_deg] "
         "[hold_sec] [csv_path] [--execute]\n\n"
      << "Default is dry-run. Add --execute to actually publish "
         "rt/arm_Command.\n\n"
      << "Examples:\n"
      << "  " << program << " eth0 5 4 1.0 0.8 /tmp/d1_joint5.csv\n"
      << "  " << program
      << " eth0 5 4 1.0 0.8 /tmp/d1_joint5.csv --execute\n\n"
      << "Joint note: physical joint 5 is normally monitored as servo4 "
         "(zero-based feedback index). The SDK single-joint command appears "
         "to use id=5 for that joint.\n";
}

void SetPhase(const std::string& phase)
{
  std::lock_guard<std::mutex> lock(g_mutex);
  g_phase = phase;
}

std::array<double, kServoCount> LatestAngles()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_latest;
}

int SampleCount()
{
  std::lock_guard<std::mutex> lock(g_mutex);
  return g_sample_count;
}

void PrintLatest(const std::string& prefix)
{
  const auto latest = LatestAngles();
  int sample_count = 0;
  std::string feedback;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    sample_count = g_sample_count;
    feedback = g_last_feedback;
  }

  std::cout << prefix << " samples=" << sample_count << " angles=[";
  for (int index = 0; index < kServoCount; ++index) {
    if (index > 0) {
      std::cout << ", ";
    }
    std::cout << std::fixed << std::setprecision(2) << latest[index];
  }
  std::cout << "]";
  if (!feedback.empty()) {
    std::cout << " last_feedback=" << feedback;
  }
  std::cout << std::endl;
}

std::string BuildJointCommand(int seq, int joint_id, double target_deg,
                              int delay_ms)
{
  std::ostringstream json;
  json << "{\"seq\":" << seq
       << ",\"address\":1,\"funcode\":1,\"data\":{\"id\":" << joint_id
       << ",\"angle\":" << std::fixed << std::setprecision(3) << target_deg
       << ",\"delay_ms\":" << delay_ms << "}}";
  return json.str();
}

void PublishJointCommand(
    unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>&
        publisher,
    int seq, int joint_id, double target_deg, int delay_ms, bool execute)
{
  const auto payload = BuildJointCommand(seq, joint_id, target_deg, delay_ms);
  std::cout << (execute ? "SEND " : "DRY_RUN ")
            << "rt/arm_Command: " << payload << std::endl;
  if (!execute) {
    return;
  }

  unitree_arm::msg::dds_::ArmString_ msg{};
  msg.data_() = payload;
  publisher.Write(msg);
}

void MonitorFor(double seconds, const std::string& label)
{
  SetPhase(label);
  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(seconds));
  while (!g_stop_requested.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  PrintLatest(label);
}

void PrintSummary(int monitor_servo_index, double min_response_deg)
{
  std::array<ServoStats, kServoCount> stats;
  std::string feedback;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    stats = g_stats;
    feedback = g_last_feedback;
  }

  std::cout << "\n=== D1 joint-5 control diagnosis summary ===\n";
  if (!stats[monitor_servo_index].initialized) {
    std::cout << "No samples for monitored servo" << monitor_servo_index
              << ". Check network interface and D1 feedback topics.\n";
    return;
  }

  double largest_range = 0.0;
  int largest_index = -1;
  for (int index = 0; index < kServoCount; ++index) {
    const auto& stat = stats[index];
    if (!stat.initialized) {
      continue;
    }
    const double range = stat.max - stat.min;
    if (range > largest_range) {
      largest_range = range;
      largest_index = index;
    }
    std::cout << "servo" << index
              << " first=" << std::fixed << std::setprecision(2) << stat.first
              << " last=" << stat.last
              << " range=" << range
              << " total_abs_delta=" << stat.total_abs_delta
              << " max_step=" << stat.max_abs_step;
    if (index == monitor_servo_index) {
      std::cout << "  <-- monitored physical joint 5";
    }
    std::cout << std::endl;
  }

  const double monitored_range =
      stats[monitor_servo_index].max - stats[monitor_servo_index].min;

  std::cout << "\nInterpretation:\n";
  if (monitored_range >= min_response_deg) {
    std::cout
        << "- servo" << monitor_servo_index
        << " responded to the small command. The motor/control path is at "
           "least partially controllable.\n";
  } else {
    std::cout
        << "- servo" << monitor_servo_index
        << " did not move enough for the requested small command. Possible "
           "causes: command rejected, wrong joint id mapping, servo/encoder "
           "fault, mechanical binding, or controller protection.\n";
  }

  if (largest_index >= 0 && largest_index != monitor_servo_index &&
      largest_range > monitored_range + min_response_deg) {
    std::cout
        << "- Another servo moved more than the monitored joint: servo"
        << largest_index
        << ". This suggests the SDK id may not map to physical joint 5, or "
           "another process/hardware loop is moving the arm.\n";
  }

  if (!feedback.empty()) {
    std::cout << "last arm feedback: " << feedback << std::endl;
  }

  std::cout
      << "- If physical joint 5 heats quickly, moves toward a hard stop, or "
         "moves in the wrong direction, stop testing and treat it as a "
         "hardware/zero/encoder fault instead of compensating in software.\n";
}

bool HasFlag(int argc, char** argv, const std::string& flag)
{
  for (int index = 1; index < argc; ++index) {
    if (std::string(argv[index]) == flag) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc > 1 && std::string(argv[1]) == "--help") {
    PrintUsage(argv[0]);
    return 0;
  }

  const std::string network_interface = argc >= 2 ? argv[1] : "eth0";
  const int joint_id = argc >= 3 ? ParseInt(argv[2], 5) : 5;
  const int monitor_servo_index = argc >= 4 ? ParseInt(argv[3], 4) : 4;
  const double delta_deg = argc >= 5 ? ParseDouble(argv[4], 1.0) : 1.0;
  const double hold_sec = argc >= 6 ? ParseDouble(argv[5], 0.8) : 0.8;
  const std::string csv_path = argc >= 7 ? argv[6] : "";
  const bool execute = HasFlag(argc, argv, "--execute");

  if (joint_id < 0 || joint_id > 20 || monitor_servo_index < 0 ||
      monitor_servo_index >= kServoCount || delta_deg <= 0.0 ||
      delta_deg > 5.0 || hold_sec <= 0.0 || hold_sec > 5.0) {
    PrintUsage(argv[0]);
    std::cerr << "Refusing unsafe arguments. Keep delta_deg in (0, 5] and "
                 "hold_sec in (0, 5].\n";
    return 2;
  }

  SetCycloneInterfaceIfNeeded(network_interface);
  if (!csv_path.empty()) {
    g_csv.open(csv_path);
    if (!g_csv.is_open()) {
      std::cerr << "Failed to open CSV path: " << csv_path << std::endl;
      return 3;
    }
    g_csv << "time_sec,phase,servo0,servo1,servo2,servo3,servo4,servo5,servo6\n";
  }

  if (network_interface.empty()) {
    unitree::robot::ChannelFactory::Instance()->Init(0);
  } else {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
  }

  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>
      servo_subscriber("current_servo_angle");
  servo_subscriber.InitChannel(ServoHandler);

  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>
      rt_servo_subscriber("rt/current_servo_angle");
  rt_servo_subscriber.InitChannel(ServoHandler);

  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>
      feedback_subscriber("arm_Feedback");
  feedback_subscriber.InitChannel(FeedbackHandler);

  unitree::robot::ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>
      rt_feedback_subscriber("rt/arm_Feedback");
  rt_feedback_subscriber.InitChannel(FeedbackHandler);

  unitree::robot::ChannelPublisher<unitree_arm::msg::dds_::ArmString_>
      command_publisher("rt/arm_Command");
  command_publisher.InitChannel();

  std::cout
      << "D1 joint-5 control diagnosis started.\n"
      << "Mode: " << (execute ? "EXECUTE" : "DRY_RUN")
      << "\nnetwork_interface=" << network_interface
      << " joint_id=" << joint_id
      << " monitor_servo_index=" << monitor_servo_index
      << " delta_deg=" << delta_deg
      << " hold_sec=" << hold_sec << "\n"
      << "This tool sends only three tiny absolute targets around the current "
         "servo angle when --execute is present.\n";

  MonitorFor(1.0, "baseline_wait");
  if (SampleCount() == 0) {
    std::cerr << "No current_servo_angle samples received. Abort.\n";
    return 4;
  }

  const auto baseline = LatestAngles();
  const double baseline_angle = baseline[monitor_servo_index];
  const double plus_target = baseline_angle + delta_deg;
  const double minus_target = baseline_angle - delta_deg;

  std::cout << "Baseline servo" << monitor_servo_index
            << "=" << std::fixed << std::setprecision(2)
            << baseline_angle << " deg. Planned targets: "
            << plus_target << " -> " << minus_target << " -> "
            << baseline_angle << " deg." << std::endl;

  if (!execute) {
    std::cout << "Dry-run only. Re-run with --execute to publish commands."
              << std::endl;
    PrintSummary(monitor_servo_index, std::min(1.0, delta_deg * 0.5));
    return 0;
  }

  int seq = 200;
  PublishJointCommand(command_publisher, seq++, joint_id, plus_target, 0, true);
  MonitorFor(hold_sec, "hold_plus_target");

  if (!g_stop_requested.load()) {
    PublishJointCommand(
        command_publisher, seq++, joint_id, minus_target, 0, true);
    MonitorFor(hold_sec, "hold_minus_target");
  }

  if (!g_stop_requested.load()) {
    PublishJointCommand(
        command_publisher, seq++, joint_id, baseline_angle, 0, true);
    MonitorFor(hold_sec, "return_baseline");
  }

  PrintSummary(monitor_servo_index, std::min(1.0, delta_deg * 0.5));
  if (g_csv.is_open()) {
    std::cout << "CSV written to: " << csv_path << std::endl;
    g_csv.close();
  }
  return 0;
}
