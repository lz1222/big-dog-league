#include <unitree/robot/channel/channel_factory.hpp>
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
  double max_abs_speed = 0.0;
  double max_step_delta = 0.0;
  double total_abs_delta = 0.0;
  int moving_samples = 0;
};

std::mutex g_mutex;
std::array<ServoStats, kServoCount> g_stats;
std::array<double, kServoCount> g_latest{};
std::chrono::steady_clock::time_point g_last_sample_time;
std::chrono::steady_clock::time_point g_start_time;
std::string g_last_feedback;
int g_sample_count = 0;
std::atomic<bool> g_running{true};
std::ofstream g_csv;

double g_speed_warn_deg_s = 1.0;
double g_step_warn_deg = 0.25;

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
  const double dt =
      g_sample_count > 0
          ? std::chrono::duration<double>(now - g_last_sample_time).count()
          : 0.0;
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

    const double delta = value - stat.last;
    const double abs_delta = std::abs(delta);
    const double abs_speed = dt > 1e-6 ? abs_delta / dt : 0.0;

    stat.last = value;
    stat.min = std::min(stat.min, value);
    stat.max = std::max(stat.max, value);
    stat.max_step_delta = std::max(stat.max_step_delta, abs_delta);
    stat.max_abs_speed = std::max(stat.max_abs_speed, abs_speed);
    stat.total_abs_delta += abs_delta;

    if (abs_speed >= g_speed_warn_deg_s || abs_delta >= g_step_warn_deg) {
      stat.moving_samples += 1;
    }
  }

  if (g_csv.is_open()) {
    g_csv << std::fixed << std::setprecision(4) << elapsed;
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

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program
      << " [network_interface] [duration_sec] [speed_warn_deg_s] "
         "[range_warn_deg] [csv_path]\n\n"
      << "Examples:\n"
      << "  " << program << " eth0 30 1.0 2.0\n"
      << "  " << program << " eth0 30 1.0 2.0 /tmp/d1_servo_diag.csv\n"
      << "  " << program << " '' 20\n\n"
      << "This tool only subscribes to current_servo_angle and arm feedback. "
         "It does not send motion commands.\n";
}

void PrintLatest()
{
  std::array<double, kServoCount> latest{};
  int sample_count = 0;
  std::string feedback;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    latest = g_latest;
    sample_count = g_sample_count;
    feedback = g_last_feedback;
  }

  std::cout << "samples=" << sample_count << " angles=[";
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

void PrintSummary(double range_warn_deg)
{
  std::array<ServoStats, kServoCount> stats;
  int sample_count = 0;
  std::string feedback;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    stats = g_stats;
    sample_count = g_sample_count;
    feedback = g_last_feedback;
  }

  std::cout << "\n=== D1 servo heat diagnosis summary ===\n";
  if (sample_count == 0) {
    std::cout
        << "No current_servo_angle samples received. Check the network "
           "interface, CYCLONEDDS_URI, Unitree SDK2 library path, and whether "
           "the D1 arm publishes current_servo_angle or rt/current_servo_angle.\n";
    return;
  }

  bool found_motion_risk = false;
  for (int index = 0; index < kServoCount; ++index) {
    const auto& stat = stats[index];
    if (!stat.initialized) {
      continue;
    }

    const double range = stat.max - stat.min;
    const bool motion_risk =
        range >= range_warn_deg ||
        stat.max_abs_speed >= g_speed_warn_deg_s ||
        stat.moving_samples > std::max(3, sample_count / 20);
    found_motion_risk = found_motion_risk || motion_risk;

    std::string note;
    if (index == 4) {
      note = "  [human joint 5 if joints are numbered 1-7]";
    } else if (index == 5) {
      note = "  [SDK servo5, sixth servo if zero-based]";
    }

    std::cout << "servo" << index
              << " first=" << std::fixed << std::setprecision(2) << stat.first
              << " last=" << stat.last
              << " range=" << range
              << " max_speed=" << stat.max_abs_speed << " deg/s"
              << " total_abs_delta=" << stat.total_abs_delta
              << " moving_samples=" << stat.moving_samples
              << (motion_risk ? "  <-- CHECK" : "")
              << note
              << std::endl;
  }

  if (!feedback.empty()) {
    std::cout << "last arm feedback: " << feedback << std::endl;
  }

  std::cout << "\nInterpretation:\n";
  if (found_motion_risk) {
    std::cout
        << "- One or more servos are still moving, drifting, or jittering while "
           "powered. Heating is likely caused by repeated position correction, "
           "command oscillation, bad target pose, or mechanical binding.\n"
        << "- First inspect the CHECK servos: loosen load, verify the commanded "
           "angle is reachable, and make sure no program is repeatedly sending "
           "conflicting D1 commands.\n";
  } else {
    std::cout
        << "- Servo angles look stable. If a joint is still heating, the likely "
           "cause is holding torque: gravity load, overextended posture, joint "
           "against a hard stop, gripper clamping too hard, calibration/zero "
           "offset, or physical friction.\n"
        << "- Move to a low-torque safe pose, unload the arm, and compare "
           "temperature after 1-2 minutes. Do not leave the hot joint powered "
           "in a stalled posture.\n";
  }
}

double ParseDouble(const char* raw, double fallback)
{
  try {
    return std::stod(raw);
  } catch (...) {
    return fallback;
  }
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc > 1 && std::string(argv[1]) == "--help") {
    PrintUsage(argv[0]);
    return 0;
  }

  const std::string network_interface = argc >= 2 ? argv[1] : "";
  const double duration_sec = argc >= 3 ? ParseDouble(argv[2], 30.0) : 30.0;
  g_speed_warn_deg_s = argc >= 4 ? ParseDouble(argv[3], 1.0) : 1.0;
  const double range_warn_deg = argc >= 5 ? ParseDouble(argv[4], 2.0) : 2.0;
  const std::string csv_path = argc >= 6 ? argv[5] : "";

  if (duration_sec <= 0.0) {
    PrintUsage(argv[0]);
    return 2;
  }

  SetCycloneInterfaceIfNeeded(network_interface);
  if (!csv_path.empty()) {
    g_csv.open(csv_path);
    if (!g_csv.is_open()) {
      std::cerr << "Failed to open CSV path: " << csv_path << std::endl;
      return 3;
    }
    g_csv << "time_sec,servo0,servo1,servo2,servo3,servo4,servo5,servo6\n";
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

  std::cout
      << "D1 servo heat diagnosis started. Listening for " << duration_sec
      << " sec. speed_warn=" << g_speed_warn_deg_s
      << " deg/s range_warn=" << range_warn_deg << " deg\n"
      << "Joint numbering note: human joint 5 is usually servo4 if joints are "
         "numbered 1-7; SDK field servo5 is the sixth servo.\n"
      << "Keep the arm powered but do not send new motion commands during the "
         "test.\n";

  const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(duration_sec));

  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    PrintLatest();
  }

  g_running.store(false);
  PrintSummary(range_warn_deg);
  if (g_csv.is_open()) {
    std::cout << "CSV written to: " << csv_path << std::endl;
    g_csv.close();
  }
  return 0;
}
