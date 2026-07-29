#include "rk_go2_sdk_bridge/udp_motion_core.hpp"

#include <cmath>
#include <iostream>
#include <string>

// 该测试不链接ROS或Unitree SDK，可在虚拟机验证全部停车判定。
namespace
{

using rk_go2_sdk_bridge::MotionAction;
using rk_go2_sdk_bridge::MotionLimits;
using rk_go2_sdk_bridge::UdpMotionCore;

int g_failures = 0;

void Expect(bool condition, const std::string& message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    ++g_failures;
  }
}

void ExpectNear(double actual, double expected, const std::string& message)
{
  Expect(std::fabs(actual - expected) < 1e-9, message);
}

void TestDirectTargetWithoutRamp()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  const auto accepted = core.AcceptPacket("0.25 0.0 0.0", 1.0);
  Expect(
      accepted.action == MotionAction::kNone,
      "valid packet should update target without immediate SDK action");

  const auto tick = core.Tick(1.01);
  Expect(tick.action == MotionAction::kMove, "fresh target should move");
  ExpectNear(tick.command.vx, 0.25, "first tick must use full target vx");
}

void TestZeroStopsImmediately()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  core.AcceptPacket("0.25 0 0", 1.0);
  const auto stop = core.AcceptPacket("0 0 0", 1.05);
  Expect(stop.action == MotionAction::kStop, "zero packet must stop");
  Expect(stop.reason == "zero_command", "zero stop reason");
  Expect(!core.active(), "zero packet must clear active target");
}

void TestDeadbandStops()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  const auto stop = core.AcceptPacket("0.01 -0.01 0.01", 1.0);
  Expect(stop.action == MotionAction::kStop, "deadband values become zero");
}

void TestInvalidPacketsStop()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  const char* invalid_packets[] = {
      "bad packet",
      "0.1 0.0",
      "0.1 0.0 0.0 extra",
      "nan 0.0 0.0",
      "inf 0.0 0.0",
  };

  for (const char* packet : invalid_packets) {
    core.AcceptPacket("0.20 0 0", 1.0);
    const auto stop = core.AcceptPacket(packet, 1.1);
    Expect(
        stop.action == MotionAction::kStop,
        std::string("invalid packet must stop: ") + packet);
    Expect(!core.active(), "invalid packet must clear target");
  }
}

void TestOutOfRangeStopsInsteadOfClamping()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  const auto stop = core.AcceptPacket("0.251 0 0", 1.0);
  Expect(stop.action == MotionAction::kStop, "out-of-range vx must stop");
  Expect(
      stop.reason == "out_of_range_packet",
      "out-of-range reason must be explicit");

  const auto boundary = core.AcceptPacket("-0.25 0.05 -0.60", 2.0);
  Expect(
      boundary.action == MotionAction::kNone,
      "configured limit boundary must be accepted");
}

void TestWatchdog()
{
  UdpMotionCore core(MotionLimits{}, 0.30);
  core.AcceptPacket("0.25 0 0", 10.0);

  const auto fresh = core.Tick(10.299);
  Expect(fresh.action == MotionAction::kMove, "fresh command should move");

  const auto stale = core.Tick(10.300);
  Expect(stale.action == MotionAction::kStop, "watchdog boundary must stop");
  Expect(stale.reason == "watchdog_timeout", "watchdog reason");
  Expect(!core.active(), "watchdog must clear target");

  const auto after_stop = core.Tick(10.4);
  Expect(
      after_stop.action == MotionAction::kNone,
      "watchdog stop must not repeat without a new command");
}

}  // namespace

int main()
{
  TestDirectTargetWithoutRamp();
  TestZeroStopsImmediately();
  TestDeadbandStops();
  TestInvalidPacketsStop();
  TestOutOfRangeStopsInsteadOfClamping();
  TestWatchdog();

  if (g_failures != 0) {
    std::cerr << g_failures << " test assertion(s) failed" << std::endl;
    return 1;
  }
  std::cout << "udp_motion_core tests passed" << std::endl;
  return 0;
}
