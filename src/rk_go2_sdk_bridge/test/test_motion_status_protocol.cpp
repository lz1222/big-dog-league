#include "rk_go2_sdk_bridge/motion_status_protocol.hpp"
#include "rk_go2_sdk_bridge/udp_motion_core.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace
{

int g_failures = 0;

void Expect(bool condition, const std::string& message)
{
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    ++g_failures;
  }
}

rk_go2_sdk_bridge::MotionStatusEvent MakeStatus(
    std::uint64_t sequence, const std::string& event, std::int32_t ret)
{
  return {
    "instance-test-001", sequence, event, ret, "test_reason",
    0.25, 0.0, 0.03, 1234567};
}

void TestMoveAndStopEncoding()
{
  const auto move = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(7, "MOVE", 0));
  Expect(move.find("\"sequence\":7") != std::string::npos,
         "MOVE must contain sequence");
  Expect(move.find("\"server_instance_id\":\"instance-test-001\"") !=
         std::string::npos, "status must contain server instance identity");
  Expect(move.find("\"event\":\"MOVE\"") != std::string::npos,
         "MOVE event must encode");
  Expect(move.find("\"ret\":0") != std::string::npos,
         "MOVE success ret must encode");

  const auto move_error = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(8, "MOVE", -5));
  Expect(move_error.find("\"ret\":-5") != std::string::npos,
         "MOVE failure ret must encode");

  const auto stop = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(9, "STOP_MOVE", 0));
  Expect(stop.find("\"event\":\"STOP_MOVE\"") != std::string::npos,
         "STOP_MOVE event must encode");
  const auto stop_error = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(10, "STOP_MOVE", -9));
  Expect(stop_error.find("\"ret\":-9") != std::string::npos,
         "STOP_MOVE failure ret must encode");
}

void TestSequenceAndInvalidNumbers()
{
  const auto first = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(11, "MOVE", 0));
  const auto second = rk_go2_sdk_bridge::EncodeMotionStatusJson(
      MakeStatus(12, "STOP_MOVE", 0));
  Expect(first.find("\"sequence\":11") != std::string::npos &&
         second.find("\"sequence\":12") != std::string::npos,
         "caller-visible sequences must remain monotonic");

  auto nan_status = MakeStatus(13, "MOVE", 0);
  nan_status.vx = std::numeric_limits<double>::quiet_NaN();
  Expect(rk_go2_sdk_bridge::EncodeMotionStatusJson(nan_status).empty(),
         "NaN must not enter JSON");
  auto inf_status = MakeStatus(14, "MOVE", 0);
  inf_status.yaw = std::numeric_limits<double>::infinity();
  Expect(rk_go2_sdk_bridge::EncodeMotionStatusJson(inf_status).empty(),
         "Inf must not enter JSON");
  auto missing_instance = MakeStatus(15, "MOVE", 0);
  missing_instance.server_instance_id.clear();
  Expect(rk_go2_sdk_bridge::EncodeMotionStatusJson(missing_instance).empty(),
         "missing server instance identity must fail encoding");
}

void TestStatusFailureDoesNotChangeMotionDecision()
{
  // status 编码失败相当于 side-channel send failure；核心决策必须仍保持原值。
  rk_go2_sdk_bridge::UdpMotionCore core(
      rk_go2_sdk_bridge::MotionLimits{}, 0.30);
  core.AcceptPacket("0.25 0 0", 1.0);
  auto status = MakeStatus(16, "MOVE", 0);
  status.vx = std::numeric_limits<double>::quiet_NaN();
  Expect(rk_go2_sdk_bridge::EncodeMotionStatusJson(status).empty(),
         "simulated status failure must be observable");
  const auto decision = core.Tick(1.01);
  Expect(decision.action == rk_go2_sdk_bridge::MotionAction::kMove,
         "status failure must not alter motion decision");
  Expect(std::fabs(decision.command.vx - 0.25) < 1e-9,
         "status failure must not alter requested vx");
}

}  // namespace

int main()
{
  TestMoveAndStopEncoding();
  TestSequenceAndInvalidNumbers();
  TestStatusFailureDoesNotChangeMotionDecision();
  return g_failures == 0 ? 0 : 1;
}
