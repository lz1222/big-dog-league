#include "rk_arm/d1_observed_command.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

namespace {

std::string ReadFixture(const std::string& name) {
  std::ifstream input(std::filesystem::path(RK_ARM_FIXTURE_DIR) / name);
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

rk_arm::FeedbackSnapshot ValidFeedback(std::chrono::steady_clock::time_point now) {
  rk_arm::FeedbackSnapshot feedback{};
  feedback.dds_ready = feedback.angle_valid = feedback.servo_valid = true;
  feedback.latest_angle = feedback.latest_servo = now;
  feedback.app_values = {1.1, -87.8, 90.0, 1.0, 5.8, -2.5, -19.8};
  feedback.servo_values = feedback.app_values;
  return feedback;
}

void AssertSame(const rk_arm::D1ObservedCommand& left, const rk_arm::D1ObservedCommand& right) {
  assert(left.funcode == right.funcode && left.address == right.address &&
         left.mode == right.mode && left.seq == right.seq);
  for (int index = 0; index < 7; ++index) assert(left.angles[index] == right.angles[index]);
}

}  // namespace

int main() {
  const std::vector<std::string> fixtures{
      "joint1_positive.json", "joint1_negative.json", "joint2_positive.json",
      "gripper_open.json", "gripper_close.json"};
  std::uint64_t previous_seq = 0;
  for (const auto& fixture : fixtures) {
    rk_arm::D1ObservedCommand parsed{};
    std::string error;
    assert(rk_arm::ParseD1ObservedCommand(ReadFixture(fixture), &parsed, &error));
    assert(parsed.funcode == 2 && parsed.address == 1 && parsed.mode == 0);
    assert(parsed.seq > previous_seq);
    previous_seq = parsed.seq;
    const auto encoded = rk_arm::EncodeD1ObservedCommand(parsed, &error);
    assert(encoded);
    rk_arm::D1ObservedCommand round_trip{};
    assert(rk_arm::ParseD1ObservedCommand(*encoded, &round_trip, &error));
    AssertSame(parsed, round_trip);
  }

  rk_arm::D1ObservedCommand command{};
  std::string error;
  assert(!rk_arm::ParseD1ObservedCommand("{\"seq\":1,\"address\":1,\"funcode\":2,\"data\":{\"mode\":0,\"angle0\":0}}", &command, &error));
  assert(!rk_arm::ParseD1ObservedCommand("{\"seq\":1,\"address\":1,\"funcode\":2,\"data\":{\"mode\":0,\"angle0\":NaN,\"angle1\":0,\"angle2\":0,\"angle3\":0,\"angle4\":0,\"angle5\":0,\"angle6\":0}}", &command, &error));
  command = {2, 1, 0, 1, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           std::numeric_limits<double>::infinity()}};
  assert(!rk_arm::EncodeD1ObservedCommand(command, &error));

  const auto now = std::chrono::steady_clock::now();
  auto feedback = ValidFeedback(now);
  const auto joint = rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 2, 91.0, 60500, now, .3, .5, false);
  assert(joint.accepted && joint.json && joint.command && joint.safety_label == "DRY_RUN_ONLY / NOT SENT");
  for (int index = 0; index < 7; ++index) assert(index == 2 || joint.command->angles[index] == feedback.app_values[index]);
  assert(joint.command->angles[2] == 91.0);
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 6, 0.0, 60500, now, .3, .5, false).accepted);
  const auto gripper = rk_arm::D1ShadowCommandGenerator::PreviewGripper(feedback, -10.0, 60500, now, .3, .5, false);
  assert(gripper.accepted && gripper.command && gripper.command->angles[6] == -10.0);
  for (int index = 0; index < 6; ++index) assert(gripper.command->angles[index] == feedback.app_values[index]);

  feedback.angle_valid = false;
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, 0.0, 60500, now, .3, .5, false).accepted);
  feedback = ValidFeedback(now); feedback.latest_angle = now - std::chrono::seconds(1);
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, 0.0, 60500, now, .3, .5, false).accepted);
  feedback = ValidFeedback(now); feedback.servo_values[0] += 1.0;
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, 0.0, 60500, now, .3, .5, false).accepted);
  feedback = ValidFeedback(now);
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, std::numeric_limits<double>::infinity(), 60500, now, .3, .5, false).accepted);
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, 0.0, std::numeric_limits<std::uint64_t>::max(), now, .3, .5, false).accepted);
  assert(!rk_arm::D1ShadowCommandGenerator::PreviewJoint(feedback, 0, 0.0, 60500, now, .3, .5, true).accepted);
}
