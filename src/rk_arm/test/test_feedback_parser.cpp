#include "rk_arm/d1_feedback_parser.hpp"
#include <cassert>
int main() {
  rk_arm::D1FeedbackFrame frame;
  assert(rk_arm::ParseD1Feedback("{\"funcode\":1,\"data\":{\"angle0\":0,\"angle1\":1,\"angle2\":2,\"angle3\":3,\"angle4\":4,\"angle5\":5,\"angle6\":6}}", &frame));
  assert(frame.app_values && (*frame.app_values)[6] == 6.0);
  assert(rk_arm::ParseD1Feedback("{\"funcode\":3,\"data\":{\"enable_status\":1,\"power_status\":0,\"error_status\":0}}", &frame));
  assert(frame.enable_status && *frame.enable_status == 1);
  assert(!rk_arm::ParseD1Feedback("", &frame)); assert(!rk_arm::ParseD1Feedback("not json", &frame));
  assert(!rk_arm::ParseD1Feedback("{\"funcode\":1,\"data\":{\"angle0\":0}}", &frame));
  assert(!rk_arm::ParseD1Feedback("{\"funcode\":1,\"data\":{\"angle0\":\"bad\",\"angle1\":1,\"angle2\":2,\"angle3\":3,\"angle4\":4,\"angle5\":5,\"angle6\":6}}", &frame));
  assert(!rk_arm::ParseD1Feedback("{\"funcode\":3,\"data\":{\"enable_status\":1,\"power_status\":0,\"error_status\":\"nan\"}}", &frame));
  assert(!rk_arm::ParseD1Feedback("{\"funcode\":1,\"data\":{\"angle0\":nan,\"angle1\":1,\"angle2\":2,\"angle3\":3,\"angle4\":4,\"angle5\":5,\"angle6\":6}}", &frame));
}
