#include "rk_arm/d1_command_encoder.hpp"
#include <cassert>
int main() {
  std::string error; auto json = rk_arm::D1CommandEncoder::EncodeSingleJointTarget(4, 5, 1.25, 0, &error);
  assert(json && *json == "{\"seq\":4,\"address\":1,\"funcode\":1,\"data\":{\"id\":5,\"angle\":1.250,\"delay_ms\":0}}");
  assert(!rk_arm::D1CommandEncoder::EncodeSingleJointTarget(-1, 0, 0, 0, &error));
  assert(!rk_arm::D1CommandEncoder::EncodeStop(&error)); assert(error == "STOP_SCHEMA_UNCONFIRMED");
}
