#pragma once

#include <optional>
#include <string>

namespace rk_arm {

/** 仅封装已在 SDK 源码中出现的 JSON 形状；本类不拥有 DDS writer。 */
class D1CommandEncoder {
 public:
  static std::optional<std::string> EncodeSingleJointTarget(
      int sequence, int command_id, double target_app_value, int delay_ms,
      std::string* error);

  /** 没有可审计的停止 JSON 时必须明确拒绝，不能发送猜测数据。 */
  static std::optional<std::string> EncodeStop(std::string* error);
};

}  // namespace rk_arm
