#pragma once

#include <array>
#include <optional>
#include <string>

namespace rk_arm {

/** D1 ArmString 的容错解析结果；只保留已观测字段，不推断物理单位。 */
struct D1FeedbackFrame {
  int funcode{-1};
  std::optional<std::array<double, 7>> app_values;
  std::optional<int> enable_status;
  std::optional<int> power_status;
  std::optional<int> error_status;
  std::string error;
};

/** 无效、缺失或非有限字段均返回 false，调用方必须 fail closed。 */
bool ParseD1Feedback(const std::string& payload, D1FeedbackFrame* frame);

}  // namespace rk_arm
