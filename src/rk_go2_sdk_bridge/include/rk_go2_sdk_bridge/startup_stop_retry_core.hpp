#pragma once

// startup StopMove 的有限重试状态机与 Unitree SDK 解耦，便于在无机器人时验证：
// 只允许三次、仅由调用方传入 StopMove 返回值，绝不产生其他 Sport 动作。
namespace rk_go2_sdk_bridge
{

struct StartupStopRetryDecision
{
  int attempt{0};
  bool success{false};
  bool retry{false};
  int backoff_ms{0};
};

class StartupStopRetryCore
{
public:
  int NextAttempt() const
  {
    return attempts_ + 1;
  }

  StartupStopRetryDecision RecordResult(int result)
  {
    ++attempts_;
    if (result == 0) {
      return StartupStopRetryDecision{attempts_, true, false, 0};
    }
    if (attempts_ < kMaxAttempts) {
      return StartupStopRetryDecision{
          attempts_, false, true, kBackoffMs[attempts_ - 1]};
    }
    return StartupStopRetryDecision{attempts_, false, false, 0};
  }

  static constexpr int kMaxAttempts = 3;

private:
  static constexpr int kBackoffMs[2] = {100, 250};
  int attempts_{0};
};

}  // namespace rk_go2_sdk_bridge
