#pragma once

// startup StopMove 的一次性状态机与 Unitree SDK 解耦，便于在无机器人时验证：
// prearm 已验证控制权与 responder，故只允许一次真实停车，失败立即 fail-closed。
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
    return StartupStopRetryDecision{attempts_, false, false, 0};
  }

  static constexpr int kMaxAttempts = 1;

private:
  int attempts_{0};
};

}  // namespace rk_go2_sdk_bridge
