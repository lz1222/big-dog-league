#include "rk_go2_sdk_bridge/startup_stop_retry_core.hpp"

#include <iostream>
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

void TestFirstAttemptSuccess()
{
  rk_go2_sdk_bridge::StartupStopRetryCore core;
  const auto decision = core.RecordResult(0);
  Expect(decision.attempt == 1, "first attempt number");
  Expect(decision.success && !decision.retry, "first success must finish");
}

void TestSecondAttemptSuccess()
{
  rk_go2_sdk_bridge::StartupStopRetryCore core;
  const auto first = core.RecordResult(-1);
  const auto second = core.RecordResult(0);
  Expect(first.retry && first.backoff_ms == 100, "first failure has bounded backoff");
  Expect(second.attempt == 2 && second.success, "second success must finish");
}

void TestThreeFailuresExhaustWithoutFourthRetry()
{
  rk_go2_sdk_bridge::StartupStopRetryCore core;
  core.RecordResult(-1);
  const auto second = core.RecordResult(-1);
  const auto third = core.RecordResult(-1);
  Expect(second.retry && second.backoff_ms == 250, "second failure has final backoff");
  Expect(third.attempt == 3 && !third.success && !third.retry,
         "third failure must exhaust; a fourth StopMove is forbidden");
}

}  // namespace

int main()
{
  TestFirstAttemptSuccess();
  TestSecondAttemptSuccess();
  TestThreeFailuresExhaustWithoutFourthRetry();
  return g_failures == 0 ? 0 : 1;
}
