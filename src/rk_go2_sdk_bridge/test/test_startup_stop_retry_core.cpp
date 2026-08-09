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

void TestFirstFailureExhaustsWithoutRetry()
{
  rk_go2_sdk_bridge::StartupStopRetryCore core;
  const auto first = core.RecordResult(-1);
  Expect(first.attempt == 1 && !first.success && !first.retry,
         "first failure must exhaust; a second StopMove is forbidden");
}

}  // namespace

int main()
{
  TestFirstAttemptSuccess();
  TestFirstFailureExhaustsWithoutRetry();
  return g_failures == 0 ? 0 : 1;
}
