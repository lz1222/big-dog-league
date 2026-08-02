/* Minimal fake SDK helper for software smoke testing.
   This executable is accepted by the smoke identity check because it
   contains the required marker.  It performs NO network, SDK, motion,
   or hardware operations — it simply returns 0. */

#include <cstdio>

/* Must be present in the binary for smoke identity verification. */
extern "C" const char RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1[] =
    "RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1";

int main()
{
    std::printf("SOFTWARE_SMOKE fake SDK helper: returning 0\n");
    return 0;
}
