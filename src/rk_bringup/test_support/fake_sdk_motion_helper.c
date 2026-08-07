/*
 * 无硬件软件验收专用 ELF helper。
 *
 * FrontJump supervisor 会检查 /proc/<pid>/exe 必须与 argv[0] 相同，因此
 * 不能用 shebang 脚本。默认短暂停留，保证 supervisor 可完成进程组身份
 * 采集；该程序不打开网络、不调用 Unitree SDK，也不产生任何运动命令。
 */

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

/*
 * 与两个真实 Action 执行路径的 smoke 身份校验对应。保留在 ELF 常量区，
 * 避免把 /usr/bin/true 或真实 SDK helper 误当成无硬件测试 helper。
 */
static const char kSmokeHelperIdentity[] =
    "RK_NON_ARM_TEST_ONLY_FAKE_SDK_HELPER_V1";


static long read_sleep_milliseconds(void)
{
  const char *raw = getenv("RK_FAKE_SDK_SLEEP_MS");
  char *end = NULL;
  long value;

  if (raw == NULL || raw[0] == '\0') {
    return 350L;
  }
  errno = 0;
  value = strtol(raw, &end, 10);
  if (errno != 0 || end == raw || *end != '\0' || value < 250L || value > 60000L) {
    return 350L;
  }
  return value;
}


int main(int argc, char **argv)
{
  const char *forced_exit = getenv("RK_FAKE_SDK_EXIT_CODE");
  struct timespec delay;
  long sleep_ms = read_sleep_milliseconds();
  int exit_code = 0;

  (void)argc;
  (void)argv;
  fprintf(stderr, "%s: no Unitree SDK or network operation is performed\n",
          kSmokeHelperIdentity);
  fflush(stderr);
  delay.tv_sec = sleep_ms / 1000L;
  delay.tv_nsec = (sleep_ms % 1000L) * 1000000L;
  while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {
    /* 终止信号会打断 sleep；保留默认信号语义以便 supervisor 清理进程组。 */
  }
  if (forced_exit != NULL && forced_exit[0] != '\0') {
    exit_code = atoi(forced_exit);
  }
  return exit_code;
}
