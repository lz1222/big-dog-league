#include <cstdint>
#include <cstring>
#include <ctime>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/**
 * 仅向本机 probe 发送人工标签；发送端记录 CLOCK_MONOTONIC 纳秒时间。
 * 本工具没有 Unitree SDK、DDS 或机器人控制能力，事件仅用于离线时间对齐。
 */
int main(int argc, char** argv) {
  std::string socket_path{"/tmp/rk_d1_command_probe_events.sock"}; std::string event;
  for (int index = 1; index < argc; ++index) { const std::string item(argv[index]); if (item == "--socket" && index + 1 < argc) socket_path = argv[++index]; else if (event.empty()) event = item; else { std::cerr << "Usage: " << argv[0] << " [--socket PATH] EVENT\n"; return 2; } }
  if (event.empty() || socket_path.size() >= sizeof(sockaddr_un::sun_path)) return 2;
  timespec clock{}; if (::clock_gettime(CLOCK_MONOTONIC, &clock) != 0) return 3;
  const std::uint64_t monotonic_ns = static_cast<std::uint64_t>(clock.tv_sec) * 1000000000ULL + static_cast<std::uint64_t>(clock.tv_nsec);
  const std::string payload = event + "\t" + std::to_string(monotonic_ns);
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0); if (fd < 0) return 3;
  sockaddr_un address{}; address.sun_family = AF_UNIX; std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1);
  const bool sent = ::sendto(fd, payload.data(), payload.size(), 0, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == static_cast<ssize_t>(payload.size()); ::close(fd);
  if (!sent) { std::cerr << "Event probe socket is unavailable\n"; return 4; }
  std::cout << event << " source_monotonic_ns=" << monotonic_ns << '\n';
  return 0;
}
