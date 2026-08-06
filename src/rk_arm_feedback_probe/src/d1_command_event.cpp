#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

/** 仅向本机 probe 发送人工标签；没有 Unitree SDK、DDS 或机器人控制能力。 */
int main(int argc, char** argv) {
  std::string socket_path{"/tmp/rk_d1_command_probe_events.sock"}; std::string event;
  for (int index = 1; index < argc; ++index) { const std::string item(argv[index]); if (item == "--socket" && index + 1 < argc) socket_path = argv[++index]; else if (event.empty()) event = item; else { std::cerr << "Usage: " << argv[0] << " [--socket PATH] EVENT\n"; return 2; } }
  if (event.empty() || socket_path.size() >= sizeof(sockaddr_un::sun_path)) return 2;
  const int fd = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC, 0); if (fd < 0) return 3;
  sockaddr_un address{}; address.sun_family = AF_UNIX; std::strncpy(address.sun_path, socket_path.c_str(), sizeof(address.sun_path) - 1);
  const bool sent = ::sendto(fd, event.data(), event.size(), 0, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) == static_cast<ssize_t>(event.size()); ::close(fd);
  if (!sent) { std::cerr << "Event probe socket is unavailable\n"; return 4; }
  return 0;
}
