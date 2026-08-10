#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

constexpr double kReplyTimeoutSec = 15.0;

double ParseNonnegativeDouble(const char* raw, const std::string& name)
{
  const double value = std::stod(raw);
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error(name + " must be finite and nonnegative");
  }
  return value;
}

int ParsePort(const char* raw)
{
  const long value = std::stol(raw);
  if (value < 1 || value > 65535) {
    throw std::runtime_error("RK_GO2_SDK_UDP_PORT must be within 1..65535");
  }
  return static_cast<int>(value);
}

bool IsActionName(const std::string& action)
{
  return action == "balance_stand" || action == "static_walk" ||
      action == "trot_run" || action == "stand_up" ||
      action == "economic_gait" || action == "front_jump" ||
      action == "hello" || action == "wave" || action == "stretch" ||
      action == "blink_front_light_3" || action == "blink_light_3" ||
      action == "recovery_stand" || action == "stop_move";
}

std::string MakeRequestId()
{
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  return "helper-" + std::to_string(getpid()) + "-" + std::to_string(ticks);
}

struct RequestContract
{
  std::string payload;
  std::string expected_prefix;
};

RequestContract BuildRequest(
    const std::string& request_id, const std::string& action)
{
  // 兼容原命令行 action 名称，但步态与特殊动作都只发给唯一 UDP server；
  // helper 本身不再创建 SportClient，也不直接写 ClassicWalk/FreeWalk。
  if (action == "classic_walk" || action == "classic_walk_on") {
    return {"GAIT " + request_id + " CLASSIC",
            "GAIT_ACK " + request_id + " CLASSIC "};
  }
  if (action == "classic_walk_off") {
    return {"GAIT " + request_id + " HOLD",
            "GAIT_ACK " + request_id + " HOLD "};
  }
  if (action == "free_walk") {
    return {"GAIT " + request_id + " FREE",
            "GAIT_ACK " + request_id + " FREE "};
  }
  if (!IsActionName(action)) {
    throw std::runtime_error("unsupported action: " + action);
  }
  return {"ACTION " + request_id + " " + action,
          "ACTION_ACK " + request_id + " " + action + " "};
}

std::string RequestServer(const RequestContract& request)
{
  const char* configured_host = std::getenv("RK_GO2_SDK_UDP_HOST");
  const char* configured_port = std::getenv("RK_GO2_SDK_UDP_PORT");
  const std::string host = configured_host == nullptr
      ? "127.0.0.1" : configured_host;
  const int port = configured_port == nullptr ? 15001 : ParsePort(configured_port);

  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    throw std::runtime_error("socket failed: " +
                             std::string(std::strerror(errno)));
  }
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, host.c_str(), &destination.sin_addr) != 1) {
    close(socket_fd);
    throw std::runtime_error("invalid RK_GO2_SDK_UDP_HOST");
  }

  const ssize_t sent = sendto(
      socket_fd, request.payload.data(), request.payload.size(), 0,
      reinterpret_cast<const sockaddr*>(&destination), sizeof(destination));
  if (sent != static_cast<ssize_t>(request.payload.size())) {
    const std::string error = std::strerror(errno);
    close(socket_fd);
    throw std::runtime_error("UDP request failed: " + error);
  }

  // ACK 必须与 request_id、动作名同时匹配；单调时限阻止旧包或 server
  // 故障让上游无限持有特殊动作锁。
  const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(kReplyTimeoutSec);
  while (std::chrono::steady_clock::now() < deadline) {
    const auto remaining = deadline - std::chrono::steady_clock::now();
    const auto remaining_us =
        std::chrono::duration_cast<std::chrono::microseconds>(remaining);
    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(remaining_us.count() / 1000000);
    timeout.tv_usec = static_cast<suseconds_t>(remaining_us.count() % 1000000);
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);
    const int ready = select(
        socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0 && errno == EINTR) continue;
    if (ready <= 0) break;
    char buffer[256];
    const ssize_t received = recv(
        socket_fd, buffer, sizeof(buffer) - 1, 0);
    if (received < 0) continue;
    buffer[received] = '\0';
    const std::string reply(buffer, static_cast<std::size_t>(received));
    if (reply.rfind(request.expected_prefix, 0) == 0) {
      close(socket_fd);
      return reply.substr(request.expected_prefix.size());
    }
  }
  close(socket_fd);
  return "TIMEOUT";
}

void PrintUsage(const char* program)
{
  std::cerr
      << "Usage:\n"
      << "  " << program << " <network_interface> <action> [wait_sec]\n\n"
      << "The network_interface argument is retained for CLI compatibility; "
         "the single SDK owner is selected by RK_GO2_SDK_UDP_HOST/PORT.\n\n"
      << "Actions:\n"
      << "  stand_up | balance_stand | classic_walk | classic_walk_on | "
         "classic_walk_off | static_walk | trot_run | free_walk | "
         "economic_gait | front_jump | stretch | hello | wave | "
         "blink_front_light_3 | recovery_stand | stop_move\n";
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 3) {
    PrintUsage(argv[0]);
    return 2;
  }
  try {
    const std::string action = argv[2];
    const double wait_sec = argc >= 4
        ? ParseNonnegativeDouble(argv[3], "wait_sec") : 0.0;
    const std::string request_id = MakeRequestId();
    const RequestContract request = BuildRequest(request_id, action);
    std::cout << "Requesting action from single SDK owner: " << action
              << " compatibility_interface=" << argv[1] << std::endl;
    const std::string result = RequestServer(request);
    std::cout << "SDK owner action result: " << result << std::endl;
    if (wait_sec > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(wait_sec));
    }
    if (result == "READY") return 0;
    // 42 保留既有上游合同：特殊动作成功但经典步态 handback 未确认时继续锁车。
    return result == "HANDBACK_FAILED" ? 42 : 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << std::endl;
    PrintUsage(argv[0]);
    return 1;
  }
}
