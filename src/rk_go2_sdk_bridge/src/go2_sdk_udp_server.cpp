#include "rk_go2_sdk_bridge/udp_motion_core.hpp"
#include "rk_go2_sdk_bridge/startup_stop_retry_core.hpp"
#include "rk_go2_sdk_bridge/motion_status_protocol.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <string>
#include <sys/select.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace
{

using rk_go2_sdk_bridge::MotionAction;
using rk_go2_sdk_bridge::MotionDecision;
using rk_go2_sdk_bridge::MotionLimits;
using rk_go2_sdk_bridge::UdpMotionCore;

volatile std::sig_atomic_t g_running = 1;

struct ServerConfig
{
  std::string network_interface{"eth1"};
  std::string listen_ip{"127.0.0.1"};
  int port{15001};
  std::string status_ip{"127.0.0.1"};
  int status_port{15002};
  std::string server_instance_id;
  double rate_hz{20.0};
  double watchdog_sec{0.30};
  MotionLimits limits{};
};

void SignalHandler(int)
{
  // 信号处理器只修改原子标志，SDK 停车在正常线程中完成。
  g_running = 0;
}

double ParseFiniteDouble(const std::string& raw, const std::string& name)
{
  std::size_t consumed = 0;
  const double value = std::stod(raw, &consumed);
  if (consumed != raw.size() || !std::isfinite(value)) {
    throw std::runtime_error(name + " must be finite");
  }
  return value;
}

double ParsePositiveDouble(const std::string& raw, const std::string& name)
{
  const double value = ParseFiniteDouble(raw, name);
  if (value <= 0.0) {
    throw std::runtime_error(name + " must be positive");
  }
  return value;
}

int ParsePort(const std::string& raw)
{
  std::size_t consumed = 0;
  const long value = std::stol(raw, &consumed);
  if (consumed != raw.size() || value <= 0 || value > 65535) {
    throw std::runtime_error("port must be in range 1..65535");
  }
  return static_cast<int>(value);
}

void PrintUsage(const char* program)
{
  std::cout
      << "Usage: " << program << " [options]\n"
      << "  --interface NAME       SDK network interface (default: eth1)\n"
      << "  --listen-ip ADDRESS    UDP listen address (default: 127.0.0.1)\n"
      << "  --port PORT            UDP port (default: 15001)\n"
      << "  --status-ip ADDRESS    Status UDP destination (default: 127.0.0.1)\n"
      << "  --status-port PORT     Status UDP destination port (default: 15002)\n"
      << "  --server-instance-id ID  Startup nonce copied to every status event\n"
      << "  --rate-hz HZ           SDK Move output rate (default: 20)\n"
      << "  --watchdog-sec SEC     Stop timeout (default: 0.30)\n"
      << "  --max-vx VALUE         Maximum |vx| (default: 0.25)\n"
      << "  --max-vy VALUE         Maximum |vy| (default: 0.05)\n"
      << "  --max-yaw VALUE        Maximum |yaw| (default: 0.60)\n"
      << "  --deadband VALUE       Zero deadband (default: 0.01)\n";
}

ServerConfig ParseArguments(int argc, char** argv)
{
  ServerConfig config;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option == "--help" || option == "-h") {
      PrintUsage(argv[0]);
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::runtime_error("missing value for " + option);
    }

    const std::string value = argv[++index];
    if (option == "--interface") {
      config.network_interface = value;
    } else if (option == "--listen-ip") {
      config.listen_ip = value;
    } else if (option == "--port") {
      config.port = ParsePort(value);
    } else if (option == "--status-ip") {
      config.status_ip = value;
    } else if (option == "--status-port") {
      config.status_port = ParsePort(value);
    } else if (option == "--server-instance-id") {
      config.server_instance_id = value;
    } else if (option == "--rate-hz") {
      config.rate_hz = ParsePositiveDouble(value, "rate_hz");
    } else if (option == "--watchdog-sec") {
      config.watchdog_sec = ParsePositiveDouble(value, "watchdog_sec");
    } else if (option == "--max-vx") {
      config.limits.max_vx = ParsePositiveDouble(value, "max_vx");
    } else if (option == "--max-vy") {
      config.limits.max_vy = ParsePositiveDouble(value, "max_vy");
    } else if (option == "--max-yaw") {
      config.limits.max_yaw = ParsePositiveDouble(value, "max_yaw");
    } else if (option == "--deadband") {
      config.limits.deadband = ParseFiniteDouble(value, "deadband");
      if (config.limits.deadband < 0.0) {
        throw std::runtime_error("deadband must be nonnegative");
      }
    } else {
      throw std::runtime_error("unknown option: " + option);
    }
  }

  if (config.network_interface.empty() || config.listen_ip.empty() ||
      config.status_ip.empty()) {
    throw std::runtime_error("interface, listen-ip and status-ip must not be empty");
  }
  if (config.server_instance_id.size() > 128U) {
    throw std::runtime_error("server-instance-id must not exceed 128 bytes");
  }
  return config;
}

double WallTimeSeconds()
{
  return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
}

std::string EnvironmentValue(const char* name)
{
  const char* value = std::getenv(name);
  return value == nullptr ? "<unset>" : value;
}

std::uint64_t Fingerprint(const std::string& value)
{
  // FNV-1a 只用于比对启动环境，不输出完整的库搜索路径。
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char character : value) {
    hash ^= character;
    hash *= 1099511628211ULL;
  }
  return hash;
}

void LogLoadedDdsLibraries()
{
  // /proc/self/maps 是动态加载器的实际结果，可避免日志只记录期望路径。
  std::ifstream maps("/proc/self/maps");
  std::string line;
  while (std::getline(maps, line)) {
    if (line.find("/libddsc.so") != std::string::npos ||
        line.find("/libddscxx.so") != std::string::npos) {
      const std::size_t path_begin = line.find('/');
      if (path_begin != std::string::npos) {
        std::cout << "[SDK] runtime loaded_dds_library="
                  << line.substr(path_begin) << std::endl;
      }
    }
  }
}

void LogRuntimeDiagnostics(const ServerConfig& config)
{
  const std::string library_path = EnvironmentValue("LD_LIBRARY_PATH");
  std::cout << "[SDK] runtime pid=" << getpid()
            << " executable=/proc/self/exe"
            << " interface=" << config.network_interface
            << " channel_factory_domain=0"
            << " ld_library_path_fingerprint=0x" << std::hex
            << Fingerprint(library_path) << std::dec
            << " cyclonedds_uri=" << EnvironmentValue("CYCLONEDDS_URI")
            << std::endl;
  LogLoadedDdsLibraries();
}

double MonotonicSeconds(
    const std::chrono::steady_clock::time_point& start_time)
{
  return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - start_time).count();
}

void LogDecision(const std::string& prefix, const MotionDecision& decision)
{
  std::cout << std::fixed << std::setprecision(6)
            << "[SDK] time=" << WallTimeSeconds() << " " << prefix
            << " action=" << rk_go2_sdk_bridge::ToString(decision.action)
            << " reason=" << decision.reason
            << " vx=" << decision.command.vx
            << " vy=" << decision.command.vy
            << " yaw=" << decision.command.yaw << std::endl;
}

class MotionStatusPublisher
{
public:
  explicit MotionStatusPublisher(const ServerConfig& config)
  : server_instance_id_(ResolveServerInstanceId(config.server_instance_id))
  {
    // status socket 只读观测失败必须可见，但不得改变 SDK 调用或停车分支。
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
      LogUnavailable("socket", errno);
      return;
    }
    destination_.sin_family = AF_INET;
    destination_.sin_port = htons(static_cast<uint16_t>(config.status_port));
    if (inet_pton(
        AF_INET, config.status_ip.c_str(), &destination_.sin_addr) != 1) {
      std::cerr << "[SDK_STATUS] disabled invalid_status_ip="
                << config.status_ip << std::endl;
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  ~MotionStatusPublisher()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
    }
  }

  MotionStatusPublisher(const MotionStatusPublisher&) = delete;
  MotionStatusPublisher& operator=(const MotionStatusPublisher&) = delete;

  const std::string& ServerInstanceId() const
  {
    return server_instance_id_;
  }

  void Publish(
      const std::string& event, int32_t ret, const std::string& reason,
      double vx, double vy, double yaw)
  {
    // sequence 在发送尝试前递增，接收端可识别发送失败后的状态缺口。
    const rk_go2_sdk_bridge::MotionStatusEvent status{
        server_instance_id_, ++sequence_, event, ret, reason, vx, vy, yaw,
        MonotonicNanoseconds()};
    const std::string payload = rk_go2_sdk_bridge::EncodeMotionStatusJson(status);
    if (payload.empty()) {
      std::cerr << "[SDK_STATUS] encode_failed sequence=" << status.sequence
                << " event=" << event << std::endl;
      return;
    }
    if (socket_fd_ < 0) {
      std::cerr << "[SDK_STATUS] send_skipped disabled sequence="
                << status.sequence << " event=" << event << std::endl;
      return;
    }
    const ssize_t sent = sendto(
        socket_fd_, payload.data(), payload.size(), 0,
        reinterpret_cast<const sockaddr*>(&destination_), sizeof(destination_));
    if (sent != static_cast<ssize_t>(payload.size())) {
      LogUnavailable("sendto", errno);
    }
  }

private:
  static std::string ResolveServerInstanceId(const std::string& configured)
  {
    if (!configured.empty()) {
      return configured;
    }
    // 非正式直接启动也必须拥有实例身份；正式入口会传入更强的 UUID nonce。
    return "auto-" + std::to_string(getpid()) + "-" +
           std::to_string(MonotonicNanoseconds());
  }

  static int64_t MonotonicNanoseconds()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  static void LogUnavailable(const char* operation, int error_number)
  {
    std::cerr << "[SDK_STATUS] " << operation << " failed: "
              << std::strerror(error_number) << std::endl;
  }

  int socket_fd_{-1};
  sockaddr_in destination_{};
  const std::string server_instance_id_;
  uint64_t sequence_{0};
};

int32_t SendStop(
    unitree::robot::go2::SportClient& client, MotionStatusPublisher& status,
    const std::string& reason)
{
  const int32_t result = client.StopMove();
  // 状态必须在真实 StopMove 返回后发送，不能把调用前的意图伪装成成功。
  status.Publish("STOP_MOVE", result, reason, 0.0, 0.0, 0.0);
  if (result != 0) {
    status.Publish("SDK_ERROR", result, "stop_move_error", 0.0, 0.0, 0.0);
  }
  std::cout << std::fixed << std::setprecision(6)
            << "[SDK] time=" << WallTimeSeconds()
            << " StopMove reason=" << reason
            << " ret=" << result << std::endl;
  return result;
}

// SDK 已确认可控后，任何异常离开作用域都会再次尝试停车。启动门禁失败前
// 不 arm，避免把“启动 StopMove 最多三次”的上限悄悄变成第四次动作调用。
class EmergencyStopGuard
{
public:
  EmergencyStopGuard(
      unitree::robot::go2::SportClient& client,
      MotionStatusPublisher& status)
  : client_(client), status_(status)
  {
  }

  ~EmergencyStopGuard()
  {
    if (armed_) {
      SendStop(client_, status_, "exception_or_unexpected_exit");
    }
  }

  void Arm()
  {
    armed_ = true;
  }

  void Disarm()
  {
    armed_ = false;
  }

private:
  unitree::robot::go2::SportClient& client_;
  MotionStatusPublisher& status_;
  bool armed_{false};
};

int32_t SendStartupStopWithRetry(
    unitree::robot::go2::SportClient& client, MotionStatusPublisher& status)
{
  // 这只是控制面已经就绪后的第二层保护，不能替代只读 DDS 门禁。
  rk_go2_sdk_bridge::StartupStopRetryCore retry_core;
  int32_t result = -1;
  while (true) {
    if (retry_core.NextAttempt() == 1) {
      std::cout << "CONTROL_PLANE_DIAG event=FIRST_STOPMOVE_ATTEMPT"
                << " time=" << WallTimeSeconds()
                << std::endl;
    }
    const auto started = std::chrono::steady_clock::now();
    result = client.StopMove();
    status.Publish("STARTUP_STOP", result, "startup_stop", 0.0, 0.0, 0.0);
    if (result != 0) {
      status.Publish("SDK_ERROR", result, "startup_stop_error", 0.0, 0.0, 0.0);
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - started).count();
    const auto decision = retry_core.RecordResult(result);
    std::cout << "SDK_STARTUP_DIAG attempt=" << decision.attempt
              << " ret=" << result
              << " elapsed_ms=" << elapsed_ms << std::endl;
    if (decision.success) {
      std::cout << "CONTROL_PLANE_DIAG event=FIRST_STOPMOVE_SUCCESS"
                << " time=" << WallTimeSeconds()
                << " attempt=" << decision.attempt << std::endl;
      std::cout << "SDK_STARTUP_DIAG classification=SUCCESS attempts="
                << decision.attempt << std::endl;
      return 0;
    }
    if (decision.retry) {
      std::cout << "SDK_STARTUP_DIAG backoff_ms=" << decision.backoff_ms
                << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(decision.backoff_ms));
      continue;
    }
    std::cerr << "SDK_STARTUP_DIAG classification=FAILED attempts="
              << decision.attempt << " final_ret=" << result << std::endl;
    return result;
  }
}

class SocketGuard
{
public:
  explicit SocketGuard(int fd) : fd_(fd) {}
  ~SocketGuard()
  {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  SocketGuard(const SocketGuard&) = delete;
  SocketGuard& operator=(const SocketGuard&) = delete;

private:
  int fd_;
};

void ExecuteDecision(
    unitree::robot::go2::SportClient& client,
    MotionStatusPublisher& status,
    UdpMotionCore& core,
    const MotionDecision& decision)
{
  if (decision.action == MotionAction::kNone) {
    return;
  }

  if (decision.action == MotionAction::kStop) {
    const int32_t result = SendStop(client, status, decision.reason);
    if (result != 0) {
      throw std::runtime_error(
          "StopMove failed with code " + std::to_string(result));
    }
    return;
  }

  const int32_t result = client.Move(
      static_cast<float>(decision.command.vx),
      static_cast<float>(decision.command.vy),
      static_cast<float>(decision.command.yaw));
  status.Publish(
      "MOVE", result, decision.reason, decision.command.vx,
      decision.command.vy, decision.command.yaw);
  std::cout << std::fixed << std::setprecision(6)
            << "[SDK] time=" << WallTimeSeconds()
            << " Move vx=" << decision.command.vx
            << " vy=" << decision.command.vy
            << " yaw=" << decision.command.yaw
            << " ret=" << result << std::endl;
  if (result != 0) {
    status.Publish(
        "SDK_ERROR", result, "sdk_move_error", decision.command.vx,
        decision.command.vy, decision.command.yaw);
    const MotionDecision stop = core.ForceStop("sdk_move_error");
    SendStop(client, status, stop.reason);
    throw std::runtime_error(
        "Move failed with code " + std::to_string(result));
  }
}

int CreateUdpSocket(const ServerConfig& config)
{
  const int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    throw std::runtime_error(
        "socket failed: " + std::string(std::strerror(errno)));
  }

  int reuse = 1;
  setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(static_cast<uint16_t>(config.port));
  if (inet_pton(AF_INET, config.listen_ip.c_str(), &address.sin_addr) != 1) {
    close(socket_fd);
    throw std::runtime_error("invalid listen-ip: " + config.listen_ip);
  }

  if (bind(
      socket_fd,
      reinterpret_cast<sockaddr*>(&address),
      sizeof(address)) < 0) {
    const std::string error = std::strerror(errno);
    close(socket_fd);
    throw std::runtime_error("bind failed: " + error);
  }
  return socket_fd;
}

int RunServer(const ServerConfig& config)
{
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  LogRuntimeDiagnostics(config);

  std::cout << "[SDK] Init ChannelFactory on "
            << config.network_interface << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(
      0, config.network_interface);

  // 正式控制链不申请 SDK motion lease；prearm 与已验收的 1003 路径均使用
  // false，避免默认构造语义随 SDK 版本变化而改变控制平面行为。
  unitree::robot::go2::SportClient client(false);
  client.SetTimeout(10.0F);
  const double init_started = WallTimeSeconds();
  client.Init();
  std::cout << std::fixed << std::setprecision(6)
            << "[SDK] time=" << WallTimeSeconds()
            << " SportClient::Init elapsed_sec="
            << (WallTimeSeconds() - init_started) << std::endl;
  MotionStatusPublisher status(config);
  EmergencyStopGuard stop_guard(client, status);

  // 启动时只清除残留运动，不调用 BalanceStand，避免擅自改变当前步态。
  // UDP socket 必须在此成功之后才可 bind，失败路径没有任何运动输入出口。
  if (SendStartupStopWithRetry(client, status) != 0) {
    throw std::runtime_error("STARTUP_STOPMOVE_RETRY_EXHAUSTED");
  }
  stop_guard.Arm();

  const int socket_fd = CreateUdpSocket(config);
  SocketGuard socket_guard(socket_fd);
  UdpMotionCore core(config.limits, config.watchdog_sec);

  std::cout << "[SDK] UDP server listening on "
            << config.listen_ip << ":" << config.port
            << " interface=" << config.network_interface
            << " rate_hz=" << config.rate_hz
            << " watchdog_sec=" << config.watchdog_sec
            << " max_vx=" << config.limits.max_vx
            << " max_vy=" << config.limits.max_vy
            << " max_yaw=" << config.limits.max_yaw
            << " deadband=" << config.limits.deadband
            << " status=" << config.status_ip << ":" << config.status_port
            << " server_instance_id=" << status.ServerInstanceId()
            << std::endl;

  const auto start_time = std::chrono::steady_clock::now();
  const auto period = std::chrono::duration<double>(1.0 / config.rate_hz);
  auto next_tick = std::chrono::steady_clock::now() + period;

  while (g_running) {
    const auto before_select = std::chrono::steady_clock::now();
    const auto wait_duration =
        next_tick > before_select
        ? next_tick - before_select
        : std::chrono::steady_clock::duration::zero();
    const auto wait_us =
        std::chrono::duration_cast<std::chrono::microseconds>(wait_duration);

    timeval timeout{};
    timeout.tv_sec = static_cast<time_t>(wait_us.count() / 1000000);
    timeout.tv_usec = static_cast<suseconds_t>(wait_us.count() % 1000000);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(socket_fd, &read_fds);

    const int ready = select(
        socket_fd + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready < 0) {
      if (errno == EINTR) {
        continue;
      }
      const MotionDecision stop = core.ForceStop("select_error");
      ExecuteDecision(client, status, core, stop);
      throw std::runtime_error(
          "select failed: " + std::string(std::strerror(errno)));
    }

    if (ready > 0 && FD_ISSET(socket_fd, &read_fds)) {
      char buffer[256];
      const ssize_t received = recvfrom(
          socket_fd, buffer, sizeof(buffer) - 1, MSG_TRUNC,
          nullptr, nullptr);
      if (received < 0) {
        const MotionDecision stop = core.ForceStop("recv_error");
        ExecuteDecision(client, status, core, stop);
        throw std::runtime_error(
            "recvfrom failed: " + std::string(std::strerror(errno)));
      }

      MotionDecision decision;
      std::string payload;
      if (received >= static_cast<ssize_t>(sizeof(buffer))) {
        decision = core.ForceStop("oversize_packet");
      } else {
        buffer[received] = '\0';
        payload.assign(buffer, static_cast<std::size_t>(received));
        decision = core.AcceptPacket(
            payload, MonotonicSeconds(start_time));
      }

      LogDecision("RX payload=\"" + payload + "\"", decision);
      ExecuteDecision(client, status, core, decision);
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= next_tick) {
      const MotionDecision decision =
          core.Tick(MonotonicSeconds(start_time));
      ExecuteDecision(client, status, core, decision);

      // 不补发已经错过的周期，避免调度抖动导致 SDK 突发调用。
      next_tick = now + period;
    }
  }

  if (SendStop(client, status, "signal_exit") != 0) {
    throw std::runtime_error("signal exit StopMove failed");
  }
  stop_guard.Disarm();
  std::cout << "[SDK] UDP server exited cleanly." << std::endl;
  return 0;
}

}  // namespace

int main(int argc, char** argv)
{
  try {
    return RunServer(ParseArguments(argc, argv));
  } catch (const std::exception& error) {
    const std::string message = error.what();
    if (message.find("STARTUP_STOPMOVE_RETRY_EXHAUSTED") != std::string::npos) {
      std::cerr << "SDK_STARTUP_DIAG classification="
                << "STARTUP_STOPMOVE_RETRY_EXHAUSTED" << std::endl;
    } else if (message.find("bind failed") != std::string::npos) {
      std::cerr << "SDK_STARTUP_DIAG classification=UDP_BIND_ERROR" << std::endl;
    }
    std::cerr << "[SDK] fatal: " << error.what() << std::endl;
    return 1;
  }
}
