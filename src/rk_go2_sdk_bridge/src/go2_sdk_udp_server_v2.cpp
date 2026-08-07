/** SDK_BRIDGE_RELIABILITY_V2 — 多线程SDK UDP服务器

Thread A (UDP Receiver): 持续recvfrom, 验证session/seq, 写latest_command
Thread B (SDK Motion):    固定50Hz, 读latest_command → SportClient.Move/StopMove
Thread C (Health/Diag):   每5秒打印诊断
*/

#include "rk_go2_sdk_bridge/udp_motion_core.hpp"

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>

#include <arpa/inet.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

namespace {

using rk_go2_sdk_bridge::MotionAction;
using rk_go2_sdk_bridge::MotionDecision;
using rk_go2_sdk_bridge::MotionLimits;
using rk_go2_sdk_bridge::UdpMotionCore;
using unitree::robot::ChannelFactory;
using unitree::robot::go2::SportClient;

volatile std::sig_atomic_t g_running = 1;

void SignalHandler(int) { g_running = 0; }

// ============================================================================
// Config
// ============================================================================

struct ServerConfig {
    std::string network_interface{"eth0"};
    std::string listen_ip{"127.0.0.1"};
    int port{15001};
    double rate_hz{50.0};
    double watchdog_sec{0.30};
    MotionLimits limits{};
};

// ============================================================================
// Shared State (between UDP thread and Main motion loop)
// ============================================================================

struct SharedState {
    std::mutex mtx;
    uint64_t current_session;
    uint64_t last_seq{0};
    double last_packet_time{0.0};
    int packets_received{0};
    int packets_dropped{0};
    bool armed{false};
    bool estop{false};
    double cmd_vx{0.0}, cmd_vy{0.0}, cmd_wz{0.0};
    int sdk_total_calls{0};
    int sdk_total_failures{0};
    int sdk_consecutive_failures{0};
    double sdk_last_success_time{0.0};

    void reset_for_new_session(uint64_t new_session) {
        current_session = new_session;
        last_seq = 0;
        cmd_vx = cmd_vy = cmd_wz = 0.0;
        armed = false;
        estop = false;
    }
};

// ============================================================================
// UDP Receiver Thread
// ============================================================================

void UdpReceiverLoop(int sock, SharedState& state, const MotionLimits& limits,
                     const std::string& listen_ip, int port) {
    char buffer[512];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    (void)listen_ip; (void)port;

    while (g_running) {
        ssize_t n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                             (struct sockaddr*)&client_addr, &addr_len);
        if (n <= 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(1000); continue;
            }
            break;
        }
        buffer[n] = '\0';

        double now_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        // Parse: try v2 format first, fallback to v1
        int version = 0;
        uint64_t session = 0, seq = 0;
        int64_t mono_ns = 0;
        double vx = 0, vy = 0, wz = 0;
        int flags = 0;

        std::istringstream ss(buffer);
        int parsed = 0;
        if (ss >> version >> session >> seq >> mono_ns >> vx >> vy >> wz >> flags) {
            parsed = 8;
        } else {
            // Fallback: v1 "vx vy wz"
            std::istringstream old(buffer);
            if (old >> vx >> vy >> wz) {
                parsed = 3;
                version = 1;
            }
        }

        if (parsed == 0) {
            std::cerr << "[UDP_RX] unparseable: " << buffer << std::endl;
            continue;
        }

        std::lock_guard<std::mutex> lock(state.mtx);
        state.packets_received++;

        // --- Validate ---
        if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz)) {
            state.cmd_vx = state.cmd_vy = state.cmd_wz = 0.0;
            state.estop = true;
            state.armed = false;
            std::cerr << "[UDP_RX] NaN packet → ESTOP seq=" << seq << std::endl;
            continue;
        }

        // Session management
        if (version >= 2 && session != 0 && session != state.current_session) {
            state.reset_for_new_session(session);
            std::cout << "[UDP_RX] new session=" << session << " → SAFE_IDLE" << std::endl;
        }

        // Emergency stop flag
        if ((version >= 2) && (flags & 0x02)) {
            state.cmd_vx = state.cmd_vy = state.cmd_wz = 0.0;
            state.estop = true;
            state.armed = false;
            std::cout << "[UDP_RX] ESTOP flag" << std::endl;
            continue;
        }

        // Out of range → STOP
        if (std::fabs(vx) > limits.max_vx ||
            std::fabs(vy) > limits.max_vy ||
            std::fabs(wz) > limits.max_yaw) {
            state.cmd_vx = state.cmd_vy = state.cmd_wz = 0.0;
            std::cerr << "[UDP_RX] OOR vx=" << vx << " vy=" << vy
                      << " wz=" << wz << " → STOP" << std::endl;
            continue;
        }

        // Deadband
        double db = limits.deadband;
        if (std::fabs(vx) <= db) vx = 0.0;
        if (std::fabs(vy) <= db) vy = 0.0;
        if (std::fabs(wz) <= db) wz = 0.0;

        // Drop old/dup seq
        if (version >= 2 && seq != 0 && seq <= state.last_seq && state.last_seq > 0) {
            state.packets_dropped++;
        }

        // Accept
        state.last_seq = (version >= 2 && seq > 0) ? seq : state.last_seq;
        state.last_packet_time = now_sec;
        state.cmd_vx = vx;
        state.cmd_vy = vy;
        state.cmd_wz = wz;

        // Auto-arm on first valid non-zero
        if (!state.armed && (vx != 0.0 || vy != 0.0 || wz != 0.0)) {
            state.armed = true;
            state.estop = false;
            std::cout << "[UDP_RX] ARMED" << std::endl;
        }
    }
}

// ============================================================================
// Argument parsing
// ============================================================================

double ParseFiniteDouble(const std::string& raw, const std::string& name) {
    std::size_t consumed = 0;
    double value = std::stod(raw, &consumed);
    if (consumed != raw.size() || !std::isfinite(value))
        throw std::runtime_error(name + " must be finite");
    return value;
}

double ParsePositiveDouble(const std::string& raw, const std::string& name) {
    double value = ParseFiniteDouble(raw, name);
    if (value <= 0.0) throw std::runtime_error(name + " must be positive");
    return value;
}

int ParsePort(const std::string& raw) {
    std::size_t consumed = 0;
    long value = std::stol(raw, &consumed);
    if (consumed != raw.size() || value <= 0 || value > 65535)
        throw std::runtime_error("port 1..65535");
    return static_cast<int>(value);
}

ServerConfig ParseArgs(int argc, char** argv) {
    ServerConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string opt = argv[i];
        if (opt == "--help" || opt == "-h") { exit(0); }
        if (i + 1 >= argc) throw std::runtime_error("missing value for " + opt);
        std::string val = argv[++i];
        if (opt == "--interface") cfg.network_interface = val;
        else if (opt == "--listen-ip") cfg.listen_ip = val;
        else if (opt == "--port") cfg.port = ParsePort(val);
        else if (opt == "--rate-hz") cfg.rate_hz = ParsePositiveDouble(val, "rate_hz");
        else if (opt == "--watchdog-sec") cfg.watchdog_sec = ParsePositiveDouble(val, "watchdog_sec");
        else if (opt == "--max-vx") cfg.limits.max_vx = ParseFiniteDouble(val, "max_vx");
        else if (opt == "--max-vy") cfg.limits.max_vy = ParseFiniteDouble(val, "max_vy");
        else if (opt == "--max-yaw") cfg.limits.max_yaw = ParseFiniteDouble(val, "max_yaw");
        else if (opt == "--deadband") cfg.limits.deadband = ParseFiniteDouble(val, "deadband");
    }
    return cfg;
}

}  // namespace

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    auto cfg = ParseArgs(argc, argv);

    // ---- DDS Init (once) ----
    ChannelFactory::Instance()->Init(0, cfg.network_interface);

    // ---- Startup StopMove (retry up to 3 times) ----
    SportClient sport_client;
    sport_client.Init();
    int stop_ret = -1;
    for (int attempt = 1; attempt <= 3; ++attempt) {
        stop_ret = sport_client.StopMove();
        if (stop_ret == 0) break;
        usleep(attempt * 100000);  // 100ms, 250ms, 500ms backoff
    }
    if (stop_ret != 0) {
        std::cerr << "[SDK_V2] fatal: STARTUP_STOPMOVE_RETRY_EXHAUSTED ret="
                  << stop_ret << std::endl;
        return 1;
    }
    std::cout << "[SDK_V2] SportClient Init + StopMove OK" << std::endl;

    // ---- Socket ----
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.port);
    inet_pton(AF_INET, cfg.listen_ip.c_str(), &addr.sin_addr);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }

    // ---- Shared state ----
    SharedState state;
    state.current_session = std::chrono::steady_clock::now().time_since_epoch().count();
    std::cout << "[SDK_V2] listening " << cfg.listen_ip << ":" << cfg.port
              << " rate=" << cfg.rate_hz << "Hz watchdog=" << cfg.watchdog_sec
              << "s session=" << state.current_session << std::endl;

    // ---- Thread A: UDP Receiver ----
    std::thread udp_thread(UdpReceiverLoop, sock, std::ref(state),
                           std::ref(cfg.limits), cfg.listen_ip, cfg.port);

    // ---- Main Thread B: SDK Motion Loop ----
    double period_sec = 1.0 / cfg.rate_hz;
    auto last_health = std::chrono::steady_clock::now();
    auto last_sdk_call = std::chrono::steady_clock::now();
    bool was_moving = false;

    while (g_running) {
        auto loop_start = std::chrono::steady_clock::now();
        double now_sec = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        double vx, vy, wz;
        double pkt_age;
        bool fresh, armed, estop;
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            vx = state.cmd_vx; vy = state.cmd_vy; wz = state.cmd_wz;
            pkt_age = now_sec - state.last_packet_time;
            fresh = (pkt_age < cfg.watchdog_sec);
            armed = state.armed;
            estop = state.estop;
        }

        int ret = 0;
        const char* reason = "none";

        if (estop) {
            ret = sport_client.StopMove();
            reason = "estop";
        } else if (!fresh) {
            ret = sport_client.StopMove();
            reason = "watchdog_udp_rx";
            if (was_moving) {
                std::cerr << "[SDK_V2] watchdog triggered: pkt_age="
                          << pkt_age << "s > " << cfg.watchdog_sec << "s" << std::endl;
            }
        } else if (!armed) {
            ret = sport_client.StopMove();
            reason = "not_armed";
        } else if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
            if (was_moving) {
                ret = sport_client.StopMove();
                reason = "zero_command";
            }
        } else {
            ret = sport_client.Move(vx, vy, wz);
            reason = "move";
        }

        was_moving = (vx != 0.0 || vy != 0.0 || wz != 0.0);

        // Record SDK result
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.sdk_total_calls++;
            if (ret != 0) {
                state.sdk_total_failures++;
                state.sdk_consecutive_failures++;
            } else {
                state.sdk_consecutive_failures = 0;
                state.sdk_last_success_time = now_sec;
            }
        }

        // Health log every 5s
        auto now_tp = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now_tp - last_health).count() > 5.0) {
            last_health = now_tp;
            std::lock_guard<std::mutex> lock(state.mtx);
            std::cout << "[SDK_V2] HEALTH"
                      << " pkts=" << state.packets_received
                      << " drop=" << state.packets_dropped
                      << " sdk_calls=" << state.sdk_total_calls
                      << " sdk_fails=" << state.sdk_total_failures
                      << " consec_fails=" << state.sdk_consecutive_failures
                      << " pkt_age=" << pkt_age << "s"
                      << " reason=" << reason
                      << " ret=" << ret
                      << std::endl;
        }

        // Rate limit
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - loop_start).count();
        double sleep_t = period_sec - elapsed;
        if (sleep_t > 0) usleep(static_cast<int>(sleep_t * 1e6));
    }

    // Cleanup
    sport_client.StopMove();
    g_running = 0;
    udp_thread.join();
    close(sock);
    std::cout << "[SDK_V2] shutdown complete" << std::endl;
    return 0;
}
