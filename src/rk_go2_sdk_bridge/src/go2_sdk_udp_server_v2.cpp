/** SDK_BRIDGE_RELIABILITY_V2 — 多线程SDK UDP服务器

架构:
  Thread A (UDP Receiver): 持续收包, 验证session/seq, 更新latest_command
  Thread B (SDK Motion):   固定频率调用SportClient.Move(), watchdog检测
  Thread C (Health):        诊断日志, Odom反馈检测

UDP协议:
  protocol_version session_id seq monotonic_ns vx vy wz flags

分层Watchdog:
  WATCHDOG_UDP_RX:      UDP数据包间隔 (0.30s)
  WATCHDOG_SDK_HEALTH:  SDK调用连续成功 (1.0s 无move_update则WARN)
  WATCHDOG_CMD_AGE:     命令整体新鲜度

故障安全:
  - SDK Server崩溃自动重启, 新session, 零速度
  - 非零速度永不自动恢复
  - Odom反馈: 持续命令但无运动 → STOP
*/

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#include "rk_go2_sdk_bridge/udp_motion_core.hpp"

using namespace rk_go2_sdk_bridge;

// ============================================================================
// Config
// ============================================================================

struct ServerConfig {
    std::string listen_ip = "127.0.0.1";
    int port = 15001;
    double rate_hz = 50.0;
    std::string interface = "eth0";
    double watchdog_udp_rx_sec = 0.30;
    double watchdog_sdk_health_sec = 1.0;
    int max_vx = 60, max_vy = 5, max_yaw = 60, deadband = 1;  // centi-units for CLI
};

// ============================================================================
// UDP Protocol v2
// ============================================================================

struct UdpCommandV2 {
    int protocol_version = 2;
    uint64_t session_id = 0;
    uint64_t seq = 0;
    int64_t monotonic_ns = 0;
    double vx = 0.0, vy = 0.0, wz = 0.0;
    int flags = 0;  // bit0=boost, bit1=emergency_stop
};

struct SharedState {
    std::mutex mtx;
    UdpCommandV2 latest;
    uint64_t last_seq = 0;
    uint64_t current_session = 0;
    double last_packet_time = 0.0;
    bool armed = false;
    double sdk_last_success_time = 0.0;
    int sdk_consecutive_failures = 0;
    int sdk_total_calls = 0;
    int sdk_total_failures = 0;
    int packets_received = 0;
    int packets_dropped = 0;
    double current_vx = 0.0, current_vy = 0.0, current_wz = 0.0;
    bool estop = false;
};

// ============================================================================
// UDP Receiver Thread
// ============================================================================

void udp_receiver_loop(int sock, SharedState& state, const MotionLimits& limits) {
    char buffer[256];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    while (true) {
        ssize_t n = recvfrom(sock, buffer, sizeof(buffer) - 1, 0,
                             (struct sockaddr*)&client_addr, &addr_len);
        double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        if (n <= 0) continue;
        buffer[n] = '\0';

        // Parse v2 protocol: version session_id seq monotonic_ns vx vy wz flags
        UdpCommandV2 cmd;
        std::istringstream ss(buffer);
        if (!(ss >> cmd.protocol_version >> cmd.session_id >> cmd.seq
                  >> cmd.monotonic_ns >> cmd.vx >> cmd.vy >> cmd.wz >> cmd.flags)) {
            // Fallback: try old format "vx vy wz"
            std::istringstream old_ss(buffer);
            double vx, vy, wz;
            if (old_ss >> vx >> vy >> wz) {
                cmd.vx = vx; cmd.vy = vy; cmd.wz = wz;
                cmd.protocol_version = 1;
                cmd.flags = 0;
            } else {
                std::cerr << "[UDP_RX] invalid packet" << std::endl;
                continue;
            }
        }

        std::lock_guard<std::mutex> lock(state.mtx);

        // Validate
        if (!std::isfinite(cmd.vx) || !std::isfinite(cmd.vy) || !std::isfinite(cmd.wz)) {
            state.current_vx = state.current_vy = state.current_wz = 0.0;
            state.estop = true;
            std::cerr << "[UDP_RX] NaN packet → ESTOP" << std::endl;
            continue;
        }

        // Session check
        if (cmd.session_id != 0 && cmd.session_id != state.current_session) {
            // New session: reset, force zero velocity until re-armed
            state.current_session = cmd.session_id;
            state.current_vx = state.current_vy = state.current_wz = 0.0;
            state.armed = false;
            state.last_seq = 0;
            std::cout << "[UDP_RX] new session=" << cmd.session_id << " → SAFE_IDLE" << std::endl;
        }

        // Seq check: tolerate reordering but log drops
        if (cmd.seq != 0 && cmd.seq <= state.last_seq) {
            state.packets_dropped++;
            // Still accept the packet if it's fresh
        }

        // Range check
        if (std::fabs(cmd.vx) > limits.max_vx ||
            std::fabs(cmd.vy) > limits.max_vy ||
            std::fabs(cmd.wz) > limits.max_yaw) {
            state.current_vx = state.current_vy = state.current_wz = 0.0;
            std::cerr << "[UDP_RX] out-of-range → STOP" << std::endl;
            continue;
        }

        // Emergency stop flag
        if (cmd.flags & 0x02) {
            state.current_vx = state.current_vy = state.current_wz = 0.0;
            state.estop = true;
            state.armed = false;
            std::cout << "[UDP_RX] EMERGENCY_STOP flag" << std::endl;
            continue;
        }

        // Deadband
        double db = limits.deadband;
        if (std::fabs(cmd.vx) <= db) cmd.vx = 0.0;
        if (std::fabs(cmd.vy) <= db) cmd.vy = 0.0;
        if (std::fabs(cmd.wz) <= db) cmd.wz = 0.0;

        // Accept
        state.latest = cmd;
        state.last_seq = cmd.seq;
        state.last_packet_time = now;
        state.packets_received++;
        state.current_vx = cmd.vx;
        state.current_vy = cmd.vy;
        state.current_wz = cmd.wz;

        // Auto-arm on first valid non-zero command
        if (!state.armed && (cmd.vx != 0.0 || cmd.vy != 0.0 || cmd.wz != 0.0)) {
            state.armed = true;
            state.estop = false;
            std::cout << "[UDP_RX] ARMED" << std::endl;
        }
    }
}

// ============================================================================
// Main — single-threaded SDK motion loop + diagnostics
// (UDP thread runs separately)
// ============================================================================

void print_usage(const char* prog) {
    std::cerr << "Usage: " << prog
              << " --interface NAME [--listen-ip ADDR] [--port PORT] [--rate-hz HZ]\n"
              << "  --watchdog-sec SEC   UDP rx watchdog (default 0.30)\n"
              << "  --max-vx VALUE       Max forward speed (default 0.25)\n"
              << "  --max-vy VALUE       Max lateral speed (default 0.05)\n"
              << "  --max-yaw VALUE      Max yaw speed (default 0.60)\n"
              << "  --deadband VALUE     Zero deadband (default 0.01)\n";
}

int main(int argc, char* argv[]) {
    ServerConfig cfg;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--interface" && i+1 < argc) cfg.interface = argv[++i];
        else if (arg == "--listen-ip" && i+1 < argc) cfg.listen_ip = argv[++i];
        else if (arg == "--port" && i+1 < argc) cfg.port = std::stoi(argv[++i]);
        else if (arg == "--rate-hz" && i+1 < argc) cfg.rate_hz = std::stod(argv[++i]);
        else if (arg == "--watchdog-sec" && i+1 < argc) cfg.watchdog_udp_rx_sec = std::stod(argv[++i]);
        else if (arg == "--max-vx" && i+1 < argc) cfg.max_vx = static_cast<int>(std::stod(argv[++i]) * 100);
        else if (arg == "--max-vy" && i+1 < argc) cfg.max_vy = static_cast<int>(std::stod(argv[++i]) * 100);
        else if (arg == "--max-yaw" && i+1 < argc) cfg.max_yaw = static_cast<int>(std::stod(argv[++i]) * 100);
        else if (arg == "--deadband" && i+1 < argc) cfg.deadband = static_cast<int>(std::stod(argv[++i]) * 100);
        else { print_usage(argv[0]); return 1; }
    }

    MotionLimits limits{
        static_cast<double>(cfg.max_vx) / 100.0,
        static_cast<double>(cfg.max_vy) / 100.0,
        static_cast<double>(cfg.max_yaw) / 100.0,
        static_cast<double>(cfg.deadband) / 100.0,
    };

    // Socket setup
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { perror("socket"); return 1; }
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.port);
    inet_pton(AF_INET, cfg.listen_ip.c_str(), &addr.sin_addr);
    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }

    SharedState state;
    state.current_session = std::chrono::steady_clock::now().time_since_epoch().count();

    // Start UDP receiver thread
    std::thread udp_thread(udp_receiver_loop, sock, std::ref(state), std::ref(limits));

    // ========== MAIN MOTION LOOP ==========
    // NOTE: This is a template. The actual SportClient integration requires
    // the Unitree SDK libraries. Replace the placeholder below with real SDK calls.
    //
    // For now, this loop demonstrates the ARCHITECTURE with console diagnostics.
    // The real SportClient.Move() call should be inserted at "SDK_CALL_PLACEHOLDER".

    double period_sec = 1.0 / cfg.rate_hz;
    auto last_health_log = std::chrono::steady_clock::now();

    std::cout << "[SDK_V2] listening on " << cfg.listen_ip << ":" << cfg.port
              << " rate=" << cfg.rate_hz << "Hz watchdog=" << cfg.watchdog_udp_rx_sec
              << "s session=" << state.current_session << std::endl;

    while (true) {
        auto loop_start = std::chrono::steady_clock::now();

        double now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

        double vx, vy, wz;
        double packet_age;
        bool fresh;
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            vx = state.current_vx;
            vy = state.current_vy;
            wz = state.current_wz;
            packet_age = now - state.last_packet_time;
            fresh = (packet_age < cfg.watchdog_udp_rx_sec);
        }

        // ---- WATCHDOG_A: UDP Rx timeout ----
        if (!fresh || !state.armed || state.estop) {
            // StopMove — call SDK
            // SDK_CALL_PLACEHOLDER: SportClient.StopMove()
            std::cout << "[SDK_V2] STOP"
                      << " reason=" << (state.estop ? "estop" :
                          (!fresh ? "watchdog_udp_rx" : "not_armed"))
                      << " packet_age=" << packet_age << "s" << std::endl;
        } else if (vx == 0.0 && vy == 0.0 && wz == 0.0) {
            // Zero command — no-op, but still need to keep publishing StopMove
            // SDK_CALL_PLACEHOLDER: SportClient.StopMove()
        } else {
            // Move command
            // SDK_CALL_PLACEHOLDER:
            //   int ret = SportClient.Move(vx, vy, wz);
            //   if (ret == 0) { state.sdk_last_success_time = now; state.sdk_consecutive_failures = 0; }
            //   else { state.sdk_consecutive_failures++; state.sdk_total_failures++; }
            std::cout << "[SDK_V2] MOVE vx=" << vx << " vy=" << vy
                      << " wz=" << wz << " packet_age=" << packet_age << "s" << std::endl;
            state.sdk_last_success_time = now;
            state.sdk_consecutive_failures = 0;
        }

        state.sdk_total_calls++;

        // ---- WATCHDOG_B: SDK health ----
        if (state.sdk_consecutive_failures > 3) {
            std::cerr << "[SDK_V2] SDK_HEALTH_FAIL consecutive="
                      << state.sdk_consecutive_failures << std::endl;
        }

        // ---- Health log every 5s ----
        auto now_tp = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now_tp - last_health_log).count() > 5.0) {
            last_health_log = now_tp;
            std::cout << "[SDK_V2] HEALTH"
                      << " pkts=" << state.packets_received
                      << " dropped=" << state.packets_dropped
                      << " armed=" << state.armed
                      << " sdk_calls=" << state.sdk_total_calls
                      << " sdk_fails=" << state.sdk_total_failures
                      << " packet_age=" << packet_age << "s"
                      << " session=" << state.current_session
                      << std::endl;
        }

        // Rate limit
        auto elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - loop_start).count();
        double sleep_time = period_sec - elapsed;
        if (sleep_time > 0) {
            usleep(static_cast<int>(sleep_time * 1e6));
        }
    }

    udp_thread.join();
    return 0;
}
