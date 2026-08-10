#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace rk_arm_feedback_probe {

/** STARTUP_HOME_POSE 的显示单位目标值；分析过程不做任何单位转换。 */
using SevenValues = std::array<double, 7>;
inline constexpr SevenValues kStartupHomeTargets{0.0, -90.0, 90.0, 0.0, 0.0, 0.0, -19.0};

/** 单路静止采样的统计量，max_jump 只比较同一数据源的相邻有效帧。 */
struct ChannelStatistics {
  std::size_t frames{0};
  double mean{0.0};
  double median{0.0};
  double min{0.0};
  double max{0.0};
  double peak_to_peak{0.0};
  double max_jump{0.0};
};

/** 结束时使用的状态帧统计；只有已观测的 funcode=3 原始值会计入。 */
struct StatusStatistics {
  std::size_t frames{0};
  std::size_t normal_frames{0};
  std::size_t abnormal_frames{0};
  std::optional<std::array<int, 3>> last;
};

/** 离线统计结果；它不包含 DDS、writer 或任何硬件控制接口。 */
struct StartupHomeSummary {
  std::array<ChannelStatistics, 7> angle_stats;
  std::array<ChannelStatistics, 7> servo_stats;
  std::array<double, 7> max_source_difference{};
  std::array<double, 7> home_tolerance{};
  std::array<bool, 7> needs_manual_check{};
  std::size_t paired_frames{0};
  double max_pair_age_ms{0.0};
  StatusStatistics status;
  bool source_consistent{false};
  bool arm_home_confirmed{false};
  int recommended_confirmation_frames{10};
};

/**
 * 纯数据统计核心。
 *
 * DDS 回调只把已解析的值传进来；本类不会创建通信对象。两源仅在接收时间差不超过
 * pair_max_age_ns 时比较，避免把历史缓存与新反馈混为同一物理瞬间。
 */
class StartupHomeValidator {
 public:
  /** 带本机单调时钟的源帧；仅用于按接收时间匹配两种只读反馈。 */
  struct TimedValues {
    SevenValues values{};
    std::int64_t timestamp_ns{0};
  };

  void RecordAngles(const SevenValues& values, std::int64_t timestamp_ns);
  void RecordServos(const SevenValues& values, std::int64_t timestamp_ns);
  void RecordStatus(int enable_status, int power_status, int error_status);
  StartupHomeSummary Summarize(double angle_feedback_hz,
                               std::int64_t pair_max_age_ns = 250000000) const;

 private:
  std::vector<TimedValues> angles_;
  std::vector<TimedValues> servos_;
  std::vector<std::array<int, 3>> statuses_;
};

}  // namespace rk_arm_feedback_probe
