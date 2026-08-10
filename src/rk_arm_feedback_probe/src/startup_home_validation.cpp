#include "rk_arm_feedback_probe/startup_home_validation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rk_arm_feedback_probe {
namespace {

ChannelStatistics Calculate(const std::vector<StartupHomeValidator::TimedValues>& samples,
                            std::size_t channel) {
  ChannelStatistics result;
  if (samples.empty()) return result;

  std::vector<double> values;
  values.reserve(samples.size());
  double sum = 0.0;
  double max_jump = 0.0;
  std::optional<double> previous;
  for (const auto& sample : samples) {
    const double value = sample.values[channel];
    if (!std::isfinite(value)) continue;
    values.push_back(value);
    sum += value;
    if (previous) max_jump = std::max(max_jump, std::abs(value - *previous));
    previous = value;
  }
  if (values.empty()) return result;
  std::sort(values.begin(), values.end());
  result.frames = values.size();
  result.mean = sum / static_cast<double>(values.size());
  result.min = values.front();
  result.max = values.back();
  result.peak_to_peak = result.max - result.min;
  result.max_jump = max_jump;
  const std::size_t middle = values.size() / 2;
  result.median = values.size() % 2 == 0 ? (values[middle - 1] + values[middle]) / 2.0
                                          : values[middle];
  return result;
}

bool RecentAnglesInTolerance(const std::vector<StartupHomeValidator::TimedValues>& samples,
                             std::size_t count, const StartupHomeSummary& summary) {
  if (samples.size() < count) return false;
  for (std::size_t index = samples.size() - count; index < samples.size(); ++index) {
    for (std::size_t channel = 0; channel < kStartupHomeTargets.size(); ++channel) {
      if (std::abs(samples[index].values[channel] - kStartupHomeTargets[channel]) >
          summary.home_tolerance[channel]) return false;
    }
  }
  return true;
}

}  // namespace

void StartupHomeValidator::RecordAngles(const SevenValues& values, std::int64_t timestamp_ns) {
  angles_.push_back({values, timestamp_ns});
}

void StartupHomeValidator::RecordServos(const SevenValues& values, std::int64_t timestamp_ns) {
  servos_.push_back({values, timestamp_ns});
}

void StartupHomeValidator::RecordStatus(int enable_status, int power_status, int error_status) {
  statuses_.push_back({enable_status, power_status, error_status});
}

StartupHomeSummary StartupHomeValidator::Summarize(double angle_feedback_hz,
                                                    std::int64_t pair_max_age_ns) const {
  StartupHomeSummary summary;
  for (std::size_t channel = 0; channel < kStartupHomeTargets.size(); ++channel) {
    summary.angle_stats[channel] = Calculate(angles_, channel);
    summary.servo_stats[channel] = Calculate(servos_, channel);
    const double uncapped = std::max(summary.angle_stats[channel].peak_to_peak * 2.0, 0.2);
    summary.needs_manual_check[channel] = uncapped > 0.5;
    summary.home_tolerance[channel] = std::min(uncapped, 0.5);
  }

  // 两组帧按接收时间单调合并。仅比较时间窗内最近的另一源，避免广播频率差导致误判。
  std::size_t servo_index = 0;
  for (const auto& angle : angles_) {
    while (servo_index + 1 < servos_.size() &&
           servos_[servo_index + 1].timestamp_ns <= angle.timestamp_ns) ++servo_index;
    if (servos_.empty()) break;
    const TimedValues* nearest = &servos_[servo_index];
    if (servo_index + 1 < servos_.size()) {
      const auto& candidate = servos_[servo_index + 1];
      if (std::llabs(candidate.timestamp_ns - angle.timestamp_ns) <
          std::llabs(nearest->timestamp_ns - angle.timestamp_ns)) nearest = &candidate;
    }
    const std::int64_t age_ns = std::llabs(nearest->timestamp_ns - angle.timestamp_ns);
    if (age_ns > pair_max_age_ns) continue;
    ++summary.paired_frames;
    summary.max_pair_age_ms = std::max(summary.max_pair_age_ms,
                                       static_cast<double>(age_ns) / 1000000.0);
    for (std::size_t channel = 0; channel < kStartupHomeTargets.size(); ++channel) {
      summary.max_source_difference[channel] = std::max(
          summary.max_source_difference[channel],
          std::abs(angle.values[channel] - nearest->values[channel]));
    }
  }

  summary.status.frames = statuses_.size();
  for (const auto& status : statuses_) {
    const bool normal = status[0] == 1 && status[1] == 0 && status[2] == 0;
    if (normal) ++summary.status.normal_frames;
    else ++summary.status.abnormal_frames;
  }
  if (!statuses_.empty()) summary.status.last = statuses_.back();

  summary.source_consistent = summary.paired_frames > 0;
  for (const double difference : summary.max_source_difference) {
    if (difference > 0.5) summary.source_consistent = false;
  }
  const int sampled_half_second = static_cast<int>(std::ceil(std::max(0.0, angle_feedback_hz) * 0.5));
  summary.recommended_confirmation_frames = std::max(10, sampled_half_second);

  const bool complete_sources = !angles_.empty() && !servos_.empty();
  const bool normal_status = summary.status.frames > 0 && summary.status.abnormal_frames == 0;
  const bool tolerances_usable = std::none_of(summary.needs_manual_check.begin(),
                                               summary.needs_manual_check.end(),
                                               [](bool value) { return value; });
  summary.arm_home_confirmed = complete_sources && normal_status && tolerances_usable &&
                               summary.source_consistent &&
                               RecentAnglesInTolerance(angles_,
                                                       summary.recommended_confirmation_frames,
                                                       summary);
  return summary;
}

}  // namespace rk_arm_feedback_probe
