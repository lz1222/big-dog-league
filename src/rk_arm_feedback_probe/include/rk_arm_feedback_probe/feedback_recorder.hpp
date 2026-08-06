#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace rk_arm_feedback_probe {

/** 只读采集配置；全部参数仅影响本地文件和失效判定。 */
struct ProbeConfig {
  std::filesystem::path output_dir{"artifacts/d1_feedback_probe"};
  std::chrono::milliseconds stale_after{1000};
  std::size_t parser_max_payload_bytes{1024U * 1024U};
};

/** 每个 topic 的被动观测统计，不赋予任何机械臂字段物理语义。 */
struct TopicStats {
  std::uint64_t frames{0};
  std::uint64_t changed_frames{0};
  std::uint64_t bad_frames{0};
  bool received{false};
  bool stale{true};
  std::int64_t first_frame_wait_ns{-1};
  std::int64_t last_frame_age_ns{-1};
  double average_hz{0.0};
};

/** JSON 字段的观测摘要；路径和类型来自原始 payload，不映射成控制含义。 */
struct FieldSummary {
  std::set<std::string> types;
  std::uint64_t observations{0};
  std::uint64_t numeric_observations{0};
  double numeric_min{0.0};
  double numeric_max{0.0};
  std::size_t min_array_length{0};
  std::size_t max_array_length{0};
  bool saw_array{false};
};

/**
 * 反馈落盘与协议摘要核心。
 * 它不包含 Unitree 通信接口，因而离线单元测试也不可能触发硬件通信。
 */
class FeedbackRecorder {
 public:
  explicit FeedbackRecorder(ProbeConfig config);
  ~FeedbackRecorder();

  FeedbackRecorder(const FeedbackRecorder&) = delete;
  FeedbackRecorder& operator=(const FeedbackRecorder&) = delete;

  bool Open(std::string* error);
  /** 注册预期 reader，即使整个采集窗口没有到帧也保留 absent/stale 证据。 */
  void RegisterTopic(const std::string& topic);
  void RecordArmFeedback(const std::string& topic, const std::string& payload);
  void RecordServoAngles(const std::string& topic,
                         const std::array<float, 7>& values);
  void RefreshStaleStates();
  void Close();

  TopicStats GetTopicStats(const std::string& topic) const;
  std::map<std::string, FieldSummary> GetSchemaSummary() const;
  bool IsOpen() const;

 private:
  struct TopicState;
  void RecordTopicFrame(const std::string& topic, const std::string& fingerprint);
  void WriteSummaryLocked();

  ProbeConfig config_;
  std::chrono::steady_clock::time_point start_;
  mutable std::mutex mutex_;
  std::map<std::string, TopicState> topics_;
  std::map<std::string, FieldSummary> schema_;
  std::ofstream* arm_output_{nullptr};
  std::ofstream* servo_output_{nullptr};
};

/** 对单帧 JSON 做容错 schema 采样；非法数据只返回 false，不抛异常。 */
bool SummarizeJsonPayload(const std::string& payload,
                          std::size_t max_payload_bytes,
                          std::map<std::string, FieldSummary>* summary,
                          std::string* error);

}  // namespace rk_arm_feedback_probe
