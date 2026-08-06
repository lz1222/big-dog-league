#include "rk_arm_feedback_probe/feedback_recorder.hpp"

#include <array>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <string>
#include <thread>

using rk_arm_feedback_probe::FeedbackRecorder;
using rk_arm_feedback_probe::FieldSummary;
using rk_arm_feedback_probe::ProbeConfig;
using rk_arm_feedback_probe::SummarizeJsonPayload;

int main() {
  std::map<std::string, FieldSummary> summary;
  std::string error;
  assert(SummarizeJsonPayload("{\"a\":1,\"items\":[true,\"x\"]}", 4096, &summary, &error));
  assert(summary.count("$") == 1 && summary.count("$.a") == 1);
  assert(summary.at("$.items").min_array_length == 2);
  assert(SummarizeJsonPayload("[]", 4096, &summary, &error));
  assert(SummarizeJsonPayload("{\"switch\":1,\"vary\":[1]}", 4096, &summary, &error));
  assert(SummarizeJsonPayload("{\"switch\":\"text\",\"vary\":[1,2,3]}", 4096, &summary, &error));
  assert(summary.at("$.switch").types.count("number") == 1);
  assert(summary.at("$.switch").types.count("string") == 1);
  assert(summary.at("$.vary").min_array_length == 1);
  assert(summary.at("$.vary").max_array_length == 3);
  assert(!SummarizeJsonPayload("", 4096, &summary, &error));
  assert(!SummarizeJsonPayload("not-json", 4096, &summary, &error));
  assert(!SummarizeJsonPayload("{\"a\":NaN}", 4096, &summary, &error));
  assert(!SummarizeJsonPayload("{\"a\":1e9999}", 4096, &summary, &error));
  assert(!SummarizeJsonPayload(std::string(4097, 'x'), 4096, &summary, &error));
  assert(!SummarizeJsonPayload("{\"a\":[1,2,3]}", 4, &summary, &error));

  const auto temp = std::filesystem::temp_directory_path() / "rk_arm_feedback_probe_test";
  std::filesystem::remove_all(temp);
  ProbeConfig config;
  config.output_dir = temp;
  config.stale_after = std::chrono::milliseconds(1);
  FeedbackRecorder recorder(config);
  assert(recorder.Open(&error));
  recorder.RegisterTopic("rt/arm_Command");
  recorder.RecordArmCommand("rt/arm_Command", "{\"funcode\":1,\"data\":{\"id\":0,\"angle\":1}}");
  assert(recorder.RecordOperatorEvent("CALIBRATION_START", 123U, &error));
  assert(!recorder.RecordOperatorEvent("NOT_AN_ALLOWED_EVENT", 123U, &error));
  recorder.RecordArmFeedback("rt/arm_Feedback", "{\"unknown\":42}");
  recorder.RecordArmFeedback("rt/arm_Feedback", "{");
  recorder.RecordArmFeedback("rt/arm_Feedback", "not-json");
  recorder.RecordArmFeedback("rt/arm_Feedback", "{");
  recorder.RecordServoAngles("current_servo_angle", {
      0.0F, 1.0F, 2.0F, 3.0F, 4.0F, 5.0F,
      std::numeric_limits<float>::infinity()});
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  recorder.RefreshStaleStates();
  assert(recorder.GetTopicStats("rt/arm_Feedback").frames == 4);
  assert(recorder.GetTopicStats("rt/arm_Feedback").bad_frames == 3);
  assert(recorder.GetTopicStats("current_servo_angle").frames == 1);
  assert(recorder.GetTopicStats("current_servo_angle").stale);
  recorder.Close();
  assert(!recorder.IsOpen());
  assert(std::filesystem::exists(temp / "arm_feedback_raw.jsonl"));
  assert(std::filesystem::exists(temp / "arm_command_raw.jsonl"));
  assert(std::filesystem::exists(temp / "servo_angle_raw.csv"));
  assert(std::filesystem::exists(temp / "operator_events.jsonl"));
  assert(std::filesystem::exists(temp / "protocol_summary.json"));
  std::ifstream events(temp / "operator_events.jsonl");
  std::string event_line;
  std::getline(events, event_line);
  assert(event_line.find("\"event_source_monotonic_ns\":123") != std::string::npos);
  assert(event_line.find("\"host_monotonic_ns\":") != std::string::npos);
  assert(event_line.find("\"event\":\"CALIBRATION_START\"") != std::string::npos);
  std::filesystem::remove_all(temp);
  return 0;
}
