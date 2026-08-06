#include "rk_arm_feedback_probe/feedback_recorder.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <variant>

namespace rk_arm_feedback_probe {
namespace {

struct JsonValue {
  using Array = std::vector<JsonValue>;
  using Object = std::map<std::string, JsonValue>;
  std::variant<std::nullptr_t, bool, double, std::string, Array, Object> value;
};

class JsonParser {
 public:
  explicit JsonParser(const std::string& text) : text_(text) {}

  bool Parse(JsonValue* output, std::string* error) {
    SkipWhitespace();
    if (!ParseValue(output, error)) return false;
    SkipWhitespace();
    if (pos_ != text_.size()) {
      *error = "trailing characters after JSON value";
      return false;
    }
    return true;
  }

 private:
  void SkipWhitespace() {
    while (pos_ < text_.size() &&
           (text_[pos_] == ' ' || text_[pos_] == '\n' || text_[pos_] == '\r' || text_[pos_] == '\t')) ++pos_;
  }
  bool Consume(char token) {
    SkipWhitespace();
    if (pos_ < text_.size() && text_[pos_] == token) { ++pos_; return true; }
    return false;
  }
  bool ParseValue(JsonValue* output, std::string* error) {
    SkipWhitespace();
    if (pos_ == text_.size()) { *error = "empty JSON value"; return false; }
    const char token = text_[pos_];
    if (token == '{') return ParseObject(output, error);
    if (token == '[') return ParseArray(output, error);
    if (token == '"') { std::string value; if (!ParseString(&value, error)) return false; output->value = value; return true; }
    if (text_.compare(pos_, 4, "true") == 0) { pos_ += 4; output->value = true; return true; }
    if (text_.compare(pos_, 5, "false") == 0) { pos_ += 5; output->value = false; return true; }
    if (text_.compare(pos_, 4, "null") == 0) { pos_ += 4; output->value = nullptr; return true; }
    return ParseNumber(output, error);
  }
  bool ParseString(std::string* output, std::string* error) {
    if (pos_ == text_.size() || text_[pos_++] != '"') { *error = "expected string"; return false; }
    while (pos_ < text_.size()) {
      const char ch = text_[pos_++];
      if (ch == '"') return true;
      if (static_cast<unsigned char>(ch) < 0x20) { *error = "control character in string"; return false; }
      if (ch != '\\') { output->push_back(ch); continue; }
      if (pos_ == text_.size()) { *error = "unterminated string escape"; return false; }
      const char escaped = text_[pos_++];
      switch (escaped) {
        case '"': output->push_back('"'); break; case '\\': output->push_back('\\'); break;
        case '/': output->push_back('/'); break; case 'b': output->push_back('\b'); break;
        case 'f': output->push_back('\f'); break; case 'n': output->push_back('\n'); break;
        case 'r': output->push_back('\r'); break; case 't': output->push_back('\t'); break;
        case 'u':
          if (pos_ + 4 > text_.size()) { *error = "short unicode escape"; return false; }
          // 原始 payload 已完整保存；摘要中以占位符表示 Unicode 转义，避免错误解码改变证据。
          output->append("<u>"); pos_ += 4; break;
        default: *error = "invalid string escape"; return false;
      }
    }
    *error = "unterminated string"; return false;
  }
  bool ParseNumber(JsonValue* output, std::string* error) {
    const std::size_t begin = pos_;
    if (pos_ < text_.size() && text_[pos_] == '-') ++pos_;
    if (pos_ == text_.size()) { *error = "invalid number"; return false; }
    if (text_[pos_] == '0') ++pos_;
    else {
      if (text_[pos_] < '1' || text_[pos_] > '9') { *error = "invalid value"; return false; }
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
    }
    if (pos_ < text_.size() && text_[pos_] == '.') {
      ++pos_; const std::size_t fraction = pos_;
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
      if (fraction == pos_) { *error = "invalid number fraction"; return false; }
    }
    if (pos_ < text_.size() && (text_[pos_] == 'e' || text_[pos_] == 'E')) {
      ++pos_; if (pos_ < text_.size() && (text_[pos_] == '+' || text_[pos_] == '-')) ++pos_;
      const std::size_t exponent = pos_;
      while (pos_ < text_.size() && text_[pos_] >= '0' && text_[pos_] <= '9') ++pos_;
      if (exponent == pos_) { *error = "invalid number exponent"; return false; }
    }
    try { output->value = std::stod(text_.substr(begin, pos_ - begin)); }
    catch (const std::exception&) { *error = "number conversion failed"; return false; }
    if (!std::isfinite(std::get<double>(output->value))) { *error = "non-finite JSON number"; return false; }
    return true;
  }
  bool ParseArray(JsonValue* output, std::string* error) {
    ++pos_; JsonValue::Array array; SkipWhitespace();
    if (Consume(']')) { output->value = std::move(array); return true; }
    while (true) {
      JsonValue element; if (!ParseValue(&element, error)) return false;
      array.push_back(std::move(element)); SkipWhitespace();
      if (Consume(']')) { output->value = std::move(array); return true; }
      if (!Consume(',')) { *error = "expected array separator"; return false; }
    }
  }
  bool ParseObject(JsonValue* output, std::string* error) {
    ++pos_; JsonValue::Object object; SkipWhitespace();
    if (Consume('}')) { output->value = std::move(object); return true; }
    while (true) {
      SkipWhitespace(); std::string key; if (!ParseString(&key, error)) return false;
      if (!Consume(':')) { *error = "expected object colon"; return false; }
      JsonValue element; if (!ParseValue(&element, error)) return false;
      object[key] = std::move(element); SkipWhitespace();
      if (Consume('}')) { output->value = std::move(object); return true; }
      if (!Consume(',')) { *error = "expected object separator"; return false; }
    }
  }
  const std::string& text_; std::size_t pos_{0};
};

std::string EscapeJson(const std::string& value) {
  std::ostringstream out;
  for (unsigned char ch : value) {
    switch (ch) {
      case '"': out << "\\\""; break; case '\\': out << "\\\\"; break;
      case '\n': out << "\\n"; break; case '\r': out << "\\r"; break; case '\t': out << "\\t"; break;
      default:
        if (ch < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(ch) << std::dec;
        else out << static_cast<char>(ch);
    }
  }
  return out.str();
}

std::string WallTime() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t value = std::chrono::system_clock::to_time_t(now);
  std::tm tm{}; gmtime_r(&value, &tm);
  std::ostringstream out; out << std::put_time(&tm, "%Y-%m-%dT%H:%M:%SZ"); return out.str();
}

std::string TypeName(const JsonValue& value) {
  if (std::holds_alternative<std::nullptr_t>(value.value)) return "null";
  if (std::holds_alternative<bool>(value.value)) return "boolean";
  if (std::holds_alternative<double>(value.value)) return "number";
  if (std::holds_alternative<std::string>(value.value)) return "string";
  if (std::holds_alternative<JsonValue::Array>(value.value)) return "array";
  return "object";
}

void ObserveValue(const JsonValue& value, const std::string& path,
                  std::map<std::string, FieldSummary>* summary) {
  FieldSummary& field = (*summary)[path]; field.observations++; field.types.insert(TypeName(value));
  if (const auto* number = std::get_if<double>(&value.value)) {
    if (std::isfinite(*number)) {
      if (field.numeric_observations++ == 0) field.numeric_min = field.numeric_max = *number;
      else { field.numeric_min = std::min(field.numeric_min, *number); field.numeric_max = std::max(field.numeric_max, *number); }
    }
  } else if (const auto* array = std::get_if<JsonValue::Array>(&value.value)) {
    const std::size_t size = array->size();
    if (!field.saw_array) { field.min_array_length = field.max_array_length = size; field.saw_array = true; }
    else { field.min_array_length = std::min(field.min_array_length, size); field.max_array_length = std::max(field.max_array_length, size); }
    for (std::size_t index = 0; index < array->size(); ++index) ObserveValue((*array)[index], path + "[" + std::to_string(index) + "]", summary);
  } else if (const auto* object = std::get_if<JsonValue::Object>(&value.value)) {
    for (const auto& [key, child] : *object) ObserveValue(child, path + "." + key, summary);
  }
}

}  // namespace

struct FeedbackRecorder::TopicState {
  TopicStats stats;
  std::chrono::steady_clock::time_point first;
  std::chrono::steady_clock::time_point last;
  std::string last_fingerprint;
};

FeedbackRecorder::FeedbackRecorder(ProbeConfig config) : config_(std::move(config)), start_(std::chrono::steady_clock::now()) {}
FeedbackRecorder::~FeedbackRecorder() { Close(); }

bool FeedbackRecorder::Open(std::string* error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (arm_output_ != nullptr) return true;
  std::error_code ec; std::filesystem::create_directories(config_.output_dir, ec);
  if (ec) { *error = "cannot create output directory: " + ec.message(); return false; }
  arm_output_ = new std::ofstream(config_.output_dir / "arm_feedback_raw.jsonl", std::ios::app);
  command_output_ = new std::ofstream(config_.output_dir / "arm_command_raw.jsonl", std::ios::app);
  servo_output_ = new std::ofstream(config_.output_dir / "servo_angle_raw.csv", std::ios::app);
  event_output_ = new std::ofstream(config_.output_dir / "operator_events.jsonl", std::ios::app);
  if (!*arm_output_ || !*command_output_ || !*servo_output_ || !*event_output_) {
    *error = "cannot open output files";
    delete arm_output_; delete command_output_; delete servo_output_; delete event_output_;
    arm_output_ = nullptr; command_output_ = nullptr; servo_output_ = nullptr; event_output_ = nullptr;
    return false;
  }
  if (std::filesystem::file_size(config_.output_dir / "servo_angle_raw.csv", ec) == 0) {
    *servo_output_ << "host_monotonic_ns,host_wall_time,topic,servo_0,servo_1,servo_2,servo_3,servo_4,servo_5,servo_6\n";
  }
  return true;
}

void FeedbackRecorder::RegisterTopic(const std::string& topic) {
  std::lock_guard<std::mutex> lock(mutex_);
  topics_.try_emplace(topic);
}

void FeedbackRecorder::RecordTopicFrame(const std::string& topic, const std::string& fingerprint) {
  const auto now = std::chrono::steady_clock::now(); TopicState& state = topics_[topic];
  if (!state.stats.received) { state.stats.received = true; state.first = now; state.stats.first_frame_wait_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - start_).count(); }
  if (state.stats.frames > 0 && fingerprint != state.last_fingerprint) ++state.stats.changed_frames;
  ++state.stats.frames; state.last = now; state.last_fingerprint = fingerprint; state.stats.stale = false;
  const double elapsed = std::chrono::duration<double>(now - state.first).count();
  state.stats.average_hz = elapsed > 0.0 ? static_cast<double>(state.stats.frames - 1) / elapsed : 0.0;
}

void FeedbackRecorder::RecordArmFeedback(const std::string& topic, const std::string& payload) {
  std::lock_guard<std::mutex> lock(mutex_); if (arm_output_ == nullptr) return;
  const auto monotonic = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  *arm_output_ << "{\"host_monotonic_ns\":" << monotonic << ",\"host_wall_time\":\"" << WallTime() << "\",\"topic\":\"" << EscapeJson(topic) << "\",\"payload_length\":" << payload.size() << ",\"payload_raw\":\"" << EscapeJson(payload) << "\"}\n";
  arm_output_->flush(); RecordTopicFrame(topic, payload);
  std::string error; if (!SummarizeJsonPayload(payload, config_.parser_max_payload_bytes, &schema_, &error)) ++topics_[topic].stats.bad_frames;
}

void FeedbackRecorder::RecordArmCommand(const std::string& topic, const std::string& payload) {
  std::lock_guard<std::mutex> lock(mutex_); if (command_output_ == nullptr) return;
  const auto monotonic = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  *command_output_ << "{\"host_monotonic_ns\":" << monotonic << ",\"host_wall_time\":\"" << WallTime() << "\",\"topic\":\"" << EscapeJson(topic) << "\",\"payload_length\":" << payload.size() << ",\"payload_raw\":\"" << EscapeJson(payload) << "\"}\n";
  command_output_->flush(); RecordTopicFrame(topic, payload);
  std::string error; if (!SummarizeJsonPayload(payload, config_.parser_max_payload_bytes, &schema_, &error)) ++topics_[topic].stats.bad_frames;
}

bool FeedbackRecorder::RecordOperatorEvent(const std::string& event, std::string* error) {
  static const std::set<std::string> allowed = {"APP_CONNECTED", "IDLE_START", "IDLE_END", "JOINT_1_POS_START", "JOINT_1_POS_END", "JOINT_1_NEG_START", "JOINT_1_NEG_END", "JOINT_2_POS_START", "JOINT_2_POS_END", "GRIPPER_OPEN_START", "GRIPPER_OPEN_END", "GRIPPER_CLOSE_START", "GRIPPER_CLOSE_END", "APP_STOP_START", "APP_STOP_END", "APP_CONTROL_PAGE_EXIT", "APP_DISCONNECTED"};
  if (allowed.count(event) == 0U) { if (error) *error = "unsupported operator event"; return false; }
  std::lock_guard<std::mutex> lock(mutex_); if (event_output_ == nullptr) { if (error) *error = "recorder is closed"; return false; }
  const auto monotonic = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  *event_output_ << "{\"host_monotonic_ns\":" << monotonic << ",\"host_wall_time\":\"" << WallTime() << "\",\"event\":\"" << event << "\"}\n";
  event_output_->flush(); return true;
}

void FeedbackRecorder::RecordServoAngles(const std::string& topic, const std::array<float, 7>& values) {
  std::lock_guard<std::mutex> lock(mutex_); if (servo_output_ == nullptr) return;
  const auto monotonic = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
  *servo_output_ << monotonic << ',' << WallTime() << ',' << topic;
  std::ostringstream fingerprint; fingerprint << std::setprecision(9);
  for (float value : values) { if (std::isfinite(value)) { *servo_output_ << ',' << std::setprecision(9) << value; fingerprint << value << ','; } else { *servo_output_ << ",nonfinite"; fingerprint << "nonfinite,"; } }
  *servo_output_ << '\n'; servo_output_->flush(); RecordTopicFrame(topic, fingerprint.str());
}

void FeedbackRecorder::RefreshStaleStates() {
  std::lock_guard<std::mutex> lock(mutex_); const auto now = std::chrono::steady_clock::now();
  for (auto& [_, state] : topics_) {
    if (!state.stats.received) continue;
    const auto age = std::chrono::duration_cast<std::chrono::nanoseconds>(now - state.last); state.stats.last_frame_age_ns = age.count(); state.stats.stale = age > config_.stale_after;
  }
}

TopicStats FeedbackRecorder::GetTopicStats(const std::string& topic) const { std::lock_guard<std::mutex> lock(mutex_); const auto it = topics_.find(topic); return it == topics_.end() ? TopicStats{} : it->second.stats; }
std::map<std::string, FieldSummary> FeedbackRecorder::GetSchemaSummary() const { std::lock_guard<std::mutex> lock(mutex_); return schema_; }
bool FeedbackRecorder::IsOpen() const { std::lock_guard<std::mutex> lock(mutex_); return arm_output_ != nullptr; }

void FeedbackRecorder::WriteSummaryLocked() {
  std::ofstream summary(config_.output_dir / "protocol_summary.json", std::ios::trunc); if (!summary) return;
  summary << "{\n  \"topics\": {"; bool first_topic = true;
  for (const auto& [name, state] : topics_) { if (!first_topic) summary << ','; first_topic = false; const TopicStats& s = state.stats; summary << "\n    \"" << EscapeJson(name) << "\": {\"frames\":" << s.frames << ",\"changed_frames\":" << s.changed_frames << ",\"bad_frames\":" << s.bad_frames << ",\"received\":" << (s.received ? "true" : "false") << ",\"stale\":" << (s.stale ? "true" : "false") << ",\"first_frame_wait_ns\":" << s.first_frame_wait_ns << ",\"last_frame_age_ns\":" << s.last_frame_age_ns << ",\"average_hz\":" << s.average_hz << '}'; }
  summary << "\n  },\n  \"json_schema\": {"; bool first_field = true;
  for (const auto& [path, field] : schema_) { if (!first_field) summary << ','; first_field = false; summary << "\n    \"" << EscapeJson(path) << "\": {\"types\":["; bool first_type = true; for (const auto& type : field.types) { if (!first_type) summary << ','; first_type = false; summary << "\"" << type << "\""; } summary << "],\"observations\":" << field.observations << ",\"numeric_observations\":" << field.numeric_observations; if (field.numeric_observations) summary << ",\"numeric_min\":" << field.numeric_min << ",\"numeric_max\":" << field.numeric_max; if (field.saw_array) summary << ",\"min_array_length\":" << field.min_array_length << ",\"max_array_length\":" << field.max_array_length; summary << '}'; }
  summary << "\n  }\n}\n";
}

void FeedbackRecorder::Close() { std::lock_guard<std::mutex> lock(mutex_); if (arm_output_ == nullptr) return; WriteSummaryLocked(); arm_output_->flush(); command_output_->flush(); servo_output_->flush(); event_output_->flush(); delete arm_output_; delete command_output_; delete servo_output_; delete event_output_; arm_output_ = nullptr; command_output_ = nullptr; servo_output_ = nullptr; event_output_ = nullptr; }

bool SummarizeJsonPayload(const std::string& payload, std::size_t max_payload_bytes, std::map<std::string, FieldSummary>* summary, std::string* error) {
  if (payload.size() > max_payload_bytes) { *error = "payload exceeds parser limit"; return false; }
  JsonValue root; JsonParser parser(payload); if (!parser.Parse(&root, error)) return false; ObserveValue(root, "$", summary); return true;
}

}  // namespace rk_arm_feedback_probe
