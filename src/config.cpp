#include "inspection/config.hpp"

#include "inspection/logger.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace inspection {
namespace {

std::string Trim(std::string value) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string Lower(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

double ReadDouble(const std::map<std::string, std::string>& values,
                  const std::string& key,
                  double fallback) {
  const auto it = values.find(key);
  if (it == values.end()) {
    return fallback;
  }
  return std::stod(it->second);
}

int ReadInt(const std::map<std::string, std::string>& values,
            const std::string& key,
            int fallback) {
  const auto it = values.find(key);
  if (it == values.end()) {
    return fallback;
  }
  return std::stoi(it->second);
}

MaterialType ParseMaterial(const std::string& raw, MaterialType fallback) {
  const std::string value = Lower(raw);
  if (value == "ball") {
    return MaterialType::Ball;
  }
  if (value == "box" || value == "cuboid") {
    return MaterialType::Box;
  }
  if (value == "pyramid") {
    return MaterialType::Pyramid;
  }
  if (value == "cylinder") {
    return MaterialType::Cylinder;
  }
  return fallback;
}

PlacementZone ParsePlacementZone(const std::string& raw, PlacementZone fallback) {
  const std::string value = Lower(raw);
  if (value == "zone1" || value == "1") {
    return PlacementZone::Zone1;
  }
  if (value == "zone2" || value == "2") {
    return PlacementZone::Zone2;
  }
  return fallback;
}

WarningSign ParseWarningSign(const std::string& raw, WarningSign fallback) {
  const std::string value = Lower(raw);
  if (value == "wave") {
    return WarningSign::Wave;
  }
  if (value == "sit") {
    return WarningSign::Sit;
  }
  if (value == "stretch") {
    return WarningSign::Stretch;
  }
  return fallback;
}

std::map<std::string, std::string> ReadKeyValues(const std::string& path) {
  std::ifstream input(path);
  std::map<std::string, std::string> values;
  if (!input) {
    Log(LogLevel::Warn, "Config file not found, using defaults: " + path);
    return values;
  }

  std::string line;
  int line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto comment = line.find('#');
    if (comment != std::string::npos) {
      line = line.substr(0, comment);
    }

    line = Trim(line);
    if (line.empty()) {
      continue;
    }

    const auto equals = line.find('=');
    if (equals == std::string::npos) {
      Log(LogLevel::Warn, "Ignoring config line without '=' at " + std::to_string(line_number));
      continue;
    }

    const std::string key = Lower(Trim(line.substr(0, equals)));
    const std::string value = Trim(line.substr(equals + 1));
    values[key] = value;
  }

  return values;
}

}  // namespace

MissionConfig LoadConfig(const std::string& path) {
  MissionConfig config;
  const auto values = ReadKeyValues(path);

  try {
    config.control_period_ms = ReadInt(values, "control_period_ms", config.control_period_ms);
    config.cruise_speed_mps = ReadDouble(values, "cruise_speed_mps", config.cruise_speed_mps);
    config.slow_speed_mps = ReadDouble(values, "slow_speed_mps", config.slow_speed_mps);
    config.turn_gain = ReadDouble(values, "turn_gain", config.turn_gain);
    config.max_yaw_rate = ReadDouble(values, "max_yaw_rate", config.max_yaw_rate);

    config.standup_settle_s = ReadDouble(values, "standup_settle_s", config.standup_settle_s);
    config.start_jump_settle_s = ReadDouble(values, "start_jump_settle_s", config.start_jump_settle_s);
    config.follow_to_avoidance_s = ReadDouble(values, "follow_to_avoidance_s", config.follow_to_avoidance_s);
    config.avoidance_s = ReadDouble(values, "avoidance_s", config.avoidance_s);
    config.stairs_s = ReadDouble(values, "stairs_s", config.stairs_s);
    config.approach_pickup_s = ReadDouble(values, "approach_pickup_s", config.approach_pickup_s);
    config.arm_action_settle_s = ReadDouble(values, "arm_action_settle_s", config.arm_action_settle_s);
    config.marker_detect_s = ReadDouble(values, "marker_detect_s", config.marker_detect_s);
    config.follow_to_transfer_s = ReadDouble(values, "follow_to_transfer_s", config.follow_to_transfer_s);
    config.follow_to_inspection_s = ReadDouble(values, "follow_to_inspection_s", config.follow_to_inspection_s);
    config.warning_action_s = ReadDouble(values, "warning_action_s", config.warning_action_s);
    config.follow_to_placement_s = ReadDouble(values, "follow_to_placement_s", config.follow_to_placement_s);
    config.follow_to_finish_s = ReadDouble(values, "follow_to_finish_s", config.follow_to_finish_s);
    config.finish_jump_settle_s = ReadDouble(values, "finish_jump_settle_s", config.finish_jump_settle_s);
    config.park_s = ReadDouble(values, "park_s", config.park_s);
    config.min_marker_confidence = ReadDouble(values, "min_marker_confidence", config.min_marker_confidence);
    config.simulate_line_loss_every = ReadInt(values, "simulate_line_loss_every", config.simulate_line_loss_every);

    const auto start = values.find("default_start_material");
    if (start != values.end()) {
      config.default_start_material = ParseMaterial(start->second, config.default_start_material);
    }

    const auto field = values.find("default_field_material");
    if (field != values.end()) {
      config.default_field_material = ParseMaterial(field->second, config.default_field_material);
    }

    const auto zone = values.find("default_placement_zone");
    if (zone != values.end()) {
      config.default_placement_zone = ParsePlacementZone(zone->second, config.default_placement_zone);
    }

    const auto warning = values.find("default_warning_sign");
    if (warning != values.end()) {
      config.default_warning_sign = ParseWarningSign(warning->second, config.default_warning_sign);
    }
  } catch (const std::exception& ex) {
    Log(LogLevel::Error, std::string("Config parse failed: ") + ex.what());
    throw;
  }

  return config;
}

void ApplyProfile(const std::string& profile, MissionConfig& config) {
  const std::string value = Lower(profile);
  if (value == "attack") {
    config.cruise_speed_mps *= 1.2;
    config.slow_speed_mps *= 1.1;
    config.follow_to_avoidance_s *= 0.9;
    config.follow_to_transfer_s *= 0.9;
    config.follow_to_inspection_s *= 0.9;
    config.follow_to_placement_s *= 0.9;
    config.follow_to_finish_s *= 0.9;
    Log(LogLevel::Info, "Applied attack profile");
    return;
  }

  if (value == "safe") {
    config.cruise_speed_mps *= 0.85;
    config.slow_speed_mps *= 0.85;
    config.max_yaw_rate *= 0.8;
    Log(LogLevel::Info, "Applied safe profile");
    return;
  }

  if (!value.empty() && value != "default") {
    Log(LogLevel::Warn, "Unknown profile '" + profile + "', using default");
  }
}

}  // namespace inspection

