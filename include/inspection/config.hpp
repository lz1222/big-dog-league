#pragma once

#include "inspection/types.hpp"

#include <string>

namespace inspection {

struct MissionConfig {
  int control_period_ms = 50;

  double cruise_speed_mps = 0.28;
  double slow_speed_mps = 0.16;
  double turn_gain = 0.85;
  double max_yaw_rate = 0.55;

  double standup_settle_s = 1.0;
  double start_jump_settle_s = 1.5;
  double follow_to_avoidance_s = 4.0;
  double avoidance_s = 5.0;
  double stairs_s = 4.5;
  double approach_pickup_s = 3.0;
  double arm_action_settle_s = 1.5;
  double marker_detect_s = 1.0;
  double follow_to_transfer_s = 3.5;
  double follow_to_inspection_s = 3.5;
  double warning_action_s = 1.5;
  double follow_to_placement_s = 4.0;
  double follow_to_finish_s = 4.0;
  double finish_jump_settle_s = 1.5;
  double park_s = 1.5;

  double min_marker_confidence = 0.55;
  int simulate_line_loss_every = 0;

  MaterialType default_start_material = MaterialType::Box;
  MaterialType default_field_material = MaterialType::Cylinder;
  PlacementZone default_placement_zone = PlacementZone::Zone1;
  WarningSign default_warning_sign = WarningSign::Wave;
};

MissionConfig LoadConfig(const std::string& path);
void ApplyProfile(const std::string& profile, MissionConfig& config);

}  // namespace inspection

