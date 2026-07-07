#pragma once

#include <string>
#include <utility>

namespace inspection {

enum class MaterialType {
  Unknown,
  Ball,
  Box,
  Pyramid,
  Cylinder,
};

enum class PlacementZone {
  Unknown,
  Zone1,
  Zone2,
};

enum class WarningSign {
  Unknown,
  Wave,
  Sit,
  Stretch,
};

enum class VisionScene {
  LineFollowing,
  PickupMarker,
  WarningMarker,
  PlacementMarker,
};

enum class MissionState {
  Idle,
  StandUp,
  StartJump,
  FollowToAvoidance,
  Avoidance,
  Stairs,
  ApproachPickup,
  PickStartMaterial,
  DetectPickupMarker,
  FollowToTransfer,
  PlaceTransfer,
  PickFieldMaterial,
  FollowToInspection,
  DetectWarning,
  PerformWarningAction,
  FollowToPlacement,
  PlaceFieldMaterial,
  FollowToFinishObstacle,
  FinishJump,
  Park,
  Completed,
  EmergencyStop,
  Error,
};

struct ActionStatus {
  bool ok = true;
  std::string message;

  static ActionStatus Success(std::string detail = {}) {
    return ActionStatus{true, std::move(detail)};
  }

  static ActionStatus Failure(std::string detail) {
    return ActionStatus{false, std::move(detail)};
  }
};

struct VisionResult {
  bool line_visible = true;
  double line_offset = 0.0;
  double confidence = 1.0;
  MaterialType material = MaterialType::Unknown;
  PlacementZone placement_zone = PlacementZone::Unknown;
  WarningSign warning_sign = WarningSign::Unknown;
};

inline std::string ToString(MaterialType value) {
  switch (value) {
    case MaterialType::Ball:
      return "ball";
    case MaterialType::Box:
      return "box";
    case MaterialType::Pyramid:
      return "pyramid";
    case MaterialType::Cylinder:
      return "cylinder";
    case MaterialType::Unknown:
    default:
      return "unknown";
  }
}

inline std::string ToString(PlacementZone value) {
  switch (value) {
    case PlacementZone::Zone1:
      return "zone1";
    case PlacementZone::Zone2:
      return "zone2";
    case PlacementZone::Unknown:
    default:
      return "unknown";
  }
}

inline std::string ToString(WarningSign value) {
  switch (value) {
    case WarningSign::Wave:
      return "wave";
    case WarningSign::Sit:
      return "sit";
    case WarningSign::Stretch:
      return "stretch";
    case WarningSign::Unknown:
    default:
      return "unknown";
  }
}

inline std::string ToString(MissionState value) {
  switch (value) {
    case MissionState::Idle:
      return "idle";
    case MissionState::StandUp:
      return "stand_up";
    case MissionState::StartJump:
      return "start_jump";
    case MissionState::FollowToAvoidance:
      return "follow_to_avoidance";
    case MissionState::Avoidance:
      return "avoidance";
    case MissionState::Stairs:
      return "stairs";
    case MissionState::ApproachPickup:
      return "approach_pickup";
    case MissionState::PickStartMaterial:
      return "pick_start_material";
    case MissionState::DetectPickupMarker:
      return "detect_pickup_marker";
    case MissionState::FollowToTransfer:
      return "follow_to_transfer";
    case MissionState::PlaceTransfer:
      return "place_transfer";
    case MissionState::PickFieldMaterial:
      return "pick_field_material";
    case MissionState::FollowToInspection:
      return "follow_to_inspection";
    case MissionState::DetectWarning:
      return "detect_warning";
    case MissionState::PerformWarningAction:
      return "perform_warning_action";
    case MissionState::FollowToPlacement:
      return "follow_to_placement";
    case MissionState::PlaceFieldMaterial:
      return "place_field_material";
    case MissionState::FollowToFinishObstacle:
      return "follow_to_finish_obstacle";
    case MissionState::FinishJump:
      return "finish_jump";
    case MissionState::Park:
      return "park";
    case MissionState::Completed:
      return "completed";
    case MissionState::EmergencyStop:
      return "emergency_stop";
    case MissionState::Error:
      return "error";
    default:
      return "unknown";
  }
}

}  // namespace inspection
