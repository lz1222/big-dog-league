#include "inspection/mission_fsm.hpp"

#include "inspection/logger.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace inspection {

MissionFSM::MissionFSM(MissionConfig config,
                       RobotMotion& motion,
                       VisionPipeline& vision,
                       ArmController& arm,
                       std::function<bool()> should_stop)
    : config_(std::move(config)),
      motion_(motion),
      vision_(vision),
      arm_(arm),
      should_stop_(std::move(should_stop)),
      target_zone_(config_.default_placement_zone),
      warning_sign_(config_.default_warning_sign) {}

MissionState MissionFSM::Run() {
  if (!Check(motion_.Initialize()) ||
      !Check(vision_.Initialize()) ||
      !Check(arm_.Initialize())) {
    EnterState();
    return state_;
  }

  Transition(MissionState::StandUp);

  const double dt_s = static_cast<double>(config_.control_period_ms) / 1000.0;
  while (true) {
    const auto tick_start = std::chrono::steady_clock::now();
    Step(dt_s);

    if (IsTerminal() && entered_) {
      break;
    }

    std::this_thread::sleep_until(
        tick_start + std::chrono::milliseconds(config_.control_period_ms));
  }

  return state_;
}

void MissionFSM::Step(double dt_s) {
  if (!IsTerminal() && should_stop_ && should_stop_()) {
    Transition(MissionState::EmergencyStop);
  }

  if (!entered_) {
    EnterState();
  }

  if (IsTerminal()) {
    return;
  }

  state_elapsed_s_ += dt_s;

  switch (state_) {
    case MissionState::StandUp:
      if (state_elapsed_s_ >= config_.standup_settle_s) {
        Transition(MissionState::StartJump);
      }
      break;

    case MissionState::StartJump:
      if (state_elapsed_s_ >= config_.start_jump_settle_s) {
        Transition(MissionState::FollowToAvoidance);
      }
      break;

    case MissionState::FollowToAvoidance:
      UpdateLineFollowing(config_.cruise_speed_mps,
                          MissionState::Avoidance,
                          config_.follow_to_avoidance_s);
      break;

    case MissionState::Avoidance:
      UpdateLineFollowing(config_.slow_speed_mps,
                          MissionState::Stairs,
                          config_.avoidance_s);
      break;

    case MissionState::Stairs:
      if (state_elapsed_s_ >= config_.stairs_s) {
        Transition(MissionState::ApproachPickup);
      }
      break;

    case MissionState::ApproachPickup:
      UpdateLineFollowing(config_.slow_speed_mps,
                          MissionState::PickStartMaterial,
                          config_.approach_pickup_s);
      break;

    case MissionState::PickStartMaterial:
      if (state_elapsed_s_ >= config_.arm_action_settle_s) {
        Transition(MissionState::DetectPickupMarker);
      }
      break;

    case MissionState::DetectPickupMarker:
      if (state_elapsed_s_ >= config_.marker_detect_s) {
        const auto result = vision_.Detect();
        if (result.confidence >= config_.min_marker_confidence &&
            result.placement_zone != PlacementZone::Unknown) {
          target_zone_ = result.placement_zone;
        } else {
          Log(LogLevel::Warn,
              "Pickup marker confidence low, using default zone " + ToString(target_zone_));
        }
        Log(LogLevel::Info, "Target placement zone: " + ToString(target_zone_));
        Transition(MissionState::FollowToTransfer);
      }
      break;

    case MissionState::FollowToTransfer:
      UpdateLineFollowing(config_.cruise_speed_mps,
                          MissionState::PlaceTransfer,
                          config_.follow_to_transfer_s);
      break;

    case MissionState::PlaceTransfer:
      if (state_elapsed_s_ >= config_.arm_action_settle_s) {
        Transition(MissionState::PickFieldMaterial);
      }
      break;

    case MissionState::PickFieldMaterial:
      if (state_elapsed_s_ >= config_.arm_action_settle_s) {
        Transition(MissionState::FollowToInspection);
      }
      break;

    case MissionState::FollowToInspection:
      UpdateLineFollowing(config_.cruise_speed_mps,
                          MissionState::DetectWarning,
                          config_.follow_to_inspection_s);
      break;

    case MissionState::DetectWarning:
      if (state_elapsed_s_ >= config_.marker_detect_s) {
        const auto result = vision_.Detect();
        if (result.confidence >= config_.min_marker_confidence &&
            result.warning_sign != WarningSign::Unknown) {
          warning_sign_ = result.warning_sign;
        } else {
          Log(LogLevel::Warn,
              "Warning marker confidence low, using default action " + ToString(warning_sign_));
        }
        Log(LogLevel::Info, "Warning action: " + ToString(warning_sign_));
        Transition(MissionState::PerformWarningAction);
      }
      break;

    case MissionState::PerformWarningAction:
      if (state_elapsed_s_ >= config_.warning_action_s) {
        Transition(MissionState::FollowToPlacement);
      }
      break;

    case MissionState::FollowToPlacement:
      UpdateLineFollowing(config_.cruise_speed_mps,
                          MissionState::PlaceFieldMaterial,
                          config_.follow_to_placement_s);
      break;

    case MissionState::PlaceFieldMaterial:
      if (state_elapsed_s_ >= config_.arm_action_settle_s) {
        Transition(MissionState::FollowToFinishObstacle);
      }
      break;

    case MissionState::FollowToFinishObstacle:
      UpdateLineFollowing(config_.cruise_speed_mps,
                          MissionState::FinishJump,
                          config_.follow_to_finish_s);
      break;

    case MissionState::FinishJump:
      if (state_elapsed_s_ >= config_.finish_jump_settle_s) {
        Transition(MissionState::Park);
      }
      break;

    case MissionState::Park:
      if (state_elapsed_s_ >= config_.park_s) {
        Transition(MissionState::Completed);
      }
      break;

    case MissionState::Idle:
    case MissionState::Completed:
    case MissionState::EmergencyStop:
    case MissionState::Error:
      break;
  }
}

void MissionFSM::EnterState() {
  entered_ = true;
  Log(LogLevel::Info, "FSM enter: " + ToString(state_));

  switch (state_) {
    case MissionState::StandUp:
      if (!Check(motion_.StandUp())) {
        return;
      }
      if (!Check(arm_.Home())) {
        return;
      }
      break;

    case MissionState::StartJump:
      if (!Check(motion_.JumpForward("start obstacle"))) {
        return;
      }
      break;

    case MissionState::FollowToAvoidance:
    case MissionState::ApproachPickup:
    case MissionState::FollowToTransfer:
    case MissionState::FollowToInspection:
    case MissionState::FollowToPlacement:
    case MissionState::FollowToFinishObstacle:
      vision_.SetScene(VisionScene::LineFollowing);
      break;

    case MissionState::Avoidance:
      vision_.SetScene(VisionScene::LineFollowing);
      if (!Check(motion_.EnableObstacleAvoidance(true))) {
        return;
      }
      break;

    case MissionState::Stairs:
      if (!Check(motion_.EnableObstacleAvoidance(false))) {
        return;
      }
      if (!Check(motion_.ClimbStairs())) {
        return;
      }
      break;

    case MissionState::PickStartMaterial:
      if (!Check(motion_.StopMove())) {
        return;
      }
      if (!Check(arm_.PickStartMaterial(config_.default_start_material))) {
        return;
      }
      break;

    case MissionState::DetectPickupMarker:
      if (!Check(motion_.StopMove())) {
        return;
      }
      vision_.SetScene(VisionScene::PickupMarker);
      break;

    case MissionState::PlaceTransfer:
      if (!Check(motion_.StopMove())) {
        return;
      }
      if (!Check(arm_.PlaceOnTransferPlatform())) {
        return;
      }
      break;

    case MissionState::PickFieldMaterial:
      if (!Check(arm_.PickFieldMaterial(config_.default_field_material))) {
        return;
      }
      break;

    case MissionState::DetectWarning:
      if (!Check(motion_.StopMove())) {
        return;
      }
      vision_.SetScene(VisionScene::WarningMarker);
      break;

    case MissionState::PerformWarningAction:
      if (!Check(motion_.PerformWarningAction(warning_sign_))) {
        return;
      }
      break;

    case MissionState::PlaceFieldMaterial:
      if (!Check(motion_.StopMove())) {
        return;
      }
      if (!Check(arm_.PlaceOnZone(target_zone_))) {
        return;
      }
      if (!Check(arm_.Release())) {
        return;
      }
      break;

    case MissionState::FinishJump:
      if (!Check(motion_.JumpForward("finish obstacle"))) {
        return;
      }
      break;

    case MissionState::Park:
      if (!Check(motion_.Park())) {
        return;
      }
      break;

    case MissionState::Completed:
      if (!Check(motion_.StopMove())) {
        return;
      }
      Log(LogLevel::Info, "Mission completed");
      break;

    case MissionState::EmergencyStop:
      motion_.EmergencyStop();
      arm_.EmergencyStop();
      Log(LogLevel::Error, "Emergency stop completed");
      break;

    case MissionState::Error:
      motion_.EmergencyStop();
      arm_.EmergencyStop();
      Log(LogLevel::Error, "Mission failed");
      break;

    case MissionState::Idle:
      break;
  }
}

void MissionFSM::Transition(MissionState next) {
  state_ = next;
  state_elapsed_s_ = 0.0;
  entered_ = false;
}

void MissionFSM::Fail(const std::string& message) {
  Log(LogLevel::Error, message);
  Transition(MissionState::Error);
}

bool MissionFSM::Check(const ActionStatus& status) {
  if (!status.ok) {
    Fail(status.message.empty() ? "Action failed" : status.message);
    return false;
  }
  return true;
}

void MissionFSM::UpdateLineFollowing(double speed_mps,
                                     MissionState next,
                                     double duration_s) {
  const auto result = vision_.Detect();
  if (!result.line_visible) {
    Log(LogLevel::Warn, "Line lost, stopping and waiting for recovery");
    if (!Check(motion_.StopMove())) {
      return;
    }
    return;
  }

  if (!Check(motion_.FollowLine(speed_mps,
                                result.line_offset,
                                config_.turn_gain,
                                config_.max_yaw_rate))) {
    return;
  }

  if (state_elapsed_s_ >= duration_s) {
    if (!Check(motion_.StopMove())) {
      return;
    }
    Transition(next);
  }
}

bool MissionFSM::IsTerminal() const {
  return state_ == MissionState::Completed ||
         state_ == MissionState::EmergencyStop ||
         state_ == MissionState::Error;
}

}  // namespace inspection
