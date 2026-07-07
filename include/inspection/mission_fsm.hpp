#pragma once

#include "inspection/arm_controller.hpp"
#include "inspection/config.hpp"
#include "inspection/robot_motion.hpp"
#include "inspection/types.hpp"
#include "inspection/vision_pipeline.hpp"

#include <functional>

namespace inspection {

class MissionFSM {
 public:
  MissionFSM(MissionConfig config,
             RobotMotion& motion,
             VisionPipeline& vision,
             ArmController& arm,
             std::function<bool()> should_stop);

  MissionState Run();
  MissionState state() const { return state_; }

 private:
  void Step(double dt_s);
  void EnterState();
  void Transition(MissionState next);
  void Fail(const std::string& message);
  bool Check(const ActionStatus& status);
  void UpdateLineFollowing(double speed_mps, MissionState next, double duration_s);
  bool IsTerminal() const;

  MissionConfig config_;
  RobotMotion& motion_;
  VisionPipeline& vision_;
  ArmController& arm_;
  std::function<bool()> should_stop_;

  MissionState state_ = MissionState::Idle;
  double state_elapsed_s_ = 0.0;
  bool entered_ = false;

  PlacementZone target_zone_ = PlacementZone::Zone1;
  WarningSign warning_sign_ = WarningSign::Wave;
};

}  // namespace inspection
