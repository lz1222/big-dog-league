#pragma once

#include "inspection/types.hpp"

#include <memory>
#include <string>

namespace inspection {

class RobotMotion {
 public:
  virtual ~RobotMotion() = default;

  virtual ActionStatus Initialize() = 0;
  virtual ActionStatus StandUp() = 0;
  virtual ActionStatus StopMove() = 0;
  virtual ActionStatus EmergencyStop() = 0;
  virtual ActionStatus FollowLine(double forward_speed_mps,
                                  double line_offset,
                                  double turn_gain,
                                  double max_yaw_rate) = 0;
  virtual ActionStatus JumpForward(const std::string& label) = 0;
  virtual ActionStatus EnableObstacleAvoidance(bool enabled) = 0;
  virtual ActionStatus ClimbStairs() = 0;
  virtual ActionStatus PerformWarningAction(WarningSign sign) = 0;
  virtual ActionStatus Park() = 0;
};

std::unique_ptr<RobotMotion> CreateSimRobotMotion();

}  // namespace inspection

