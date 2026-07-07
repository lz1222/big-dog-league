#include "inspection/robot_motion.hpp"

#include "inspection/logger.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

namespace inspection {
namespace {

class SimRobotMotion final : public RobotMotion {
 public:
  ActionStatus Initialize() override {
    Log(LogLevel::Info, "Motion(sim): initialized Go2-compatible backend");
    return ActionStatus::Success();
  }

  ActionStatus StandUp() override {
    Log(LogLevel::Info, "Motion(sim): stand up");
    return ActionStatus::Success();
  }

  ActionStatus StopMove() override {
    Log(LogLevel::Info, "Motion(sim): stop move");
    return ActionStatus::Success();
  }

  ActionStatus EmergencyStop() override {
    Log(LogLevel::Error, "Motion(sim): emergency stop");
    return ActionStatus::Success();
  }

  ActionStatus FollowLine(double forward_speed_mps,
                          double line_offset,
                          double turn_gain,
                          double max_yaw_rate) override {
    const double yaw_rate = std::clamp(-line_offset * turn_gain,
                                       -max_yaw_rate,
                                       max_yaw_rate);
    ++follow_counter_;
    if (follow_counter_ % 10 == 1) {
      std::ostringstream out;
      out << std::fixed << std::setprecision(3)
          << "Motion(sim): follow line vx=" << forward_speed_mps
          << " offset=" << line_offset
          << " yaw_rate=" << yaw_rate;
      Log(LogLevel::Info, out.str());
    }
    return ActionStatus::Success();
  }

  ActionStatus JumpForward(const std::string& label) override {
    Log(LogLevel::Info, "Motion(sim): jump forward over " + label);
    return ActionStatus::Success();
  }

  ActionStatus EnableObstacleAvoidance(bool enabled) override {
    Log(LogLevel::Info,
        std::string("Motion(sim): obstacle avoidance ") + (enabled ? "enabled" : "disabled"));
    return ActionStatus::Success();
  }

  ActionStatus ClimbStairs() override {
    Log(LogLevel::Info, "Motion(sim): climb stairs with conservative gait");
    return ActionStatus::Success();
  }

  ActionStatus PerformWarningAction(WarningSign sign) override {
    Log(LogLevel::Info, "Motion(sim): perform warning action " + ToString(sign));
    return ActionStatus::Success();
  }

  ActionStatus Park() override {
    Log(LogLevel::Info, "Motion(sim): park inside start-stop zone");
    return ActionStatus::Success();
  }

 private:
  int follow_counter_ = 0;
};

}  // namespace

std::unique_ptr<RobotMotion> CreateSimRobotMotion() {
  return std::make_unique<SimRobotMotion>();
}

}  // namespace inspection

