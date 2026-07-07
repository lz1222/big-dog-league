#include "inspection/arm_controller.hpp"

#include "inspection/logger.hpp"

namespace inspection {
namespace {

class SimArmController final : public ArmController {
 public:
  ActionStatus Initialize() override {
    Log(LogLevel::Info, "Arm(sim): initialized D1-compatible scripted backend");
    return ActionStatus::Success();
  }

  ActionStatus Home() override {
    Log(LogLevel::Info, "Arm(sim): home");
    return ActionStatus::Success();
  }

  ActionStatus PickStartMaterial(MaterialType material) override {
    Log(LogLevel::Info, "Arm(sim): pick start material " + ToString(material));
    return ActionStatus::Success();
  }

  ActionStatus PlaceOnTransferPlatform() override {
    Log(LogLevel::Info, "Arm(sim): place on transfer platform");
    return ActionStatus::Success();
  }

  ActionStatus PickFieldMaterial(MaterialType material) override {
    Log(LogLevel::Info, "Arm(sim): pick field material " + ToString(material));
    return ActionStatus::Success();
  }

  ActionStatus PlaceOnZone(PlacementZone zone) override {
    Log(LogLevel::Info, "Arm(sim): place field material on " + ToString(zone));
    return ActionStatus::Success();
  }

  ActionStatus Release() override {
    Log(LogLevel::Info, "Arm(sim): release gripper");
    return ActionStatus::Success();
  }

  ActionStatus EmergencyStop() override {
    Log(LogLevel::Error, "Arm(sim): emergency stop");
    return ActionStatus::Success();
  }
};

}  // namespace

std::unique_ptr<ArmController> CreateSimArmController() {
  return std::make_unique<SimArmController>();
}

}  // namespace inspection

