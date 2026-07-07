#pragma once

#include "inspection/types.hpp"

#include <memory>

namespace inspection {

class ArmController {
 public:
  virtual ~ArmController() = default;

  virtual ActionStatus Initialize() = 0;
  virtual ActionStatus Home() = 0;
  virtual ActionStatus PickStartMaterial(MaterialType material) = 0;
  virtual ActionStatus PlaceOnTransferPlatform() = 0;
  virtual ActionStatus PickFieldMaterial(MaterialType material) = 0;
  virtual ActionStatus PlaceOnZone(PlacementZone zone) = 0;
  virtual ActionStatus Release() = 0;
  virtual ActionStatus EmergencyStop() = 0;
};

std::unique_ptr<ArmController> CreateSimArmController();

}  // namespace inspection

