#pragma once

#include "inspection/config.hpp"
#include "inspection/types.hpp"

#include <memory>

namespace inspection {

class VisionPipeline {
 public:
  virtual ~VisionPipeline() = default;

  virtual ActionStatus Initialize() = 0;
  virtual void SetScene(VisionScene scene) = 0;
  virtual VisionResult Detect() = 0;
};

std::unique_ptr<VisionPipeline> CreateSimVisionPipeline(const MissionConfig& config);

}  // namespace inspection

