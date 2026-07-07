#include "inspection/vision_pipeline.hpp"

#include "inspection/logger.hpp"

#include <cmath>
#include <memory>
#include <utility>

namespace inspection {
namespace {

class SimVisionPipeline final : public VisionPipeline {
 public:
  explicit SimVisionPipeline(MissionConfig config) : config_(std::move(config)) {}

  ActionStatus Initialize() override {
    Log(LogLevel::Info, "Vision(sim): initialized Go2 video pipeline simulator");
    return ActionStatus::Success();
  }

  void SetScene(VisionScene scene) override {
    scene_ = scene;
    frame_ = 0;
  }

  VisionResult Detect() override {
    ++frame_;

    VisionResult result;
    result.confidence = 0.92;

    if (config_.simulate_line_loss_every > 0 &&
        frame_ % config_.simulate_line_loss_every == 0) {
      result.line_visible = false;
      result.confidence = 0.0;
      return result;
    }

    switch (scene_) {
      case VisionScene::LineFollowing:
        result.line_visible = true;
        result.line_offset = 0.12 * std::sin(static_cast<double>(frame_) * 0.2);
        break;

      case VisionScene::PickupMarker:
        result.line_visible = false;
        result.placement_zone = config_.default_placement_zone;
        result.material = config_.default_start_material;
        break;

      case VisionScene::WarningMarker:
        result.line_visible = false;
        result.warning_sign = config_.default_warning_sign;
        break;

      case VisionScene::PlacementMarker:
        result.line_visible = false;
        result.placement_zone = config_.default_placement_zone;
        break;
    }

    return result;
  }

 private:
  MissionConfig config_;
  VisionScene scene_ = VisionScene::LineFollowing;
  int frame_ = 0;
};

}  // namespace

std::unique_ptr<VisionPipeline> CreateSimVisionPipeline(const MissionConfig& config) {
  return std::make_unique<SimVisionPipeline>(config);
}

}  // namespace inspection
