#ifndef RK_GO2_SDK_BRIDGE__JPEG_FRAME_VALIDATION_HPP_
#define RK_GO2_SDK_BRIDGE__JPEG_FRAME_VALIDATION_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace rk_go2_sdk_bridge
{

// 在进入长度前缀管道前做最低限度的帧边界保护：缺少 SOI/EOI 的帧
// 不能交给下游解码器，以免造成误导性的 decode failure 或图像链不稳定。
inline bool IsCompleteJpegFrame(const std::vector<uint8_t>& jpeg)
{
  return jpeg.size() >= 4U &&
         jpeg[0] == 0xffU && jpeg[1] == 0xd8U &&
         jpeg[jpeg.size() - 2U] == 0xffU &&
         jpeg[jpeg.size() - 1U] == 0xd9U;
}

}  // namespace rk_go2_sdk_bridge

#endif  // RK_GO2_SDK_BRIDGE__JPEG_FRAME_VALIDATION_HPP_
