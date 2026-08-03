#include <cassert>
#include <cstdint>
#include <vector>

#include "rk_go2_sdk_bridge/jpeg_frame_validation.hpp"

int main()
{
  // 有效边界允许进入下游 JPEG 解码；内容完整性仍由解码器负责。
  assert(rk_go2_sdk_bridge::IsCompleteJpegFrame(
      std::vector<uint8_t>{0xffU, 0xd8U, 0x00U, 0xffU, 0xd9U}));
  assert(!rk_go2_sdk_bridge::IsCompleteJpegFrame(std::vector<uint8_t>{}));
  assert(!rk_go2_sdk_bridge::IsCompleteJpegFrame(
      std::vector<uint8_t>{0xffU, 0xd8U, 0x00U}));
  assert(!rk_go2_sdk_bridge::IsCompleteJpegFrame(
      std::vector<uint8_t>{0x00U, 0xd8U, 0x00U, 0xffU, 0xd9U}));
  assert(!rk_go2_sdk_bridge::IsCompleteJpegFrame(
      std::vector<uint8_t>{0xffU, 0xd8U, 0x00U, 0x00U, 0x00U}));
  return 0;
}
