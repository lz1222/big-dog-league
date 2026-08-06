#pragma once

#include <filesystem>
#include <string>

namespace rk_arm {

/** 本地进程互斥锁，防止两个本机驱动同时创建 rt/arm_Command writer。 */
class SingleWriterGuard {
 public:
  explicit SingleWriterGuard(std::filesystem::path lock_path);
  ~SingleWriterGuard();
  SingleWriterGuard(const SingleWriterGuard&) = delete;
  SingleWriterGuard& operator=(const SingleWriterGuard&) = delete;

  bool Acquire(std::string* error);
  bool held() const { return held_; }

 private:
  std::filesystem::path lock_path_;
  bool held_{false};
};

}  // namespace rk_arm
