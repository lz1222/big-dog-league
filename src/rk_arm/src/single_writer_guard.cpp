#include "rk_arm/single_writer_guard.hpp"

#include <cerrno>
#include <csignal>
#include <fcntl.h>
#include <fstream>
#include <unistd.h>

namespace rk_arm {
SingleWriterGuard::SingleWriterGuard(std::filesystem::path lock_path) : lock_path_(std::move(lock_path)) {}
SingleWriterGuard::~SingleWriterGuard() { if (held_) { std::error_code ignored; std::filesystem::remove(lock_path_, ignored); } }

bool SingleWriterGuard::Acquire(std::string* error) {
  if (held_) return true;
  const int fd = ::open(lock_path_.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0600);
  if (fd >= 0) {
    const std::string pid = std::to_string(::getpid()) + "\n";
    (void)::write(fd, pid.data(), pid.size()); ::close(fd); held_ = true; return true;
  }
  if (errno != EEXIST) { if (error) *error = "cannot create writer lock"; return false; }
  std::ifstream input(lock_path_); long pid = -1; input >> pid;
  if (pid > 0 && (::kill(static_cast<pid_t>(pid), 0) == 0 || errno == EPERM)) {
    if (error) *error = "active local writer PID " + std::to_string(pid) + " holds " + lock_path_.string();
    return false;
  }
  std::error_code remove_error; std::filesystem::remove(lock_path_, remove_error);
  if (remove_error) { if (error) *error = "cannot remove stale writer lock"; return false; }
  return Acquire(error);
}
}  // namespace rk_arm
