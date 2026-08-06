#include "rk_arm/single_writer_guard.hpp"
#include <cassert>
#include <fstream>
#include <unistd.h>
int main() {
  const auto path = std::filesystem::temp_directory_path() / ("rk_arm_writer_test_" + std::to_string(getpid()));
  std::string error; { rk_arm::SingleWriterGuard first(path); assert(first.Acquire(&error)); rk_arm::SingleWriterGuard second(path); assert(!second.Acquire(&error)); }
  { std::ofstream stale(path); stale << "99999999\n"; } rk_arm::SingleWriterGuard recovered(path); assert(recovered.Acquire(&error));
}
