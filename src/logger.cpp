#include "inspection/logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

namespace inspection {
namespace {

std::mutex& LogMutex() {
  static std::mutex mutex;
  return mutex;
}

const char* LevelName(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    default:
      return "LOG";
  }
}

std::string Timestamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};

#if defined(_WIN32)
  localtime_s(&local_time, &now_time);
#else
  localtime_r(&now_time, &local_time);
#endif

  std::ostringstream out;
  out << std::put_time(&local_time, "%H:%M:%S");
  return out.str();
}

}  // namespace

void Log(LogLevel level, const std::string& message) {
  std::lock_guard<std::mutex> lock(LogMutex());
  std::ostream& output = (level == LogLevel::Error) ? std::cerr : std::cout;
  output << "[" << Timestamp() << "][" << LevelName(level) << "] " << message << '\n';
}

}  // namespace inspection

