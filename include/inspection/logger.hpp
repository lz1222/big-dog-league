#pragma once

#include <string>

namespace inspection {

enum class LogLevel {
  Debug,
  Info,
  Warn,
  Error,
};

void Log(LogLevel level, const std::string& message);

}  // namespace inspection

