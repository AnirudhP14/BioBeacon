#ifndef SWARMBOT_LOGGING_H
#define SWARMBOT_LOGGING_H

#include <cstddef>

class Logger {
 public:
  void logMsg(const char* msg);
  void logFmt(const char* fmt, ...);
  void clear();
  const char* buffer() const;

 private:
  static constexpr size_t kLogSize = 4096;
  char logBuffer_[kLogSize] = {0};
  size_t logIndex_ = 0;
};

#endif  // SWARMBOT_LOGGING_H
