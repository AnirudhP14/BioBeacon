#include "logging.h"

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>

void Logger::logMsg(const char* msg) {
  size_t len = strlen(msg);
  if (len + 2 > kLogSize) return;

  if (logIndex_ + len + 2 >= kLogSize) {
    logIndex_ = 0;
  }

  strcpy(&logBuffer_[logIndex_], msg);
  logIndex_ += len;
  logBuffer_[logIndex_++] = '\n';
  logBuffer_[logIndex_] = '\0';
}

void Logger::logFmt(const char* fmt, ...) {
  char buf[128];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  logMsg(buf);
}

void Logger::clear() {
  logIndex_ = 0;
  logBuffer_[0] = '\0';
}

const char* Logger::buffer() const {
  return logBuffer_;
}
