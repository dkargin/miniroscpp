//
// Created by dkargin on 2/3/26.
//

#include <stdarg.h>

#include "miniros/internal/local_log.h"

#include <cstring>

namespace miniros {
namespace internal {

int getVerbosity()
{
  return static_cast<int>(console::Level::Debug);
}

void InternalLog::log(const console::LogLocation& loc, const char* channel, const char* fmt, ...) {
  va_list va;
  char buf[512] = {};
  va_start( va, fmt);
  int written = std::vsnprintf(buf,sizeof(buf)-2,fmt,va);
  va_end(va);

  if (written <= 0)
    return;

  // vsnprintf can return value longer than initial buffer.
  // This value corresponds to actual length of the line.
  if (written > sizeof(buf)) {
    written = sizeof(buf) - 1;
  }

  buf[written] = 0;

  if (loc.level_ == console::Level::Error || loc.level_ == console::Level::Warn) {
    fprintf(stderr,"%s\n", buf);
  } else {
    fprintf(stdout,"%s\n", buf);
  }
}
}
}

