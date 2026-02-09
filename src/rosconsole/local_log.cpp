//
// Created by dkargin on 2/3/26.
//

#include <stdarg.h>

#include "miniros/rosconsole/local_log.h"

#include <cstring>

namespace miniros {
namespace internal {

int getVerbosity()
{
  return static_cast<int>(console::Level::Debug);
}

void InternalLog::print(const console::LogLocation& loc, const char* channel,
  const char* file, int line, const char* function,
  const char* fmt, ...) {
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

  ::miniros::console::backend::print(0, loc.level_, buf, file, function, line);
}
}
}

