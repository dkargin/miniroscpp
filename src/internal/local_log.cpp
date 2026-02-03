//
// Created by dkargin on 2/3/26.
//

#include <stdarg.h>

#include "miniros/internal/local_log.h"

namespace miniros {
namespace internal {

int getVerbosity()
{
  return static_cast<int>(console::Level::Debug);
}

void InternalLog::log(console::Level level, const char* channel, const char* fmt, ...) {
  if (static_cast<int>(level) > getVerbosity())
    return;
  va_list va;
  char buf[1024];
  va_start( va, fmt);
  int written = std::vsnprintf(buf,sizeof(buf)-1,fmt,va);
  va_end(va);

  if (written < 0)
    return;

  buf[sizeof(buf)-1] = 0;

  if (level == console::Level::Error || level == console::Level::Warn) {
    fprintf(stderr, "%s\n", buf);
  } else {
    fprintf(stdout, "%s\n", buf);
  }
}
}
}

