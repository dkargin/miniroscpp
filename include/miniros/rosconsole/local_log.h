//
// Created by dkargin on 2/3/26.
//

#ifndef MINIROS_LOCAL_LOG_H
#define MINIROS_LOCAL_LOG_H

#include <string>

#include "miniros/macros.h"
#include "miniros/console.h"

namespace miniros {

namespace internal {

//! Utilities for XML parsing, encoding, and decoding and message handlers.
class MINIROS_DECL InternalLog {
public:
  //! Print message without using any network infrastructure.
  static void print(const console::LogLocation& loc, const char* channel,
    const char* file, int line, const char* function,
    const char* fmt, ...) MINIROS_CONSOLE_PRINTF_ATTRIBUTE(6, 7);
};

}
}

// These log utilities follow regular rosconsole macros, but messages are written only to stdout.

#define LOCAL_LOG_COND(cond, level, name, ...) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(cond, level, name); \
    \
    if (MINIROS_UNLIKELY(enabled)) \
    { \
      ::miniros::internal::InternalLog::print(loc, (name), __FILE__, __LINE__, __MINIROS_CONSOLE_FUNCTION__, __VA_ARGS__); \
    } \
  } while(0)

#define LOCAL_LOG(level, ...) LOCAL_LOG_COND(true, level, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)

#define LOCAL_DEBUG(...) LOCAL_LOG(::miniros::console::Level::Debug, __VA_ARGS__)
#define LOCAL_INFO(...) LOCAL_LOG(::miniros::console::Level::Info, __VA_ARGS__)
#define LOCAL_WARN(...) LOCAL_LOG(miniros::console::Level::Warn, __VA_ARGS__)
#define LOCAL_ERROR(...) LOCAL_LOG(::miniros::console::Level::Error, __VA_ARGS__)

#define LOCAL_DEBUG_NAMED(channel, ...) LOCAL_LOG_COND(true, ::miniros::console::Level::Debug, (channel),  __VA_ARGS__)
#define LOCAL_WARN_NAMED(channel, ...) LOCAL_LOG_COND(true, ::miniros::console::Level::Warn, (channel), __VA_ARGS__)


#endif // MINIROS_LOCAL_LOG_H
