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
  //! Dump messages somewhere
  static void log(console::Level level, const char* channel, const char* fmt, ...);
};

}
}


#define LOCAL_LOG_NAMED(level, channel, format, ...) miniros::internal::InternalLog::log((level), (format), __VA_ARGS__);
#define LOCAL_LOG(level, format, ...) miniros::internal::InternalLog::log((level), nullptr, (format), __VA_ARGS__);

#define LOCAL_INFO(format, ...) LOCAL_LOG(miniros::console::Level::Info, format, __VA_ARGS__)
#define LOCAL_DEBUG(format, ...) LOCAL_LOG(miniros::console::Level::Debug, format, __VA_ARGS__)
#define LOCAL_DEBUG_NAMED(channel, format, ...) LOCAL_LOG_NAMED(miniros::console::Level::Debug, (channel),  format, __VA_ARGS__)

#define LOCAL_WARN(format, ...) LOCAL_LOG(miniros::console::Level::Warn, format, __VA_ARGS__)
#define LOCAL_ERROR(format, ...) LOCAL_LOG(miniros::console::Level::Error, format, __VA_ARGS__)

#endif // MINIROS_LOCAL_LOG_H
