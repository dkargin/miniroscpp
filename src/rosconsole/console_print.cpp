/*
 * Copyright (c) 2013, Open Source Robotics Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <atomic>
#include <mutex>
#include <set>

#include "miniros/console.h"
#include "miniros/internal/threading.h"
#include "console_impl.h"

namespace miniros
{
namespace console
{
namespace impl
{

struct Logger {

  /// Number of published messages into this topic.
  std::atomic_int messages = 0;

  /// Log level for this particular logger.
  /// Mutations and reads other than via a stable handle must hold LoggerConfig::mutex.
  Level level = Level::Info;

  explicit Logger(const std::string& name) : m_name(name) {}
  explicit Logger(const std::string& name, Level level) : m_name(name), level(level) {}
  Logger(const Logger&) = delete;

  const std::string& name() const { return m_name; }

protected:
  const std::string m_name;
};

/// Comparator for loggers
struct LoggerCmp {
  using is_transparent = void;

  bool operator () (const Logger& a, const Logger& b) const
  {
    return a.name() < b.name();
  }

  bool operator() (const std::string& lhs, const Logger& rhs) const { return lhs < rhs.name(); }
  bool operator() (const Logger& lhs, const std::string& rhs) const { return lhs.name() < rhs; }
};

struct LoggerConfig {
  std::mutex mutex;

  std::set<Logger, LoggerCmp> loggers GUARDED_BY(mutex);

  LogAppender* rosconsole_print_appender GUARDED_BY(mutex) = nullptr;

  /// Default logging level.
  Level level GUARDED_BY(mutex) = Level::Info;
};

LoggerConfig g_loggerConfig;

void initialize()
{}

void print(void* handle, Level level, const char* str, const char* file, const char* function, int line)
{
  ::miniros::console::backend::print(0, level, str, file, function, line);
  LogAppender* appender = nullptr;
  {
    std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
    appender = g_loggerConfig.rosconsole_print_appender;
  }
  if (appender) {
    appender->log(level, str, file, function, line);
  }
}

bool isEnabledFor(void* handle, Level level)
{
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  if (handle) {
    const Logger* logger = static_cast<const Logger*>(handle);
    return level >= logger->level;
  }
  return level >= g_loggerConfig.level;
}

void* getHandle(const std::string& name)
{
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  auto it = g_loggerConfig.loggers.find(name);
  if (it == g_loggerConfig.loggers.end()) {
    auto p = g_loggerConfig.loggers.emplace(name);
    return &const_cast<Logger&>(*p.first);
  }
  return &const_cast<Logger&>(*it);
}

std::string getName(void* handle)
{
  // Handles point at Logger nodes that stay in g_loggerConfig.loggers for the
  // process lifetime; m_name is immutable. No LoggerConfig mutex needed here.
  if (handle) {
    auto* logger = static_cast<const Logger*>(handle);
    return logger->name().c_str();
  }
  return "";
}

void register_appender(LogAppender* appender)
{
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  g_loggerConfig.rosconsole_print_appender = appender;
}

void deregister_appender(LogAppender* appender){
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  if (g_loggerConfig.rosconsole_print_appender == appender)
  {
    g_loggerConfig.rosconsole_print_appender = nullptr;
  }
}

void shutdown()
{}

bool get_loggers(std::map<std::string, console::Level>& loggers)
{
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  for (const Logger& l : g_loggerConfig.loggers) {
    loggers[l.name()] = l.level;
  }
  return true;
}

bool set_logger_level(const std::string& name, console::Level level)
{
  std::lock_guard<std::mutex> lock(g_loggerConfig.mutex);
  auto it = g_loggerConfig.loggers.find(name);
  if (it == g_loggerConfig.loggers.end()) {
    g_loggerConfig.loggers.emplace(name, level);
  } else {
    /// We are only modifying level
    const_cast<Logger&>(*it).level = level;
  }
  return true;
}

} // namespace impl
} // namespace console
} // namespace ros
