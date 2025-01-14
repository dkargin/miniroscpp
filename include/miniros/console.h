/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ryan Luna, Ioan Sucan */

#pragma once

#include <map>
#include <string>
#include <memory>
#include <vector>
#include <sstream>

#include "miniros/macros.h"


#ifdef __GNUC__
#if __GNUC__ >= 3
#define MINIROS_CONSOLE_PRINTF_ATTRIBUTE(a, b) __attribute__ ((__format__ (__printf__, a, b)))
#endif
#endif

#ifndef MINIROS_CONSOLE_PRINTF_ATTRIBUTE
#define MINIROS_CONSOLE_PRINTF_ATTRIBUTE(a, b)
#endif

/** \file console.h
    \defgroup logging Logging Macros
    \{

    \def CONSOLE_BRIDGE_logError(fmt, ...)
    \brief Log a formatted error string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def CONSOLE_BRIDGE_logWarn(fmt, ...)
    \brief Log a formatted warning string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def CONSOLE_BRIDGE_logInform(fmt, ...)
    \brief Log a formatted information string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \def CONSOLE_BRIDGE_logDebug(fmt, ...)
    \brief Log a formatted debugging string.
    \remarks This macro takes the same arguments as <a href="http://www.cplusplus.com/reference/clibrary/cstdio/printf">printf</a>.

    \}
*/
#define MINIROS_CONSOLE_BRIDGE_logError(fmt, ...)  \
  miniros_console_bridge::log(__FILE__, __LINE__, miniros_console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)

#define MINIROS_CONSOLE_BRIDGE_logWarn(fmt, ...)   \
  miniros_console_bridge::log(__FILE__, __LINE__, miniros_console_bridge::CONSOLE_BRIDGE_LOG_WARN,  fmt, ##__VA_ARGS__)

#define MINIROS_CONSOLE_BRIDGE_logInform(fmt, ...) \
  miniros_console_bridge::log(__FILE__, __LINE__, miniros_console_bridge::CONSOLE_BRIDGE_LOG_INFO,  fmt, ##__VA_ARGS__)

#define MINIROS_CONSOLE_BRIDGE_logDebug(fmt, ...)  \
  miniros_console_bridge::log(__FILE__, __LINE__, miniros_console_bridge::CONSOLE_BRIDGE_LOG_DEBUG, fmt, ##__VA_ARGS__)


/** \brief Message namespace. This contains classes needed to
    output error messages (or logging) from within the library.
    Message logging can be performed with \ref logging "logging macros" */
namespace miniros_console_bridge
{
/** \brief The set of priorities for message logging */
enum MINIROS_DECL LogLevel
  {
    CONSOLE_BRIDGE_LOG_DEBUG = 0,
    CONSOLE_BRIDGE_LOG_INFO,
    CONSOLE_BRIDGE_LOG_WARN,
    CONSOLE_BRIDGE_LOG_ERROR,
    CONSOLE_BRIDGE_LOG_NONE
  };

/** \brief Generic class to handle output from a piece of
    code.
    
    In order to handle output from the library in different
    ways, an implementation of this class needs to be
    provided. This instance can be set with the useOutputHandler
    function. */
class MINIROS_DECL OutputHandler
{
public:
  
  OutputHandler(void)
  {
  }
  
  virtual ~OutputHandler(void)
  {
  }
  
  /** \brief log a message to the output handler with the given text
      and logging level from a specific file and line number */
  virtual void log(const std::string &text, LogLevel level, const char *filename, int line) = 0;
};

/** \brief Default implementation of OutputHandler. This sends
    the information to the console. */
class MINIROS_DECL OutputHandlerSTD : public OutputHandler
{
public:
  
  OutputHandlerSTD(void) : OutputHandler()
  {
  }
  
  virtual void log(const std::string &text, LogLevel level, const char *filename, int line);
  
};

/** \brief Implementation of OutputHandler that saves messages in a file. */
class MINIROS_DECL OutputHandlerFile : public OutputHandler
{
public:
  
  /** \brief The name of the file in which to save the message data */
  OutputHandlerFile(const char *filename);
  
  virtual ~OutputHandlerFile(void);
  
  virtual void log(const std::string &text, LogLevel level, const char *filename, int line);
  
private:
  
  /** \brief The file to save to */
  FILE *file_;
  
};

/** \brief This function instructs ompl that no messages should be outputted. Equivalent to useOutputHandler(NULL) */
MINIROS_DECL void noOutputHandler(void);

/** \brief Restore the output handler that was previously in use (if any) */
MINIROS_DECL void restorePreviousOutputHandler(void);

/** \brief Specify the instance of the OutputHandler to use. By default, this is OutputHandlerSTD */
MINIROS_DECL void useOutputHandler(OutputHandler *oh);

/** \brief Get the instance of the OutputHandler currently used. This is NULL in case there is no output handler. */
MINIROS_DECL OutputHandler* getOutputHandler(void);

/** \brief Set the minimum level of logging data to output.  Messages
    with lower logging levels will not be recorded. */
MINIROS_DECL void setLogLevel(LogLevel level);

/** \brief Retrieve the current level of logging data.  Messages
    with lower logging levels will not be recorded. */
MINIROS_DECL LogLevel getLogLevel(void);

/** \brief Root level logging function.  This should not be invoked directly,
    but rather used via a \ref logging "logging macro".  Formats the message
    string given the arguments and forwards the string to the output handler */
MINIROS_DECL void log(const char *file, int line, LogLevel level, const char* m, ...);
} // namespace miniros_console_bridge

namespace miniros
{
namespace console
{

enum class Level
{
  Debug,
  Info,
  Warn,
  Error,
  Fatal,

  Count
};

class LogAppender
{
public:

  virtual ~LogAppender() {}

  virtual void log(Level level, const char* str, const char* file, const char* function, int line) = 0;

};

struct Token
{
  virtual ~Token() {}
  /*
   * @param level
   * @param message
   * @param file
   * @param function
   * @param  line
   */
  virtual std::string getString(void*, ::miniros::console::Level, const char*, const char*, const char*, int) = 0;
};

typedef std::shared_ptr<Token> TokenPtr;
typedef std::vector<TokenPtr> V_Token;

MINIROS_DECL void register_appender(LogAppender* appender);

MINIROS_DECL void deregister_appender(LogAppender* appender);

struct MINIROS_DECL Formatter
{
  void init(const char* fmt);
  void print(void* logger_handle, Level level, const char* str, const char* file, const char* function, int line);

  std::string getTokenStrings(void *logger_handle, Level level, const char *str,
    const char *file, const char *function, int line) const;

  std::string format_;
  V_Token tokens_;
};

/**
 * \brief Only exported because the implementation need it.  Do not use directly.
 */
extern MINIROS_DECL Formatter g_formatter;

/**
 * \brief Only exported because the macros need it.  Do not use directly.
 */
extern MINIROS_DECL bool g_initialized;

/**
 * \brief Don't call this directly.  Performs any required initialization/configuration.  Happens automatically when using the macro API.
 *
 * If you're going to be using log4cxx or any of the ::ros::console functions, and need the system to be initialized, use the
 * ROSCONSOLE_AUTOINIT macro.
 */
MINIROS_DECL void initialize();

/**
 * Performs any required initialization. This variant waits until initialization is actually complete.
 */
MINIROS_DECL void initializeSafe();

MINIROS_DECL void shutdown();

extern MINIROS_DECL bool get_loggers(std::map<std::string, Level>& loggers);
extern MINIROS_DECL bool set_logger_level(const std::string& name, Level level);

MINIROS_DECL void notifyLoggerLevelsChanged();

MINIROS_DECL void setFixedFilterToken(const std::string& key, const std::string& val);

/**
 * \brief Only exported because the TopicManager need it.  Do not use directly.
 */
extern MINIROS_DECL std::string g_last_error_message;

/**
 * \brief Parameter structure passed to FilterBase::isEnabled(...);.  Includes both input and output parameters
 */
struct FilterParams
{
  // input parameters
  const char* file;                         ///< [input] File the message came from
  int line;                                 ///< [input] Line the message came from
  const char* function;                     ///< [input] Function the message came from
  const char* message;                      ///< [input] The formatted message that will be output

  // input/output parameters
  void* logger;                             ///< [input/output] Handle identifying logger that this message will be output to.  If changed, uses the new logger
  Level level;                              ///< [input/output] Severity level.  If changed, uses the new level

  // output parameters
  std::string out_message;                  ///< [output] If set, writes this message instead of the original
};

/**
 * \brief Base-class for filters.  Filters allow full user-defined control over whether or not a message should print.
 * The ROS_X_FILTER... macros provide the filtering functionality.
 *
 * Filters get a chance to veto the message from printing at two times: first before the message arguments are
 * evaluated and the message is formatted, and then once the message is formatted before it is printed.  It is also possible
 * to change the message, logger and severity level at this stage (see the FilterParams struct for more details).
 *
 * When a ROS_X_FILTER... macro is called, here is the high-level view of how it uses the filter passed in:
\verbatim
if (<logging level is enabled> && filter->isEnabled())
{
  <format message>
  <fill out FilterParams>
  if (filter->isEnabled(params))
  {
    <print message>
  }
}
\endverbatim
 */
class FilterBase
{
public:
  virtual ~FilterBase() {}
  /**
   * \brief Returns whether or not the log statement should be printed.  Called before the log arguments are evaluated
   * and the message is formatted.
   */
  inline virtual bool isEnabled() { return true; }
  /**
   * \brief Returns whether or not the log statement should be printed.  Called once the message has been formatted,
   * and allows you to change the message, logger and severity level if necessary.
   */
  inline virtual bool isEnabled(FilterParams&) { return true; }
};

struct MINIROS_DECL LogLocation;
/**
 * \brief Internal
 */
MINIROS_DECL void initializeLogLocation(LogLocation* loc, const std::string& name, Level level);
/**
 * \brief Internal
 */
MINIROS_DECL void setLogLocationLevel(LogLocation* loc, Level level);
/**
 * \brief Internal
 */
MINIROS_DECL void checkLogLocationEnabled(LogLocation* loc);

/**
 * \brief Internal
 */
struct LogLocation
{
  bool initialized_;
  bool logger_enabled_;
  Level level_;
  void* logger_;
};

MINIROS_DECL void vformatToBuffer(std::shared_ptr<char[]>& buffer, size_t& buffer_size, const char* fmt, va_list args);
MINIROS_DECL void formatToBuffer(std::shared_ptr<char[]>& buffer, size_t& buffer_size, const char* fmt, ...);
MINIROS_DECL std::string formatToString(const char* fmt, ...);

/**
 * \brief Don't call this directly.  Use the ROS_LOG() macro instead.
 * @param level Logging level
 * @param file File this logging statement is from (usually generated with __FILE__)
 * @param line Line of code this logging statement is from (usually generated with __LINE__)
 * @param fmt Format string
 */
MINIROS_DECL void print(FilterBase* filter, void* logger, Level level,
     const char* file, int line,
     const char* function, const char* fmt, ...) MINIROS_CONSOLE_PRINTF_ATTRIBUTE(7, 8);

MINIROS_DECL void print(FilterBase* filter, void* logger, Level level,
     const std::stringstream& str, const char* file, int line, const char* function);

namespace backend
{

MINIROS_DECL void notifyLoggerLevelsChanged();

MINIROS_DECL extern void (*function_notifyLoggerLevelsChanged)();

MINIROS_DECL void print(void* logger_handle, console::Level level, const char* str, const char* file, const char* function, int line);

MINIROS_DECL extern void (*function_print)(void*, console::Level, const char*, const char*, const char*, int);

} // namespace backend
} // namespace console
} // namespace miniros



#ifdef WIN32
#define MINIROS_LIKELY(x)       (x)
#define MINIROS_UNLIKELY(x)     (x)
#else
#define MINIROS_LIKELY(x)       __builtin_expect((x),1)
#define MINIROS_UNLIKELY(x)     __builtin_expect((x),0)
#endif


#if defined(MSVC)
#define __MINIROS_CONSOLE_FUNCTION__ __FUNCSIG__
#elif defined(__GNUC__)
#define __MINIROS_CONSOLE_FUNCTION__ __PRETTY_FUNCTION__
#else
#define __MINIROS_CONSOLE_FUNCTION__ ""
#endif


#ifdef MINIROS_PACKAGE_NAME
#define MINIROS_CONSOLE_PACKAGE_NAME MINIROS_PACKAGE_NAME
#else
#define MINIROS_CONSOLE_PACKAGE_NAME "unknown_package"
#endif

#define MINIROS_CONSOLE_ROOT_LOGGER_NAME "miniros"
#define MINIROS_CONSOLE_NAME_PREFIX MINIROS_CONSOLE_ROOT_LOGGER_NAME "." MINIROS_CONSOLE_PACKAGE_NAME
#define MINIROS_CONSOLE_DEFAULT_NAME MINIROS_CONSOLE_NAME_PREFIX

/**
 * Initializes the rosconsole library.  Usually unnecessary to call directly.
 */
#define MINIROS_CONSOLE_AUTOINIT \
  do \
  { \
    if (MINIROS_UNLIKELY(!::miniros::console::g_initialized)) \
    { \
      ::miniros::console::initialize(); \
    } \
  } while(0)

#define MINIROS_CONSOLE_DEFINE_LOCATION(cond, level, name) \
  MINIROS_CONSOLE_AUTOINIT; \
  static ::miniros::console::LogLocation loc = {false, false, ::miniros::console::Level::Count, 0}; /* Initialized at compile-time */ \
  if (MINIROS_UNLIKELY(!loc.initialized_)) \
  { \
    initializeLogLocation(&loc, name, level); \
  } \
  if (MINIROS_UNLIKELY(loc.level_ != level)) \
  { \
    setLogLocationLevel(&loc, level); \
    checkLogLocationEnabled(&loc); \
  } \
  bool enabled = loc.logger_enabled_ && (cond);

#define MINIROS_CONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, ...) \
    ::miniros::console::print(filter, loc.logger_, loc.level_, __FILE__, __LINE__, __MINIROS_CONSOLE_FUNCTION__, __VA_ARGS__)

#define MINIROS_CONSOLE_PRINT_AT_LOCATION(...) \
    MINIROS_CONSOLE_PRINT_AT_LOCATION_WITH_FILTER(0, __VA_ARGS__)

#define MINIROS_CONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args) \
  do \
  { \
    std::stringstream ss; \
    ss << args; \
    ::miniros::console::print(filter, loc.logger_, loc.level_, ss, __FILE__, __LINE__, __MINIROS_CONSOLE_FUNCTION__); \
  } while (0)

#define MINIROS_CONSOLE_PRINT_STREAM_AT_LOCATION(args) \
    MINIROS_CONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(0, args)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with printf-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_COND(cond, level, name, ...) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(cond, level, name); \
    \
    if (MINIROS_UNLIKELY(enabled)) \
    { \
      MINIROS_CONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only if a given condition has been met, with stream-style formatting
 *
 * \note The condition will only be evaluated if this logging statement is enabled
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_STREAM_COND(cond, level, name, args) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(cond, level, name); \
    if (MINIROS_UNLIKELY(enabled)) \
    { \
      MINIROS_CONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_ONCE(level, name, ...) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool hit = false; \
    if (MINIROS_UNLIKELY(enabled) && MINIROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      MINIROS_CONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, only the first time it is hit when enabled, with printf-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_STREAM_ONCE(level, name, args) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    static bool hit = false; \
    if (MINIROS_UNLIKELY(enabled) && MINIROS_UNLIKELY(!hit)) \
    { \
      hit = true; \
      MINIROS_CONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define MINIROS_LOG_THROTTLE(rate, level, name, ...) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::Time now = ::ros::Time::now(); \
    if (MINIROS_UNLIKELY(enabled) && MINIROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      MINIROS_CONSOLE_PRINT_AT_LOCATION(__VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, limited to a specific rate of printing, with printf-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 * \param rate The rate it should actually trigger at
 */
#define MINIROS_LOG_STREAM_THROTTLE(rate, level, name, args) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    static double last_hit = 0.0; \
    ::ros::Time now = ::ros::Time::now(); \
    if (MINIROS_UNLIKELY(enabled) && MINIROS_UNLIKELY(last_hit + rate <= now.toSec())) \
    { \
      last_hit = now.toSec(); \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION(args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with printf-style formatting
 *
 * \param filter pointer to the filter to be used
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_FILTER(filter, level, name, ...) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    if (MINIROS_UNLIKELY(enabled) && (filter)->isEnabled()) \
    { \
      MINIROS_CONSOLE_PRINT_AT_LOCATION_WITH_FILTER(filter, __VA_ARGS__); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with user-defined filtering, with stream-style formatting
 *
 * \param cond Boolean condition to be evaluated
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_STREAM_FILTER(filter, level, name, args) \
  do \
  { \
    MINIROS_CONSOLE_DEFINE_LOCATION(true, level, name); \
    if (ROS_UNLIKELY(enabled) && (filter)->isEnabled()) \
    { \
      ROSCONSOLE_PRINT_STREAM_AT_LOCATION_WITH_FILTER(filter, args); \
    } \
  } while(0)

/**
 * \brief Log to a given named logger at a given verbosity level, with printf-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG(level, name, ...) MINIROS_LOG_COND(true, level, name, __VA_ARGS__)
/**
 * \brief Log to a given named logger at a given verbosity level, with stream-style formatting
 *
 * \param level One of the levels specified in ::miniros::console::Level::Level
 * \param name Name of the logger.  Note that this is the fully qualified name, and does NOT include "ros.<package_name>".  Use MINIROS_CONSOLE_DEFAULT_NAME if you would like to use the default name.
 */
#define MINIROS_LOG_STREAM(level, name, args) MINIROS_LOG_STREAM_COND(true, level, name, args)

#define MINIROS_DEBUG(...) MINIROS_LOG(::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM(args) MINIROS_LOG_STREAM(::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_DEBUG_NAMED(name, ...) MINIROS_LOG(::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_NAMED(name, args) MINIROS_LOG_STREAM(::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_INFO(...) MINIROS_LOG(::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_INFO_STREAM(args) MINIROS_LOG_STREAM(::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_INFO_NAMED(name, ...) MINIROS_LOG(::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_INFO_STREAM_NAMED(name, args) MINIROS_LOG_STREAM(::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_WARN(...) MINIROS_LOG(::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_WARN_STREAM(args) MINIROS_LOG_STREAM(::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_WARN_NAMED(name, ...) MINIROS_LOG(::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_WARN_STREAM_NAMED(name, args) MINIROS_LOG_STREAM(::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_ERROR(...) MINIROS_LOG(::miniros::console::Level::Error, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_ERROR_STREAM(args) MINIROS_LOG_STREAM(::miniros::console::Level::Error, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_ERROR_NAMED(name, ...) MINIROS_LOG(::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_NAMED(name, args) MINIROS_LOG_STREAM(::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_FATAL(...) MINIROS_LOG(::miniros::console::Level::Fatal, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_FATAL_STREAM(args) MINIROS_LOG_STREAM(::miniros::console::Level::Fatal, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_FATAL_NAMED(name, ...) MINIROS_LOG(::miniros::console::Level::Fatal, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_NAMED(name, args) MINIROS_LOG_STREAM(::miniros::console::Level::Fatal, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#ifdef MINIROS_SUPPORT_COMPLICATED_LOGGING
// These logging macros do not seem to be absolutely necessary and can be implemented on customer side.
#define MINIROS_DEBUG_COND(cond, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_COND(cond, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_DEBUG_COND_NAMED(cond, name, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_COND_NAMED(cond, name, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_DEBUG_ONCE(...) ROS_LOG_ONCE(::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_ONCE(args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_DEBUG_ONCE_NAMED(name, ...) ROS_LOG_ONCE(::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_ONCE_NAMED(name, args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_DEBUG_THROTTLE(rate, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_THROTTLE(rate, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_DEBUG_THROTTLE_NAMED(rate, name, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_THROTTLE_NAMED(rate, name, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_DEBUG_FILTER(filter, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_FILTER(filter, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Debug, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_DEBUG_FILTER_NAMED(filter, name, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_DEBUG_STREAM_FILTER_NAMED(filter, name, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Debug, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_INFO_COND(cond, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_INFO_STREAM_COND(cond, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_INFO_COND_NAMED(cond, name, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_INFO_STREAM_COND_NAMED(cond, name, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_INFO_ONCE(...) ROS_LOG_ONCE(::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_INFO_STREAM_ONCE(args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_INFO_ONCE_NAMED(name, ...) ROS_LOG_ONCE(::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_INFO_STREAM_ONCE_NAMED(name, args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_INFO_THROTTLE(rate, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_INFO_STREAM_THROTTLE(rate, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_INFO_THROTTLE_NAMED(rate, name, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_INFO_STREAM_THROTTLE_NAMED(rate, name, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_INFO_FILTER(filter, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_INFO_STREAM_FILTER(filter, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Info, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_INFO_FILTER_NAMED(filter, name, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_INFO_STREAM_FILTER_NAMED(filter, name, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Info, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_WARN_COND(cond, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_WARN_STREAM_COND(cond, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_WARN_COND_NAMED(cond, name, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_WARN_STREAM_COND_NAMED(cond, name, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_WARN_ONCE(...) ROS_LOG_ONCE(::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_WARN_STREAM_ONCE(args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_WARN_ONCE_NAMED(name, ...) ROS_LOG_ONCE(::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_WARN_STREAM_ONCE_NAMED(name, args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_WARN_THROTTLE(rate, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_WARN_STREAM_THROTTLE(rate, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_WARN_THROTTLE_NAMED(rate, name, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_WARN_STREAM_THROTTLE_NAMED(rate, name, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_WARN_FILTER(filter, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_WARN_STREAM_FILTER(filter, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Warn, MINIROS_CONSOLE_DEFAULT_NAME, args)
#define MINIROS_WARN_FILTER_NAMED(filter, name, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_WARN_STREAM_FILTER_NAMED(filter, name, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Warn, std::string(MINIROS_CONSOLE_NAME_PREFIX) + "." + name, args)

#define MINIROS_ERROR_COND(cond, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_COND(cond, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_ERROR_COND_NAMED(cond, name, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_COND_NAMED(cond, name, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, args)
#define MINIROS_ERROR_ONCE(...) ROS_LOG_ONCE(::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_ONCE(args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_ERROR_ONCE_NAMED(name, ...) ROS_LOG_ONCE(::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_ONCE_NAMED(name, args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, args)
#define MINIROS_ERROR_THROTTLE(rate, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_THROTTLE(rate, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_ERROR_THROTTLE_NAMED(rate, name, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_THROTTLE_NAMED(rate, name, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, args)
#define MINIROS_ERROR_FILTER(filter, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_FILTER(filter, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Error, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_ERROR_FILTER_NAMED(filter, name, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, __VA_ARGS__)
#define MINIROS_ERROR_STREAM_FILTER_NAMED(filter, name, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Error, std::string(MINIROS_CONSOLE_PACKAGE_NAME) + "." + name, args)

#define MINIROS_FATAL_COND(cond, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_COND(cond, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_FATAL_COND_NAMED(cond, name, ...) ROS_LOG_COND(cond, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_COND_NAMED(cond, name, args) ROS_LOG_STREAM_COND(cond, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_FATAL_ONCE(...) ROS_LOG_ONCE(::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_ONCE(args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_FATAL_ONCE_NAMED(name, ...) ROS_LOG_ONCE(::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_ONCE_NAMED(name, args) ROS_LOG_STREAM_ONCE(::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_FATAL_THROTTLE(rate, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_THROTTLE(rate, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_FATAL_THROTTLE_NAMED(rate, name, ...) ROS_LOG_THROTTLE(rate, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_THROTTLE_NAMED(rate, name, args) ROS_LOG_STREAM_THROTTLE(rate, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)
#define MINIROS_FATAL_FILTER(filter, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_FILTER(filter, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Fatal, ROSCONSOLE_DEFAULT_NAME, args)
#define MINIROS_FATAL_FILTER_NAMED(filter, name, ...) ROS_LOG_FILTER(filter, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, __VA_ARGS__)
#define MINIROS_FATAL_STREAM_FILTER_NAMED(filter, name, args) ROS_LOG_STREAM_FILTER(filter, ::miniros::console::Level::Fatal, std::string(ROSCONSOLE_NAME_PREFIX) + "." + name, args)

#endif
