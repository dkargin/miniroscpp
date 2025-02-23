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

#include "miniros/console.h"
#include "console_impl.h"
#include "miniros/platform.h"
#include "miniros/rostime.h"

#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <sstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <map>
#include <regex>


/// @cond IGNORE

struct DefaultOutputHandler
{
    DefaultOutputHandler(void)
    {
        output_handler_ = static_cast<miniros_console_bridge::OutputHandler*>(&std_output_handler_);
        previous_output_handler_ = output_handler_;
        logLevel_ = miniros_console_bridge::CONSOLE_BRIDGE_LOG_WARN;
    }

    miniros_console_bridge::OutputHandlerSTD std_output_handler_;
    miniros_console_bridge::OutputHandler   *output_handler_;
    miniros_console_bridge::OutputHandler   *previous_output_handler_;
    miniros_console_bridge::LogLevel         logLevel_;
    std::mutex                       lock_; // it is likely the outputhandler does some I/O, so we serialize it
};

// we use this function because we want to handle static initialization correctly
// however, the first run of this function is not thread safe, due to the use of a static
// variable inside the function. For this reason, we ensure the first call happens during
// static initialization using a proxy class
static DefaultOutputHandler* getDOH(void)
{
    static DefaultOutputHandler DOH;
    return &DOH;
}

#define USE_DOH                                                                \
    DefaultOutputHandler *doh = getDOH();                                      \
    std::lock_guard<std::mutex> lock_guard(doh->lock_)

#define MAX_BUFFER_SIZE 1024

/// @endcond

void miniros_console_bridge::noOutputHandler(void)
{
    USE_DOH;
    doh->previous_output_handler_ = doh->output_handler_;
    doh->output_handler_ = NULL;
}

void miniros_console_bridge::restorePreviousOutputHandler(void)
{
    USE_DOH;
    std::swap(doh->previous_output_handler_, doh->output_handler_);
}

void miniros_console_bridge::useOutputHandler(OutputHandler *oh)
{
    USE_DOH;
    doh->previous_output_handler_ = doh->output_handler_;
    doh->output_handler_ = oh;
}

miniros_console_bridge::OutputHandler* miniros_console_bridge::getOutputHandler(void)
{
    return getDOH()->output_handler_;
}

void miniros_console_bridge::log(const char *file, int line, LogLevel level, const char* m, ...)
{
    USE_DOH;
    if (doh->output_handler_ && level >= doh->logLevel_)
    {
        va_list __ap;
        va_start(__ap, m);
        char buf[MAX_BUFFER_SIZE];
#ifdef _MSC_VER
        vsnprintf_s(buf, sizeof(buf), _TRUNCATE, m, __ap);
#else
        vsnprintf(buf, sizeof(buf), m, __ap);
#endif
        va_end(__ap);
        buf[MAX_BUFFER_SIZE - 1] = '\0';

        doh->output_handler_->log(buf, level, file, line);
    }
}

void miniros_console_bridge::setLogLevel(LogLevel level)
{
    USE_DOH;
    doh->logLevel_ = level;
}

miniros_console_bridge::LogLevel miniros_console_bridge::getLogLevel(void)
{
    USE_DOH;
    return doh->logLevel_;
}

static const char* LogLevelString[4] = {"Debug:   ", "Info:    ", "Warning: ", "Error:   "};

void miniros_console_bridge::OutputHandlerSTD::log(const std::string &text, LogLevel level, const char *filename, int line)
{
    if (level >= CONSOLE_BRIDGE_LOG_WARN)
    {
        std::cerr << LogLevelString[level] << text << std::endl;
        std::cerr << "         at line " << line << " in " << filename << std::endl;
        std::cerr.flush();
    }
    else
    {
        std::cout << LogLevelString[level] << text << std::endl;
        std::cout.flush();
    }
}

miniros_console_bridge::OutputHandlerFile::OutputHandlerFile(const char *filename) : OutputHandler()
{
#ifdef _MSC_VER
    errno_t err = fopen_s(&file_, filename, "a");
    if (err != 0 || !file_)
#else
    file_ = fopen(filename, "a");
    if (!file_)
#endif
        std::cerr << "Unable to open log file: '" << filename << "'" << std::endl;
}

miniros_console_bridge::OutputHandlerFile::~OutputHandlerFile(void)
{
    if (file_)
    {
        if (fclose(file_) != 0)
            std::cerr << "Error closing logfile" << std::endl;
    }
}

void miniros_console_bridge::OutputHandlerFile::log(const std::string &text, LogLevel level, const char *filename, int line)
{
    if (file_)
    {
        fprintf(file_, "%s%s\n", LogLevelString[level], text.c_str());
        if(level >= CONSOLE_BRIDGE_LOG_WARN)
            fprintf(file_, "         at line %d in %s\n", line, filename);
        fflush(file_);
    }
}

namespace miniros
{
namespace console
{

bool g_initialized = false;
bool g_shutting_down = false;
std::mutex g_init_mutex;

#ifdef ROSCONSOLE_BACKEND_LOG4CXX
log4cxx::LevelPtr g_level_lookup[levels::Count] =
{
  log4cxx::Level::getDebug(),
  log4cxx::Level::getInfo(),
  log4cxx::Level::getWarn(),
  log4cxx::Level::getError(),
  log4cxx::Level::getFatal(),
};
#endif
std::string g_last_error_message = "Unknown Error";

#ifdef _WIN32
  #define COLOR_NORMAL ""
  #define COLOR_RED ""
  #define COLOR_GREEN ""
  #define COLOR_YELLOW ""
#else
  #define COLOR_NORMAL "\033[0m"
  #define COLOR_RED "\033[31m"
  #define COLOR_GREEN "\033[32m"
  #define COLOR_YELLOW "\033[33m"
#endif
const char* g_format_string = "[${severity}] [${time}]: ${message}";

bool g_force_stdout_line_buffered = false;
bool g_stdout_flush_failure_reported = false;
bool g_color = true;

typedef std::map<std::string, std::string> M_string;
M_string g_extra_fixed_tokens;

void setFixedFilterToken(const std::string& key, const std::string& val)
{
  g_extra_fixed_tokens[key] = val;
}


struct FixedToken : public Token
{
  FixedToken(const std::string& str)
  : str_(str)
  {}

  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    return str_.c_str();
  }

  std::string str_;
};

struct FixedMapToken : public Token
{
  FixedMapToken(const std::string& str)
  : str_(str)
  {}

  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    M_string::iterator it = g_extra_fixed_tokens.find(str_);
    if (it == g_extra_fixed_tokens.end())
    {
      return ("${" + str_ + "}").c_str();
    }

    return it->second.c_str();
  }

  std::string str_;
};

struct PlaceHolderToken : public Token
{
  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    return "PLACEHOLDER";
  }
};

struct SeverityToken : public Token
{
  virtual std::string getString(void*, console::Level level, const char* str, const char* file, const char* function, int line)
  {
    (void)str;
    (void)file;
    (void)function;
    (void)line;
    if (level == Level::Fatal)
    {
      return "FATAL";
    }
    else if (level == Level::Error)
    {
      return "ERROR";
    }
    else if (level == Level::Warn)
    {
      return "WARN";
    }
    else if (level == Level::Info)
    {
      return "INFO";
    }
    else if (level == Level::Debug)
    {
      return "DEBUG";
    }

    return "UNKNO";
  }
};

struct MessageToken : public Token
{
  virtual std::string getString(void*, console::Level, const char* str, const char*, const char*, int)
  {
    return str;
  }
};

struct TimeToken : public Token
{
  explicit TimeToken(const std::string &format) : format_(format) {};

  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    std::stringstream ss;

    if (format_.empty())
    {
      ss << miniros::WallTime::now();
    }
    else
    {
#ifdef USE_SOPHISTICATED_TIME_FORMAT
      boost::posix_time::time_facet *facet = new boost::posix_time::time_facet();
      facet->format(format_.c_str());
      ss.imbue(std::locale(std::locale::classic(), facet));
#endif
      ss << miniros::WallTime::now();
    }

    if (miniros::Time::isValid() && miniros::Time::isSimTime())
    {
      ss << ", ";
#ifdef USE_SOPHISTICATED_TIME_FORMAT
      if (format_.empty())
      {
        ss << miniros::Time::now();
      }
      else
      {
        ss << miniros::Time::now().toBoost();
      }
#else
      ss << miniros::Time::now();
#endif
    }
    return ss.str();
  }

  const std::string format_;
};

struct WallTimeToken : public Token
{
  explicit WallTimeToken(const std::string &format) : format_(format) {};

  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    std::stringstream ss;

    if (format_.empty())
    {
      ss << miniros::WallTime::now();
    }
    else
    {
#ifdef USE_SOPHISTICATED_TIME_FORMAT
      boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
      facet->format(format_.c_str());
      ss.imbue(std::locale(std::locale::classic(), facet));
      ss << miniros::WallTime::now().toBoost();
#else
      ss << miniros::WallTime::now();
#endif
    }

    return ss.str();
  }

  const std::string format_;
};

struct ThreadToken : public Token
{
  virtual std::string getString(void*, console::Level, const char*, const char*, const char*, int)
  {
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
  }
};

struct LoggerToken : public Token
{
  virtual std::string getString(void* logger_handle, console::Level level, const char* str, const char* file, const char* function, int line)
  {
    (void)level;
    (void)str;
    (void)file;
    (void)function;
    (void)line;
    return console::impl::getName(logger_handle);
  }
};

struct FileToken : public Token
{
  virtual std::string getString(void*, console::Level,
    const char*, const char* file, const char*, int)
  {
    return file;
  }
};

struct FunctionToken : public Token
{
  virtual std::string getString(void*, console::Level,
    const char*, const char*, const char* function, int)
  {
    return function;
  }
};

struct LineToken : public Token
{
  virtual std::string getString(void*, Level, const char*, const char*, const char*, int line)
  {
    std::stringstream ss;
    ss << line;
    return ss.str();
  }
};

TokenPtr createTokenFromType(const std::string& type)
{
  if (type == "severity")
  {
    return TokenPtr(std::make_shared<SeverityToken>());
  }
  else if (type == "message")
  {
    return TokenPtr(std::make_shared<MessageToken>());
  }
  else if (type == "time" || strncmp(type.c_str(), std::string("time:").c_str(), std::string("time:").size()) == 0)
  {
    std::string format;

    std::size_t found = type.find(':');
    if (found != std::string::npos)
    {
      format = type.substr(found + 1, type.size());
    }

    return TokenPtr(std::make_shared<TimeToken>(format));
  }
  else if (type == "walltime" || strncmp(type.c_str(), std::string("walltime:").c_str(), std::string("walltime:").size()) == 0)
  {
    std::string format;

    std::size_t found = type.find(':');
    if (found != std::string::npos)
    {
      format = type.substr(found + 1, type.size());
    }

    return TokenPtr(std::make_shared<WallTimeToken>(format));
  }
  else if (type == "thread")
  {
    return TokenPtr(std::make_shared<ThreadToken>());
  }
  else if (type == "logger")
  {
    return TokenPtr(std::make_shared<LoggerToken>());
  }
  else if (type == "file")
  {
    return TokenPtr(std::make_shared<FileToken>());
  }
  else if (type == "line")
  {
    return TokenPtr(std::make_shared<LineToken>());
  }
  else if (type == "function")
  {
    return TokenPtr(std::make_shared<FunctionToken>());
  }

  return TokenPtr(std::make_shared<FixedMapToken>(type));
}

void Formatter::init(const char* fmt)
{
  format_ = fmt;

  std::regex e("\\$\\{([^\\}]+)\\}");
  std::match_results<std::string::const_iterator> results;
  std::string::const_iterator start, end;
  start = format_.begin();
  end = format_.end();
  bool matched_once = false;
  std::string last_suffix;
  while (std::regex_search(start, end, results, e))
  {
#if 0
    for (size_t i = 0; i < results.size(); ++i)
    {
      std::cout << i << "|" << results.prefix() << "|" <<  results[i] << "|" << results.suffix() << std::endl;
    }
#endif

    std::string token = results[1];
    last_suffix = results.suffix();
    tokens_.push_back(TokenPtr(std::make_shared<FixedToken>(results.prefix())));
    tokens_.push_back(createTokenFromType(token));

    start = results[0].second;
    matched_once = true;
  }

  if (matched_once)
  {
    tokens_.push_back(TokenPtr(std::make_shared<FixedToken>(last_suffix)));
  }
  else
  {
    tokens_.push_back(TokenPtr(std::make_shared<FixedToken>(format_)));
  }
}

void Formatter::print(void* logger_handle, Level level, const char* str, const char* file, const char* function, int line)
{
  // print in red to stderr if level doesn't match any of the predefined ones
  const char* color = COLOR_RED;
  FILE* f = stderr;

  if (level == Level::Fatal)
  {
    color = COLOR_RED;
  }
  else if (level == Level::Error)
  {
    color = COLOR_RED;
  }
  else if (level == Level::Warn)
  {
    color = COLOR_YELLOW;
  }
  else if (level == Level::Info)
  {
    color = COLOR_NORMAL;
    f = stdout;
  }
  else if (level == Level::Debug)
  {
    color = COLOR_GREEN;
    f = stdout;
  }

  std::stringstream ss;
  if (g_color)
  {
    ss << color;
  }
  ss << getTokenStrings(logger_handle, level, str, file, function, line);
  if (g_color)
  {
    ss << COLOR_NORMAL;
  }

  fprintf(f, "%s\n", ss.str().c_str());
  
  if (g_force_stdout_line_buffered && f == stdout)
  {
    int flush_result = fflush(f);
    if (flush_result != 0 && !g_stdout_flush_failure_reported)
    {
      g_stdout_flush_failure_reported = true;
      fprintf(stderr, "Error: failed to perform fflush on stdout, fflush return code is %d\n", flush_result);
    }
  }
}

std::string Formatter::getTokenStrings(void *logger_handle, Level level,
    const char *str, const char *file,
    const char *function, int line) const
{
  std::stringstream ss;

  for (V_Token::const_iterator it = tokens_.begin(); it != tokens_.end(); ++it)
  {
    ss << (*it)->getString(logger_handle, level, str, file, function, line);
  }

  return ss.str();
}

Formatter g_formatter;


void _print(void* logger_handle, Level level,
    const char* str, const char* file, const char* function, int line)
{
  g_formatter.print(logger_handle, level, str, file, function, line);
}

void initialize()
{
  std::scoped_lock<std::mutex> lock(g_init_mutex);

  if (!g_initialized)
  {
    // Check for the format string environment variable
    char* format_string = NULL;
#ifdef _MSC_VER
    _dupenv_s(&format_string, NULL, "ROSCONSOLE_FORMAT");
#else
    format_string =  getenv("ROSCONSOLE_FORMAT");
#endif
    if (format_string)
    {
      g_format_string = format_string;
    }

    g_formatter.init(g_format_string);
    backend::function_notifyLoggerLevelsChanged = notifyLoggerLevelsChanged;
    backend::function_print = _print;

    std::string line_buffered;
    if (get_environment_variable(line_buffered, "ROSCONSOLE_STDOUT_LINE_BUFFERED"))
    {
      if (line_buffered == "1")
      {
        g_force_stdout_line_buffered = true;
      }
      else if (line_buffered != "0")
      {
        fprintf(stderr, "Warning: unexpected value %s specified for ROSCONSOLE_STDOUT_LINE_BUFFERED. Default value 0 "
          "will be used. Valid values are 1 or 0.\n", line_buffered.c_str());
      }
    }

    std::string no_color;
    if (get_environment_variable(no_color, "NO_COLOR"))
    {
      g_color = false;
    }

    ::miniros::console::impl::initialize();
    g_initialized = true;
  }
}

void vformatToBuffer(std::shared_ptr<char[]>& buffer, size_t& buffer_size, const char* fmt, va_list args)
{
  va_list arg_copy;
  va_copy(arg_copy, args);
  size_t total = vsnprintf(buffer.get(), buffer_size, fmt, args);
  if (total >= buffer_size)
  {
    buffer_size = total + 1;
    buffer.reset(new char[buffer_size]);
    vsnprintf(buffer.get(), buffer_size, fmt, arg_copy);
  }
  va_end(arg_copy);
}

void formatToBuffer(std::shared_ptr<char[]>& buffer, size_t& buffer_size, const char* fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  vformatToBuffer(buffer, buffer_size, fmt, args);

  va_end(args);
}

std::string formatToString(const char* fmt, ...)
{
  std::shared_ptr<char[]> buffer;
  size_t size = 0;

  va_list args;
  va_start(args, fmt);

  vformatToBuffer(buffer, size, fmt, args);

  va_end(args);

  return std::string(buffer.get(), size);
}

#define INITIAL_BUFFER_SIZE 4096
static std::mutex g_print_mutex;
static std::shared_ptr<char[]> g_print_buffer(new char[INITIAL_BUFFER_SIZE]);
static size_t g_print_buffer_size = INITIAL_BUFFER_SIZE;
static std::thread::id g_printing_thread_id;

void print(FilterBase* filter, void* logger_handle, Level level,
    const char* file, int line, const char* function, const char* fmt, ...)
{
  if (g_shutting_down)
    return;

  if (g_printing_thread_id == std::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  std::scoped_lock<std::mutex> lock(g_print_mutex);

  g_printing_thread_id = std::this_thread::get_id();

  va_list args;
  va_start(args, fmt);

  vformatToBuffer(g_print_buffer, g_print_buffer_size, fmt, args);

  va_end(args);

  bool enabled = true;

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_handle;
    params.message = g_print_buffer.get();
    enabled = filter->isEnabled(params);
    level = params.level;

    if (!params.out_message.empty())
    {
      size_t msg_size = params.out_message.size();
      if (g_print_buffer_size <= msg_size)
      {
        g_print_buffer_size = msg_size + 1;
        g_print_buffer.reset(new char[g_print_buffer_size]);
      }

      memcpy(g_print_buffer.get(), params.out_message.c_str(), msg_size + 1);
    }
  }

  if (enabled)
  {
    if (level == Level::Error)
    {
      g_last_error_message = g_print_buffer.get();
    }
    try
    {
      ::miniros::console::impl::print(logger_handle, level, g_print_buffer.get(), file, function, line);
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

  g_printing_thread_id = std::thread::id();
}

void print(FilterBase* filter, void* logger_handle, Level level,
       const std::stringstream& ss, const char* file, int line, const char* function)
{
  if (g_shutting_down)
    return;

  if (g_printing_thread_id == std::this_thread::get_id())
  {
    fprintf(stderr, "Warning: recursive print statement has occurred.  Throwing out recursive print.\n");
    return;
  }

  std::scoped_lock<std::mutex> lock(g_print_mutex);

  g_printing_thread_id = std::this_thread::get_id();

  bool enabled = true;
  std::string str = ss.str();

  if (filter)
  {
    FilterParams params;
    params.file = file;
    params.function = function;
    params.line = line;
    params.level = level;
    params.logger = logger_handle;
    params.message = str.c_str();
    enabled = filter->isEnabled(params);
    level = params.level;

    if (!params.out_message.empty())
    {
      str = params.out_message;
    }
  }

  if (enabled)
  {
    if (level == Level::Error)
    {
      g_last_error_message = str;
    }
    try
    {
      ::miniros::console::impl::print(logger_handle, level, str.c_str(), file, function, line);
    }
    catch (std::exception& e)
    {
      fprintf(stderr, "Caught exception while logging: [%s]\n", e.what());
    }
  }

  g_printing_thread_id = std::thread::id();
}

std::vector<LogLocation*> g_log_locations;
std::mutex g_locations_mutex;

void registerLogLocation(LogLocation* loc)
{
  std::scoped_lock<std::mutex> lock(g_locations_mutex);

  g_log_locations.push_back(loc);
}

void checkLogLocationEnabledNoLock(LogLocation* loc)
{
  loc->logger_enabled_ = ::miniros::console::impl::isEnabledFor(loc->logger_, loc->level_);
}

void initializeLogLocation(LogLocation* loc, const std::string& name, Level level)
{
  std::scoped_lock<std::mutex> lock(g_locations_mutex);

  if (loc->initialized_)
  {
    return;
  }

  loc->logger_ = ::miniros::console::impl::getHandle(name);
  loc->level_ = level;

  g_log_locations.push_back(loc);

  checkLogLocationEnabledNoLock(loc);

  loc->initialized_ = true;
}

void setLogLocationLevel(LogLocation* loc, Level level)
{
  std::scoped_lock<std::mutex> lock(g_locations_mutex);
  loc->level_ = level;
}

void checkLogLocationEnabled(LogLocation* loc)
{
  std::scoped_lock<std::mutex> lock(g_locations_mutex);
  checkLogLocationEnabledNoLock(loc);
}

void notifyLoggerLevelsChanged()
{
  std::scoped_lock<std::mutex> lock(g_locations_mutex);

  for (LogLocation* loc: g_log_locations)
    checkLogLocationEnabledNoLock(loc);
}

void initializeSafe()
{
  do
  {
    if (MINIROS_UNLIKELY(!::miniros::console::g_initialized))
    {
      ::miniros::console::initialize();
    }
  } while(false);
}

class StaticInit
{
public:
  StaticInit()
  {
      initializeSafe();
  }
};

StaticInit g_static_init;


void register_appender(LogAppender* appender)
{
  miniros::console::impl::register_appender(appender);
}

void deregister_appender(LogAppender* appender)
{
  miniros::console::impl::deregister_appender(appender);
}

void shutdown()
{
  g_shutting_down = true;
  miniros::console::impl::shutdown();
}

bool get_loggers(std::map<std::string, Level>& loggers)
{
  return miniros::console::impl::get_loggers(loggers);
}

bool set_logger_level(const std::string& name, Level level)
{
  return miniros::console::impl::set_logger_level(name, level);
}

namespace backend
{

void notifyLoggerLevelsChanged()
{
  if (function_notifyLoggerLevelsChanged)
  {
    function_notifyLoggerLevelsChanged();
  }
}

void (*function_notifyLoggerLevelsChanged)() = 0;

void print(void* logger_handle, console::Level level, const char* str, const char* file, const char* function, int line)
{
  if (function_print)
  {
    function_print(logger_handle, level, str, file, function, line);
  }
}

void (*function_print)(void*, console::Level, const char*, const char*, const char*, int) = 0;

} // namespace backend
} // namespace console
} // namespace miniros

