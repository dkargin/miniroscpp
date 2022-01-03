/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

#include "miniros/init.h"
#include "miniros/names.h"
#include "miniros/xmlrpc_manager.h"
#include "miniros/poll_manager.h"
#include "miniros/connection_manager.h"
#include "miniros/topic_manager.h"
#include "miniros/service_manager.h"
#include "miniros/this_node.h"
#include "miniros/network.h"
#include "miniros/file_log.h"
#include "miniros/callback_queue.h"
#include "miniros/param.h"
#include "miniros/rosout_appender.h"
#include "miniros/subscribe_options.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/internal_timer_manager.h"
#include "xmlrpcpp/XmlRpcSocket.h"

#include "roscpp/GetLoggers.h"
#include "roscpp/SetLoggerLevel.h"
#include "roscpp/Empty.h"

#include <ros/console.h>
#include <ros/time.h>
#include <rosgraph_msgs/Clock.h>

#include <algorithm>

#include <signal.h>

#include <cstdlib>

namespace miniros
{

namespace master
{
void init(const M_string& remappings);
}

namespace this_node
{
void init(const std::string& names, const M_string& remappings, uint32_t options);
}

namespace network
{
void init(const M_string& remappings);
}

namespace param
{
void init(const M_string& remappings);
}

namespace file_log
{
void init(const M_string& remappings);
}

CallbackQueuePtr g_global_queue;
ROSOutAppender* g_rosout_appender;
static CallbackQueuePtr g_internal_callback_queue;

static bool g_initialized = false;
static bool g_started = false;
static bool g_atexit_registered = false;
static std::mutex g_start_mutex;
static bool g_ok = false;
static uint32_t g_init_options = 0;
static bool g_shutdown_requested = false;
static volatile bool g_shutting_down = false;
static std::recursive_mutex g_shutting_down_mutex;
static boost::thread g_internal_queue_thread;

bool isInitialized()
{
  return g_initialized;
}

bool isShuttingDown()
{
  return g_shutting_down;
}

void checkForShutdown()
{
  if (g_shutdown_requested)
  {
    // Since this gets run from within a mutex inside PollManager, we need to prevent ourselves from deadlocking with
    // another thread that's already in the middle of shutdown()
    std::recursive_mutex::scoped_try_lock lock(g_shutting_down_mutex, boost::defer_lock);
    while (!lock.try_lock() && !g_shutting_down)
    {
      miniros::WallDuration(0.001).sleep();
    }

    if (!g_shutting_down)
    {
      shutdown();
    }

    g_shutdown_requested = false;
  }
}

void requestShutdown()
{
  g_shutdown_requested = true;
}

void atexitCallback()
{
  if (ok() && !isShuttingDown())
  {
    ROSCPP_LOG_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles");
    g_started = false; // don't shutdown singletons, because they are already destroyed
    shutdown();
  }
}

void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received.");
    ROS_WARN("Reason given for shutdown: [%s]", reason.c_str());
    requestShutdown();
  }

  result = xmlrpc::responseInt(1, "", 0);
}

bool getLoggers(roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response& resp)
{
  std::map<std::string, miniros::console::levels::Level> loggers;
  bool success = ::miniros::console::get_loggers(loggers);
  if (success)
  {
    for (std::map<std::string, miniros::console::levels::Level>::const_iterator it = loggers.begin(); it != loggers.end(); it++)
    {
      roscpp::Logger logger;
      logger.name = it->first;
      miniros::console::levels::Level level = it->second;
      if (level == miniros::console::levels::Debug)
      {
        logger.level = "debug";
      }
      else if (level == miniros::console::levels::Info)
      {
        logger.level = "info";
      }
      else if (level == miniros::console::levels::Warn)
      {
        logger.level = "warn";
      }
      else if (level == miniros::console::levels::Error)
      {
        logger.level = "error";
      }
      else if (level == miniros::console::levels::Fatal)
      {
        logger.level = "fatal";
      }
      resp.loggers.push_back(logger);
    }
  }
  return success;
}

bool setLoggerLevel(roscpp::SetLoggerLevel::Request& req, roscpp::SetLoggerLevel::Response&)
{
  std::transform(req.level.begin(), req.level.end(), req.level.begin(), (int(*)(int))std::toupper);

  miniros::console::levels::Level level;
  if (req.level == "DEBUG")
  {
    level = miniros::console::levels::Debug;
  }
  else if (req.level == "INFO")
  {
    level = miniros::console::levels::Info;
  }
  else if (req.level == "WARN")
  {
    level = miniros::console::levels::Warn;
  }
  else if (req.level == "ERROR")
  {
    level = miniros::console::levels::Error;
  }
  else if (req.level == "FATAL")
  {
    level = miniros::console::levels::Fatal;
  }
  else
  {
    return false;
  }

  bool success = ::miniros::console::set_logger_level(req.logger, level);
  if (success)
  {
    console::notifyLoggerLevelsChanged();
  }

  return success;
}

bool closeAllConnections(roscpp::Empty::Request&, roscpp::Empty::Response&)
{
  ROSCPP_LOG_DEBUG("close_all_connections service called, closing connections");
  ConnectionManager::instance()->clear(Connection::TransportDisconnect);
  return true;
}

void clockCallback(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  Time::setNow(msg->clock);
}

CallbackQueuePtr getInternalCallbackQueue()
{
  if (!g_internal_callback_queue)
  {
    g_internal_callback_queue.reset(new CallbackQueue);
  }

  return g_internal_callback_queue;
}

void basicSigintHandler(int sig)
{
  (void)sig;
  miniros::requestShutdown();
}

void internalCallbackQueueThreadFunc()
{
  disableAllSignalsInThisThread();

  CallbackQueuePtr queue = getInternalCallbackQueue();

  while (!g_shutting_down)
  {
    queue->callAvailable(WallDuration(0.1));
  }
}

bool isStarted()
{
  return g_started;
}

void start()
{
  std::scoped_lock<std::mutex> lock(g_start_mutex);
  if (g_started)
  {
    return;
  }

  g_shutdown_requested = false;
  g_shutting_down = false;
  g_started = true;
  g_ok = true;

  bool enable_debug = false;
  std::string enable_debug_env;
  if ( get_environment_variable(enable_debug_env,"ROSCPP_ENABLE_DEBUG") )
  {
    try
    {
      enable_debug = boost::lexical_cast<bool>(enable_debug_env.c_str());
    }
    catch (boost::bad_lexical_cast&)
    {
    }
  }

  param::param("/tcp_keepalive", TransportTCP::s_use_keepalive_, TransportTCP::s_use_keepalive_);

  PollManager::instance()->addPollThreadListener(checkForShutdown);
  XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

  initInternalTimerManager();

  TopicManager::instance()->start();
  ServiceManager::instance()->start();
  ConnectionManager::instance()->start();
  PollManager::instance()->start();
  XMLRPCManager::instance()->start();

  if (!(g_init_options & init_options::NoSigintHandler))
  {
    signal(SIGINT, basicSigintHandler);
  }

  miniros::Time::init();

  if (!(g_init_options & init_options::NoRosout))
  {
    g_rosout_appender = new ROSOutAppender;
    miniros::console::register_appender(g_rosout_appender);
  }

  if (g_shutting_down) goto end;

  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::GetLoggers>(names::resolve("~get_loggers"), getLoggers);
    ops.callback_queue = getInternalCallbackQueue().get();
    ServiceManager::instance()->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::SetLoggerLevel>(names::resolve("~set_logger_level"), setLoggerLevel);
    ops.callback_queue = getInternalCallbackQueue().get();
    ServiceManager::instance()->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  if (enable_debug)
  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::Empty>(names::resolve("~debug/close_all_connections"), closeAllConnections);
    ops.callback_queue = getInternalCallbackQueue().get();
    ServiceManager::instance()->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  {
    bool use_sim_time = false;
    param::param("/use_sim_time", use_sim_time, use_sim_time);

    if (use_sim_time)
    {
      Time::setNow(miniros::Time());
    }

    if (g_shutting_down) goto end;

    if (use_sim_time)
    {
      miniros::SubscribeOptions ops;
      ops.init<rosgraph_msgs::Clock>(names::resolve("/clock"), 1, clockCallback);
      ops.callback_queue = getInternalCallbackQueue().get();
      TopicManager::instance()->subscribe(ops);
    }
  }

  if (g_shutting_down) goto end;

  g_internal_queue_thread = boost::thread(internalCallbackQueueThreadFunc);
  getGlobalCallbackQueue()->enable();

  ROSCPP_LOG_DEBUG("Started node [%s], pid [%d], bound on [%s], xmlrpc port [%d], tcpros port [%d], using [%s] time", 
		   this_node::getName().c_str(), getpid(), network::getHost().c_str(), 
		   XMLRPCManager::instance()->getServerPort(), ConnectionManager::instance()->getTCPPort(), 
		   Time::useSystemTime() ? "real" : "sim");

  // Label used to abort if we've started shutting down in the middle of start(), which can happen in
  // threaded code or if Ctrl-C is pressed while we're initializing
end:
  // If we received a shutdown request while initializing, wait until we've shutdown to continue
  if (g_shutting_down)
  {
    std::scoped_lock<std::recursive_mutex> lock(g_shutting_down_mutex);
  }
}

void check_ipv6_environment() {
  char* env_ipv6 = NULL;
#ifdef _MSC_VER
  _dupenv_s(&env_ipv6, NULL, "ROS_IPV6");
#else
  env_ipv6 = getenv("ROS_IPV6");
#endif

  bool use_ipv6 = (env_ipv6 && strcmp(env_ipv6,"on") == 0);
  TransportTCP::s_use_ipv6_ = use_ipv6;
  XmlRpc::XmlRpcSocket::s_use_ipv6_ = use_ipv6;

#ifdef _MSC_VER
  if (env_ipv6)
  {
    free(env_ipv6);
  }
#endif
}

void init(const M_string& remappings, const std::string& name, uint32_t options)
{
  if (!g_atexit_registered)
  {
    g_atexit_registered = true;
    atexit(atexitCallback);
  }

  if (!g_global_queue)
  {
    g_global_queue.reset(new CallbackQueue);
  }

  if (!g_initialized)
  {
    g_init_options = options;
    g_ok = true;

    ROSCONSOLE_AUTOINIT;
    // Disable SIGPIPE
#ifndef WIN32
    signal(SIGPIPE, SIG_IGN);
#else
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 0), &wsaData);
#endif
    check_ipv6_environment();
    network::init(remappings);
    master::init(remappings);
    // names:: namespace is initialized by this_node
    this_node::init(name, remappings, options);
    file_log::init(remappings);
    param::init(remappings);

    g_initialized = true;
  }
}

void init(int& argc, char** argv, const std::string& name, uint32_t options)
{
  M_string remappings;

  int full_argc = argc;
  // now, move the remapping argv's to the end, and decrement argc as needed
  for (int i = 0; i < argc; )
  {
    std::string arg = argv[i];
    size_t pos = arg.find(":=");
    if (pos != std::string::npos)
    {
      std::string local_name = arg.substr(0, pos);
      std::string external_name = arg.substr(pos + 2);

      ROSCPP_LOG_DEBUG("remap: %s => %s", local_name.c_str(), external_name.c_str());
      remappings[local_name] = external_name;

      // shuffle everybody down and stuff this guy at the end of argv
      char *tmp = argv[i];
      for (int j = i; j < full_argc - 1; j++)
        argv[j] = argv[j+1];
      argv[argc-1] = tmp;
      argc--;
    }
    else
    {
      i++; // move on, since we didn't shuffle anybody here to replace it
    }
  }

  init(remappings, name, options);
}

void init(const VP_string& remappings, const std::string& name, uint32_t options)
{
  M_string remappings_map;
  VP_string::const_iterator it = remappings.begin();
  VP_string::const_iterator end = remappings.end();
  for (; it != end; ++it)
  {
    remappings_map[it->first] = it->second;
  }

  init(remappings_map, name, options);
}

std::string getROSArg(int argc, const char* const* argv, const std::string& arg)
{
  for (int i = 0; i < argc; ++i)
  {
    std::string str_arg = argv[i];
    size_t pos = str_arg.find(":=");
    if (str_arg.substr(0,pos) == arg)
    {
      return str_arg.substr(pos+2);
    }
  }
  return "";
}

void removeROSArgs(int argc, const char* const* argv, V_string& args_out)
{
  for (int i = 0; i < argc; ++i)
  {
    std::string arg = argv[i];
    size_t pos = arg.find(":=");
    if (pos == std::string::npos)
    {
      args_out.push_back(arg);
    }
  }
}

void spin()
{
  SingleThreadedSpinner s;
  spin(s);
}

void spin(Spinner& s)
{
  s.spin();
}

void spinOnce()
{
  g_global_queue->callAvailable(miniros::WallDuration());
}

void waitForShutdown()
{
  while (ok())
  {
    WallDuration(0.05).sleep();
  }
}

CallbackQueue* getGlobalCallbackQueue()
{
  return g_global_queue.get();
}

bool ok()
{
  return g_ok;
}

void shutdown()
{
  std::scoped_lock<std::recursive_mutex> lock(g_shutting_down_mutex);
  if (g_shutting_down)
    return;
  else
    g_shutting_down = true;

  miniros::console::shutdown();

  g_global_queue->disable();
  g_global_queue->clear();

  if (g_internal_queue_thread.get_id() != boost::this_thread::get_id())
  {
    g_internal_queue_thread.join();
  }
  //miniros::console::deregister_appender(g_rosout_appender);
  delete g_rosout_appender;
  g_rosout_appender = 0;

  if (g_started)
  {
    TopicManager::instance()->shutdown();
    ServiceManager::instance()->shutdown();
    PollManager::instance()->shutdown();
    ConnectionManager::instance()->shutdown();
    XMLRPCManager::instance()->shutdown();
  }

  g_started = false;
  g_ok = false;
  Time::shutdown();
}

}
