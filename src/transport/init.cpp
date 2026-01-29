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

#define MINIROS_PACKAGE_NAME "init"

#include "miniros/init.h"
#include "../../include/miniros/network/network.h"
#include "miniros/file_log.h"
#include "miniros/master_link.h"
#include "miniros/names.h"
#include "miniros/this_node.h"
#include "miniros/transport/callback_queue.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/internal_timer_manager.h"
#include "miniros/transport/poll_manager.h"
#include "miniros/transport/rosout_appender.h"
#include "miniros/transport/rpc_manager.h"
#include "miniros/transport/service_manager.h"
#include "miniros/transport/subscribe_options.h"
#include "miniros/transport/topic_manager.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/xmlrpcpp/XmlRpcSocket.h"

// Standard ROS services.
#include "roscpp/GetLoggers.hxx"
#include "roscpp/SetLoggerLevel.hxx"
#include "roscpp/Empty.hxx"

#include <miniros/console.h>
#include <miniros/rostime.h>
#include <rosgraph_msgs/Clock.hxx>

#include <atomic>
#include <algorithm>

#include <signal.h>

#include <cstdlib>

#include <mutex>

namespace miniros
{

namespace this_node
{
void init(const std::string& names, const M_string& remappings, uint32_t options);
}

namespace network
{
void init(const M_string& remappings);
}

namespace file_log
{
void init(const M_string& remappings);
}

CallbackQueuePtr g_global_queue;
std::atomic<ROSOutAppender*> g_rosout_appender = nullptr;
static CallbackQueuePtr g_internal_callback_queue;

static bool g_initialized = false;
static bool g_started = false;
static bool g_atexit_registered = false;
static std::mutex g_start_mutex;
static bool g_ok = false;
static uint32_t g_init_options = 0;
static std::atomic_bool g_shutdown_requested = false;
static std::atomic_bool g_shutting_down = false;
static std::recursive_mutex g_shutting_down_mutex;
static std::thread g_internal_queue_thread;
static MasterLinkPtr g_master_link;

void shutdownLocked(std::unique_lock<std::recursive_mutex>& lock);

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

    std::unique_lock<std::recursive_mutex> lock(g_shutting_down_mutex, std::defer_lock);

    while (!lock.try_lock() && !g_shutting_down)
    {
      miniros::WallDuration(0.001).sleep();
    }

    if (!g_shutting_down)
    {
      shutdownLocked(lock);
      MINIROS_INFO("Shutdown procedure is complete");
    } else {
      MINIROS_WARN("Shutdown procedure was missed");
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
    MINIROS_DEBUG("shutting down due to exit() or end of main() without cleanup of all NodeHandles");
    g_started = false; // don't shutdown singletons, because they are already destroyed
    shutdown();
  }
}

void shutdownCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    MINIROS_WARN("XMLRPC shutdown request received, reason given: [%s]", reason.c_str());
    requestShutdown();
  }

  result = xmlrpc::responseInt(1, "", 0);
}

bool getLoggers(roscpp::GetLoggers::Request&, roscpp::GetLoggers::Response& resp)
{
  std::map<std::string, miniros::console::Level> loggers;
  bool success = ::miniros::console::get_loggers(loggers);
  if (success)
  {
    for (std::map<std::string, miniros::console::Level>::const_iterator it = loggers.begin(); it != loggers.end(); it++)
    {
      roscpp::Logger logger;
      logger.name = it->first;
      miniros::console::Level level = it->second;
      if (level == miniros::console::Level::Debug)
      {
        logger.level = "debug";
      }
      else if (level == miniros::console::Level::Info)
      {
        logger.level = "info";
      }
      else if (level == miniros::console::Level::Warn)
      {
        logger.level = "warn";
      }
      else if (level == miniros::console::Level::Error)
      {
        logger.level = "error";
      }
      else if (level == miniros::console::Level::Fatal)
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

  miniros::console::Level level;
  if (req.level == "DEBUG")
  {
    level = miniros::console::Level::Debug;
  }
  else if (req.level == "INFO")
  {
    level = miniros::console::Level::Info;
  }
  else if (req.level == "WARN")
  {
    level = miniros::console::Level::Warn;
  }
  else if (req.level == "ERROR")
  {
    level = miniros::console::Level::Error;
  }
  else if (req.level == "FATAL")
  {
    level = miniros::console::Level::Fatal;
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
  MINIROS_DEBUG("close_all_connections service called, closing connections");
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

  MINIROS_DEBUG("Got SIGINT. Initiating shutdown");
  miniros::requestShutdown();
}

void internalCallbackQueueThreadFunc(CallbackQueuePtr queue)
{
  setThreadName("ROS::internalCallbackQueue");
  disableAllSignalsInThisThread();

  while (!g_shutting_down)
  {
    queue->callAvailable(WallDuration(0.1));
  }
}

bool isStarted()
{
  return g_started;
}

/// Start is issued by the first NodeHandle::construct.
/// This function is expected to be called after miniros::init.
Error start()
{
  SteadyTime time_start = SteadyTime::now();

  std::scoped_lock<std::mutex> lock(g_start_mutex);
  if (g_started)
  {
    return Error::Ok;
  }

  g_shutdown_requested = false;
  g_shutting_down = false;
  g_started = true;
  g_ok = true;

  bool enable_debug = false;
  std::string enable_debug_env;
  if ( get_environment_variable(enable_debug_env,"ROSCPP_ENABLE_DEBUG") )
  {
    // Former boost::lexical_cast treated {1.0, 1, +1, ...} values as true and zeroes as false.
    // I really do not see a reason to be that complicated. So "0" is false, "1" is true, all other values are ignored.
    int rawValue = std::stoi(enable_debug_env);
    if (rawValue == 1)
        enable_debug = true;
    else if (rawValue == 0)
        enable_debug = false;
  }

  if (g_master_link)
    g_master_link->param("/tcp_keepalive", TransportTCP::s_use_keepalive_, TransportTCP::s_use_keepalive_);

  PollManagerPtr pm = PollManager::instance();

  XMLRPCManagerPtr rpcm = RPCManager::instance();
  rpcm->bind("shutdown", shutdownCallback);

  initInternalTimerManager();

  ConnectionManagerPtr connectionManager = ConnectionManager::instance();

  TopicManagerPtr topicManager = TopicManager::instance();
  ServiceManagerPtr serviceManager = ServiceManager::instance();
  if (auto err = topicManager->start(pm, g_master_link, connectionManager, rpcm); !err) {
    MINIROS_ERROR("Failed to start TopicManager: %s", err.toString());
    shutdown();
    return Error::SystemError;
  }
  serviceManager->start(pm, g_master_link, connectionManager, rpcm);

  if (!connectionManager->start(pm)) {
    MINIROS_ERROR("Failed to start ConnectionManager. Something is very wrong with TCP network");
    shutdown();
    return Error::SystemError;
  }

  pm->start();

  PollSet& pollSet = pm->getPollSet();
  int rpcPort = network::getRPCPort();
  if (Error err = rpcm->start(&pollSet, rpcPort); !err) {
    // We can arrive here only if we are completely unable to host TCP/http server.
    MINIROS_ERROR("Failed to start RPCManager at port %d: %s", rpcPort, err.toString());
    shutdown();
    return Error::SystemError;
  }

  if (!(g_init_options & init_options::NoSigintHandler))
  {
    signal(SIGINT, basicSigintHandler);
  }

  miniros::Time::init();

  if (!(g_init_options & init_options::NoRosout))
  {
    ROSOutAppender* appender = new ROSOutAppender(topicManager);
    if (!appender->init()) {}
    auto prev = g_rosout_appender.exchange(appender);
    assert(!prev);
    miniros::console::register_appender(g_rosout_appender);
  }

  auto internalCallbackQueue = getInternalCallbackQueue();

  if (g_shutting_down) goto end;

  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::GetLoggers>(names::resolve("~get_loggers"), getLoggers);
    ops.callback_queue = internalCallbackQueue.get();
    serviceManager->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::SetLoggerLevel>(names::resolve("~set_logger_level"), setLoggerLevel);
    ops.callback_queue = internalCallbackQueue.get();
    serviceManager->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  if (enable_debug)
  {
    miniros::AdvertiseServiceOptions ops;
    ops.init<roscpp::Empty>(names::resolve("~debug/close_all_connections"), closeAllConnections);
    ops.callback_queue = internalCallbackQueue.get();
    serviceManager->advertiseService(ops);
  }

  if (g_shutting_down) goto end;

  {
    bool use_sim_time = false;
    if (!(g_init_options & init_options::NoSimTime))
    {
      g_master_link->param("/use_sim_time", use_sim_time, use_sim_time);
    }

    if (use_sim_time)
    {
      Time::setNow(miniros::Time());
    }

    if (g_shutting_down) goto end;

    if (use_sim_time)
    {
      miniros::SubscribeOptions ops;
      ops.init<rosgraph_msgs::Clock>(names::resolve("/clock"), 1, clockCallback);
      ops.callback_queue = internalCallbackQueue.get();
      topicManager->subscribe(ops);
    }
  }

  if (g_shutting_down) goto end;

  g_internal_queue_thread = std::thread(internalCallbackQueueThreadFunc, internalCallbackQueue);
  getGlobalCallbackQueue()->enable();

  MINIROS_DEBUG("Started node [%s], pid [%d], bound on [%s], xmlrpc port [%d], tcpros port [%d], using [%s] time",
           this_node::getName().c_str(), getpid(), network::getHost().c_str(),
           rpcm->getServerPort(), connectionManager->getTCPPort(),
           Time::useSystemTime() ? "real" : "sim");

  // Label used to abort if we've started shutting down in the middle of start(), which can happen in
  // threaded code or if Ctrl-C is pressed while we're initializing
end:
  // If we received a shutdown request while initializing, wait until we've shutdown to continue
  if (g_shutting_down)
  {
    std::scoped_lock<std::recursive_mutex> lock(g_shutting_down_mutex);
  }

  auto start_total = SteadyTime::now() - time_start;
  MINIROS_INFO("root ::start() done in %fms", start_total.toSec()*1000);
  return Error::Ok;
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

void initDefaultLogLevels(const M_string& remappings)
{
  // Setting up default log levels to some practical values.
  auto it = remappings.find("__miniros.debug");
  if (it != remappings.end() && it->second == "1") {
    console::set_logger_level("miniros.http", console::Level::Debug);
    console::set_logger_level("miniros.net", console::Level::Debug);
    console::set_logger_level("miniros.http.client", console::Level::Debug);
    console::set_logger_level("miniros.poll_set", miniros::console::Level::Debug);
  } else {
    console::set_logger_level("miniros.http", console::Level::Error);
    console::set_logger_level("miniros.http.client", console::Level::Error);
    console::set_logger_level("miniros.net", console::Level::Fatal);
  }
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

    console::initializeSafe();
    // Disable SIGPIPE
#ifndef WIN32
    signal(SIGPIPE, SIG_IGN);
#else
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 0), &wsaData);
#endif
    check_ipv6_environment();
    network::init(remappings);
    auto rpcManager = RPCManager::instance();
    g_master_link.reset(new MasterLink());
    bool local = options & init_options::LocalMaster;
    g_master_link->initLink(remappings, rpcManager, local);

    // names:: namespace is initialized by this_node
    this_node::init(name, remappings, options);
    file_log::init(remappings);
    g_master_link->initParam(remappings);

    g_initialized = true;
  }

  initDefaultLogLevels(remappings);
}

M_string extractRemappings(int& argc, char** argv, const std::string& name, uint32_t options)
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

      MINIROS_DEBUG("remap: %s => %s", local_name.c_str(), external_name.c_str());
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
  return remappings;
}

void init(int& argc, char** argv, const std::string& name, uint32_t options)
{
  M_string remappings = extractRemappings(argc, argv, name, options);
  init(remappings, name, options);
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
    (void)WallDuration(0.05).sleep();
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

void shutdownLocked(std::unique_lock<std::recursive_mutex>& lock)
{
  MINIROS_DEBUG("Running shutdown procedure");
  g_shutting_down = true;

  miniros::console::shutdown();

  if (g_global_queue) {
    g_global_queue->disable();
    g_global_queue->clear();
  }

  if (g_internal_queue_thread.get_id() != std::this_thread::get_id())
  {
    if (g_internal_queue_thread.joinable())
      g_internal_queue_thread.join();
  }

  //miniros::console::deregister_appender(g_rosout_appender);
  if (auto rosout_appender = g_rosout_appender.exchange(nullptr)) {
    delete rosout_appender;
  }

  if (g_started)
  {
    TopicManager::instance()->shutdown();
    ServiceManager::instance()->shutdown();
    PollManager::instance()->shutdown();
    ConnectionManager::instance()->shutdown();
    RPCManager::instance()->shutdown();
  }

  g_started = false;
  g_ok = false;
  Time::shutdown();
}

void shutdown()
{
  std::unique_lock<std::recursive_mutex> lock(g_shutting_down_mutex);
  if (g_shutting_down) {
    return;
  }
  shutdownLocked(lock);
}

MasterLinkPtr getMasterLink()
{
  return g_master_link;
}

}
