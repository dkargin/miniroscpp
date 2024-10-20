/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#include "miniros/master.h"
#include "miniros/transport/xmlrpc_manager.h"
#include "miniros/this_node.h"
#include "miniros/init.h"
#include "miniros/transport/network.h"

#include <miniros/console.h>
#include <miniros/rosassert.h>

#include "miniros/xmlrpcpp/XmlRpc.h"

namespace miniros
{

namespace master
{

uint32_t g_port = 0;
std::string g_host;
std::string g_uri;
miniros::WallDuration g_retry_timeout;

void init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.find("__master");
  if (it != remappings.end())
  {
    g_uri = it->second;
  }

  if (g_uri.empty())
  {
    char *master_uri_env = NULL;
    #ifdef _MSC_VER
      _dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
    #else
      master_uri_env = getenv("ROS_MASTER_URI");
    #endif
    if (!master_uri_env)
    {
      MINIROS_FATAL( "ROS_MASTER_URI is not defined in the environment. Either " \
                 "type the following or (preferrably) add this to your " \
                 "~/.bashrc file in order set up your " \
                 "local machine as a ROS master:\n\n" \
                 "export ROS_MASTER_URI=http://localhost:11311\n\n" \
                 "then, type 'roscore' in another shell to actually launch " \
                 "the master program.");
      MINIROS_BREAK();
    }

    g_uri = master_uri_env;

#ifdef _MSC_VER
    // http://msdn.microsoft.com/en-us/library/ms175774(v=vs.80).aspx
    free(master_uri_env);
#endif
  }

  // Split URI into
  if (!network::splitURI(g_uri, g_host, g_port))
  {
    MINIROS_FATAL( "Couldn't parse the master URI [%s] into a host:port pair.", g_uri.c_str());
    MINIROS_BREAK();
  }
}

const std::string& getHost()
{
  return g_host;
}

uint32_t getPort()
{
  return g_port;
}

const std::string& getURI()
{
  return g_uri;
}

void setRetryTimeout(miniros::WallDuration timeout)
{
  if (timeout < miniros::WallDuration(0))
  {
    MINIROS_FATAL("retry timeout must not be negative.");
    MINIROS_BREAK();
  }
  g_retry_timeout = timeout;
}

bool check()
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  return execute("getPid", args, result, payload, false);
}

bool getTopics(V_TopicInfo& topics)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ""; //TODO: Fix this

  if (!execute("getPublishedTopics", args, result, payload, true))
  {
    return false;
  }

  topics.clear();
  for (int i = 0; i < payload.size(); i++)
  {
    topics.push_back(TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
  }

  return true;
}

bool getNodes(V_string& nodes)
{
  XmlRpc::XmlRpcValue args, result, payload;
  args[0] = this_node::getName();

  if (!execute("getSystemState", args, result, payload, true))
  {
    return false;
  }

  S_string node_set;
  for (int i = 0; i < payload.size(); ++i)
  {
    for (int j = 0; j < payload[i].size(); ++j)
    {
      XmlRpc::XmlRpcValue val = payload[i][j][1];
      for (int k = 0; k < val.size(); ++k)
      {
        std::string name = payload[i][j][1][k];
        node_set.insert(name);
      }
    }
  }

  nodes.insert(nodes.end(), node_set.begin(), node_set.end());

  return true;
}

#if defined(__APPLE__)
std::mutex g_xmlrpc_call_mutex;
#endif

bool execute(const std::string& method, const XmlRpc::XmlRpcValue& request, XmlRpc::XmlRpcValue& response, XmlRpc::XmlRpcValue& payload, bool wait_for_master)
{
  miniros::SteadyTime start_time = miniros::SteadyTime::now();

  std::string master_host = getHost();
  uint32_t master_port = getPort();
  XMLRPCManagerPtr manager = XMLRPCManager::instance();
  if (!manager)
      return false;
  XmlRpc::XmlRpcClient *c = manager->getXMLRPCClient(master_host, master_port, "/");
  bool printed = false;
  bool slept = false;
  bool ok = true;
  bool b = false;
  do
  {
    {
#if defined(__APPLE__)
      std::scoped_lock<std::mutex> lock(g_xmlrpc_call_mutex);
#endif

      b = c->execute(method.c_str(), request, response);
    }

    ok = !miniros::isShuttingDown() && !manager->isShuttingDown();

    if (!b && ok)
    {
      if (!printed && wait_for_master)
      {
        MINIROS_ERROR("[%s] Failed to contact master at [%s:%d].  %s", method.c_str(), master_host.c_str(), master_port, wait_for_master ? "Retrying..." : "");
        printed = true;
      }

      if (!wait_for_master)
      {
        manager->releaseXMLRPCClient(c);
        return false;
      }

      if (!g_retry_timeout.isZero() && (miniros::SteadyTime::now() - start_time) >= g_retry_timeout)
      {
        MINIROS_ERROR("[%s] Timed out trying to connect to the master after [%f] seconds", method.c_str(), g_retry_timeout.toSec());
        manager->releaseXMLRPCClient(c);
        return false;
      }

      miniros::WallDuration(0.05).sleep();
      slept = true;
    }
    else
    {
      if (!manager->validateXmlrpcResponse(method, response, payload))
      {
        manager->releaseXMLRPCClient(c);

        return false;
      }

      break;
    }

    ok = !miniros::isShuttingDown() && !manager->isShuttingDown();
  } while(ok);

  if (ok && slept)
  {
    MINIROS_INFO("Connected to master at [%s:%d]", master_host.c_str(), master_port);
  }

  manager->releaseXMLRPCClient(c);

  return b;
}

} // namespace master

} // namespace miniros
