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

#define MINIROS_PACKAGE_NAME "xmlrpc"

#include <sstream>

#include "miniros/master_link.h"
#include "miniros/rosassert.h"
#include "miniros/file_log.h"
#include "miniros/transport/io.h"
#include "miniros/transport/network.h"
#include "miniros/transport/rpc_manager.h"

#include "miniros/xmlrpcpp/XmlRpcServerConnection.h"

using namespace XmlRpc;

namespace miniros
{

namespace xmlrpc
{

XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response)
{
  XmlRpc::XmlRpcValue v;
  v[0] = int(code);
  v[1] = msg;
  v[2] = response;
  return v;
}

}

/// Wrapper for regular callback functor.
class XMLRPCCallWrapper : public XmlRpcServerMethod
{
public:
  XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcServer *s)
  : XmlRpcServerMethod(function_name, s)
  , m_func(cb)
  { }

  void execute(const XmlRpcValue &params, XmlRpcValue &result, XmlRpcServerConnection* connection) override
  {
    if (m_func)
      m_func(params, result);
  }

private:
  XMLRPCFunc m_func;
};

/// Wrapper for extended callback functor.
class XMLRPCCallWrapperEx : public XmlRpcServerMethod
{
public:
  XMLRPCCallWrapperEx(const std::string& function_name, const XMLRPCFuncEx& cb, XmlRpcServer *s)
  : XmlRpcServerMethod(function_name, s)
  , m_func(cb)
  { }

  void execute(const XmlRpcValue &params, XmlRpcValue &result, XmlRpcServerConnection* connection) override
  {
    if (m_func)
      m_func(params, result, connection);
  }

private:
  XMLRPCFuncEx m_func;
};

void getPid(const XmlRpcValue& params, XmlRpcValue& result)
{
  (void)params;
  result = xmlrpc::responseInt(1, "", (int)getpid());
}

const miniros::WallDuration CachedXmlRpcClient::s_zombie_time_(30.0); // reap after 30 seconds

const RPCManagerPtr& RPCManager::instance()
{
  static RPCManagerPtr xmlrpc_manager = std::make_shared<RPCManager>();
  return xmlrpc_manager;
}

RPCManager::RPCManager()
: port_(0)
, shutting_down_(false)
, unbind_requested_(false)
{
}

RPCManager::~RPCManager()
{
  shutdown();
}

bool RPCManager::start(int port)
{
  if (server_thread_.joinable()) {
    MINIROS_INFO_NAMED("RPCManager", "Manager is running at port %d", port_);
    return true;
  }

  MINIROS_INFO_NAMED("RPCManager", "Starting manager at port %d", port);
  shutting_down_ = false;
  bind("getPid", getPid);

  if (!server_.bindAndListen(port))
    return false;

  port_ = server_.get_port();
  MINIROS_ASSERT(port_ != 0);

  std::stringstream ss;
  ss << "http://" << network::getHost() << ":" << port_ << "/";
  uri_ = ss.str();

  server_thread_ = std::thread(&RPCManager::serverThreadFunc, this);
  return true;
}

void RPCManager::shutdown()
{
  if (shutting_down_)
  {
    return;
  }

  shutting_down_ = true;
  if (server_thread_.joinable())
      server_thread_.join();

  server_.close();

  // kill the last few clients that were started in the shutdown process
  {
    std::scoped_lock<std::mutex> lock(clients_mutex_);

    for (auto i = clients_.begin(); i != clients_.end();)
    {
      if (!i->in_use_)
      {
        i->client_->close();
        delete i->client_;
        i = clients_.erase(i);
      }
      else
      {
        ++i;
      }
    }
  }

  // Wait for the clients that are in use to finish and remove themselves from clients_
  for (int wait_count = 0; !clients_.empty() && wait_count < 10; wait_count++)
  {
    MINIROS_DEBUG("waiting for xmlrpc connection to finish...");
    miniros::WallDuration(0.01).sleep();
  }

  std::scoped_lock<std::mutex> lock(functions_mutex_);
  functions_.clear();

  {
    auto it = connections_.begin();
    auto end = connections_.end();
    for (; it != end; ++it)
    {
      (*it)->removeFromDispatch(server_.get_dispatch());
    }
  }

  connections_.clear();

  {
    std::scoped_lock<std::mutex> lock(added_connections_mutex_);
    added_connections_.clear();
  }

  {
    std::scoped_lock<std::mutex> lock(removed_connections_mutex_);
    removed_connections_.clear();
  }
}

bool RPCManager::validateXmlrpcResponse(const std::string& method, XmlRpcValue &response,
                                    XmlRpcValue &payload)
{
  if (response.getType() != XmlRpcValue::TypeArray)
  {
    MINIROS_DEBUG("XML-RPC call [%s] didn't return an array",
        method.c_str());
    return false;
  }
  if (response.size() != 2 && response.size() != 3)
  {
    MINIROS_DEBUG("XML-RPC call [%s] didn't return a 2 or 3-element array",
        method.c_str());
    return false;
  }
  if (response[0].getType() != XmlRpcValue::TypeInt)
  {
    MINIROS_DEBUG("XML-RPC call [%s] didn't return a int as the 1st element",
        method.c_str());
    return false;
  }
  int status_code = response[0];
  if (response[1].getType() != XmlRpcValue::TypeString)
  {
    MINIROS_DEBUG("XML-RPC call [%s] didn't return a string as the 2nd element",
        method.c_str());
    return false;
  }
  std::string status_string = response[1];
  if (status_code != 1)
  {
    MINIROS_DEBUG("XML-RPC call [%s] returned an error (%d): [%s]",
        method.c_str(), status_code, status_string.c_str());
    return false;
  }
  if (response.size() > 2)
  {
    payload = response[2];
  }
  else
  {
    std::string empty_array = "<value><array><data></data></array></value>";
    int offset = 0;
    payload = XmlRpcValue(empty_array, &offset);
  }
  return true;
}

void RPCManager::serverThreadFunc()
{
  disableAllSignalsInThisThread();
  setThreadName("RPCManager");

  while(!shutting_down_)
  {
    {
      std::scoped_lock<std::mutex> lock(added_connections_mutex_);
      auto it = added_connections_.begin();
      auto end = added_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->addToDispatch(server_.get_dispatch());
        connections_.insert(*it);
      }

      added_connections_.clear();
    }

    // Update the XMLRPC server, blocking for at most 100ms in select()
    {
      std::scoped_lock<std::mutex> lock(functions_mutex_);
      server_.work(0.1);
    }

    while (unbind_requested_)
    {
      WallDuration(0.01).sleep();
    }

    if (shutting_down_)
    {
      return;
    }

    {
      for (auto& connection: connections_)
      {
        if (connection->check())
          removeASyncConnection(connection);
      }
    }

    {
      std::scoped_lock<std::mutex> lock(removed_connections_mutex_);
      auto it = removed_connections_.begin();
      auto end = removed_connections_.end();
      for (; it != end; ++it)
      {
        (*it)->removeFromDispatch(server_.get_dispatch());
        connections_.erase(*it);
      }

      removed_connections_.clear();
    }
  }
}

XmlRpcClient* RPCManager::getXMLRPCClient(const std::string &host, const int port, const std::string &uri)
{
  // go through our vector of clients and grab the first available one
  XmlRpcClient *c = nullptr;

  std::scoped_lock<std::mutex> lock(clients_mutex_);

  for (auto i = clients_.begin(); !c && i != clients_.end(); )
  {
    if (!i->in_use_)
    {
      // see where it's pointing
      if (i->client_->getHost() == host &&
          i->client_->getPort() == port &&
          i->client_->getUri()  == uri)
      {
        // hooray, it's pointing at our destination. re-use it.
        if (i->client_->isReady()) {
          c = i->client_;
          i->in_use_ = true;
          i->last_use_time_ = SteadyTime::now();
          break;
        }
      }
      if (i->last_use_time_ + CachedXmlRpcClient::s_zombie_time_ < SteadyTime::now())
      {
        // toast this guy. he's dead and nobody is reusing him.
        delete i->client_;
        i = clients_.erase(i);
      }
      else
      {
        ++i; // move along. this guy isn't dead yet.
      }
    }
    else
    {
      ++i;
    }
  }

  if (!c)
  {
    // allocate a new one
    c = new XmlRpcClient(host.c_str(), port, uri.c_str());
    CachedXmlRpcClient mc(c);
    mc.in_use_ = true;
    mc.last_use_time_ = SteadyTime::now();
    clients_.push_back(mc);
    //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
  }
  // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
  // by calling releaseXMLRPCClient
  return c;
}

void RPCManager::releaseXMLRPCClient(XmlRpcClient *c)
{
  std::scoped_lock<std::mutex> lock(clients_mutex_);

  for (auto i = clients_.begin(); i != clients_.end(); ++i)
  {
    if (c == i->client_)
    {
      if (shutting_down_)
      {
        // if we are shutting down we won't be re-using the client
        i->client_->close();
        delete i->client_;
        clients_.erase(i);
      }
      else
      {
        i->in_use_ = false;
      }
      break;
    }
  }
}

void RPCManager::addASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  std::scoped_lock<std::mutex> lock(added_connections_mutex_);
  added_connections_.insert(conn);
}

void RPCManager::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  std::scoped_lock<std::mutex> lock(removed_connections_mutex_);
  removed_connections_.insert(conn);
}

bool RPCManager::bind(const std::string& function_name, const XMLRPCFunc& cb)
{
  std::scoped_lock<std::mutex> lock(functions_mutex_);
  if (functions_.find(function_name) != functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.function = cb;
  info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &server_));
  functions_[function_name] = info;

  return true;
}

bool RPCManager::bindEx(const std::string& function_name, const XMLRPCFuncEx& cb, void* object)
{
  std::scoped_lock<std::mutex> lock(functions_mutex_);
  if (functions_.find(function_name) != functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.functionEx = cb;
  info.wrapper.reset(new XMLRPCCallWrapperEx(function_name, cb, &server_));
  info.object = object;
  functions_[function_name] = info;

  return true;
}

void RPCManager::unbind(const std::string& function_name)
{
  unbind_requested_ = true;
  std::scoped_lock<std::mutex> lock(functions_mutex_);
  functions_.erase(function_name);
  unbind_requested_ = false;
}

size_t RPCManager::unbind(const void* object)
{
  if (!object)
    return 0;
  unbind_requested_ = true;
  std::scoped_lock<std::mutex> lock(functions_mutex_);
  std::vector<std::string> keysToRemove;
  for (const auto& [key, info]: functions_) {
    if (info.object == object) {
      keysToRemove.push_back(key);
    }
  }

  for (const auto& key: keysToRemove) {
    functions_.erase(key);
  }

  unbind_requested_ = false;
  return keysToRemove.size();
}

bool RPCManager::isShuttingDown() const
{
  return shutting_down_;
}

} // namespace miniros
