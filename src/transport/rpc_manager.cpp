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

// It will log to "miniros.RPCManager" channel.
#define MINIROS_PACKAGE_NAME "RPCManager"

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <set>
#include <thread>

#include "miniros/network/network.h"
#include "miniros/file_log.h"
#include "miniros/master_link.h"
#include "miniros/rosassert.h"
#include "miniros/io/io.h"
#include "miniros/transport/rpc_manager.h"

#include "http/http_client.h"
#include "io/poll_set.h"

#include <miniros/rostime.h>

#include "miniros/http/http_server.h"
#include "miniros/http/endpoints/xmlrpc.h"
#include "miniros/http/http_filters.h"

#include "miniros/internal/xml_tools.h"
#include "miniros/xmlrpcpp/XmlRpcServerConnection.h"
#include "miniros/xmlrpcpp/XmlRpcServerMethod.h"
#include "network/url.h"
#include "xmlrpcpp/XmlRpcServer.h"

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
  XMLRPCCallWrapper(const std::string& function_name, const XMLRPCFunc& cb, XmlRpcMethods *s)
  : XmlRpcServerMethod(function_name, s)
  , m_func(cb)
  { }

  void execute(const XmlRpcValue &params, XmlRpcValue &result, const network::ClientInfo&) override
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
  XMLRPCCallWrapperEx(const std::string& function_name, const XMLRPCFuncEx& cb, XmlRpcMethods *s)
  : XmlRpcServerMethod(function_name, s)
  , m_func(cb)
  { }

  void execute(const XmlRpcValue &params, XmlRpcValue &result, const network::ClientInfo& clientInfo) override
  {
    if (m_func)
      m_func(params, result, clientInfo);
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

struct FunctionInfo
{
  std::string name;
  /// Regular callback.
  XMLRPCFunc function;
  /// Extended callback.
  XMLRPCFuncEx functionEx;
  /// Object to be tracked.
  void* object = nullptr;

  std::shared_ptr<XmlRpc::XmlRpcServerMethod> wrapper;
};

struct RPCManager::Internal {
  /// Own URL for RPC requests.
  network::URL url_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  std::mutex xmlrpc_call_mutex_;
#endif
  XmlRpc::XmlRpcMethods server_;

  /// Time to close http::HttpClient from Idle state.
  int clientIdleTimeoutMs = 30*1000;

  /// Assigned poll set.
  PollSet* poll_set_ = nullptr;
  std::unique_ptr<http::HttpServer> http_server_;

  std::map<network::URL, std::shared_ptr<http::HttpClient>> clients_;

  std::mutex clients_mutex_;

  std::atomic_bool shutting_down_{false};

  WallDuration master_retry_timeout_;

  std::mutex connections_mutex_;
  std::condition_variable connections_event_;
  std::set<ASyncXMLRPCConnectionPtr> removed_connections_;

  std::set<ASyncXMLRPCConnectionPtr> connections_;

  std::mutex functions_mutex_;
  std::map<std::string, FunctionInfo> functions_;

  std::atomic_bool unbind_requested_{false};
};

const RPCManagerPtr& RPCManager::instance()
{
  static RPCManagerPtr xmlrpc_manager = std::make_shared<RPCManager>();
  return xmlrpc_manager;
}

RPCManager::RPCManager()
{
  internal_ = std::make_unique<Internal>();
}

RPCManager::~RPCManager()
{
  shutdown();
}

Error RPCManager::start(int port)
{
  if (!internal_)
    return Error::InternalError;

  if (!internal_->poll_set_) {
    MINIROS_FATAL("RPCManager::start(%d) - poll set was zero", port);
    return Error::InternalError;
  }

  if (internal_->http_server_) {
    MINIROS_INFO("Manager is already running at port %d", internal_->url_.port);
    return Error::Ok;
  }

  internal_->shutting_down_ = false;
  bind("getPid", getPid);

  if (!internal_->http_server_) {
    internal_->http_server_.reset(new http::HttpServer(internal_->poll_set_));
  }

  auto xmlrpc_enpoint = std::make_shared<http::XmlRpcHandler>(&internal_->server_);

  // miniroscore endpoints tend to go here.
  internal_->http_server_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/"), xmlrpc_enpoint);

  // Common endpoint for regular ROS endpoints.
  internal_->http_server_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/RPC2"), xmlrpc_enpoint);

  if (Error err = internal_->http_server_->start(port); !err) {
    return err;
  }

  std::string host = network::getHost();
  assert(!host.empty());
  internal_->url_.port = internal_->http_server_->getPort();
  internal_->url_.host = host;
  internal_->url_.scheme = "http://";

  if (port == 0) {
    MINIROS_INFO("Started RPC manager at free port %d", internal_->url_.port);
  } else {
    MINIROS_INFO("Started RPC manager at port %d", internal_->url_.port);
  }

  MINIROS_ASSERT(internal_->url_.port != 0);

  return Error::Ok;
}

void RPCManager::shutdown()
{
  if (!internal_)
    return;

  if (internal_->shutting_down_)
    return;

  if (internal_->http_server_) {
    internal_->http_server_->stop();
  }

  internal_->shutting_down_ = true;

  // kill the last few clients that were started in the shutdown process
  {
    std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);
    for (auto& [key, client]: internal_->clients_)
    {
      client->close();
    }
    internal_->clients_.clear();
  }

  // This assumes each client will actively unregister itself. It will not happen.
#ifdef USE_DEPRECATED
  // Wait for the clients that are in use to finish and remove themselves from clients_
  for (int wait_count = 0; !internal_->clients_.empty() && wait_count < 10; wait_count++)
  {
    MINIROS_WARN("There were %d dangling xmlrpc connections. Waiting them to finish...", (int)internal_->clients_.size());
    miniros::WallDuration(0.01).sleep();
  }
#endif

  std::scoped_lock<std::mutex> lock(internal_->functions_mutex_);
  internal_->functions_.clear();

  {
    PollSet* ps = internal_->poll_set_;
    if (ps) {
      for (auto c: internal_->connections_)
      {
        c->removeFromDispatch(ps);
      }
    }
  }

  internal_->connections_.clear();

  {
    std::scoped_lock<std::mutex> lock(internal_->connections_mutex_);
    internal_->removed_connections_.clear();
  }

  internal_->http_server_.reset();
}

std::shared_ptr<http::HttpClient> RPCManager::getXMLRPCClient(const std::string &host, const int port, const std::string &path)
{
  if (!internal_ || !internal_->poll_set_)
    return nullptr;

  // Construct URL key for the map
  network::URL url;
  url.scheme = "http://";
  url.host = host;
  url.port = static_cast<uint32_t>(port);
  url.path = path;

  std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);

  // Try to find existing client
  auto it = internal_->clients_.find(url);
  if (it != internal_->clients_.end())
  {
    // Found existing client, return it
    return it->second;
  }

  // Create new client
  auto client = std::make_shared<http::HttpClient>(internal_->poll_set_);
  int reconnectTimeoutMs = 500;
  client->setDisconnectHandler(
    [wclient = std::weak_ptr(client), reconnectTimeoutMs](std::shared_ptr<network::NetSocket>& socket, http::HttpClient::State state)
    {
      auto client = wclient.lock();
      if (client && client->getReconnectAttempts() < 10 && client->getQueuedRequests() > 0) {
        client->reconnect(reconnectTimeoutMs);
      }
    });
  client->setIdleTimeoutHandler(internal_->clientIdleTimeoutMs,
    [wclient = std::weak_ptr(client), this](std::shared_ptr<network::NetSocket>& socket, http::HttpClient::State state)
    {
      auto client = wclient.lock();
      if (client) {
        releaseXMLRPCClient(client);
      }
    });
  // Start connection to host
  Error connectErr = client->connect(host, port);
  if (connectErr != Error::Ok && connectErr != Error::WouldBlock)
  {
    // Connection failed (WouldBlock is expected for non-blocking connections)
    MINIROS_WARN("RPCManager::getXMLRPCClient: Failed to initiate connection to %s:%d: %s",
                 host.c_str(), port, connectErr.toString());
    return nullptr;
  }

  // Store client in map
  internal_->clients_[url] = client;

  return client;
}

void RPCManager::releaseXMLRPCClient(std::shared_ptr<http::HttpClient> c)
{
  if (!internal_)
    return;
  // TODO: Put client on timeout.
  // In case of "idle" state it should disconnect itself and remove it from RPC manager.
  std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);
  auto rootUrl = c->getRootURL("http://");
  auto it = internal_->clients_.find(rootUrl);

  if (it == internal_->clients_.end()) {
    // Brute force search for this client.
    for (it = internal_->clients_.begin(); it != internal_->clients_.end(); it++) {
      if (it->second == c) {
        break;
      }
    }
  }

  if (it != internal_->clients_.end()) {
    MINIROS_INFO("RPCManager::releaseXMLRPCClient dropping stale client to %s fd=%d", rootUrl.str().c_str(), c->fd());
    internal_->clients_.erase(it);
  }
}

bool RPCManager::bind(const std::string& function_name, const XMLRPCFunc& cb)
{
  if (!internal_)
    return false;
  std::scoped_lock<std::mutex> lock(internal_->functions_mutex_);
  if (internal_->functions_.find(function_name) != internal_->functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.function = cb;
  info.wrapper.reset(new XMLRPCCallWrapper(function_name, cb, &internal_->server_));
  internal_->functions_[function_name] = info;

  return true;
}

bool RPCManager::bindEx(const std::string& function_name, const XMLRPCFuncEx& cb, void* object)
{
  if (!internal_)
    return false;
  std::scoped_lock<std::mutex> lock(internal_->functions_mutex_);
  if (internal_->functions_.find(function_name) != internal_->functions_.end())
  {
    return false;
  }

  FunctionInfo info;
  info.name = function_name;
  info.functionEx = cb;
  info.wrapper.reset(new XMLRPCCallWrapperEx(function_name, cb, &internal_->server_));
  info.object = object;
  internal_->functions_[function_name] = info;

  return true;
}

void RPCManager::unbind(const std::string& function_name)
{
  if (!internal_)
    return;
  internal_->unbind_requested_ = true;
  std::scoped_lock<std::mutex> lock(internal_->functions_mutex_);
  internal_->functions_.erase(function_name);
  internal_->unbind_requested_ = false;
}

size_t RPCManager::unbind(const void* object)
{
  if (!object)
    return 0;
  if (!internal_)
    return 0;
  internal_->unbind_requested_ = true;
  std::scoped_lock<std::mutex> lock(internal_->functions_mutex_);
  std::vector<std::string> keysToRemove;
  for (const auto& [key, info]: internal_->functions_) {
    if (info.object == object) {
      keysToRemove.push_back(key);
    }
  }

  for (const auto& key: keysToRemove) {
    internal_->functions_.erase(key);
  }

  internal_->unbind_requested_ = false;
  return keysToRemove.size();
}

bool RPCManager::isShuttingDown() const
{
  if (internal_)
    return internal_->shutting_down_;
  return false;
}

XmlRpcValue generateFaultResponse(std::string const& errorMsg, int errorCode=-1)
{
  XmlRpcValue faultStruct;
  faultStruct["faultCode"] = errorCode;
  faultStruct["faultString"] = errorMsg;
  return faultStruct;
}

bool RPCManager::executeLocalMethod(const std::string& methodName, const RpcValue& request, RpcValue& response)
{
  if (!internal_)
    return false;
  XmlRpcServerMethod* method = internal_->server_.findMethod(methodName);

  if ( ! method) return false;

  network::ClientInfo clientInfo;
  clientInfo.sameProcess = true;
  method->execute(request, response, clientInfo);

  // Ensure a valid result value
  if ( !response.valid())
    response = std::string();

  return true;
}

namespace {
const char SYSTEM_MULTICALL[] = "system.multicall";
const char METHODNAME[] = "methodName";
const char PARAMS[] = "params";

const char FAULTCODE[] = "faultCode";
const char FAULTSTRING[] = "faultString";
}

bool RPCManager::executeLocalMulticall(const std::string& method, const RpcValue& request, RpcValue& response)
{
  // Multicall XMLRPC requests are the ancient evil. It appeared in the age of HTTP1.0, where TCP connection was terminated
  // after each request, so sending the sequence of HTTP/XMLRPC requests was very expensive. HTTP1.1 has inherent
  // KeepAlive=true. And even then, the only multicall request is sent only when node is exiting and trying to unregister
  // all of its subscriptions, publications and services. It could be done by just creating additional "unregisterMe"
  // request.

  if (method != SYSTEM_MULTICALL)
    return false;

  // There ought to be 1 parameter, an array of structs
  if (request.size() != 1 || request[0].getType() != XmlRpcValue::TypeArray)
    throw XmlRpcException(std::string(SYSTEM_MULTICALL) + ": Invalid argument (expected an array)");

  int nc = request[0].size();
  response.setSize(nc);

  for (int i=0; i<nc; ++i) {

    if ( ! request[0][i].hasMember(METHODNAME) || ! request[0][i].hasMember(PARAMS))
    {
      response[i][FAULTCODE] = -1;
      response[i][FAULTSTRING] = std::string(SYSTEM_MULTICALL) +
              ": Invalid argument (expected a struct with members methodName and params)";
      continue;
    }

    const std::string& methodName = request[0][i][METHODNAME];
    XmlRpcValue& methodParams = request[0][i][PARAMS];

    XmlRpcValue resultValue;
    resultValue.setSize(1);
    try {
      if ( ! executeLocalMethod(methodName, methodParams, resultValue[0]) &&
           ! executeLocalMulticall(methodName, request, resultValue[0]))
      {
        response[i][FAULTCODE] = -1;
        response[i][FAULTSTRING] = methodName + ": unknown method name";
      }
      else {
        response[i] = resultValue;
      }
    } catch (const XmlRpcException& fault) {
      response[i][FAULTCODE] = fault.getCode();
      response[i][FAULTSTRING] = fault.getMessage();
    }
  }

  return true;
}

Error RPCManager::executeLocalRPC(const std::string& method, const RpcValue& request, RpcValue& response)
{
  try {
    if (!executeLocalMethod(method, request, response) && ! executeLocalMulticall(method, request, response))
      response = generateFaultResponse(method + ": unknown method name");
  } catch (const XmlRpcException& fault) {
    MINIROS_ERROR("executeLocalRPC::executeLocalRPC: fault %s.", fault.getMessage().c_str());
    response = generateFaultResponse(fault.getMessage(), fault.getCode());
  }
  return Error::Ok;
}

bool RPCManager::isLocalRPC(const std::string& host, int port) const
{
  return getServerPort() == port;
}

std::string RPCManager::getServerURI() const
{
  static std::string empty;
  return internal_ ? internal_->url_.str() : empty;
}

uint32_t RPCManager::getServerPort() const
{
  return internal_ ? internal_->url_.port : 0;
}

http::HttpServer* RPCManager::getHttpServer()
{
  return internal_ ? internal_->http_server_.get() : nullptr;
}

PollSet* RPCManager::getPollSet() const
{
  return internal_ ? internal_->poll_set_ : nullptr;
}

void RPCManager::setPollSet(PollSet* poll_set)
{
  assert(internal_);
  if (!internal_)
    return;
  internal_->poll_set_ = poll_set;
}

} // namespace miniros
