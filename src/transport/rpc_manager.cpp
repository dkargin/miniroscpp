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
#include "miniros/transport/io.h"
#include "miniros/transport/rpc_manager.h"

#include <miniros/rostime.h>

#include "miniros/http/http_server.h"
#include "miniros/http/endpoints/xmlrpc.h"
#include "miniros/http/http_filters.h"

#include "miniros/xmlrpcpp/XmlRpcServerConnection.h"
#include "miniros/internal/xml_tools.h"

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

  std::string uri_;
  int port_ = 0;
  std::thread server_thread_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  std::mutex xmlrpc_call_mutex_;
#endif
  XmlRpc::XmlRpcMethods server_;

  /// Assigned poll set.
  PollSet* poll_set_ = nullptr;
  std::unique_ptr<http::HttpServer> http_server_;
  std::vector<CachedXmlRpcClient> clients_;
  std::mutex clients_mutex_;

  std::atomic_bool shutting_down_{false};

  WallDuration master_retry_timeout_;

  std::set<ASyncXMLRPCConnectionPtr> added_connections_;
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

Error RPCManager::start(PollSet* poll_set, int port)
{
  if (!internal_)
    return Error::InternalError;

  if (internal_->server_thread_.joinable()) {
    MINIROS_INFO("Manager is already running at port %d", internal_->port_);
    return Error::Ok;
  }

  MINIROS_INFO("Starting manager at port %d", port);
  internal_->shutting_down_ = false;
  bind("getPid", getPid);

  internal_->poll_set_ = poll_set;

  if (!internal_->http_server_) {
    internal_->http_server_.reset(new http::HttpServer(poll_set));
  }

  auto xmlrpc_enpoint = std::make_shared<http::XmlRpcHandler>(&internal_->server_);

  // miniros endpoints tend to go here.
  internal_->http_server_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/"), xmlrpc_enpoint);

  // Common endpoint for regular ROS endpoints.
  internal_->http_server_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/RPC2"), xmlrpc_enpoint);

  if (Error err = internal_->http_server_->start(port); !err) {
    return err;
  }

  internal_->port_ = internal_->http_server_->getPort();
  MINIROS_ASSERT(internal_->port_ != 0);

  std::string host = network::getHost();
  assert(!host.empty());
  std::stringstream ss;
  ss << "http://" << host << ":" << internal_->port_ << "/";
  internal_->uri_ = ss.str();

  internal_->server_thread_ = std::thread(&RPCManager::serverThreadFunc, this);
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
  if (internal_->server_thread_.joinable())
      internal_->server_thread_.join();

  // kill the last few clients that were started in the shutdown process
  {
    std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);

    for (auto i = internal_->clients_.begin(); i != internal_->clients_.end();)
    {
      if (!i->in_use_)
      {
        i->client_->close();
        delete i->client_;
        i = internal_->clients_.erase(i);
      }
      else
      {
        ++i;
      }
    }
  }

  // Wait for the clients that are in use to finish and remove themselves from clients_
  for (int wait_count = 0; !internal_->clients_.empty() && wait_count < 10; wait_count++)
  {
    MINIROS_WARN("There were %d dangling xmlrpc connections. Waiting them to finish...", (int)internal_->clients_.size());
    miniros::WallDuration(0.01).sleep();
  }

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
    internal_->added_connections_.clear();
    internal_->removed_connections_.clear();
  }

  internal_->http_server_.reset();
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
    size_t offset = 0;
    xml::XmlCodec codec;
    codec.parseXmlRpcValue(payload, empty_array, offset);
  }
  return true;
}

void RPCManager::serverThreadFunc()
{
  disableAllSignalsInThisThread();
  setThreadName("RPCManager");

  while(!internal_->shutting_down_)
  {
    {
      std::unique_lock<std::mutex> lock(internal_->connections_mutex_);
      internal_->connections_event_.wait_for(lock, std::chrono::milliseconds(100));

      PollSet* ps = internal_->poll_set_;
      for (ASyncXMLRPCConnectionPtr c: internal_->added_connections_)
      {
        c->addToDispatch(ps);
        internal_->connections_.insert(c);
      }
      internal_->added_connections_.clear();

      for (ASyncXMLRPCConnectionPtr c: internal_->removed_connections_)
      {
        c->removeFromDispatch(ps);
        internal_->connections_.erase(c);
      }

      internal_->removed_connections_.clear();
    }

    while (internal_->unbind_requested_)
    {
      WallDuration(0.01).sleep();
    }

    if (internal_->shutting_down_)
    {
      return;
    }

    // Lazy check for removed connections.
    for (auto& connection: internal_->connections_)
    {
      if (connection->check())
        removeASyncConnection(connection);
    }
  }
}

XmlRpcClient* RPCManager::getXMLRPCClient(const std::string &host, const int port, const std::string &uri)
{
  if (!internal_)
    return nullptr;
  // go through our vector of clients and grab the first available one
  XmlRpcClient *c = nullptr;

  std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);

  for (auto i = internal_->clients_.begin(); !c && i != internal_->clients_.end(); )
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
        i = internal_->clients_.erase(i);
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
    internal_->clients_.push_back(mc);
    //ROS_INFO("%d xmlrpc clients allocated\n", xmlrpc_clients.size());
  }
  // ONUS IS ON THE RECEIVER TO UNSET THE IN_USE FLAG
  // by calling releaseXMLRPCClient
  return c;
}

void RPCManager::releaseXMLRPCClient(XmlRpcClient *c)
{
  if (!internal_)
    return;
  std::scoped_lock<std::mutex> lock(internal_->clients_mutex_);

  for (auto i = internal_->clients_.begin(); i != internal_->clients_.end(); ++i)
  {
    if (c == i->client_)
    {
      if (internal_->shutting_down_)
      {
        // if we are shutting down we won't be re-using the client
        i->client_->close();
        delete i->client_;
        internal_->clients_.erase(i);
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
  if (!internal_)
    return;
  std::scoped_lock<std::mutex> lock(internal_->connections_mutex_);
  internal_->added_connections_.insert(conn);
  internal_->connections_event_.notify_all();
}

void RPCManager::removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn)
{
  if (!internal_)
    return;
  std::scoped_lock<std::mutex> lock(internal_->connections_mutex_);
  internal_->removed_connections_.insert(conn);
  internal_->connections_event_.notify_all();
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

const std::string& RPCManager::getServerURI() const
{
  static std::string empty;
  return internal_ ? internal_->uri_ : empty;
}

uint32_t RPCManager::getServerPort() const
{
  return internal_ ? internal_->port_ : 0;
}

http::HttpServer* RPCManager::getHttpServer()
{
  return internal_ ? internal_->http_server_.get() : nullptr;
}

PollSet* RPCManager::getPollSet() const
{
  return internal_ ? internal_->poll_set_ : nullptr;
}


} // namespace miniros
