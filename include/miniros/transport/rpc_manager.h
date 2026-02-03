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

#ifndef MINIROS_XMLRPC_MANAGER_H
#define MINIROS_XMLRPC_MANAGER_H

#include <string>
#include <memory>

#include "miniros/common.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "miniros/xmlrpcpp/XmlRpcException.h"

namespace XmlRpc {
class XmlRpcServerConnection;
class XmlRpcClient;
}

namespace miniros
{

namespace network {
struct ClientInfo;
}

namespace http {
class HttpServer;
class HttpClient;
}
/**
 * \brief internal
 */
namespace xmlrpc
{

MINIROS_DECL XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response);
MINIROS_DECL XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response);
MINIROS_DECL XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response);
}

class PollSet;
class XMLRPCCallWrapper;
typedef std::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

/// Asynchronous RPC connection.
/// The only subclass is Subscription::PendingConnection.
/// Processing and spinning is done by a spinner from RpcManager::server_.
class MINIROS_DECL ASyncXMLRPCConnection : public std::enable_shared_from_this<ASyncXMLRPCConnection>
{
public:
  virtual ~ASyncXMLRPCConnection() {}

  virtual void removeFromDispatch(PollSet* disp) = 0;

  /// Check if request is complete.
  virtual bool check() = 0;
};
typedef std::shared_ptr<ASyncXMLRPCConnection> ASyncXMLRPCConnectionPtr;

/// RPC Client for running simple RPC requests to other nodes or master.
/// It is reused for running several requests in sequence.
class MINIROS_DECL CachedXmlRpcClient
{
public:
  CachedXmlRpcClient(XmlRpc::XmlRpcClient *c)
  : in_use_(false)
  , client_(c)
  {
  }

  bool in_use_;
  SteadyTime last_use_time_; // for reaping
  XmlRpc::XmlRpcClient* client_;

  static const miniros::WallDuration s_zombie_time_; // how long before it is toasted
};

class RPCManager;
typedef std::shared_ptr<RPCManager> RPCManagerPtr;

// Compact RPC callback function.
typedef std::function<void(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)> XMLRPCFunc;

// Extended RPC callback function.
typedef std::function<int (const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result, const network::ClientInfo& conn)> XMLRPCFuncEx;


class MINIROS_DECL RPCManager
{
public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using ClientInfo = network::ClientInfo;

  static const RPCManagerPtr& instance();

  RPCManager();
  ~RPCManager();

  /**
   * @brief Get the xmlrpc server URI of this node
   */
  std::string getServerURI() const;
  uint32_t getServerPort() const;

  /// @param host - hostname of node
  /// @param port - XMLRPC port of node
  /// @param path - URL path. Requests to master typically go to "/RPC2". Node API goes to "/".
  std::shared_ptr<http::HttpClient> getXMLRPCClient(const std::string& host, const int port, const std::string& path);

  /// Notify that specific client is not needed.
  /// It should put this client on some timer after which it will be removed.
  void releaseXMLRPCClient(std::shared_ptr<http::HttpClient> c);

  /// Get access to HTTP server.
  /// This pointer lifetime is tied to this instance of RPCManager.
  http::HttpServer* getHttpServer();

  /// Check if RPC API is already pointing to this manager.
  bool isLocalRPC(const std::string& host, int port) const;

  /// Execute RPC request locally.
  Error executeLocalRPC(const std::string& method, const RpcValue& request, RpcValue& response);

  /// Bind regular callback method.
  bool bind(const std::string& function_name, const XMLRPCFunc& cb);

  /// Bind extended callback.
  /// @param function_name - name of function to bind
  /// @param cb - callback functor
  /// @param object - attached object
  bool bindEx(const std::string& function_name, const XMLRPCFuncEx& cb, void* object = nullptr);

  template <class Object>
  bool bindEx0(const std::string& function_name,
    Object* object, RpcValue (Object::*method)(const ClientInfo& conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, const ClientInfo& conn) {
      try {
        result = (object->*method)(conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request \"%s\": %s", function_name.c_str(), ex.getMessage().c_str());
        result = RpcValue::Array(3);
        result[0] = 0;
        result[1] = ex.getMessage();
        result[2] = ex.getCode();
        return false;
      }
      return true;
    }, object);
  }

  template <class Object, class T0>
  bool bindEx1(const std::string& function_name,
    Object* object, RpcValue (Object::*method)(const T0& arg0, const ClientInfo& conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, const ClientInfo& conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        result = (object->*method)(arg0, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request \"%s\": %s", function_name.c_str(), ex.getMessage().c_str());
        result = RpcValue::Array(3);
        result[0] = 0;
        result[1] = ex.getMessage();
        result[2] = ex.getCode();
        return false;
      }
      return true;
    }, object);
  }

  template <class Object, class T0, class T1>
  bool bindEx2(const std::string& function_name, Object* object, RpcValue (Object::*method)(const T0& arg0, const T1& arg1, const ClientInfo& conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, const ClientInfo& conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        result = (object->*method)(arg0, arg1, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request \"%s\": %s", function_name.c_str(), ex.getMessage().c_str());
        result = RpcValue::Array(3);
        result[0] = 0;
        result[1] = ex.getMessage();
        result[2] = ex.getCode();
        return false;
      }
      return true;
    }, object);
  }

  template <class Object, class T0, class T1, class T2>
  bool bindEx3(const std::string& function_name, Object* object,
    RpcValue (Object::*method)(const T0& arg0, const T1& arg1, const T2& arg2, const ClientInfo& conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, const ClientInfo& conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        result = (object->*method)(arg0, arg1, arg2, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request \"%s\": %s", function_name.c_str(), ex.getMessage().c_str());
        result = RpcValue::Array(3);
        result[0] = 0;
        result[1] = ex.getMessage();
        result[2] = ex.getCode();
        return false;
      }
      return true;
    }, object);
  }

  template <class Object, class T0, class T1, class T2, class T3>
  bool bindEx4(const std::string& function_name, Object* object,
    RpcValue (Object::*method)(const T0& arg0, const T1& arg1, const T2& arg2, const T3& arg3, const ClientInfo& conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, const ClientInfo& conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        T3 arg3 = param[3].as<T3>();
        result = (object->*method)(arg0, arg1, arg2, arg3, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request \"%s\": %s", function_name.c_str(), ex.getMessage().c_str());
        result = RpcValue::Array(3);
        result[0] = 0;
        result[1] = ex.getMessage();
        result[2] = ex.getCode();
        return false;
      }
      return true;
    }, object);
  }

  void unbind(const std::string& function_name);

  /// Unbind all callbacks, associated with specific object.
  size_t unbind(const void* object);

  NODISCARD Error start(int port = 0);
  void shutdown();

  bool isShuttingDown() const;

  /// Get assigned poll set.
  PollSet* getPollSet() const;

  /// Assign poll set.
  /// It should be done before calling 'start'.
  /// It is done with separate function to allow usage of 'getXMLRPCClient` before calling 'start'.
  void setPollSet(PollSet* poll_set);

private:

  bool executeLocalMethod(const std::string& methodName, const RpcValue& request, RpcValue& response);

  bool executeLocalMulticall(const std::string& methodName, const RpcValue& request, RpcValue& response);

  struct Internal;
  std::unique_ptr<Internal> internal_;
};

} // namespace miniros

#endif // MINIROS_XMLRPC_MANAGER_H
