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
#include <set>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include "miniros/common.h"
// TODO: Move it to impl section
#include "miniros/xmlrpcpp/XmlRpc.h"

#include <miniros/rostime.h>

namespace miniros
{

/**
 * \brief internal
 */
namespace xmlrpc
{

class XmlRpcServerConnection;

MINIROS_DECL XmlRpc::XmlRpcValue responseStr(int code, const std::string& msg, const std::string& response);
MINIROS_DECL XmlRpc::XmlRpcValue responseInt(int code, const std::string& msg, int response);
MINIROS_DECL XmlRpc::XmlRpcValue responseBool(int code, const std::string& msg, bool response);
}

class XMLRPCCallWrapper;
typedef std::shared_ptr<XMLRPCCallWrapper> XMLRPCCallWrapperPtr;

class MINIROS_DECL ASyncXMLRPCConnection : public std::enable_shared_from_this<ASyncXMLRPCConnection>
{
public:
  virtual ~ASyncXMLRPCConnection() {}

  virtual void addToDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;
  virtual void removeFromDispatch(XmlRpc::XmlRpcDispatch* disp) = 0;

  virtual bool check() = 0;
};
typedef std::shared_ptr<ASyncXMLRPCConnection> ASyncXMLRPCConnectionPtr;

class MINIROS_DECL CachedXmlRpcClient
{
public:
  CachedXmlRpcClient(XmlRpc::XmlRpcClient *c)
  : in_use_(false)
  , client_(c)
  {
  }

  bool in_use_;
  miniros::SteadyTime last_use_time_; // for reaping
  XmlRpc::XmlRpcClient* client_;

  static const miniros::WallDuration s_zombie_time_; // how long before it is toasted
};

class RPCManager;
typedef std::shared_ptr<RPCManager> RPCManagerPtr;

// Compact RPC callback function.
typedef std::function<void(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)> XMLRPCFunc;

// Extended RPC callback function.
typedef std::function<int (const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result, XmlRpc::XmlRpcServerConnection* conn)> XMLRPCFuncEx;


class MINIROS_DECL RPCManager
{
public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using RpcConnection = XmlRpc::XmlRpcServerConnection;

  static const RPCManagerPtr& instance();

  RPCManager();
  ~RPCManager();

  /** @brief Validate an XML/RPC response
   *
   * @param method The RPC method that was invoked.
   * @param response The resonse that was received.
   * @param payload The payload that was received.
   *
   * @return true if validation succeeds, false otherwise.
   *
   * @todo Consider making this private.
   */
  bool validateXmlrpcResponse(const std::string& method, RpcValue &response, RpcValue &payload);

  /**
   * @brief Get the xmlrpc server URI of this node
   */
  const std::string& getServerURI() const { return uri_; }
  uint32_t getServerPort() const { return port_; }

  XmlRpc::XmlRpcClient* getXMLRPCClient(const std::string& host, const int port, const std::string& uri);
  void releaseXMLRPCClient(XmlRpc::XmlRpcClient* c);

  void addASyncConnection(const ASyncXMLRPCConnectionPtr& conn);
  void removeASyncConnection(const ASyncXMLRPCConnectionPtr& conn);

  /// Bind regular callback method.
  bool bind(const std::string& function_name, const XMLRPCFunc& cb);

  /// Bind extended callback.
  /// @param function_name - name of function to bind
  /// @param cb - callback functor
  /// @param object - attached object
  bool bindEx(const std::string& function_name, const XMLRPCFuncEx& cb, void* object = nullptr);

  template <class Object>
  bool bindEx0(const std::string& function_name,
    Object* object, RpcValue (Object::*method)(RpcConnection* conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, RpcConnection* conn) {
      try {
        result = (object->*method)(conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request: %s", ex.getMessage().c_str());
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
    Object* object, RpcValue (Object::*method)(const T0& arg0, RpcConnection* conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, RpcConnection* conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        result = (object->*method)(arg0, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request: %s", ex.getMessage().c_str());
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
  bool bindEx2(const std::string& function_name,Object* object, RpcValue (Object::*method)(const T0& arg0, const T1& arg1, RpcConnection* conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, RpcConnection* conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        result = (object->*method)(arg0, arg1, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request: %s", ex.getMessage().c_str());
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
  bool bindEx3(const std::string& function_name,Object* object,
    RpcValue (Object::*method)(const T0& arg0, const T1& arg1, const T2& arg2, RpcConnection* conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, RpcConnection* conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        result = (object->*method)(arg0, arg1, arg2, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request: %s", ex.getMessage().c_str());
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
    RpcValue (Object::*method)(const T0& arg0, const T1& arg1, const T2& arg2, const T3& arg3, RpcConnection* conn))
  {
    return bindEx(function_name, [=](const RpcValue& param, RpcValue& result, RpcConnection* conn) {
      try {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        T3 arg3 = param[3].as<T3>();
        result = (object->*method)(arg0, arg1, arg2, arg3, conn);
      } catch (XmlRpc::XmlRpcException ex) {
        MINIROS_WARN("RPCManager: got exception while handling request: %s", ex.getMessage().c_str());
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

  NODISCARD bool start(int port = 0);
  void shutdown();

  bool isShuttingDown() const;

private:
  void serverThreadFunc();

  std::string uri_;
  int port_;
  std::thread server_thread_;

#if defined(__APPLE__)
  // OSX has problems with lots of concurrent xmlrpc calls
  std::mutex xmlrpc_call_mutex_;
#endif
  XmlRpc::XmlRpcServer server_;
  std::vector<CachedXmlRpcClient> clients_;
  std::mutex clients_mutex_;

  std::atomic_bool shutting_down_;

  miniros::WallDuration master_retry_timeout_;

  std::set<ASyncXMLRPCConnectionPtr> added_connections_;
  std::mutex added_connections_mutex_;
  std::set<ASyncXMLRPCConnectionPtr> removed_connections_;
  std::mutex removed_connections_mutex_;

  std::set<ASyncXMLRPCConnectionPtr> connections_;


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

  std::mutex functions_mutex_;
  std::map<std::string, FunctionInfo> functions_;

  std::atomic_bool unbind_requested_;
};

} // namespace miniros

#endif // MINIROS_XMLRPC_MANAGER_H
