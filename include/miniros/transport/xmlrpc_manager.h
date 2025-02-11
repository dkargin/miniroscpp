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

#ifndef ROSCPP_XMLRPC_MANAGER_H
#define ROSCPP_XMLRPC_MANAGER_H

#include <string>
#include <set>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>

#include "common.h"
// TODO: Move it to impl section
#include "miniros/xmlrpcpp/XmlRpc.h"

#include <miniros/rostime.h>
#include <xmlrpcpp/XmlRpcServerConnection.h>

namespace miniros
{

/**
 * \brief internal
 */
namespace xmlrpc
{
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
typedef std::set<ASyncXMLRPCConnectionPtr> S_ASyncXMLRPCConnection;

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

class XMLRPCManager;
typedef std::shared_ptr<XMLRPCManager> XMLRPCManagerPtr;

// Compact RPC callback function.
typedef std::function<void(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)> XMLRPCFunc;

// Extended RPC callback function.
typedef std::function<int (const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result, XmlRpc::XmlRpcServerConnection* conn)> XMLRPCFuncEx;


class MINIROS_DECL XMLRPCManager
{
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  static const XMLRPCManagerPtr& instance();

  XMLRPCManager(int port = 0);
  ~XMLRPCManager();

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
  bool validateXmlrpcResponse(const std::string& method, 
			      XmlRpc::XmlRpcValue &response, XmlRpc::XmlRpcValue &payload);

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
  bool bindEx(const std::string& function_name, const XMLRPCFuncEx& cb);

  void unbind(const std::string& function_name);

  NODISCARD bool start();
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
  typedef std::vector<CachedXmlRpcClient> V_CachedXmlRpcClient;
  V_CachedXmlRpcClient clients_;
  std::mutex clients_mutex_;

  std::atomic_bool shutting_down_;

  miniros::WallDuration master_retry_timeout_;

  S_ASyncXMLRPCConnection added_connections_;
  std::mutex added_connections_mutex_;
  S_ASyncXMLRPCConnection removed_connections_;
  std::mutex removed_connections_mutex_;

  S_ASyncXMLRPCConnection connections_;


  struct FunctionInfo
  {
    std::string name;
    // Regular callback.
    XMLRPCFunc function;
    // Extended callback.
    XMLRPCFuncEx functionEx;
    std::shared_ptr<XmlRpc::XmlRpcServerMethod> wrapper;
  };
  typedef std::map<std::string, FunctionInfo> M_StringToFuncInfo;
  std::mutex functions_mutex_;
  M_StringToFuncInfo functions_;

  std::atomic_bool unbind_requested_;
};

}

#endif
