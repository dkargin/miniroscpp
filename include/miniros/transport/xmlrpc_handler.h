//
// Created by dkargin on 7/22/25.
//

#ifndef MINIROS_XMLRPC_HANDLER_H
#define MINIROS_XMLRPC_HANDLER_H

#include "http_server.h"

namespace XmlRpc {
class XmlRpcMethods;
class XmlRpcValue;
}

namespace miniros {
namespace network {

/// Handler for XMLRPC endpoints.
class XmlRpcHandler : public HttpServer::EndpointHandler {
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  explicit XmlRpcHandler(XmlRpc::XmlRpcMethods* server)
    :server_(server)
  {}

  Error handle(const HttpFrame& frame, const ClientInfo& clientInfo,
      HttpResponseHeader& responseHeader, std::string& body) override;

  // Execute a named method with the specified params.
  bool executeMethod(const ClientInfo& clientInfo, const std::string& methodName,
    RpcValue& params, RpcValue& result);

  // Execute multiple calls and return the results in an array.
  bool executeMulticall(const ClientInfo& clientInfo, const std::string& methodName, RpcValue& params, RpcValue& result);

  XmlRpc::XmlRpcMethods* server_;
};

}
}
#endif //MINIROS_XMLRPC_HANDLER_H
