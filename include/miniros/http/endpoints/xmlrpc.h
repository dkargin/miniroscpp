//
// Created by dkargin on 7/22/25.
//

#ifndef MINIROS_XMLRPC_HANDLER_H
#define MINIROS_XMLRPC_HANDLER_H

#include "../http_endpoint.h"
#include "../http_tools.h"

#include "miniros/network/net_address.h"

namespace XmlRpc {
class XmlRpcMethods;
class XmlRpcValue;
}

namespace miniros {
namespace http {

/// Handler for XMLRPC endpoints.
class XmlRpcHandler : public EndpointHandler {
public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using ClientInfo = network::ClientInfo;

  explicit XmlRpcHandler(XmlRpc::XmlRpcMethods* server)
    :server_(server)
  {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request) override;

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
