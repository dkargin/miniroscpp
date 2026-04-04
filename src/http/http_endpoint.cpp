//
// Created for EndpointHandler default implementations.
//

#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_server_connection.h"

namespace miniros {
namespace http {

Error EndpointHandler::upgradeComplete(const std::shared_ptr<network::NetSocket>& connection,
  const network::ClientInfo& clientInfo, const std::shared_ptr<HttpRequest>& request)
{
  (void)connection;
  (void)clientInfo;
  (void)request;
  return Error::NotImplemented;
}

} // namespace http
} // namespace miniros
