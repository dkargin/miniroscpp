//
// Created by dkargin on 8/2/25.
//

#ifndef MINIROS_HTTP_ENDPOINT_H
#define MINIROS_HTTP_ENDPOINT_H

#include <memory>

#include "miniros/errors.h"

namespace miniros {

namespace network {
  struct ClientInfo;
}
namespace http {

struct HttpParserFrame;
struct HttpResponseHeader;

class HttpRequest;

/// Base class for endpoint filters.
class EndpointFilter {
public:
  virtual ~EndpointFilter() {}
  virtual bool check(const HttpParserFrame& frame) const = 0;
};

class EndpointHandler {
public:
  virtual ~EndpointHandler() {}

  /// Handle request and return pointer to response object.
  /// Response will be immediately serialized to output buffer in the same thread.
  /// @param clientInfo - connection information.
  /// @param request - contains all request information. Response body should also be stored there.
  /// @returns:
  ///   Error::Ok - request is immediately handled and response can be sent back.
  ///   Error::Postponed - request will be processed later.
  virtual Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request) = 0;
};

}
}
#endif //MINIROS_HTTP_ENDPOINT_H
