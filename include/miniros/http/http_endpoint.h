//
// Created by dkargin on 8/2/25.
//

#ifndef MINIROS_HTTP_ENDPOINT_H
#define MINIROS_HTTP_ENDPOINT_H

#include <string>

#include "miniros/errors.h"

namespace miniros {

namespace network {
  struct ClientInfo;
}
namespace http {

struct HttpParserFrame;
struct HttpResponseHeader;

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
  /// @param frame - request data
  /// @param clientInfo - connection information.
  /// @param responseHeader - header of response.
  /// @param body - buffer for serialized body.
  virtual Error handle(const HttpParserFrame& frame, const network::ClientInfo& clientInfo,
    HttpResponseHeader& responseHeader, std::string& body) = 0;
};

}
}
#endif //MINIROS_HTTP_ENDPOINT_H
