//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_HTTP_SERVER_H
#define MINIROS_HTTP_SERVER_H

#include <memory>
#include <string>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"

#include <regex>

namespace miniros {

class PollSet;

namespace network {
class NetSocket;
struct ClientInfo;
}

namespace http {

class HttpServerConnection;

/// HttpServer serves XMLRPC MasterApi, SlaveApi and additional HTTP endpoints from user.
class HttpServer {
public:
  HttpServer(PollSet* pollSet);
  ~HttpServer();

  /// Start server on specific port.
  Error start(int port);

  /// Stop all sockets.
  Error stop();

  /// Get port for IPv4
  int getPortIp4() const;

  /// Get port for IPv4
  int getPortIp6() const;

  /// Get poll set.
  PollSet* getPollSet() const;

  /// Register endpoint.
  /// Multiple filters can be used for the same endpoint.
  /// @param filter - filter object. HTTP server will take care of removal of this object.
  /// @param handler - handler for specified endpoint.
  Error registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler);

  /// Find endpoint for request.
  EndpointHandler* findEndpoint(const HttpFrame& frame);

protected:
  /// Accept client and add it to event processing.
  /// For internal usage only.
  /// @param sock - listening socket.
  void acceptClient(network::NetSocket* sock);

  /// Closes and unregisters a connection.
  /// For internal usage only.
  void closeConnection(int fd, const char* reason);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}
#endif //MINIROS_HTTP_SERVER_H
