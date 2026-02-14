//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_HTTP_SERVER_H
#define MINIROS_HTTP_SERVER_H

#include <memory>
#include <string>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"

namespace miniros {

class PollSet;
class CallbackQueue;
template <class T> struct Lifetime;

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
  int getPort() const;

  /// Get port for IPv4
  int getPortIp6() const;

  /// Get poll set.
  PollSet* getPollSet() const;

  /// Register endpoint.
  /// Multiple filters can be used for the same endpoint.
  /// @param filter - filter object. HTTP server will take care of removal of this object.
  /// @param handler - handler for specified endpoint.
  /// @param cb - callback queue to send requests to. Set to nullptr to execute request right inside spinner thread.
  Error registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const std::shared_ptr<CallbackQueue>& cb);

  /// Find endpoint for request.
  std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> findEndpoint(const HttpParserFrame& frame);

  /// Called when HTTP connection is closed.
  using CloseConnectionHandler = std::function<void (const std::shared_ptr<HttpServerConnection>&, const std::string& reason)>;

  void setCloseConnectionHandler(CloseConnectionHandler&& handler);

protected:
  /// Accept client and add it to event processing.
  /// For internal usage only.
  /// @param sock - listening socket.
  void acceptClient(const std::shared_ptr<Lifetime<HttpServer>>& lifetime, network::NetSocket* sock);

  /// Close connection and send all notifications.
  void closeConnection(int fd, const std::string& reason);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}
#endif //MINIROS_HTTP_SERVER_H
