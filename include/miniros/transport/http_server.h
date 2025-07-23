//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_HTTP_SERVER_H
#define MINIROS_HTTP_SERVER_H

#include <string>

#include "miniros/transport/socket.h"
#include "miniros/transport/http_tools.h"
#include "poll_set.h"

namespace miniros {
namespace network {

class HttpServerConnection;

/// HttpServer serves XMLRPC MasterApi, SlaveApi and additional HTTP endpoints from user.
class HttpServer {
public:
  HttpServer(PollSet* pollSet);

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

  class EndpointHandler {
  public:
    virtual ~EndpointHandler() {}

    /// Handle request and return pointer to response object.
    /// Response will be immediately serialized to output buffer in the same thread.
    /// @param frame - request data
    /// @param clientInfo - connection information.
    /// @param responseHeader - header of response.
    /// @param body - buffer for serialized body.
    virtual Error handle(const HttpFrame& frame, const ClientInfo& clientInfo,
      HttpResponseHeader& responseHeader, std::string& body) = 0;
  };

  Error registerEndpoint(HttpMethod method, const std::string& path, const std::shared_ptr<EndpointHandler>& handler);

  /// Find endpoint for request.
  EndpointHandler* findEndpoint(const HttpFrame& frame);

  /// Accept client and add it to event processing.
  /// @param sock - listening socket.
  void acceptClient(NetSocket* sock);

  /// Closes and unregisters a connection.
  void closeConnection(int fd);

protected:

  using Binding = std::pair<std::string, HttpMethod>;

  /// A collection of endpoints.
  std::map<Binding, std::shared_ptr<EndpointHandler>> endpoints_;

  /// Active connections.
  std::map<int, HttpServerConnection*> connections_;

  /// Socket for accepting HTTP connections.
  NetSocket socket_v4_;

  /// IPv6 socket.
  NetSocket socket_v6_;

  /// Poller to handle events.
  PollSet* pollSet_;

  std::mutex mutex_;
};

}
}
#endif //MINIROS_HTTP_SERVER_H
