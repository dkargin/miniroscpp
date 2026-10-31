//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_HTTP_SERVER_H
#define MINIROS_HTTP_SERVER_H

#include <memory>
#include <mutex> // for unique_lock and scoped_lock
#include <string>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_proxy.h"

namespace miniros {

class PollSet;
class CallbackQueue;
template <class T> class Lifetime;

namespace network {
class NetSocket;
struct ClientInfo;
}

namespace http {

class HttpServerConnection;

/// HttpServer serves XMLRPC MasterApi, SlaveApi and additional HTTP endpoints from user.
class MINIROS_DECL HttpServer {
public:
  HttpServer(PollSet* pollSet);
  virtual ~HttpServer();

  /// Start server on specific port.
  Error start(int port);

  /// Start IPv6 server on specific port.
  Error start6(int port);

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
  void registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const std::shared_ptr<CallbackQueue>& cb);

  /// Unregister all endpoints using specific handler.
  void unregisterAll(const std::shared_ptr<EndpointHandler>& handler);

  /// Find endpoint for request.
  std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> findEndpoint(const HttpParserFrame& frame);

  /// Called when HTTP connection is closed.
  using CloseConnectionHandler = std::function<void (const std::shared_ptr<HttpServerConnection>&, const std::string& reason)>;

  void setCloseConnectionHandler(CloseConnectionHandler&& handler);

  /// Get number of connections.
  size_t getConnectionsCount() const;

  /// Hand requests under @p pathPrefix to @p acceptor once their headers are complete.
  /// @p pathPrefix is copied. It is an absolute path without a trailing slash.
  ///
  /// The pointer is not owned. @p acceptor must stay alive while it is installed,
  /// including during acceptHttpProxy on the poll thread. That call may use this
  /// HttpServer. Pass nullptr before destroying the acceptor, and clear it before
  /// this HttpServer is destroyed. stop() does not clear it.
  /// A null acceptor or an empty prefix disables the handoff.
  void setHttpProxyAcceptor(HttpProxyAcceptor* acceptor, const std::string& pathPrefix);

  /// True when a request path should be detached for the installed proxy acceptor.
  /// Takes the server lifetime lock.
  bool matchesHttpProxy(std::string_view path) const;

protected:
  friend class HttpServerConnection;

  /// It is called by HttpServerConnection when it is closed by any reason.
  void onConnectionClosed(std::unique_lock<std::mutex>& /*lock*/,
    const std::shared_ptr<HttpServerConnection>& connection, bool upgrade, const std::string& reason);

  /// Hand a proxy-prefix socket to the installed acceptor. False if nobody took it.
  /// Caller holds the server lifetime lock.
  bool offerHttpProxy(const HttpParserFrame& frame, const std::shared_ptr<network::NetSocket>& client);

  /// Same test as matchesHttpProxy. Caller holds the server lifetime lock.
  bool proxyPrefixMatches(std::string_view path) const;

  /// Creates HTTP connection.
  /// This method is virtual to be able to override it in tests.
  virtual std::shared_ptr<HttpServerConnection> makeConnection(const std::shared_ptr<network::NetSocket>& socket);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}
#endif //MINIROS_HTTP_SERVER_H
