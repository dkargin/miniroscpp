//
// Created by dkargin on 7/20/25.
//

#include <cassert>
#include <mutex>

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

#include "miniros/network/socket.h"
#include "miniros/io/poll_set.h"
#include "miniros/internal/lifetime.h"

#include "miniros/http/http_server.h"

#include "internal/scoped_locks.h"
#include "io/io.h"
#include "miniros/callback_queue.h"
#include "miniros/http/endpoint_collection.h"
#include "miniros/http/http_server_connection.h"

namespace miniros {
namespace http {

struct HttpServer::Internal {
  using Lock = Lifetime<HttpServer>::Lock;

  /// Owner.
  HttpServer* server = nullptr;

  CloseConnectionHandler onCloseConnection;

  /// Thread safe collection of endpoints.
  std::shared_ptr<internal::EndpointCollection> endpoints;

  /// Active connections.
  std::set<std::shared_ptr<HttpServerConnection>> connections;

  /// Socket for accepting HTTP connections.
  network::NetSocket socket_v4;

  /// IPv6 socket.
  network::NetSocket socket_v6;

  /// Flag indicates that server is stopping and all incoming clients should be rejected.
  volatile bool stoppingListeners = false;

  /// Poller to handle events.
  PollSet* pollSet;

  /// Global mutex.
  std::shared_ptr<Lifetime<HttpServer>> lifetime;

  Internal(HttpServer* server, PollSet* ps) : server(server), pollSet(ps)
  {
    assert(pollSet);
    endpoints = std::make_shared<internal::EndpointCollection>();
    lifetime = std::make_shared<Lifetime<HttpServer>>(server);
  }

  ~Internal()
  {
    MINIROS_DEBUG("HttpServer::~Internal()");
  }

  void onConnectionClosed(Lock& lock, const std::shared_ptr<HttpServerConnection>& connection, bool upgrade, const std::string& reason);

  Error start(network::NetSocket& socket, network::NetAddress::Type addrType, int port);

  /// Accept client and add it to event processing.
  /// For internal usage only.
  /// @param lock - acquired lock on internal mutex.
  /// @param sock - listening socket.
  void acceptClient(Lock& lock, network::NetSocket* sock);
};

Error HttpServer::Internal::start(network::NetSocket& socket, network::NetAddress::Type addrType, int port)
{
  std::unique_lock lock(*lifetime);
  if (Error err = socket.tcpListen(port, addrType, 100); !err) {
    return err;
  }

  if (Error err = socket.setNonBlock(); !err) {
    return err;
  }

  int fd = socket.fd();
  std::weak_ptr weakLifetime = lifetime;

  bool added = pollSet->addSocket(fd, PollSet::EventIn,
    [this, weakLifetime, &socket](int flags)
    {
      auto l = weakLifetime.lock();
      if (!l)
        return 0;
      // This callback is called from PollSet thread.
      // Check if server instance is still alive and usable.
      auto lock = l->getLock();
      if (!l->valid(lock))
        return 0;

      if (flags & PollSet::EventIn) {
        this->acceptClient(lock, &socket);
      } else {
        MINIROS_WARN("HttpServer socket: unexpected event %d", flags);
      }
      return 0;
    }, lifetime, THIS_LOCATION);

  if (!added)
  {
    MINIROS_ERROR("Failed to register listening socket");
    return Error::InternalError;
  }

  MINIROS_INFO("HttpServer::start() created HTTP server, fd=%d", fd);
  return Error::Ok;
}

void HttpServer::Internal::acceptClient(Lock& /*lock*/, network::NetSocket* sock)
{
  if (!sock)
    return;

  if (stoppingListeners)
    return;

  if (!server)
    return;
  int listenFd = sock->fd();
  auto [clientSock, err] = sock->accept();

  if (!clientSock) {
    MINIROS_ERROR("HttpServer[%d]::accept() - failed to accept client: %s", listenFd, err.toString());
    return;
  }

  MINIROS_DEBUG("HttpServer[%d]::accept() - accepting new client fd=%d", listenFd, clientSock->fd());

  clientSock->setNonBlock();
  clientSock->setNoDelay(true);

  assert(pollSet);

  std::shared_ptr<HttpServerConnection> connection = server->makeConnection(clientSock);
  connection->setEndpointCollection(endpoints);
  connections.insert(connection);

  // Connection object will start receiving events from external thread.
  connection->attachPollSet(pollSet);
}

void HttpServer::Internal::onConnectionClosed(Lock& lock, const std::shared_ptr<HttpServerConnection>& connection,
  bool upgrade, const std::string& reason)
{
  int fd = connection->fd();

  auto it = connections.find(connection);
  if (it == connections.end()) {
    MINIROS_WARN("HttpServer::onConnectionClosed() fd=%d: %s - unknown connection", fd, reason.c_str());
    return;
  }

  MINIROS_DEBUG("HttpServer::onConnectionClosed() fd=%d: %s", fd, reason.c_str());
  connections.erase(connection);
  if (onCloseConnection) {
    auto handler = onCloseConnection;
    ScopedUnlock unlock(lock);
    handler(connection, reason);
  }
}

void HttpServer::setCloseConnectionHandler(CloseConnectionHandler&& handler)
{
  internal_->onCloseConnection = std::move(handler);
}

void HttpServer::onConnectionClosed(std::unique_lock<std::mutex>& lock,
  const std::shared_ptr<HttpServerConnection>& connection, bool upgrade, const std::string& reason)
{
  assert(internal_);
  if (!internal_)
    return;
  internal_->onConnectionClosed(lock, connection, upgrade, reason);
}

HttpServer::HttpServer(PollSet* pollSet)
{
  assert(pollSet);
  internal_ = std::make_unique<Internal>(this, pollSet);
}

HttpServer::~HttpServer()
{
  if (internal_) {
    auto lifetime = internal_->lifetime;

    // 1. Close all connections.
    // 2. Detach all endpoints.
    stop();
    auto lock = lifetime->getLock();
    internal_->lifetime->reset(lock);
    internal_->pollSet = nullptr;
    internal_->server = nullptr;
    internal_.reset();
  }
  MINIROS_DEBUG("HttpServer::~HttpServer()");
}

Error HttpServer::start(int port)
{
  if (!internal_)
    return Error::InternalError;

  return internal_->start(internal_->socket_v4, network::NetAddress::AddressIPv4, port);
}

Error HttpServer::start6(int port)
{
  if (!internal_)
    return Error::InternalError;

  return internal_->start(internal_->socket_v6, network::NetAddress::AddressIPv6, port);
}

Error HttpServer::stop()
{
  if (!internal_)
    return Error::InternalError;

  std::set<std::shared_ptr<HttpServerConnection>> connectionsCopy;


  {
    std::unique_lock lock(*internal_->lifetime);

    if (internal_->pollSet) {
      if (internal_->socket_v4.valid()) {
        int fd = internal_->socket_v4.fd();
        LOCAL_INFO("HttpServer::stop() - stopping ipv4 listener socket %d", fd);
        if (!internal_->pollSet->delSocket(fd)) {
          LOCAL_WARN("HttpServer::stop() - failed to detach ipv4 socket from PollSet");
        }
      }

      if (internal_->socket_v6.valid()) {
        int fd = internal_->socket_v6.fd();
        LOCAL_INFO("HttpServer::stop() - stopping ipv6 listener socket %d", fd);
        if (!internal_->pollSet->delSocket(fd)) {
          LOCAL_WARN("HttpServer::stop() - failed to detach ipv6 socket from PollSet");
        }
      }
    }

    internal_->socket_v4.close();
    internal_->socket_v6.close();

    std::swap(connectionsCopy, internal_->connections);
  }

  for (auto connection: connectionsCopy) {
    int fd = connection->fd();
    LOCAL_INFO("HttpServer[]::stop() - dropping connection %d", fd);
    connection->detachByServer();
  }

  return Error::Ok;
}

int HttpServer::getPort() const
{
  return internal_->socket_v4.port();
}

int HttpServer::getPortIp6() const
{
  return internal_->socket_v6.port();
}

PollSet* HttpServer::getPollSet() const
{
  return internal_ ? internal_->pollSet : nullptr;
}

std::shared_ptr<HttpServerConnection> HttpServer::makeConnection(const std::shared_ptr<network::NetSocket>& socket)
{
  return std::make_shared<HttpServerConnection>(internal_->lifetime, socket);
}

Error HttpServer::registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const CallbackQueuePtr& cb)
{
  if (!internal_ || !internal_->endpoints)
    return Error::InternalError;

  return internal_->endpoints->registerEndpoint(std::move(filter), handler, cb);
}

std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> HttpServer::findEndpoint(const HttpParserFrame& frame)
{
  if (!internal_ || !internal_->endpoints)
    return {};

  return internal_->endpoints->findEndpoint(frame);
}

size_t HttpServer::getConnectionsCount() const
{
  if (!internal_)
    return 0;

  auto lock = internal_->lifetime->getLock();
  return internal_->connections.size();
}


}
}