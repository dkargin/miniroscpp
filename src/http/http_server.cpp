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
  using Lock = std::unique_lock<Lifetime<HttpServer>>;

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

  Internal(HttpServer* server, PollSet* ps) : pollSet(ps)
  {
    assert(pollSet);
    endpoints = std::make_shared<internal::EndpointCollection>();
    lifetime = std::make_shared<Lifetime<HttpServer>>(server);
  }

  ~Internal()
  {
    MINIROS_DEBUG("HttpServer::~Internal()");
  }

  void closeConnection(Lock& lock, const std::shared_ptr<HttpServerConnection>& connection, const std::string& reason);
};

void HttpServer::Internal::closeConnection(Lock& lock, const std::shared_ptr<HttpServerConnection>& connection,
  const std::string& reason)
{
  int fd = connection->fd();
  MINIROS_INFO("HttpServer::closeConnection() fd=%d: %s", fd, reason.c_str());

  auto it = connections.find(connection);
  if (it == connections.end())
    return;
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

void HttpServer::onConnectionClosed(const std::shared_ptr<HttpServerConnection>& connection, const std::string& reason)
{
  assert(internal_);
  if (!internal_)
    return;
  std::unique_lock lock(*internal_->lifetime);
  internal_->closeConnection(lock, connection, reason);
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
    internal_.reset();
  }
  MINIROS_DEBUG("HttpServer::~HttpServer()");
}

Error HttpServer::start(int port)
{
  if (!internal_)
    return Error::InternalError;

  std::unique_lock lock(*internal_->lifetime);
  if (Error err = internal_->socket_v4.tcpListen(port, network::NetAddress::AddressIPv4, 100); !err) {
    return err;
  }

  if (Error err = internal_->socket_v4.setNonBlock(); !err) {
    return err;
  }

  int fd = internal_->socket_v4.fd();
  auto lifetime = internal_->lifetime;

  bool added = internal_->pollSet->addSocket(fd, PollSet::EventIn,
    [this, lifetime](int flags)
    {
      if (flags & PollSet::EventIn) {
        this->acceptClient(lifetime, &internal_->socket_v4);
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

  MINIROS_INFO("HttpServer::start() created HTTP ipv4 server, fd=%d", fd);
  // TODO: ipv6 listener.
  return Error::Ok;
}

Error HttpServer::stop()
{
  if (!internal_)
    return Error::InternalError;

  std::set<std::shared_ptr<HttpServerConnection>> connectionsCopy;
  int listenFd = MINIROS_INVALID_SOCKET;

  {
    std::unique_lock lock(*internal_->lifetime);

    if (internal_->pollSet && internal_->socket_v4.valid()) {
      listenFd = internal_->socket_v4.fd();
      LOCAL_INFO("HttpServer[%d]::stop() - stopping listener socket", listenFd);
      if (!internal_->pollSet->delSocket(internal_->socket_v4.fd())) {
        return Error::InternalError;
      }
    }

    internal_->socket_v4.close();

    std::swap(connectionsCopy, internal_->connections);
  }

  for (auto connection: connectionsCopy) {
    int fd = connection->fd();
    LOCAL_INFO("HttpServer[%d]::stop() - dropping connection %d", listenFd, fd);
    connection->detachFromServer(true);
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

void HttpServer::acceptClient(const std::shared_ptr<Lifetime<HttpServer>>& lifetime, network::NetSocket* sock)
{
  if (!sock)
    return;

  // This callback is called from PollSet thread.
  // Check if server instance is still alive and usable.
  auto lock = lifetime->getLock();
  if (!lifetime->valid(lock))
    return;

  if (!internal_)
    return;

  if (internal_->stoppingListeners)
    return;

  auto [clientSock, err] = sock->accept();

  if (!clientSock) {
    MINIROS_ERROR("HttpServer::accept() - failed to accept client: %s", err.toString());
    return;
  }

  MINIROS_DEBUG("HttpServer::accept() - accepting new client fd=%d", clientSock->fd());

  clientSock->setNonBlock();
  clientSock->setNoDelay(true);

  assert(internal_->pollSet);

  std::shared_ptr<HttpServerConnection> connection = makeConnection(clientSock);
  connection->setEndpointCollection(internal_->endpoints);
  internal_->connections.insert(connection);
  // Connection object will start receiving events from external thread.
  connection->attachPollSet(internal_->pollSet);
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

}
}