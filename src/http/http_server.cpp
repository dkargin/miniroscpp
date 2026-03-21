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
#include "miniros/http/http_server_connection.h"
#include "miniros/callback_queue.h"
#include "miniros/http/endpoint_collection.h"

namespace miniros {
namespace http {

struct HttpServer::Internal {
  using Lock = std::unique_lock<Lifetime<HttpServer>>;

  CloseConnectionHandler onCloseConnection;

  /// Thread safe collection of endpoints.
  std::shared_ptr<internal::EndpointCollection> endpoints;

  /// Active connections.
  std::map<int, std::shared_ptr<HttpServerConnection>> connections;

  /// Socket for accepting HTTP connections.
  network::NetSocket socket_v4;

  /// IPv6 socket.
  network::NetSocket socket_v6;

  /// Poller to handle events.
  PollSet* pollSet;

  /// Global mutex.
  std::shared_ptr<Lifetime<HttpServer>> lifetime = std::make_shared<Lifetime<HttpServer>>();

  Internal(PollSet* ps) : pollSet(ps)
  {
    assert(pollSet);
    endpoints = std::make_shared<internal::EndpointCollection>();
  }

  ~Internal()
  {
    MINIROS_DEBUG("HttpServer::~Internal()");
  }

  void closeConnection(Lock& lock, int fd, const std::string& reason);
};

void HttpServer::Internal::closeConnection(Lock& lock, int fd, const std::string& reason)
{
  MINIROS_INFO("HttpServer::closeConnection() fd=%d: %s", fd, reason.c_str());
  std::shared_ptr<HttpServerConnection> connection;
  // TODO: Connection can close and unsubscribe itself.
  // TODO: Still need to notify external users with onCloseConnection.
  auto it = connections.find(fd);
  if (it != connections.end()) {
    if (pollSet) {
      pollSet->delSocket(fd);
    }
    connection = it->second;
    connection->detach();
    connections.erase(fd);
    connection->close();

    if (onCloseConnection) {
      auto handler = onCloseConnection;
      ScopedUnlock unlock(lock);
      handler(connection, reason);
    }
  }
}

void HttpServer::setCloseConnectionHandler(CloseConnectionHandler&& handler)
{
  internal_->onCloseConnection = std::move(handler);
}

void HttpServer::closeConnection(int fd, const std::string& reason)
{
  assert(internal_);
  if (!internal_)
    return;
  std::unique_lock lock(*internal_->lifetime);
  internal_->closeConnection(lock, fd, reason);
}

HttpServer::HttpServer(PollSet* pollSet)
{
  assert(pollSet);
  internal_ = std::make_unique<Internal>(pollSet);
}

HttpServer::~HttpServer()
{
  if (internal_) {
    auto lifetime = internal_->lifetime;

    // 1. Close all connections.
    // 2. Detach all endpoints.
    stop();
    std::unique_lock lock(*lifetime);
    internal_->lifetime->alive = false;
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

  std::unique_lock lock(*internal_->lifetime);

  int fd = 0;
  if (internal_->pollSet && internal_->socket_v4.valid()) {
    fd = internal_->socket_v4.fd();
    LOCAL_INFO("HttpServer[%d]::stop() - stopping listener socket", fd);
    if (!internal_->pollSet->delSocket(internal_->socket_v4.fd())) {
      return Error::InternalError;
    }
  }

  internal_->socket_v4.close();
  while (!internal_->connections.empty()) {
    auto it = internal_->connections.begin();
    LOCAL_INFO("HttpServer[%d]::stop() - closing connection %d", fd, it->first);
    internal_->closeConnection(lock, it->first, "HttpServer::stop()");
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
  return std::make_shared<HttpServerConnection>(this, socket, internal_->pollSet);
}


void HttpServer::acceptClient(const std::shared_ptr<Lifetime<HttpServer>>& lifetime, network::NetSocket* sock)
{
  // This callback is called from PollSet thread.
  std::unique_lock lock(*lifetime);
  if (!lifetime->alive)
    return;

  if (!internal_)
    return;
  if (!sock)
    return;

  auto [clientSock, err] = sock->accept();

  if (!clientSock) {
    MINIROS_ERROR("HttpServer::accept() - failed to accept client: %s", err.toString());
    return;
  }

  int fd = clientSock->fd();
  MINIROS_DEBUG("HttpServer::accept() - accepting new client fd=%d", clientSock->fd());

  clientSock->setNonBlock();
  clientSock->setNoDelay(true);

  assert(internal_->pollSet);

  std::shared_ptr<HttpServerConnection> connection = makeConnection(clientSock);
  connection->setEndpointCollection(internal_->endpoints);

  internal_->connections[fd] = connection;
  auto internalCopy = internal_->lifetime;

  internal_->pollSet->addSocket(fd, PollSet::EventIn | PollSet::EventUpdate,
    [this, wconnection = std::weak_ptr(connection), fd, internalCopy](int flags)
    {
      // HttpServerConnection can probably be destroyed here.
      std::unique_lock lock(*internalCopy);

      auto connection = wconnection.lock();
      if (!connection)
        return 0;
      int newFlags = connection->handleEvents(flags);
      if (!internalCopy->alive) {
        return 0;
      }
      /*
      if (!newFlags) {
        internal_->closeConnection(lock, fd, "EOF");
      }*/
      return newFlags;
  }, internalCopy, THIS_LOCATION);
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