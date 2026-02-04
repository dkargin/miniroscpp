//
// Created by dkargin on 7/20/25.
//

#include <cassert>
#include <mutex>

#include "miniros/network/socket.h"
#include "miniros/io/poll_set.h"
#include "miniros/internal/lifetime.h"

#include "miniros/http/http_server.h"

#include "internal/scoped_locks.h"
#include "miniros/http/http_server_connection.h"

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

namespace miniros {
namespace http {

struct Binding {
  std::unique_ptr<EndpointFilter> filter;
  std::shared_ptr<EndpointHandler> endpoint;
  CallbackQueuePtr callbackQueue;

  Binding(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& endpoint, const CallbackQueuePtr& cb)
    : filter(std::move(filter)), endpoint(endpoint), callbackQueue(cb)
  {}
};

struct HttpServer::Internal {
  using Lock = std::unique_lock<Lifetime<HttpServer>>;

  CloseConnectionHandler onCloseConnection;
  /// A collection of endpoints.
  std::vector<Binding> endpoints;
  /// A mutex for endpoints.
  std::mutex endpointsGuard;

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
  }

  ~Internal()
  {
    MINIROS_DEBUG("HttpServer::~Internal()");
  }

  void closeConnection(Lock& lock, int fd, const std::string& reason);
};

void HttpServer::Internal::closeConnection(Lock& lock, int fd, const std::string& reason)
{
  MINIROS_DEBUG("HttpServer[%d]::closeConnection(): %s", fd, reason.c_str());
  std::shared_ptr<HttpServerConnection> connection;
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
    }, lifetime);

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

  if (internal_->pollSet && internal_->socket_v4.valid()) {
    if (!internal_->pollSet->delSocket(internal_->socket_v4.fd())) {
      return Error::InternalError;
    }
  }

  internal_->socket_v4.close();
  while (!internal_->connections.empty()) {
    auto it = internal_->connections.begin();
    internal_->closeConnection(lock, it->first, "HttpServer::stop()");
  }
  MINIROS_DEBUG("HttpServer::stop() finished");
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

  auto [client, err] = sock->accept();

  if (!client) {
    MINIROS_ERROR("HttpServer::accept() - failed to accept client: %s", err.toString());
    return;
  }

  int fd = client->fd();
  MINIROS_DEBUG("HttpServer::accept() - accepting new client fd=%d", client->fd());

  client->setNonBlock();
  client->setNoDelay(true);

  assert(internal_->pollSet);

  std::shared_ptr<HttpServerConnection> connection(new HttpServerConnection(this, client, internal_->pollSet));
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
      if (!newFlags) {
        internal_->closeConnection(lock, fd, "EOF");
      }
      return newFlags;
  }, internalCopy);
}

Error HttpServer::registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const CallbackQueuePtr& cb)
{
  if (!internal_)
    return Error::InternalError;


  std::unique_lock<std::mutex> lock(internal_->endpointsGuard);
  internal_->endpoints.emplace_back(std::move(filter), handler, cb);
  return Error::Ok;
}

std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> HttpServer::findEndpoint(const HttpParserFrame& frame)
{
  std::unique_lock<std::mutex> lock(internal_->endpointsGuard);

  for (const Binding& binding: internal_->endpoints) {
    if (binding.filter->check(frame)) {
      return {binding.endpoint, binding.callbackQueue};
    }
  }

  return {};
}

}
}