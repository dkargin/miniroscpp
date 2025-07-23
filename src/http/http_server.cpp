//
// Created by dkargin on 7/20/25.
//

#include <cassert>
#include <mutex>

#include "../../include/miniros/network/socket.h"
#include "miniros/transport/poll_set.h"

#include "miniros/transport/io.h"

#include "miniros/http/http_server.h"
#include "miniros/http/http_server_connection.h"

#define MINIROS_PACKAGE_NAME "http_server"

namespace miniros {
namespace http {

struct Binding {
  std::unique_ptr<EndpointFilter> filter;
  std::shared_ptr<EndpointHandler> endpoint;
};

struct HttpServer::Internal {

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
  std::mutex mutex;

  Internal(PollSet* ps) : pollSet(ps)
  {}
};

HttpServer::HttpServer(PollSet* pollSet)
{
  internal_ = std::make_unique<Internal>(pollSet);
}

HttpServer::~HttpServer()
{
  if (internal_) {
    // 1. Close all connections.
    // 2. Detach all endpoints.
    std::unique_lock lock(internal_->mutex);
    stop();
    internal_->pollSet = nullptr;
  }
}

Error HttpServer::start(int port)
{
  if (!internal_)
    return Error::InternalError;

  std::unique_lock lock(internal_->mutex);
  if (Error err = internal_->socket_v4.tcpListen(port, network::NetAddress::AddressIPv4, 100); !err) {
    return err;
  }

  if (Error err = internal_->socket_v4.setNonBlock(); !err) {
    return err;
  }

  int fd = internal_->socket_v4.fd();
  bool added = internal_->pollSet->addSocket(fd,
    [this](int flags)
    {
      if (flags & POLLIN) {
        this->acceptClient(&internal_->socket_v4);
      } else {
        MINIROS_WARN("HttpServer socket: unexpected event %d", flags);
      }
    });

  if (!added)
  {
    MINIROS_ERROR("Failed to register listening socket");
    return Error::InternalError;
  }

  if (!internal_->pollSet->addEvents(fd, POLLIN)) {
    MINIROS_ERROR("Failed to add events");
    return Error::InternalError;
  }

  MINIROS_INFO("Created HTTP ipv4 server fd=%d", fd);
  // TODO: ipv6 listener.
  return Error::Ok;
}

Error HttpServer::stop()
{
  if (!internal_->pollSet->delSocket(internal_->socket_v4.fd())) {
    return Error::InternalError;
  }
  internal_->socket_v4.close();
  while (!internal_->connections.empty()) {
    auto it = internal_->connections.begin();
    closeConnection(it->first, "HttpServer::stop()");
  }
  return Error::Ok;
}

int HttpServer::getPortIp4() const
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

void HttpServer::acceptClient(network::NetSocket* sock)
{
  auto [client, err] = sock->accept();

  if (!client) {
    MINIROS_ERROR("Failed to accept HTTP client: %s", err.toString());
    return;
  }

  int fd = client->fd();
  MINIROS_DEBUG("Accepting new HTTP client fd=%d", client->fd());

  client->setNonBlock();
  client->setNoDelay(true);

  std::shared_ptr<HttpServerConnection> connection(new HttpServerConnection(this, client));
  internal_->connections[fd] = connection;
  internal_->pollSet->addSocket(fd, [this, connection, fd](int flags) {
    int newFlags = connection->handleEvents(flags);
    if (!newFlags) {
      closeConnection(fd, "EOF");
    } else {
      internal_->pollSet->setEvents(fd, newFlags);
    }
  }, connection);

  internal_->pollSet->addEvents(fd, POLLIN);
}

Error HttpServer::registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler)
{
  if (!internal_)
    return Error::InternalError;

  Binding binding{std::move(filter), handler};

  std::unique_lock<std::mutex> lock(internal_->endpointsGuard);
  internal_->endpoints.emplace_back(std::move(binding));
  return Error::Ok;
}

EndpointHandler* HttpServer::findEndpoint(const HttpFrame& frame)
{
  std::unique_lock<std::mutex> lock(internal_->endpointsGuard);

  for (const Binding& binding: internal_->endpoints) {
    if (binding.filter->check(frame)) {
      return binding.endpoint.get();
    }
  }

  return nullptr;
}

void HttpServer::closeConnection(int fd, const char* reason)
{
  if (!internal_)
    return;
  MINIROS_DEBUG("HttpServer::closeConnection(%d): %s", fd, reason);
  std::shared_ptr<HttpServerConnection> connection;
  std::unique_lock<std::mutex> lock(internal_->mutex);
  auto it = internal_->connections.find(fd);
  if (it != internal_->connections.end()) {
    internal_->pollSet->delSocket(fd);
    connection = it->second;
    internal_->connections.erase(fd);
    connection->close();
  }
}

}
}