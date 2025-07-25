//
// Created by dkargin on 7/20/25.
//

#include <cassert>

#include "miniros/transport/http_server.h"
#include "miniros/transport/http_server_connection.h"

#include "miniros/transport/xmlrpc_handler.h"

#include "xmlrpcpp/XmlRpcSocket.h"
#include "xmlrpcpp/XmlRpcUtil.h"

#define MINIROS_PACKAGE_NAME "http_server"

namespace miniros {
namespace network {

HttpServer::HttpServer(PollSet* pollSet)
  :pollSet_(pollSet)
{}

Error HttpServer::start(int port)
{
  if (Error err = socket_v4_.tcpListen(port, NetAddress::AddressIPv4, 100); !err) {
    return err;
  }

  if (Error err = socket_v4_.setNonBlock(); !err) {
    return err;
  }

  bool added = pollSet_->addSocket(socket_v4_.fd(),
    [this](int flags)
    {
      if (flags & POLLIN) {
        this->acceptClient(&socket_v4_);
      } else {
        MINIROS_WARN("HttpServer socket: unexpected event %d", flags);
      }
    });

  if (!added)
  {
    MINIROS_ERROR("Failed to register listening socket");
    return Error::InternalError;
  }

  if (!pollSet_->addEvents(socket_v4_.fd(), POLLIN)) {
    MINIROS_ERROR("Failed to add events");
    return Error::InternalError;
  }

  MINIROS_INFO("Created HTTP ipv4 server fd=%d", socket_v4_.fd());
  // TODO: ipv6 listener.
  return Error::Ok;
}

Error HttpServer::stop()
{
  // TODO: Can we be here while socket is being used in some event?
  if (!pollSet_->delSocket(socket_v4_.fd())) {
    return Error::InternalError;
  }
  socket_v4_.close();
  return Error::Ok;
}

int HttpServer::getPortIp4() const
{
  return socket_v4_.port();
}

int HttpServer::getPortIp6() const
{
  return socket_v6_.port();
}

PollSet* HttpServer::getPollSet() const
{
  return pollSet_;
}

void HttpServer::acceptClient(NetSocket* sock)
{
  auto [client, err] = sock->accept();

  if (!client) {
    MINIROS_ERROR("Failed to accept HTTP client: %s", err.toString());
    return;
  }

  int fd = client->fd();
  MINIROS_INFO("Accepting new HTTP client fd=%d", client->fd());

  client->setNonBlock();
  auto* connection = new HttpServerConnection(this, client);
  connections_[fd] = connection;
  pollSet_->addSocket(fd, [this, connection, fd](int flags) {
    int newFlags = connection->handleEvents(flags);
    if (!newFlags) {
      MINIROS_INFO("HttpServerConnection(%d) returned 0 - closing", fd);
      closeConnection(fd);
    } else {
      pollSet_->setEvents(fd, newFlags);
    }
  });

  pollSet_->addEvents(fd, POLLIN);
}

Error HttpServer::registerEndpoint(HttpMethod method, const std::string& path, const std::shared_ptr<EndpointHandler>& handler)
{
  Binding binding{path, method};
  std::unique_lock<std::mutex> lock(mutex_);
  auto it = endpoints_.find(binding);
  if (it != endpoints_.end()) {
    return Error::AddressInUse;
  }
  endpoints_.emplace(binding, handler);
  return Error::Ok;
}

HttpServer::EndpointHandler* HttpServer::findEndpoint(const HttpFrame& frame)
{
  Binding binding{frame.getPath(), frame.requestMethod};

  std::unique_lock<std::mutex> lock(mutex_);
  auto it = endpoints_.find(binding);
  if (it != endpoints_.end()) {
    return it->second.get();
  }
  return nullptr;
}

void HttpServer::closeConnection(int fd)
{
  HttpServerConnection* connection = nullptr;
  std::unique_lock<std::mutex> lock(mutex_);
  auto it = connections_.find(fd);
  if (it != connections_.end()) {
    pollSet_->delSocket(fd);
    connection = it->second;
    connections_.erase(fd);
    connection->close();
  }

  if (connection) {
    delete connection;
  }
}

}
}