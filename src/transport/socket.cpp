//
// Created by dkargin on 7/19/25.
//

#include <cstring>

#include "miniros/transport/io.h"
#include "miniros/transport/socket.h"

#include "transport/transport_tcp.h"
#include "xmlrpcpp/XmlRpcSocket.h"

#define MINIROS_PACKAGE_NAME "net_socket"

namespace miniros {
namespace network {

NetSocket::NetSocket()
{}

NetSocket::NetSocket(int fd, bool own)
  :fd_(fd), own_(own)
{}

NetSocket::~NetSocket()
{
  close();
}

void NetSocket::close()
{
  if (own_ && fd_ != ROS_INVALID_SOCKET) {
    close_socket(fd_);
    fd_ = ROS_INVALID_SOCKET;
  }
  own_ = false;
  listening_ = false;
  peer_address_.reset();
}

Error NetSocket::listen(int maxQueuedClients)
{
  if (!valid())
    return Error::InvalidValue;
  if (::listen(fd_, maxQueuedClients) == 0) {
    listening_ = true;
    return Error::Ok;
  }
  int err = last_socket_error();
  if (err == EADDRINUSE) {
    return Error::Ok;
  }
  return Error::SystemError;
}

Error NetSocket::bind(int port, NetAddress::Type type)
{
  sockaddr_storage ss;
  size_t ss_len = 0;
  memset(&ss, 0, sizeof(ss));

  if (type == NetAddress::AddressIPv6)
  {
    sockaddr_in6 *address = reinterpret_cast<sockaddr_in6*>(&ss);
    ss_len = sizeof(sockaddr_in6);

    address->sin6_family = AF_INET6;
    address->sin6_addr = in6addr_any;
    address->sin6_port = htons(static_cast<uint16_t>(port));
  }
  else if (type == NetAddress::AddressIPv4)
  {
    sockaddr_in *address = reinterpret_cast<sockaddr_in*>(&ss);
    ss_len = sizeof(sockaddr_in);

    address->sin_family = AF_INET;
    address->sin_addr.s_addr = htonl(INADDR_ANY);
    address->sin_port = htons(static_cast<uint16_t>(port));
  } else {
    return Error::InvalidValue;
  }

  if (int ret = ::bind(fd_, reinterpret_cast<sockaddr*>(&ss), ss_len); ret != 0) {
    return Error::SystemError;
  }

  readLocalAddress(fd_, peer_address_);

  return Error::Ok;
}

int NetSocket::port() const
{
  return peer_address_.port;
}

bool NetSocket::valid() const
{
  return fd_ != ROS_INVALID_SOCKET;
}

int NetSocket::fd() const
{
  return fd_;
}

Error NetSocket::tcpListen(int port, NetAddress::Type type, int maxQueuedClients)
{
  if (valid())
    close();
  int addrType = 0;
  if (type == NetAddress::AddressIPv6)
    addrType = AF_INET6;
  else if (type == NetAddress::AddressIPv4)
    addrType = AF_INET;
  if (!addrType)
    return Error::InvalidValue;
  int fd = socket(addrType, SOCK_STREAM, 0);
  if (fd < 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("Error while trying to create listening socket: %s", err);
    return Error::SystemError;
  }
  fd_ = fd;

  if (Error err = setReuseAddr(); !err) {
    return err;
  }

  if (Error err = bind(port, type); !err) {
    MINIROS_ERROR("Error while binding socket to port %d: %s", port, err.toString());
    return err;
  }
  if (Error err = listen(maxQueuedClients); !err) {
    MINIROS_ERROR("Error while entering listening mode: %s", err.toString());
    return err;
  }
  return Error::Ok;
}

std::pair<std::shared_ptr<NetSocket>, Error> NetSocket::accept()
{
  sockaddr_in addr;

  socklen_t addrlen = sizeof(addr);
  // accept will truncate the address if the buffer is too small.
  // As we are not using it, no special case for IPv6
  // has to be made.
  int fd = ::accept(fd_, (struct sockaddr*)&addr, &addrlen);

  std::shared_ptr<NetSocket> sock(new NetSocket(fd, true));
  NetAddress address;
  fillAddress(addr, address);
  sock->setPeerAddress(address);
  return {sock, Error::Ok};
}

void NetSocket::setPeerAddress(const NetAddress& address)
{
  peer_address_ = address;
}

NetAddress NetSocket::peerAddress() const
{
  return peer_address_;
}

Error NetSocket::setNonBlock()
{
  if (!valid())
    return Error::InvalidValue;

  int result = set_non_blocking(fd_);
  if ( result != 0 ) {
    MINIROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", fd_, result);
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setReuseAddr()
{
  // Allow this port to be re-bound immediately so server re-starts are not delayed
  int sflag = 1;
  if (setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, (const char *)&sflag, sizeof(sflag)) != 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("setting socket [%d] as reuse_addr failed with error: %s", fd_, err);
    return Error::SystemError;
  }
  return Error::Ok;
}

// Read available text from the specified socket. Returns false on error.
std::pair<size_t, Error> NetSocket::read(std::string& s)
{
  constexpr int READ_SIZE = 4096;   // Number of bytes to attempt to read at a time
  char readBuf[READ_SIZE];

  size_t received = 0;
  bool wouldBlock = false;

  while ( !wouldBlock) {
    int n = recv(fd_, readBuf, READ_SIZE-1, 0);
    if (n > 0) {
      readBuf[n] = 0;
      s.append(readBuf, n);
      received += n;
    } else if (n == 0) {
      return {received, Error::EndOfFile};
    } else {
      const int err = last_socket_error();
      if (err == EINPROGRESS || err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
        wouldBlock = true;
      } else {
        return {received, Error::SystemError};
      }
    }
  }
  return {received, Error::Ok};
}

std::pair<size_t, Error> NetSocket::write(const char* sp, size_t size)
{
  size_t written = 0;
  bool wouldBlock = false;

  while ( written < size > 0 && ! wouldBlock ) {
    int n = send(fd_, sp, size - written, 0);
    MINIROS_DEBUG("NetSocket::nbWrite: send/write returned %d.", n);
    if (n > 0) {
      sp += n;
      written += n;
      continue;
    }

    int err = last_socket_error();
    if (err == EINPROGRESS || err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
      wouldBlock = true;
    } else {
      return {written, Error::SystemError};
    }
  }
  return {written, Error::Ok};
}

}
}