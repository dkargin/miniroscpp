//
// Created by dkargin on 7/19/25.
//

#include <cstring>

#include "miniros/transport/io.h"
#include "miniros/transport/socket.h"

#include "transport/transport_tcp.h"
#include "xmlrpcpp/XmlRpcSocket.h"

#ifndef WIN32
#include <sys/uio.h>
#endif

#define MINIROS_PACKAGE_NAME "net_socket"

namespace miniros {
namespace network {

struct NetSocket::Internal {
  int fd = -1;
  /// Do we own the FD and close it on exit.
  bool own = false;

  /// Socket is listening for connections.
  bool listening = false;

  /// Address of connected endpoint.
  NetAddress peer_address;

  /// Close socket and reset all data.
  void close()
  {
    if (own && fd != ROS_INVALID_SOCKET) {
      close_socket(fd);
      fd = ROS_INVALID_SOCKET;
    }
    own = false;
    listening = false;
    peer_address.reset();
  }
};

NetSocket::NetSocket()
{
  internal_.reset(new Internal());
}

NetSocket::NetSocket(int fd, bool own)
{
  internal_.reset(new Internal());
  internal_->fd = fd;
  internal_->own = own;
}

NetSocket::~NetSocket()
{
  close();
}

void NetSocket::close()
{
  internal_->close();
}

Error NetSocket::listen(int maxQueuedClients)
{
  if (!valid())
    return Error::InvalidValue;
  if (::listen(internal_->fd, maxQueuedClients) == 0) {
    internal_->listening = true;
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

  if (int ret = ::bind(internal_->fd, reinterpret_cast<sockaddr*>(&ss), ss_len); ret != 0) {
    return Error::SystemError;
  }

  readLocalAddress(internal_->fd, internal_->peer_address);

  return Error::Ok;
}

int NetSocket::port() const
{
  return internal_->peer_address.port;
}

bool NetSocket::valid() const
{
  return internal_->fd != ROS_INVALID_SOCKET;
}

int NetSocket::fd() const
{
  return internal_->fd;
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
  internal_->fd = fd;

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
  int fd = ::accept(internal_->fd, (struct sockaddr*)&addr, &addrlen);

  std::shared_ptr<NetSocket> sock(new NetSocket(fd, true));
  NetAddress address;
  fillAddress(addr, address);
  sock->internal_->peer_address = address;
  return {sock, Error::Ok};
}

NetAddress NetSocket::peerAddress() const
{
  return internal_->peer_address;
}

Error NetSocket::setNonBlock()
{
  if (!valid())
    return Error::InvalidValue;

  int result = set_non_blocking(internal_->fd);
  if ( result != 0 ) {
    MINIROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", internal_->fd, result);
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setReuseAddr()
{
  // Allow this port to be re-bound immediately so server re-starts are not delayed
  int sflag = 1;
  if (setsockopt(internal_->fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&sflag, sizeof(sflag)) != 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("setting socket [%d] as reuse_addr failed with error: %s", internal_->fd, err);
    return Error::SystemError;
  }
  return Error::Ok;
}

bool expectingBlock(int err)
{
  return err == EINPROGRESS || err == EAGAIN || err == EWOULDBLOCK || err == EINTR;
}

// Read available text from the specified socket. Returns false on error.
std::pair<size_t, Error> NetSocket::read(std::string& s)
{
  constexpr int READ_SIZE = 4096;   // Number of bytes to attempt to read at a time
  char readBuf[READ_SIZE];

  size_t received = 0;
  bool wouldBlock = false;

  while ( !wouldBlock) {
    int n = recv(internal_->fd, readBuf, READ_SIZE-1, 0);
    if (n > 0) {
      readBuf[n] = 0;
      s.append(readBuf, n);
      received += n;
    } else if (n == 0) {
      return {received, Error::EndOfFile};
    } else {
      const int err = last_socket_error();
      if (expectingBlock(err)) {
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
    int n = send(internal_->fd, sp, size - written, 0);
    MINIROS_DEBUG("NetSocket::nbWrite: send/write returned %d.", n);
    if (n > 0) {
      sp += n;
      written += n;
      continue;
    }

    int err = last_socket_error();
    if (expectingBlock(err)) {
      wouldBlock = true;
    } else {
      return {written, Error::SystemError};
    }
  }
  return {written, Error::Ok};
}

#ifdef WIN32
int fillIoVec(WSABUF out[2], const char* header, size_t headerSize, const char* body, size_t bodySize, size_t written)
{
  if (written < headerSize) {
    out[0].buf = (char*)(header + written);
    out[0].len = headerSize - written;
    out[1].buf = (char*)(body);
    out[1].len = bodySize;
    return 2;
  }
  written -= headerSize;
  out[0].buf = (char*)(body + written);
  out[0].len = bodySize - written;
  return 1;
}
#else
int fillIoVec(iovec out[2], const char* header, size_t headerSize, const char* body, size_t bodySize, size_t written)
{
  if (written < headerSize) {
    out[0].iov_base = (void*)(header + written);
    out[0].iov_len = headerSize - written;
    out[1].iov_base = (void*)(body);
    out[1].iov_len = bodySize;
    return 2;
  }
  written -= headerSize;
  out[0].iov_base = (void*)(body + written);
  out[0].iov_len = bodySize - written;
  return 1;
}
#endif

std::pair<size_t, Error> NetSocket::write2(
  const char* header, size_t headerSize,
  const char* body, size_t bodySize,
  size_t written)
{
#ifdef WIN32
  WSABUF out[2] = {};
#else
  iovec out[2] = {};
#endif

  // Number of blocks to be written.
  int blocks = fillIoVec(out, header, headerSize, body, bodySize, 0);

  bool wouldBlock = false;
  while (written < headerSize + bodySize && !wouldBlock) {
    bool error = false;
    if (written < headerSize) {
#ifdef WIN32
      DWORD n = 0;
      int ret = WSASend(internal_->fd, out, blocks, &n, 0, 0, 0);
      if (ret != 0)
        error = true;
#else
      int n = writev(internal_->fd, out, blocks);
#endif
      if (n > 0) {
        written += n;
        blocks = fillIoVec(out, header, headerSize, body, bodySize, written);
        continue;
      }
      error = true;
    }

    // We are here only if some error has happened.
    int err = last_socket_error();
    if (expectingBlock(err)) {
      wouldBlock = true;
    } else {
      return {written, Error::SystemError};
    }
  }

  return {written, Error::Ok};
}

}
}