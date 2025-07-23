//
// Created by dkargin on 7/19/25.
//

#include <cassert>
#include <cstring>

#include "../../include/miniros/network/socket.h"
#include "miniros/transport/io.h"

#include "transport/transport_tcp.h"
#include "xmlrpcpp/XmlRpcSocket.h"

#ifndef WIN32
#include <sys/uio.h>
#endif

#define MINIROS_PACKAGE_NAME "net_socket"

namespace miniros {
namespace network {

struct NetSocket::Internal {
  Type type = Type::Invalid;
  socket_fd_t fd = MINIROS_INVALID_SOCKET;
  /// Do we own the FD and close it on exit.
  bool own = false;

  /// Socket is listening for connections.
  bool listening = false;

  /// Address of connected endpoint.
  NetAddress peer_address;

  /// Close socket and reset all data.
  void close()
  {
    if (own && fd != MINIROS_INVALID_SOCKET) {
      close_socket(fd);
      fd = MINIROS_INVALID_SOCKET;
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

NetSocket::NetSocket(int fd, Type type, bool own)
{
  internal_.reset(new Internal());
  internal_->fd = fd;
  internal_->own = own;
  internal_->type = type;
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

Error NetSocket::bind(int port)
{
  assert(internal_);
  if (!internal_)
    return Error::InternalError;
  sockaddr_storage ss;
  socklen_t ss_len = 0;
  memset(&ss, 0, sizeof(ss));

  if (isIpv6())
  {
    sockaddr_in6 *address = reinterpret_cast<sockaddr_in6*>(&ss);
    ss_len = sizeof(sockaddr_in6);

    address->sin6_family = AF_INET6;
    address->sin6_addr = in6addr_any;
    address->sin6_port = htons(static_cast<uint16_t>(port));
  }
  else if (isIpv4())
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
    MINIROS_WARN("Failed to bind fd=%d to port=%d : %s", internal_->fd, port, strerror(errno));
    return Error::SystemError;
  }

  readLocalAddress(internal_->fd, internal_->peer_address);

  return Error::Ok;
}

int NetSocket::port() const
{
  return internal_ ? internal_->peer_address.port() : 0;
}

bool NetSocket::valid() const
{
  return internal_ && internal_->fd != MINIROS_INVALID_SOCKET;
}

int NetSocket::fd() const
{
  return internal_ ? internal_->fd : MINIROS_INVALID_SOCKET;
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
  else {
    return Error::InvalidValue;
  }
  if (!addrType)
    return Error::InvalidValue;
  int fd = socket(addrType, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("Error while trying to create listening socket: %s", err);
    return Error::SystemError;
  }
  internal_->fd = fd;

  if (type == NetAddress::AddressIPv4)
    internal_->type = Type::TCP;
  else if (type == NetAddress::AddressIPv6)
    internal_->type = Type::TCPv6;

  if (Error err = setReuseAddr(); !err) {
    return err;
  }

  if (Error err = bind(port); !err) {
    MINIROS_ERROR("Error while binding socket to port %d: %s", port, err.toString());
    return err;
  }
  if (Error err = listen(maxQueuedClients); !err) {
    MINIROS_ERROR("Error while entering listening mode: %s", err.toString());
    return err;
  }
  return Error::Ok;
}

Error NetSocket::initUDP(bool ipv6)
{
  if (valid()) {
    close();
  }
  //
  //PF_INET
  int domain = ipv6 ? PF_INET6 : PF_INET;
  int fd = socket(domain, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    /*
       EACCES Permission to create a socket of the specified type and/or protocol is denied.
       EAFNOSUPPORT
              The implementation does not support the specified address family.
       EINVAL Unknown protocol, or protocol family not available.
       EINVAL Invalid flags in type.
       EMFILE The per-process limit on the number of open file descriptors has been reached.
       ENFILE The system-wide limit on the total number of open files has been reached.
       ENOBUFS or ENOMEM
              Insufficient memory is available.  The socket cannot be created until sufficient resources are freed.
       EPROTONOSUPPORT
              The protocol type or the specified protocol is not supported within this domain.
     */
    switch (errno) {
      case EMFILE:
      case ENOMEM:
      case ENOBUFS:
        return Error::OutOfMemory;
      case EAFNOSUPPORT:
      case EPROTONOSUPPORT:
        return Error::NotSupported;
      case EINVAL:
        break;
      default:
        break;
    }
    return Error::SystemError;
  }
  internal_->fd = fd;
  internal_->type = ipv6 ? Type::UDPv6 : Type::UDP;
  return Error::Ok;
}

std::pair<std::shared_ptr<NetSocket>, Error> NetSocket::accept()
{
  if (!internal_)
    return {nullptr, Error::InternalError};
  if (internal_->fd < 0)
    return {nullptr, Error::InvalidHandle};

  sockaddr addr{};
  socklen_t addrLen = sizeof(addr);
  // accept will truncate the address if the buffer is too small.
  // As we are not using it, no special case for IPv6
  // has to be made.
  int fd = ::accept(internal_->fd, &addr, &addrLen);
  if (fd < 0) {
    // TODO: decode errno.
    return {{}, Error::SystemError};
  }

  std::shared_ptr<NetSocket> sock(new NetSocket(fd, internal_->type, true));
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
  if (!internal_)
    return Error::InternalError;
  if (internal_->fd < 0)
    return Error::InvalidHandle;

  int result = set_non_blocking(internal_->fd);
  if ( result != 0 ) {
    MINIROS_ERROR("setting socket [%d] as non_blocking failed with error [%d]", internal_->fd, result);
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setNoDelay(bool noDelay)
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->fd < 0)
    return Error::InvalidHandle;

  int flag = noDelay ? 1 : 0;
  int result = setsockopt(internal_->fd, IPPROTO_TCP, TCP_NODELAY, (char *) &flag, sizeof(int));
  if (result < 0)
  {
    MINIROS_ERROR("setsockopt failed to set TCP_NODELAY on socket [%d]: %s", internal_->fd, last_socket_error_string());
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setBroadcast(bool broadcast)
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->fd < 0)
    return Error::InvalidHandle;

  int flag = broadcast ? 1 : 0;
  if (setsockopt(internal_->fd, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char*>(&flag), sizeof(flag)) < 0)
  {
    MINIROS_ERROR("NetSocket::setBroadcast failed to set SO_BROADCAST on socket [%d]: %s", internal_->fd, last_socket_error_string());
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setReuseAddr(bool reuse)
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->fd < 0)
    return Error::InvalidHandle;

  // Allow this port to be re-bound immediately so server re-starts are not delayed
  int flag = reuse ? 1 : 0;
  if (setsockopt(internal_->fd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&flag), sizeof(flag)) != 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("NetSocket::setReuseAddr() setting socket [%d] as reuse_addr failed with error: %s", internal_->fd, err);
    return Error::SystemError;
  }
  return Error::Ok;
}

Error NetSocket::setReusePort(bool reuse)
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->fd < 0)
    return Error::InvalidHandle;

#ifdef SO_REUSEPORT
  // Allow this port to be re-bound immediately so server re-starts are not delayed
  int flag = reuse ? 1 : 0;
  if (setsockopt(internal_->fd, SOL_SOCKET, SO_REUSEPORT, &flag, sizeof(flag)) != 0) {
    const char* err = last_socket_error_string();
    MINIROS_ERROR("NetSocket::setReusePort() socket [%d] as reuse_addr failed with error: %s", internal_->fd, err);
    return Error::SystemError;
  }
#endif
  return Error::Ok;
}

bool expectingBlock(int err)
{
  return err == EINPROGRESS || err == EAGAIN || err == EWOULDBLOCK || err == EINTR;
}

// Read available text from the specified socket. Returns false on error.
std::pair<size_t, Error> NetSocket::recv(WriteBuf& s, NetAddress* address)
{
  if (!internal_) {
    return {0, Error::InternalError};
  }
  if (internal_->fd < 0) {
    return {0, Error::InvalidHandle};
  }

  constexpr int READ_SIZE = 4096;   // Number of bytes to attempt to read at a time
  char readBuf[READ_SIZE];

  size_t received = 0;
  bool wouldBlock = false;

  while ( !wouldBlock) {
    sockaddr peerAddress;
    socklen_t peerAddressLength = sizeof(peerAddress);
    std::memset(&peerAddress, 0, sizeof(peerAddress));

    int n = ::recvfrom(internal_->fd, readBuf, READ_SIZE-1, 0, &peerAddress, &peerAddressLength);
    if (n > 0) {
      readBuf[n] = 0;
      s.append(readBuf, n);
      received += n;
      if (address) {
        fillAddress(peerAddress, *address);
      }
      // Receive only one packet with datagram socket.
      if (isDatagram())
        break;
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

std::pair<size_t, Error> NetSocket::send(const void* rawData, size_t size, const NetAddress* address)
{
  if (!internal_) {
    return {0, Error::InternalError};
  }
  if (internal_->fd < 0) {
    return {0, Error::InvalidHandle};
  }
  const char* sp = static_cast<const char*>(rawData);
  size_t written = 0;
  bool wouldBlock = false;

  const sockaddr* rawAddr = address ? static_cast<const sockaddr*>(address->rawAddress()) : nullptr;

  while ( written < size && ! wouldBlock ) {
    int n = 0;
    constexpr int flags = 0;
    if (isDatagram() && address) {
      int port = address->port();
      if (!port) {
        return {0, Error::InvalidAddress};
      }
      size_t rawAddrSize = address->rawAddressSize();
      n = ::sendto(internal_->fd, sp, static_cast<socklen_t>(size), flags, rawAddr, rawAddrSize);
    } else {
      n = ::send(internal_->fd, sp, static_cast<socklen_t>(size - written), flags);
    }
    MINIROS_DEBUG("NetSocket::send returned %d.", n);
    if (n > 0) {
      if (isDatagram()) {
        break;
      }
      sp += n;
      written += n;
      continue;
    }

    int err = last_socket_error();
    if (expectingBlock(err)) {
      wouldBlock = true;
    } else {
      MINIROS_WARN("Error while sending: %s", strerror(errno));
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
    out[0].len = static_cast<socklen_t>(headerSize - written);
    out[1].buf = (char*)(body);
    out[1].len = static_cast<socklen_t>(bodySize);
    return 2;
  }
  written -= headerSize;
  out[0].buf = (char*)(body + written);
  out[0].len = static_cast<socklen_t>(bodySize - written);
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

bool NetSocket::isDatagram() const
{
  if (!internal_)
    return false;
  const Type t = internal_->type;
  return t == Type::UDP || t == Type::UDPv6;
}

bool NetSocket::isStream() const
{
  if (!internal_)
    return false;
  const Type t = internal_->type;
  return t == Type::TCP || t == Type::TCPv6 || t == Type::Unix;
}

bool NetSocket::isIpv4() const
{
  if (!internal_)
    return false;
  const Type t = internal_->type;
  return t == Type::TCP || t == Type::UDP;
}

bool NetSocket::isIpv6() const
{
  if (!internal_)
    return false;
  const Type t = internal_->type;
  return t == Type::TCPv6 || t == Type::UDPv6;
}

}
}