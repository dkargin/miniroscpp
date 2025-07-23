//
// Created by dkargin on 3/16/25.
//

#include <cassert>
#include <cstring>

#include "miniros/transport/io.h" // cross-platform headers needed

#include "miniros/network/net_address.h"

namespace miniros {
namespace network {


NetAddress::NetAddress()
{
}

NetAddress::NetAddress(NetAddress&& other) noexcept
{
  std::swap(rawAddress_, other.rawAddress_);
  std::swap(rawAddressSize_, other.rawAddressSize_);
  std::swap(address, other.address);

  type_ = other.type_;
}

size_t getAddressSize(const NetAddress::Type type)
{
  switch (type) {
    case NetAddress::Type::AddressIPv4:
      return sizeof (sockaddr_in);
    case NetAddress::Type::AddressIPv6:
      return sizeof (sockaddr_in6);
    default:
      return 0;
  }
}

NetAddress::NetAddress(const NetAddress& other)
{
  if (other.rawAddress_ && (other.type_ == Type::AddressIPv4 || other.type_ == Type::AddressIPv6)) {
    size_t size = getAddressSize(other.type_);
    sockaddr_in* addr = static_cast<sockaddr_in*>(malloc(size));
    rawAddress_ = addr;
    memcpy(rawAddress_, other.rawAddress_, getAddressSize(other.type_));
    type_ = other.type_;
  } else {
    type_ = Type::AddressInvalid;
  }
  address = other.address;
  unspecified = other.unspecified;
  rawAddressSize_ = other.rawAddressSize_;
}

NetAddress::~NetAddress()
{
  reset();
}

NetAddress& NetAddress::operator=(const NetAddress& other)
{
  if (this == &other)
    return *this;
  reset();

  if (other.rawAddress_ && (other.type_ == Type::AddressIPv4 || other.type_ == Type::AddressIPv6)) {
    sockaddr_in* addr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
    rawAddress_ = addr;
    memcpy(rawAddress_, other.rawAddress_, getAddressSize(other.type_));
    type_ = other.type_;
  } else {
    type_ = Type::AddressInvalid;
  }
  address = other.address;
  unspecified = other.unspecified;
  rawAddressSize_ = other.rawAddressSize_;
  return *this;
}

void NetAddress::reset()
{
  if (rawAddress_) {
    free(rawAddress_);
    rawAddress_ = nullptr;
  }
  address = "";
  type_ = Type::AddressInvalid;
}

bool NetAddress::isLoopback() const
{
  return address == "127.0.0.1" || address == "localhost" || address == "::1";
}

bool NetAddress::isUnspecified() const
{
  return unspecified;
}

NetAddress NetAddress::fromString(Type type, const std::string& address, int port)
{
  NetAddress result;

  if (type == NetAddress::AddressIPv4) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    if (!inet_pton(AF_INET, address.c_str(), &addr.sin_addr)) {
      return result;
    }
    result.assignRawAddress(type, &addr, sizeof(addr));
    result.setPort(port);
  } else if (type == NetAddress::AddressIPv6) {
    sockaddr_in6 addr{};
    addr.sin6_family = AF_INET6;
    if (!inet_pton(AF_INET6, address.c_str(), &addr.sin6_addr)) {
      return result;
    }
    result.assignRawAddress(type, &addr, sizeof(addr));
    result.setPort(port);
  }

  return result;
}

Error NetAddress::setPort(int port)
{
  if (type_ == Type::AddressInvalid)
    return Error::InvalidValue;

  if (!rawAddress_)
    return Error::InternalError;

  switch (type_) {
    case Type::AddressIPv4:
      static_cast<sockaddr_in*>(rawAddress_)->sin_port = htons(port);
      break;
    case Type::AddressIPv6:
      static_cast<sockaddr_in6*>(rawAddress_)->sin6_port = htons(port);
      break;
    default:
      break;
  }
  return Error::Ok;
}

void NetAddress::assignRawAddress(Type type, const void* addr, size_t size)
{
  assert(size);
  if (!size)
    return;
  if (rawAddress_) {
    free(rawAddress_);
    rawAddress_ = nullptr;
    rawAddressSize_ = 0;
  }
  rawAddress_ = malloc(size);
  memcpy(rawAddress_, addr, size);
  rawAddressSize_ = size;
  type_ = type;
}

Error fillAddress(const sockaddr& sysAddr, NetAddress& address)
{
  address.reset();

  char ipBuffer[255] = {};

  const void* addrPtr = nullptr;

  if (sysAddr.sa_family == AF_INET) {
    const sockaddr_in* addr4 = reinterpret_cast<const sockaddr_in*>(&sysAddr);
    address.assignRawAddress(NetAddress::AddressIPv4, &sysAddr, getAddressSize(NetAddress::Type::AddressIPv4));
    address.setPort(ntohs(addr4->sin_port));
    addrPtr = &addr4->sin_addr;
  } else if (sysAddr.sa_family == AF_INET6) {
    const sockaddr_in6* addr6 = reinterpret_cast<const sockaddr_in6*>(&sysAddr);
    address.assignRawAddress(NetAddress::AddressIPv6, &sysAddr, getAddressSize(NetAddress::Type::AddressIPv6));
    address.setPort(ntohs(addr6->sin6_port));
    addrPtr = &addr6->sin6_addr;
  } else {
    return Error::InvalidValue;
  }

  if (!addrPtr || !inet_ntop(sysAddr.sa_family, addrPtr, ipBuffer, sizeof(ipBuffer))) {
    address.reset();
    return Error::InvalidValue;
  }

  address.address = ipBuffer;
  // Just drop unspecified address
  if (address.address == "::" || address.address == "::0"
    || address.address == "0.0.0.0" || address.address == "0:0:0:0:0:0:0:0")
  {
    address.unspecified = true;
  }

  return Error::Ok;
}

Error fillBroadcastAddress(NetAddress& address, int port, NetAddress::Type type)
{
  address.reset();

  if (type == NetAddress::Type::AddressIPv4) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = INADDR_BROADCAST;
    address.address = "0.0.0.0";
    address.assignRawAddress(NetAddress::AddressIPv4, &addr, sizeof(addr));
  } else if (type == NetAddress::Type::AddressIPv6) {
    // TODO: write proper multicast address
    sockaddr_in6 addr{};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port);
    //addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    address.address = "0.0.0.0";
    address.assignRawAddress(NetAddress::AddressIPv6, &addr, sizeof(addr));
  } else {
    return Error::InvalidValue;
  }

  address.unspecified = true;
  address.setPort(port);
  return Error::Ok;
}

Error readLocalAddress(int sockfd, NetAddress& address)
{
  sockaddr my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getsockname(sockfd, &my_addr, &len) != 0) {
    /*
      Possible errors from man 7 getsockname:
      EBADF  The argument sockfd is not a valid file descriptor.
      EFAULT The addr argument points to memory not in a valid part of the process address space.
      EINVAL addrlen is invalid (e.g., is negative).
      ENOBUFS Insufficient resources were available in the system to perform the operation.
      ENOTSOCK The file descriptor sockfd does not refer to a socket.
    */
    switch (errno) {
      case EBADF:
      case EFAULT:
      case EINVAL:
      case ENOTSOCK:
        return Error::InvalidValue;
      case ENOBUFS:
        return Error::OutOfMemory;
      default:
        break;
    }
    return Error::SystemError;
  }

  return fillAddress(my_addr, address);
}

Error readRemoteAddress(int sockfd, NetAddress& address)
{
  sockaddr my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getpeername(sockfd, &my_addr, &len) != 0) {
    /* Possible errors from man 7 getpeername:
       EBADF  The argument sockfd is not a valid file descriptor.
       EFAULT The addr argument points to memory not in a valid part of the process address space.
       EINVAL addrlen is invalid (e.g., is negative).
       ENOBUFS Insufficient resources were available in the system to perform the operation.
       ENOTCONN The socket is not connected.
       ENOTSOCK The file descriptor sockfd does not refer to a socket.
     */
    switch (errno) {
      case ENOBUFS:
        return Error::OutOfMemory;
      case ENOTCONN:
        return Error::NotConnected;
      default:
        break;
    }
    return Error::InvalidValue;
  }

  return fillAddress(my_addr, address);
}

int NetAddress::port() const
{
  if (!rawAddress_)
    return 0;
  if (type_ == Type::AddressIPv4) {
    const sockaddr_in* addr = static_cast<const sockaddr_in*>(rawAddress_);
    return ntohs(addr->sin_port);
  }

  if (type_ == Type::AddressIPv6) {
    const sockaddr_in6* addr = static_cast<const sockaddr_in6*>(rawAddress_);
    return ntohs(addr->sin6_port);
  }
  return 0;
}


std::string NetAddress::str() const
{
  std::stringstream ss;
  ss << address;
  int p = port();
  if (p) {
    ss << ":" << p;
  }
  return ss.str();
}

NetAddress::Type NetAddress::checkAddressType(const std::string& address)
{
  sockaddr_in sa = {};
  if (inet_pton(AF_INET, address.c_str(), &sa.sin_addr) == 1) {
    return Type::AddressIPv4;
  }
  if (inet_pton(AF_INET6, address.c_str(), &sa.sin_addr) == 1) {
    return Type::AddressIPv6;
  }
  return Type::AddressInvalid;
}

bool operator < (const NetAddress& a, const NetAddress& b)
{
  return a.str() < b.str();
}

bool operator == (const NetAddress& a, const NetAddress& b)
{
  if (a.type_  != b.type_)
    return false;
  return a.rawAddress_ && b.rawAddress_ && a.rawAddressSize_ == b.rawAddressSize_
      && memcmp(a.rawAddress_, b.rawAddress_, getAddressSize(a.type_)) == 0;
}

bool operator != (const NetAddress& a, const NetAddress& b)
{
  return !(a == b);
}

bool AddressCompare::operator()(const NetAddress& a, const NetAddress& b) const
{
  if (a.type() == b.type()) {
    return a.address < b.address;
  }
  return a.type() < b.type();
}

} // namespace network
} // namespace miniros