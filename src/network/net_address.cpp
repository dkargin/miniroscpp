//
// Created by dkargin on 3/16/25.
//

#include <cassert>
#include <cstring>
#include <sstream>

#include "miniros/io/io.h" // cross-platform headers needed

#include "miniros/network/net_address.h"
#include "miniros/network/url.h"
#include "rosconsole/local_log.h"

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
  std::swap(unspecified, other.unspecified);

  type_ = other.type_;
}

size_t getAddressSize(const NetAddress::Type type)
{
  switch (type) {
    case NetAddress::Type::AddressIPv4:
      return sizeof (sockaddr_in);
    case NetAddress::Type::AddressIPv6:
      return sizeof (sockaddr_in6);
    case NetAddress::AddressInvalid:
    case NetAddress::AddressUnspecified:
    default:
      return 0;
  }
}

NetAddress::NetAddress(const NetAddress& other)
{
  if (other.rawAddress_ && other.rawAddressSize_ > 0) {
    rawAddress_ = malloc(other.rawAddressSize_);
    memcpy(rawAddress_, other.rawAddress_, other.rawAddressSize_);
  }
  type_ = other.type_;
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

  if (other.rawAddress_ && other.rawAddressSize_ > 0) {
    rawAddress_ = malloc(other.rawAddressSize_);
    memcpy(rawAddress_, other.rawAddress_, other.rawAddressSize_);
  }
  type_ = other.type_;
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

Error addressFromString(NetAddress::Type type, const std::string& address, int port, NetAddress& result)
{
  // Port 0 is specifically allowed.
  if (address.empty() || port < 0 || port > 65535) {
    return Error::InvalidValue;
  }

  // First, try to parse as IP address directly
  if (type == NetAddress::AddressIPv4) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    if (inet_pton(AF_INET, address.c_str(), &addr.sin_addr) == 1) {
      addr.sin_port = htons(port);
      result.assignRawAddress(type, &addr, sizeof(addr));
      result.address = address;
      return Error::Ok;
    }
    // Fallback to DNS resolver.
  } else if (type == NetAddress::AddressIPv6) {
    sockaddr_in6 addr{};
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(port);
    if (inet_pton(AF_INET6, address.c_str(), &addr.sin6_addr) == 1) {
      result.assignRawAddress(type, &addr, sizeof(addr));
      result.address = address;
      return Error::Ok;
    }
    // Fallback to DNS resolver.
  } else if (type == NetAddress::AddressUnspecified) {
    // Try IPv6 first, then IPv4
    sockaddr_in6 addr6{};
    addr6.sin6_family = AF_INET6;
    addr6.sin6_port = htons(port);
    if (inet_pton(AF_INET6, address.c_str(), &addr6.sin6_addr) == 1) {
      result.assignRawAddress(NetAddress::AddressIPv6, &addr6, sizeof(addr6));
      result.address = address;
      return Error::Ok;
    }
    sockaddr_in addr4{};
    addr4.sin_family = AF_INET;
    if (inet_pton(AF_INET, address.c_str(), &addr4.sin_addr) == 1) {
      addr4.sin_port = htons(port);
      result.assignRawAddress(NetAddress::AddressIPv4, &addr4, sizeof(addr4));
      result.address = address;
      return Error::Ok;
    }
    // Fallback to DNS resolver.
  } else {
    // Invalid type
    return Error::InvalidValue;
  }

  // If IP parsing failed, try DNS resolution
  struct addrinfo hints{};
  struct addrinfo* res = nullptr;

  // Set up hints based on type preference
  if (type == NetAddress::AddressIPv4) {
    hints.ai_family = AF_INET;
  } else if (type == NetAddress::AddressIPv6) {
    hints.ai_family = AF_INET6;
  } else {
    // AddressUnspecified: prefer IPv6 but allow IPv4 fallback
    hints.ai_family = AF_UNSPEC;
  }
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_ADDRCONFIG;

  std::string portStr = std::to_string(port);

  int err = getaddrinfo(address.c_str(), portStr.c_str(), &hints, &res);

  if (err != 0) {
    // Errors and descriptions are taken from `man getaddrinfo`.
    switch (err) {
#if defined(EAI_SYSTEM)
      case EAI_SYSTEM: {
        const char* sysError = strerror(errno);
        LOCAL_DEBUG("addressFromString(%s) - failed: %s", address.c_str(), sysError);
        return Error::SystemError;
      }
#endif
      case EAI_MEMORY:
        return Error::OutOfMemory;
#if defined(EAI_ADDRFAMILY)
      case EAI_ADDRFAMILY:
#endif
      case EAI_BADFLAGS:
      case EAI_FAMILY:
      case EAI_SERVICE:
      case EAI_SOCKTYPE:
        // Invalid call parameters or unsupported combination.
      case EAI_NONAME:
        // Invalid host name (or unknown host/service).
        return Error::InvalidValue;
      case EAI_AGAIN:
#ifndef WIN32
      // This constant is defined as an alias on Win32 and will not compile.
      case EAI_NODATA:
#endif
      case EAI_FAIL:
        // Name can be valid but is not resolvable right now.
        return Error::AddressIsUnknown;
      default:
        break;
    }
    return Error::SystemError;
  }

  std::unique_ptr<addrinfo, void (*)(addrinfo*)> wrapAddr(res, &freeaddrinfo);

  // Try to find a matching address
  // For AddressUnspecified, prefer IPv4 first, then IPv6. It follows behaviour of old ROS1, 
  // which works mostly with IPv6.
  bool assigned = false;
  if (type == NetAddress::AddressUnspecified) {
    // First pass: look for IPv4 addresses.
    for (struct addrinfo* rp = res; !assigned && rp != nullptr; rp = rp->ai_next) {
      if (rp->ai_family == AF_INET) {
        const sockaddr_in* addr4 = reinterpret_cast<const sockaddr_in*>(rp->ai_addr);
        result.assignRawAddress(NetAddress::AddressIPv4, rp->ai_addr, rp->ai_addrlen);
        char ipBuffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &addr4->sin_addr, ipBuffer, sizeof(ipBuffer))) {
          result.address = ipBuffer;
        }
        assigned = true;
        break;
      }
    }
    // Second pass: look for IPv6 addresses if no IPv4 address found.
    for (struct addrinfo* rp = res; !assigned && rp != nullptr; rp = rp->ai_next) {
      if (rp->ai_family == AF_INET6) {
        const sockaddr_in6* addr6 = reinterpret_cast<const sockaddr_in6*>(rp->ai_addr);
        result.assignRawAddress(NetAddress::AddressIPv6, rp->ai_addr, rp->ai_addrlen);
        char ipBuffer[INET6_ADDRSTRLEN];
        if (inet_ntop(AF_INET6, &addr6->sin6_addr, ipBuffer, sizeof(ipBuffer))) {
          result.address = ipBuffer;
        }
        assigned = true;
        break;
      }
    }
  } else {
    // For specific types, find matching address family
    for (struct addrinfo* rp = res; rp != nullptr; rp = rp->ai_next) {
      if (rp->ai_family == AF_INET && type == NetAddress::AddressIPv4) {
        const sockaddr_in* addr4 = reinterpret_cast<const sockaddr_in*>(rp->ai_addr);
        result.assignRawAddress(NetAddress::AddressIPv4, rp->ai_addr, rp->ai_addrlen);
        char ipBuffer[INET_ADDRSTRLEN];
        if (inet_ntop(AF_INET, &addr4->sin_addr, ipBuffer, sizeof(ipBuffer))) {
          result.address = ipBuffer;
        }
        assigned = true;
        break;
      } else if (rp->ai_family == AF_INET6 && type == NetAddress::AddressIPv6) {
        const sockaddr_in6* addr6 = reinterpret_cast<const sockaddr_in6*>(rp->ai_addr);
        result.assignRawAddress(NetAddress::AddressIPv6, rp->ai_addr, rp->ai_addrlen);
        char ipBuffer[INET6_ADDRSTRLEN];
        if (inet_ntop(AF_INET6, &addr6->sin6_addr, ipBuffer, sizeof(ipBuffer))) {
          result.address = ipBuffer;
        }
        assigned = true;
        break;
      }
    }
  }

  return assigned ? Error::Ok : Error::InvalidValue;
}


NetAddress NetAddress::fromString(Type type, const std::string& address, int port)
{
  NetAddress result;
  Error err = addressFromString(type, address, port, result);
  return result;
}

NetAddress NetAddress::fromURL(const std::string& address, int port)
{
  URL url;
  if (!url.fromString(address, false)) {
    // If URL parsing fails, return invalid address
    return NetAddress();
  }
  return fromURL(url);
}

NetAddress NetAddress::fromURL(const URL& url)
{
  // If host is empty, return invalid address
  if (url.host.empty()) {
    return NetAddress();
  }

  // Use port from URL if available, otherwise return invalid address
  // (since we don't have a default port parameter in this overload)
  if (url.port == 0) {
    return NetAddress();
  }

  int targetPort = static_cast<int>(url.port);

  // Use AddressUnspecified to allow both IPv4 and IPv6, and DNS resolution
  return fromString(AddressUnspecified, url.host, targetPort);
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
    case NetAddress::AddressInvalid:
    case NetAddress::AddressUnspecified:
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

Error fillAddress(const sockaddr* sysAddr, NetAddress& address)
{
  address.reset();
  if (!sysAddr)
    return Error::InvalidAddress;

  char ipBuffer[255] = {};

  const void* addrPtr = nullptr;

  if (sysAddr->sa_family == AF_INET) {
    const sockaddr_in* addr4 = reinterpret_cast<const sockaddr_in*>(sysAddr);
    address.assignRawAddress(NetAddress::AddressIPv4, sysAddr, getAddressSize(NetAddress::Type::AddressIPv4));
    addrPtr = &addr4->sin_addr;
  } else if (sysAddr->sa_family == AF_INET6) {
    const sockaddr_in6* addr6 = reinterpret_cast<const sockaddr_in6*>(sysAddr);
    address.assignRawAddress(NetAddress::AddressIPv6, sysAddr, getAddressSize(NetAddress::Type::AddressIPv6));
    addrPtr = &addr6->sin6_addr;
  } else {
    return Error::InvalidValue;
  }

  if (!addrPtr || !inet_ntop(sysAddr->sa_family, addrPtr, ipBuffer, sizeof(ipBuffer))) {
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

  return fillAddress(&my_addr, address);
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

  return fillAddress(&my_addr, address);
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

std::string NetAddress::lstr() const
{
  std::stringstream ss;
  switch (type_) {
    case Type::AddressIPv4:
      ss << "ip4:";
      break;
    case Type::AddressIPv6:
      ss << "ip6:";
      break;
    case Type::AddressUnspecified:
      ss << "un:";
      break;
    case Type::AddressInvalid:
      ss << "invalid:";
    default:
      break;
  }
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

bool AddressCompareNoPort::operator()(const NetAddress& a, const NetAddress& b) const
{
  if (a.type() == b.type()) {
    return a.address < b.address;
  }
  return a.type() < b.type();
}

} // namespace network
} // namespace miniros