//
// Created by dkargin on 3/16/25.
//

#include <cstring>
#include "miniros/transport/io.h" // cross-platform headers needed

#include "miniros/transport/net_address.h"

namespace miniros {
namespace network {


NetAddress::NetAddress()
{
}

NetAddress::NetAddress(NetAddress&& other) noexcept
{
  std::swap(rawAddress, other.rawAddress);
  std::swap(address, other.address);

  port = other.port;
  type = other.type;
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
  if (other.rawAddress && (other.type == Type::AddressIPv4 || other.type == Type::AddressIPv6)) {
    size_t size = getAddressSize(other.type);
    sockaddr_in* addr = static_cast<sockaddr_in*>(malloc(size));
    rawAddress = addr;
    memcpy(rawAddress, other.rawAddress, getAddressSize(other.type));
    type = other.type;
  } else {
    type = Type::AddressInvalid;
  }
  port = other.port;
  address = other.address;
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

  if (other.rawAddress && (other.type == Type::AddressIPv4 || other.type == Type::AddressIPv6)) {
    sockaddr_in* addr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
    rawAddress = addr;
    memcpy(rawAddress, other.rawAddress, getAddressSize(other.type));
    type = other.type;
  } else {
    type = Type::AddressInvalid;
  }
  port = other.port;
  address = other.address;
  return *this;
}

void NetAddress::reset()
{
  if (rawAddress) {
    free(rawAddress);
    rawAddress = nullptr;
  }
  address = "";
  port = 0;
  type = Type::AddressInvalid;
}

bool NetAddress::isLocal() const
{
  return address == "127.0.0.1" || address == "localhost";
}

bool fillAddress(const sockaddr_in& my_addr, NetAddress& address)
{
  address.reset();

  if (my_addr.sin_family == AF_INET) {
    address.type = NetAddress::AddressIPv4;
    address.port = ntohs(my_addr.sin_port);
  } else if (my_addr.sin_family == AF_INET6) {
    address.type = NetAddress::AddressIPv6;
    address.port = ntohs(my_addr.sin_port);
  } else {
    address.type = NetAddress::AddressInvalid;
    return false;
  }

  char ipBuffer[255];
  if (!inet_ntop(my_addr.sin_family, &my_addr.sin_addr, ipBuffer, sizeof(ipBuffer))) {
    address.port = 0;
    return false;
  }
  address.address = ipBuffer;

  size_t size = getAddressSize(address.type);
  sockaddr_in* outAddr = static_cast<sockaddr_in*>(malloc(size));
  address.rawAddress = outAddr;
  memcpy(outAddr, &my_addr, size);
  return true;
}

/// Fills in local address from socket.
bool readLocalAddress(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getsockname(sockfd, (sockaddr*)&my_addr, &len) != 0)
    return false;

  return fillAddress(my_addr, address);
}

/// Fills in remote address from socket.
bool readRemoteAddress(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getpeername(sockfd, (sockaddr*)&my_addr, &len) != 0)
    return false;

  return fillAddress(my_addr, address);
}

std::string NetAddress::str() const
{
  std::stringstream ss;
  ss << address;
  if (port) {
    ss << ":" << port;
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
  if (a.type  == b.type && a.port == b.port) {
    return a.rawAddress && b.rawAddress && memcmp(a.rawAddress, b.rawAddress, getAddressSize(a.type)) == 0;
  }
  return false;
}

bool operator != (const NetAddress& a, const NetAddress& b)
{
  return !(a == b);
}

bool AddressCompare::operator()(const NetAddress& a, const NetAddress& b) const
{
  if (a.type == b.type) {
    return a.address < b.address;
  }
  return a.type < b.type;
}

} // namespace network
} // namespace miniros