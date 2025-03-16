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
  std::swap(rawAddressSize, other.rawAddressSize);
  std::swap(address, other.address);

  port = other.port;
  type = other.type;
}

NetAddress::NetAddress(const NetAddress& other)
{
  if (other.rawAddress && (other.type == Type::AddressIPv4 || other.type == Type::AddressIPv6)) {
    sockaddr_in* addr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
    rawAddress = addr;
    rawAddressSize = other.rawAddressSize;
    memcpy(rawAddress, other.rawAddress, other.rawAddressSize);
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
    rawAddressSize = other.rawAddressSize;
    memcpy(rawAddress, other.rawAddress, other.rawAddressSize);
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

bool fillAddress(const sockaddr_in& my_addr, int len, NetAddress& address)
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

  sockaddr_in* outAddr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
  address.rawAddress = outAddr;
  address.rawAddressSize = len;
  memcpy(outAddr, &my_addr, sizeof(sockaddr_in));
  return true;
}

/// Fills in local address from socket.
bool readLocalAddress(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getsockname(sockfd, (sockaddr*)&my_addr, &len) != 0)
    return false;

  return fillAddress(my_addr, len, address);
}

/// Fills in remote address from socket.
bool readRemoteAddress(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr{};
  socklen_t len = sizeof(my_addr);
  if (getpeername(sockfd, (sockaddr*)&my_addr, &len) != 0)
    return false;

  return fillAddress(my_addr, len, address);
}

} // namespace network
} // namespace miniros