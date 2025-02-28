//
// Created by dkargin on 2/28/25.
//

#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "miniros/transport/http_tools.h"

namespace miniros {
namespace net {

NetAddress::~NetAddress()
{
  reset();
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

/// Fills in local address from socket.
bool readLocalAddressv4(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr;
  // Get my ip address and port
  memset(&my_addr, 0, sizeof(my_addr));
  socklen_t len = sizeof(my_addr);
  if (getsockname(sockfd, (sockaddr*) &my_addr, &len) != 0)
    return false;

  char ipBuffer[255];
  if (!inet_ntop(AF_INET, &my_addr.sin_addr, ipBuffer, sizeof(ipBuffer)))
    return false;
  address.address = ipBuffer;
  address.port = ntohs(my_addr.sin_port);
  address.type = NetAddress::AddressIPv4;
  sockaddr_in* outAddr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
  address.rawAddress = outAddr;
  memcpy(outAddr, &my_addr, sizeof(sockaddr_in));
  return true;
}

/// Fills in remote address from socket.
bool readRemoteAddressv4(int sockfd, NetAddress& address)
{
  sockaddr_in my_addr;
  // Get my ip address and port
  memset(&my_addr, 0, sizeof(my_addr));
  socklen_t len = sizeof(my_addr);
  if (getpeername(sockfd, (sockaddr*) &my_addr, &len) != 0)
    return false;

  char ipBuffer[255];
  if (!inet_ntop(AF_INET, &my_addr.sin_addr, ipBuffer, sizeof(ipBuffer)))
    return false;
  address.address = ipBuffer;
  address.port = ntohs(my_addr.sin_port);
  address.type = NetAddress::AddressIPv4;
  sockaddr_in* outAddr = static_cast<sockaddr_in*>(malloc(sizeof(sockaddr_in)));
  address.rawAddress = outAddr;
  memcpy(outAddr, &my_addr, sizeof(sockaddr_in));
  return true;
}


void HttpFrame::reset()
{
  header = "";
  request = "";
  fields.clear();
  requestType = {};
  requestUrl = {};
  requestHttpVersion = {};
}

} // namespace net
} // namespace miniros
