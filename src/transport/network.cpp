/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstring>

#include "internal_config.h"

#include "miniros/file_log.h"
#include "miniros/internal/exceptions.h"
#include "miniros/transport/io.h" // cross-platform headers needed
#include "miniros/transport/network.h"
#include <miniros/console.h>
#include <miniros/rosassert.h>

#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#endif

namespace miniros {

namespace network {

std::string g_host;
uint16_t g_tcpros_server_port = 0;

const std::string& getHost()
{
  return g_host;
}

bool splitURI(const std::string& uri, std::string& host, uint32_t& port)
{
  // skip over the protocol if it's there
  if (uri.substr(0, 7) == std::string("http://"))
    host = uri.substr(7);
  else if (uri.substr(0, 9) == std::string("rosrpc://"))
    host = uri.substr(9);
  // split out the port
  std::string::size_type colon_pos = host.find_first_of(":");
  if (colon_pos == std::string::npos)
    return false;
  std::string port_str = host.substr(colon_pos + 1);
  std::string::size_type slash_pos = port_str.find_first_of("/");
  if (slash_pos != std::string::npos)
    port_str = port_str.erase(slash_pos);
  port = atoi(port_str.c_str());
  host = host.erase(colon_pos);
  return true;
}

uint16_t getTCPROSPort()
{
  return g_tcpros_server_port;
}

static bool isPrivateIP(const char* ip)
{
  bool b = !strncmp("192.168", ip, 7) || !strncmp("10.", ip, 3) || !strncmp("169.254", ip, 7);
  return b;
}

std::string determineHost()
{
  std::string ip_env;
  // First, did the user set ROS_HOSTNAME?
  if (get_environment_variable(ip_env, "ROS_HOSTNAME")) {
    MINIROS_DEBUG_NAMED("roscxx", "determineIP: using value of ROS_HOSTNAME:%s:", ip_env.c_str());
    if (ip_env.size() == 0) {
      MINIROS_WARN("invalid ROS_HOSTNAME (an empty string)");
    }
    return ip_env;
  }

  // Second, did the user set ROS_IP?
  if (get_environment_variable(ip_env, "ROS_IP")) {
    MINIROS_DEBUG("determineIP: using value of ROS_IP:%s:", ip_env.c_str());
    if (ip_env.size() == 0) {
      MINIROS_WARN("invalid ROS_IP (an empty string)");
    }
    return ip_env;
  }

  // Third, try the hostname
  char host[1024];
  memset(host, 0, sizeof(host));
  if (gethostname(host, sizeof(host) - 1) != 0) {
    MINIROS_ERROR("determineIP: gethostname failed");
  }
  // We don't want localhost to be our ip
  else if (strlen(host) && strcmp("localhost", host)) {
    return std::string(host);
  }

  // Fourth, fall back on interface search, which will yield an IP address

#ifdef HAVE_IFADDRS_H
  struct ifaddrs *ifa = NULL, *ifp = NULL;
  int rc;
  if ((rc = getifaddrs(&ifp)) < 0) {
    MINIROS_FATAL("error in getifaddrs: [%s]", strerror(rc));
    MINIROS_BREAK();
  }
  char preferred_ip[200] = {0};
  for (ifa = ifp; ifa; ifa = ifa->ifa_next) {
    char ip_[200];
    socklen_t salen;
    if (!ifa->ifa_addr)
      continue; // evidently this interface has no ip address
    if (ifa->ifa_addr->sa_family == AF_INET)
      salen = sizeof(struct sockaddr_in);
    else if (ifa->ifa_addr->sa_family == AF_INET6)
      salen = sizeof(struct sockaddr_in6);
    else
      continue;
    if (getnameinfo(ifa->ifa_addr, salen, ip_, sizeof(ip_), NULL, 0, NI_NUMERICHOST) < 0) {
      MINIROS_DEBUG_NAMED("roscxx", "getnameinfo couldn't get the ip of interface [%s]", ifa->ifa_name);
      continue;
    }
    // ROS_INFO( "ip of interface [%s] is [%s]", ifa->ifa_name, ip);
    //  prefer non-private IPs over private IPs
    if (!strcmp("127.0.0.1", ip_) || strchr(ip_, ':'))
      continue; // ignore loopback unless we have no other choice
    if (ifa->ifa_addr->sa_family == AF_INET6 && !preferred_ip[0])
      strcpy(preferred_ip, ip_);
    else if (isPrivateIP(ip_) && !preferred_ip[0])
      strcpy(preferred_ip, ip_);
    else if (!isPrivateIP(ip_) && (isPrivateIP(preferred_ip) || !preferred_ip[0]))
      strcpy(preferred_ip, ip_);
  }
  freeifaddrs(ifp);
  if (!preferred_ip[0]) {
    MINIROS_ERROR_NAMED("roscxx", "Couldn't find a preferred IP via the getifaddrs() call; I'm assuming that your IP "
                                  "address is 127.0.0.1.  This should work for local processes, "
                                  "but will almost certainly not work if you have remote processes."
                                  "Report to the ROS development team to seek a fix.");
    return std::string("127.0.0.1");
  }
  MINIROS_DEBUG_NAMED("roscxx", "preferred IP is guessed to be %s", preferred_ip);
  return std::string(preferred_ip);
#else
  // @todo Fix IP determination in the case where getifaddrs() isn't
  // available.
  MINIROS_ERROR("You don't have the getifaddrs() call; I'm assuming that your IP "
                "address is 127.0.0.1.  This should work for local processes, "
                "but will almost certainly not work if you have remote processes."
                "Report to the ROS development team to seek a fix.");
  return std::string("127.0.0.1");
#endif
}

void init(const M_string& remappings)
{
  M_string::const_iterator it = remappings.find("__hostname");
  if (it != remappings.end()) {
    g_host = it->second;
  } else {
    it = remappings.find("__ip");
    if (it != remappings.end()) {
      g_host = it->second;
    }
  }

  it = remappings.find("__tcpros_server_port");
  if (it != remappings.end()) {
    bool failed = true;
    try {
      auto rawValue = std::stoul(it->second);
      if (rawValue < 65535) {
        g_tcpros_server_port = rawValue;
        failed = false;
      }
    } catch (std::invalid_argument&) {
    } catch (std::out_of_range&) {
    }

    if (failed) {
      throw miniros::InvalidPortException(
        "__tcpros_server_port [" + it->second + "] was not specified as a number within the 0-65535 range");
    }
  }

  if (g_host.empty()) {
    g_host = determineHost();
  }
}

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

URL::URL()
{
}

bool URL::fromString(const std::string& uri, bool defaultPort)
{
  reset();

  // http://192.156.54.23/:11223/RPC2/request?.......

  constexpr char schemeMarker[] = "://";
  std::string::size_type schemePos = uri.find_first_of(schemeMarker);
  if (schemePos == std::string::npos)
    return false;

  schemePos += sizeof(schemeMarker) - 1;
  scheme = uri.substr(0, schemePos);

  std::string::size_type partPos = uri.find_first_of('/', schemePos);

  // Port is somewhere between hostname and resource part.
  std::string::size_type portPos = uri.find_first_of(':', schemePos);
  if (portPos == std::string::npos) {

  } else {

  }

  host = uri.substr(schemePos);

  // split out the port
  std::string::size_type colon_pos = host.find_first_of(":");
  if (colon_pos == std::string::npos)
    return false;
  std::string port_str = host.substr(colon_pos + 1);
  std::string::size_type slash_pos = port_str.find_first_of("/");
  if (slash_pos != std::string::npos)
    port_str = port_str.erase(slash_pos);
  port = atoi(port_str.c_str());
  host = host.erase(colon_pos);
  return true;
}

void URL::reset()
{
  host = {};
  port = 0;
  path = {};
  scheme = {};
}

bool URL::empty() const
{
  return host.empty();
}

std::string URL::toString() const
{
  std::stringstream ss;
  ss << scheme << host;
  if (port)
    ss << ":" << port;
  if (path.empty())
    ss << "/" << path;
  return ss.str();
}

const char* addressFamilyName(sa_family_t family)
{
  switch (family) {
    case AF_INET: return "AF_INET";
    case AF_INET6: return "AF_INET6";
    case AF_PACKET: return "AF_PACKET";
    default:
      return "????";
  }
}

int listNetworkInterfaces()
{
#ifdef HAVE_IFADDRS_H
  ifaddrs* addressList = nullptr;

  int ret = getifaddrs(&addressList);
  if (ret < 0) {
    return ret;
  }
  if (addressList == nullptr)
    return -1;

  for (const ifaddrs *ifa = addressList; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;

    sa_family_t family = ifa->ifa_addr->sa_family;
    if (family == AF_INET || family == AF_INET6) {
      /* Display interface name and family (including symbolic form of the latter for the common families). */
      printf("%-8s %s (%d)\n", ifa->ifa_name, addressFamilyName(family), family);

      char tmpHost[NI_MAXHOST];
      /* For an AF_INET* interface address, display the address. */
      int s = getnameinfo(ifa->ifa_addr, (family == AF_INET) ? sizeof(struct sockaddr_in) :
                                    sizeof(struct sockaddr_in6),
              tmpHost, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
        printf("getnameinfo() failed: %s\n", gai_strerror(s));
      }

      std::string host = tmpHost;

      /* For an AF_INET* interface address, display the address. */
      s = getnameinfo(ifa->ifa_netmask, (family == AF_INET) ? sizeof(struct sockaddr_in) :
                                    sizeof(struct sockaddr_in6),
              tmpHost, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
        printf("getnameinfo() failed: %s\n", gai_strerror(s));
      }
      std::string mask = tmpHost;
      printf("\t\taddress: <%s> mask: <%s>\n", host.c_str(), mask.c_str());
    }

    /*
    else if (family == AF_PACKET && ifa->ifa_data != NULL) {
      struct rtnl_link_stats *stats = ifa->ifa_data;

      printf("\t\ttx_packets = %10u; rx_packets = %10u\n"
             "\t\ttx_bytes   = %10u; rx_bytes   = %10u\n",
             stats->tx_packets, stats->rx_packets,
             stats->tx_bytes, stats->rx_bytes);
    }*/
  }

  if (addressList)
    freeifaddrs(addressList);
#endif
  return 0;
}

} // namespace network

} // namespace miniros
