//
// Created by dkargin on 3/16/25.
//

#include "internal_config.h"

#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#endif
#include <cstring>

#include "miniros/transport/io.h"

#include "resolver.h"

namespace miniros {
namespace network {
// Defined in net_address.cpp
bool fillAddress(const sockaddr_in& my_addr, NetAddress& address);
}

namespace master {

const std::string& AddressResolver::getHost() const
{
  std::scoped_lock lock(m_mutex);
  return m_hostname;
}

Error AddressResolver::scanAdapters()
{
  std::scoped_lock lock(m_mutex);
  char host[1024] = {};
  if (gethostname(host, sizeof(host) - 1) != 0) {
    MINIROS_ERROR("determineIP: gethostname failed");
  } else {
    m_hostname = host;
  }

#ifdef HAVE_IFADDRS_H
  ifaddrs* addressList = nullptr;

  int ret = getifaddrs(&addressList);
  if (ret < 0 || addressList == nullptr) {
    return Error::SystemError;
  }

  m_adapters.clear();

  for (ifaddrs *ifa = addressList; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;

    sa_family_t family = ifa->ifa_addr->sa_family;
    if (family == AF_INET || family == AF_INET6) {
      NetAdapter adapter;
      adapter.name = ifa->ifa_name;
      const size_t addrLen = (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
      if (!fillAddress(*reinterpret_cast<sockaddr_in*>(ifa->ifa_addr), adapter.address)) {
        continue;
      }
      if (!fillAddress(*reinterpret_cast<sockaddr_in*>(ifa->ifa_netmask), adapter.mask)) {
        continue;
      }
      // TODO: Probably read additional flags, like DHCP or whatever.
      m_adapters.push_back(adapter);
    }
  } // end for

  if (addressList)
    freeifaddrs(addressList);
#endif
  return Error::Ok ;
}

std::string AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node,
  const network::NetAddress& remoteAddress,
  const network::NetAddress& localAddress) const
{
  if (!m_resolveIp)
    return node->getApi();

  std::string host = node->getHost();
  if (isLocalhost(host)) {
    // If remote is also belongs to this host, then maybe it is ok to return default API.
  }
  // TODO: Check if node is located on a localhost:
  // 1. Its hostname is equal to current address and we had a connection to this node, which used localhost adapter.

  return node->getApi();
}

std::string AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node, const std::shared_ptr<NodeRef>& requester) const
{
  if (!m_resolveIp)
    return node->getApi();

  std::string host = node->getHost();
  if (isLocalhost(host)) {
    // If remote is also belongs to this host, then maybe it is ok to return default API.
  }

  return node->getApi();
}

bool AddressResolver::isLocalhost(const std::string& host) const
{
  if (host == m_hostname)
    return true;
  if (host == "localhost")
    return true;
  if (host == "127.0.0.1")
    return true;
  return false;
}

void AddressResolver::setResolveIp(bool resolve)
{
  std::scoped_lock<std::mutex> lock(m_mutex);
  m_resolveIp = resolve;
}

} // namespace master
} // namespace miniros
