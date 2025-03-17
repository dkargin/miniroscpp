//
// Created by dkargin on 3/16/25.
//

#include "internal_config.h"

#include <cassert>
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
    if (!m_hostname.empty() && m_hostname != host) {
      MINIROS_WARN("Observed change in the hostname");
      // TODO: Handle change of a hostname.
    }
    m_hostname = host;

    // Generate localhost object.
    auto it = m_hosts.find(host);
    if (it == m_hosts.end()) {
      it = m_hosts.emplace(host, std::make_shared<HostInfo>(host)).first;
      it->second->local = true;
    }
    m_hosts["localhost"] = it->second;
  }

#ifdef HAVE_IFADDRS_H
  // TODO: subscribe to updates on network interfaces.
  // NETLINK kernel interface can be used to subscribe to notification socket. But it will
  // require some integration with a local PollSet interface. Meanwhile, windows will need something very different.
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
  }

  if (addressList)
    freeifaddrs(addressList);
#endif

  if (!m_adapters.empty()) {
    auto it = m_hosts.find("localhost");
    assert(it != m_hosts.end());
    if (it != m_hosts.end()) {
      for (const auto& adapter: m_adapters) {
        it->second->addresses.insert(adapter.address);
      }
    }
  }
  return Error::Ok ;
}

network::URL AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node,
  const network::NetAddress& remoteAddress,
  const network::NetAddress& localAddress) const
{
  assert(node);
  if (!node)
    return {};

  network::URL url = node->getUrl();
  if (!m_resolveIp)
    return url;

  auto nodeHost = node->hostInfo().lock();
  if (!nodeHost) {
    MINIROS_WARN_NAMED("resolver", "No HostInfo for node %s", node->id().c_str());
    return url;
  }

  auto requesterHost = findHost(remoteAddress);
  if (requesterHost && nodeHost == requesterHost) {
    // Both requester and node are on the same host. No additional resolution is needed.
    return url;
  }

  if (nodeHost->local) {
    // Node located on this host. We can use localAddress as a good IP for a requester.
    url.host = localAddress.address;
    return url;
  }

  // Here both node and requester are located on different machines and node is not located on a local host.
  if (!nodeHost->addresses.empty()) {
    // Find first usable address.
    for (const auto& addr: nodeHost->addresses) {
      if (!addr.isLocal()) {
        url.host = addr.str();
        break;
      }
    }
  }
  return url;
}

network::URL AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node, const std::shared_ptr<NodeRef>& requester) const
{
  assert(node);
  if (!node)
    return {};

  network::URL url = node->getUrl();
  if (!m_resolveIp)
    return url;

  auto nodeHost = node->hostInfo().lock();
  if (!nodeHost) {
    MINIROS_WARN_NAMED("resolver", "No HostInfo for node %s", node->id().c_str());
    return url;
  }

  auto requesterHost = requester->hostInfo().lock();
  if (requesterHost && nodeHost == requesterHost) {
    // Both requester and node are on the same host. No additional resolution is needed.
    return url;
  }

  return url;
}

std::shared_ptr<HostInfo> AddressResolver::findHost(const network::NetAddress& address) const
{
  for (const auto& [name, hostPtr] : m_hosts) {
    if (!hostPtr) {
      continue;
    }
    if (hostPtr->addresses.find(address) != hostPtr->addresses.end()) {
      return hostPtr;
    }
  }
  return {};
}

bool AddressResolver::isLocalhost(const std::string& host) const
{
  if (host == m_hostname)
    return true;
  if (host == "localhost")
    return true;
  if (host == "127.0.0.1")
    return true;
  if (host == "0:0:0:0:0:0:0:1"  || host == "::1") // IP v6 address
    return true;
  return false;
}

void AddressResolver::setResolveIp(bool resolve)
{
  std::scoped_lock<std::mutex> lock(m_mutex);
  m_resolveIp = resolve;
}

std::shared_ptr<HostInfo> AddressResolver::updateHost(const  RequesterInfo& requesterInfo)
{
  if (requesterInfo.callerApi.empty())
    return {};
  network::URL url;
  url.fromString(requesterInfo.callerApi, /*defaultPort*/false);

  // Host can contain either hostname or direct IP address.
  // We should try to make sure that m_hosts[host] points to the same object for both hostname and IP.

  // check if we got a direct IP address instead of a string hostname.
  bool isIP = network::NetAddress::checkAddressType(url.host) != network::NetAddress::AddressInvalid;
  bool isSameMachine = requesterInfo.clientAddress.isLocal();

  if (isIP) {
    if (isSameMachine) {
      auto it = m_hosts.find("localhost");
      if (it != m_hosts.end())
        return it->second;
    }
    // TODO: find host which corresponds to client IP.
    return {};
  }

  auto it = m_hosts.find(url.host);
  if (it == m_hosts.end()) {
    // Avahi or similar DNS services can provide an alias for some host. Suppose we have some host="rpi.robot". Avahi
    // will provide DNS for "rpi.robot.local" hostname. Some users can force usage of Avahi hostname by specifying
    // ROS_HOSTNAME=rpi.robot.local. We should make sure this alias is points to the same HostInfo object.

    // In the same time, we should be ready that some hosts can have their DHCP ip addresses reassigned.
    // TODO: Check if this IP is already used.
    it = m_hosts.emplace(url.host, std::make_shared<HostInfo>(url.host)).first;
  }

  if (!isSameMachine)
    it->second->addresses.insert((requesterInfo.clientAddress));
  else
    it->second->local = true;

  return it->second;
}

HostInfo::HostInfo(const std::string& name)
  :hostname(name)
{}


} // namespace master
} // namespace miniros
