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
      it = m_hosts.emplace(host, std::make_shared<network::HostInfo>(host)).first;
      it->second->local = true;
    }
    m_hosts["localhost"] = it->second;
  }

  Error err = network::scanAdapters(m_adapters);
  if (!err) {
    MINIROS_WARN("ScanAdapters has failed with err=%s", err.toString());
  }

  if (!m_adapters.empty()) {
    auto it = m_hosts.find("localhost");
    assert(it != m_hosts.end());
    if (it != m_hosts.end()) {
      for (const auto& adapter: m_adapters) {
        it->second->addresses.insert(adapter.address);
      }
    }
  }
  return Error::Ok;
}

network::URL AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node,
  const network::NetAddress& remoteAddress,
  const network::NetAddress& localAddress) const
{
  assert(node);
  if (!node)
    return {};

  network::URL url = node->getUrl();

  std::scoped_lock lock(m_mutex);
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
      if (!addr.isLoopback()) {
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

  assert(requester);
  if (!requester) {
    MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - requester is null", node->id().c_str());
    return url;
  }

  std::scoped_lock lock(m_mutex);
  if (!m_resolveIp)
    return url;

  auto nodeHost = node->hostInfo().lock();
  if (!nodeHost) {
    MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - no HostInfo", node->id().c_str());
    return url;
  }

  auto requesterHost = requester->hostInfo().lock();
  if (!requesterHost) {
    MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - no HostInfo for requester %s", node->id().c_str(), requester->id().c_str());
    return url;
  }

  if (requesterHost && nodeHost == requesterHost) {
    // Both requester and node are on the same host. No additional resolution is needed.
    return url;
  }

  if (nodeHost->local) {
    for (const network::NetAdapter& adapter: m_adapters) {
      if (adapter.hasAccessTo(*requesterHost)) {
        url.host = adapter.address.str();
        return url;
      }
    }
    MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - no adapter matched for %s", node->id().c_str(), requester->id().c_str());
  } else {
    // Just return any address. That will fork for simple networks.
    for (const auto& addr: nodeHost->addresses) {
      if (!addr.isLoopback()) {
        url.host = addr.str();
        return url;
      }
    }
    MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - no suitable host address found for node %s", node->id().c_str(), requester->id().c_str());
  }

  MINIROS_WARN_NAMED("resolver", "resolveAddressFor(%s) - failed to resolve address for %s", node->id().c_str(), requester->id().c_str());
  return url;
}

std::shared_ptr<network::HostInfo> AddressResolver::findHost(const network::NetAddress& address) const
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
  auto it = m_hosts.find(host);
  if (it != m_hosts.end() && it->second->local) {
    return true;
  }
  return false;
}

void AddressResolver::setResolveIp(bool resolve)
{
  std::scoped_lock<std::mutex> lock(m_mutex);
  m_resolveIp = resolve;
}

std::shared_ptr<network::HostInfo> AddressResolver::updateHost(const  RequesterInfo& requesterInfo)
{
  if (requesterInfo.callerApi.empty())
    return {};

  network::URL url;
  url.fromString(requesterInfo.callerApi, /*defaultPort*/false);

  // Host can contain either hostname or direct IP address.
  // We should try to make sure that m_hosts[host] points to the same object for both hostname and IP.

  // check if we got a direct IP address instead of a string hostname.
  bool isIP = network::NetAddress::checkAddressType(url.host) != network::NetAddress::AddressInvalid;
  bool isSameMachine = requesterInfo.clientAddress.isLoopback();

  std::scoped_lock lock(m_mutex);

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
    it = m_hosts.emplace(url.host, std::make_shared<network::HostInfo>(url.host)).first;
  }

  if (!isSameMachine)
    it->second->addresses.insert((requesterInfo.clientAddress));
  else
    it->second->local = true;

  return it->second;
}

std::set<std::shared_ptr<network::HostInfo>> AddressResolver::getHosts() const
{
  std::scoped_lock lock(m_mutex);
  std::set<std::shared_ptr<network::HostInfo>> result;
  for (auto [key, pInfo]: m_hosts) {
    result.insert(pInfo);
  }
  return result;
}

} // namespace master
} // namespace miniros
