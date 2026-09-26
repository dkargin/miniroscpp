//
// Created by dkargin on 3/16/25.
//

#include "internal_config.h"

#include <cassert>
#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#endif
#include <cstring>

#include "miniros/io/io.h"

#include "resolver.h"

#include "miniros/network/url.h"

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
        it->second->addAddress(adapter.address);
      }
    }
  }
  return Error::Ok;
}

std::string AddressResolver::pickLanIp(const network::NetAddress& peerAddr) const
{
  Lock lock(m_mutex);
  const network::NetAdapter* fallback = nullptr;
  for (const auto& adapter : m_adapters) {
    if (!adapter.isUp() || adapter.isLoopback() || !adapter.isIPv4())
      continue;
    if (peerAddr.valid() && peerAddr.type() == network::NetAddress::AddressIPv4 &&
        adapter.matchNetAddress(peerAddr))
      return adapter.address.address;
    if (!fallback)
      fallback = &adapter;
  }
  return fallback ? fallback->address.address : std::string{};
}

namespace {

void appendUniqueAddress(std::vector<network::NetAddress>& out, const network::NetAddress& addr)
{
  if (!addr.valid() || addr.isUnspecified() || addr.address.empty())
    return;
  for (const network::NetAddress& existing : out) {
    if (existing.type() == addr.type() && existing.address == addr.address)
      return;
  }
  out.push_back(addr);
}

} // namespace

std::vector<network::NetAddress> AddressResolver::listNodeAddresses(const std::shared_ptr<NodeRef>& node) const
{
  std::vector<network::NetAddress> result;
  if (!node)
    return result;

  const network::URL url = node->getUrl();
  const std::shared_ptr<const network::HostInfo> info = node->hostInfo().lock();
  const bool hostLocal = info && info->local;
  if (info) {
    for (const network::NetAddress& addr : info->addresses())
      appendUniqueAddress(result, addr);
  }

  {
    Lock lock(m_mutex);
    if (hostLocal || isLoopbackNameLocked(lock, url.host)) {
      for (const network::NetAdapter& adapter : m_adapters) {
        if (adapter.isUp() && adapter.isValid())
          appendUniqueAddress(result, adapter.address);
      }
    }
  }

  network::NetAddress literal = network::NetAddress::fromIp4String(url.host, 0);
  if (!literal.valid())
    literal = network::NetAddress::fromIp6String(url.host, 0);
  appendUniqueAddress(result, literal);
  return result;
}

network::URL AddressResolver::resolveAddressFor(const std::shared_ptr<NodeRef>& node,
  const network::NetAddress& remoteAddress,
  const network::NetAddress& localAddress) const
{
  assert(node);
  if (!node)
    return {};

  network::URL url = node->getUrl();

  Lock lock(m_mutex);
  if (!m_resolveIp)
    return url;

  auto nodeHost = node->hostInfo().lock();
  if (!nodeHost) {
    MINIROS_WARN_NAMED("resolver", "No HostInfo for node %s", node->id().c_str());
    return url;
  }

  auto requesterHost = findHostLocked(lock, remoteAddress);
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
  if (nodeHost->hasAnyAddress()) {
    // Find first usable address.
    for (const auto& addr: nodeHost->addresses()) {
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
    for (const auto& addr: nodeHost->addresses()) {
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
  Lock lock(m_mutex);
  return findHostLocked(lock, address);
}

std::shared_ptr<network::HostInfo> AddressResolver::findHostLocked(Lock& lock, const network::NetAddress& address) const
{
  assert(lock.owns_lock());
  assert(lock.mutex() == &m_mutex);
  for (const auto& [name, hostPtr] : m_hosts) {
    if (!hostPtr)
      continue;
    if (hostPtr->hasAddress(address))
      return hostPtr;
  }
  return {};
}

bool AddressResolver::isLocalhost(const std::string& host) const
{
  Lock lock(m_mutex);
  return isLoopbackNameLocked(lock, host);
}

bool AddressResolver::isLoopbackNameLocked(Lock& lock, const std::string& host) const
{
  assert(lock.owns_lock());
  assert(lock.mutex() == &m_mutex);
  if (host == m_hostname)
    return true;
  if (host == "localhost")
    return true;
  if (host == "127.0.0.1")
    return true;
  if (host == "0:0:0:0:0:0:0:1" || host == "::1")
    return true;
  auto it = m_hosts.find(host);
  if (it != m_hosts.end() && it->second && it->second->local)
    return true;
  return false;
}

void AddressResolver::setResolveIp(bool resolve)
{
  std::scoped_lock<std::mutex> lock(m_mutex);
  m_resolveIp = resolve;
}

bool AddressResolver::resolveIp() const
{
  std::scoped_lock<std::mutex> lock(m_mutex);
  return m_resolveIp;
}

std::shared_ptr<network::HostInfo> AddressResolver::updateHost(const RequesterInfo& requesterInfo)
{
  if (requesterInfo.callerApi.empty())
    return {};

  network::URL url;
  url.fromString(requesterInfo.callerApi, /*defaultPort*/false);

  const bool sameMachine = requesterInfo.clientAddress.isLoopback();
  Lock lock(m_mutex);

  if (sameMachine) {
    auto it = m_hosts.find("localhost");
    if (it == m_hosts.end())
      return {};
    it->second->local = true;
    if (!url.host.empty())
      m_hosts[url.host] = it->second;
    return it->second;
  }

  std::string key = url.host.empty() ? requesterInfo.clientAddress.address : url.host;
  if (key.empty())
    return {};
  auto it = m_hosts.find(key);
  if (it == m_hosts.end())
    it = m_hosts.emplace(key, std::make_shared<network::HostInfo>(key)).first;
  if (requesterInfo.clientAddress.valid())
    it->second->addAddress(requesterInfo.clientAddress);
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
