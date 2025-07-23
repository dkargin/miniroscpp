//
// Created by dkargin on 3/16/25.
//

#ifndef MINIROS_RESOLVER_H
#define MINIROS_RESOLVER_H

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "miniros/errors.h"

#include "miniros/network/net_adapter.h"
#include "miniros/network/host_info.h"
#include "miniros/network/net_address.h"

#include "node_ref.h"
#include "requester_info.h"

namespace miniros {
namespace master {


class MINIROS_DECL AddressResolver {
public:
  using Lock = std::unique_lock<AddressResolver>;

  /// Scan or update existing network adapters.
  Error scanAdapters();

  /// Find adapter for specific address.
  const network::NetAdapter* findAdapterForRemoteAddress(const network::NetAddress& address) const;

  /// Find adapter for specific local address.
  const network::NetAdapter* findAdapterForLocalAddress(const network::NetAddress& address) const;

  std::shared_ptr<network::HostInfo> updateHost(const  RequesterInfo& requesterInfo);

  /// Finds host by its ip address.
  std::shared_ptr<network::HostInfo> findHost(const network::NetAddress& address) const;

  /// Determine good URI for a node.
  /// @returns resolved URI of a node.
  network::URL resolveAddressFor(const std::shared_ptr<NodeRef>& node,
    const network::NetAddress& remoteAddress,
    const network::NetAddress& localAddress) const;

  /// Determine good URI for a node.
  /// @returns resolved URI of a node.
  network::URL resolveAddressFor(const std::shared_ptr<NodeRef>& node, const std::shared_ptr<NodeRef>& requester) const;

  /// Get local hostname.
  const std::string& getHost() const;

  /// Enable/disable IP resolution.
  void setResolveIp(bool resolve);

  /// Check if specified address is a localhost.
  bool isLocalhost(const std::string& hostname) const;

  /// List all known hosts.
  std::set<std::shared_ptr<network::HostInfo>> getHosts() const;

  using AdapterCallback = std::function<void (const network::NetAdapter*)>;

  /// Iterate over all adapters.
  template <class Callback>
  void iterateAdapters(const Callback& callback) const
  {
    std::scoped_lock lock(m_mutex);
    for (const network::NetAdapter& adapter : m_adapters) {
      callback(&adapter);
    }
  }

protected:
  /// Name of the host, as reported by a system.
  std::string m_hostname;

  /// Collection of network adapters.
  std::vector<network::NetAdapter> m_adapters;

  /// A collection of hosts.
  std::map<std::string, std::shared_ptr<network::HostInfo>> m_hosts;

  bool m_resolveIp = false;
  mutable std::mutex m_mutex;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_RESOLVER_H
