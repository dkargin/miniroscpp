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
  using Lock = std::unique_lock<std::mutex>;

  /// Scan or update existing network adapters.
  Error scanAdapters();

  /// Find adapter for specific address.
  const network::NetAdapter* findAdapterForRemoteAddress(const network::NetAddress& address) const;

  /// Find adapter for specific local address.
  const network::NetAdapter* findAdapterForLocalAddress(const network::NetAddress& address) const;

  /// IPv4 address on a local up non-loopback adapter that can reach `peerAddr`.
  /// Prefers the same subnet; otherwise the first such adapter. Empty if none.
  std::string pickLanIp(const network::NetAddress& peerAddr) const;

  /// Learn a host from an XML-RPC caller (`--resolve`).
  /// `clientAddress` is the TCP source; `callerApi` may contain a hostname or IP.
  /// Never calls getaddrinfo.
  std::shared_ptr<network::HostInfo> updateHost(const RequesterInfo& requesterInfo);

  /// Finds host by its ip address.
  std::shared_ptr<network::HostInfo> findHost(const network::NetAddress& address) const;

  /// IPs known for this node: HostInfo, local adapters when the node is on this host,
  /// and the registered URI host when it is already an IP literal.
  std::vector<network::NetAddress> listNodeAddresses(const std::shared_ptr<NodeRef>& node) const;

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

  /// Enable/disable IP resolution for local Master API replies (`--resolve`).
  void setResolveIp(bool resolve);

  /// Current `--resolve` flag. Does not perform any lookup.
  bool resolveIp() const;

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
  /// Caller must already own m_mutex (pass the lock as proof).
  std::shared_ptr<network::HostInfo> findHostLocked(Lock& lock, const network::NetAddress& address) const;
  bool isLoopbackNameLocked(Lock& lock, const std::string& host) const;

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
