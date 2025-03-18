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
#include "miniros/transport/net_address.h"
#include "node_ref.h"
#include "requester_info.h"

namespace miniros {
namespace master {

/// Information about network host.
struct HostInfo {
  std::string hostname;

  /// A list IP addresses, usable for external clients.
  std::set<network::NetAddress, network::AddressCompare> addresses;

  /// This host is local to master.
  /// It uses loopback or some other kind of local connection for communication.
  bool local = false;

  HostInfo(const std::string& name);
};

/// NetAdapter encapsulates all the information about specific network adapter.
struct NetAdapter {
  /// Name of the adapter.
  std::string name;
  /// Address on the adapter.
  network::NetAddress address;
  /// IPv4 netmask.
  network::NetAddress mask;

  /// Check it is localhost/loopback interface.
  bool isLoopback() const;

  /// Check if specified address belongs to this address range and mask.
  bool matchNetAddress(const network::NetAddress& address) const;

  /// Check if this adapter can be used to access specified host.
  bool hasAccessTo(const HostInfo& host) const;
};

class MINIROS_DECL AddressResolver {
public:
  /// Scan or update existing network adapters.
  Error scanAdapters();

  /// Find adapter for specific address.
  const NetAdapter* findAdapterForRemoteAddress(const network::NetAddress& address) const;

  /// Find adapter for specific local address.
  const NetAdapter* findAdapterForLocalAddress(const network::NetAddress& address) const;

  std::shared_ptr<HostInfo> updateHost(const  RequesterInfo& requesterInfo);

  /// Finds host by its ip address.
  std::shared_ptr<HostInfo> findHost(const network::NetAddress& address) const;

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

protected:
  /// Name of the host, as reported by a system.
  std::string m_hostname;

  /// Collection of network adapters.
  std::vector<NetAdapter> m_adapters;

  /// A collection of hosts.
  std::map<std::string, std::shared_ptr<HostInfo>> m_hosts;

  bool m_resolveIp = false;
  mutable std::mutex m_mutex;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_RESOLVER_H
