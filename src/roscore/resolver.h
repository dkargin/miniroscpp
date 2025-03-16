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

#include "miniros/transport/net_address.h"
#include "miniros/errors.h"

namespace miniros {
namespace network {

class MINIROS_DECL AddressResolver {
public:
  struct NetAdapter {
    /// Name of the adapter.
    std::string name;
    /// Address on the adapter.
    NetAddress address;
    uint8_t ip4mask[4];

    NetAddress mask;
  };

  /// Scan or update existing network adapters.
  Error scanAdapters();

  /// Find adapter for specific address.
  const NetAdapter* findAdapterForRemoteAddress(const NetAddress& address) const;

  /// Find adapter for specific local address.
  const NetAdapter* findAdapterForLocalAddress(const NetAddress& address) const;

  struct HostInfo {
  };

  /// A collection of hosts.
  std::map<std::string, std::shared_ptr<HostInfo>> m_hosts;

protected:
  /// Name of the host, as reported by a system.
  std::string m_hostname;

  /// Collection of network adapters.
  std::vector<NetAdapter> m_adapters;

  mutable std::mutex m_mutex;
};

} // namespace network
} // namespace miniros

#endif //MINIROS_RESOLVER_H
