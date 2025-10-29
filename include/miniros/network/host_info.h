//
// Created by dkargin on 8/25/25.
//

#ifndef MINIROS_HOST_INFO_H
#define MINIROS_HOST_INFO_H

#include <set>
#include <string>
#include <functional>
#include <mutex>

#include "miniros/network/net_address.h"

namespace miniros {
namespace network {

/// Information about network host.
struct HostInfo {
  std::string hostname;

  /// This host is local to master.
  /// It uses loopback or some other kind of local connection for communication.
  bool local = false;

  HostInfo(const std::string& name) : hostname(name)
  {}

  /// Register new address for host.
  void addAddress(const network::NetAddress& addr);

  /// Check if address belongs to this host.
  bool hasAddress(const network::NetAddress& addr) const;

  /// Check if host has any address.
  bool hasAnyAddress() const;

  /// Get access to addresses.
  std::set<network::NetAddress, network::AddressCompareNoPort> addresses() const;

  /// Iterate over addresses in thread safe way.
  /// It WILL deadlock if try to access any methods of the same HostInfo.
  void iterate(std::function<void (const network::NetAddress& addr)> function) const;

protected:
  /// A list IP addresses, usable for external clients.
  std::set<network::NetAddress, network::AddressCompareNoPort> addresses_;
  mutable std::mutex mutex_;

};

}
}
#endif // MINIROS_HOST_INFO_H
