//
// Created by dkargin on 8/25/25.
//

#ifndef MINIROS_NET_ADAPTER_H
#define MINIROS_NET_ADAPTER_H

#include <string>
#include <vector>

#include "miniros/network/net_address.h"

namespace miniros {
namespace network {

struct HostInfo;

/// NetAdapter encapsulates all the information about specific network adapter.
struct NetAdapter {
  /// Name of the adapter.
  std::string name;
  /// Address on the adapter.
  network::NetAddress address;
  /// IPv4 netmask.
  network::NetAddress mask;

  bool loopback = false;
  bool up = false;

  /// Address for broadcasts.
  network::NetAddress broadcastAddress;

  /// Check it is localhost/loopback interface.
  bool isLoopback() const;

  /// Check if there is valid network address.
  bool isValid() const;

  /// Check if adapter uses IPv4 address.
  bool isIPv4() const;

  /// Check if adapter uses IPv6 address.
  bool isIPv6() const;

  /// Check if specified address belongs to this address range and mask.
  bool matchNetAddress(const network::NetAddress& address) const;

  /// Check if this adapter can be used to access specified host.
  bool hasAccessTo(const HostInfo& host) const;
};

NODISCARD Error scanAdapters(std::vector<NetAdapter>& adapters);

}
}
#endif // MINIROS_NET_ADAPTER_H
