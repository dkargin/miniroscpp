//
// Created by dkargin on 8/25/25.
//

#ifndef MINIROS_NET_ADAPTER_H
#define MINIROS_NET_ADAPTER_H

#include <cstdint>
#include <string>
#include <vector>

#include "miniros/macros.h"
#include "miniros/network/net_address.h"

namespace miniros {
namespace network {

struct HostInfo;

/// NetAdapter encapsulates all the information about specific network adapter.
struct MINIROS_DECL NetAdapter {
  /// Name of the adapter.
  std::string name;

  /// Kernel interface index (0 if unknown).
  int ifindex = 0;

  /// Hardware address as "aa:bb:cc:dd:ee:ff", empty if unknown.
  std::string mac;

  /// Address on the adapter (may be invalid when the link is up with no IP).
  NetAddress address;
  /// IPv4 netmask.
  NetAddress mask;

  /// Address for broadcasts.
  NetAddress broadcastAddress;

  /// Check it is localhost/loopback interface.
  bool isLoopback() const;
  void setLoopback(bool loopback);

  bool isUp() const;
  void setUp(bool up);

  /// Check if there is valid network address.
  bool isValid() const;

  /// Unicast IPv4/IPv6 assigned (not 0.0.0.0 / ::).
  bool hasUnicastAddress() const;

  bool hasMulticast() const;
  void setMulticast(bool multicast);
  bool hasBroadcast() const;
  void setBroadcast(bool broadcast);

  /// Check if adapter uses IPv4 address.
  bool isIPv4() const;

  /// Check if adapter uses IPv6 address.
  bool isIPv6() const;

  /// Check if specified address belongs to this address range and mask.
  bool matchNetAddress(const NetAddress& address) const;

  /// Check if this adapter can be used to access specified host.
  bool hasAccessTo(const HostInfo& host) const;

protected:

  /// Packed collection of flags.
  union {
    struct {
      /// Interface is properly configured and running.
      unsigned int up: 1;

      /// Interface is a loopback device.
      unsigned int loop: 1;
      unsigned int multicast: 1;
      unsigned int broadcast: 1;
    }f;
    uint16_t raw = 0;
  } flags_;
};

MINIROS_NODISCARD MINIROS_DECL Error scanAdapters(std::vector<NetAdapter>& adapters);

}
}
#endif // MINIROS_NET_ADAPTER_H
