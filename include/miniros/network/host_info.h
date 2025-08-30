//
// Created by dkargin on 8/25/25.
//

#ifndef MINIROS_HOST_INFO_H
#define MINIROS_HOST_INFO_H

#include <set>
#include <string>

#include "miniros/network/net_address.h"

namespace miniros {
namespace network {

/// Information about network host.
struct HostInfo {
  std::string hostname;

  /// A list IP addresses, usable for external clients.
  std::set<network::NetAddress, network::AddressCompare> addresses;

  /// This host is local to master.
  /// It uses loopback or some other kind of local connection for communication.
  bool local = false;

  HostInfo(const std::string& name) : hostname(name)
  {}
};

}
}
#endif // MINIROS_HOST_INFO_H
