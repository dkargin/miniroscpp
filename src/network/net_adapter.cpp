//
// Created by dkargin on 8/25/25.
//

#include "internal_config.h"

#include <cassert>

#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#include <net/if.h>

#endif

#include "miniros/network/net_adapter.h"
#include "miniros/network/host_info.h"

#include "miniros/transport/io.h"

namespace miniros {
namespace network {


bool NetAdapter::isLoopback() const
{
  return address.isLoopback();
}

bool NetAdapter::isValid() const
{
  return address.valid();
}

bool NetAdapter::isIPv4() const
{
  return address.type() == NetAddress::AddressIPv4;
}

bool NetAdapter::isIPv6() const
{
  return address.type() == NetAddress::AddressIPv6;
}

bool NetAdapter::matchNetAddress(const network::NetAddress& other) const
{
  if (address.type() != other.type()) {
    return false;
  }

  if (!mask.valid() || !address.valid() || !other.valid())
    return false;

  if (other.type() == network::NetAddress::AddressIPv4) {
    const sockaddr_in* rawAddr = static_cast<const sockaddr_in*>(address.rawAddress());
    const sockaddr_in* rawMask = static_cast<const sockaddr_in*>(mask.rawAddress());
    const sockaddr_in* rawOtherAddr = static_cast<const sockaddr_in*>(other.rawAddress());

    uint32_t ip = rawAddr->sin_addr.s_addr;
    uint32_t netip = rawOtherAddr->sin_addr.s_addr;
    uint32_t netmask = rawMask->sin_addr.s_addr;
    // is on same subnet...
    return ((netip & netmask) == (ip & netmask));
  }
  // Not implemented.
  assert(false);
  return false;
}

bool NetAdapter::hasAccessTo(const HostInfo& host) const
{
  for (const auto& addr: host.addresses) {
    if (addr.isLoopback() || addr.isUnspecified())
      continue;
    if (matchNetAddress(addr))
      return true;
  }
  return false;
}

Error scanAdapters(std::vector<network::NetAdapter>& adapters)
{
#ifdef HAVE_IFADDRS_H
  // https://man7.org/linux/man-pages/man3/getifaddrs.3.html
  // TODO: subscribe to updates on network interfaces.
  // NETLINK kernel interface can be used to subscribe to notification socket. But it will
  // require some integration with a local PollSet interface. Meanwhile, windows will need something very different.
  ifaddrs* addressList = nullptr;

  int ret = getifaddrs(&addressList);
  if (ret < 0 || addressList == nullptr) {
    return Error::SystemError;
  }

  adapters.clear();

  for (ifaddrs *ifa = addressList; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;

    sa_family_t family = ifa->ifa_addr->sa_family;
    if (family == AF_INET || family == AF_INET6) {
      network::NetAdapter adapter;
      adapter.name = ifa->ifa_name;

      adapter.loopback = (ifa->ifa_flags & IFF_LOOPBACK) != 0;
      adapter.up = (ifa->ifa_flags & IFF_UP) != 0;

      // TODO: The error is not expected here.
      if (!fillAddress(*ifa->ifa_addr, adapter.address)) {
        continue;
      }
      if (adapter.address.isUnspecified()) {
        continue;
      }
      if (!fillAddress(*ifa->ifa_netmask, adapter.mask)) {
        continue;
      }

      if (ifa->ifa_flags & IFF_BROADCAST) {
        if (!fillAddress(*ifa->ifa_ifu.ifu_broadaddr, adapter.broadcastAddress)) {
          continue;
        }
      }
      // TODO: Probably read additional flags, like DHCP or whatever.
      adapters.push_back(adapter);
    }
  }

  if (addressList)
    freeifaddrs(addressList);
#else
  return Error::NotImplemented;
#endif

  return Error::Ok;
}

}
}