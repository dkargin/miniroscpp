//
// Created by dkargin on 8/25/25.
//

#include "internal_config.h"

#include <cassert>
#include <cstdio>
#include <map>
#include <set>

#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#include <net/if.h>
#if defined(AF_PACKET)
#include <netpacket/packet.h>
#endif
#endif

#include "miniros/network/net_adapter.h"
#include "miniros/network/host_info.h"

#include "miniros/io/io.h"

namespace miniros {
namespace network {


bool NetAdapter::isLoopback() const
{
  return flags_.f.loop;
}

void NetAdapter::setLoopback(bool loopback)
{
  flags_.f.loop = loopback;
}

bool NetAdapter::isUp() const
{
  return flags_.f.up;
}

void NetAdapter::setUp(bool up)
{
  flags_.f.up = up;
}

bool NetAdapter::isValid() const
{
  return address.valid();
}

bool NetAdapter::hasUnicastAddress() const
{
  return address.valid() && !address.isUnspecified() && !address.isAny() &&
         !address.isLimitedBroadcast() && !address.isMulticast();
}

bool NetAdapter::hasMulticast() const
{
  return flags_.f.multicast;
}

void NetAdapter::setMulticast(bool multicast)
{
  flags_.f.multicast = multicast;
}

bool NetAdapter::hasBroadcast() const
{
  return flags_.f.broadcast;
}

void NetAdapter::setBroadcast(bool broadcast)
{
  flags_.f.broadcast = broadcast;
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
  for (const auto& addr: host.addresses()) {
    if (addr.isLoopback() || addr.isUnspecified())
      continue;
    if (matchNetAddress(addr))
      return true;
  }
  return false;
}

namespace {

#ifdef HAVE_IFADDRS_H
struct LinkInfo {
  std::string name;
  std::string mac;
  int ifindex = 0;
  unsigned flags = 0;
};

std::string macFromSockaddr(const sockaddr* sa)
{
#if defined(AF_PACKET)
  if (!sa || sa->sa_family != AF_PACKET)
    return {};
  const auto* ll = reinterpret_cast<const sockaddr_ll*>(sa);
  if (ll->sll_halen < 6)
    return {};
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%02x:%02x:%02x:%02x:%02x:%02x",
    ll->sll_addr[0], ll->sll_addr[1], ll->sll_addr[2],
    ll->sll_addr[3], ll->sll_addr[4], ll->sll_addr[5]);
  return buf;
#else
  (void)sa;
  return {};
#endif
}

int ifindexFromSockaddr(const sockaddr* sa, const char* ifname)
{
#if defined(AF_PACKET)
  if (sa && sa->sa_family == AF_PACKET) {
    const auto* ll = reinterpret_cast<const sockaddr_ll*>(sa);
    if (ll->sll_ifindex > 0)
      return ll->sll_ifindex;
  }
#endif
  if (ifname && ifname[0])
    return static_cast<int>(if_nametoindex(ifname));
  return 0;
}

void applyLinkFlags(NetAdapter& adapter, unsigned flags)
{
  adapter.setLoopback(flags & IFF_LOOPBACK);
  adapter.setUp(flags & IFF_UP);
  adapter.setMulticast(flags & IFF_MULTICAST);
  adapter.setBroadcast(flags & IFF_BROADCAST);
}
#endif

} // namespace

Error scanAdapters(std::vector<network::NetAdapter>& adapters)
{
#ifdef HAVE_IFADDRS_H
  // https://man7.org/linux/man-pages/man3/getifaddrs.3.html
  ifaddrs* addressList = nullptr;

  int ret = getifaddrs(&addressList);
  if (ret < 0 || addressList == nullptr) {
    return Error::SystemError;
  }

  adapters.clear();

  std::map<std::string, LinkInfo> links;
  std::set<std::string> namesWithIp;

  for (ifaddrs *ifa = addressList; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_name)
      continue;
    const std::string ifname = ifa->ifa_name;

    if (!ifa->ifa_addr) {
      // Interface present but with no address family entry yet.
      LinkInfo& link = links[ifname];
      link.name = ifname;
      link.flags |= ifa->ifa_flags;
      if (!link.ifindex)
        link.ifindex = static_cast<int>(if_nametoindex(ifname.c_str()));
      continue;
    }

    const sa_family_t family = ifa->ifa_addr->sa_family;
#if defined(AF_PACKET)
    if (family == AF_PACKET) {
      LinkInfo& link = links[ifname];
      link.name = ifname;
      link.flags |= ifa->ifa_flags;
      if (link.mac.empty())
        link.mac = macFromSockaddr(ifa->ifa_addr);
      if (!link.ifindex)
        link.ifindex = ifindexFromSockaddr(ifa->ifa_addr, ifname.c_str());
      continue;
    }
#endif

    if (family != AF_INET && family != AF_INET6)
      continue;

    network::NetAdapter adapter;
    adapter.name = ifname;
    applyLinkFlags(adapter, ifa->ifa_flags);
    adapter.ifindex = ifindexFromSockaddr(nullptr, ifname.c_str());

    if (!fillAddress(ifa->ifa_addr, adapter.address))
      continue;

    if (ifa->ifa_netmask)
      (void)fillAddress(ifa->ifa_netmask, adapter.mask);

    if ((ifa->ifa_flags & IFF_BROADCAST) && ifa->ifa_broadaddr)
      (void)fillAddress(ifa->ifa_broadaddr, adapter.broadcastAddress);

    namesWithIp.insert(ifname);
    adapters.push_back(adapter);
  }

  for (auto& adapter : adapters) {
    auto it = links.find(adapter.name);
    if (it == links.end())
      continue;
    if (adapter.mac.empty())
      adapter.mac = it->second.mac;
    if (!adapter.ifindex)
      adapter.ifindex = it->second.ifindex;
    adapter.setMulticast(it->second.flags & IFF_MULTICAST);
    adapter.setBroadcast(it->second.flags & IFF_BROADCAST);
  }

  // IBSS / unconfigured ethernet: link is up, no IPv4/IPv6 yet. Still useful for
  // interface-scoped UDP broadcast (source 0.0.0.0).
  for (const auto& [name, link] : links) {
    if (namesWithIp.count(name))
      continue;
    if (!(link.flags & IFF_UP))
      continue;
    network::NetAdapter adapter;
    adapter.name = name;
    adapter.mac = link.mac;
    adapter.ifindex = link.ifindex ? link.ifindex : static_cast<int>(if_nametoindex(name.c_str()));
    applyLinkFlags(adapter, link.flags);
    adapters.push_back(adapter);
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