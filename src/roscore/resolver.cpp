//
// Created by dkargin on 3/16/25.
//

#include "internal_config.h"

#ifdef HAVE_IFADDRS_H
#include <ifaddrs.h>
#endif

#include "miniros/transport/io.h"

#include "resolver.h"

namespace miniros {
namespace network {

const char* addressFamilyName(int family)
{
  switch (family) {
    case AF_INET: return "AF_INET";
    case AF_INET6: return "AF_INET6";
    case AF_PACKET: return "AF_PACKET";
    default:
      return "????";
  }
}

// Defined in net_address.cpp
bool fillAddress(const sockaddr_in& my_addr, int len, NetAddress& address);

Error AddressResolver::scanAdapters()
{
#ifdef HAVE_IFADDRS_H
  ifaddrs* addressList = nullptr;

  int ret = getifaddrs(&addressList);
  if (ret < 0 || addressList == nullptr) {
    return Error::SystemError;
  }

  m_adapters.clear();

  for (ifaddrs *ifa = addressList; ifa; ifa = ifa->ifa_next) {
    if (!ifa->ifa_addr)
      continue;

    sa_family_t family = ifa->ifa_addr->sa_family;
    if (family == AF_INET || family == AF_INET6) {
      NetAdapter adapter;
      adapter.name = ifa->ifa_name;

      /* Display interface name and family (including symbolic form of the latter for the common families). */
      printf("%-8s %s (%d)\n", ifa->ifa_name, addressFamilyName(family), family);

      char tmpHost[NI_MAXHOST];
      const size_t addrLen = (family == AF_INET) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6);
      /* For an AF_INET* interface address, display the address. */
      int s = getnameinfo(ifa->ifa_addr, addrLen,
              tmpHost, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
        printf("getnameinfo() failed: %s\n", gai_strerror(s));
      }

      fillAddress(*reinterpret_cast<sockaddr_in*>(ifa->ifa_addr), addrLen, adapter.address);
      std::string host = tmpHost;

      /* For an AF_INET* interface address, display the address. */
      s = getnameinfo(ifa->ifa_netmask, (family == AF_INET) ? sizeof(struct sockaddr_in) :
                                    sizeof(struct sockaddr_in6),
              tmpHost, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);
      if (s != 0) {
        printf("getnameinfo() failed: %s\n", gai_strerror(s));
      }
      std::string mask = tmpHost;
      printf("\t\taddress: <%s> mask: <%s>\n", host.c_str(), mask.c_str());

      m_adapters.push_back(adapter);
    }
  } // end for

  if (addressList)
    freeifaddrs(addressList);
#endif
  return Error::Ok ;
}
} // namespace network
} // namespace miniros
