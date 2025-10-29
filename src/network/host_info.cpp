//
// Created by dkargin on 10/29/25.
//

#include "miniros/network/host_info.h"

namespace miniros {
namespace network {

void HostInfo::addAddress(const network::NetAddress& addr)
{
  std::unique_lock<std::mutex> lock(mutex_);
  // External net addresses can contain port. We need to clean up it.
  NetAddress cleanAddr = addr;
  cleanAddr.setPort(0);

  addresses_.insert(cleanAddr);
}

std::set<network::NetAddress, network::AddressCompareNoPort> HostInfo::addresses() const
{
  std::unique_lock<std::mutex> lock(mutex_);
  return addresses_;
}

bool HostInfo::hasAddress(const network::NetAddress& addr) const
{
  std::unique_lock<std::mutex> lock(mutex_);

  auto it = addresses_.find(addr);
  return it != addresses_.end();
}

bool HostInfo::hasAnyAddress() const
{
  std::unique_lock<std::mutex> lock(mutex_);

  return !addresses_.empty();
}

void HostInfo::iterate(std::function<void (const network::NetAddress& addr)> function) const
{
  if (!function)
    return;

  std::unique_lock<std::mutex> lock(mutex_);
  for (const auto& addr: addresses_) {
    function(addr);
  }
}


}
}