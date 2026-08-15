//
// NetworkMonitor: optional Linux netlink watcher for NIC / address changes.
//

#ifndef MINIROS_NETWORK_MONITOR_H
#define MINIROS_NETWORK_MONITOR_H

#include <functional>
#include <memory>
#include <string>

#include "miniros/errors.h"
#include "miniros/macros.h"
#include "miniros/network/net_address.h"

namespace miniros {

class PollSet;

namespace network {

/// Live notifications about local network interfaces and addresses.
///
/// On Linux with MINIROS_USE_NETLINK, uses RTNETLINK (RTMGRP_LINK / IPV4_IFADDR).
/// Elsewhere start() returns NotSupported and multimaster falls back to polling.
class MINIROS_DECL NetworkMonitor {
public:
  enum class EventKind {
    LinkUp,
    LinkDown,
    AddressAdded,
    AddressRemoved,
  };

  struct Event {
    EventKind kind = EventKind::LinkUp;
    /// Kernel interface index (0 if unknown).
    int ifindex = 0;
    /// Interface name when available (e.g. "eth0").
    std::string ifname;
    /// Address for AddressAdded / AddressRemoved (empty for link events).
    NetAddress address;
  };

  using Callback = std::function<void(const Event&)>;

  NetworkMonitor();
  ~NetworkMonitor();

  NetworkMonitor(const NetworkMonitor&) = delete;
  NetworkMonitor& operator=(const NetworkMonitor&) = delete;

  /// True when this build includes a working netlink implementation.
  static bool supported();

  /// Open netlink socket and register it with @p pollSet.
  /// @return Ok on success, NotSupported if unavailable, SystemError on failure.
  Error start(PollSet* pollSet, Callback callback);

  void stop();

  bool running() const;

private:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

} // namespace network
} // namespace miniros

#endif
