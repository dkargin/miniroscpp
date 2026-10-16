//
// Passive listener for miniroscore UDP discovery advertisements.
//

#ifndef MINIROS_MASTER_DISCOVERY_H
#define MINIROS_MASTER_DISCOVERY_H

#include <memory>
#include <string>
#include <vector>

#include "miniros/duration.h"
#include "miniros/errors.h"
#include "miniros/macros.h"
#include "miniros/rostime.h"
#include "miniros/uuid.h"

namespace miniros {

/// One miniroscore heard on the discovery group.
/// `address` and `uri` are literal IPs. `hostname` is display-only and is never resolved.
struct MINIROS_DECL DiscoveredMaster {
  UUID uuid;
  std::string hostname;
  /// Literal IP from the offer, or the UDP source when the offer has no usable IP.
  std::string address;
  /// XML-RPC / HTTP port from the offer. 0 when the advertisement did not include one.
  uint16_t masterPort = 0;
  /// `http://<address>:<masterPort>/` when both address and port are known.
  /// Pass this as the `__master` remapping to `miniros::init`.
  std::string uri;
  SteadyTime lastSeen;
  /// Remote advertisement carried a non-empty collective token hash.
  bool hasToken = false;
};

/// Changes observed by one `MasterDiscovery::update()` call.
struct MINIROS_DECL DiscoveryReport {
  /// Masters seen for the first time since they were last removed.
  std::vector<DiscoveredMaster> added;
  /// Already known masters whose address, port, hostname, or token flag changed.
  std::vector<DiscoveredMaster> updated;
  /// Masters that were not heard from within the stale timeout.
  std::vector<DiscoveredMaster> removed;

  bool empty() const
  {
    return added.empty() && updated.empty() && removed.empty();
  }
};

/// Listens for nearby miniroscore instances on the discovery multicast group.
///
/// This object does not create threads. The owner calls `update()` from its main
/// thread; that call drains pending datagrams and returns what changed. All methods
/// are used from that same thread.
///
/// The default group is `239.255.42.42:11312`, the same one `miniroscore` joins.
/// Masters advertise with DISCOVER (about every 5s while unpaired, about every 30s
/// while paired). `update()` also sends a DISCOVER probe about every 3s, on the
/// multicast group and on subnet broadcast, so a master can answer with a unicast
/// OFFER. That reply is what arrives on Wi-Fi when the access point does not
/// forward multicast. This listener does not answer probes and does not pair.
///
/// On Android, multicast packets are dropped until Java code holds a
/// `WifiManager.MulticastLock`. Acquire that lock before `start()`.
class MINIROS_DECL MasterDiscovery {
public:
  /// Same default as `miniroscore` (`--multicast 239.255.42.42:11312`).
  static constexpr const char* kDefaultGroup = "239.255.42.42";
  static constexpr int kDefaultPort = 11312;

  MasterDiscovery();
  ~MasterDiscovery();

  MasterDiscovery(const MasterDiscovery&) = delete;
  MasterDiscovery& operator=(const MasterDiscovery&) = delete;

  /// Multicast group to join. Call before `start()`. Default is `kDefaultGroup:kDefaultPort`.
  Error setGroup(const std::string& address, int port);

  /// Drop a master that has been silent for this long. Default is 90s
  /// (about three missed advertisements from a paired master).
  /// A zero or negative duration disables expiry.
  void setStaleTimeout(WallDuration timeout);

  /// Bind the discovery UDP port and join the multicast group.
  Error start();

  void stop();

  bool started() const;

  /// Drain datagrams received since the previous call and expire silent masters.
  /// Call this from the main thread.
  DiscoveryReport update();

  /// Masters currently held. Same thread as `update()`.
  std::vector<DiscoveredMaster> masters() const;

private:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

} // namespace miniros

#endif
