//
// Created by dkargin on 8/24/25.
//

#ifndef MINIROS_DISCOVERY_H
#define MINIROS_DISCOVERY_H

#include "common.h"

#include <functional>
#include <memory>

#include "miniros/network/net_address.h"
#include "miniros/network/url.h"

#include "miniros/macros.h"
#include "miniros/errors.h"

namespace miniros {

class PollSet;
struct UUID;

namespace master {

class AddressResolver;

struct DiscoveryPacket;

struct DiscoveryEvent {
  /// Address extracted from UDP socket.
  network::NetAddress senderAddress;

  /// Corresponding ROS_MASTER_URI.
  network::URL masterUri;

  /// UUID of master.
  UUID uuid;

  /// Name of master node.
  std::string name;

  /// Version of master.
  int version = 0;
};

/// Discovery deals with finding other miniros masters across the network.
class MINIROS_DECL Discovery {
public:
  /// Constructor
  /// @param resolver - address resolver.
  Discovery(AddressResolver* resolver);
  ~Discovery();

  /// Start discovery.
  /// @param pollSet - pollset to spin events.
  /// @param uuid - UUID of master.
  /// @param rpcUrl - URL of master RPC endpoint.
  Error start(PollSet* pollSet, const UUID& uuid, const network::URL& rpcUrl);

  /// Stop discovery.
  void stop();

  /// Run broadcast about this master.
  Error doBroadcast();

  using DiscoveryEventCallback = std::function<void (const DiscoveryEvent& event)>;

  /// Set multicast group for discovery broadcasts.
  Error setMulticast(const std::string& group);

  /// Enable/disable local UDP broadcasts
  void setUdpBroadcasts(int port);

  void setDiscoveryCallback(DiscoveryEventCallback callback);

  void fillDiscoveryPacket(DiscoveryPacket& packet);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}
#endif // MINIROS_DISCOVERY_H
