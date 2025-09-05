//
// Created by dkargin on 8/24/25.
//

#ifndef MINIROS_DISCOVERY_H
#define MINIROS_DISCOVERY_H

#include "common.h"

#include <functional>
#include <memory>

#include "miniros/network/net_address.h"
#include "miniros/macros.h"
#include "miniros/errors.h"

namespace miniros {

class PollSet;
struct UUID;

namespace master {

class AddressResolver;

struct DiscoveryEvent {
  /// Address extracted from UDP socket.
  network::NetAddress senderAddress;
  /// Address of the master in packet itself.
  network::NetAddress masterAddress;

  /// UUID of master.
  UUID uuid;

  /// Name of master node.
  std::string name;
};

/// Discovery deals with finding other miniros masters across the network.
class MINIROS_DECL Discovery {
public:
  /// Constructor
  /// @param resolver - address resolver.
  /// @param callbackQueue - callback queue to put discovery events.
  Discovery(AddressResolver* resolver);
  ~Discovery();

  /// Start discovery.
  /// @param pollSet - pollset to spin events.
  /// @param uuid - UUID of master.
  /// @param rpcPort - TCP port of master RPC endpoint.
  /// @param broadcastPort - port for UDP discovery broadcasts. Set to 0 to make it equal to rpcPort.
  Error start(PollSet* pollSet, const UUID& uuid, int rpcPort, int broadcastPort = 0);

  /// Stop discovery.
  void stop();

  /// Run broadcast about this master.
  Error doBroadcast();

  using DiscoveryEventCallback = std::function<void (const DiscoveryEvent& event)>;

  void setDiscoveryCallback(DiscoveryEventCallback callback);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};
}
}
#endif // MINIROS_DISCOVERY_H
