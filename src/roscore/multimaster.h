//
// Multimaster manager: DHCP-like discovery + UDP registration sync.
//

#ifndef MINIROS_MULTIMASTER_H
#define MINIROS_MULTIMASTER_H

#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "multimaster_protocol.h"
#include "master_cache.h"

#include "miniros/errors.h"
#include "miniros/macros.h"
#include "miniros/uuid.h"
#include "miniros/network/net_address.h"
#include "miniros/network/url.h"
#include "miniros/rostime.h"

#include "miniros_msgs/RegistrationRecord.hxx"

namespace miniros {

class PollSet;

namespace master {

class AddressResolver;
class RegistrationManager;
class MasterHandler;

/// Default multicast discovery group (robots may share; collectives are split by token).
constexpr const char* kDefaultMulticastAddr = "239.255.42.42";
constexpr int kDefaultMulticastPort = 11312;

/// High-level peer state for UI / diagnostics.
enum class PeerState {
  Discovered,
  Offering,
  Requesting,
  Paired,
  Stale,
  /// Remote packet used this master's GUID (cloned cache / identity). Not pairable.
  GuidCollision,
};

struct PeerInfo {
  UUID uuid;
  PeerState state = PeerState::Discovered;
  network::URL masterUri;
  network::NetAddress lastAddress;
  SteadyTime lastSeen;
  /// Local token fingerprint matches remote packet fingerprint.
  bool tokenMatch = false;
  /// Remote advertised a non-empty token hash (has a configured token).
  bool remoteHasToken = false;
  size_t foreignPubs = 0;
  size_t foreignSubs = 0;
  size_t foreignSrvs = 0;
};

/// Owns UDP sockets used for discovery (optional multicast) and unicast sync.
class MINIROS_DECL MultimasterManager {
public:
  using ApplyRecordsFn = std::function<void(const UUID& peer, const std::vector<miniros_msgs::RegistrationRecord>& records, bool snapshot)>;
  using DropPeerFn = std::function<void(const UUID& peer)>;
  using CollectSnapshotFn = std::function<std::vector<miniros_msgs::RegistrationRecord>()>;

  MultimasterManager(AddressResolver* resolver, RegistrationManager* regs);
  ~MultimasterManager();

  /// Shared secret used for collective membership.
  /// Empty means discovery-only until a token is set, or until an operator
  /// manually pairs into an open (no-token) mesh via the HTTP form / API.
  /// Changing the token while started leaves the current collective first.
  void setToken(const std::string& token);

  /// UDP port for unicast sync. 0 means "use master RPC port" at start().
  void setUdpPort(int port);

  /// Multicast discovery group host:port. Empty host disables multicast.
  /// Changing the endpoint while started leaves the current collective and rejoins.
  void setMulticast(const std::string& addr, int port);

  void setLocalOnlyTopics(std::set<std::string> topics);

  /// Optional collector of local registrations for outbound snapshots.
  void setCollectSnapshot(CollectSnapshotFn fn);

  /// Called when inbound sync records should be applied to local master state.
  void setApplyRecords(ApplyRecordsFn fn);

  /// Called when a paired peer is lost (timeout / BYE / disconnect).
  void setDropPeer(DropPeerFn fn);

  /// Bind UDP, register with PollSet. rpcUrl is the local Master HTTP/RPC URI.
  Error start(PollSet* pollSet, const UUID& uuid, const network::URL& rpcUrl);

  void stop();

  /// Periodic maintenance: retransmits, heartbeats, discover retries, timeouts.
  void update();

  /// Broadcast / multicast / probe DISCOVER (always available once started).
  Error sendDiscover();

  /// Add explicit peer sync address to probe (same-host / no multicast).
  Error addPeerProbe(const network::NetAddress& addr);

  /// Force REQUEST toward a peer known by UUID.
  /// @param token - if non-empty and different from current, leaves old collective first.
  Error requestPair(const UUID& peerUuid, const std::string& token = {});

  /// Force REQUEST toward discovered master registered as node name.
  Error requestPairByNodeName(const std::string& nodeName, const std::string& token = {});

  /// Leave the multimaster collective: BYE all paired peers and drop their foreign regs.
  Error disconnectAll();

  /// Notify manager about a local registration change (outbound delta).
  void announceLocalChange(const miniros_msgs::RegistrationRecord& record);

  /// Called when pairing topology changes (pair / stale / disconnect) so callers
  /// can persist state (e.g. MasterCache::markDirty).
  void setTopologyChanged(std::function<void()> fn);

  /// Re-probe and REQUEST toward peers restored from MasterCache after restart.
  void restoreCachedPeers(const std::vector<CachedPeer>& peers);

  std::vector<PeerInfo> listPeers() const;

  /// Local master instance GUID (same as /run_id).
  UUID localUuid() const;

  /// Snapshot of currently paired peers suitable for MasterCache persistence.
  std::vector<CachedPeer> collectCachedPeers() const;

  bool hasToken() const;
  bool hasPairedPeers() const;
  int udpPort() const;
  std::string multicastEndpoint() const;
  /// Empty when multicast is active. Otherwise a short reason (join/bind failure).
  std::string multicastError() const;

  static const char* peerStateName(PeerState s);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

} // namespace master
} // namespace miniros

#endif // MINIROS_MULTIMASTER_H
