//
// Multimaster manager implementation.
//

#include "multimaster.h"

#include "miniros/console.h"
#include "miniros/io/poll_set.h"
#include "miniros/network/socket.h"
#include "miniros/network/net_adapter.h"
#include "miniros/network/network_monitor.h"

#include "registration_manager.h"
#include "resolver.h"
#include "node_ref.h"

#include "miniros_msgs/MasterOffer.hxx"
#include "miniros_msgs/MasterSync.hxx"

#include <array>
#include <map>
#include <set>

namespace miniros {
namespace master {

namespace {

std::string uuidKey(const UUID& u)
{
  return u.toString();
}

/// Peers that reuse our GUID are stored under a host-keyed id so they do not
/// collide with the real uuid map entry (there is none for self).
std::string collisionPeerKey(const network::NetAddress& sender)
{
  return std::string("guid-collision:") + sender.address;
}

} // namespace

struct PendingSend {
  std::vector<uint8_t> packet;
  network::NetAddress dest;
  uint32_t seq = 0;
  SteadyTime nextRetry;
  int attempts = 0;
  /// True for SyncSnapshot — ACK (or give-up) must clear snapshotInFlight.
  bool isSnapshot = false;
};

struct Peer {
  PeerInfo info;
  uint32_t last_rx_seq = 0;
  uint32_t next_tx_seq = 1;
  SteadyTime nextHeartbeat;
  std::map<uint32_t, PendingSend> inflight; // seq -> pending
  /// Snapshot packet unacked: do not send deltas (they can race ahead of an
  /// older snapshot and then be wiped by dropMultimasterPeer on apply).
  bool snapshotInFlight = false;
  /// localRegGen sampled when the in-flight snapshot was collected.
  uint64_t snapshotCollectGen = 0;
};

struct MultimasterManager::Internal {
  AddressResolver* resolver = nullptr;
  RegistrationManager* regs = nullptr;
  PollSet* pollSet = nullptr;

  UUID uuid;
  network::URL rpcUrl;
  int configuredPort = 0; // 0 => use rpc port for sync
  int boundPort = 0;      // sync UDP port

  std::string token;
  std::array<uint8_t, mm::kTokenHashSize> token_hash{};
  bool discoveryEnabled = true;

  std::string multicastHost = kDefaultMulticastAddr;
  int multicastPort = kDefaultMulticastPort;
  /// User/config wants multicast discovery (false only for --multicast off).
  bool multicastEnabled = true;
  std::string multicastError;
  /// Subnet broadcast addresses (IPv4, port = sync UDP port) used when multicast is down
  /// or as a LAN fallback alongside multicast.
  std::vector<network::NetAddress> subnetBroadcasts;

  std::set<std::string> localOnlyTopics{"/rosout", "/rosout_agg"};
  /// IPv4 addresses of local adapters (to distinguish multicast echo from a cloned GUID).
  std::set<std::string> localIps;
  /// Interfaces already joined for multicast (adapter IPv4 address strings).
  std::set<std::string> joinedMulticastIfaces;

  /// Unicast sync (+ broadcast fallback when multicast disabled).
  /// After initSyncSocket(): either fully attached to PollSet, or invalid.
  network::NetSocket syncSocket;
  /// Shared-port multicast discovery (optional).
  /// After initDiscoverySocket(): either fully attached to PollSet, or invalid.
  network::NetSocket discoverySocket;
  network::NetAddress broadcastAddr;
  network::NetAddress multicastGroup;
  std::vector<network::NetAddress> peerProbes;
  network::NetworkMonitor networkMonitor;

  CollectSnapshotFn collectSnapshot;
  ApplyRecordsFn applyRecords;
  DropPeerFn dropPeer;
  std::function<void()> topologyChanged;

  mutable std::mutex guard;
  std::map<std::string, Peer> peers; // uuid string -> peer

  uint32_t next_control_seq = 1;
  SteadyTime nextDiscover;
  SteadyTime nextNetworkRefresh;

  /// Bumped on every local registration announce; snapshot collect loops until stable.
  uint64_t localRegGen = 0;

  bool started = false;

  Internal(AddressResolver* r, RegistrationManager* m) : resolver(r), regs(m) {}

  void fillHeader(mm::Header& h, uint8_t op, uint16_t flags, uint32_t seq) const
  {
    h.magic = mm::kMagic;
    h.version = mm::kVersion;
    h.op = op;
    h.flags = flags;
    h.seq = seq;
    h.ack_seq = 0;
    h.uuid = uuid.toBytes();
    h.token_hash = token_hash;
  }

  Error sendRaw(network::NetSocket& sock, const std::vector<uint8_t>& packet, const network::NetAddress& dest)
  {
    if (!sock.valid())
      return Error::NotConnected;
    auto [n, err] = sock.send(packet.data(), packet.size(), &dest);
    (void)n;
    return err;
  }

  Error sendOp(uint8_t op, const std::vector<uint8_t>& payload, const network::NetAddress& dest,
    bool needsAck, bool viaDiscovery = false, uint32_t* outSeq = nullptr)
  {
    mm::Header h;
    uint32_t seq = next_control_seq++;
    fillHeader(h, op, needsAck ? mm::Header::FLAG_NEEDS_ACK : 0, seq);
    std::vector<uint8_t> packet;
    if (Error e = mm::buildPacket(h, payload, packet); !e)
      return e;
    network::NetSocket& sock = (viaDiscovery && discoverySocket.valid()) ? discoverySocket : syncSocket;
    if (Error e = sendRaw(sock, packet, dest); !e)
      return e;
    if (needsAck) {
      PendingSend ps;
      ps.packet = packet;
      ps.dest = dest;
      ps.seq = seq;
      ps.nextRetry = SteadyTime::now() + WallDuration(0.5);
      (void)ps;
    }
    if (outSeq)
      *outSeq = seq;
    return Error::Ok;
  }

  Peer* findPeer(const UUID& id)
  {
    auto it = peers.find(uuidKey(id));
    return it == peers.end() ? nullptr : &it->second;
  }

  Peer& ensurePeer(const UUID& id, const network::NetAddress& addr)
  {
    Peer& p = peers[uuidKey(id)];
    if (!p.info.uuid.valid())
      p.info.uuid = id;
    if (addr.valid())
      p.info.lastAddress = addr;
    p.info.lastSeen = SteadyTime::now();
    return p;
  }

  /// Tear down a socket that was successfully attached (or is already invalid).
  void detachSocket(network::NetSocket& sock)
  {
    if (pollSet && sock.valid())
      pollSet->delSocket(sock.fd());
    sock.close();
  }

  void detachSockets()
  {
    detachSocket(syncSocket);
    detachSocket(discoverySocket);
  }

  Error attachReadable(network::NetSocket& sock, void (Internal::*handler)())
  {
    int fd = sock.fd();
    if (!pollSet->addSocket(fd, PollSet::EventIn,
          [this, handler](int flags) {
            if (flags & PollSet::EventIn)
              (this->*handler)();
            return 0;
          },
          {}, THIS_LOCATION)) {
      return Error::InternalError;
    }
    return Error::Ok;
  }

  Error initSyncSocket(int port)
  {
    auto dispose = [this](Error e) {
      syncSocket.close();
      return e;
    };

    Error error = syncSocket.initUDP(false);
    if (!error)
      return dispose(error);
    error = syncSocket.setReuseAddr(true);
    if (!error)
      return dispose(error);
    error = syncSocket.setReusePort(true);
    if (!error)
      return dispose(error);
    error = syncSocket.setBroadcast(true);
    if (!error)
      return dispose(error);
    error = syncSocket.bind(port);
    if (!error)
      return dispose(error);
    boundPort = port == 0 ? syncSocket.port() : port;
    broadcastAddr = network::NetAddress::fromIp4String("255.255.255.255", boundPort);
    refreshLocalIps();
    if (Error e = attachReadable(syncSocket, &Internal::onSyncReadable); !e)
      return dispose(e);
    return Error::Ok;
  }

  /// On success: discoverySocket is bound, joined, and in PollSet.
  /// On failure: discoverySocket is closed/invalid (never left half-registered).
  Error initDiscoverySocket()
  {
    auto dispose = [this](Error e) {
      discoverySocket.close();
      multicastGroup = {};
      joinedMulticastIfaces.clear();
      return e;
    };

    multicastError.clear();
    joinedMulticastIfaces.clear();
    if (!multicastEnabled || multicastHost.empty() || multicastPort <= 0)
      return Error::Ok;

    multicastGroup = network::NetAddress::fromIp4String(multicastHost, multicastPort);
    if (!multicastGroup.valid())
      return dispose(Error::InvalidAddress);

    Error error = discoverySocket.initUDP(false);
    if (!error)
      return dispose(error);
    error = discoverySocket.setReuseAddr(true);
    if (!error)
      return dispose(error);
    error = discoverySocket.setReusePort(true);
    if (!error)
      return dispose(error);
    error = discoverySocket.bind(multicastPort);
    if (!error)
      return dispose(error);

    const int joined = joinMulticastOnNewAdapters();
    if (joined == 0) {
      // No usable NIC yet (common when master starts before ethernet is up).
      // Join INADDR_ANY so membership can attach when a route appears, and keep
      // retrying per-iface joins from refreshNetworkUnlocked().
      Error anyJoin = discoverySocket.joinMulticastGroup(multicastGroup, /*loop=*/true);
      if (!anyJoin)
        return dispose(anyJoin);
      MINIROS_WARN_NAMED("multimaster",
        "No up IPv4 interface for multicast yet; joined %s on INADDR_ANY (will rejoin per-iface when NICs appear)",
        multicastGroup.str().c_str());
    }
    if (Error e = attachReadable(discoverySocket, &Internal::onDiscoveryReadable); !e)
      return dispose(e);
    return Error::Ok;
  }

  std::vector<network::NetAdapter> localAdapters() const
  {
    std::vector<network::NetAdapter> adapters;
    (void)network::scanAdapters(adapters);
    return adapters;
  }

  /// Join multicast on adapters not yet in joinedMulticastIfaces. @return new joins.
  int joinMulticastOnNewAdapters()
  {
    if (!discoverySocket.valid() || !multicastGroup.valid())
      return 0;
    int newly = 0;
    for (const auto& adapter : localAdapters()) {
      if (!adapter.isUp() || adapter.isLoopback() || !adapter.isIPv4())
        continue;
      const std::string key = adapter.address.address;
      if (key.empty() || joinedMulticastIfaces.count(key))
        continue;
      Error e = discoverySocket.joinMulticastGroup(multicastGroup, adapter.address, /*loop=*/true);
      if (e) {
        joinedMulticastIfaces.insert(key);
        ++newly;
        MINIROS_INFO_NAMED("multimaster", "Joined multicast %s on %s (%s)",
          multicastGroup.str().c_str(), adapter.name.c_str(), key.c_str());
      } else {
        MINIROS_WARN_NAMED("multimaster", "Multicast join on %s (%s) failed: %s",
          adapter.name.c_str(), key.c_str(), e.toString());
      }
    }
    return newly;
  }

  void refreshLocalIps()
  {
    localIps.clear();
    subnetBroadcasts.clear();
    for (const auto& a : localAdapters()) {
      if (!a.address.address.empty())
        localIps.insert(a.address.address);
      if (!a.isUp() || a.isLoopback() || !a.isIPv4())
        continue;
      if (a.broadcastAddress.valid()) {
        network::NetAddress bcast = a.broadcastAddress;
        if (boundPort > 0)
          (void)bcast.setPort(boundPort);
        subnetBroadcasts.push_back(bcast);
      }
    }
  }

  /// Re-scan NICs: update broadcasts / localIps, retry multicast if it failed at boot,
  /// join newly appeared ethernet interfaces.
  void refreshNetworkUnlocked()
  {
    refreshLocalIps();

    if (resolver)
      (void)resolver->scanAdapters();

    if (!multicastEnabled || multicastHost.empty() || multicastPort <= 0)
      return;

    if (!discoverySocket.valid()) {
      if (Error e = initDiscoverySocket(); !e) {
        multicastError = e.toString();
        MINIROS_DEBUG_NAMED("multimaster",
          "Multicast discovery still unavailable after NIC refresh: %s", e.toString());
      } else {
        multicastError.clear();
        MINIROS_INFO_NAMED("multimaster",
          "Multicast discovery came up after network appeared (%s:%d)",
          multicastHost.c_str(), multicastPort);
        nextDiscover = SteadyTime::now();
      }
      return;
    }

    if (joinMulticastOnNewAdapters() > 0)
      nextDiscover = SteadyTime::now();
  }

  void onNetworkEvent(const network::NetworkMonitor::Event& ev)
  {
    std::lock_guard lock(guard);
    if (!started)
      return;
    MINIROS_DEBUG_NAMED("multimaster", "Network event on %s — refreshing discovery",
      ev.ifname.empty() ? "?" : ev.ifname.c_str());
    refreshNetworkUnlocked();
    // Kick an immediate DISCOVER once the new address/link is usable.
    if (ev.kind == network::NetworkMonitor::EventKind::LinkUp ||
        ev.kind == network::NetworkMonitor::EventKind::AddressAdded) {
      nextDiscover = SteadyTime::now();
    }
  }

  Error sendDiscoverUnlocked()
  {
    refreshNetworkUnlocked();

    // No usable IPv4 yet — avoid ENETUNREACH spam on every discover tick.
    bool haveLan = !subnetBroadcasts.empty();
    if (!haveLan) {
      for (const auto& a : localAdapters()) {
        if (a.isUp() && a.isIPv4() && !a.isLoopback()) {
          haveLan = true;
          break;
        }
      }
    }
    if (!haveLan && peerProbes.empty()) {
      MINIROS_DEBUG_NAMED("multimaster",
        "Skipping DISCOVER: no up IPv4 interface yet");
      return Error::Ok;
    }

    Error last = Error::Ok;
    if (multicastEnabled && multicastGroup.valid() && discoverySocket.valid()) {
      MINIROS_DEBUG_NAMED("multimaster", "Sending DISCOVER multicast to %s",
        multicastGroup.str().c_str());
      last = sendOp(mm::Header::OP_DISCOVER, {}, multicastGroup, false, true);
    }
    if (broadcastAddr.valid()) {
      MINIROS_DEBUG_NAMED("multimaster", "Sending DISCOVER limited-broadcast on UDP %d", boundPort);
      last = sendOp(mm::Header::OP_DISCOVER, {}, broadcastAddr, false);
    }
    for (const network::NetAddress& bcast : subnetBroadcasts) {
      MINIROS_DEBUG_NAMED("multimaster", "Sending DISCOVER subnet-broadcast to %s", bcast.str().c_str());
      last = sendOp(mm::Header::OP_DISCOVER, {}, bcast, false);
    }
    for (const network::NetAddress& peer : peerProbes) {
      MINIROS_DEBUG_NAMED("multimaster", "Sending DISCOVER probe to %s", peer.str().c_str());
      last = sendOp(mm::Header::OP_DISCOVER, {}, peer, false);
    }
    return last;
  }

  bool senderLooksLocal(const network::NetAddress& sender) const
  {
    if (sender.isLoopback())
      return true;
    return localIps.count(sender.address) != 0;
  }

  void onSyncReadable();
  void onDiscoveryReadable();
  void recvFrom(network::NetSocket& sock);
  void handlePacket(const uint8_t* data, size_t size, const network::NetAddress& sender);
  /// Record a remote sender that advertised this master's GUID. Caller holds guard.
  void noteGuidCollision(const mm::Header& h, const uint8_t* payload, size_t payload_len,
    const network::NetAddress& sender);
  void handleDiscover(const mm::Header& h, const network::NetAddress& sender);
  void handleOffer(const mm::Header& h, const uint8_t* payload, size_t len, const network::NetAddress& sender);
  void handleRequest(const mm::Header& h, const network::NetAddress& sender, std::unique_lock<std::mutex>& lock);
  void handleAck(const mm::Header& h, const network::NetAddress& sender, std::unique_lock<std::mutex>& lock);
  void handleNak(const mm::Header& h, const network::NetAddress& sender);
  void handleBye(const mm::Header& h, std::unique_lock<std::mutex>& lock);
  void handleHeartbeat(const mm::Header& h, const network::NetAddress& sender);
  void handleSync(const mm::Header& h, const uint8_t* payload, size_t len, bool snapshot,
    std::unique_lock<std::mutex>& lock);
  void handleSyncAck(const mm::Header& h, std::unique_lock<std::mutex>& lock);

  Error sendOfferTo(const network::NetAddress& dest, bool viaDiscovery);
  Error sendRequestTo(Peer& peer);
  Error sendAckTo(const network::NetAddress& dest, uint32_t ackSeq);
  Error sendHeartbeatTo(Peer& peer);
  /// Send full registration snapshot. Releases @p lock while collecting local regs
  /// (those callbacks take RegistrationManager / NodeRef locks).
  Error sendSnapshotTo(Peer& peer, std::unique_lock<std::mutex>& lock);
  Error sendDeltaTo(Peer& peer, const miniros_msgs::RegistrationRecord& record);

  /// @return true if this call transitioned the peer into Paired.
  bool markPaired(Peer& peer);
  void markStale(Peer& peer, const std::string& reason, std::unique_lock<std::mutex>& lock);
  bool topicAllowed(const std::string& name) const;
  bool tokenAccepts(const std::array<uint8_t, mm::kTokenHashSize>& remote) const;
  void applyTokenUnlocked(const std::string& newToken);
  /// Leave collective; caller must hold guard. Returns peers to drop after unlock.
  std::vector<UUID> disconnectAllUnlocked();
  network::NetAddress syncAddressFromOffer(const miniros_msgs::MasterOffer& offer, const network::NetAddress& sender) const;
};

void MultimasterManager::Internal::recvFrom(network::NetSocket& sock)
{
  std::string raw;
  network::NetAddress sender;
  auto [n, err] = sock.recv(raw, &sender);
  if (err != Error::Ok) {
    MINIROS_ERROR_NAMED("multimaster", "UDP recv failed: %s", err.toString());
    return;
  }
  if (n == 0)
    return;
  handlePacket(reinterpret_cast<const uint8_t*>(raw.data()), raw.size(), sender);
}

void MultimasterManager::Internal::onSyncReadable()
{
  recvFrom(syncSocket);
}

void MultimasterManager::Internal::onDiscoveryReadable()
{
  recvFrom(discoverySocket);
}

network::NetAddress MultimasterManager::Internal::syncAddressFromOffer(const miniros_msgs::MasterOffer& offer,
  const network::NetAddress& sender) const
{
  const std::string host = offer.host.empty() ? sender.address : offer.host;
  const int port = offer.master_port ? offer.master_port : boundPort;
  network::NetAddress addr = network::NetAddress::fromIp4String(host, port);
  if (!addr.valid())
    addr = network::NetAddress::fromString(network::NetAddress::AddressUnspecified, host, port);
  if (!addr.valid())
    return sender;
  return addr;
}

bool MultimasterManager::Internal::tokenAccepts(const std::array<uint8_t, mm::kTokenHashSize>& remote) const
{
  // Open mesh: both sides have no token — accept (manual pair / inbound REQUEST only;
  // auto-pair still requires a non-empty shared token).
  if (token.empty())
    return mm::tokenHashEmpty(remote);
  if (mm::tokenHashEmpty(remote))
    return false;
  return mm::tokenHashEqual(token_hash, remote);
}

void MultimasterManager::Internal::applyTokenUnlocked(const std::string& newToken)
{
  token = newToken;
  mm::tokenHash(token, token_hash);
}

std::vector<UUID> MultimasterManager::Internal::disconnectAllUnlocked()
{
  std::vector<UUID> toDrop;
  for (auto& [k, peer] : peers) {
    if (peer.info.state != PeerState::Paired && peer.info.state != PeerState::Requesting)
      continue;
    if (peer.info.lastAddress.valid())
      sendOp(mm::Header::OP_BYE, {}, peer.info.lastAddress, false);
    peer.inflight.clear();
    peer.info.state = PeerState::Discovered;
    toDrop.push_back(peer.info.uuid);
  }
  if (!toDrop.empty() && topologyChanged)
    topologyChanged();
  return toDrop;
}

bool MultimasterManager::Internal::topicAllowed(const std::string& name) const
{
  return localOnlyTopics.count(name) == 0;
}

void MultimasterManager::Internal::handlePacket(const uint8_t* data, size_t size, const network::NetAddress& sender)
{
  mm::Header h;
  const uint8_t* payload = nullptr;
  size_t payload_len = 0;
  if (Error e = mm::parsePacket(data, size, h, payload, payload_len); !e) {
    MINIROS_DEBUG_NAMED("multimaster", "Ignoring bad packet from %s", sender.str().c_str());
    return;
  }
  if (h.version != mm::kVersion)
    return;

  UUID peerUuid = UUID::fromBytes(h.uuid);
  if (peerUuid == uuid) {
    std::lock_guard lock(guard);
    if (!senderLooksLocal(sender))
      noteGuidCollision(h, payload, payload_len, sender);
    return; // echo or cloned identity — do not OFFER / REQUEST / sync
  }

  std::unique_lock lock(guard);

  switch (h.op) {
  case mm::Header::OP_DISCOVER:
    handleDiscover(h, sender);
    break;
  case mm::Header::OP_OFFER:
    handleOffer(h, payload, payload_len, sender);
    break;
  case mm::Header::OP_REQUEST:
    handleRequest(h, sender, lock);
    break;
  case mm::Header::OP_ACK:
    handleAck(h, sender, lock);
    break;
  case mm::Header::OP_NAK:
    handleNak(h, sender);
    break;
  case mm::Header::OP_BYE:
    handleBye(h, lock);
    break;
  case mm::Header::OP_HEARTBEAT:
    handleHeartbeat(h, sender);
    break;
  case mm::Header::OP_SYNC_SNAPSHOT:
    handleSync(h, payload, payload_len, true, lock);
    break;
  case mm::Header::OP_SYNC_DELTA:
    handleSync(h, payload, payload_len, false, lock);
    break;
  case mm::Header::OP_SYNC_ACK:
    handleSyncAck(h, lock);
    break;
  default:
    break;
  }
}

void MultimasterManager::Internal::noteGuidCollision(const mm::Header& h, const uint8_t* payload, size_t payload_len,
  const network::NetAddress& sender)
{
  const std::string key = collisionPeerKey(sender);
  Peer& peer = peers[key];
  const bool firstSeen = peer.info.state != PeerState::GuidCollision;
  peer.info.uuid = uuid;
  peer.info.state = PeerState::GuidCollision;
  peer.info.lastAddress = sender;
  peer.info.lastSeen = SteadyTime::now();
  peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
  peer.info.tokenMatch = tokenAccepts(h.token_hash);

  if (h.op == mm::Header::OP_OFFER && payload && payload_len) {
    miniros_msgs::MasterOffer offer;
    if (mm::deserializePayload(payload, payload_len, offer)) {
      peer.info.masterUri.scheme = "http://";
      peer.info.masterUri.host = offer.host.empty() ? sender.address : offer.host;
      peer.info.masterUri.port = offer.master_port ? offer.master_port : sender.port();
    }
  } else if (peer.info.masterUri.empty()) {
    peer.info.masterUri.scheme = "http://";
    peer.info.masterUri.host = sender.address;
    peer.info.masterUri.port = sender.port();
  }

  if (firstSeen) {
    MINIROS_WARN_NAMED("multimaster",
      "Another master at %s is advertising this master's GUID %s. "
      "It will appear on the status page but cannot be paired. "
      "Delete cache.<port> on that host (or pass --no-cache) so each master gets a unique identity.",
      sender.str().c_str(), uuid.toString().c_str());
  }
}

void MultimasterManager::Internal::handleDiscover(const mm::Header& h, const network::NetAddress& sender)
{
  // Always answer DISCOVER so peers appear on the status page even without a shared token.
  // Do not treat the discovery source port as the sync address — OFFER carries master_port.
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer& peer = peers[uuidKey(peerUuid)];
  if (!peer.info.uuid.valid())
    peer.info.uuid = peerUuid;
  peer.info.lastSeen = SteadyTime::now();
  peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
  peer.info.tokenMatch = tokenAccepts(h.token_hash);
  if (peer.info.state == PeerState::Paired)
    return;
  peer.info.state = PeerState::Offering;
  sendOfferTo(sender, /*viaDiscovery=*/discoverySocket.valid());
}

Error MultimasterManager::Internal::sendOfferTo(const network::NetAddress& dest, bool viaDiscovery)
{
  miniros_msgs::MasterOffer offer;
  offer.master_port = static_cast<uint16_t>(rpcUrl.port ? rpcUrl.port : boundPort);
  offer.host = rpcUrl.host;
  std::vector<uint8_t> payload;
  if (Error e = mm::serializePayload(offer, payload); !e)
    return e;
  return sendOp(mm::Header::OP_OFFER, payload, dest, false, viaDiscovery);
}

void MultimasterManager::Internal::handleOffer(const mm::Header& h, const uint8_t* payload, size_t len,
  const network::NetAddress& sender)
{
  const bool match = tokenAccepts(h.token_hash);
  UUID peerUuid = UUID::fromBytes(h.uuid);

  miniros_msgs::MasterOffer offer;
  network::NetAddress syncAddr = sender;
  if (mm::deserializePayload(payload, len, offer)) {
    syncAddr = syncAddressFromOffer(offer, sender);
    Peer& peer = ensurePeer(peerUuid, syncAddr);
    peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
    peer.info.tokenMatch = match;
    peer.info.masterUri.scheme = "http://";
    peer.info.masterUri.host = offer.host.empty() ? sender.address : offer.host;
    peer.info.masterUri.port = offer.master_port ? offer.master_port : sender.port();

    if (peer.info.state == PeerState::Paired)
      return;

    if (peer.info.state == PeerState::Discovered || peer.info.state == PeerState::Stale ||
        peer.info.state == PeerState::Offering)
      peer.info.state = PeerState::Discovered;

    // Auto-pair only when a non-empty collective token is configured.
    if (match && !token.empty()) {
      peer.info.state = PeerState::Requesting;
      sendRequestTo(peer);
    }
    return;
  }

  Peer& peer = ensurePeer(peerUuid, syncAddr);
  peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
  peer.info.tokenMatch = match;

  if (peer.info.state == PeerState::Paired)
    return;

  if (peer.info.state == PeerState::Discovered || peer.info.state == PeerState::Stale ||
      peer.info.state == PeerState::Offering)
    peer.info.state = PeerState::Discovered;

  if (match && !token.empty()) {
    peer.info.state = PeerState::Requesting;
    sendRequestTo(peer);
  }
}

Error MultimasterManager::Internal::sendRequestTo(Peer& peer)
{
  if (peer.info.state == PeerState::GuidCollision)
    return Error::PermissionDenied;
  if (!peer.info.lastAddress.valid())
    return Error::InvalidAddress;
  peer.info.state = PeerState::Requesting;
  return sendOp(mm::Header::OP_REQUEST, {}, peer.info.lastAddress, true);
}

void MultimasterManager::Internal::handleRequest(const mm::Header& h, const network::NetAddress& sender,
  std::unique_lock<std::mutex>& lock)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer& peer = ensurePeer(peerUuid, sender);
  peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
  peer.info.tokenMatch = tokenAccepts(h.token_hash);

  // Accept when fingerprints match (including both-empty for open meshes).
  if (!tokenAccepts(h.token_hash)) {
    MINIROS_INFO_NAMED("multimaster", "Rejecting pair request from %s (token mismatch)",
      peerUuid.toString().c_str());
    sendOp(mm::Header::OP_NAK, {}, sender, false);
    if (peer.info.state != PeerState::Paired)
      peer.info.state = PeerState::Discovered;
    return;
  }

  sendAckTo(sender, h.seq);
  if (markPaired(peer))
    sendSnapshotTo(peer, lock);
}

void MultimasterManager::Internal::handleAck(const mm::Header& h, const network::NetAddress& sender,
  std::unique_lock<std::mutex>& lock)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer& peer = ensurePeer(peerUuid, sender);
  peer.info.remoteHasToken = !mm::tokenHashEmpty(h.token_hash);
  peer.info.tokenMatch = tokenAccepts(h.token_hash);
  if (markPaired(peer))
    sendSnapshotTo(peer, lock);
}

void MultimasterManager::Internal::handleNak(const mm::Header& h, const network::NetAddress& sender)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  if (Peer* peer = findPeer(peerUuid)) {
    peer->info.lastAddress = sender;
    peer->info.state = PeerState::Discovered;
    MINIROS_INFO_NAMED("multimaster", "Peer %s rejected pair request", peerUuid.toString().c_str());
  }
}

void MultimasterManager::Internal::handleBye(const mm::Header& h, std::unique_lock<std::mutex>& lock)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  if (Peer* peer = findPeer(peerUuid))
    markStale(*peer, "bye", lock);
}

void MultimasterManager::Internal::handleHeartbeat(const mm::Header& h, const network::NetAddress& sender)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer& peer = ensurePeer(peerUuid, sender);
  if (peer.info.state == PeerState::Paired || peer.info.state == PeerState::Stale) {
    peer.info.state = PeerState::Paired;
  }
  if (h.flags & mm::Header::FLAG_NEEDS_ACK)
    sendAckTo(sender, h.seq);
}

void MultimasterManager::Internal::handleSync(const mm::Header& h, const uint8_t* payload, size_t len, bool snapshot,
  std::unique_lock<std::mutex>& lock)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer* peer = findPeer(peerUuid);
  if (!peer || peer->info.state != PeerState::Paired) {
    if (!tokenAccepts(h.token_hash))
      return;
    network::NetAddress addr = peer ? peer->info.lastAddress : network::NetAddress();
    peer = &ensurePeer(peerUuid, addr);
    markPaired(*peer);
  }
  peer->info.lastSeen = SteadyTime::now();
  network::NetAddress peerAddr = peer->info.lastAddress;

  miniros_msgs::MasterSync sync;
  if (Error e = mm::deserializePayload(payload, len, sync); !e) {
    MINIROS_WARN_NAMED("multimaster", "Failed to decode sync from %s", peerUuid.toString().c_str());
    return;
  }

  std::vector<miniros_msgs::RegistrationRecord> filtered;
  filtered.reserve(sync.records.size());
  for (auto& r : sync.records) {
    if (!topicAllowed(r.name))
      continue;
    filtered.push_back(std::move(r));
  }

  if (h.flags & mm::Header::FLAG_NEEDS_ACK) {
    mm::Header ah;
    fillHeader(ah, mm::Header::OP_SYNC_ACK, 0, next_control_seq++);
    ah.ack_seq = h.seq;
    std::vector<uint8_t> packet;
    if (mm::buildPacket(ah, {}, packet))
      sendRaw(syncSocket, packet, peerAddr);
  }

  if (snapshot) {
    peer->info.foreignPubs = 0;
    peer->info.foreignSubs = 0;
    peer->info.foreignSrvs = 0;
  }
  for (const auto& r : filtered) {
    switch (r.kind) {
    case miniros_msgs::RegistrationRecord::KIND_PUB_REGISTER:
      ++peer->info.foreignPubs;
      break;
    case miniros_msgs::RegistrationRecord::KIND_PUB_UNREGISTER:
      if (peer->info.foreignPubs)
        --peer->info.foreignPubs;
      break;
    case miniros_msgs::RegistrationRecord::KIND_SUB_REGISTER:
      ++peer->info.foreignSubs;
      break;
    case miniros_msgs::RegistrationRecord::KIND_SUB_UNREGISTER:
      if (peer->info.foreignSubs)
        --peer->info.foreignSubs;
      break;
    case miniros_msgs::RegistrationRecord::KIND_SRV_REGISTER:
      ++peer->info.foreignSrvs;
      break;
    case miniros_msgs::RegistrationRecord::KIND_SRV_UNREGISTER:
      if (peer->info.foreignSrvs)
        --peer->info.foreignSrvs;
      break;
    }
  }

  if (applyRecords && !filtered.empty()) {
    ApplyRecordsFn fn = applyRecords;
    lock.unlock();
    fn(peerUuid, filtered, snapshot);
    lock.lock();
  }
}

void MultimasterManager::Internal::handleSyncAck(const mm::Header& h,
  std::unique_lock<std::mutex>& lock)
{
  UUID peerUuid = UUID::fromBytes(h.uuid);
  Peer* peer = findPeer(peerUuid);
  if (!peer)
    return;
  peer->info.lastSeen = SteadyTime::now();

  auto it = peer->inflight.find(h.ack_seq);
  const bool wasSnapshot = it != peer->inflight.end() && it->second.isSnapshot;
  if (it != peer->inflight.end())
    peer->inflight.erase(it);

  if (wasSnapshot) {
    peer->snapshotInFlight = false;
    // Local regs changed while the snapshot was in flight (or during collect):
    // send a fresh authoritative snapshot so deltas cannot race ahead of it.
    if (localRegGen != peer->snapshotCollectGen && peer->info.state == PeerState::Paired)
      sendSnapshotTo(*peer, lock);
  }
}

Error MultimasterManager::Internal::sendAckTo(const network::NetAddress& dest, uint32_t ackSeq)
{
  mm::Header h;
  fillHeader(h, mm::Header::OP_ACK, 0, next_control_seq++);
  h.ack_seq = ackSeq;
  std::vector<uint8_t> packet;
  if (Error e = mm::buildPacket(h, {}, packet); !e)
    return e;
  return sendRaw(syncSocket, packet, dest);
}

Error MultimasterManager::Internal::sendHeartbeatTo(Peer& peer)
{
  if (!peer.info.lastAddress.valid())
    return Error::InvalidAddress;
  return sendOp(mm::Header::OP_HEARTBEAT, {}, peer.info.lastAddress, false);
}

Error MultimasterManager::Internal::sendSnapshotTo(Peer& peer, std::unique_lock<std::mutex>& lock)
{
  if (!collectSnapshot || !peer.info.lastAddress.valid())
    return Error::Ok;

  // collectSnapshot walks RegistrationManager / NodeRef locks. Never hold
  // multimaster::guard across that callback (ABBA with Master::update).
  CollectSnapshotFn fn = collectSnapshot;
  const UUID peerUuid = peer.info.uuid;
  network::NetAddress dest = peer.info.lastAddress;

  // Block deltas until this snapshot is ACKed; otherwise a delta can land on the
  // peer first and then be wiped when an older snapshot applies (drop+replace).
  peer.snapshotInFlight = true;

  std::vector<miniros_msgs::RegistrationRecord> all;
  uint64_t gen = 0;
  do {
    gen = localRegGen;
    lock.unlock();
    all = fn();
    lock.lock();
  } while (gen != localRegGen);

  Peer* live = findPeer(peerUuid);
  if (!live || live->info.state != PeerState::Paired) {
    if (live)
      live->snapshotInFlight = false;
    return Error::Ok;
  }
  if (!live->info.lastAddress.valid()) {
    live->snapshotInFlight = false;
    return Error::Ok;
  }
  dest = live->info.lastAddress;
  live->snapshotCollectGen = gen;

  miniros_msgs::MasterSync sync;
  sync.fragment_index = 0;
  sync.fragment_count = 1;
  for (auto& r : all) {
    if (topicAllowed(r.name))
      sync.records.push_back(std::move(r));
  }
  std::vector<uint8_t> payload;
  if (Error e = mm::serializePayload(sync, payload); !e) {
    live->snapshotInFlight = false;
    return e;
  }
  // TODO: fragment if payload > kMaxPayload
  mm::Header h;
  uint32_t seq = live->next_tx_seq++;
  fillHeader(h, mm::Header::OP_SYNC_SNAPSHOT, mm::Header::FLAG_NEEDS_ACK, seq);
  std::vector<uint8_t> packet;
  if (Error e = mm::buildPacket(h, payload, packet); !e) {
    live->snapshotInFlight = false;
    return e;
  }
  if (Error e = sendRaw(syncSocket, packet, dest); !e) {
    live->snapshotInFlight = false;
    return e;
  }
  PendingSend ps;
  ps.packet = std::move(packet);
  ps.dest = dest;
  ps.seq = seq;
  ps.nextRetry = SteadyTime::now() + WallDuration(0.75);
  ps.isSnapshot = true;
  live->inflight[seq] = std::move(ps);
  return Error::Ok;
}

Error MultimasterManager::Internal::sendDeltaTo(Peer& peer, const miniros_msgs::RegistrationRecord& record)
{
  if (!topicAllowed(record.name) || peer.info.state != PeerState::Paired)
    return Error::Ok;
  miniros_msgs::MasterSync sync;
  sync.fragment_index = 0;
  sync.fragment_count = 1;
  sync.records.push_back(record);
  std::vector<uint8_t> payload;
  if (Error e = mm::serializePayload(sync, payload); !e)
    return e;
  mm::Header h;
  uint32_t seq = peer.next_tx_seq++;
  fillHeader(h, mm::Header::OP_SYNC_DELTA, mm::Header::FLAG_NEEDS_ACK, seq);
  std::vector<uint8_t> packet;
  if (Error e = mm::buildPacket(h, payload, packet); !e)
    return e;
  if (Error e = sendRaw(syncSocket, packet, peer.info.lastAddress); !e)
    return e;
  PendingSend ps;
  ps.packet = packet;
  ps.dest = peer.info.lastAddress;
  ps.seq = seq;
  ps.nextRetry = SteadyTime::now() + WallDuration(0.5);
  peer.inflight[seq] = std::move(ps);
  return Error::Ok;
}

bool MultimasterManager::Internal::markPaired(Peer& peer)
{
  const bool was = peer.info.state == PeerState::Paired;
  peer.info.state = PeerState::Paired;
  peer.info.lastSeen = SteadyTime::now();
  peer.nextHeartbeat = SteadyTime::now() + WallDuration(2.0);
  if (!was) {
    MINIROS_INFO_NAMED("multimaster", "Paired with master %s at %s", peer.info.uuid.toString().c_str(),
      peer.info.masterUri.str().c_str());
    if (topologyChanged)
      topologyChanged();
  }
  return !was;
}

void MultimasterManager::Internal::markStale(Peer& peer, const std::string& reason, std::unique_lock<std::mutex>& lock)
{
  if (peer.info.state == PeerState::Stale)
    return;
  MINIROS_WARN_NAMED("multimaster", "Peer %s stale (%s)", peer.info.uuid.toString().c_str(), reason.c_str());
  peer.info.state = PeerState::Stale;
  peer.inflight.clear();
  peer.snapshotInFlight = false;
  peer.snapshotCollectGen = 0;
  UUID id = peer.info.uuid;
  DropPeerFn fn = dropPeer;
  auto onChanged = topologyChanged;
  if (fn) {
    lock.unlock();
    fn(id);
    lock.lock();
  }
  if (onChanged)
    onChanged();
}

// --- MultimasterManager public API ---

MultimasterManager::MultimasterManager(AddressResolver* resolver, RegistrationManager* regs)
{
  internal_ = std::make_unique<Internal>(resolver, regs);
}

MultimasterManager::~MultimasterManager()
{
  stop();
}

void MultimasterManager::setToken(const std::string& token)
{
  if (!internal_)
    return;

  std::vector<UUID> toDrop;
  DropPeerFn dropFn;
  bool rediscover = false;
  {
    std::unique_lock lock(internal_->guard);
    if (token == internal_->token)
      return;

    if (internal_->started) {
      toDrop = internal_->disconnectAllUnlocked();
      dropFn = internal_->dropPeer;
      rediscover = true;
    }
    internal_->applyTokenUnlocked(token);
    if (rediscover)
      internal_->nextDiscover = SteadyTime::now();
  }

  if (dropFn) {
    for (const UUID& id : toDrop)
      dropFn(id);
  }
  if (rediscover)
    sendDiscover();
}

void MultimasterManager::setUdpPort(int port)
{
  if (!internal_)
    return;
  internal_->configuredPort = port;
}

void MultimasterManager::setMulticast(const std::string& addr, int port)
{
  if (!internal_)
    return;

  std::vector<UUID> toDrop;
  DropPeerFn dropFn;
  bool rejoin = false;
  {
    std::unique_lock lock(internal_->guard);
    const bool enable = !addr.empty() && port > 0;
    if (internal_->multicastHost == addr && internal_->multicastPort == port &&
        internal_->multicastEnabled == enable)
      return;

    if (internal_->started) {
      toDrop = internal_->disconnectAllUnlocked();
      dropFn = internal_->dropPeer;
      internal_->detachSocket(internal_->discoverySocket);
      internal_->multicastHost = addr;
      internal_->multicastPort = port;
      internal_->multicastEnabled = enable;
      if (Error e = internal_->initDiscoverySocket(); !e) {
        MINIROS_ERROR_NAMED("multimaster", "Failed to rejoin multicast: %s", e.toString());
      }
      rejoin = true;
      internal_->nextDiscover = SteadyTime::now();
    } else {
      internal_->multicastHost = addr;
      internal_->multicastPort = port;
      internal_->multicastEnabled = enable;
    }
  }

  if (dropFn) {
    for (const UUID& id : toDrop)
      dropFn(id);
  }
  if (rejoin)
    sendDiscover();
}

void MultimasterManager::setLocalOnlyTopics(std::set<std::string> topics)
{
  if (!internal_)
    return;
  std::lock_guard lock(internal_->guard);
  internal_->localOnlyTopics = std::move(topics);
}

void MultimasterManager::setCollectSnapshot(CollectSnapshotFn fn)
{
  if (internal_)
    internal_->collectSnapshot = std::move(fn);
}

void MultimasterManager::setApplyRecords(ApplyRecordsFn fn)
{
  if (internal_)
    internal_->applyRecords = std::move(fn);
}

void MultimasterManager::setDropPeer(DropPeerFn fn)
{
  if (internal_)
    internal_->dropPeer = std::move(fn);
}

Error MultimasterManager::start(PollSet* pollSet, const UUID& uuid, const network::URL& rpcUrl)
{
  if (!internal_)
    return Error::InternalError;
  if (!uuid.valid())
    return Error::InvalidValue;
  if (!pollSet)
    return Error::InvalidValue;

  std::lock_guard lock(internal_->guard);
  if (internal_->started)
    return Error::Ok;

  internal_->pollSet = pollSet;
  internal_->uuid = uuid;
  internal_->rpcUrl = rpcUrl;

  int port = internal_->configuredPort;
  if (port <= 0)
    port = rpcUrl.port > 0 ? rpcUrl.port : 11311;

  if (Error e = internal_->initSyncSocket(port); !e) {
    MINIROS_ERROR_NAMED("multimaster", "Failed to bind sync UDP %d: %s", port, e.toString());
    internal_->detachSockets();
    return e;
  }

  if (Error e = internal_->initDiscoverySocket(); !e) {
    // Keep the configured multicast endpoint; retry from refreshNetworkUnlocked()
    // once ethernet (or another NIC) comes up. Until then fall back to UDP broadcast.
    // initDiscoverySocket() already closed discoverySocket on failure.
    internal_->multicastError = e.toString();
    MINIROS_WARN_NAMED("multimaster",
      "Failed to join multicast discovery: %s (continuing with UDP broadcast; will retry when NICs appear)",
      e.toString());
  }

  internal_->refreshLocalIps();
  internal_->started = true;

  if (Error e = internal_->networkMonitor.start(pollSet,
        [this](const network::NetworkMonitor::Event& ev) {
          if (internal_)
            internal_->onNetworkEvent(ev);
        }); !e) {
    if (e == Error::NotSupported) {
      MINIROS_DEBUG_NAMED("multimaster",
        "Netlink network monitor unavailable; using periodic NIC polling");
    } else {
      MINIROS_WARN_NAMED("multimaster",
        "Failed to start netlink network monitor: %s (using periodic NIC polling)",
        e.toString());
    }
  }

  MINIROS_INFO_NAMED("multimaster",
    "Listening sync UDP %d, multicast=%s (token=%s, discovery=on, guid=%s, netlink=%s)",
    internal_->boundPort,
    internal_->multicastEnabled
      ? (internal_->multicastHost + ":" + std::to_string(internal_->multicastPort)).c_str()
      : "off",
    internal_->token.empty() ? "none" : "set",
    internal_->uuid.toString().c_str(),
    internal_->networkMonitor.running() ? "on" : "off");

  internal_->discoveryEnabled = true;
  internal_->nextDiscover = SteadyTime::now();
  internal_->nextNetworkRefresh = SteadyTime::now();
  return Error::Ok;
}

void MultimasterManager::stop()
{
  if (!internal_)
    return;
  std::lock_guard lock(internal_->guard);
  if (!internal_->started)
    return;
  for (auto& [k, peer] : internal_->peers) {
    if (peer.info.state == PeerState::Paired && peer.info.lastAddress.valid()) {
      internal_->sendOp(mm::Header::OP_BYE, {}, peer.info.lastAddress, false);
    }
  }
  internal_->networkMonitor.stop();
  internal_->detachSockets();
  internal_->peers.clear();
  internal_->started = false;
}

Error MultimasterManager::sendDiscover()
{
  if (!internal_ || !internal_->started)
    return Error::NotConnected;
  std::lock_guard lock(internal_->guard);
  if (!internal_->discoveryEnabled)
    return Error::Ok;
  return internal_->sendDiscoverUnlocked();
}

Error MultimasterManager::addPeerProbe(const network::NetAddress& addr)
{
  if (!internal_)
    return Error::InternalError;
  if (!addr.valid())
    return Error::InvalidAddress;
  std::lock_guard lock(internal_->guard);
  for (const auto& existing : internal_->peerProbes) {
    if (existing == addr)
      return Error::Ok;
  }
  internal_->peerProbes.push_back(addr);
  return Error::Ok;
}

Error MultimasterManager::requestPair(const UUID& peerUuid, const std::string& token)
{
  if (!internal_)
    return Error::InternalError;

  std::vector<UUID> toDrop;
  DropPeerFn dropFn;
  {
    std::unique_lock lock(internal_->guard);
    if (peerUuid == internal_->uuid)
      return Error::PermissionDenied;
    if (!token.empty() && token != internal_->token) {
      toDrop = internal_->disconnectAllUnlocked();
      dropFn = internal_->dropPeer;
      internal_->applyTokenUnlocked(token);
    } else if (!token.empty()) {
      internal_->applyTokenUnlocked(token);
    }
    // Empty local token is allowed for open (no-token) meshes; tokenized peers
    // still require a matching token via the connect form / CLI.
    Peer* peer = internal_->findPeer(peerUuid);
    if (!peer)
      return Error::FileNotFound;
    if (peer->info.state == PeerState::GuidCollision)
      return Error::PermissionDenied;
    Error err = internal_->sendRequestTo(*peer);
    lock.unlock();
    if (dropFn) {
      for (const UUID& id : toDrop)
        dropFn(id);
    }
    return err;
  }
}

Error MultimasterManager::requestPairByNodeName(const std::string& nodeName, const std::string& token)
{
  if (!internal_ || !internal_->regs)
    return Error::InternalError;
  auto node = internal_->regs->getNodeByName(nodeName);
  if (!node)
    return Error::FileNotFound;
  if (node->getNodeFlags() & NodeRef::NODE_LOCAL)
    return Error::PermissionDenied;

  std::vector<UUID> toDrop;
  DropPeerFn dropFn;
  std::unique_lock lock(internal_->guard);
  if (!token.empty() && token != internal_->token) {
    toDrop = internal_->disconnectAllUnlocked();
    dropFn = internal_->dropPeer;
    internal_->applyTokenUnlocked(token);
  } else if (!token.empty()) {
    internal_->applyTokenUnlocked(token);
  }

  Error err = Error::FileNotFound;
  bool found = false;
  for (auto& [k, peer] : internal_->peers) {
    if (peer.info.state == PeerState::GuidCollision)
      continue;
    if (peer.info.masterUri.str() == node->getApi() ||
        peer.info.lastAddress.address == node->getHost()) {
      err = internal_->sendRequestTo(peer);
      found = true;
      break;
    }
  }
  if (!found) {
    network::URL url = node->getUrl();
    if (url.host == internal_->rpcUrl.host && url.port == internal_->rpcUrl.port)
      return Error::PermissionDenied;
    network::NetAddress addr = network::NetAddress::fromURL(url);
    if (!addr.valid()) {
      addr = network::NetAddress::fromIp4String(url.host, internal_->boundPort);
    } else {
      addr.setPort(internal_->boundPort);
    }
    UUID empty;
    Peer& peer = internal_->ensurePeer(empty, addr);
    peer.info.masterUri = url;
    peer.info.state = PeerState::Requesting;
    err = internal_->sendRequestTo(peer);
  }
  lock.unlock();
  if (dropFn) {
    for (const UUID& id : toDrop)
      dropFn(id);
  }
  return err;
}

Error MultimasterManager::disconnectAll()
{
  if (!internal_)
    return Error::InternalError;
  std::unique_lock lock(internal_->guard);
  if (!internal_->started)
    return Error::NotConnected;

  std::vector<UUID> toDrop = internal_->disconnectAllUnlocked();
  DropPeerFn fn = internal_->dropPeer;
  lock.unlock();
  if (fn) {
    for (const UUID& id : toDrop)
      fn(id);
  }
  MINIROS_INFO_NAMED("multimaster", "Disconnected from %zu peer master(s)", toDrop.size());
  return Error::Ok;
}

void MultimasterManager::announceLocalChange(const miniros_msgs::RegistrationRecord& record)
{
  if (!internal_)
    return;
  std::lock_guard lock(internal_->guard);
  if (!internal_->topicAllowed(record.name))
    return;
  ++internal_->localRegGen;
  for (auto& [k, peer] : internal_->peers) {
    if (peer.info.state != PeerState::Paired)
      continue;
    // Snapshot still unacked: a delta can arrive before the older snapshot and
    // then be erased by dropMultimasterPeer. Catch up with another snapshot on ACK.
    if (peer.snapshotInFlight)
      continue;
    internal_->sendDeltaTo(peer, record);
  }
}

void MultimasterManager::setTopologyChanged(std::function<void()> fn)
{
  if (!internal_)
    return;
  std::lock_guard lock(internal_->guard);
  internal_->topologyChanged = std::move(fn);
}

void MultimasterManager::restoreCachedPeers(const std::vector<CachedPeer>& peers)
{
  if (!internal_ || peers.empty())
    return;

  std::unique_lock lock(internal_->guard);
  if (!internal_->started)
    return;

  for (const CachedPeer& cp : peers) {
    UUID uuid;
    if (!uuid.fromString(cp.uuid)) {
      MINIROS_WARN_NAMED("multimaster", "Skipping cached peer with invalid uuid \"%s\"", cp.uuid.c_str());
      continue;
    }
    if (uuid == internal_->uuid)
      continue;

    network::NetAddress addr = network::NetAddress::fromIp4String(cp.sync_host, cp.sync_port);
    if (!addr.valid())
      addr = network::NetAddress::fromString(network::NetAddress::AddressUnspecified, cp.sync_host, cp.sync_port);
    if (!addr.valid()) {
      MINIROS_WARN_NAMED("multimaster", "Skipping cached peer %s: bad sync address %s:%d",
                         cp.uuid.c_str(), cp.sync_host.c_str(), cp.sync_port);
      continue;
    }

    bool knownProbe = false;
    for (const auto& existing : internal_->peerProbes) {
      if (existing == addr) {
        knownProbe = true;
        break;
      }
    }
    if (!knownProbe)
      internal_->peerProbes.push_back(addr);

    Peer& peer = internal_->ensurePeer(uuid, addr);
    if (!cp.uri.empty())
      peer.info.masterUri.fromString(cp.uri, true);
    peer.info.lastAddress = addr;
    peer.info.state = PeerState::Requesting;
    MINIROS_INFO_NAMED("multimaster", "Restoring pairing toward %s at %s (uri=%s)",
                       cp.uuid.c_str(), addr.str().c_str(), cp.uri.c_str());
    if (Error err = internal_->sendRequestTo(peer); !err) {
      MINIROS_WARN_NAMED("multimaster", "Failed to REQUEST restored peer %s: %s",
                         cp.uuid.c_str(), err.toString());
    }
  }
  internal_->nextDiscover = SteadyTime::now();
}

std::vector<CachedPeer> MultimasterManager::collectCachedPeers() const
{
  std::vector<CachedPeer> out;
  if (!internal_)
    return out;
  std::lock_guard lock(internal_->guard);
  for (const auto& [k, peer] : internal_->peers) {
    if (peer.info.state != PeerState::Paired && peer.info.state != PeerState::Requesting)
      continue;
    if (!peer.info.uuid.valid() || !peer.info.lastAddress.valid())
      continue;
    CachedPeer cp;
    cp.uuid = peer.info.uuid.toString();
    cp.uri = peer.info.masterUri.str();
    cp.sync_host = peer.info.lastAddress.address;
    cp.sync_port = peer.info.lastAddress.port();
    cp.state = peerStateName(peer.info.state);
    if (!cp.uuid.empty() && !cp.sync_host.empty() && cp.sync_port > 0)
      out.push_back(std::move(cp));
  }
  return out;
}

void MultimasterManager::update()
{
  if (!internal_ || !internal_->started)
    return;
  std::unique_lock lock(internal_->guard);
  const SteadyTime now = SteadyTime::now();

  // Masters often start before ethernet is configured (embedded boards). Re-scan NICs
  // so multicast membership and subnet broadcasts catch up without a restart.
  // With netlink, events drive refresh; keep a slow poll as a safety net.
  if (now >= internal_->nextNetworkRefresh) {
    internal_->refreshNetworkUnlocked();
    const double period = internal_->networkMonitor.running() ? 30.0 : 5.0;
    internal_->nextNetworkRefresh = now + WallDuration(period);
  }

  if (internal_->discoveryEnabled && now >= internal_->nextDiscover) {
    internal_->sendDiscoverUnlocked();
    // DHCP-like: rediscover on backoff when unpaired; rare keep-alive search when paired.
    bool anyPaired = false;
    for (const auto& [k, p] : internal_->peers) {
      if (p.info.state == PeerState::Paired) {
        anyPaired = true;
        break;
      }
    }
    internal_->nextDiscover = now + (anyPaired ? WallDuration(30.0) : WallDuration(5.0));
  }

  const WallDuration collisionTimeout(15.0);
  for (auto it = internal_->peers.begin(); it != internal_->peers.end(); ) {
    if (it->second.info.state == PeerState::GuidCollision &&
        now - it->second.info.lastSeen > collisionTimeout) {
      it = internal_->peers.erase(it);
    } else {
      ++it;
    }
  }

  const WallDuration peerTimeout(10.0);
  for (auto& [k, peer] : internal_->peers) {
    if (peer.info.state == PeerState::Paired) {
      if (now - peer.info.lastSeen > peerTimeout) {
        internal_->markStale(peer, "heartbeat timeout", lock);
        if (internal_->discoveryEnabled)
          internal_->nextDiscover = now;
        continue;
      }
      if (now >= peer.nextHeartbeat) {
        internal_->sendHeartbeatTo(peer);
        peer.nextHeartbeat = now + WallDuration(2.0);
      }
      for (auto it = peer.inflight.begin(); it != peer.inflight.end(); ) {
        PendingSend& ps = it->second;
        if (now < ps.nextRetry) {
          ++it;
          continue;
        }
        if (ps.attempts < 5) {
          internal_->sendRaw(internal_->syncSocket, ps.packet, ps.dest);
          ps.attempts++;
          ps.nextRetry = now + WallDuration(0.5 * (1 + ps.attempts));
          ++it;
          continue;
        }
        const bool wasSnapshot = ps.isSnapshot;
        it = peer.inflight.erase(it);
        if (wasSnapshot) {
          peer.snapshotInFlight = false;
          if (internal_->localRegGen != peer.snapshotCollectGen)
            internal_->sendSnapshotTo(peer, lock);
        }
      }
    }
  }
}

std::vector<PeerInfo> MultimasterManager::listPeers() const
{
  std::vector<PeerInfo> out;
  if (!internal_)
    return out;
  std::lock_guard lock(internal_->guard);
  out.reserve(internal_->peers.size());
  for (const auto& [k, p] : internal_->peers)
    out.push_back(p.info);
  return out;
}

UUID MultimasterManager::localUuid() const
{
  if (!internal_)
    return {};
  std::lock_guard lock(internal_->guard);
  return internal_->uuid;
}

bool MultimasterManager::hasToken() const
{
  return internal_ && !internal_->token.empty();
}

bool MultimasterManager::hasPairedPeers() const
{
  if (!internal_)
    return false;
  std::lock_guard lock(internal_->guard);
  for (const auto& [k, p] : internal_->peers) {
    if (p.info.state == PeerState::Paired)
      return true;
  }
  return false;
}

int MultimasterManager::udpPort() const
{
  return internal_ ? internal_->boundPort : 0;
}

std::string MultimasterManager::multicastEndpoint() const
{
  if (!internal_ || !internal_->multicastEnabled)
    return {};
  return internal_->multicastHost + ":" + std::to_string(internal_->multicastPort);
}

std::string MultimasterManager::multicastError() const
{
  if (!internal_)
    return {};
  std::lock_guard lock(internal_->guard);
  if (!internal_->multicastEnabled)
    return {};
  // Sticky boot-time / no-NIC error until discovery socket is actually up.
  if (internal_->discoverySocket.valid() && internal_->multicastGroup.valid())
    return {};
  return internal_->multicastError;
}

const char* MultimasterManager::peerStateName(PeerState s)
{
  switch (s) {
  case PeerState::Discovered:
    return "discovered";
  case PeerState::Offering:
    return "offering";
  case PeerState::Requesting:
    return "requesting";
  case PeerState::Paired:
    return "paired";
  case PeerState::Stale:
    return "stale";
  case PeerState::GuidCollision:
    return "guid-collision";
  }
  return "unknown";
}

} // namespace master
} // namespace miniros
