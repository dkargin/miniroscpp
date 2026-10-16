//
// MasterDiscovery: main-thread poll of the multimaster discovery group.
//

#include "miniros/master_discovery.h"

#include "multimaster_protocol.h"

#include "miniros/console.h"
#include "miniros/network/net_adapter.h"
#include "miniros/network/net_address.h"
#include "miniros/network/socket.h"
#include "miniros/network/url.h"

#include "miniros_msgs/MasterOffer.hxx"

#include <map>
#include <set>
#include <utility>

namespace miniros {

namespace {

constexpr int kMaxDatagramsPerUpdate = 64;

bool uuidPresent(const UUID& id)
{
  for (int i = 0; i < UUID::Dim; ++i) {
    if (id.bytes[i] != 0)
      return true;
  }
  return false;
}

bool isLiteralIp(const std::string& host)
{
  return network::NetAddress::fromIp4String(host, 0).valid() ||
         network::NetAddress::fromIp6String(host, 0).valid();
}

/// `MasterOffer.host` is `ip`, `hostname`, or `ip|hostname` (display-only name).
void splitOfferHost(const std::string& host, std::string& ip, std::string& hostname)
{
  ip.clear();
  hostname.clear();
  std::string left = host;
  std::string right;
  if (const auto sep = host.find('|'); sep != std::string::npos) {
    left = host.substr(0, sep);
    right = host.substr(sep + 1);
  }
  if (isLiteralIp(left))
    ip = left;
  else if (!left.empty() && left != "localhost")
    hostname = left;
  if (isLiteralIp(right)) {
    if (ip.empty())
      ip = right;
  } else if (!right.empty() && right != "localhost" && hostname.empty()) {
    hostname = right;
  }
}

bool decodeOffer(const uint8_t* data, size_t size, std::string& ip, std::string& hostname, uint16_t& port)
{
  ip.clear();
  hostname.clear();
  port = 0;
  if (!data || size == 0)
    return false;
  miniros_msgs::MasterOffer offer;
  if (!master::mm::deserializePayload(data, size, offer))
    return false;
  splitOfferHost(offer.host, ip, hostname);
  port = offer.master_port;
  return port != 0 || !ip.empty() || !hostname.empty();
}

void refreshUri(DiscoveredMaster& master)
{
  if (master.address.empty() || master.masterPort == 0) {
    master.uri.clear();
    return;
  }
  network::URL url;
  url.scheme = "http://";
  url.host = master.address;
  url.port = master.masterPort;
  url.path = "/";
  master.uri = url.str();
}

} // namespace

struct MasterDiscovery::Internal {
  std::string groupHost = MasterDiscovery::kDefaultGroup;
  int groupPort = MasterDiscovery::kDefaultPort;
  WallDuration staleTimeout{90.0};

  network::NetSocket socket;
  network::NetAddress group;
  bool started = false;
  bool joinedAny = false;
  bool joinWarned = false;
  std::set<std::string> joinedIfaces;
  SteadyTime nextJoinRefresh;

  std::map<std::string, DiscoveredMaster> masters;

  void refreshJoins()
  {
    if (!socket.valid() || !group.valid())
      return;

    std::vector<network::NetAdapter> adapters;
    if (network::scanAdapters(adapters)) {
      for (const auto& adapter : adapters) {
        if (!adapter.isUp() || adapter.isLoopback() || !adapter.isIPv4())
          continue;
        const std::string key = adapter.address.address;
        if (key.empty() || joinedIfaces.count(key))
          continue;
        if (Error e = socket.joinMulticastGroup(group, adapter.address, /*loop=*/true)) {
          joinedIfaces.insert(key);
          MINIROS_DEBUG_NAMED("discovery", "Joined %s on %s (%s)",
            group.str().c_str(), adapter.name.c_str(), key.c_str());
        } else {
          MINIROS_DEBUG_NAMED("discovery", "Multicast join on %s (%s) failed: %s",
            adapter.name.c_str(), key.c_str(), e.toString());
        }
      }
    }

    if (joinedIfaces.empty() && !joinedAny) {
      if (Error e = socket.joinMulticastGroup(group, /*loop=*/true)) {
        joinedAny = true;
        joinWarned = false;
        MINIROS_DEBUG_NAMED("discovery", "Joined %s on INADDR_ANY", group.str().c_str());
      } else if (!joinWarned) {
        joinWarned = true;
        MINIROS_WARN_NAMED("discovery", "Multicast join %s failed: %s",
          group.str().c_str(), e.toString());
      } else {
        MINIROS_DEBUG_NAMED("discovery", "Multicast join %s still failing: %s",
          group.str().c_str(), e.toString());
      }
    }
  }

  void note(const UUID& id, const std::string& routed, uint16_t port, const std::string& hostname,
    bool hasToken, DiscoveryReport& report)
  {
    const std::string key = id.toString();
    auto it = masters.find(key);
    const bool isNew = it == masters.end();
    DiscoveredMaster& info = isNew ? masters[key] : it->second;
    if (isNew)
      info.uuid = id;

    bool changed = false;
    if (!routed.empty() && routed != info.address) {
      info.address = routed;
      changed = true;
    }
    if (port != 0 && port != info.masterPort) {
      info.masterPort = port;
      changed = true;
    }
    if (!hostname.empty() && hostname != info.hostname) {
      info.hostname = hostname;
      changed = true;
    }
    if (hasToken != info.hasToken) {
      info.hasToken = hasToken;
      changed = true;
    }
    if (isNew || changed)
      refreshUri(info);
    info.lastSeen = SteadyTime::now();

    if (isNew) {
      MINIROS_DEBUG_NAMED("discovery", "Master %s at %s (%s)",
        info.uuid.toString().c_str(), info.uri.c_str(), info.hostname.c_str());
      report.added.push_back(info);
    } else if (changed) {
      report.updated.push_back(info);
    }
  }

  void handleDatagram(const uint8_t* data, size_t size, const network::NetAddress& sender, DiscoveryReport& report)
  {
    master::mm::Header header;
    const uint8_t* payload = nullptr;
    size_t payloadLen = 0;
    if (!master::mm::parsePacket(data, size, header, payload, payloadLen))
      return;
    if (header.version != master::mm::kVersion)
      return;
    if (header.op != master::mm::Header::OP_DISCOVER && header.op != master::mm::Header::OP_OFFER)
      return;

    UUID id = UUID::fromBytes(header.uuid);
    if (!uuidPresent(id))
      return;

    std::string offerIp;
    std::string hostname;
    uint16_t offerPort = 0;
    (void)decodeOffer(payload, payloadLen, offerIp, hostname, offerPort);

    std::string routed;
    if (sender.valid() && !sender.isUnspecified() && !sender.address.empty())
      routed = sender.address;
    if (!offerIp.empty()) {
      network::NetAddress advertised = network::NetAddress::fromIp4String(offerIp, 0);
      if (!advertised.valid())
        advertised = network::NetAddress::fromIp6String(offerIp, 0);
      if (advertised.valid() && !advertised.isLoopback())
        routed = offerIp;
    }

    note(id, routed, offerPort, hostname, !master::mm::tokenHashEmpty(header.token_hash), report);
  }
};

MasterDiscovery::MasterDiscovery()
  : internal_(std::make_unique<Internal>())
{
}

MasterDiscovery::~MasterDiscovery()
{
  stop();
}

Error MasterDiscovery::setGroup(const std::string& address, int port)
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->started)
    return Error::ResourceInUse;
  if (port <= 0 || port > 65535)
    return Error::InvalidValue;
  if (!network::NetAddress::fromIp4String(address, port).valid())
    return Error::InvalidAddress;
  internal_->groupHost = address;
  internal_->groupPort = port;
  return Error::Ok;
}

void MasterDiscovery::setStaleTimeout(WallDuration timeout)
{
  if (!internal_)
    return;
  internal_->staleTimeout = timeout;
}

Error MasterDiscovery::start()
{
  if (!internal_)
    return Error::InternalError;
  if (internal_->started)
    return Error::Ok;

  auto dispose = [this](Error e) {
    internal_->socket.close();
    internal_->joinedIfaces.clear();
    internal_->joinedAny = false;
    internal_->joinWarned = false;
    return e;
  };

  internal_->group = network::NetAddress::fromIp4String(internal_->groupHost, internal_->groupPort);
  if (!internal_->group.valid())
    return Error::InvalidAddress;

  if (Error e = internal_->socket.initUDP(false); !e)
    return dispose(e);
  if (Error e = internal_->socket.setReuseAddr(true); !e)
    return dispose(e);
  if (Error e = internal_->socket.setReusePort(true); !e)
    return dispose(e);
  if (Error e = internal_->socket.setNonBlock(); !e)
    return dispose(e);
  if (Error e = internal_->socket.bind(internal_->groupPort); !e)
    return dispose(e);

  internal_->started = true;
  internal_->refreshJoins();
  internal_->nextJoinRefresh = SteadyTime::now() + WallDuration(5.0);
  return Error::Ok;
}

void MasterDiscovery::stop()
{
  if (!internal_)
    return;
  internal_->socket.close();
  internal_->joinedIfaces.clear();
  internal_->joinedAny = false;
  internal_->joinWarned = false;
  internal_->masters.clear();
  internal_->group = {};
  internal_->started = false;
}

bool MasterDiscovery::started() const
{
  return internal_ && internal_->started;
}

DiscoveryReport MasterDiscovery::update()
{
  DiscoveryReport report;
  if (!internal_ || !internal_->started)
    return report;

  const SteadyTime now = SteadyTime::now();
  if (now >= internal_->nextJoinRefresh) {
    internal_->refreshJoins();
    internal_->nextJoinRefresh = now + WallDuration(5.0);
  }

  for (int i = 0; i < kMaxDatagramsPerUpdate; ++i) {
    std::string raw;
    network::NetAddress sender;
    auto [n, err] = internal_->socket.recv(raw, &sender);
    if (err == Error::WouldBlock || n == 0)
      break;
    if (!err) {
      MINIROS_DEBUG_NAMED("discovery", "UDP recv failed: %s", err.toString());
      break;
    }
    internal_->handleDatagram(reinterpret_cast<const uint8_t*>(raw.data()), n, sender, report);
  }

  if (internal_->staleTimeout > WallDuration(0)) {
    const SteadyTime expiredAt = SteadyTime::now();
    for (auto it = internal_->masters.begin(); it != internal_->masters.end(); ) {
      if (expiredAt - it->second.lastSeen > internal_->staleTimeout) {
        report.removed.push_back(it->second);
        it = internal_->masters.erase(it);
      } else {
        ++it;
      }
    }
  }
  return report;
}

std::vector<DiscoveredMaster> MasterDiscovery::masters() const
{
  std::vector<DiscoveredMaster> out;
  if (!internal_)
    return out;
  out.reserve(internal_->masters.size());
  for (const auto& [key, master] : internal_->masters) {
    (void)key;
    out.push_back(master);
  }
  return out;
}

} // namespace miniros
