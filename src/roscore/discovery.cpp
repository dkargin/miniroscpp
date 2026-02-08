//
// Created by dkargin on 8/24/25.
//

#include <cassert>

#include "internal_config.h"

#include "discovery.h"

#include "miniros/network/net_adapter.h"
#include "miniros/network/socket.h"
#include "miniros/io/io.h"
#include "miniros/io/poll_set.h"

#include "resolver.h"

namespace miniros {

namespace master {

constexpr int DHCP_UDP_PORT = 67;
constexpr int DNS_UDP_PORT = 53;
constexpr int MDNS_UDP_PORT = 5353;
constexpr int NTP_UDP_PORT = 123;

#pragma pack(push, 1)
struct DiscoveryPacket {
  /// Operation
  int16_t op = 0;

  /// Additional flags.
  int16_t flags = 0;

  /// Size of packet.
  int16_t size = 0;

  /// Master RPC port.
  int16_t masterPort = 0;

  /// Internal API version of node.
  int16_t version = 0;

  /// Sender address.
  sockaddr addr{};

  /// UUID of master instance.
  UUID uuid;
};
#pragma pack(pop)

struct Discovery::Internal {
  PollSet* pollSet = nullptr;

  AddressResolver* resolver = nullptr;

  /// Multicast group for discovery packets.
  network::NetAddress multicastGroup;
  bool multicastEnabled = false;

  /// Port for discovery broadcast.
  int broadcastPort = 0;
  /// Pre-computed address for broadcasts.
  network::NetAddress broadcastAddr;

  /// Iterate adapters for direct broadcasting.
  bool useAdapterBroadcasts = false;

  /// Master XMLRPC port to advertise/
  network::NetAddress rpcAddress;

  network::URL rpcUrl;

  /// UUID to be broadcasted.
  UUID uuid;

  /// Regular IPv4 socket for sending broadcasts.
  network::NetSocket socket;

  /// A collection of adapter-related sockets.
  std::multimap<std::string, network::NetSocket> discoverySockets;

  DiscoveryEventCallback callback;

  Internal(AddressResolver* resolver)
    : resolver(resolver)
  {
    assert(resolver);
  }

  NODISCARD Error initSockets(int port);

  void onSocketEvent(network::NetSocket& s, int role, int event);

  void detachSocket(network::NetSocket& s)
  {
    pollSet->delSocket(s.fd());
    s.close();
  }

  void detachSockets()
  {
    if (!pollSet)
      return;
    detachSocket(socket);

    for (auto& [adapter, sock]: discoverySockets) {
      if (sock.valid()) {
        pollSet->delSocket(sock.fd());
        sock.close();
      }
    }
  }
};

Error Discovery::Internal::initSockets(int port)
{
  Error error = Error::Ok;
  error = socket.initUDP(false);
  if (!error) {
    return error;
  }

  error = socket.setReuseAddr(true);
  if (!error) {
    detachSockets();
    return error;
  }

  error = socket.setReusePort(true);
  if (!error) {
    detachSockets();
    return error;
  }

  error = socket.setBroadcast(true);
  if (!error) {
    detachSockets();
    return error;
  }

  error = socket.bind(port);
  if (!error) {
    detachSockets();
    return error;
  }

  // Actual port can change during bind.
  broadcastPort = port == 0 ? socket.port() : port;

  if (multicastGroup.valid()) {
    multicastGroup.setPort(broadcastPort);
    error = socket.joinMulticastGroup(multicastGroup);
    if (!error) {
      MINIROS_WARN("Discovery has failed to join multicast group \"%s\"", multicastGroup.str().c_str());
      multicastEnabled = false;
    } else {
      multicastEnabled = true;
    }
  }

  int fd = socket.fd();

  if (!pollSet->addSocket(fd, PollSet::EventIn,
    [this](int flags)
    {
      if (flags & PollSet::EventIn) {
        onSocketEvent(socket, 0, flags);
      } else {
        MINIROS_WARN("Discovery socket: unexpected event %d", flags);
      }
      return 0;
    }, {}, THIS_LOCATION))
  {
    MINIROS_ERROR("Failed to register listening socket");
    return Error::InternalError;
  }

  return Error::Ok;
}

void Discovery::Internal::onSocketEvent(network::NetSocket& s, int role, int event)
{
  std::string rawData;
  DiscoveryEvent discoveryEvent;
  auto [numBytes, error] = s.recv(rawData, &discoveryEvent.senderAddress);
  if (error != Error::Ok) {
    MINIROS_ERROR("Discovery socket: recv error %s", error.toString());
    return;
  }
  if (numBytes < sizeof(DiscoveryPacket)) {
    MINIROS_ERROR("Unexpected size of incoming discovery packet: %zu", numBytes);
    return;
  }

  if (!discoveryEvent.senderAddress.valid()) {
    MINIROS_ERROR("Discovery socket: address invalid");
    return;
  }

  DiscoveryPacket packet{};
  memcpy(&packet, rawData.data(), sizeof(packet));

  network::NetAddress masterAddr;
  error = network::fillAddress(&packet.addr, masterAddr);
  if (!error) {
    MINIROS_WARN("Failed to extract address from discovery packet from %s: %s", discoveryEvent.senderAddress.str().c_str(), error.toString());
    return;
  }

  discoveryEvent.uuid = packet.uuid;
  discoveryEvent.version = packet.version;
  discoveryEvent.masterUri.scheme = "http://";
  discoveryEvent.masterUri.port = packet.masterPort;
  discoveryEvent.masterUri.host = masterAddr.address;

  MINIROS_INFO("Got broadcast from %s", discoveryEvent.masterUri.host.c_str());
  if (callback) {
    callback(discoveryEvent);
  }
}


Discovery::Discovery(AddressResolver* resolver)
{
  internal_.reset(new Internal(resolver));
}

Discovery::~Discovery()
{
  if (internal_)
    internal_->detachSockets();
}

void Discovery::fillDiscoveryPacket(DiscoveryPacket& packet)
{
  packet.op = 0;
  packet.size = sizeof(packet);
  packet.uuid = internal_->uuid;
  packet.masterPort = internal_->rpcAddress.port();
  packet.version = MINIROS_INTERNAL_API_VERSION;

  // Set packet.addr from rpcAddress
  if (internal_->rpcAddress.valid() && internal_->rpcAddress.rawAddress()) {
    const sockaddr* rpcAddr = static_cast<const sockaddr*>(internal_->rpcAddress.rawAddress());
    size_t addrSize = internal_->rpcAddress.rawAddressSize();
    memcpy(&packet.addr, rpcAddr, addrSize);
  }
}


Error Discovery::start(PollSet* pollSet, const UUID& uuid, const network::URL& rpcUrl)
{
  if (!internal_) {
    return Error::InternalError;
  }

  if (!uuid.valid())
    return Error::InvalidValue;

  internal_->uuid = uuid;
  internal_->pollSet = pollSet;
  internal_->rpcUrl = rpcUrl;
  auto rpcAddr = network::NetAddress::fromURL(rpcUrl);
  if (!rpcAddr.valid()) {
    MINIROS_ERROR("Failed to get address for RPC URL=%s", rpcUrl.str().c_str());
    return Error::InvalidAddress;
  }
  internal_->rpcAddress = rpcAddr;
  internal_->broadcastAddr = network::NetAddress::fromIp4String("255.255.255.255", internal_->broadcastPort);
  assert(internal_->broadcastAddr.valid());

  if (Error err = internal_->initSockets(internal_->broadcastPort); err != Error::Ok) {
    MINIROS_ERROR("Failed to initialize discovery sockets for port %d: %s", internal_->broadcastPort, err.toString());
    return err;
  }

  return Error::Ok;
}

void Discovery::stop()
{
  if (!internal_)
    return;
  internal_->detachSockets();
}

Error Discovery::doBroadcast()
{
  if (!internal_) {
    return Error::InternalError;
  }

  if (!internal_->resolver) {
    return Error::InternalError;
  }
  if (!internal_->socket.valid()) {
    return Error::NotConnected;
  }

  assert(internal_->broadcastPort);

  DiscoveryPacket packet;
  fillDiscoveryPacket(packet);

  if (internal_->multicastEnabled) {
    auto [written, error] = internal_->socket.send(&packet, sizeof(packet), &internal_->multicastGroup);
    if (error != Error::Ok) {
      MINIROS_WARN("Failed to multicast");
    }
  }
  return Error::Ok;
}

void Discovery::setDiscoveryCallback(DiscoveryEventCallback callback)
{
  if (!internal_)
    return;
  internal_->callback = callback;
}

Error Discovery::setMulticast(const std::string& group)
{
  if (!internal_)
    return Error::InternalError;

  if (internal_->multicastEnabled) {
    assert(false);
    // TODO: implement resubscription to multicast group.
    return Error::NotImplemented;
  }
  internal_->multicastGroup = network::NetAddress::fromIp4String(group, 0);
  return Error::Ok;
}

void Discovery::setUdpBroadcasts(int port)
{
  if (!internal_)
    return;
  internal_->broadcastPort = port;
}

}

}