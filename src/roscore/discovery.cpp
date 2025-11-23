//
// Created by dkargin on 8/24/25.
//

#include <cassert>

#include "discovery.h"

#include "miniros/network/net_adapter.h"
#include "miniros/network/socket.h"
#include "miniros/transport/io.h"
#include "miniros/transport/poll_set.h"

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

  /// Adapter-based broadcasts.
  bool adapterBroadcasts = false;

  int rpcPort = 0;

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

  if (!pollSet->addSocket(fd, PollSet::EventIn | PollSet::EventError,
    [this](int flags)
    {
      if (flags & PollSet::EventIn) {
        onSocketEvent(socket, 0, flags);
      } else {
        MINIROS_WARN("Discovery socket: unexpected event %d", flags);
      }
      return 0;
    }))
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

  error = network::fillAddress(&packet.addr, discoveryEvent.masterAddress);
  if (!error) {
    MINIROS_WARN("Faled to extract address from discovery packet: %s", error.toString());
    return;
  }

  discoveryEvent.uuid = packet.uuid;
  discoveryEvent.masterAddress.setPort(packet.masterPort);

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

Error Discovery::start(PollSet* pollSet, const UUID& uuid, int rpcPort, int broadcastPort)
{
  if (!internal_) {
    return Error::InternalError;
  }

  if (!uuid.valid())
    return Error::InvalidValue;

  if (broadcastPort == 0)
    broadcastPort = rpcPort;

  internal_->uuid = uuid;
  internal_->pollSet = pollSet;
  internal_->rpcPort = rpcPort;

  if (Error err = internal_->initSockets(broadcastPort); err != Error::Ok) {
    MINIROS_ERROR("Failed to initialize discovery sockets for port %d: %s", broadcastPort, err.toString());
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

  assert(internal_->broadcastPort);

  DiscoveryPacket packet;
  packet.op = 0;
  packet.size = sizeof(packet);
  packet.uuid = internal_->uuid;
  packet.masterPort = internal_->rpcPort;

  int interfaces = 0;
  int skipped = 0;
  if (internal_->adapterBroadcasts) {
    internal_->resolver->iterateAdapters(
      [&interfaces, &skipped, &packet, this](const network::NetAdapter* adapter)
      {
        if (adapter->isLoopback())
          return;
        if (!adapter->isValid())
          return;
        if (adapter->isIPv4() && adapter->broadcastAddress.valid()) {
          const sockaddr* localAddr = static_cast<const sockaddr*>(adapter->address.rawAddress());
          size_t addrSize = adapter->address.rawAddressSize();
          memcpy(&packet.addr, localAddr, addrSize);

          network::NetAddress brAddr = adapter->broadcastAddress;
          Error err = brAddr.setPort(internal_->broadcastPort);
          assert(err);

          auto [written, error] = internal_->socket.send(&packet, sizeof(packet), &brAddr);
          if (error != Error::Ok) {
            MINIROS_WARN("Failed to broadcast to adapter \"%s\" addr=%s", adapter->name.c_str(), brAddr.str().c_str());
          }
          interfaces++;
        } else {
          skipped++;
        }
      });
    if (interfaces == 0 && skipped > 0) {
      MINIROS_DEBUG("No interface has valid broadcast address, skipped=%d", skipped);
    } else {
      MINIROS_DEBUG("Broadcasted to %d interfaces", interfaces);
    }
  }

  if (internal_->multicastEnabled) {
    auto [written, error] = internal_->socket.send(&packet, sizeof(packet), &internal_->multicastGroup);
    if (error != Error::Ok) {
      MINIROS_WARN("Failed to multicast");
    }
  }
  MINIROS_DEBUG_NAMED("Discovery", "Broadcasted discovery packets to %d interfaces", interfaces);
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
    // TODO: implement resubscription to multicast group.
    return Error::NotImplemented;
  }
  internal_->multicastGroup = network::NetAddress::fromIp4String(group, 0);
  return Error::Ok;
}

void Discovery::setAdapterBroadcasts(bool flag)
{
  if (!internal_)
    return;
  internal_->adapterBroadcasts = flag;
}

}

}