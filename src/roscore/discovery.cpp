//
// Created by dkargin on 8/24/25.
//

#include "discovery.h"

#include "miniros/network/net_adapter.h"
#include "miniros/network/socket.h"
#include "miniros/transport/io.h"
#include "miniros/transport/poll_set.h"

#include "resolver.h"
#include "transport/poll_manager.h"

namespace miniros {

namespace master {

constexpr int DHCP_UDP_PORT = 67;
constexpr int DNS_UDP_PORT = 53;
constexpr int MDNS_UDP_PORT = 5353;
constexpr int NTP_UDP_PORT = 123;

struct Discovery::Internal {
  PollSet* pollSet = nullptr;

  AddressResolver* resolver = nullptr;

  /// Port for discovery broadcast.
  int broadcastPort = 0;

  /// UUID to be broadcasted.
  UUID uuid;

  /// Regular IPv4 socket for sending broadcasts.
  network::NetSocket socket;

  /// A collection of adapter-related sockets.
  std::multimap<std::string, network::NetSocket> discoverySockets;

  Internal(AddressResolver* resolver)
    : resolver(resolver)
  {
    assert(resolver);
  }

  NODISCARD Error initSockets(int port)
  {
    Error error = Error::Ok;
    error = socket.initUDP(false);
    if (!error) {
      return error;
    }

    error = socket.setBroadcast(true);
    if (!error) {
      detachSockets();
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

    error = socket.bind(port);
    if (!error) {
      detachSockets();
      return error;
    }

    error = socket.setBroadcast(true);
    if (!error) {
      detachSockets();
      return error;
    }

    broadcastPort = port;

    int fd = socket.fd();

    bool added = pollSet->addSocket(fd,
      [this](int flags)
      {
        if (flags & POLLIN) {
          onSocketEvent(socket, 0, flags);
        } else {
          MINIROS_WARN("Discovery socket: unexpected event %d", flags);
        }
      });

    if (!added)
    {
      MINIROS_ERROR("Failed to register listening socket");
      return Error::InternalError;
    }

    if (!pollSet->addEvents(fd, POLLIN)) {
      MINIROS_ERROR("Failed to add events");
      return Error::InternalError;
    }

    return Error::Ok;
  }

  void onSocketEvent(network::NetSocket& s, int role, int event)
  {
    std::string rawData;
    network::NetAddress address;;
    auto [numBytes, error] = s.recv(rawData, &address);
  }

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

Discovery::Discovery(AddressResolver* resolver)
{
  internal_.reset(new Internal(resolver));
}

Discovery::~Discovery()
{
  internal_->detachSockets();
}

Error Discovery::start(PollSet* pollSet, const UUID& uuid, int port)
{
  if (!internal_) {
    return Error::InternalError;
  }

  if (!uuid.valid())
    return Error::InvalidValue;

  internal_->uuid = uuid;
  internal_->pollSet = pollSet;

  if (Error err = internal_->initSockets(port); err != Error::Ok) {
    MINIROS_ERROR("Failed to initialize discovery sockets for port %d: %s", port, err.toString());
    return err;
  }


  return Error::Ok;
}

void Discovery::stop()
{

}

#pragma pack(push, 1)
struct DiscoveryPacket {
  /// Operation
  int16_t op = 0;

  /// Additional flags.
  int16_t flags = 0;

  /// Size of packet.
  int16_t size = 0;

  /// Sender address.
  sockaddr addr{};

  /// UUID of master instance.
  UUID uuid;
};
#pragma pack(pop)

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
  packet.size = 0;

  int interfaces = 0;
  int skipped = 0;
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

  if (interfaces == 0) {

  }

  MINIROS_INFO("Broadcasted discovery packets to %d interfaces", interfaces);
  return Error::Ok;
}

}

}