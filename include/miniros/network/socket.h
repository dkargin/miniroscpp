//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_SOCKET_H
#define MINIROS_SOCKET_H

#include <memory>

#include "miniros/errors.h"
#include "net_address.h"

namespace miniros {
namespace network {

struct NetAdapter;

/// Owner of a network socket.
/// Most of this class is not thread safe.
class MINIROS_DECL NetSocket {
public:

  /// Socket type.
  enum class Type {
    Invalid,
    /// Pure unix socket
    Unix,
    /// Regular TCP over ipv4.
    TCP,
    /// TCP ipv6
    TCPv6,
    /// Regular UDP.
    UDP,
    /// UDP ipv6.
    UDPv6
  };

  /// Creates object without any bound socket.
  NetSocket();

  /// Attach object to existing socket.
  /// @param fd - socket file descriptor
  /// @param type - socket type.
  /// @param own - should socket completely own descriptor and close it in destructor.
  NetSocket(int fd, Type type, bool own);

  ~NetSocket();

  /// No copy allowed.
  NetSocket(const NetSocket& other) = delete;
  NetSocket& operator=(const NetSocket& other) = delete;

  /// Bind socket to some port.
  Error bind(int port);

  /// Enable non-blocking mode.
  Error setNonBlock();

  /// Enable reusing of socket address.
  Error setReuseAddr(bool reuse = true);

  /// Enable reusing of socket port.
  Error setReusePort(bool reuse);

  /// Set TCP NO_DELAY option.
  Error setNoDelay(bool noDelay);

  /// Enable broadcast mode.
  /// Works only with datagram sockets.
  Error setBroadcast(bool broadcast);

  /// Allocate tcp port and start listening.
  Error tcpListen(int port, NetAddress::Type type, int maxQueuedClients=100);

  /// Connect to specified TCP address.
  /// @param address - destination address
  /// @param nonblock - connect in nonblock mode. Actual state should be processed by PollSet.
  Error tcpConnect(const NetAddress& address, bool nonblock);

  /// Enter listening mode.
  /// Socket must be already created.
  Error listen(int maxQueuedClients);


  /// Initialize UDP ipv4 socket.
  /// @param ip6 - should create ipv6 socket.
  Error initUDP(bool ip6 = false);

  /// Get current peer address.
  NetAddress peerAddress() const;

  /// Join to multicast group on any interface.
  /// Only IP address is taken from `group`. Port value is ignored.
  Error joinMulticastGroup(const NetAddress& group);

  int fd() const;

  /// Get current port number.
  int port() const;

  /// Check if object is associated with any valid fd.
  bool valid() const;

  /// Close and reset socket.
  void close();

  /// Check if socket is in datagram mode.
  bool isDatagram() const;

  /// Check if socket is in stream mode.
  bool isStream() const;

  /// Check if socket is using ip v4.
  bool isIpv4() const;

  /// Check if socket is using ip v6.
  bool isIpv6() const;

  /// Check if socket is connecting to remote host.
  bool isConnecting() const;

  /// Check if connection is complete and clear connecting flag.
  /// @returns:
  ///   Error::Ok - connection is complete
  ///   Error::Timeout - still connecting.
  ///   Any other error - failed to connect.
  Error checkConnected();

  /// Bind socket to a specific network adapter.
  Error bind(const NetAddress& adapter);

  /// Accept connection.
  std::pair<std::shared_ptr<NetSocket>, Error> accept();

  using WriteBuf = std::string;
  /// Read data from connected socket.
  ///
  /// @param s - output buffer.
  /// @param address - address of sender. Can be nullptr
  std::pair<size_t, Error> recv(WriteBuf& s, NetAddress* address);

  /// Send data into a socket.
  /// Stream sockets will try to send data iteratively until EAWOULDBLOCK happens.
  /// It will return number of bytes actually sent if partial transfer has happened.
  /// Datagram sockets will make only one attempt to send.
  /// @param rawData - pointer to data buffer.
  /// @param size - size of data in bytes.
  /// @param address - destination address. It is only relevant to datagram sockets.
  /// @returns a tuple with number of bytes actually sent and error code.
  std::pair<size_t, Error> send(const void* rawData, size_t size, const NetAddress* address);

  /// Write two separate buffers to a socket.
  /// It is often HTTP header and a payload from a separate buffer.
  std::pair<size_t, Error> write2(const char* header, size_t headerSize,
    const char* body, size_t bodySize,
    size_t written = 0);

  /// Get system error from socket.
  /// It can be used to extract error when poll/epoll returned POLLERR event.
  int getSysError() const;

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}

#endif //MINIROS_SOCKET_H
