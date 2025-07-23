//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_SOCKET_H
#define MINIROS_SOCKET_H

#include "miniros/errors.h"
#include "miniros/transport/net_address.h"

namespace miniros {
namespace network {

/// Owner of a  network socket.
class MINIROS_DECL NetSocket {
public:
  NetSocket();
  NetSocket(socket_fd_t fd, bool own);
  ~NetSocket();

  /// No copy allowed.
  NetSocket(const NetSocket& other) = delete;
  NetSocket& operator=(const NetSocket& other) = delete;

  /// Bind socket to some port.
  Error bind(int port, NetAddress::Type type);

  /// Enable non-blocking mode.
  Error setNonBlock();

  /// Enable reusing of socket address.
  Error setReuseAddr();

  /// Allocate tcp port and start listening.
  Error tcpListen(int port, NetAddress::Type type, int maxQueuedClients=100);

  /// Enter listening mode.
  /// Socket must be already created.
  Error listen(int maxQueuedClients);

  /// Update peer address.
  void setPeerAddress(const NetAddress& address);

  /// Get current peer address.
  NetAddress peerAddress() const;

  int fd() const;

  /// Get current port number.
  int port() const;

  /// Check if object is associated with any valid fd.
  bool valid() const;

  /// Close and reset socket.
  void close();

  /// Accept connection.
  std::pair<std::shared_ptr<NetSocket>, Error> accept();

  std::pair<size_t, Error> read(std::string& s);

  /// Write data to a socket.
  std::pair<size_t, Error> write(const char* data, size_t size);

protected:
  int fd_ = -1;
  /// Do we own the FD and close it on exit.
  bool own_ = false;

  /// Socket is listening for connections.
  bool listening_ = false;

  /// Address of connected endpoint.
  NetAddress peer_address_;
};

}
}

#endif //MINIROS_SOCKET_H
