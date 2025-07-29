//
// Created by dkargin on 7/19/25.
//

#ifndef MINIROS_SOCKET_H
#define MINIROS_SOCKET_H

#include <memory>

#include "miniros/errors.h"
#include "miniros/transport/net_address.h"

namespace miniros {
namespace network {

/// Owner of a  network socket.
class MINIROS_DECL NetSocket {
public:
  /// Creates object without any bound socket.
  NetSocket();

  /// Attach object to existing socket.
  /// @param fd - socket file descriptor
  /// @param own - should socket completely own descriptor and close it in destructor.
  NetSocket(int fd, bool own);

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

  /// Set TCP NO_DELAY option.
  Error setNoDelay(bool nodelay);

  /// Allocate tcp port and start listening.
  Error tcpListen(int port, NetAddress::Type type, int maxQueuedClients=100);

  /// Enter listening mode.
  /// Socket must be already created.
  Error listen(int maxQueuedClients);

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

  /// Write two separate buffers to a socket.
  /// It is often HTTP header and a payload from a separate buffer.
  std::pair<size_t, Error> write2(const char* header, size_t headerSize,
    const char* body, size_t bodySize,
    size_t written = 0);

protected:
  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}
}

#endif //MINIROS_SOCKET_H
