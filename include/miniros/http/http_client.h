//
// Created by dkargin on 11/14/25.
//

#ifndef MINIROS_HTTP_CLIENT_H
#define MINIROS_HTTP_CLIENT_H

#include <deque>
#include <memory>
#include <mutex>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_request.h"

#include <condition_variable>

namespace miniros {

class PollSet;

namespace network {
class NetSocket;
struct ClientInfo;
}

namespace http {

/// Persistent HTTP connection.
class HttpClient {
public:
  using NetSocket = network::NetSocket;

  HttpClient(PollSet* ps);
  ~HttpClient();

  enum class State {
    /// No socket or connection available.
    Invalid,
    /// Connecting to server or on timeout.
    Connecting,
    /// Has valid socket but there are no requests to process.
    Idle,
    /// Writing request header+body (if any) into socket.
    WriteRequest,
    /// Waiting for response or reading response.
    ReadResponse,
    /// Processing response. TODO: Is it actually needed?
    ProcessResponse,
  };

  /// Attach client to socket and set up event loop.
  Error attach(std::shared_ptr<NetSocket> sock);

  /// Start connecting to specified server.
  /// Actual connection will be completed later.
  Error connect(const std::string& address, int port);

  /// Add request to queue.
  Error enqueueRequest(const std::shared_ptr<HttpRequest>& request);

  /// Close connection and detach from socket.
  void close();

  /// Detach from socket.
  std::shared_ptr<network::NetSocket> detach();

  /// Wait until connected or timeout.
  /// @returns:
  ///   Error::Ok if everything is fine.
  ///   Error::InvalidHandle if internal socket was not initialized.
  ///   Error::Timeout if duration limit is hit.
  ///   Error::NotConnected - connection refused.
  Error waitConnected(const WallDuration& timeout);

protected:
  struct Internal;
  /// Shared pointer is used to resolve racing condition with handleEvent callback from other thread.
  std::shared_ptr<Internal> internal_;
};

}
}

#endif // MINIROS_HTTP_CLIENT_H
