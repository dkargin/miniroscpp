//
// Created by dkargin on 11/14/25.
//

#ifndef MINIROS_HTTP_CLIENT_H
#define MINIROS_HTTP_CLIENT_H

#include <deque>
#include <memory>
#include <mutex>
#include <functional>

#include "miniros/http/http_tools.h"
#include "miniros/http/http_request.h"

#include <condition_variable>

namespace miniros {

class PollSet;

namespace network {
class NetSocket;
class NetAddress;
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
    /// Waiting for reconnect.
    WaitReconnect,
    /// Has valid socket but there are no requests to process.
    Idle,
    /// Writing request header+body (if any) into socket.
    WriteRequest,
    /// Waiting for response or reading response.
    ReadResponse,
    /// Processing response. TODO: Is it actually needed?
    ProcessResponse,
  };

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

  struct DisconnectResponse {
    bool reconnect = false;
    size_t reconnectTimeout = 0;
  };

  std::function<DisconnectResponse (std::shared_ptr<NetSocket> socket)> onDisconnect;

protected:
  /// Get socket events for specified state.
  static int eventsForState(State state);

  Error connectImplUnsafe(const network::NetAddress& address);

  /// Handle socket events from PollSet.
  int handleSocketEvents(int events);

  /// Handle State::Connecting.
  /// Can switch to Idle.
  void handleConnecting(int events);

  /// Handle State::WriteRequest.
  /// Can switch to:
  ///   - ReadResponse when all request is sent
  ///   - WaitReconnect if error and reconnect is required.
  void handleWriteRequest(int events, bool fallThrough);

  /// Handle State::ReadResponse.
  /// Can switch to:
  ///   - ReadResponse when all request is sent
  ///   - WaitReconnect if error and reconnect is required.
  void handleReadResponse(int events, bool fallThrough);

  /// Handle State::ProcessResponse.
  /// Can switch to:
  ///    - WriteRequest if there is another request.
  ///    - Idle if no requests.
  void handleProcessResponse();

  /// Handle disconnection: call onDisconnect callback and attempt reconnection if requested.
  void handleDisconnect();

private:
  struct Internal;
  /// Shared pointer is used to resolve racing condition with handleEvent callback from other thread.
  std::shared_ptr<Internal> internal_;
};

}
}

#endif // MINIROS_HTTP_CLIENT_H
