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
#include "miniros/network/url.h"

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
class MINIROS_DECL HttpClient {
public:
  using NetSocket = network::NetSocket;

  HttpClient(PollSet* ps);
  ~HttpClient();

  struct MINIROS_DECL State {
    enum State_t {
      /// No socket or connection available.
      Invalid,
      /// Connecting to server or on timeout.
      /// Client has some valid socket, valid destination address, but connection is not established.
      /// User can start adding requests to client. They will be processed right after connection is established.
      Connecting,
      /// Disconnected from server.
      /// Client has some destination address, valid socket FD, but do not participate in PollSet spinning.
      Disconnected,
      /// Waiting for reconnection timer.
      /// Client has some destination address, valid socket fd and waits for PollSet timeout event.
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

    State(State_t val) : value(val) {}

    operator State_t() const
    {
      return value;
    }

    /// Convert state to string.
    const char* toString() const;

  protected:
    State_t value;
  };

  /// Start connecting to specified server.
  /// Actual connection will be completed later.
  /// @returns Possible errors:
  ///  - InvalidAddress if address/port are invalid.
  ///  - other errors from NetSocket::tcpConnect.
  Error connect(const std::string& address, int port);

  /// Add request to queue.
  /// @returns Possible errors:
  ///  - InvalidValue if request is empty.
  ///  - InternalError if internal_ is empty.
  Error enqueueRequest(const std::shared_ptr<HttpRequest>& request);

  /// Drop request from queue.
  /// @returns possible errors:
  ///  - InvalidValue if request is empty or if request is not actually stored in queue.
  ///  - Internal error if internal_ is empty.
  Error dropRequest(const std::shared_ptr<HttpRequest>& request);

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

  /// Get number of queued requests, including active one.
  size_t getQueuedRequests() const;

  /// Called when TCP connection is broken.
  using DisconnectHandler = std::function<void (std::shared_ptr<NetSocket>& socket, State state)>;

  /// Sets callback for disconnect event.
  /// Handler will be called when disconnect will happen. It can be during async connection, or any other active state.
  /// This handler will be called in some unspecified background thread (from PollSet callbacks).
  /// User can call any method of HttpClient from this handler.
  void setDisconnectHandler(DisconnectHandler&& handler);

  using TimeoutHandler = std::function<void (std::shared_ptr<NetSocket>& socket, State state)>;
  /// Set handler for timeout in idle state.
  void setIdleTimeoutHandler(int timeoutMs, TimeoutHandler&& handler);

  /// Activate reconnection.
  /// @param timeoutMs - number of milliseconds to wait for reconnection.
  /// @returns possible errors:
  ///  - InternalError if internal structures are null.
  ///  - InvalidHandle - socket is null.
  ///  - other errors from connect methods if timeout is 0.
  Error reconnect(int timeoutMs);

  /// Get current number of reconnection attempts.
  int getReconnectAttempts() const;

  /// Called when client has successfully established TCP connection.
  /// @returns if connection should be kept. false if connection should be dropped.
  using ConnectHandler = std::function<bool (const std::shared_ptr<NetSocket>& socket)>;
  void setConnectHandler(ConnectHandler&& onConnect);

  /// Called when client has successfully established TCP connection.
  using ResponseHandler = std::function<void (const std::shared_ptr<HttpRequest>& req)>;

  /// Sets callback function for processed event.
  void setResponseHandler(ResponseHandler&& onResponse);

  /// Get high level state.
  State getState() const;

  /// Check if client is in any "active" state.
  bool isActive() const;

  /// Get internal file descriptor.
  int fd() const;

  /// Get root URL to base address of connected server (without path and query parameters).
  network::URL getRootURL(const std::string& scheme) const;

protected:
  /// Get socket events for specified state.
  static int eventsForState(State state);

  struct Internal;

  using Lock = std::unique_lock<std::mutex>;

  /// Handle socket events from PollSet.
  static int handleSocketEvents(const std::shared_ptr<Internal>& I, int events);

private:
  /// Shared pointer is used to resolve racing condition with handleEvent callback from other thread.
  std::shared_ptr<Internal> internal_;
};

}
}

#endif // MINIROS_HTTP_CLIENT_H
