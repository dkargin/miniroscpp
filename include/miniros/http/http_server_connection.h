//
// Created by dkargin on 7/23/25.
//
// This is an internal header

#ifndef MINIROS_HTTP_SERVER_CONNECTION_H
#define MINIROS_HTTP_SERVER_CONNECTION_H

#include <mutex>

#include "miniros/network/socket.h"
#include "miniros/http/http_tools.h"
#include "miniros/http/http_request.h"

#include "miniros/internal/threading.h"

namespace miniros {

class PollSet;

namespace http {

namespace internal {
class EndpointCollection;
}

class HttpServer;

/// A Connection to HttpServer from Client.
/// It handles parsing HTTP request from client, picking right endpoint handler and sending response back.
/// Relations to server:
///  - Responses are handled in different thread and server can be destroyed in the same time.
///  - HttpServer provides collection of endpoints. This collection must exist until handleEvents is processed.
///  - HttpServer keeps pointers to all HttpServerConnection instances.
class HttpServerConnection : public std::enable_shared_from_this<HttpServerConnection> {
public:
  using Lock = TimeCheckLock<std::mutex>;


  HttpServerConnection(const std::shared_ptr<Lifetime<HttpServer>>& server,
    std::shared_ptr<network::NetSocket> socket);
  ~HttpServerConnection();

  void setEndpointCollection(const std::shared_ptr<internal::EndpointCollection>& endpoints);

  void attachPollSet(PollSet* pollSet);

  struct MINIROS_DECL State {
    enum State_t {
      /// Reading HTTP request from client.
      ReadRequest,
      /// Waiting for response from some background handler.
      WaitResponse,
      /// Writing HTTP response to client.
      WriteResponse,
      /// Connection is closing/exiting.
      Disconnected,
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

  void resetResponse();

  /// Get internal file descriptor/socket.
  int fd() const;

  /// Fill in fault response.
  static void prepareFaultResponse(Error error, http::HttpRequest& request);

  /// Extract socket for WebSocket upgrade.
  /// This method should only be called during WebSocket upgrade.
  /// It detaches the connection from the server and returns the socket.
  /// After calling this, the HttpServerConnection should be discarded.
  /// @returns the socket, or nullptr if extraction fails
  std::shared_ptr<network::NetSocket> extractSocketForUpgrade();

  /// Allocate new request object or reuse some existing object from the pool.
  std::shared_ptr<HttpRequest> makeRequestObject(Lock& lock);

  /// It is called from CallbackQueue thread when response is ready.
  void onAsyncRequestComplete(const std::shared_ptr<HttpRequest>& request, Error err);

  int eventsForState(State state) const;

  /// It is called by HttpServer instance when it decides to drop connections.
  void detachByServer();

  /// Check if this connection is detached from HTTP server instance.
  bool isDetachedFromParent() const;

protected:

  void detachPollSet(Lock& lock);

  /// Updates internal state.
  void updateState(Lock& lock, State newState);

  /// Incremental reading of request.
  Error doReadRequest(Lock& lock);

  struct MINIROS_NODISCARD EventReport {
    enum Command {
      Invalid,
      /// Proceed to next state.
      Continue,
      /// Repeat same state.
      Repeat,
      /// Response will be generated in background thread.
      BackgroundResponse,
      /// Need to process disconnect event and detach self from server.
      Disconnect,
      /// Need to drop ownership of socket and detach self from server.
      Upgrade,
    };

    Command cmd = Invalid;

    /// Number of bytes received in handler.
    size_t bytesWritten = 0;

    /// Leftover event flags.
    int evtFlags = 0;

    /// Message for handleDisconnect.
    std::string disconnectMsg;
  };

  /// Handler for socket/poll events.
  /// @param lock - acquired lock to internal state.
  /// @param evtFlags event flags from poll.
  EventReport handleSocketEvents(Lock& lock, int evtFlags);

  /// Handles State::ReadRequest.
  /// Expected transitions:
  ///  - ProcessRequest if got full HTTP request.
  ///  - Disconnected if error on socket.
  EventReport handleReadRequest(Lock& lock, int evtFlags);

  /// Handle State::WaitResponse.
  /// Connection waits until someone sends notification to socket that request is ready.
  /// Expected transitions:
  ///  - WriteResponse if got response immediately
  ///  - stay in WaitResponse if there were no changes.
  EventReport handleWaitResponse(Lock& lock, int evtFlags);

  /// Handle State::WriteResponse.
  /// Object stays in this state until all response is sent.
  EventReport handleWriteResponse(Lock& lock, int evtFlags);

  /// Detach from HttpServer.
  /// This function should be called without lock on `guard_`
  void detachFromServer(const std::shared_ptr<Lifetime<HttpServer>>& lifetime, bool upgrade, const std::string& reason);

  /// Update event flags for socket fd in PollSet according to current state.
  void updateEventsForSocket(Lock& lock);

  /// Start writing response to socket. It serializes response to buffers and switches state to WriteResponse.
  void doSerializeResponse(Lock& lock, const std::shared_ptr<HttpRequest>& requestObject, Error error);

  /// Close and drop socket.
  void closeSocket();

  State state_ = State::ReadRequest;

  std::shared_ptr<Lifetime<HttpServer>> server_lifetime_;

  HttpParserFrame http_frame_;

  std::shared_ptr<HttpRequest> active_request_;

  /// Intermediate storage for HttpResponseHeader.
  std::string response_header_buffer_;

  /// Number of bytes sent from current part (header or body).
  size_t data_sent_ = 0;

  SteadyTime request_start_;
  std::shared_ptr<network::NetSocket> socket_;

  /// Cached file descriptor.
  int debugFd_ = 0;

  PollSet* poll_set_ = nullptr;

  /// Guard for internals.
  mutable std::mutex guard_;

  std::weak_ptr<const internal::EndpointCollection> endpoints_;
};

}
}

#endif //MINIROS_HTTP_SERVER_CONNECTION_H
