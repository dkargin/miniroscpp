//
// Created by dkargin on 7/23/25.
//

#ifndef MINIROS_HTTP_SERVER_CONNECTION_H
#define MINIROS_HTTP_SERVER_CONNECTION_H

#include <mutex>

#include "miniros/network/socket.h"
#include "miniros/http/http_tools.h"
#include "miniros/http/http_request.h"

#include "miniros/steady_timer.h"


namespace miniros {

namespace http {

class HttpServer;

/// A Connection to HttpServer from Client.
/// It handles parsing HTTP request from client, picking right endpoint handler and sending response back.
class HttpServerConnection : public std::enable_shared_from_this<HttpServerConnection> {
public:
  HttpServerConnection(HttpServer* server, std::shared_ptr<network::NetSocket> socket, PollSet* pollSet);
  ~HttpServerConnection();

  /// Handler for socket/poll events.
  /// @param evtFlags event flags from poll
  /// @returns new event mask
  int handleEvents(int evtFlags);

  struct MINIROS_DECL State {
    enum State_t {
      /// Reading HTTP request from client.
      ReadRequest,
      /// Processing request (finding handler, executing it).
      ProcessRequest,
      /// Writing HTTP response to client.
      WriteResponse,
      /// Connection is closing/exiting.
      Exit,
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

  /// Incremental reading of request.
  Error readRequest();

  void resetResponse();

  /// Close connection.
  void close();

  /// Fill in fault response.
  static void prepareFaultResponse(Error error, http::HttpRequest& request);

  /// Detach from server.
  /// Breaks link with Http server.
  void detach();

  using Lock = TimeCheckLock<std::mutex>;

  /// Allocate new request object or reuse some existing object from the pool.
  std::shared_ptr<HttpRequest> makeRequestObject(Lock& lock);

  /// It is called from CallbackQueue thread when response is ready.
  void onAsyncRequestComplete(std::shared_ptr<HttpRequest> request, Error err);

  int eventsForState(State state) const;

protected:
  /// Updates internal state.
  void updateState(Lock& lock, State newState);

  /// Handle State::ProcessRequest.
  /// Expected transitions:
  ///  - WriteResponse if got response immediately
  ///  - stay in ProcessRequest if response is sent to callback queue.
  bool handleProcessRequest(Lock& lock);

  /// Handle State::WriteResponse.
  void handleWriteResponse(Lock& lock, int evtFlags, bool fallThrough);

  /// Start writing response to socket. It serializes response to buffers and switches state to WriteResponse.
  void doWriteResponse(Lock& lock, const std::shared_ptr<HttpRequest>& requestObject, Error error);

  State state_ = State::ReadRequest;

  HttpServer* server_;

  HttpParserFrame http_frame_;

  std::shared_ptr<HttpRequest> active_request;

  /// Intermediate storage for HttpResponseHeader.
  std::string response_header_buffer_;

  /// Number of bytes sent from current part (header or body).
  size_t data_sent_ = 0;

  SteadyTime request_start_;
  std::shared_ptr<network::NetSocket> socket_;

  /// Cached file descriptor.
  int debugFd_ = 0;

  PollSet* poll_set_ = nullptr;

  mutable std::mutex guard_;
};

}
}

#endif //MINIROS_HTTP_SERVER_CONNECTION_H
