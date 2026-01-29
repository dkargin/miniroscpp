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
class HttpServerConnection {
public:
  HttpServerConnection(HttpServer* server, std::shared_ptr<network::NetSocket> socket);
  ~HttpServerConnection();

  /// Handler for socket/poll events.
  /// @param evtFlags event flags from poll
  /// @returns new event mask
  int handleEvents(int evtFlags);

  enum class State {
    ReadRequest,
    ProcessRequest,
    WriteResponse,
    Exit,
  };

  /// Incremental reading of request.
  Error readRequest();

  void resetResponse();

  /// Close connection.
  void close();

  /// Fill in fault response.
  void prepareFaultResponse(Error error, http::HttpRequest& request) const;

  /// Detach from server.
  /// Breaks link with Http server.
  void detach();

  /// Allocate new request object or reuse some existing object from the pool.
  std::shared_ptr<HttpRequest> makeRequestObject();

protected:
  volatile State state_ = State::ReadRequest;

  HttpServer* server_;

  HttpParserFrame http_frame_;

  std::shared_ptr<HttpRequest> active_request;

  /// Intermediate storage for HttpResponseHeader.
  std::string response_header_buffer_;

  /// Number of bytes sent from current part (header or body).
  size_t data_sent_ = 0;

  SteadyTime request_start_;
  std::shared_ptr<network::NetSocket> socket_;

  std::mutex guard_;
};

}
}

#endif //MINIROS_HTTP_SERVER_CONNECTION_H
