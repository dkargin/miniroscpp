//
// Created by dkargin on 7/23/25.
//

#ifndef MINIROS_HTTP_SERVER_CONNECTION_H
#define MINIROS_HTTP_SERVER_CONNECTION_H

#include "miniros/network/socket.h"

#include "miniros/http/http_tools.h"

#include "miniros/steady_timer.h"

#include <mutex>

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
  void prepareFaultResponse(Error error, HttpResponseHeader& header, std::string& body) const;

  /// Detach from server.
  /// Breaks link with Http server.
  void detach();

protected:
  State state_ = State::ReadRequest;

  HttpServer* server_;

  HttpParserFrame http_frame_;

  HttpResponseHeader response_header_;
  /// Intermediate storage for response header.
  std::string response_header_buffer_;
  /// Intermediate storage for response body.
  std::string response_body_;
  /// Number of bytes sent from current part (header or body).
  size_t data_sent_ = 0;

  SteadyTime request_start_;
  std::shared_ptr<network::NetSocket> socket_;

  std::mutex guard_;
};

}
}

#endif //MINIROS_HTTP_SERVER_CONNECTION_H
