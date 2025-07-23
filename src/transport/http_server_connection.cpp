//
// Created by dkargin on 7/23/25.
//

#include <cassert>

#include "miniros/transport/http_server.h"
#include "miniros/transport/http_server_connection.h"

namespace miniros {
namespace network {

HttpServerConnection::HttpServerConnection(HttpServer* server, std::shared_ptr<NetSocket> socket)
  :server_(server), socket_(socket)
{
  http_frame_.finishRequest();
}

HttpServerConnection::~HttpServerConnection()
{}

Error HttpServerConnection::readRequest()
{
  auto [transferred, readErr] = socket_->read(http_frame_.data);

  const int fd = socket_->fd();
  const int parsed = http_frame_.incrementalParse();

  MINIROS_DEBUG("HttpServerConnection(%d)::readHeader: ContentLength=%d, parsed=%d", fd, http_frame_.contentLength(), parsed);

  // If we haven't gotten the entire request yet, return (keep reading)
  if (http_frame_.state() != miniros::network::HttpFrame::ParseComplete) {
    if (readErr == Error::EndOfFile) {
      MINIROS_ERROR("HttpServerConnection(%d)::readRequest: EOF while reading request", fd);
      http_frame_.finishRequest();
      return Error::EndOfFile;
      // Either way we close the connection
    }
    MINIROS_DEBUG("HttpServerConnection(%d)::readRequest got only %d/%d bytes.", fd, http_frame_.bodyLength(), http_frame_.contentLength());
    return Error::Ok;
  }

  assert(http_frame_.state() == miniros::network::HttpFrame::ParseComplete);
  auto body = http_frame_.body();
  // Otherwise, parse and dispatch the request
  MINIROS_DEBUG("HttpServerConnection(%d)::readRequest read %d/%d bytes.", fd, http_frame_.bodyLength(), http_frame_.contentLength());
  state_ = State::ProcessRequest;
  return Error::Ok;
}

int HttpServerConnection::handleEvents(int evtFlags)
{
  if (state_ == State::ReadRequest) {
    Error err = readRequest();
    // What to do with EOF?

    if (err == Error::EndOfFile) {
      // Returning 0 will make this connection be closed.
      return 0;
    }

    // Request is not finished. Need to wait for POLLIN
    if (state_ == State::ReadRequest) {
      return POLLIN;
    }
  }

  if (state_ == State::ProcessRequest) {
    resetResponse();

    // Execute request:
    auto* handler = server_->findEndpoint(http_frame_);
    assert(handler);
    std::string endpoint{http_frame_.getPath()};
    if (!handler) {
      MINIROS_ERROR("No handler for endpoint \"%s\"", endpoint.c_str());
      response_header_.statusCode = 404;
      response_header_.status = "Not Found";
    } else {
      ClientInfo clientInfo;
      MINIROS_INFO("Handling HTTP request to %s", endpoint.c_str());
      Error err = handler->handle(http_frame_, clientInfo, response_header_, response_body_);
      if (!err) {
        // TODO: What to do?
        // return internal server error.
      }
    }
    response_header_.writeHeader(response_header_buffer_, response_body_.size());
    state_ = State::WriteResponseHeader;
  }

  if (state_ == State::WriteResponseHeader) {
    auto [written, err] = socket_->write(response_header_buffer_.c_str() + data_sent_, response_header_buffer_.size() - data_sent_);
    if (err == Error::Ok) {
      if (written > 0) {
        data_sent_ += written;
        MINIROS_INFO_NAMED("HttpServerConnection", "Written %d/%d bytes of header",
          static_cast<int>(written), static_cast<int>(response_header_buffer_.size()));
      }

      if (data_sent_ < response_header_buffer_.size()) {
        // Failed to write all data, so we need to yield to spinner.
        return POLLOUT;
      }
      data_sent_ = 0;
      // Move to the next state.
      if (response_header_buffer_.empty()) {
        // Empty body payload, so switch to waiting for the next request.
        state_ = State::ReadRequest;
        http_frame_.finishRequest();
        return POLLIN;
      }
      state_ = State::WriteResponseBody;
    }
  }

  if (state_ == State::WriteResponseBody) {
    auto [written, err] = socket_->write(response_body_.c_str() + data_sent_, response_body_.size() - data_sent_);
    if (err == Error::Ok) {
      if (written > 0) {
        data_sent_ += written;
        MINIROS_INFO_NAMED("HttpServerConnection", "Written %d/%d bytes of body",
          static_cast<int>(written), static_cast<int>(response_body_.size()));
      }

      if (data_sent_ == response_body_.size()) {
        // Move to the next state.
        state_ = State::ReadRequest;
        http_frame_.finishRequest();
        data_sent_ = 0;
        return POLLIN;
      }
      // Failed to write all data, so we need to yield to spinner.
      return POLLOUT;
    }
  }

  MINIROS_WARN("HttpServerConnection no events to listen");
  return 0;
}

void HttpServerConnection::resetResponse()
{
  response_body_.resize(0);
  response_header_buffer_.resize(0);
  response_header_.reset();
  data_sent_ = 0;
}

}
}
