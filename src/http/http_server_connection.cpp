//
// Created by dkargin on 7/23/25.
//

#include <cassert>

#include "miniros/http/http_server.h"
#include "miniros/http/http_server_connection.h"

#include "miniros/transport/io.h"

namespace miniros {
namespace http {

HttpServerConnection::HttpServerConnection(HttpServer* server, std::shared_ptr<network::NetSocket> socket)
  :server_(server), socket_(socket)
{
  http_frame_.finishRequest();
}

HttpServerConnection::~HttpServerConnection()
{}

Error HttpServerConnection::readRequest()
{
  if (http_frame_.state() == HttpFrame::ParseRequestHeader) {
    request_start_ = SteadyTime::now();
  }
  auto [transferred, readErr] = socket_->recv(http_frame_.data, nullptr);

  const int fd = socket_->fd();
  const int parsed = http_frame_.incrementalParse();

  double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
  MINIROS_DEBUG("HttpServerConnection(%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", fd, http_frame_.contentLength(), parsed, dur);

  // If we haven't gotten the entire request yet, return (keep reading)
  if (http_frame_.state() != HttpFrame::ParseComplete) {
    if (readErr == Error::EndOfFile) {
      MINIROS_DEBUG("HttpServerConnection(%d)::readRequest: EOF while reading request", fd);
      http_frame_.finishRequest();
      return Error::EndOfFile;
      // Either way we close the connection
    }
    MINIROS_DEBUG("HttpServerConnection(%d)::readRequest got only %d/%d bytes.", fd, http_frame_.bodyLength(), http_frame_.contentLength());
    return Error::Ok;
  }

  assert(http_frame_.state() == HttpFrame::ParseComplete);
  auto body = http_frame_.body();
  // Otherwise, parse and dispatch the request
  MINIROS_DEBUG("HttpServerConnection(%d)::readRequest read %d/%d bytes.", fd, http_frame_.bodyLength(), http_frame_.contentLength());
  state_ = State::ProcessRequest;
  return Error::Ok;
}

int HttpServerConnection::handleEvents(int evtFlags)
{
  bool stateFallback = false;
  if (state_ == State::ReadRequest) {
    if (evtFlags & POLLIN) {
      Error err = readRequest();
      // What to do with EOF?

      if (err == Error::Ok) {
        // Request is not finished. Need to wait for another packet.
        if (state_ == State::ReadRequest) {
          return POLLIN;
        }
      }
      else if (err == Error::EndOfFile) {
        // Returning 0 will make this connection be closed.
        return 0;
      } else {
        MINIROS_ERROR("Unexpected error %s", err.toString());
        return 0;
      }
    } else {
      MINIROS_WARN("HttpServerConnection unhandled events in ReadRequest: %o", evtFlags);
      return 0;
    }
  }

  if (state_ == State::ProcessRequest) {
    resetResponse();
    // Execute request:
    auto* handler = server_->findEndpoint(http_frame_);
    std::string endpoint{http_frame_.getPath()};
    if (!handler) {
      MINIROS_ERROR("No handler for endpoint \"%s\"", endpoint.c_str());
      prepareFaultResponse(Error::FileNotFound, response_header_, response_body_);
    } else {
      network::ClientInfo clientInfo;
      clientInfo.fd = socket_->fd();
      clientInfo.remoteAddress = socket_->peerAddress();
      MINIROS_DEBUG("Handling HTTP request to %s", endpoint.c_str());
      Error err = handler->handle(http_frame_, clientInfo, response_header_, response_body_);
      if (!err) {
        MINIROS_ERROR("Failed to handle HTTP request to \"%s\"", err.toString());
        prepareFaultResponse(err, response_header_, response_body_);
      }
    }
    response_header_.writeHeader(response_header_buffer_, response_body_.size());
    state_ = State::WriteResponse;
    stateFallback = true;
  }

  if (state_ == State::WriteResponse) {
    if (evtFlags & POLLOUT || stateFallback) {
      std::pair<size_t, Error> r;

      if (response_body_.size() > 0) {
        r = socket_->write2(
          response_header_buffer_.c_str(), response_header_buffer_.size(),
          response_body_.c_str(), response_body_.size(), data_sent_);

      } else {
        const char* data = response_header_buffer_.c_str() + data_sent_;
        size_t toSend = response_header_buffer_.size() - data_sent_;
        r = socket_->send(data, toSend, nullptr);
      }

      const size_t totalSize = response_header_buffer_.size() + response_body_.size();
      auto [written, err] = r;

      if (written > 0) {
        data_sent_ += written;
        double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
        MINIROS_DEBUG("HttpServerConnection written %d/%d bytes of header, t=%fms",
          static_cast<int>(written), static_cast<int>(totalSize), dur);
      }

      if (err == Error::WouldBlock)
        return POLLOUT;

      if (err == Error::Ok) {
        if (data_sent_ < totalSize) {
          // Failed to write all data, so we need to yield to spinner.
          return POLLOUT;
        }
        data_sent_ = 0;
        state_ = State::ReadRequest;
        http_frame_.finishRequest();
        auto dur = SteadyTime::now() - request_start_;
        MINIROS_DEBUG("Served HTTP response in %fms", dur.toSec()*1000);
        return POLLIN;
      } else {
        MINIROS_ERROR("HttpServerConnection writeResponse error: %s", err.toString());
        return 0;
      }
    } else {
      MINIROS_WARN("HttpServerConnection unhandled event in WriteResponse: %o", evtFlags);
      return 0;
    }
  }

  MINIROS_WARN("HttpServerConnection unhandled events: %o in state %d", evtFlags, (int)state_);
  return 0;
}

void HttpServerConnection::close()
{
  socket_->close();
  socket_.reset();
}

void HttpServerConnection::resetResponse()
{
  response_body_.resize(0);
  response_header_buffer_.resize(0);
  response_header_.reset();
  data_sent_ = 0;
}

void HttpServerConnection::prepareFaultResponse(Error error, HttpResponseHeader& responseHeader, std::string& body) const
{
  responseHeader.reset();
  responseHeader.contentType = "text/html";
  switch (error.code) {
    case Error::FileNotFound:
      responseHeader.statusCode = 404;
      responseHeader.status = "Not Found";
      body = "<!doctype html>"
      "<html>"
      "<title>404 Not Found</title>"
      "<body>404 Not Found</body>"
      "</html>";
      break;
    case Error::NotImplemented:
      responseHeader.statusCode = 501;
      responseHeader.status = "Not Implemented";
      body = "<!doctype html>"
      "<html>"
      "<title>501 Not Implemented</title>"
      "<body>501 Not Implemented</body>"
      "</html>";
      break;
    default:
      responseHeader.statusCode = 500;
      responseHeader.status = "Internal Server Error";
      body = "<!doctype html>"
      "<html>"
      "<title>500 Internal server error</title>"
      "<body>500 Internal server error</body>"
      "</html>";
      break;
  }
}

}
}
