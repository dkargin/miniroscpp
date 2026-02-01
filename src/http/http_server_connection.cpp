//
// Created by dkargin on 7/23/25.
//

#include <cassert>

#include "miniros/http/http_server.h"
#include "miniros/http/http_server_connection.h"

#include "internal/invokers.h"
#include "miniros/io/poll_set.h"

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

namespace miniros {
namespace http {

HttpServerConnection::HttpServerConnection(HttpServer* server, std::shared_ptr<network::NetSocket> socket)
  :server_(server), socket_(socket)
{
  http_frame_.finishRequest();
}

HttpServerConnection::~HttpServerConnection()
{
  MINIROS_DEBUG("~HttpServerConnection()");
  std::unique_lock<std::mutex> lock(guard_);
}

Error HttpServerConnection::readRequest()
{
  if (http_frame_.state() == HttpParserFrame::ParseRequestHeader) {
    request_start_ = SteadyTime::now();
  }
  auto [transferred, readErr] = socket_->recv(http_frame_.data, nullptr);

  const int fd = socket_->fd();
  const int parsed = http_frame_.incrementalParse();

  std::string peer = socket_->peerAddress().str();
  double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
  MINIROS_DEBUG("HttpServerConnection(peer=%s fd=%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", peer.c_str(), fd, http_frame_.contentLength(), parsed, dur);

  // If we haven't gotten the entire request yet, return (keep reading)
  if (http_frame_.state() != HttpParserFrame::ParseComplete) {
    if (readErr == Error::EndOfFile) {
      MINIROS_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest: EOF while reading request", peer.c_str(), fd);
      http_frame_.finishRequest();
      return Error::EndOfFile;
      // Either way we close the connection
    }
    MINIROS_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest got only %d/%d bytes.", peer.c_str(), fd, http_frame_.bodyLength(), http_frame_.contentLength());
    return Error::Ok;
  }

  assert(http_frame_.state() == HttpParserFrame::ParseComplete);
  // Otherwise, parse and dispatch the request
  MINIROS_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest read %d/%d bytes.", peer.c_str(), fd, http_frame_.bodyLength(), http_frame_.contentLength());
  state_ = State::ProcessRequest;
  return Error::Ok;
}

std::shared_ptr<HttpRequest> HttpServerConnection::makeRequestObject()
{
  return std::make_shared<HttpRequest>();
}

int HttpServerConnection::handleEvents(int evtFlags)
{
  std::unique_lock<std::mutex> lock(guard_);
  bool stateFallback = false;
  if (state_ == State::ReadRequest) {
    if (evtFlags & PollSet::EventIn) {
      Error err = readRequest();
      // What to do with EOF?

      if (err == Error::Ok) {
        // Request is not finished. Need to wait for another packet.
        if (state_ == State::ReadRequest) {
          return PollSet::EventIn;
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
    EndpointHandler* handler = server_ ? server_->findEndpoint(http_frame_) : nullptr;
    auto requestObject = makeRequestObject();
    requestObject->updateState(HttpRequest::State::ServerHandleRequest);
    requestObject->resetResponse();

    std::string endpoint{http_frame_.getPath()};
    if (!handler) {
      MINIROS_ERROR("No handler for endpoint \"%s\"", endpoint.c_str());
      prepareFaultResponse(Error::FileNotFound, *requestObject);
    } else {
      network::ClientInfo clientInfo;
      clientInfo.fd = socket_->fd();
      clientInfo.remoteAddress = socket_->peerAddress();
      MINIROS_DEBUG("Handling HTTP request to path=\"%s\"", endpoint.c_str());

      requestObject->setPath(std::string{http_frame_.getPath()});
      requestObject->setRequestBody(std::string{http_frame_.body()});

      Error err = Error::Ok;
      {
        ScopedUnlock unlock(lock);
        err = handler->handle(clientInfo, requestObject);
      }
      if (!err) {
        MINIROS_ERROR("Failed to handle HTTP request to \"%s\"", err.toString());
        prepareFaultResponse(err, *requestObject);
      }
    }

    const HttpResponseHeader& responseHeader = requestObject->responseHeader();
    requestObject->updateState(HttpRequest::State::ServerSendResponse);
    active_request = requestObject;
    responseHeader.writeHeader(response_header_buffer_, requestObject->responseBody().size());
    state_ = State::WriteResponse;
    stateFallback = true;

    if (!server_) {
      close();
    }
  }

  if (state_ == State::WriteResponse) {
    if (evtFlags & PollSet::EventOut || stateFallback) {
      std::pair<size_t, Error> r;

      const std::string& responseBody = active_request->responseBody();
      size_t responseBodySize = responseBody.size();

      if (responseBodySize > 0) {
        r = socket_->write2(
          response_header_buffer_.c_str(), response_header_buffer_.size(),
          responseBody.c_str(), responseBodySize, data_sent_);

      } else {
        const char* data = response_header_buffer_.c_str() + data_sent_;
        size_t toSend = response_header_buffer_.size() - data_sent_;
        r = socket_->send(data, toSend, nullptr);
      }

      const size_t totalSize = response_header_buffer_.size() + responseBodySize;
      auto [written, err] = r;

      if (written > 0) {
        data_sent_ += written;
        double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
        MINIROS_DEBUG("HttpServerConnection written %d/%d bytes of header, t=%fms",
          static_cast<int>(written), static_cast<int>(totalSize), dur);
      }

      if (err == Error::WouldBlock)
        return PollSet::EventOut;

      if (err == Error::Ok) {
        if (data_sent_ < totalSize) {
          // Failed to write all data, so we need to yield to spinner.
          return PollSet::EventOut;
        }
        data_sent_ = 0;
        state_ = State::ReadRequest;
        http_frame_.finishRequest();
        auto dur = SteadyTime::now() - request_start_;
        MINIROS_DEBUG("Served HTTP response in %fms", dur.toSec()*1000);
        return PollSet::EventIn;
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
  response_header_buffer_.resize(0);
  active_request.reset();
  data_sent_ = 0;
}

void HttpServerConnection::prepareFaultResponse(Error error, http::HttpRequest& request) const
{
  request.resetResponse();
  switch (error.code) {
    case Error::FileNotFound:
      request.setResponseStatus(404, "Not Found");
      request.setResponseBody("<!doctype html>"
      "<html>"
      "<title>404 Not Found</title>"
      "<body>404 Not Found</body>"
      "</html>", "text/html");
      break;
    case Error::NotImplemented:
      request.setResponseStatus(501, "Not Implemented)");
      request.setResponseBody("<!doctype html>"
      "<html>"
      "<title>501 Not Implemented</title>"
      "<body>501 Not Implemented</body>"
      "</html>", "text/html");
      break;
    default:
      request.setResponseStatus(500, "Internal Server Error");
      request.setResponseBody("<!doctype html>"
      "<html>"
      "<title>500 Internal server error</title>"
      "<body>500 Internal server error</body>"
      "</html>", "text/html");
      break;
  }
}

void HttpServerConnection::detach()
{
  std::unique_lock<std::mutex> lock(guard_);
  server_ = nullptr;
}

}
}
