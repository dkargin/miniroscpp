//
// Created by dkargin on 7/23/25.
//

#include <cassert>

#include "miniros/console.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_server_connection.h"

#include "internal/scoped_locks.h"
#include "miniros/io/poll_set.h"
#include "xmlrpcpp/XmlRpcException.h"

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

namespace miniros {
namespace http {

HttpServerConnection::HttpServerConnection(HttpServer* server, std::shared_ptr<network::NetSocket> socket, PollSet* poll_set)
  :server_(server), socket_(socket), poll_set_(poll_set)
{
  http_frame_.finishRequest();
  assert(socket);
  if (socket_) {
    debugFd_ = socket_->fd();
  }
}

HttpServerConnection::~HttpServerConnection()
{
  std::unique_lock<std::mutex> lock(guard_);
  LOCAL_DEBUG("~HttpServerConnection(%d)", debugFd_);
}

const char* HttpServerConnection::State::toString() const
{
  switch (value) {
    case ReadRequest:
      return "ReadRequest";
    case ProcessRequest:
      return "ProcessRequest";
    case WriteResponse:
      return "WriteResponse";
    case Disconnected:
      return "Disconnected";
    default:
      assert(false);
  }
  return "Unknown";
}

Error HttpServerConnection::doReadRequest(Lock& lock)
{
  if (http_frame_.state() == HttpParserFrame::ParseRequestHeader) {
    request_start_ = SteadyTime::now();
  }
  auto [transferred, readErr] = socket_->recv(http_frame_.data, nullptr);

  const int fd = socket_->fd();
  const int parsed = http_frame_.incrementalParse();

  std::string peer = socket_->peerAddress().str();
  double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
  LOCAL_DEBUG("HttpServerConnection(peer=%s fd=%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", peer.c_str(), fd, http_frame_.contentLength(), parsed, dur);

  // If we haven't gotten the entire request yet, return (keep reading)
  if (http_frame_.state() != HttpParserFrame::ParseComplete) {
    if (readErr == Error::EndOfFile) {
      LOCAL_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest: EOF while reading request", peer.c_str(), fd);
      http_frame_.finishRequest();
      return Error::EndOfFile;
      // Either way we close the connection
    }
    LOCAL_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest got only %d/%d bytes.", peer.c_str(), fd, http_frame_.bodyLength(), http_frame_.contentLength());
    return Error::Ok;
  }

  assert(http_frame_.state() == HttpParserFrame::ParseComplete);
  // Otherwise, parse and dispatch the request
  LOCAL_DEBUG("HttpServerConnection(peer=%s fd=%d)::readRequest read %d/%d bytes.", peer.c_str(), fd, http_frame_.bodyLength(), http_frame_.contentLength());
  updateState(lock, state_ = State::ProcessRequest);
  return Error::Ok;
}

class HttpServerCallback : public CallbackInterface {
public:
  ~HttpServerCallback() override
  {
  }

  CallResult call() override
  {
    int reqId = request->id();

    auto conn = connection.lock();
    if (!conn) {
      LOCAL_DEBUG("HttpServerCallback::call(%d) connection expired", reqId);
      return Invalid;
    }

    // Execute request handler
    Error err = Error::Ok;

    // Must provide some reasonable response even in case of internal exception.
    try {
      err = handler->handle(clientInfo, request);
      if (!err) {
        LOCAL_ERROR("Failed to handle HTTP request: %s", err.toString());
      }
    } catch (XmlRpc::XmlRpcException& ex) {
      LOCAL_ERROR("HttpServerCallback::call(req=%d) XMLRPC exception: %s", reqId, ex.getMessage().c_str());
      err = Error::InvalidValue;
    } catch (std::runtime_error& ex) {
      LOCAL_ERROR("HttpServerCallback::call(req=%d) runtime exception: %s", reqId, ex.what());
      err = Error::InternalError;
    } catch (...) {
      LOCAL_ERROR("HttpServerCallback::call(req=%d) unknown exception", reqId);
      err = Error::InternalError;
    }
    // Complete request processing (this will prepare response and trigger EventOut)
    conn->onAsyncRequestComplete(request, err);
    return Success;
  }

  /// Request object.
  std::shared_ptr<HttpRequest> request;
  /// Handler to serve response.
  std::shared_ptr<EndpointHandler> handler;

  /// Client information captured at request time.
  network::ClientInfo clientInfo;

  /// Response should be sent to this connection.
  std::weak_ptr<HttpServerConnection> connection;
};

std::shared_ptr<HttpRequest> HttpServerConnection::makeRequestObject(Lock& lock)
{
  auto requestObject = std::make_shared<HttpRequest>();
  requestObject->resetResponse();
  requestObject->updateState(HttpRequest::State::ServerHandleRequest);

  requestObject->setPath(std::string{http_frame_.getPath()});
  requestObject->setRequestBody(std::string{http_frame_.body()});
  return requestObject;
}

bool HttpServerConnection::handleProcessRequest(Lock& lock)
{
  resetResponse();

  auto requestObject = makeRequestObject(lock);

  std::string endpoint{http_frame_.getPath()};

  // Execute request:
  if (!server_) {
    LOCAL_ERROR("HttpServerConnection::handleEvents connection fd=%d detached from server", socket_->fd());
    prepareFaultResponse(Error::InternalError, *requestObject);
    return false;
  }

  network::ClientInfo clientInfo;
  clientInfo.fd = socket_->fd();
  clientInfo.remoteAddress = socket_->peerAddress();
  LOCAL_DEBUG("Handling HTTP request to path=\"%s\"", endpoint.c_str());

  auto [handler, cb] = server_->findEndpoint(http_frame_);

  if (!handler) {
    LOCAL_ERROR("No handler for endpoint \"%s\"", endpoint.c_str());
    doWriteResponse(lock, requestObject, Error::FileNotFound);
    return true;
  }

  if (cb) {
    // Send request to CallbackQueue for processing in different thread
    auto callback = std::make_shared<HttpServerCallback>();
    callback->handler = handler;
    callback->request = requestObject;
    callback->clientInfo = clientInfo;
    callback->connection = weak_from_this();
    cb->addCallback(callback, reinterpret_cast<uint64_t>(requestObject.get()));
    // Return 0 - no events to wait for. Callback will trigger EventOut when done.
    return false;
  }

  // Process request immediately in current thread
  Error err = Error::Ok;
  {
    ScopedUnlock unlock(lock);
    err = handler->handle(clientInfo, requestObject);
  }

  doWriteResponse(lock, requestObject, err);
  return true;
}

void HttpServerConnection::handleDisconnect(Lock& lock)
{
  updateState(lock, State::Disconnected);
}


void HttpServerConnection::handleReadRequest(Lock& lock, int& evtFlags, bool fallThrough)
{
  if (!fallThrough && evtFlags & PollSet::EventError) {
    handleDisconnect(lock);
    evtFlags = 0;
  }

  if (evtFlags & PollSet::EventIn || fallThrough) {
    Error err = doReadRequest(lock);

    evtFlags &= (~PollSet::EventIn);

    // What to do with EOF?

    if (err == Error::Ok) {
      // Remain in the same ReadRequest state.
    }
    else if (err == Error::EndOfFile) {
      handleDisconnect(lock);
    } else {
      LOCAL_ERROR("Unexpected error %s", err.toString());
    }
  } else {
    LOCAL_WARN("HttpServerConnection::handleReadRequest() unhandled events in ReadRequest: %o", evtFlags);
  }
}


void HttpServerConnection::handleWriteResponse(Lock& lock, int& evtFlags, bool fallThrough)
{
  if (!fallThrough && evtFlags & PollSet::EventError) {
    handleDisconnect(lock);
    evtFlags = 0;
  }
  else if (evtFlags & PollSet::EventOut || fallThrough) {
    evtFlags &= (~PollSet::EventOut);

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
      LOCAL_DEBUG("HttpServerConnection written %d/%d bytes of header, t=%fms",
        static_cast<int>(written), static_cast<int>(totalSize), dur);
    }

    if (err == Error::WouldBlock)
      return;

    if (err == Error::Ok) {
      if (data_sent_ < totalSize) {
        // Failed to write all data, so we need to yield to spinner.
        return;
      }
      data_sent_ = 0;

      http_frame_.finishRequest();
      updateState(lock, State::ReadRequest);
      double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
      LOCAL_DEBUG("Served HTTP response in %fms", dur);
    } else {
      LOCAL_ERROR("HttpServerConnection writeResponse error: %s", err.toString());
    }
  }
}

int HttpServerConnection::handleEvents(int evtFlags)
{
  Lock lock(guard_, THIS_LOCATION);

  const int startEvt = evtFlags;

  // Set to true if we transition multiple FSM states.
  bool stateFallback = false;
  if (state_ == State::ReadRequest) {
    handleReadRequest(lock, evtFlags, stateFallback);
    evtFlags = 0;
  }

  if (state_ == State::ProcessRequest) {
    stateFallback = handleProcessRequest(lock);
  }

  if (state_ == State::WriteResponse) {
    handleWriteResponse(lock, evtFlags, stateFallback);
    evtFlags = 0;
  }

  if (evtFlags != 0) {
    LOCAL_WARN("HttpServerConnection::handleEvents(%s) unhandled events: %s in state %s",
      PollSet::eventToString(startEvt).c_str(),
      PollSet::eventToString(evtFlags).c_str(),
      state_.toString());
  }
  return eventsForState(state_);
}

void HttpServerConnection::doWriteResponse(Lock& lock, const std::shared_ptr<HttpRequest>& req, Error error)
{
  // Prepare fault response if handler returned error
  if (!error) {
    prepareFaultResponse(error, *req);
  }
  const HttpResponseHeader& responseHeader = req->responseHeader();
  req->updateState(HttpRequest::State::ServerSendResponse);
  active_request = req;
  responseHeader.writeHeader(response_header_buffer_, req->responseBody().size());
  updateState(lock, State::WriteResponse);
}

int HttpServerConnection::eventsForState(State state) const
{
  switch (state) {
    case State::WriteResponse:
      return PollSet::EventOut;
    case State::ReadRequest:
      return PollSet::EventIn;
    case State::ProcessRequest:
      return 0;
    case State::Disconnected:
      return PollSet::ResultDropFD;
    default:
      return 0;
  }
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

void HttpServerConnection::detach()
{
  Lock lock(guard_, THIS_LOCATION);
  server_ = nullptr;
}

void HttpServerConnection::onAsyncRequestComplete(std::shared_ptr<HttpRequest> request, Error err)
{
  Lock lock(guard_, THIS_LOCATION);
  
  // Check if connection is still in ProcessRequest state
  if (state_ != State::ProcessRequest) {
    LOCAL_DEBUG("HttpServerConnection::onAsyncRequestComplete(%d) unexpected state for sending response %s", request->id(), state_.toString());
    return;
  }

  // Prepare response
  doWriteResponse(lock, request, err);

  assert(socket_);
  int fd = socket_->fd();
  poll_set_->addEvents(fd, PollSet::EventOut);
}

void HttpServerConnection::updateState(Lock& lock, State newState)
{
  if (state_ == newState)
    return;

  auto oldState = state_;
  state_ = newState;
  LOCAL_INFO("HttpServerConnection::updateState from %s to %s", oldState.toString(), newState.toString());
}

void HttpServerConnection::prepareFaultResponse(Error error, http::HttpRequest& request)
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

}
}
