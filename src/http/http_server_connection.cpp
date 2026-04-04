//
// Created by dkargin on 7/23/25.
//

#include <cassert>

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

#include "internal/scoped_locks.h"
#include "io/io.h"
#include "miniros/callback_queue.h"
#include "miniros/console.h"
#include "miniros/http/endpoint_collection.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_server_connection.h"
#include "internal/lifetime.h"
#include "miniros/io/poll_set.h"
#include "xmlrpcpp/XmlRpcException.h"

namespace miniros {
namespace http {

HttpServerConnection::HttpServerConnection(const std::shared_ptr<Lifetime<HttpServer>>& server_lifetime,
  std::shared_ptr<network::NetSocket> socket)
  :server_lifetime_(server_lifetime), socket_(socket)
{
  http_frame_.finishRequest();
  assert(socket);
  if (socket_) {
    debugFd_ = socket_->fd();
  }
}

HttpServerConnection::~HttpServerConnection()
{
  Lock lock(guard_, THIS_LOCATION);
  LOCAL_DEBUG("~HttpServerConnection(%d)", debugFd_);
  detachPollSet(lock);
}

void HttpServerConnection::setEndpointCollection(const std::shared_ptr<internal::EndpointCollection>& endpoints)
{
  endpoints_ = endpoints;
}

void HttpServerConnection::detachPollSet(Lock& lock)
{
  if (poll_set_) {
    PollSet* ps = poll_set_;
    poll_set_ = nullptr;

    if (socket_ && socket_->valid()) {
      ScopedUnlock unlock(lock);
      ps->delSocket(socket_->fd());
    }
  }
}

void HttpServerConnection::attachPollSet(PollSet* poll_set)
{
  assert(!poll_set_);
  assert(poll_set);
  poll_set_ = poll_set;
  poll_set_->addSocket(socket_->fd(), PollSet::EventIn,
    [wconnection = weak_from_this()](int flags)
    {
      auto self = wconnection.lock();
      if (!self)
        return 0;
      std::shared_ptr<Lifetime<HttpServer>> server_lifetime;
      EventReport report;
      {
        Lock lock(self->guard_, THIS_LOCATION);
        report = self->handleSocketEvents(lock, flags);
        if (report.cmd == EventReport::Disconnect || report.cmd == EventReport::Upgrade) {
          // Gather all objects to detach self without keeping lock to `guard_` mutex.
          // It resolves issues with lock order, reported by helgrind.
          std::swap(server_lifetime, self->server_lifetime_);
          self->detachPollSet(lock);
        }
        self->updateEventsForSocket(lock);
      }

      int numRefs = self.use_count();
      if (report.cmd == EventReport::Disconnect || report.cmd == EventReport::Upgrade) {
        bool upgrade = (report.cmd == EventReport::Upgrade);
        self->detachFromServer(server_lifetime, upgrade, report.disconnectMsg);
        numRefs = self.use_count();
      }
      return 0;
  }, {}, THIS_LOCATION);
}

const char* HttpServerConnection::State::toString() const
{
  switch (value) {
    case ReadRequest:
      return "ReadRequest";
    case WaitResponse:
      return "WaitResponse";
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
  if (http_frame_.state() == HttpParserFrame::ParseRequestHeaderMethod) {
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
    return readErr;
  }

  assert(http_frame_.state() == HttpParserFrame::ParseComplete);
  return Error::Ok;
}

int HttpServerConnection::fd() const
{
  return socket_ ? socket_->fd() : MINIROS_INVALID_SOCKET;
}

bool HttpServerConnection::isDetachedFromParent() const
{
  if (!server_lifetime_)
    return true;

  auto lock = server_lifetime_->getLock();
  if (!server_lifetime_->valid(lock))
    return true;
  return false;
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

void HttpServerConnection::detachByServer()
{
  Lock lock(guard_, THIS_LOCATION);
  server_lifetime_ = {};

  if (poll_set_ && socket_ && socket_->valid()) {
    poll_set_->signalFd(socket_->fd());
  }
}

void HttpServerConnection::detachFromServer(const std::shared_ptr<Lifetime<HttpServer>>& lifetime, bool upgrade, const std::string& reason)
{
  if (!lifetime)
    return;
  auto lock2 = lifetime->getLock();
  if (lifetime->valid(lock2)) {
    HttpServer* server = lifetime->ptr();
    server->onConnectionClosed(lock2, shared_from_this(), upgrade, reason);
  }
}

std::shared_ptr<HttpRequest> HttpServerConnection::makeRequestObject(Lock& lock)
{
  auto requestObject = std::make_shared<HttpRequest>();
  requestObject->resetResponse();
  requestObject->setPath(std::string{http_frame_.getPath()});
  requestObject->setRequestBody(std::string{http_frame_.body()});

  if (auto query = http_frame_.getQuery(); !query.empty()) {
    requestObject->setQueryParameters(query);
  }

  // Populate headers from HttpParserFrame
  for (const auto& field : http_frame_.fields) {
    requestObject->setHeader(field.name, field.value);
  }

  return requestObject;
}

HttpServerConnection::EventReport HttpServerConnection::handleReadRequest(Lock& lock, int evtFlags)
{
  EventReport report;
  report.cmd = EventReport::Continue;

  if (evtFlags & PollSet::EventError) {
    report.cmd = EventReport::Disconnect;
    report.disconnectMsg = "POLLERR in ReadRequest";
    report.evtFlags = 0;
    return report;
  }

  if (evtFlags & PollSet::EventIn) {
    // It can switch state to State::ProcessRequest;
    Error err = doReadRequest(lock);

    // Clean up "read" flag.
    report.evtFlags = evtFlags & (~PollSet::EventIn);

    if (err == Error::WouldBlock) {
      report.cmd = EventReport::Repeat;
      return report;
    }
    if (err == Error::EndOfFile) {
      report.cmd = EventReport::Disconnect;
      report.disconnectMsg = "closed by EOF";
      return report;
    } if (err != Error::Ok) {
      LOCAL_ERROR("Unexpected error %s", err.toString());
      report.cmd = EventReport::Disconnect;
      report.disconnectMsg = std::string("closed by unexpected error: ") + err.toString();
      return report;
    }

    // Here we have full request.

    resetResponse();
    auto requestObject = makeRequestObject(lock);
    active_request_ = requestObject;
    std::string path{http_frame_.getPath()};

    network::ClientInfo clientInfo;
    clientInfo.fd = socket_->fd();
    clientInfo.remoteAddress = socket_->peerAddress();
    LOCAL_DEBUG("Handling HTTP request to path=\"%s\"", path.c_str());

    auto endpoints = endpoints_.lock();
    if (!endpoints) {
      LOCAL_ERROR("No handler for endpoint \"%s\"", path.c_str());
      doSerializeResponse(lock, requestObject, Error::FileNotFound);
      report.cmd = EventReport::Continue;
      return report;
    }

    auto [handler, cb] = endpoints->findEndpoint(http_frame_);
    if (!handler) {
      LOCAL_ERROR("No handler for endpoint \"%s\"", path.c_str());
      doSerializeResponse(lock, requestObject, Error::FileNotFound);
      report.cmd = EventReport::Continue;
      return report;
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
      report.cmd = EventReport::BackgroundResponse;
      return report;
    }

    // Process request immediately in current thread
    {
      ScopedUnlock unlock(lock);
      handler->handle(clientInfo, requestObject);
    }

    doSerializeResponse(lock, requestObject, Error::Ok);
    report.cmd = EventReport::Continue;
    // Got full request.
    return report;
  }

  LOCAL_WARN("HttpServerConnection[%d]::handleReadRequest() unhandled events in ReadRequest: %o", debugFd_, evtFlags);
  return report;
}

HttpServerConnection::EventReport HttpServerConnection::handleWaitResponse(Lock& lock, int evtFlags)
{
  EventReport report;

  if (evtFlags & PollSet::EventSoftSignal) {
    report.evtFlags = evtFlags & (~PollSet::EventSoftSignal);
    report.cmd = EventReport::Continue;
  } else {
    report.cmd = EventReport::Repeat;
  }
  // Shift responsibility for errors in this state to next state.

  doSerializeResponse(lock, active_request_, Error::Ok);
  return report;
}

HttpServerConnection::EventReport HttpServerConnection::handleWriteResponse(Lock& lock, int evtFlags)
{
  EventReport report;
  report.cmd = EventReport::Continue;

  if (evtFlags & PollSet::EventError) {
    report.cmd = EventReport::Disconnect;
    report.disconnectMsg = "POLLERR in WriteResponse";
    evtFlags = 0;
    return report;
  }

  if (evtFlags & PollSet::EventOut) {
    assert(active_request_);
    assert(response_header_buffer_.size());

    report.evtFlags = evtFlags & (~PollSet::EventOut);

    std::pair<size_t, Error> r;
    const std::string& responseBody = active_request_->responseBody();
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

    report.bytesWritten = written;

    if (err == Error::WouldBlock) {
      report.cmd = EventReport::Repeat;
      return report;
    }

    if (err == Error::Ok) {
      if (data_sent_ < totalSize) {
        // Failed to write all data, so we need to yield to spinner and try later.
        return report;
      }
      data_sent_ = 0;

      if (active_request_ && active_request_->isUpgradeResponse()) {
        // Connection upgrade: let the handler handle it
        LOCAL_DEBUG("Connection upgrade complete, calling handler");

        auto endpoints = endpoints_.lock();

        if (!endpoints) {
          LOCAL_ERROR("Connection upgrade: endpoints are lost");
          report.cmd = EventReport::Disconnect;
          report.disconnectMsg = "endpoints are lost";
          return report;
        }

        // Get the handler
        auto [handler, cb] = endpoints->findEndpoint(http_frame_);
        if (!handler) {
          LOCAL_ERROR("Connection upgrade: handler not found");
          report.cmd = EventReport::Disconnect;
          report.disconnectMsg = "http endpoint for \"" + std::string(http_frame_.getPath()) + "\"is not found";
          return report;
        }

        network::ClientInfo clientInfo;
        clientInfo.fd = socket_->fd();
        clientInfo.remoteAddress = socket_->peerAddress();

        // Call handler's upgradeComplete method
        Error upgradeErr = Error::NotImplemented;
        {
          auto socket = socket_;
          detachPollSet(lock);
          socket_ = nullptr;
          ScopedUnlock unlock(lock);
          upgradeErr = handler->upgradeComplete(socket, clientInfo, active_request_);
        }
        if (upgradeErr == Error::NotImplemented) {
          LOCAL_ERROR("Connection upgrade: handler does not support upgrades");
        } else if (!upgradeErr) {
          LOCAL_ERROR("Connection upgrade: handler returned error %s", upgradeErr.toString());
        }

        // Don't continue with normal HTTP processing
        report.cmd = EventReport::Upgrade;
        return report;
      }

      http_frame_.finishRequest();
      report.cmd = EventReport::Continue;
      double dur = (SteadyTime::now() - request_start_).toSec() * 1000;
      LOCAL_DEBUG("Served HTTP response in %fms", dur);
    } else {
      LOCAL_ERROR("HttpServerConnection writeResponse error: %s", err.toString());
    }
  }
  return report;
}

HttpServerConnection::EventReport HttpServerConnection::handleSocketEvents(Lock& lock, int evtFlags)
{
  const int startEvt = evtFlags;

  LOCAL_DEBUG("HttpServerConnection[%d]::handleEvents(%s) in state %s", debugFd_, PollSet::eventToString(startEvt).c_str(), state_.toString());

  // PollSet::EventSoftSignal is sent either when server is dropping all of its connections,
  // or notification from some thread that response is ready.
  if (evtFlags & PollSet::EventSoftSignal) {
    if (!server_lifetime_) {
      EventReport report;
      report.cmd = EventReport::Disconnect;
      report.disconnectMsg = "Detached from HttpServer";
      report.evtFlags = 0;
      return report;
    }
  }

  // Set to true if we transition multiple FSM states.
  if (state_ == State::ReadRequest) {
    EventReport report = handleReadRequest(lock, evtFlags);
    assert(report.cmd != EventReport::Invalid);
    evtFlags = report.evtFlags;
    if (report.cmd == EventReport::Disconnect) {
      updateState(lock, State::Disconnected);
      return report;
    } else if (report.cmd == EventReport::Continue) {
      updateState(lock, State::WriteResponse);
      evtFlags |= PollSet::EventOut;
    } else if (report.cmd == EventReport::BackgroundResponse) {
      updateState(lock, State::WaitResponse);
      // Yield to poller and wait for EventSoftSignal.
      return {};
    }
  }

  if (state_ == State::WaitResponse) {
    EventReport report = handleWaitResponse(lock, evtFlags);
    assert(report.cmd != EventReport::Invalid);
    evtFlags = report.evtFlags;
    if (report.cmd == EventReport::Continue) {
      updateState(lock, State::WriteResponse);
      evtFlags |= PollSet::EventOut;
    }
  }

  if (state_ == State::WriteResponse) {
    EventReport report = handleWriteResponse(lock, evtFlags);
    assert(report.cmd != EventReport::Invalid);
    evtFlags = report.evtFlags;

    if (report.cmd == EventReport::Disconnect || report.cmd == EventReport::Upgrade) {
      updateState(lock, State::Disconnected);
      return report;
    }

    if (report.cmd == EventReport::Continue) {
      updateState(lock, State::ReadRequest);
    }
    // We could instantly try to read next data right now and make a loop here.
    // But we are lazy and try to read next time.
  }

  if (evtFlags != 0) {
    LOCAL_WARN("HttpServerConnection::handleEvents(%s) unhandled events: %s in state %s",
      PollSet::eventToString(startEvt).c_str(),
      PollSet::eventToString(evtFlags).c_str(),
      state_.toString());
  }

  return {};
}

void HttpServerConnection::updateEventsForSocket(Lock& /*lock*/)
{
  if (!poll_set_)
    return;
  if (socket_ && socket_->valid()) {
    poll_set_->setEvents(socket_->fd(), eventsForState(state_));
  }
}

void HttpServerConnection::doSerializeResponse(Lock& lock, const std::shared_ptr<HttpRequest>& req, Error error)
{
  // Prepare fault response if handler returned error
  if (!error) {
    prepareFaultResponse(error, *req);
  }
  const HttpResponseHeader& responseHeader = req->responseHeader();
  active_request_ = req;
  responseHeader.writeHeader(response_header_buffer_, req->responseBody().size());
}

int HttpServerConnection::eventsForState(State state) const
{
  switch (state) {
    case State::WriteResponse:
      return PollSet::EventOut;
    case State::ReadRequest:
      return PollSet::EventIn;
    case State::WaitResponse:
      return 0; //< PollSet::EventSoftSignal are implicit events, no need to subscribe to them.
    case State::Disconnected:
      return 0;
    default:
      return 0;
  }
}

void HttpServerConnection::closeSocket()
{
  socket_->close();
  socket_.reset();
}

void HttpServerConnection::resetResponse()
{
  response_header_buffer_.resize(0);
  active_request_.reset();
  data_sent_ = 0;
}

std::shared_ptr<network::NetSocket> HttpServerConnection::extractSocketForUpgrade()
{
  Lock lock(guard_, THIS_LOCATION);
  server_lifetime_.reset();

  if (!socket_) {
    LOCAL_ERROR("HttpServerConnection::extractSocketForUpgrade: socket is null");
    return nullptr;
  }

  auto socket = socket_;
  socket_.reset();
  return socket;
}

void HttpServerConnection::onAsyncRequestComplete(const std::shared_ptr<HttpRequest>& request, Error err)
{
  Lock lock(guard_, THIS_LOCATION);
  
  // Check if connection is still in ProcessRequest state
  if (state_ != State::WaitResponse) {
    LOCAL_DEBUG("HttpServerConnection[%d]::onAsyncRequestComplete(%d) unexpected state for sending response %s", debugFd_, request->id(), state_.toString());
    return;
  }

  if (poll_set_ && socket_ && socket_->valid()) {
    poll_set_->signalFd(socket_->fd());
  } else {
    // TODO: Close self.
    LOCAL_ERROR("HttpServerConnection[%d]::onAsyncRequestComplete(%d) socket is lost in state %s", debugFd_, request->id(), state_.toString());
  }
}

void HttpServerConnection::updateState(Lock& lock, State newState)
{
  if (state_ == newState)
    return;

  auto oldState = state_;
  state_ = newState;
  LOCAL_DEBUG("HttpServerConnection[%d]::updateState from %s to %s", debugFd_, oldState.toString(), newState.toString());
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
