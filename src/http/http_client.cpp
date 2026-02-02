//
// Created by dkargin on 11/14/25.
//

#include <atomic>
#include <cassert>
#include <chrono>
#include <memory>

#include "miniros/network/socket.h"
#include "miniros/http/http_client.h"

#include "internal/invokers.h"
#include "network/net_adapter.h"
#include "io/poll_set.h"

#define MINIROS_PACKAGE_NAME "http"

namespace miniros {
namespace http {

const char* HttpClient::State::toString() const
{
  switch (value) {
    case Invalid:
      return "Invalid";
    case Connecting:
      return "Connecting";
    case WaitReconnect:
      return "WaitReconnect";
    case Idle:
      return "Idle";
    case WriteRequest:
      return "WriteRequest";
    case ReadResponse:
      return "ReadResponse";
    case ProcessResponse:
      return "ProcessResponse";
    default:
      assert(false);
      return "Unknown";
  }
}

struct HttpClient::Internal {

  /// A queue of requests.
  std::deque<std::shared_ptr<HttpRequest>> requests;
  /// Mutex for storing requests
  std::mutex requests_guard;

  std::shared_ptr<HttpRequest> active_request;

  /// Remote address.
  /// It is set during `connect` call and reused in reconnects.
  network::NetAddress address;

  /// Internal state.
  State state = State::Invalid;

  /// Number of bytes sent from current part (header or body).
  size_t data_sent = 0;

  HttpParserFrame http_frame;

  /// Intermediate storage for request header.
  std::string request_header_buffer;
  /// Intermediate storage for request body.
  std::string request_body;

  /// Assigned socket.
  std::shared_ptr<network::NetSocket> socket;

  /// Serving PollSet.
  PollSet* poll_set = nullptr;

  /// Mutex for processing socket state.
  std::mutex process_guard;

  std::atomic_int updateCounter = 0;

  /// Condition variable for state change.
  std::condition_variable_any cv;

  Internal(PollSet* ps) : poll_set(ps)
  {
    assert(ps);
  }

  ~Internal()
  {
    if (socket) {
      poll_set->delSocket(socket->fd());
    }
    MINIROS_DEBUG("HttpClient::~Internal()");
  }

  void resetResponse();
  Error readResponse();

  int fd() const
  {
    return socket ? socket->fd() : 0;
  }

  /// Get name for debug otput
  std::string debugName() const
  {
    return address.str() + std::string(" fd=") + std::to_string(fd());
  }

  void updateState(State newState)
  {
    if (newState == state)
      return;

    MINIROS_DEBUG("HttpClient[%s]::updateState(%s) from %s at step %d", debugName().c_str(), newState.toString(), state.toString(), updateCounter.load());
    state = newState;
    cv.notify_all();
  }

  /// Close socket.
  void close()
  {
    if (socket && poll_set) {
      poll_set->delSocket(socket->fd());
    }
    socket.reset();
  }

  /// Pulls new task from queue and updates state.
  bool pullNewTaskUnsafe();
};

int HttpClient::eventsForState(State state)
{
  switch (state) {
    case State::Invalid:
      return 0;
    case State::Connecting:
      return PollSet::EventOut;
    case State::WaitReconnect:
      return 0;
    case State::Idle:
      return PollSet::EventError;
    case State::WriteRequest:
      return PollSet::EventOut | PollSet::EventError;
    case State::ReadResponse:
      return PollSet::EventIn;
    case State::ProcessResponse:
      return 0;
    default:
      return 0;
  }
  return 0;
}

void HttpClient::Internal::resetResponse()
{
  request_body.resize(0);
  request_header_buffer.resize(0);
  data_sent = 0;
}

bool HttpClient::Internal::pullNewTaskUnsafe()
{
  if (requests.empty()) {
    active_request = {};
    return false;
  }

  active_request = requests.front();
  requests.pop_front();

  if (!active_request) {
    MINIROS_WARN_NAMED("client", "HttpClient::pullNewTask: null request");
    return false;
  }

  // Update request status
  active_request->updateState(HttpRequest::State::ClientSending);
  active_request->setRequestStart(SteadyTime::now());

  // Build request header from HttpRequest (host/port managed by HttpClient)
  request_header_buffer = active_request->buildHeader(address.address, address.port());
  request_body = active_request->requestBody();
  data_sent = 0;

  return true;
}

void HttpClient::handleDisconnect(Lock& lock)
{
  if (!internal_)
    return;

  assert(internal_->socket);
  // Note: This method assumes the lock is already held by the caller (handleEvents)
  // We don't acquire the lock here to avoid deadlock

  int fd = internal_->fd();
  // Call onDisconnect callback if set.
  if (onDisconnect && internal_->socket) {

    DisconnectResponse response = invokeUnlocked(lock, onDisconnect, internal_->socket, internal_->state);
    
    if (response.reconnect && internal_->address.valid()) {
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleDisconnect: attempting reconnection to %s", internal_->debugName().c_str(), internal_->address.str().c_str());
      // If reconnect timeout is specified, we'd need a timer mechanism
      // For now, attempt immediate reconnection (timeout of 0 means immediate)
      if (response.reconnectTimeout > 0) {
        MINIROS_WARN_NAMED("client", "HttpClient[%s]::handleDisconnect: reconnectTimeout=%zu ms not yet implemented, reconnecting immediately", internal_->debugName().c_str(), response.reconnectTimeout);
      }

      // Enqueue reconnect event.
      internal_->poll_set->setTimerEvent(internal_->socket->fd(), response.reconnectTimeout);
      internal_->updateState(State::WaitReconnect);
      return;
    }
  } else {
    MINIROS_INFO_NAMED("client", "HttpClient[%s]::handleDisconnect() - unhandled disconnect", internal_->debugName().c_str());
  }
  // No callback set or no reconnect required. Just close the current socket and reset state.
  internal_->close();
  internal_->updateState(State::Invalid);
  MINIROS_INFO_NAMED("client", "HttpClient[%s]::handleDisconnect() - connection is closed", internal_->debugName().c_str());
}

Error HttpClient::Internal::readResponse()
{
  auto [transferred, readErr] = socket->recv(http_frame.data, nullptr);

  if (transferred > 0) {
    const int parsed = http_frame.incrementalParse();

    const int fd = socket->fd();
    double dur = (SteadyTime::now() - active_request->getRequestStart()).toSec() * 1000;
    MINIROS_DEBUG_NAMED("client", "HttpClient(%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", fd, http_frame.contentLength(), parsed, dur);

    // If we haven't gotten the entire request yet, return (keep reading)
    if (http_frame.state() != HttpParserFrame::ParseComplete) {
      if (readErr == Error::EndOfFile) {
        MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::readRequest: EOF while reading request", debugName().c_str());
        http_frame.finishRequest();
        return Error::EndOfFile;
        // Either way we close the connection
      }
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::readRequest got only %d/%d bytes.", debugName().c_str(), http_frame.bodyLength(), http_frame.contentLength());
      return Error::WouldBlock;
    }

    assert(http_frame.state() == HttpParserFrame::ParseComplete);
    // Otherwise, parse and dispatch the request
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::readRequest read %d/%d bytes.", debugName().c_str(), http_frame.bodyLength(), http_frame.contentLength());
  } else if (readErr == Error::WouldBlock) {
    return Error::WouldBlock;
  }
  return Error::Ok;
}

HttpClient::HttpClient(PollSet* ps)
{
  internal_.reset(new Internal(ps));
}

HttpClient::~HttpClient()
{
  if (internal_) {
    std::unique_lock guard(internal_->process_guard);
    std::shared_ptr<Internal> copy;
    std::swap(internal_, copy);
    copy->close();
  }
  MINIROS_DEBUG_NAMED("client", "HttpClient::~HttpClient()");
}

HttpClient::State HttpClient::getState() const
{
  if (!internal_)
    return State::Invalid;
  std::unique_lock lock(internal_->process_guard);
  return internal_->state;
}

bool HttpClient::isActive() const
{
  State s = getState();
  if (s == State::Invalid || s == State::Idle)
    return false;
  return true;
}

void HttpClient::close()
{
  if (!internal_)
    return;

  std::unique_lock lock(internal_->process_guard);
  if (internal_->socket && internal_->socket->valid()) {
    internal_->socket->close();
  }

  internal_->state = State::Invalid;
}

std::shared_ptr<network::NetSocket> HttpClient::detach()
{
  internal_->state = State::Invalid;
  auto sock = internal_->socket;
  internal_->socket.reset();
  return sock;
}

Error HttpClient::enqueueRequest(const std::shared_ptr<HttpRequest>& request)
{
  assert(request);
  if (!request) {
    return Error::InvalidValue;
  }

  if (!internal_) {
    return Error::InternalError;
  }

  {
    std::unique_lock lock(internal_->requests_guard);
    // Set request status to Queued
    request->updateState(HttpRequest::State::ClientQueued);
    internal_->requests.push_back(request);
  }

  std::unique_lock lock2(internal_->process_guard);
  // If we're in Idle state and have a valid socket, trigger processing
  if (internal_->state == State::Idle && internal_->socket && internal_->socket->valid()) {
    // Trigger pullNewTask by updating socket events
    // The socket is already in PollSet, so we can trigger it by updating events
    // For now, we'll let the next event cycle handle it, or we could use a callback
    // The simplest approach is to let the existing event loop handle it
    if (internal_->pullNewTaskUnsafe()) {
      internal_->updateState(State::WriteRequest);
      if (!internal_->poll_set->setEvents(internal_->socket->fd(), eventsForState(internal_->state))) {
        MINIROS_ERROR_NAMED("client", "HttpClient(%d) Failed to update poll set to enable events to send request", internal_->fd());
      }
    }
  }

  return Error::Ok;
}

Error HttpClient::connect(const std::string& host, int port)
{
  network::NetAddress address = network::NetAddress::fromString(network::NetAddress::AddressIPv4, host, port);
  if (!address.valid())
    return Error::InvalidAddress;

  std::unique_lock lock(internal_->process_guard);
  return connectImplUnsafe(address);
}

Error HttpClient::connectImplUnsafe(const network::NetAddress& address)
{
  if (internal_->socket && internal_->socket->valid()) {
    close();
  }

  if (!internal_->socket)
    internal_->socket.reset(new NetSocket());

  Error err = internal_->socket->tcpConnect(address, true);
  bool connected = false;
  if (err == Error::WouldBlock) {
    // This is fine.
  } else if (err == Error::Ok) {
    connected = true;
  } else {
    // Failed to initialize connection.
    MINIROS_WARN_NAMED("client", "HttpClient::connect failed to initialize connection to %s:%d", address.address.c_str(), address.port());
    return err;
  }

  internal_->socket->setKeepAlive(true);

  // Store host and port for building request headers
  internal_->address = address;

  auto copy = internal_;
  internal_->poll_set->addSocket(internal_->socket->fd(), PollSet::EventIn | PollSet::EventOut | PollSet::EventUpdate | PollSet::EventError,
    [copy, this](int event)
    {
      return handleSocketEvents(event);
    }, copy);

  internal_->updateState(connected ? State::Idle : State::Connecting);
  return Error::Ok;
}

Error HttpClient::waitConnected(const WallDuration& duration)
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->socket || !internal_->socket->valid())
    return Error::InvalidHandle;

  std::unique_lock lock(internal_->process_guard);
  if (internal_->cv.wait_for(lock, std::chrono::duration<double>(duration.toSec()),
    [this]() {
      if (internal_->state == State::Connecting)
        return false;
      return true;
    })) {
    if (internal_->state == State::Idle)
      return Error::Ok;

    return Error::NotConnected;
  }
  return Error::Timeout;
}

int HttpClient::handleSocketEvents(int evtFlags)
{
  if (!internal_)
    return 0;

  std::unique_lock lock(internal_->process_guard);
  ++internal_->updateCounter;

  // The following states are expected to fall through.
  // Each "handle" method can update internal state.
  // The order of processing of these states corresponds to expected flow of FSM.
  int stateChanges = 0;
  bool loop = true;
  do {
    if (internal_->state == State::WaitReconnect) {
      // Expecting to be here on a timeout.
      handleReconnecting(lock, evtFlags);
      if (internal_->state == State::Invalid)
        break;
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::Connecting) {
      handleConnecting(lock, evtFlags);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::Idle) {
      if (evtFlags & PollSet::EventError) {
        handleDisconnect(lock);
      } else {
        std::unique_lock lock2(internal_->requests_guard);
        if (internal_->pullNewTaskUnsafe()) {
          internal_->updateState(State::WriteRequest);
        }
      }
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::WriteRequest) {
      handleWriteRequest(lock, evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::ReadResponse) {
      handleReadResponse(lock, evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::ProcessResponse) {
      handleProcessResponse(lock);
      stateChanges++;
      // Restart loop to attempt to start sending new request.
      if (internal_->state == State::Idle)
        continue;
    }
    loop = false;
  }while (loop);

  if (internal_->state == State::Idle && (evtFlags & PollSet::EventOut)) {
    // Nothing to do here.
  } else if (stateChanges == 0) {
    std::string evt = PollSet::eventToString(evtFlags);
    MINIROS_WARN("HttpClient unhandled events: %s in state %s", evt.c_str(), internal_->state.toString());
  }
  return eventsForState(internal_->state);
}

void HttpClient::handleReconnecting(Lock& lock, int events)
{
  if (events & PollSet::EventOut) {
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - connected", internal_->debugName().c_str(), PollSet::eventToString(events).c_str());
    connectImplUnsafe(internal_->address);
  } else if (events & PollSet::EventError) {
    MINIROS_ERROR_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - failed to connect", internal_->debugName().c_str(), PollSet::eventToString(events).c_str());
  } else {
    MINIROS_ERROR_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - unhandled events", internal_->debugName().c_str(), PollSet::eventToString(events).c_str());
  }
}

void HttpClient::handleConnecting(Lock& lock, int evtFlags)
{
  if (internal_->state != State::Connecting)
    return;

  // Assume socket is in "connecting" state
  assert(internal_->socket->isConnecting());
  if (evtFlags & PollSet::EventError) {
    MINIROS_ERROR_NAMED("client", "HttpClient[%s]::handleEvents(state=Connecting) - failed to connect", internal_->debugName().c_str());
    handleDisconnect(lock);
    return;
  }

  if (evtFlags & PollSet::EventOut) {
    Error err = internal_->socket->checkConnected();
    if (err == Error::Timeout) // Still connecting.
      return;

    if (err == Error::Ok) {
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleEvents(state=Connecting) - connected", internal_->debugName().c_str());
      // Connection is complete.
      internal_->updateState(State::Idle);
    } else {
      MINIROS_ERROR_NAMED("HttpClient", "HttpClient[%s]::handleEvents(state=Connecting) - error: %s", internal_->debugName().c_str(), err.toString());
      handleDisconnect(lock);
    }
  }
}

void HttpClient::handleWriteRequest(Lock& lock, int evtFlags, bool fallThrough)
{
  if (evtFlags & PollSet::EventError) {
    int err = internal_->socket->getSysError();
    // Here we can have some disconnect problem or some generic network problem.
    MINIROS_WARN_NAMED("client", "HttpClient[%s]::handleWriteRequest socket error in WriteRequest: %i", internal_->debugName().c_str(), err);
    handleDisconnect(lock);
    return;
  }

  if (fallThrough || (evtFlags & PollSet::EventOut)) {
    std::pair<size_t, Error> r;
    if (internal_->request_body.size() > 0) {
      // Send header+body packet.
      r = internal_->socket->write2(
        internal_->request_header_buffer.c_str(), internal_->request_header_buffer.size(),
        internal_->request_body.c_str(), internal_->request_body.size(), internal_->data_sent);
    } else {
      // Send single header packet.
      const char* data = internal_->request_header_buffer.c_str() + internal_->data_sent;
      size_t toSend = internal_->request_header_buffer.size() - internal_->data_sent;
      r = internal_->socket->send(data, toSend, nullptr);
    }

    const size_t totalSize = internal_->request_header_buffer.size() + internal_->request_body.size();
    auto [written, err] = r;

    if (written > 0) {
      internal_->data_sent += written;
      double dur = (SteadyTime::now() - internal_->active_request->getRequestStart()).toSec() * 1000;
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleWriteRequest() HttpClient written %d/%d bytes of header, t=%fms",
        internal_->debugName().c_str(), static_cast<int>(written), static_cast<int>(totalSize), dur);
    }

    if (err == Error::WouldBlock) {
      // Failed to write all data. Retry later.
      return;
    }

    if (err != Error::Ok) {
      MINIROS_ERROR_NAMED("client", "HttpClient[%s]::handleWriteRequest error: %s", internal_->debugName().c_str(), err.toString());
      handleDisconnect(lock);
      return;
    }

    if (internal_->data_sent < totalSize) {
      // Failed to write all data, so we need to yield to spinner.
      return;
    }

    internal_->data_sent = 0;
    // Request sent, now wait for response
    internal_->active_request->updateState(HttpRequest::State::ClientWaitResponse);
    internal_->updateState(State::ReadResponse);
    internal_->http_frame.resetParseState(false); // false = response mode
    auto dur = SteadyTime::now() - internal_->active_request->getRequestStart();
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleWriteRequest sent request in %fms, waiting for response",
      internal_->debugName().c_str(), dur.toSec()*1000);
    return;
  }

  // Some unhandled event.
  std::string evt = PollSet::eventToString(evtFlags);
  MINIROS_WARN_NAMED("client", "HttpClient[%s]::handleWriteRequest unhandled event in WriteRequest: %s", internal_->debugName().c_str(), evt.c_str());
}

void HttpClient::handleReadResponse(Lock& lock, int evtFlags, bool fallThrough)
{
  if (evtFlags & PollSet::EventError) {
    MINIROS_WARN_NAMED("client", "HttpClient[%s]::handleReadResponse() got socket error", internal_->debugName().c_str());
    handleDisconnect(lock);
    return;
  }

  if (fallThrough || evtFlags & PollSet::EventIn) {
    Error err = internal_->readResponse();

    if (err == Error::Ok) {
      internal_->updateState(State::ProcessResponse);
    } else if (err == Error::WouldBlock) {
      // Got partial data. Need to wait.
      return;
    }
    else if (err == Error::EndOfFile) {
      MINIROS_ERROR("Got end of file from remote server");
      // Connection closed by peer
      handleDisconnect(lock);
    } else {
      MINIROS_ERROR("Unexpected error %s", err.toString());
      handleDisconnect(lock);
    }
  } else {
    std::string evt = PollSet::eventToString(evtFlags);
    MINIROS_WARN_NAMED("client", "HttpClient[%s]::handleReadResponse() unhandled events in ReadResponse: %s at %d",
      internal_->debugName().c_str(), evt.c_str(), internal_->updateCounter.load());
  }
}

void HttpClient::handleProcessResponse(Lock& lock)
{
  // Response received and parsed - store it in HttpRequest
  if (internal_->active_request) {
    auto req = internal_->active_request;
    req->setResponseHeader(internal_->http_frame);

    // Store response body.
    std::string_view body = internal_->http_frame.body();
    if (!body.empty()) {
      req->setResponseBody(body.data(), body.size());
    }

    // Status should be updated last to prevent possible racing with access to response body.
    req->updateState(HttpRequest::State::ClientHasResponse);

    internal_->resetResponse();

    auto dur = req->getRequestFinish() - req->getRequestStart();
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleProcessResponse() received response in %fms",
      internal_->debugName().c_str(), dur.toSec()*1000);

    if (onResponse) {
      invokeUnlocked(lock, onResponse, req);
    }
    req->processResponse();
  }

  internal_->updateState(State::Idle);
}

size_t HttpClient::getQueuedRequests() const
{
  if (!internal_)
    return 0;
  std::unique_lock lock(internal_->requests_guard);
  size_t num = internal_->requests.size();
  if (internal_->active_request)
    num++;
  return num;
}

}
}