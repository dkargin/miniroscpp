//
// Created by dkargin on 11/14/25.
//

#include "http/http_client.h"

#include <cassert>
#include <chrono>
#include <cstring>
#include <memory>

#include "miniros/network/socket.h"
#include "miniros/http/http_client.h"

#include "network/net_adapter.h"
#include "transport/poll_set.h"

#include <atomic>

namespace miniros {
namespace http {

const char* toString(HttpClient::State state)
{
  switch (state) {
    case HttpClient::State::Invalid:
      return "Invalid";
    case HttpClient::State::Connecting:
      return "Connecting";
    case HttpClient::State::WaitReconnect:
      return "WaitReconnect";
    case HttpClient::State::Idle:
      return "Idle";
    case HttpClient::State::WriteRequest:
      return "WriteRequest";
    case HttpClient::State::ReadResponse:
      return "ReadResponse";
    case HttpClient::State::ProcessResponse:
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
  std::condition_variable cv;

  Internal(PollSet* ps) : poll_set(ps)
  {
    assert(ps);
  }

  ~Internal()
  {
    if (socket) {
      poll_set->delSocket(socket->fd());
    }
    MINIROS_DEBUG_NAMED("destructor", "HttpClient::~Internal()");
  }

  void resetResponse();
  Error readResponse();

  void updateState(State newState)
  {
    if (newState == state)
      return;

    MINIROS_INFO_NAMED("HttoClient", "Changing state from %s to %s at %d", toString(state), toString(newState), updateCounter.load());
    state = newState;
    cv.notify_all();
  }

  /// Close socket.
  void close()
  {
    if (socket) {
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
      return 0;
    case State::WriteRequest:
      return PollSet::EventOut;
    case State::ReadResponse:
      return PollSet::EventIn;
    case State::ProcessResponse:
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
    MINIROS_WARN("HttpClient::pullNewTask: null request");
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

void HttpClient::handleDisconnect()
{
  if (!internal_)
    return;

  assert(internal_->socket);
  // Note: This method assumes the lock is already held by the caller (handleEvents)
  // We don't acquire the lock here to avoid deadlock

  // Call onDisconnect callback if set
  if (onDisconnect && internal_->socket) {
    DisconnectResponse response = onDisconnect(internal_->socket);
    
    if (response.reconnect && internal_->address.valid()) {
      MINIROS_DEBUG_NAMED("HttpClient", "handleDisconnect: attempting reconnection to %s", internal_->address.str().c_str());
      // If reconnect timeout is specified, we'd need a timer mechanism
      // For now, attempt immediate reconnection (timeout of 0 means immediate)
      if (response.reconnectTimeout > 0) {
        MINIROS_WARN_NAMED("HttpClient", "handleDisconnect: reconnectTimeout=%zu ms not yet implemented, reconnecting immediately", response.reconnectTimeout);
      }

      // Enqueue reconnect event.
      internal_->poll_set->setTimerEvent(internal_->socket->fd(), response.reconnectTimeout);
      internal_->updateState(State::WaitReconnect);
      return;
    }
  }
  // No callback set or no reconnect required. Just close the current socket and reset state.
  internal_->close();
  internal_->updateState(State::Invalid);
}

Error HttpClient::Internal::readResponse()
{
  auto [transferred, readErr] = socket->recv(http_frame.data, nullptr);

  if (transferred > 0) {
    const int parsed = http_frame.incrementalParse();

    const int fd = socket->fd();
    double dur = (SteadyTime::now() - active_request->getRequestStart()).toSec() * 1000;
    MINIROS_DEBUG("HttpClient(%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", fd, http_frame.contentLength(), parsed, dur);

    // If we haven't gotten the entire request yet, return (keep reading)
    if (http_frame.state() != HttpParserFrame::ParseComplete) {
      if (readErr == Error::EndOfFile) {
        MINIROS_DEBUG("HttpClient(%d)::readRequest: EOF while reading request", fd);
        http_frame.finishRequest();
        return Error::EndOfFile;
        // Either way we close the connection
      }
      MINIROS_DEBUG("HttpClient(%d)::readRequest got only %d/%d bytes.", fd, http_frame.bodyLength(), http_frame.contentLength());
      return Error::WouldBlock;
    }

    assert(http_frame.state() == HttpParserFrame::ParseComplete);
    // Otherwise, parse and dispatch the request
    MINIROS_DEBUG("HttpClient(%d)::readRequest read %d/%d bytes.", fd, http_frame.bodyLength(), http_frame.contentLength());
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
    std::unique_lock<std::mutex> guard(internal_->process_guard);
    std::shared_ptr<Internal> copy;
    std::swap(internal_, copy);
    copy->close();
  }
  MINIROS_DEBUG_NAMED("destructor", "HttpClient::~HttpClient()");
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
  if (!request) {
    return Error::InvalidValue;
  }

  if (!internal_) {
    return Error::InternalError;
  }

  {
    std::unique_lock<std::mutex> lock(internal_->requests_guard);
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
      internal_->poll_set->setEvents(internal_->socket->fd(), eventsForState(internal_->state));
    }
  }

  return Error::Ok;
}

Error HttpClient::connect(const std::string& host, int port)
{
  network::NetAddress address = network::NetAddress::fromIp4String(host, port);
  if (!address.valid())
    return Error::InvalidAddress;

  std::unique_lock<std::mutex> lock(internal_->process_guard);
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
    MINIROS_WARN("HttpClient::connect failed to initialize connection to %s:%d", address.address.c_str(), address.port());
    return err;
  }

  // Store host and port for building request headers
  internal_->address = address;

  auto copy = internal_;
  internal_->poll_set->addSocket(internal_->socket->fd(), PollSet::EventIn | PollSet::EventOut | PollSet::EventUpdate,
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

  std::unique_lock<std::mutex> lock(internal_->process_guard);
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

  std::unique_lock<std::mutex> lock(internal_->process_guard);
  ++internal_->updateCounter;

  if (internal_->state == State::WaitReconnect) {
    connectImplUnsafe(internal_->address);
    return eventsForState(internal_->state);
  }

  // The following states are expected to fall through.
  // The order of processing of these states corresponds to the flow of FSM.
  int stateChanges = 0;
  bool loop = true;
  do {
    if (internal_->state == State::Connecting) {
      handleConnecting(evtFlags);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::Idle) {
      std::unique_lock lock2(internal_->requests_guard);
      if (internal_->pullNewTaskUnsafe()) {
        internal_->updateState(State::WriteRequest);
      }
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::WriteRequest) {
      handleWriteRequest(evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::ReadResponse) {
      handleReadResponse(evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (internal_->state == State::ProcessResponse) {
      handleProcessResponse();
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
    MINIROS_WARN("HttpClient unhandled events: %s in state %s", evt.c_str(), toString(internal_->state));
  }
  return eventsForState(internal_->state);
}

void HttpClient::handleConnecting(int evtFlags)
{
  if (internal_->state != State::Connecting)
    return;

  // Assume socket is in "connecting" state
  assert(internal_->socket->isConnecting());
  if (evtFlags & PollSet::EventError) {
    MINIROS_ERROR_NAMED("HttpClient", "handleEvents(state=Connecting) - failed to connect");
    handleDisconnect();
    return;
  }

  if (evtFlags & PollSet::EventOut) {
    Error err = internal_->socket->checkConnected();
    if (err == Error::Timeout) // Still connecting.
      return;

    if (err == Error::Ok) {
      MINIROS_DEBUG_NAMED("HttpClient", "handleEvents(state=Connecting) - connected");
      // Connection is complete.
      internal_->updateState(State::Idle);
    } else {
      MINIROS_ERROR_NAMED("HttpClient", "handleEvents(state=Connecting) - error: %s", err.toString());
      handleDisconnect();
    }
  }
}

void HttpClient::handleWriteRequest(int evtFlags, bool fallThrough)
{
  if (evtFlags & PollSet::EventError) {
    int err = internal_->socket->getSysError();
    // Here we can have some disconnect problem or some generic network problem.
    MINIROS_WARN("HttpClient::handleEvents socket error in WriteRequest: %i", err);
    handleDisconnect();
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
      MINIROS_DEBUG("HttpClient written %d/%d bytes of header, t=%fms",
        static_cast<int>(written), static_cast<int>(totalSize), dur);
    }

    if (err == Error::WouldBlock) {
      // Failed to write all data. Retry later.
      return;
    }

    if (err != Error::Ok) {
      MINIROS_ERROR("HttpClient writeResponse error: %s", err.toString());
      handleDisconnect();
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
    MINIROS_DEBUG("HttpClient sent request in %fms, waiting for response", dur.toSec()*1000);
    return;
  }

  // Some unhandled event.
  std::string evt = PollSet::eventToString(evtFlags);
  MINIROS_WARN("HttpClient unhandled event in WriteRequest: %s", evt.c_str());
}

void HttpClient::handleReadResponse(int evtFlags, bool fallThrough)
{
  if (evtFlags & PollSet::EventError) {
    MINIROS_WARN("HttpClient::handleEvents socket error in ReadResponse");
    handleDisconnect();
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
      handleDisconnect();
    } else {
      MINIROS_ERROR("Unexpected error %s", err.toString());
      handleDisconnect();
    }
  } else {
    std::string evt = PollSet::eventToString(evtFlags);
    MINIROS_WARN("HttpClient unhandled events in ReadResponse: %s at %d", evt.c_str(), internal_->updateCounter.load());
  }
}

void HttpClient::handleProcessResponse()
{
  // Response received and parsed - store it in HttpRequest
  if (internal_->active_request) {
    internal_->active_request->setResponseHeader(internal_->http_frame);

    // Store response body
    std::string_view body = internal_->http_frame.body();
    if (!body.empty()) {
      internal_->active_request->setResponseBody(body.data(), body.size());
    }

    // Status should be updated last to prevent possible racing with access to response body.
    internal_->active_request->updateState(HttpRequest::State::ClientHasResponse);
    auto dur = internal_->active_request->getRequestFinish() - internal_->active_request->getRequestStart();
    MINIROS_DEBUG("HttpClient received response in %fms", dur.toSec()*1000);
  }

  internal_->resetResponse();
  internal_->updateState(State::Idle);
}

}
}