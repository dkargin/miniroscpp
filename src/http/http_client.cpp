//
// Created by dkargin on 11/14/25.
//

#include <deque>
#include <atomic>
#include <cassert>
#include <chrono>
#include <memory>

#define MINIROS_PACKAGE_NAME "http"

#include "miniros/console.h"
#include "miniros/network/socket.h"
#include "miniros/http/http_client.h"

#include "internal/profiling.h"
#include "internal/scoped_locks.h"
#include "io/poll_set.h"
#include "network/net_adapter.h"

namespace miniros {
namespace http {

using Lock = TimeCheckLock<std::mutex>;

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
    case Disconnected:
      return "Disconnected";
    default:
      assert(false);
  }
  return "Unknown";
}

bool HttpClient::State::hasRequest() const
{
  return value == WriteRequest || value == ReadResponse || value == ProcessResponse;
}


struct HttpClient::Internal {

  DisconnectHandler onDisconnect;

  ConnectHandler onConnect;

  ResponseHandler onResponse;
  /// Idle timeout handler and timeout value.
  TimeoutHandler onIdleTimeout;
  int idleTimeoutMs = 0;
  int reconnectAttempts = 0;

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

  /// HTTP parser state for response.
  HttpParserFrame response_http_frame;

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
    Lock lock(process_guard, THIS_LOCATION);
    release(lock);
  }

  void resetIntermediateBuffers();
  Error readResponse();

  int fd() const
  {
    return socket ? socket->fd() : -1;
  }

  /// Get socket events for specified state.
  int eventsForState(State state) const;



  /// Does actual connection.
  /// @param I - reference to self/Internal. Used to keep instance alive.
  /// @param lock - reference to locked mutex. Used to enforce locking before entrance to this method.
  /// @param newAddress - address for connection.
  Error connectImpl(const std::shared_ptr<Internal>& I, Lock& lock, const network::NetAddress& newAddress);

  void handleDisconnect(Lock& lock);

  /// Handle State::Connecting.
  /// Can switch to Idle.
  /// It is expected that idle state is handled right after 'handleConnecting'
  void handleConnecting(Lock& lock, int evtFlags);

  /// Handle State::Reconnecting.
  /// Can switch to:
  ///    - Connecting
  ///    - Invalid
  void handleReconnecting(const std::shared_ptr<Internal>& I , Lock& lock, int events);

  /// Handle State::WriteRequest.
  /// Can switch to:
  ///   - ReadResponse when all request is sent
  ///   - WaitReconnect if error and reconnect is required.
  void handleWriteRequest(Lock& lock, int events, bool fallThrough);

  /// Handle State::ReadResponse.
  /// Can switch to:
  ///   - ReadResponse when all request is sent
  ///   - WaitReconnect if error and reconnect is required.
  void handleReadResponse(Lock& lock, int events, bool fallThrough);

  /// Handle State::ProcessResponse.
  /// Can switch to:
  ///    - WriteRequest if there is another request.
  ///    - Idle if no requests.
  void handleProcessResponse(Lock& lock);

  /// Get name for debug otput
  std::string debugName() const
  {
    return address.str() + std::string(" fd=") + std::to_string(fd());
  }

  void updateState(Lock& lock, State newState)
  {
    if (newState == state)
      return;

    auto oldState = state;
    state = newState;
    cv.notify_all();

    std::shared_ptr<HttpRequest> activeReq;
    activeReq = active_request;
    if (activeReq) {
      LOCAL_DEBUG("HttpClient[%s]::updateState(%s) from %s at step=%d req=%d dt=%f", debugName().c_str(),
        newState.toString(), oldState.toString(),
        updateCounter.load(), activeReq->id(), activeReq->elapsed().toSec());
    } else {
      assert(!state.hasRequest());
      LOCAL_DEBUG("HttpClient[%s]::updateState(%s) from %s at step=%d", debugName().c_str(),
        newState.toString(), oldState.toString(), updateCounter.load());
    }
  }

  void detachPollSet(Lock& lock)
  {
    if (socket && poll_set) {
      poll_set->delSocket(socket->fd());
      poll_set = nullptr;
    }
  }

  void closeSocket(Lock& lock)
  {
    if (socket) {
      socket->close();
      socket.reset();
    }
  }

  /// Pulls new task from queue and updates state.
  /// @returns true if managed to pull new task.
  bool pullNewTask(Lock& processLock);

  /// Completely release own state.
  void release(Lock& lock)
  {
    LOCAL_INFO("HttpClient::Internal[%d]::release()", fd());
    detachPollSet(lock);
    closeSocket(lock);
    updateState(lock, State::Invalid);

    std::shared_ptr<HttpRequest> activeReq;
    std::deque<std::shared_ptr<HttpRequest>> requestsCopy;
    {
      std::unique_lock reqLock(requests_guard);
      if (active_request) {
        activeReq = active_request;
        active_request.reset();
      }
      std::swap(requestsCopy, requests);
    }

    {
      ScopedUnlock unlock(lock);
      if (activeReq) {
        activeReq->updateState(http::HttpRequest::State::Idle);
        activeReq->notifyFailToSend();
        activeReq.reset();
      }
      for (const auto& req: requestsCopy) {
        req->updateState(http::HttpRequest::State::Idle);
        req->notifyFailToSend();
      }
    }
  }
};

int HttpClient::Internal::eventsForState(State s) const
{
  switch (s) {
    case State::Invalid:
      return 0;
    case State::Connecting:
      return PollSet::EventOut;
    case State::WaitReconnect:
      return PollSet::EventTimer;
    case State::Idle:
      return 0;
    case State::WriteRequest:
      return PollSet::EventOut | PollSet::EventError;
    case State::ReadResponse:
      return PollSet::EventIn;
    case State::ProcessResponse:
      return 0;
    case State::Disconnected:
      return 0;
    default:
      return 0;
  }
  return 0;
}

void HttpClient::Internal::resetIntermediateBuffers()
{
  request_body.resize(0);
  request_header_buffer.resize(0);
  response_http_frame.finishResponse();

  data_sent = 0;
}

bool HttpClient::Internal::pullNewTask(Lock& processLock)
{
  if (requests.empty()) {
    active_request = {};
    return false;
  }

  active_request = requests.front();
  requests.pop_front();

  if (!active_request) {
    LOCAL_WARN("HttpClient[%s]::pullNewTask: null request", debugName().c_str());
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

void HttpClient::Internal::handleDisconnect(Lock& lock)
{
  State stateCopy = state;

  poll_set->setEvents(fd(), 0);
  updateState(lock, State::Disconnected);
  // Push active request back to queue.
  if (active_request) {
    std::unique_lock reqLock(requests_guard);
    {
      auto copy = active_request;
      active_request.reset();
      if (copy->shouldRetry()) {
        requests.push_front(copy);
        copy->updateState(HttpRequest::State::ClientQueued);
      } else {
        copy->updateState(HttpRequest::State::Idle);
        copy->notifyFailToSend();
      }
    }

    auto requestsCopy = requests;
    reqLock.unlock();

    for (auto req: requestsCopy) {
      req->notifyFailToSend();
    }
  }

  // No callback set or no reconnect required. Just close the current socket astd::strchr(nd reset state.
  LOCAL_INFO("HttpClient[%s]::handleDisconnect() - connection is closed from state %s", debugName().c_str(), stateCopy.toString());

  if (onDisconnect) {
    // Copying all mutable data, which can be changed during callback.
    auto callback = onDisconnect;
    auto socketCopy = socket;
    ScopedUnlock unlock(lock);
    // User can issue reconnect right here.
    callback(socketCopy, stateCopy);
  }
}

void HttpClient::setDisconnectHandler(DisconnectHandler&& handler)
{
  if (!internal_)
    return;
  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->onDisconnect = std::move(handler);
}

void HttpClient::setConnectHandler(ConnectHandler&& onConnect)
{
  if (!internal_)
    return;
  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->onConnect = std::move(onConnect);
}

void HttpClient::setResponseHandler(ResponseHandler&& onResponse)
{
  if (!internal_)
    return;
  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->onResponse = std::move(onResponse);
}


void HttpClient::setIdleTimeoutHandler(int timeoutMs, TimeoutHandler&& handler)
{
  if (!internal_)
    return;
  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->onIdleTimeout = std::move(handler);
  internal_->idleTimeoutMs = timeoutMs;
  
  // If we're currently in Idle state and have a valid socket, set the timer
  if (internal_->state == State::Idle && internal_->socket && internal_->poll_set && timeoutMs > 0) {
    internal_->poll_set->setTimerEvent(internal_->socket->fd(), timeoutMs);
    LOCAL_INFO("HttpClient[%s]::setIdleTimeoutHandler: Set idle timeout timer to %d ms", internal_->debugName().c_str(), timeoutMs);
  }
}

Error HttpClient::reconnect(int timeoutMs)
{
  if (!internal_ || !internal_->poll_set)
    return Error::InternalError;

  Lock lock(internal_->process_guard, THIS_LOCATION);

  if (!internal_->socket || !internal_->socket->valid())
    return Error::InvalidHandle;

  if (timeoutMs > 0) {
    internal_->poll_set->setTimerEvent(internal_->socket->fd(), timeoutMs);
    internal_->updateState(lock, State::WaitReconnect);
    return Error::Ok;
  }
  internal_->reconnectAttempts++;
  return internal_->connectImpl(internal_, lock, internal_->address);
}

Error HttpClient::Internal::readResponse()
{
  auto [transferred, readErr] = socket->recv(response_http_frame.data, nullptr);

  if (transferred > 0) {
    const int parsed = response_http_frame.incrementalParse();

    const int fd = socket->fd();
    double dur = (SteadyTime::now() - active_request->getRequestStart()).toSec() * 1000;
    LOCAL_DEBUG("HttpClient(%d)::readHeader: ContentLength=%d, parsed=%d t=%fms", fd, response_http_frame.contentLength(), parsed, dur);

    // If we haven't gotten the entire request yet, return (keep reading)
    if (response_http_frame.state() != HttpParserFrame::ParseComplete) {
      if (readErr == Error::EndOfFile) {
        LOCAL_DEBUG("HttpClient[%s]::readRequest: EOF while reading request", debugName().c_str());
        response_http_frame.finishResponse();
        return Error::EndOfFile;
        // Either way we close the connection
      }
      LOCAL_DEBUG("HttpClient[%s]::readRequest got only %d/%d bytes.", debugName().c_str(), response_http_frame.bodyLength(), response_http_frame.contentLength());
      return Error::WouldBlock;
    }

    assert(response_http_frame.state() == HttpParserFrame::ParseComplete);
    // Otherwise, parse and dispatch the request
    LOCAL_DEBUG("HttpClient[%s]::readRequest read %d/%d bytes.", debugName().c_str(), response_http_frame.bodyLength(), response_http_frame.contentLength());
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
  int fd = 0;
  int refs = 0;
  if (internal_) {
    Lock lock(internal_->process_guard, THIS_LOCATION);
    fd = internal_->fd();
    std::shared_ptr<Internal> copy;
    std::swap(internal_, copy);
    copy->release(lock);
    refs = copy.use_count();
    // Explicitly unlock to prevent potential deadlock in destructor if Internal.
    lock.unlock();
  }
  LOCAL_INFO("HttpClient::~HttpClient(%d) refs=%d", fd, refs);
}

HttpClient::State HttpClient::getState() const
{
  if (!internal_)
    return State::Invalid;
  Lock lock(internal_->process_guard, THIS_LOCATION);
  return internal_->state;
}

bool HttpClient::isActive() const
{
  State s = getState();
  if (s == State::Invalid || s == State::Idle)
    return false;
  return true;
}

int HttpClient::fd() const
{
  return internal_ ? internal_->fd() : -1;
}

network::URL HttpClient::getRootURL(const std::string& scheme) const
{
  network::URL url;
  if (!internal_)
    return url;

  Lock lock(internal_->process_guard, THIS_LOCATION);
  if (!internal_->address.valid())
    return url;

  url.scheme = scheme;
  url.host = internal_->address.address;
  url.port = static_cast<uint32_t>(internal_->address.port());
  // path and query remain empty (default constructed)
  return url;
}

void HttpClient::release()
{
  if (!internal_)
    return;

  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->release(lock);
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

  assert(internal_);
  if (!internal_) {
    return Error::InternalError;
  }

  bool immediate = false;
  {
    std::unique_lock lock(internal_->requests_guard);
    // Set request status to Queued
    request->updateState(HttpRequest::State::ClientQueued);
    internal_->requests.push_back(request);
  }

  Lock lock2(internal_->process_guard, THIS_LOCATION);
  // If we're in Idle state and have a valid socket, trigger processing
  if (internal_->state == State::Idle && internal_->socket && internal_->socket->valid()) {
    // Trigger pullNewTask by updating socket events
    // The socket is already in PollSet, so we can trigger it by updating events
    // For now, we'll let the next event cycle handle it, or we could use a callback
    // The simplest approach is to let the existing event loop handle it
    if (internal_->pullNewTask(lock2)) {
      internal_->updateState(lock2, State::WriteRequest);
      if (!internal_->poll_set->setEvents(internal_->socket->fd(), internal_->eventsForState(internal_->state))) {
        LOCAL_ERROR("HttpClient(%d) Failed to update poll set to enable events to send request", internal_->fd());
      }
      immediate = true;
    }
  }

  if (immediate) {
    LOCAL_INFO("HttpClient[%s]::enqueueRequest %s and started processing", internal_->debugName().c_str(), request->debugName().c_str());
  } else {
    LOCAL_INFO("HttpClient[%s]::enqueueRequest %s to queue", internal_->debugName().c_str(), request->debugName().c_str());
  }

  return Error::Ok;
}

Error HttpClient::dropRequest(const std::shared_ptr<HttpRequest>& request)
{
  assert(request);
  if (!request) {
    return Error::InvalidValue;
  }

  assert(internal_);
  if (!internal_) {
    return Error::InternalError;
  }

  LOCAL_INFO("HttpClient[%s]::dropRequest %s", internal_->debugName().c_str(), request->debugName().c_str());
  Lock processLock(internal_->process_guard, THIS_LOCATION);
  std::unique_lock lock(internal_->requests_guard);

  if (request == internal_->active_request) {
    auto s = request->state();
    // Possible states: ClientSending, ClientWaitResponse, ClientHasResponse, Done.
    if (s == HttpRequest::State::ClientSending) {
      if (internal_->data_sent != 0)
        return Error::ResourceInUse;

      internal_->active_request.reset();
      internal_->pullNewTask(processLock);
      request->updateState(HttpRequest::State::Idle);
      return Error::Ok;
    }

    if (s == HttpRequest::State::ClientWaitResponse || s == HttpRequest::State::ClientHasResponse) {
      return Error::ResourceInUse;
    }
    internal_->active_request.reset();
    internal_->pullNewTask(processLock);
    request->updateState(HttpRequest::State::Idle);
    return Error::Ok;
  } else {
    for (auto it = internal_->requests.begin(); it != internal_->requests.end(); it++) {
      if (*it == request) {
        internal_->requests.erase(it);
        request->updateState(HttpRequest::State::Idle);
        return Error::Ok;
      }
    }
  }

  return Error::InvalidValue;
}

Error HttpClient::connect(const std::string& host, int port)
{
  network::NetAddress address = network::NetAddress::fromString(network::NetAddress::AddressIPv4, host, port);
  if (!address.valid())
    return Error::InvalidAddress;

  Lock lock(internal_->process_guard, THIS_LOCATION);
  return internal_->connectImpl(internal_, lock, address);
}

int HttpClient::getReconnectAttempts() const
{
  if (!internal_)
    return 0;
  std::unique_lock lock(internal_->process_guard);
  return internal_->reconnectAttempts;
}

Error HttpClient::Internal::connectImpl(const std::shared_ptr<Internal>& I, Lock& lock, const network::NetAddress& newAddress)
{
  assert(poll_set);
  if (!poll_set) {
    return Error::InternalError;
  }

  if (socket && socket->valid()) {
    // TODO: Could reuse existing socket.
    closeSocket(lock);
  }

  socket.reset(new NetSocket());

  Error err = socket->tcpConnect(newAddress, true);

  // Store host and port for:
  // 1. Reconnect.
  // 2. Help building requests.
  address = newAddress;

  bool connected = false;
  if (err == Error::WouldBlock) {
    // This is fine.
  } else if (err == Error::Ok) {
    connected = true;
  } else {
    // Failed to initialize connection.
    LOCAL_WARN("HttpClient::connect failed to initialize connection to %s:%d", address.address.c_str(), address.port());
    return err;
  }

  socket->setKeepAlive(true);
  poll_set->addSocket(socket->fd(), PollSet::EventIn | PollSet::EventOut | PollSet::EventUpdate | PollSet::EventError,
    [wptr = std::weak_ptr(I)](int event)
    {
      auto ptr = wptr.lock();
      if (ptr)
        return handleSocketEvents(ptr, event);
      return 0;
    }, I, THIS_LOCATION);

  updateState(lock, connected ? State::Idle : State::Connecting);
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

int HttpClient::handleSocketEvents(const std::shared_ptr<Internal>& I, int evtFlags)
{
  if (!I)
    return 0;

  MINIROS_PROFILE_SCOPE2("HttpClient", "handleSocketEvents");

  Lock lock(I->process_guard, THIS_LOCATION);
  /// Keep a copy of itself.
  ++I->updateCounter;
  State& state = I->state;
  State initialState = state;

  // The following states are expected to fall through.
  // Each "handle" method can update internal state.
  // The order of processing of these states corresponds to expected flow of FSM.
  int stateChanges = 0;
  bool loop = true;
  do {
    if (state == State::Disconnected) {
      // Just ignore all events in disconnected state.
      // It is probably combination of error | SIGHUP events.
      evtFlags = 0;
      break;
    }
    if (state == State::WaitReconnect) {
      // Expecting to be here on a timeout.
      I->handleReconnecting(I, lock, evtFlags);
      if (state == State::Invalid)
        break;
      evtFlags = 0;
      stateChanges++;
    }

    if (state == State::Connecting) {
      // Expected to transition to Idle state.
      I->handleConnecting(lock, evtFlags);
      evtFlags = 0;
      stateChanges++;
    }

    if (state == State::Idle) {
      if (evtFlags & PollSet::EventError) {
        // Can we actually be here?
        I->handleDisconnect(lock);
      } else if (evtFlags & PollSet::EventIn) {
        // Not expecting to read anything. But we can get this event to call 'read' and receive 0 bytes.
        // This is disconnect event.
      } else if (evtFlags & PollSet::EventTimer) {
        // Got timeout waiting - call idle timeout handler
        if (I->onIdleTimeout && I->socket) {
          auto callback = I->onIdleTimeout;
          auto socket = I->socket;
          State stateCopy = state;
          ScopedUnlock unlock(lock);
          callback(socket, stateCopy);
        }
      }
      else {
        std::unique_lock lock2(I->requests_guard);
        if (I->pullNewTask(lock)) {
          I->updateState(lock, State::WriteRequest);
        }
      }
      evtFlags = 0;
      stateChanges++;
    }

    if (I->state == State::WriteRequest) {
      I->handleWriteRequest(lock, evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (I->state == State::ReadResponse) {
      I->handleReadResponse(lock, evtFlags, stateChanges > 0);
      evtFlags = 0;
      stateChanges++;
    }

    if (I->state == State::ProcessResponse) {
      I->handleProcessResponse(lock);
      stateChanges++;
      // Restart loop to attempt to start sending new request.
      if (I->state == State::Idle)
        continue;
    }
    loop = false;
  }while (loop);

  if (I->state == State::Idle && (evtFlags & PollSet::EventOut)) {
    // Nothing to do here.
  } else if (stateChanges == 0 && state != State::Invalid) {
    std::string evt = PollSet::eventToString(evtFlags);
    LOCAL_WARN("HttpClient unhandled events: %s in state %s", evt.c_str(), state.toString());
  }

  if (state == State::Idle && initialState != State::Idle && I->idleTimeoutMs > 0) {
    I->poll_set->setTimerEvent(I->socket->fd(), I->idleTimeoutMs);
  }

  return I->eventsForState(I->state);
}

void HttpClient::Internal::handleReconnecting(const std::shared_ptr<Internal>& I, Lock& lock, int events)
{
  if (events & PollSet::EventTimer) {
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - connected", debugName().c_str(), PollSet::eventToString(events).c_str());
    connectImpl(I, lock, address);
  } else if (events & PollSet::EventError) {
    //LOCAL_ERROR_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - failed to connect", debugName().c_str(), PollSet::eventToString(events).c_str());
  } else {
    LOCAL_ERROR("HttpClient[%s]::handleReconnecting(%s) - unhandled events", debugName().c_str(), PollSet::eventToString(events).c_str());
  }
}

void HttpClient::Internal::handleConnecting(Lock& lock, int evtFlags)
{
  if (state != State::Connecting)
    return;
  MINIROS_PROFILE_SCOPE2("HttpClient", "handleConnecting");

  // Assume socket is in "connecting" state
  assert(socket->isConnecting());
  if (evtFlags & PollSet::EventError) {
    LOCAL_ERROR("HttpClient[%s]::handleEvents(state=Connecting, evt=%s) - failed to connect", debugName().c_str(), PollSet::eventToString(evtFlags).c_str());
    handleDisconnect(lock);
    return;
  }

  if (evtFlags & PollSet::EventOut) {
    Error err = socket->checkConnected();
    if (err == Error::Timeout) // Still connecting.
      return;

    if (err == Error::Ok) {
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleEvents(state=Connecting) - connected", debugName().c_str());
      // Connection is complete.
      // Attempt to pull requests will be done in "handleSocketEvents" method.
      reconnectAttempts = 0;
      updateState(lock, State::Idle);
    } else {
      LOCAL_ERROR("HttpClient[%s]::handleEvents(state=Connecting) - error: %s", debugName().c_str(), err.toString());
      handleDisconnect(lock);
    }
  }
}

void HttpClient::Internal::handleWriteRequest(Lock& lock, int evtFlags, bool fallThrough)
{
  MINIROS_PROFILE_SCOPE2("HttpClient", "handleWriteRequest");
  if (evtFlags & PollSet::EventError) {
    int err = socket->getSysError();
    // Here we can have some disconnect problem or some generic network problem.
    LOCAL_WARN("HttpClient[%s]::handleWriteRequest socket error in WriteRequest: %i", debugName().c_str(), err);
    handleDisconnect(lock);
    return;
  }

  if (!socket) {

  }

  if (fallThrough || (evtFlags & PollSet::EventOut)) {
    std::pair<size_t, Error> r;
    if (request_body.size() > 0) {
      // Send header+body packet.
      r = socket->write2(
        request_header_buffer.c_str(), request_header_buffer.size(),
        request_body.c_str(), request_body.size(), data_sent);
    } else {
      // Send single header packet.
      const char* data = request_header_buffer.c_str() + data_sent;
      size_t toSend = request_header_buffer.size() - data_sent;
      r = socket->send(data, toSend, nullptr);
    }

    const size_t totalSize = request_header_buffer.size() + request_body.size();
    auto [written, err] = r;

    if (written > 0) {
      data_sent += written;
      double dur = (SteadyTime::now() - active_request->getRequestStart()).toSec() * 1000;
      MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleWriteRequest() HttpClient written %d/%d bytes of header, t=%fms",
        debugName().c_str(), static_cast<int>(written), static_cast<int>(totalSize), dur);
    }

    if (err == Error::WouldBlock) {
      // Failed to write all data. Retry later.
      return;
    }

    if (err != Error::Ok) {
      LOCAL_ERROR("HttpClient[%s]::handleWriteRequest error: %s", debugName().c_str(), err.toString());
      handleDisconnect(lock);
      return;
    }

    if (data_sent < totalSize) {
      // Failed to write all data, so we need to yield to spinner.
      return;
    }

    data_sent = 0;
    // Request sent, now wait for response
    active_request->updateState(HttpRequest::State::ClientWaitResponse);
    updateState(lock, State::ReadResponse);
    response_http_frame.resetParseState(false); // false = response mode
    auto dur = SteadyTime::now() - active_request->getRequestStart();
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleWriteRequest sent request in %fms, waiting for response",
      debugName().c_str(), dur.toSec()*1000);
    return;
  }

  // Some unhandled event.
  std::string evt = PollSet::eventToString(evtFlags);
  LOCAL_WARN("HttpClient[%s]::handleWriteRequest unhandled event in WriteRequest: %s", debugName().c_str(), evt.c_str());
}

void HttpClient::Internal::handleReadResponse(Lock& lock, int evtFlags, bool fallThrough)
{
  MINIROS_PROFILE_SCOPE2("HttpClient", "handleReadResponse");

  if (evtFlags & PollSet::EventError) {
    LOCAL_WARN("HttpClient[%s]::handleReadResponse() got socket error", debugName().c_str());
    handleDisconnect(lock);
    return;
  }

  if (fallThrough || evtFlags & PollSet::EventIn) {
    Error err = readResponse();

    if (err == Error::Ok) {
      updateState(lock, State::ProcessResponse);
    } else if (err == Error::WouldBlock) {
      // Got partial data. Need to wait.
      return;
    }
    else if (err == Error::EndOfFile) {
      LOCAL_INFO("HttpClient[%s] - Got end of file from remote server", debugName().c_str());
      // Connection closed by peer
      handleDisconnect(lock);
    } else {
      LOCAL_ERROR("HttpClient[%s] - Unexpected error %s", debugName().c_str(), err.toString());
      handleDisconnect(lock);
    }
  } else {
    std::string evt = PollSet::eventToString(evtFlags);
    LOCAL_WARN("HttpClient[%s]::handleReadResponse() unhandled events in ReadResponse: %s at %d",
      debugName().c_str(), evt.c_str(), updateCounter.load());
  }
}

void HttpClient::Internal::handleProcessResponse(Lock& lock)
{
  MINIROS_PROFILE_SCOPE2("HttpClient", "handleProcessResponse");
  // Response received and parsed - store it in HttpRequest
  if (active_request) {
    auto req = active_request;
    req->setResponseHeader(response_http_frame);

    // Store response body.
    std::string_view body = response_http_frame.body();
    if (!body.empty()) {
      req->setResponseBody(body.data(), body.size());
    }

    // Status should be updated last to prevent possible racing with access to response body.
    req->updateState(HttpRequest::State::ClientHasResponse);

    resetIntermediateBuffers();

    auto dur = req->getRequestFinish() - req->getRequestStart();
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleProcessResponse() received response in %fms",
      debugName().c_str(), dur.toSec()*1000);

    if (onResponse) {
      auto copy = onResponse;
      ScopedUnlock unlock(lock);
      copy(req);
    }
    req->processResponse();
    req->updateState(HttpRequest::State::Done);
  }

  updateState(lock, State::Idle);
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