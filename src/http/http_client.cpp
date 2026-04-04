//
// Created by dkargin on 11/14/25.
//

#include <deque>
#include <atomic>
#include <cassert>
#include <chrono>
#include <memory>

/// This log channel maps to miniros.http
#define MINIROS_PACKAGE_NAME "http"

#include "miniros/console.h"
#include "miniros/network/socket.h"
#include "miniros/http/http_client.h"

#include "internal/profiling.h"
#include "internal/scoped_locks.h"
#include "io/poll_set.h"
#include "network/net_adapter.h"

namespace {
std::atomic_int32_t g_lastClientId{0};
}

namespace miniros {
namespace http {

using Lock = TimeCheckLock<std::mutex>;

const char* HttpClient::State::toString() const
{
  switch (value) {
    case Invalid:
      return "Invalid";
    case WaitingAddress:
      return "WaitingAddress";
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

  /// Some internal ID of HTTP client.
  /// It is used for writing more distinguishable debugName() in logs.
  int id = 0;
  /// A queue of requests.
  std::deque<std::shared_ptr<HttpRequest>> requests;

  /// Mutex for storing requests
  mutable std::mutex requests_guard;

  std::shared_ptr<HttpRequest> active_request;

  /// Remote address.
  /// It is set during `connect` call and reused in reconnects.
  network::NetAddress address;
  /// Host and port as specified by `HttpClient::connect`.
  /// Unlike `address`, this may remain unresolved hostname.
  std::string remoteHost;
  int remotePort = 0;

  /// Internal state.
  /// State was stored as a struct originally, but storing it as an atomic proved to be problematic.
  std::atomic<State::State_t> state{State::Invalid};

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
  bool poll_registered = false;

  /// Mutex for processing socket state.
  std::mutex process_guard;

  /// Counter for update cycles. It is incremented every time handleSocketEvents is called.
  /// It helps debugging multiple FSM transitions.
  std::atomic_int updateCounter = 0;

  /// True if owning HttpClient is still alive.
  /// It is set to false in ~HttpClient()
  std::atomic_bool alive{true};

  /// Condition variable for state change.
  std::condition_variable_any cv;

  Internal(PollSet* ps) : poll_set(ps)
  {
    id = ++g_lastClientId;
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
  int eventsForState(State::State_t state) const;

  /// Does actual connection.
  /// @param I - reference to self/Internal. Used to keep instance alive.
  /// @param lock - reference to locked mutex. Used to enforce locking before entrance to this method.
  /// @param newAddress - address for connection.
  Error connectImpl(const std::shared_ptr<Internal>& I, Lock& lock, const network::NetAddress& newAddress);

  void handleDisconnect(Lock& lock, Error disconnectError);

  /// Handle State::Connecting.
  /// Can switch to Idle.
  /// It is expected that idle state is handled right after 'handleConnecting'
  void handleConnecting(Lock& lock, int evtFlags);

  /// Handle State::WaitAddress.
  /// Can switch to:
  ///    - Connecting
  ///    - Invalid
  void handleWaitAddress(const std::shared_ptr<Internal>& I, Lock& lock, int events);

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

  /// Get name for debug output.
  std::string debugName() const
  {
    return std::to_string(id) + "@" + address.lstr() + std::string(" fd=") + std::to_string(fd());
  }

  void updateState(Lock& lock, State newState);

  /// Pulls new task from queue and updates state.
  /// @returns true if managed to pull new task.
  bool pullNewTask(Lock& processLock);

  /// Completely release own state.
  void release(Lock& lock);

  ///  Attach socket to PollSet.
  bool initSocketEvents(const std::shared_ptr<Internal>& I, Lock& /*lock*/);

  size_t getQueuedRequests() const
  {
    std::unique_lock lock(requests_guard);
    size_t num = requests.size();
    if (active_request)
      num++;
    return num;
  }

  void detachFromPollSet()
  {
    if (socket && poll_set && poll_registered) {
      [[maybe_unused]] bool ret = poll_set->delSocket(socket->fd());
      assert(ret);
      poll_registered = false;
    }
  }
};

void HttpClient::Internal::release(Lock& lock)
{
  LOCAL_DEBUG("HttpClient::Internal[%d]::release()", fd());

  detachFromPollSet();

  onConnect = {};
  onDisconnect = {};
  onIdleTimeout = {};
  onResponse = {};

  updateState(lock, State::Invalid);
  socket.reset();

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

void HttpClient::Internal::updateState(Lock& lock, State newState)
{
  State oldState{State::Invalid};
  {
    if (newState == state)
      return;

    oldState = state.load();
    state = newState;
    cv.notify_all();
  }

  std::shared_ptr<HttpRequest> activeReq;
  activeReq = active_request;
  if (activeReq) {
    LOCAL_DEBUG("HttpClient[%s]::updateState(%s) from %s at step=%d req=%d dt=%f", debugName().c_str(),
      newState.toString(), oldState.toString(),
      updateCounter.load(), activeReq->id(), activeReq->elapsed().toSec());
  } else {
    assert(!State(state).hasRequest());
    LOCAL_DEBUG("HttpClient[%s]::updateState(%s) from %s at step=%d", debugName().c_str(),
      newState.toString(), oldState.toString(), updateCounter.load());
  }
}

int HttpClient::Internal::eventsForState(State::State_t s) const
{
  switch (s) {
    case State::Invalid:
      return 0;
    case State::Connecting:
      return PollSet::EventOut;
    case State::WaitReconnect:
      return 0;
    case State::WaitingAddress:
      return 0;
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
  LOCAL_ERROR("HttpClient::eventsForState(%s) - unhandled state", State(s).toString());
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

  // Build request header from HttpRequest (host/port managed by HttpClient).
  // Use configured host/port from connect() to preserve original host name.
  const std::string& requestHost = remoteHost.empty() ? address.address : remoteHost;
  const int requestPort = remotePort > 0 ? remotePort : address.port();
  request_header_buffer = active_request->buildHeader(requestHost, requestPort);
  request_body = active_request->requestBody();
  data_sent = 0;

  return true;
}

void HttpClient::Internal::handleDisconnect(Lock& lock, Error disconnectError)
{
  State stateCopy = state.load();

  if (socket && socket->valid()) {
    poll_set->setEvents(fd(), 0);
  }

  // TODO: change state only if callback has not changed state anyhow.
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

  if (onDisconnect && alive) {
    LOCAL_INFO("HttpClient[%s]::handleDisconnect() - connection is closed from state %s, err=%s, invoking callback", debugName().c_str(), stateCopy.toString(), disconnectError.toString());
    // Copying all mutable data, which can be changed during callback.
    auto callback = onDisconnect;
    auto socketCopy = socket;
    ScopedUnlock unlock(lock);
    // Danger: if callback references HttpClient object then there is small chance
    // this instance is destroyed right after lock is released.
    callback(socketCopy, stateCopy, disconnectError);
  } else {
    LOCAL_INFO("HttpClient[%s]::handleDisconnect() - connection is closed from state %s, err=%s, no callback", debugName().c_str(), stateCopy.toString(), disconnectError.toString());
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
  if (internal_->state.load() == State::Idle && internal_->socket && internal_->poll_set && timeoutMs > 0) {
    internal_->poll_set->setTimerEvent(internal_->socket->fd(), timeoutMs);
    LOCAL_INFO("HttpClient[%s]::setIdleTimeoutHandler: Set idle timeout timer to %d ms", internal_->debugName().c_str(), timeoutMs);
  }
}

Error HttpClient::reconnect(int timeoutMs)
{
  if (!internal_ || !internal_->poll_set)
    return Error::InternalError;

  Lock lock(internal_->process_guard, THIS_LOCATION);
  //if (internal_->remoteHost.empty() || internal_->remotePort <= 0)
  //  return Error::InvalidAddress;

  network::NetAddress newAddress;
  Error resolveErr = network::addressFromString(network::NetAddress::AddressUnspecified,
    internal_->remoteHost, internal_->remotePort, newAddress);

  bool needAddress = false;
  if (!resolveErr) {
    if (resolveErr == Error::AddressIsUnknown) {
      // Can continue reconnect
      needAddress = true;
    } else {
      MINIROS_ERROR("Failed to make address for reconnection: %s", resolveErr.toString());
      return resolveErr;
    }
  }

  internal_->reconnectAttempts++;

  if (timeoutMs <= 0) {
    // Immediate connection.
    return internal_->connectImpl(internal_, lock, newAddress);
  }

  // Schedule reconnect by timer event from PollSet.
  assert(internal_->socket && internal_->socket->valid());
  if (!internal_->socket || !internal_->socket->valid()) {
    return Error::InvalidHandle;
  }
  internal_->updateState(lock, needAddress ? State::WaitingAddress : State::WaitReconnect);
  auto ret = internal_->poll_set->setTimerEvent(internal_->socket->fd(), timeoutMs);
  assert(ret);
  if (!ret) {
    LOCAL_WARN("HttpClient[%s]::reconnect() - failed to set timer event", internal_->debugName().c_str());
  }
  return Error::Ok;
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
        LOCAL_DEBUG("HttpClient[%s]::readResponse: EOF while reading request", debugName().c_str());
        response_http_frame.finishResponse();
        return Error::EndOfFile;
        // Either way we close the connection
      }
      LOCAL_DEBUG("HttpClient[%s]::readResponse got only %d/%d bytes.", debugName().c_str(), response_http_frame.bodyLength(), response_http_frame.contentLength());
      return Error::WouldBlock;
    }

    assert(response_http_frame.state() == HttpParserFrame::ParseComplete);
    // Otherwise, parse and dispatch the request
    LOCAL_DEBUG("HttpClient[%s]::readResponse read %d/%d bytes.", debugName().c_str(), response_http_frame.bodyLength(), response_http_frame.contentLength());
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
    copy->alive = false;
    copy->release(lock);
    refs = copy.use_count();
    // Explicitly unlock to prevent potential deadlock in destructor if Internal.
    lock.unlock();
  }
  LOCAL_DEBUG("HttpClient::~HttpClient(%d) refs=%d", fd, refs);
}

HttpClient::State HttpClient::getState() const
{
  if (!internal_)
    return State::Invalid;
  return internal_->state.load();
}

bool HttpClient::isActive() const
{
  State s = internal_->state.load();
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
  if (internal_->state.load() == State::Idle && internal_->socket && internal_->socket->valid()) {
    if (internal_->pullNewTask(lock2)) {
      // There is some probability that the task we pulled is not the same task we have enqueued.
      // It can happen if several threads are enqueuing requests.
      LOCAL_INFO("HttpClient[%s]::enqueueRequest %s and started processing", internal_->debugName().c_str(), request->debugName().c_str());
      internal_->updateState(lock2, State::WriteRequest);
      // Process sending immediately.
      internal_->handleWriteRequest(lock2, 0, true);

      if (!internal_->poll_set->setEvents(internal_->socket->fd(), internal_->eventsForState(internal_->state))) {
        LOCAL_ERROR("HttpClient(%s)::enqueueRequest failed to update poll set to enable events to send request", internal_->debugName().c_str());
      }
      immediate = true;
    } else {
      LOCAL_ERROR("HttpClient(%s)::enqueueRequest failed to update poll set to enable events to send request", internal_->debugName().c_str());
    }
  }

  if (!immediate) {
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
  network::NetAddress address;
  Error resolveErr = network::addressFromString(network::NetAddress::AddressUnspecified, host, port, address);

  if (!resolveErr && resolveErr != Error::AddressIsUnknown) {
    // Some unrecoverable error, which prevents any further connection.
    return resolveErr;
  }

  Lock lock(internal_->process_guard, THIS_LOCATION);
  internal_->remoteHost = host;
  internal_->remotePort = port;

  if (resolveErr == Error::AddressIsUnknown) {
    if (!internal_->socket) {
      // Just create some dummy socket, so it is eligible for adding into PollSet with no events.
      internal_->socket.reset(new NetSocket());
      internal_->socket->tcpSocket(network::NetAddress::Type::AddressIPv4);
      internal_->initSocketEvents(internal_, lock);
    }
    internal_->handleDisconnect(lock, Error::AddressIsUnknown);
    return Error::Ok;
  }

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
  if (!poll_set) {
    return Error::InternalError;
  }

  if (socket) {
    // Need to close socket since its state after failed 'connect' is undefined.
    detachFromPollSet();
    socket->close();
  } else {
    socket.reset(new NetSocket());
  }

  Error err = socket->tcpConnect(newAddress, true);

  LOCAL_WARN("HttpClient[%s]::connectImpl initiating connection to %s:%d", debugName().c_str(), address.address.c_str(), address.port());

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
    LOCAL_WARN("HttpClient[%s]::connect failed to initialize connection to %s:%d", debugName().c_str(), address.address.c_str(), address.port());
    return err;
  }
  socket->setKeepAlive(true);
  updateState(lock, connected ? State::Idle : State::Connecting);

  bool ret = initSocketEvents(I, lock);
  assert(ret);

  return Error::Ok;
}

bool HttpClient::Internal::initSocketEvents(const std::shared_ptr<Internal>& I, Lock& /*lock*/)
{
  const int events = eventsForState(state);
  poll_registered = poll_set->addSocket(socket->fd(), events,
    [wptr = std::weak_ptr(I)](int event)
      {
        if (auto ptr = wptr.lock()) {
          handleSocketEvents(ptr, event);
          return 0;
        }
        LOCAL_WARN("HttpClient[?]::handleSocketEvents() - call for destroyed client instance");
        return 0;
      }, {}, THIS_LOCATION);
  return poll_registered;
}


Error HttpClient::waitConnected(const WallDuration& duration)
{
  if (!internal_)
    return Error::InternalError;

  if (!internal_->socket || !internal_->socket->valid())
    return Error::InvalidHandle;

  Lock lock(internal_->process_guard, THIS_LOCATION, 0.02);
  if (internal_->cv.wait_for(lock, std::chrono::duration<double>(duration.toSec()),
    [this]() {
      switch (internal_->state) {
        case State::ProcessResponse:
        case State::Idle:
        case State::ReadResponse:
        case State::WriteRequest:
          return true;
        default:
          return false;
      }
    })) {
    if (internal_->state == State::Idle)
      return Error::Ok;

    return Error::NotConnected;
  }
  return Error::Timeout;
}

void HttpClient::handleSocketEvents(const std::shared_ptr<Internal>& I, int evtFlags)
{
  if (!I) {
    LOCAL_ERROR("HttpClient::handleSocketEvents() - invalid pointer to internal");
  }

  MINIROS_PROFILE_SCOPE2("HttpClient", "handleSocketEvents");

  Lock lock(I->process_guard, THIS_LOCATION, 0.02);

  LOCAL_DEBUG("HttpClient[%s]::handleSocketEvents(%s) in state %s", I->debugName().c_str(),
    PollSet::eventToString(evtFlags).c_str(), State(I->state).toString());

  /// Keep a copy of itself.
  ++I->updateCounter;

  auto& state = I->state;
  State initialState = state.load();

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
    if (state == State::WaitingAddress) {
      // Expecting to be here on a timeout.
      I->handleWaitAddress(I, lock, evtFlags);
      if (state == State::Invalid)
        break;
      evtFlags = 0;
      stateChanges++;
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
        I->handleDisconnect(lock, Error::SystemError);
      } else if (evtFlags & PollSet::EventIn) {
        // Not expecting to read anything. But we can get this event to call 'read' and receive 0 bytes.
        // This is disconnect event.
      } else if (evtFlags & PollSet::EventTimer) {
        // Got timeout waiting - call idle timeout handler
        if (I->onIdleTimeout && I->socket) {
          auto callback = I->onIdleTimeout;
          auto socket = I->socket;
          State stateCopy = state.load();
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
    LOCAL_WARN("HttpClient unhandled events: %s in state %s", evt.c_str(), State(state).toString());
  }

  // Update event flags.
  // Socket can be destroyed in disconnect handler.
  if (I->socket && I->socket->valid()) {
    int fd = I->socket->fd();
    if (state == State::Idle && initialState != State::Idle && I->idleTimeoutMs > 0) {
      I->poll_set->setTimerEvent(fd, I->idleTimeoutMs);
    }

    int newEvt = I->eventsForState(state);
    LOCAL_DEBUG("HttpClient::handleSocketEvents() exit in state %s new evt=%s", State(state).toString(),
      PollSet::eventToString(newEvt).c_str());
    I->poll_set->setEvents(fd, newEvt);
  }
}

void HttpClient::Internal::handleWaitAddress(const std::shared_ptr<Internal>& I, Lock& lock, int events)
{
  if (events & PollSet::EventTimer) {
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleWaitAddress(%s) - retry address", debugName().c_str(), PollSet::eventToString(events).c_str());

    Error resolveErr = network::addressFromString(network::NetAddress::AddressUnspecified, remoteHost, remotePort, address);
    if (!resolveErr) {
      handleDisconnect(lock, resolveErr);
    } else {
      // Now we have proper address.
      connectImpl(I, lock, address);
    }
  } else if (events & PollSet::EventError) {
  } else {
    LOCAL_ERROR("HttpClient[%s]::handleWaitAddress(%s) - unhandled events", debugName().c_str(), PollSet::eventToString(events).c_str());
  }
}

void HttpClient::Internal::handleReconnecting(const std::shared_ptr<Internal>& I, Lock& lock, int events)
{
  if (events & PollSet::EventTimer) {
    MINIROS_DEBUG_NAMED("client", "HttpClient[%s]::handleReconnecting(%s) - retry connect", debugName().c_str(), PollSet::eventToString(events).c_str());
    connectImpl(I, lock, address);
  } else if (events & PollSet::EventError) {
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
    LOCAL_ERROR("HttpClient[%s]::handleEvents(state=Connecting, evt=%s) - failed to connect, q=%zu", debugName().c_str(), PollSet::eventToString(evtFlags).c_str(), getQueuedRequests());
    handleDisconnect(lock, Error::ConnectionRefused);
    return;
  }

  if (evtFlags & PollSet::EventOut) {
    Error err = socket->checkConnected();
    if (err == Error::Timeout) // Still connecting.
      return;

    if (err == Error::Ok) {
      MINIROS_DEBUG("HttpClient[%s]::handleEvents(state=Connecting) - connected", debugName().c_str());
      // Connection is complete.
      // Attempt to pull requests will be done in "handleSocketEvents" method.
      reconnectAttempts = 0;
      updateState(lock, State::Idle);

      if (onConnect && alive) {
        auto callback = onConnect;
        auto socketCopy = socket;
        ScopedUnlock unlock(lock);
        // Danger: if callback references HttpClient object then there is small chance
        // this instance is destroyed right after lock is released.
        (void)callback(socketCopy);
      }
    } else {
      LOCAL_ERROR("HttpClient[%s]::handleEvents(state=Connecting) - checkConnected() error: %s", debugName().c_str(), err.toString());
      handleDisconnect(lock, err);
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
    handleDisconnect(lock, Error::SystemError);
    return;
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
      MINIROS_DEBUG("HttpClient[%s]::handleWriteRequest() HttpClient written %d/%d bytes of header, t=%fms",
        debugName().c_str(), static_cast<int>(written), static_cast<int>(totalSize), dur);
    }

    if (err == Error::WouldBlock) {
      // Failed to write all data. Retry later.
      return;
    }

    if (err != Error::Ok) {
      LOCAL_ERROR("HttpClient[%s]::handleWriteRequest error: %s", debugName().c_str(), err.toString());
      handleDisconnect(lock, Error::SystemError);
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
    handleDisconnect(lock, Error::SystemError);
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
      handleDisconnect(lock, err);
    } else {
      LOCAL_ERROR("HttpClient[%s] - Unexpected error %s", debugName().c_str(), err.toString());
      handleDisconnect(lock, err);
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

    if (onResponse && alive) {
      auto callback = onResponse;
      ScopedUnlock unlock(lock);
      // Danger: if callback references HttpClient object then there is small chance
      // this instance is destroyed right after lock is released.
      callback(req);
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
  return internal_->getQueuedRequests();
}

}
}