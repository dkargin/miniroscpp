//
// Created by dkargin on 11/14/25.
//

#include <cassert>
#include <chrono>

#include "miniros/network/socket.h"
#include "miniros/http/http_client.h"

#include "network/net_adapter.h"
#include "transport/poll_set.h"

namespace miniros {
namespace http {

struct HttpClient::Internal {

  /// A queue of requests.
  std::deque<std::shared_ptr<HttpRequest>> requests;

  std::shared_ptr<HttpRequest> active_request;

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

  /// Connected host name
  std::string host_;
  /// Connected port number
  int port_ = -1;

  std::mutex guard;
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
    MINIROS_INFO_NAMED("destructor", "HttpClient::~Internal()");
  }

  void resetResponse();
  Error readResponse();

  /// Close socket.
  void close()
  {
    if (socket) {
      poll_set->delSocket(socket->fd());
    }
    socket.reset();
  }

  int handleEvents(int evtFlags);
  /// Pulls new task and returns event flags.
  int pullNewTaskUnsafe();
};

void HttpClient::Internal::resetResponse()
{
  request_body.resize(0);
  request_header_buffer.resize(0);
  data_sent = 0;
}

int HttpClient::Internal::pullNewTaskUnsafe()
{
  if (requests.empty())
    return 0;

  active_request = requests.front();
  requests.pop_front();

  if (!active_request) {
    MINIROS_WARN("HttpClient::pullNewTask: null request");
    return 0;
  }

  // Update request status
  active_request->updateStatus(HttpRequest::Status::Sending);
  active_request->setRequestStart(SteadyTime::now());

  // Build request header from HttpRequest (host/port managed by HttpClient)
  request_header_buffer = active_request->buildHeader(host_, port_);
  request_body = active_request->body();
  data_sent = 0;

  state = State::WriteRequest;

  return PollSet::EventIn;
}


Error HttpClient::Internal::readResponse()
{
  auto [transferred, readErr] = socket->recv(http_frame.data, nullptr);

  const int fd = socket->fd();
  const int parsed = http_frame.incrementalParse();

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
    return Error::Ok;
  }

  assert(http_frame.state() == HttpParserFrame::ParseComplete);
  // Otherwise, parse and dispatch the request
  MINIROS_DEBUG("HttpClient(%d)::readRequest read %d/%d bytes.", fd, http_frame.bodyLength(), http_frame.contentLength());
  state = State::ProcessResponse;
  return Error::Ok;
}

int HttpClient::Internal::handleEvents(int evtFlags)
{
  std::unique_lock<std::mutex> lock(guard);

  // If we're idle and have requests, start processing
  if (state == State::Idle && !requests.empty()) {
    return pullNewTaskUnsafe();
  }

  if (state == State::Connecting) {
    // Assume socket is in "connecting" state
    assert(socket->isConnecting());
    if (evtFlags | PollSet::EventOut) {
      Error err = socket->checkConnected();
      if (err == Error::Ok) {
        MINIROS_DEBUG_NAMED("HttpClient", "handleEvents(state=Connecting) - connected");
        // Connection is complete.
        state = State::Idle;
        cv.notify_all();
        return pullNewTaskUnsafe();
      } else if (err == Error::Timeout) {
        // Still connecting.
      } else {
        MINIROS_ERROR_NAMED("HttpClient", "handleEvents(state=Connecting) - error: %s", err.toString());
        // TODO: Disconnect.
        return 0;
      }
    }
    if (evtFlags | PollSet::EventError) {
      // TODO: Process error.
    }
  }

  if (state == State::WriteRequest) {
    if (evtFlags & PollSet::EventOut) {
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
        MINIROS_DEBUG("HttpClient written %d/%d bytes of header, t=%fms",
          static_cast<int>(written), static_cast<int>(totalSize), dur);
      }

      if (err == Error::WouldBlock) {
        // Failed to write all data. Retry later.
        return PollSet::EventOut;
      }

      if (err != Error::Ok) {
        MINIROS_ERROR("HttpClient writeResponse error: %s", err.toString());
        // TODO: Should we reconnect here?
        return 0;
      }

      if (data_sent < totalSize) {
        // Failed to write all data, so we need to yield to spinner.
        return PollSet::EventOut;
      }

      data_sent = 0;
      // Request sent, now wait for response
      active_request->updateStatus(HttpRequest::Status::WaitResponse);
      cv.notify_all();
      state = State::ReadResponse;
      http_frame.resetParseState(false); // false = response mode
      auto dur = SteadyTime::now() - active_request->getRequestStart();
      MINIROS_DEBUG("HttpClient sent request in %fms, waiting for response", dur.toSec()*1000);
      return PollSet::EventIn;
    }
    if (evtFlags & PollSet::EventError) {
      int err = socket->getSysError();
      // Here we can have some disconnect problem or some generic network problem.
      MINIROS_WARN("HttpClient::handleEvents socket error in WriteRequest: %i", err);
      // TODO: Should we reconnect here?
      return 0;
    }
    // Some unhandled event.
    MINIROS_WARN("HttpClient unhandled event in WriteRequest: %o", evtFlags);
    return 0;
  }

  if (state == State::ReadResponse) {
    if (evtFlags & PollSet::EventIn) {
      Error err = readResponse();
      // What to do with EOF?

      if (err == Error::Ok) {
        // Request is not finished. Need to wait for another packet.
        if (state == State::ReadResponse) {
          return PollSet::EventIn;
        }
      }
      else if (err == Error::EndOfFile) {
        // Returning 0 will make this connection be closed.
        // TODO: maybe send some signal to user.
        return 0;
      } else {
        MINIROS_ERROR("Unexpected error %s", err.toString());
        return 0;
      }
    } else {
      MINIROS_WARN("HttpClient unhandled events in ReadResponse: %o", evtFlags);
      return 0;
    }
  }

  if (state == State::ProcessResponse) {
    // Response received and parsed - store it in HttpRequest
    if (active_request) {
      active_request->updateStatus(HttpRequest::Status::HasResponse);

      active_request->setResponseHeader(http_frame);
      
      // Store response body
      std::string_view body = http_frame.body();
      if (!body.empty()) {
        active_request->setResponseBody(body.data(), body.size());
      }
      
      auto dur = active_request->getRequestFinish() - active_request->getRequestStart();
      MINIROS_DEBUG("HttpClient received response in %fms", dur.toSec()*1000);
    }

    resetResponse();
    cv.notify_all();
    state = State::Idle;
    return pullNewTaskUnsafe();
  }

  if (state == State::Idle && evtFlags & PollSet::EventOut) {
    // Nothing to do here.
  } else {
    MINIROS_WARN("HttpClient unhandled events: %o in state %d", evtFlags, (int)state);
  }
  return 0;
}

HttpClient::HttpClient(PollSet* ps)
{
  internal_.reset(new Internal(ps));
}

HttpClient::~HttpClient()
{
  if (internal_) {
    std::unique_lock<std::mutex> guard(internal_->guard);
    std::shared_ptr<Internal> copy;
    std::swap(internal_, copy);
    copy->close();
  }
  MINIROS_INFO("HttpClient::~HttpClient()");
}

void HttpClient::close()
{
  if (!internal_)
    return;

  std::unique_lock lock(internal_->guard);
  if (internal_->socket && internal_->socket->valid()) {
    internal_->socket->close();
  }

  internal_->state = State::Invalid;
}

Error HttpClient::attach(std::shared_ptr<NetSocket> sock)
{
  if (!internal_)
    return Error::InternalError;

  if (!sock || !sock->valid()) {
    return Error::InvalidValue;
  }

  internal_->socket = sock;
  if (sock && sock->valid()) {
    internal_->state = State::Idle;
  }
  auto copy = internal_;

  if (!internal_->poll_set->addSocket(sock->fd(), PollSet::EventIn | PollSet::EventOut,
    [copy](int evtFlags) {
      return copy->handleEvents(evtFlags);
    }, copy))
  {
    MINIROS_ERROR("Failed to register socket in PollSet");
    return Error::InternalError;
  }

  return Error::Ok;
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

  std::unique_lock<std::mutex> lock(internal_->guard);
  
  // Set request status to Queued
  request->updateStatus(HttpRequest::Status::Queued);
  internal_->requests.push_back(request);

  // If we're in Idle state and have a valid socket, trigger processing
  if (internal_->state == State::Idle && internal_->socket && internal_->socket->valid()) {
    // Trigger pullNewTask by updating socket events
    // The socket is already in PollSet, so we can trigger it by updating events
    // For now, we'll let the next event cycle handle it, or we could use a callback
    // The simplest approach is to let the existing event loop handle it
    internal_->cv.notify_all();
  }

  return Error::Ok;
}

Error HttpClient::connect(const std::string& host, int port)
{
  network::NetAddress address = network::NetAddress::fromIp4String(host, port);
  if (!address.valid())
    return Error::InvalidAddress;

  // TODO: Do not reconnect if destination address is the same.
  std::unique_lock<std::mutex> lock(internal_->guard);
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
    MINIROS_WARN("HttpClient::connect failed to initialize connection to %s:%d", host.c_str(), port);
    return err;
  }

  // Store host and port for building request headers
  internal_->host_ = host;
  internal_->port_ = port;

  auto copy = internal_;
  internal_->poll_set->addSocket(internal_->socket->fd(), PollSet::EventIn | PollSet::EventOut | PollSet::EventUpdate,
    [copy](int event)
    {
      return copy->handleEvents(event);
    }, copy);

  if (connected) {
    internal_->state = State::Idle;
  } else {
    internal_->state = State::Connecting;
  }

  return Error::Ok;
}

Error HttpClient::waitConnected(const WallDuration& duration)
{
  if (!internal_)
    return Error::InternalError;
  if (!internal_->socket || !internal_->socket->valid())
    return Error::InvalidHandle;
  std::unique_lock<std::mutex> lock(internal_->guard);
  if (internal_->cv.wait_for(lock, std::chrono::duration<double>(duration.toSec()),
    [this]() {
      if (internal_->state == State::Invalid || internal_->state == State::Connecting)
        return false;
      return true;
    })) {
    return Error::Ok;
  }
  return Error::Timeout;
}

}
}