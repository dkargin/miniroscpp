//
// Created by dkargin on 11/14/25.
//

#include <sstream>
#include <cstdio>
#include <cctype>
#include <iomanip>

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "http"

#include "miniros/http/http_request.h"
#include "miniros/http/http_tools.h"

#include "internal/scoped_locks.h"
#include "miniros/rosconsole/local_log.h"

#include <atomic>
#include <cassert>

namespace {
static std::atomic_int g_requestId{0};
}
namespace miniros {
namespace http {


HttpRequest::HttpRequest() : method_(HttpMethod::Get), path_("/"), request_id_(++g_requestId)
{}

HttpRequest::HttpRequest(HttpMethod method, const std::string& path)
  : method_(method), path_(path), request_id_(++g_requestId)
{
  assert(!path.empty());
}

HttpRequest::~HttpRequest()
{}

void HttpRequest::setMethod(HttpMethod method)
{
  method_ = method;
}

HttpMethod HttpRequest::method() const
{
  return method_;
}

/// Set request path/URI
void HttpRequest::setPath(const std::string& path)
{
  path_ = path;
}

const std::string& HttpRequest::path() const
{
  return path_;
}

std::string HttpRequest::debugName() const
{
  return path() + "[" + std::to_string(id()) + "]";
}

/// Add URL parameter (query string parameter)
void HttpRequest::addParameter(const std::string& name, const std::string& value)
{
  url_parameters_[name] = value;
}

/// Set URL parameter (query string parameter)
void HttpRequest::setUrlParameter(const std::string& name, const std::string& value)
{
  url_parameters_[name] = value;
}

/// Parse query string and assign parameters
void HttpRequest::setQueryParameters(const std::string_view& queryString)
{
  if (queryString.empty())
    return;

  size_t start = 0;
  while (start < queryString.size()) {
    // Find the next '&' or end of string
    size_t end = queryString.find('&', start);
    if (end == std::string::npos) {
      end = queryString.size();
    }

    // Extract the key=value pair
    auto pair = queryString.substr(start, end - start);
    
    // Find the '=' separator
    size_t eqPos = pair.find('=');
    if (eqPos != std::string::npos) {
      // Extract key and value
      auto key = pair.substr(0, eqPos);
      auto value = pair.substr(eqPos + 1);
      
      // URL decode both key and value
      url_parameters_[urlDecode(key)] = urlDecode(value);
    } else if (!pair.empty()) {
      // Key without value (treat as empty value)
      url_parameters_[urlDecode(pair)] = "";
    }

    // Move to next pair (skip the '&')
    start = end + 1;
  }
}

/// Get URL parameter value, or empty string if not found
std::string HttpRequest::getParameter(const std::string& name) const
{
  auto it = url_parameters_.find(name);
  if (it != url_parameters_.end())
    return it->second;
  return std::string();
}

/// Get all parameters
const std::map<std::string, std::string>& HttpRequest::urlParameters() const
{
  return url_parameters_;
}

void HttpRequest::setHeader(const std::string& name, const std::string& value)
{
  headers_[name] = value;
}

/// Get a header value, or empty string if not found
std::string HttpRequest::getHeader(const std::string& name) const
{
  auto it = headers_.find(name);
  if (it != headers_.end())
    return it->second;
  return std::string();
}

/// Set request body (for POST, PUT, etc.)
void HttpRequest::setRequestBody(const std::string& body)
{
  request_body_ = body;
}

void HttpRequest::setRequestBody(const char* data, size_t size)
{
  request_body_.assign(data, size);
}

const std::string& HttpRequest::requestBody() const
{
  return request_body_;
}


std::string HttpRequest::buildPathWithQuery() const
{
  std::string result = path_;

  if (!url_parameters_.empty()) {
    result += "?";
    bool first = true;
    for (const auto& [name, value] : url_parameters_) {
      if (!first) {
        result += "&";
      }
      first = false;
      result += urlEncode(name) + "=" + urlEncode(value);
    }
  }

  return result;
}

bool needBody(HttpMethod method)
{
  if (method == HttpMethod::Get || method == HttpMethod::Put || method == HttpMethod::Options) {
    return false;
  }
  return true;
}

std::string HttpRequest::buildHeader(const std::string& host, int port) const
{
  std::ostringstream header;

  // Build path with query string
  std::string fullPath = buildPathWithQuery();

  // Request line: METHOD PATH HTTP/1.1
  header << method_.toString() << " " << fullPath << " HTTP/1.1\r\n";

  // Host header (required in HTTP/1.1)
  if (!host.empty()) {
    header << "Host: " << host;
    if (port > 0 && port != 80 && port != 443) {
      header << ":" << port;
    }
    header << "\r\n";
  }

  // User-Agent header (if not already set)
  if (headers_.find("User-Agent") == headers_.end()) {
    header << "User-Agent: miniros-http-client\r\n";
  }

  // Content-Length header (if body is present)
  if (needBody(method_) && request_body_.empty()) {
    LOCAL_WARN("Unexpected empty request to %s", fullPath.c_str());
  } else {
    char buff[40];
    std::snprintf(buff, sizeof(buff), "%zu", request_body_.size());
    header << "Content-Length: " << buff << "\r\n";
  }

  // Other custom headers
  for (const auto& [name, value] : headers_) {
    // Skip headers we've already added
    if (name == "Host" || name == "Content-Length" || name == "User-Agent")
      continue;
    header << name << ": " << value << "\r\n";
  }

  // End of headers
  header << "\r\n";

  return header.str();
}

/// Reset request for reuse
void HttpRequest::reset()
{
  state_ = State::Idle;
  method_ = HttpMethod::Get;
  path_ = "/";
  url_parameters_.clear();
  headers_.clear();
  request_body_.clear();
  request_start_ = SteadyTime();
  request_finish_ = SteadyTime();
  response_header_.reset();
  response_body_.clear();
}

void HttpRequest::updateState(State state)
{
  Lock lock(mutex_, THIS_LOCATION);
  if (state_ == state)
    return;

  if (state == State::ClientHasResponse) {
    request_finish_ = SteadyTime::now();
  }
  state_ = state;
  cv_.notify_all();
}

HttpRequest::State HttpRequest::state() const
{
  Lock lock(mutex_, THIS_LOCATION);
  return state_;
}

const char* HttpRequest::State::toString() const
{
  switch (value) {
    case Idle:
      return "Idle";
    case ClientQueued:
      return "ClientQueued";
    case ClientSending:
      return "ClientSending";
    case ClientWaitResponse:
      return "ClientWaitResponse";
    case ClientHasResponse:
      return "ClientHasResponse";
    case Done:
      return "Done";
    case ServerReceive:
      return "ServerReceive";
    case ServerHandleRequest:
      return "ServerHandleRequest";
    case ServerSendResponse:
      return "ServerSendResponse";
    default:
      assert(false);
  }
  return "Unknown";
}

void HttpRequest::setRequestStart(const SteadyTime& time)
{
  Lock lock(mutex_, THIS_LOCATION);
  request_start_ = time;
}

SteadyTime HttpRequest::getRequestStart() const
{
  Lock lock(mutex_, THIS_LOCATION);
  return request_start_;
}

SteadyTime HttpRequest::getRequestFinish() const
{
  Lock lock(mutex_, THIS_LOCATION);
  return request_finish_;
}

WallDuration HttpRequest::elapsed() const
{
  Lock lock(mutex_, THIS_LOCATION);
  return SteadyTime::now() - request_start_;
}


void HttpRequest::setResponseHeader(const HttpParserFrame& frame)
{
  Lock lock(mutex_, THIS_LOCATION);
  response_header_.protocol = frame.getProtocolName();
  response_header_.statusCode = frame.responseCode;
  response_header_.status = frame.getResponseStatus();
  response_header_.contentType = frame.getContentType();
}

void HttpRequest::setFailureCallback(std::function<void()>&& cb)
{
  on_fail_ = std::move(cb);
}

void HttpRequest::notifyFailToSend()
{
  if (on_fail_) {
    on_fail_();
  }
}

const std::string& HttpRequest::responseBody() const
{
  return response_body_;
}

void HttpRequest::setResponseBody(const std::string& body, const std::string& contentType)
{
  Lock lock(mutex_, THIS_LOCATION);
  response_body_ = body;
  response_header_.contentType = contentType;
}

void HttpRequest::setResponseBody(const char* data, size_t size)
{
  Lock lock(mutex_, THIS_LOCATION);
  response_body_.assign(data, size);
}

void HttpRequest::resetResponse()
{
  Lock lock(mutex_, THIS_LOCATION);
  response_header_.reset();
  response_body_.clear();
}

void HttpRequest::setResponseStatus(int code, const char* status)
{
  Lock lock(mutex_, THIS_LOCATION);
  response_header_.statusCode = code;
  response_header_.status = status;
}

void HttpRequest::setResponseStatusOk()
{
  Lock lock(mutex_, THIS_LOCATION);
  response_header_.statusCode = 200;
  response_header_.status = "OK";
}

const HttpResponseHeader& HttpRequest::responseHeader() const
{
  return response_header_;
}

Error HttpRequest::waitForState(State state, const WallDuration& duration) const {
  Lock lock(mutex_, THIS_LOCATION);

  if (state_ == state)
    return Error::Ok;
  std::chrono::duration<double> d(duration.toSec());
  if (!cv_.wait_for(lock, d, [this, state]() { return state_ == state; }))
  {
    return Error::Timeout;
  }
  return Error::Ok;
}

Error HttpRequest::waitForResponse(const WallDuration& duration) const
{
  Lock lock(mutex_, THIS_LOCATION);

  if (state_ == State::Done)
    return Error::Ok;
  std::chrono::duration<double> d(duration.toSec());
  if (!cv_.wait_for(lock, d, [this]() {return state_ == State::Done;}))
  {
    return Error::Timeout;
  }
  return Error::Ok;
}


} // namespace http
} // namespace miniros

