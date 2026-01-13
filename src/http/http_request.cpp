//
// Created by dkargin on 11/14/25.
//

#include "miniros/http/http_request.h"
#include <sstream>
#include <cstdio>
#include <cctype>
#include <iomanip>

namespace miniros {
namespace http {

HttpRequest::HttpRequest() : method_(HttpMethod::Get), path_("/")
{}

HttpRequest::HttpRequest(HttpMethod method, const char* path) : method_(method), path_(path)
{}

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

// URL encode a string (percent encoding)
static std::string urlEncode(const std::string& str)
{
  std::ostringstream encoded;
  encoded.fill('0');
  encoded << std::hex;

  for (unsigned char c : str) {
    // Unreserved characters: ALPHA, DIGIT, '-', '.', '_', '~'
    if (std::isalnum(c) || c == '-' || c == '.' || c == '_' || c == '~') {
      encoded << c;
    } else {
      // Percent encode everything else
      encoded << '%' << std::setw(2) << static_cast<int>(c);
    }
  }

  return encoded.str();
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
  if (!request_body_.empty()) {
    char buff[40];
    std::snprintf(buff, sizeof(buff), "%zu", request_body_.size());
    header << "Content-Length: " << buff << "\r\n";
  } else {
    MINIROS_WARN("Unexpected empty request");
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

void HttpRequest::updateState(State status)
{
  std::unique_lock lock(mutex_);
  if (state_ == status)
    return;

  if (status == State::ClientHasResponse) {
    request_finish_ = SteadyTime::now();
  }
  state_ = status;
  cv_.notify_all();
}

HttpRequest::State HttpRequest::state() const
{
  std::unique_lock lock(mutex_);
  return state_;
}

void HttpRequest::setRequestStart(const SteadyTime& time)
{
  std::unique_lock lock(mutex_);
  request_start_ = time;
}

SteadyTime HttpRequest::getRequestStart() const
{
  std::unique_lock lock(mutex_);
  return request_start_;
}

SteadyTime HttpRequest::getRequestFinish() const
{
  std::unique_lock lock(mutex_);
  return request_finish_;
}

void HttpRequest::setResponseHeader(const HttpParserFrame& frame)
{
  std::unique_lock lock(mutex_);
  response_header_.protocol = frame.getProtocolName();
  response_header_.statusCode = frame.responseCode;
  response_header_.status = frame.getResponseStatus();
  response_header_.contentType = frame.getContentType();
}

const std::string& HttpRequest::responseBody() const
{
  return response_body_;
}

void HttpRequest::setResponseBody(const std::string& body, const std::string& contentType)
{
  std::unique_lock lock(mutex_);
  response_body_ = body;
  response_header_.contentType = contentType;
}

void HttpRequest::setResponseBody(const char* data, size_t size)
{
  std::unique_lock lock(mutex_);
  response_body_.assign(data, size);
}

void HttpRequest::setResponseHeader(const HttpResponseHeader& responseHeader)
{
  std::unique_lock lock(mutex_);
  response_header_ = responseHeader;
}

void HttpRequest::resetResponse()
{
  std::unique_lock lock(mutex_);
  response_header_.reset();
  response_body_.clear();
}

void HttpRequest::setResponseStatus(int code, const char* status)
{
  std::unique_lock lock(mutex_);
  response_header_.statusCode = code;
  response_header_.status = status;
}

void HttpRequest::setResponseStatusOk()
{
  std::unique_lock lock(mutex_);
  response_header_.statusCode = 200;
  response_header_.status = "OK";
}

const HttpResponseHeader& HttpRequest::responseHeader() const
{
  return response_header_;
}

Error HttpRequest::waitForResponse(const WallDuration& duration) const
{
  std::unique_lock lock(mutex_);
  if (state_ == State::ClientHasResponse)
    return Error::Ok;
  std::chrono::duration<double> d(duration.toSec());
  if (!cv_.wait_for(lock, d, [this]() {
    return state_ == State::ClientHasResponse;
  })) {
    return Error::Timeout;
  }
  return Error::Ok;
}


} // namespace http
} // namespace miniros

