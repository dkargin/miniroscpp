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
void HttpRequest::setBody(const std::string& body)
{
  body_ = body;
}

void HttpRequest::setBody(const char* data, size_t size)
{
  body_.assign(data, size);
}

const std::string& HttpRequest::body() const
{
  return body_;
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
  if (!body_.empty()) {
    char buff[40];
    std::snprintf(buff, sizeof(buff), "%zu", body_.size());
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
  status_ = Status::Idle;
  method_ = HttpMethod::Get;
  path_ = "/";
  url_parameters_.clear();
  headers_.clear();
  body_.clear();
  request_start_ = SteadyTime();
  request_finish_ = SteadyTime();
  response_header_.reset();
  response_body_.clear();
}

void HttpRequest::updateStatus(Status status)
{
  std::unique_lock lock(mutex_);
  if (status_ == status)
    return;

  if (status == Status::HasResponse) {
    request_finish_ = SteadyTime::now();
  }
  status_ = status;
  cv_.notify_all();
}

HttpRequest::Status HttpRequest::status() const
{
  std::unique_lock lock(mutex_);
  return status_;
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

const HttpResponseHeader& HttpRequest::responseHeader() const
{
  return response_header_;
}


} // namespace http
} // namespace miniros

