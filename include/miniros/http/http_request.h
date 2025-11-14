//
// Created by dkargin on 11/14/25.
//

#ifndef MINIROS_HTTP_REQUEST_H
#define MINIROS_HTTP_REQUEST_H

#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <condition_variable>

#include "miniros/macros.h"
#include "miniros/steady_timer.h"
#include "miniros/http/http_tools.h"

namespace miniros {
namespace http {

/// Contains some HTTP request.
/// This object can be reused for further requests once it has been finished.
class MINIROS_DECL HttpRequest {
public:
  enum class Status {
    /// Waiting to be added to queue.
    Idle,
    /// Request is in queue and waiting to be sent.
    Queued,
    /// Sending request. Large requests can be sent in several steps.
    Sending,
    /// Waiting for response from server.
    WaitResponse,
    /// Got HTTP response but waiting for client to process it.
    HasResponse,
  };

  HttpRequest();

  HttpRequest(HttpMethod method, const char* path);

  /// Set HTTP method (GET, POST, etc.)
  void setMethod(HttpMethod method);

  HttpMethod method() const;

  /// Set request path/URI
  void setPath(const std::string& path);

  const std::string& path() const;

  /// Add URL parameter (query string parameter)
  void addParameter(const std::string& name, const std::string& value);

  /// Set URL parameter (query string parameter)
  void setUrlParameter(const std::string& name, const std::string& value);

  /// Get URL parameter value, or empty string if not found
  std::string getParameter(const std::string& name) const;

  /// Get all parameters
  const std::map<std::string, std::string>& urlParameters() const;

  /// Build path with query string from path and parameters
  std::string buildPathWithQuery() const;

  /// Add or set a header
  void setHeader(const std::string& name, const std::string& value);

  /// Get a header value, or empty string if not found
  std::string getHeader(const std::string& name) const;

  /// Set request body (for POST, PUT, etc.)
  void setBody(const std::string& body);

  void setBody(const char* data, size_t size);

  const std::string& body() const;

  /// Build HTTP request header string
  /// @param host - host name for Host header (managed by HttpClient)
  /// @param port - port number for Host header (managed by HttpClient)
  std::string buildHeader(const std::string& host, int port = -1) const;

  /// Reset request for reuse
  void reset();

  void updateStatus(Status status);

  Status status() const;

  void setRequestStart(const SteadyTime& time);

  SteadyTime getRequestStart() const;

  SteadyTime getRequestFinish() const;

  /// Response body
  const std::string& responseBody() const
  {
    return response_body_;
  }

  void setResponseBody(const std::string& body)
  {
    response_body_ = body;
  }

  void setResponseBody(const char* data, size_t size)
  {
    response_body_.assign(data, size);
  }

  /// Set response header from HttpFrame parser state.
  void setResponseHeader(const HttpParserFrame& frame);

  /// Get response header.
  /// It is not thread safe until state == HasResponse.
  const HttpResponseHeader& responseHeader() const;

protected:
  Status status_ = Status::Idle;
  /// Timestamp when request was sent.
  SteadyTime request_start_;
  /// Stamp when response was received.
  SteadyTime request_finish_;

  /// HTTP method
  HttpMethod method_;
  /// Request path/URI
  std::string path_;
  /// URL parameters (query string)
  std::map<std::string, std::string> url_parameters_;
  /// HTTP headers
  std::map<std::string, std::string> headers_;
  /// Request body
  std::string body_;

  HttpResponseHeader response_header_;

  /// Response body
  std::string response_body_;

  mutable std::mutex mutex_;
  std::condition_variable cv_;
};

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_REQUEST_H

