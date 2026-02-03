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
#include "miniros/http/http_tools.h"

namespace miniros {
namespace http {

/// Contains some HTTP request.
/// This object can be reused for further requests once it has been finished.
/// Role for HttpClient:
///  - its instance is created when client sends request and when server handles response.
/// Role for HttpServerConnection
///  - its instance is created when server has received all parts of request.
class MINIROS_DECL HttpRequest {
public:
  /// Represents high-level state of Http request object.
  enum class State {
    /// Waiting to be added to queue.
    Idle,

    /// Request is in queue and waiting to be sent.
    ClientQueued,
    /// Sending request. Large requests can be sent in several steps.
    ClientSending,
    /// Waiting for response from server.
    ClientWaitResponse,
    /// Got HTTP response but waiting for client to process it.
    ClientHasResponse,
    /// processResponse was called.
    ClientDone,

    /// Receiving request.
    ServerReceive,
    /// Request is received and parsed, but not processed.
    ServerHandleRequest,
    /// Sending back response data,
    ServerSendResponse,
  };

  HttpRequest();

  HttpRequest(HttpMethod method, const std::string& path);

  virtual ~HttpRequest();

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
  void setRequestBody(const std::string& body);

  void setRequestBody(const char* data, size_t size);

  const std::string& requestBody() const;

  /// Build HTTP request header string
  /// @param host - host name for Host header (managed by HttpClient)
  /// @param port - port number for Host header (managed by HttpClient)
  std::string buildHeader(const std::string& host, int port = -1) const;

  /// Reset request for reuse
  virtual void reset();

  void updateState(State status);

  State state() const;

  void setRequestStart(const SteadyTime& time);

  SteadyTime getRequestStart() const;

  SteadyTime getRequestFinish() const;

  /// Get reference to response body.
  /// It is thread unsafe.
  const std::string& responseBody() const;

  void setResponseBody(const std::string& body, const std::string& contentType);

  void setResponseBody(const char* data, size_t size);

  /// Set response status and code.
  void setResponseStatus(int code, const char* status);

  /// Set response status OK 200
  void setResponseStatusOk();

  /// Set response header from HttpFrame parser state.
  void setResponseHeader(const HttpParserFrame& frame);

  /// Get response header.
  /// It is not thread safe until state == HasResponse.
  const HttpResponseHeader& responseHeader() const;

  /// Wait until status changes to Status::ClientHasResponse.
  Error waitForResponse(const WallDuration& duration) const;

  /// Wait for specific status.
  Error waitForState(State state, const WallDuration& duration) const;

  /// Reset response-related data.
  void resetResponse();

  /// Process response.
  /// It is called by HttpClient/Server. No additional lock is set on response object.
  /// HttpClient updates state of request to ClientDone right after processResponse is called.
  virtual Error processResponse() { return Error::Ok; }

  /// Called by HttpClient when it has failed to send request.
  void notifyFailToSend();

  void setFailureCallback(std::function<void()>&& cb);

  bool shouldRetry() const
  {
    return retry_ > 0;
  }

  /// Enable sending retries.
  void setRetry(int val)
  {
    retry_ = val;
  }

protected:
  State state_ = State::Idle;
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
  std::string request_body_;

  /// Retry to send connection.
  int retry_ = 0;

  HttpResponseHeader response_header_;

  /// Response body
  std::string response_body_;

  /// Called by HttpClient when it has failed to send request.
  std::function<void ()> on_fail_;

  mutable std::mutex mutex_;
  mutable std::condition_variable cv_;
};

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_REQUEST_H

