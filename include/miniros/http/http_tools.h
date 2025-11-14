//
// Created by dkargin on 2/28/25.
//

#ifndef MINIROS_HTTP_H
#define MINIROS_HTTP_H

#include <string>
#include <string_view>
#include <vector>

#include "miniros/common.h"

namespace miniros {
namespace http {

struct HttpMethod {
  enum Value_t {
    Invalid, //< Invalid method type.
    Get,
    Head,
    Options,
    Trace,
    Put,
    Delete,
    Post,
    Patch,
    Connect
  }value;

  HttpMethod(Value_t val) : value(val) {}

  operator Value_t() const
  {
    return value;
  }

  const char* toString() const;
};

/// Intermediate storage for HTTP data.
struct MINIROS_DECL HttpParserFrame {
  enum ParserState {
    /// Parser is in invalid state.
    ParseInvalid,
    /// Parsing request line.
    ParseRequestHeader,
    /// Parsing response line.
    ParseResponseHeader,
    /// Parsing name of request field.
    ParseFieldName,
    /// Parsing value of request field.
    ParseFieldValue,
    /// Reading body of request.
    ParseBody,
    /// Finished reading request + body (or only request part if ContentLength=0 or GET).
    ParseComplete
  };

  /// Represent a position of a token in some external buffer, Token = [start, end)
  struct Token {
    int start = -1;
    /// Position of the next symbol after last
    int end = -1;

    void assign(int start_, int end_);

    void reset()
    {
      start = -1;
      end = -1;
    }

    /// Check of token is valid.
    bool valid() const
    {
      return start >= 0 && start < end;
    }

    /// check if token is empty.
    bool empty() const
    {
      return start == end;
    }

    /// Return size of a token.
    size_t size() const
    {
      return end - start;
    }
  };

  /// Parse http method.
  static HttpMethod parseMethod(const char* data, const Token& token);

  /// Container for the full request.
  std::string data;

  /// Number of bytes expected in the request body (parsed from header).
  int contentLength() const
  {
    return m_contentLength;
  }

  /// Current length of request body.
  int bodyLength() const;

  /// GET, POST, ...
  Token requestMethodToken;

  /// Response code: 200, ...
  Token responseCodeToken;
  int responseCode = 0;

  /// Response status: OK, ...
  Token responseStatus;

  /// Request method: {GET, POST, PUT, ...}
  HttpMethod requestMethod = HttpMethod::Invalid;

  /// Requested path
  Token requestPath;

  /// Protocol name. Expect string like:
  ///   HTTP/1.1
  ///   RTSP/1.0,
  ///   ...
  Token protocol;

  /// HTTP Field.
  struct Field {
    std::string name;
    std::string value;
  };

  /// Content type of request or response.
  Token contentType;

  /// Contains all unspecialized request fields.
  std::vector<Field> fields;

  /// Get a pointer to HTTP version part.
  std::string_view getProtocolName() const
  {
    return getTokenView(data, protocol);
  }

  std::string_view getResponseStatus() const
  {
    return getTokenView(data, responseCodeToken);
  }

  std::string_view getContentType() const
  {
    return getTokenView(data, contentType);
  }

  std::string_view getPath() const
  {
    return getTokenView(data, requestPath);
  }

  ParserState state() const
  {
    return m_state;
  }

  /// Return true if all header is parsed.
  bool hasHeader() const;

  static std::string_view getTokenView(const std::string& data, const Token& token);

  /// Continue parsing data.
  int incrementalParse();

  /// Finish parsing request.
  /// It will reset all parser-related state and fields and drop all data, related to current packet (if any).
  void finishRequest()
  {
    finish(true);
  }

  void finishResponse()
  {
    finish(false);
  }

  void finish(bool request);


  /// Reset only tokens.
  /// @param request - should reset to request or response.
  void resetParseState(bool request);

  /// Check if parser has content length field.
  bool hasContentLength() const;

  /// Get reference to header of request.
  std::string_view header() const;

  /// Get reference to body of request.
  std::string_view body() const;

  bool keepAlive() const;

protected:

  /// Current position of a parser.
  int m_currentPosition = 0;
  /// Current position of request body.
  int m_bodyPosition = 0;
  /// End position of request body.
  int m_bodyEnd = 0;

  /// Current position of a token. Part of incremental state.
  int m_tokenStart = 0;

  int m_contentLength = -1;

  bool m_keepAlive = true;

  /// Position of name field. We need to keep it while parsing value field.
  Token m_fieldName;

  /// Current state of a parser.
  ParserState m_state = HttpParserFrame::ParseInvalid;
};

/// Header of HTTP response.
/// It encapsulates most important parts of HTTP response before serializing it to a buffer.
struct HttpResponseHeader {
  /// Internal error.
  Error error = Error::Ok;

  std::string protocol = "HTTP/1.1";

  /// Status code.
  int statusCode = 200;

  /// Text representation of status.
  std::string status = "OK";

  std::string server;

  /// Type of the content.
  std::string contentType;

  /// Reset contents of response.
  void reset();

  /// Serialize to string.
  void writeHeader(std::string& output, size_t bodySize) const;
};

/// Checks if a string starts with a prefix.
bool startsWith(const std::string_view& str, const std::string_view& prefix);

} // namespace network

} // namespace miniros
#endif // MINIROS_HTTP_H
