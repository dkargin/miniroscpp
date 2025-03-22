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
namespace network {

/// Intermediate storage for HTTP data.
struct MINIROS_DECL HttpFrame {
  enum ParserState {
    /// Parsing request line.
    ParseRequest,
    /// Parsing name of request field.
    ParseFieldName,
    /// Parsing value of request field.
    ParseFieldValue,
    /// Reading body of request.
    ParseBody,
    /// Finished reading request + body (or only request part if ContentLength=0 or GET).
    ParseComplete,
    /// Parser is in invalid state.
    ParseInvalid
  };

  /// Represent a position of a token in some external buffer, Token = [start, end)
  struct Token {
    int start = -1;
    /// Position of the next symbol after last
    int end = -1;

    void assign(int start_, int end_)
    {
      start = start_;
      end = end_;
    }

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

  /// Container for the full request.
  std::string data;

  /// Number of bytes expected in the request body (parsed from header).
  int contentLength() const
  {
    return m_contentLength;
  }

  /// Current length of request body.
  int bodyLength() const;

  /// Raw request type: {GET, POST, PUT, ...}
  Token requestType;

  /// Request URL
  Token requestUrl;

  /// HTTP Version.
  Token requestHttpVersion;

  /// HTTP Field.
  struct Field {
    std::string name;
    std::string value;
  };

  /// Parsed fields.
  std::vector<Field> fields;

  /// Get a pointer to HTTP version part.
  std::string_view getHttpVersion() const
  {
    return getTokenView(data, requestHttpVersion);
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
  void finishReqeust();

  /// Reset only tokens.
  void resetParseState();

  /// Check if parser has content length field.
  bool hasContentLength() const;

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
  ParserState m_state = HttpFrame::ParseRequest;
};

} // namespace network

} // namespace miniros
#endif // MINIROS_HTTP_H
