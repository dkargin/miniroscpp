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
    ParseRequest, ParseFieldName, ParseFieldValue, ParseBody, ParseInvalid
  };

  /// Current state of a parser.
  ParserState state = ParseInvalid;

  /// Request headers.
  std::string header;

  /// Number of bytes expected in the request body (parsed from header).
  int contentLength = 0;

  /// Request body.
  std::string request;

  /// Raw request type: {GET, POST, PUT, ...}
  std::string_view requestType;

  /// Request URL
  std::string_view requestUrl;

  /// HTTP Version.
  std::string_view requestHttpVersion;

  /// HTTP Field.
  struct Field {
    std::string_view name;
    std::string_view value;
  };

  /// Parsed fields.
  std::vector<Field> fields;

  /// Reset all fields
  void reset();
};

} // namespace network

} // namespace miniros
#endif // MINIROS_HTTP_H
