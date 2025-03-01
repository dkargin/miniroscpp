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
namespace net {

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

/// Network address.
struct MINIROS_DECL NetAddress {
  enum Type {
    AddressInvalid, AddressIPv4, AddressIPv6
  };

  Type type = AddressInvalid;

  /// String representation of network address.
  std::string address;

  /// Network port.
  int port = 0;

  /// Pointer to actual address implementation.
  void* rawAddress = nullptr;

  ~NetAddress();

  /// Reset internal address.
  void reset();

  /// Check if address is valid.
  bool valid() const { return type != AddressInvalid; }
};

/// Fills in local address from socket.
MINIROS_DECL bool readLocalAddressv4(int sockfd, NetAddress& address);

/// Fills in remote address from socket.
MINIROS_DECL bool readRemoteAddressv4(int sockfd, NetAddress& address);

} // namespace net

} // namespace miniros
#endif // MINIROS_HTTP_H
