//
// Created by dkargin on 3/16/25.
//

#ifndef MINIROS_URL_H
#define MINIROS_URL_H

#include <cstdint>
#include <string>

#include "miniros/macros.h"

namespace miniros {
namespace network {

/// Annotated URL.
///
/// `host` is stored without RFC 3986 brackets (ready for getaddrinfo).
/// `str()` wraps IPv6 literals in `[]`.
struct MINIROS_DECL URL {
  /// Network address (unbracketed hostname or IP).
  std::string host;

  uint32_t port = 0;
  /// Address scheme, "http://", "ws://", ...
  std::string scheme;
  /// Path part of URL.
  std::string path;

  /// Unparsed query.
  std::string query;

  URL();

  /// Parse a URI. Bracketed IPv6 (`http://[fd00::1]:11311/`) is supported.
  /// Unbracketed IPv6 is rejected. `file:///etc/hosts` has an empty host
  /// and path `/etc/hosts`.
  bool fromString(const std::string& urlStr, bool defaultPort);

  void reset();

  /// Check if URL s empty.
  bool empty() const;

  /// Convert URL back to string.
  std::string str() const;

  friend bool operator < (const URL& a, const URL& b);
  friend bool operator == (const URL& a, const URL& b);
  friend bool operator != (const URL& a, const URL& b);
};

} // namespace network
} // namespace miniros

#endif //MINIROS_URL_H
