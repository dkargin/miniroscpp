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
struct MINIROS_DECL URL {
  /// Network address.
  std::string host;

  uint32_t port = 0;
  /// Address scheme, "http://", "ws://", ...
  std::string scheme;
  /// Path part of URL.
  std::string path;

  URL();

  bool fromString(const std::string& urlStr, bool defaultPort);

  void reset();

  /// Check if URL s empty.
  bool empty() const;

  /// Convert URL back to string.
  std::string str() const;
};

} // namespace network
} // namespace miniros

#endif //MINIROS_URL_H
