//
// Created by dkargin on 8/2/25.
//

#ifndef MINIROS_HTTP_FILTERS_H
#define MINIROS_HTTP_FILTERS_H

#include <string>
#include <regex>

#include "http_endpoint.h"
#include "http_tools.h"

namespace miniros {
namespace http {

/// Simple filter for exact match for path and method.
class SimpleFilter : public EndpointFilter {
public:
  enum class CheckType {
    Exact,
    Prefix,
  };

  SimpleFilter(HttpMethod method, const std::string& path, CheckType checkType = CheckType::Exact);

  bool check(const HttpFrame& frame) const override;

protected:
  std::string path_;
  HttpMethod method_;
  CheckType checkType_;
};

/// Filter based on regular expression.
class RegexFilter : public EndpointFilter {
public:
  RegexFilter(const std::regex& regex, HttpMethod method);

  bool check(const HttpFrame& frame) const override;

protected:
  std::regex regex_;
  HttpMethod method_;
};

/// Get final name from path:
/// @param path path from http request
/// @param prefix to be removed from path.
/// @param trailingSlash - should add trailing slash to output path.
///  "/node/ws1/node1" returns "/ws1/node1".
///  "/node" returns an empty string.
std::string_view getNameFromUrlPath(const std::string_view& path,
  const std::string_view& prefix, bool trailingSlash);

}
}

#endif //MINIROS_HTTP_FILTERS_H
