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

}
}

#endif //MINIROS_HTTP_FILTERS_H
