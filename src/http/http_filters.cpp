//
// Created by dkargin on 8/2/25.
//

#include "miniros/http/http_filters.h"

#include <cstring>

namespace miniros {
namespace http {

SimpleFilter::SimpleFilter(HttpMethod method, const std::string& path, CheckType checkType)
  :path_(path), method_(method), checkType_(checkType)
{}

bool SimpleFilter::check(const HttpParserFrame& frame) const
{
  if (frame.requestMethod != method_)
    return false;
  if (checkType_ == CheckType::Prefix) {
    return startsWith(frame.getPath(), path_);
  }
  // Exact match.
  return frame.getPath() == path_;
}

RegexFilter::RegexFilter(const std::regex& regex, HttpMethod method)
  :regex_(regex), method_(method)
{}

bool RegexFilter::check(const HttpParserFrame& frame) const
{
  if (frame.requestMethod == method_)
    return false;

  std::string_view path = frame.getPath();
  if (std::regex_match(path.begin(), path.end(), regex_)) {
    return true;
  }
  return false;
}

/// Get final name from path: "/node/ws1/node1" returns "/ws1/node1"
std::string_view getNameFromUrlPath(const std::string_view& path,
  const std::string_view& prefix, bool trailingSlash)
{
  // Even if path == prefix, it is still invalid input, because output name will be empty.
  if (path.size() <= prefix.size()) {
    return {};
  }

  for (size_t i = 0; i < prefix.size(); i++) {
    if (path[i] != prefix[i]) {
      return {};
    }
  }
  size_t cut = prefix.size();
  if (trailingSlash && prefix.size() > 1)
    cut -= 1;
  return path.substr(cut);
}


}
}