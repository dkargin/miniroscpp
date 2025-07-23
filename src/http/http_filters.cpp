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

bool SimpleFilter::check(const HttpFrame& frame) const
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

bool RegexFilter::check(const HttpFrame& frame) const
{
  if (frame.requestMethod == method_)
    return false;

  std::string_view path = frame.getPath();
  if (std::regex_match(path.begin(), path.end(), regex_)) {
    return true;
  }
  return false;
}

}
}