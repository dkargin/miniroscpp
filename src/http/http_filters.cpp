//
// Created by dkargin on 8/2/25.
//

#include "miniros/http/http_filters.h"

namespace miniros {
namespace http {

SimpleFilter::SimpleFilter(const std::string& path, HttpMethod method)
  :path_(path), method_(method)
{}

bool SimpleFilter::check(const HttpFrame& frame) const
{
  return frame.requestMethod == method_ && frame.getPath() == path_;
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