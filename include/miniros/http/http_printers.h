//
// Created by dkargin on 9/3/25.
//

#ifndef MINIROS_HTTP_PRINTERS_H
#define MINIROS_HTTP_PRINTERS_H

#include <ostream>
#include <string_view>

namespace miniros {
namespace http {

namespace print {
struct Url {
  Url(const std::string_view& link, const std::string_view& text)
    :link_(link), text_(text)
  {}

  friend std::ostream& operator << (std::ostream& out, const Url& url)
  {
    out << "<a href=\"" << url.link_ << "\">" << url.text_ << "</a>";
    return out;
  }
  std::string_view link_, text_;
};

struct PrefixUrl {
  PrefixUrl(const std::string_view& prefix, const std::string_view& link, const std::string_view& text)
    :prefix_(prefix), link_(link), text_(text)
  {}

  friend std::ostream& operator << (std::ostream& out, const PrefixUrl& url)
  {
    out << "<a href=\"" << url.prefix_ << url.link_ << "\">" << url.text_ << "</a>";
    return out;
  }

  std::string_view prefix_, link_, text_;
};

struct HB {
  HB(const std::string_view& text) : text_(text) {}

  friend std::ostream& operator << (std::ostream& out, const HB& hb)
  {
    out << "<h><b>" << hb.text_ << "</b></h>";
    return out;
  }
  std::string_view text_;
};
}

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_PRINTERS_H
