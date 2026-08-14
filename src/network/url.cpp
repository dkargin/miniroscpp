//
// Created by dkargin on 3/16/25.
//

#include <cassert>
#include <cstdlib>
#include <sstream>

#include "../../include/miniros/network/url.h"

namespace miniros {
namespace network {

namespace {

using size_type = std::string::size_type;

/// IPv6 literals contain ':' and must be wrapped in [] in a URI (RFC 3986).
bool hostNeedsBrackets(const std::string& host)
{
  if (host.empty() || host.front() == '[')
    return false;
  return host.find(':') != std::string::npos;
}

void parsePathAndQuery(URL& url, const std::string& uri, size_type i)
{
  if (i >= uri.size())
    return;
  if (uri[i] == '/') {
    const size_type q = uri.find('?', i);
    if (q != std::string::npos) {
      url.path = uri.substr(i, q - i);
      url.query = uri.substr(q + 1);
    } else {
      url.path = uri.substr(i);
    }
  } else if (uri[i] == '?') {
    url.query = uri.substr(i + 1);
  }
}

uint32_t parsePortDigits(const std::string& port_str)
{
  return static_cast<uint32_t>(std::atoi(port_str.c_str()));
}

} // namespace

URL::URL()
{
}

bool URL::fromString(const std::string& uri, bool defaultPort)
{
  reset();
  (void)defaultPort;
  //
  // URL with all parts: http://192.156.54.23:11223/RPC2/request?.......
  // IPv6 literal: http://[fd00:a::10]:11311/RPC2
  // File URL with empty host: file:///etc/hosts
  // Extreme URL with only path: /RPC2
  constexpr char schemeMarker[] = "://";

  size_type hostStart = 0;
  const size_type schemePos = uri.find(schemeMarker);
  if (schemePos != std::string::npos) {
    hostStart = schemePos + sizeof(schemeMarker) - 1;
    scheme = uri.substr(0, hostStart);
  }

  if (hostStart >= uri.size())
    return true;

  // RFC 3986 IP-literal: '[' IPv6address [ '%' zone ] ']' [ ':' port ]
  if (uri[hostStart] == '[') {
    const size_type close = uri.find(']', hostStart + 1);
    if (close == std::string::npos)
      return false;

    host = uri.substr(hostStart + 1, close - hostStart - 1);
    size_type i = close + 1;
    if (i < uri.size() && uri[i] == ':') {
      const size_type portStart = i + 1;
      size_type portEnd = uri.find_first_of("/?", portStart);
      if (portEnd == std::string::npos)
        portEnd = uri.size();
      if (portStart == portEnd)
        return false;
      port = parsePortDigits(uri.substr(portStart, portEnd - portStart));
      i = portEnd;
    } else if (i < uri.size() && uri[i] != '/' && uri[i] != '?') {
      return false;
    }
    parsePathAndQuery(*this, uri, i);
    return true;
  }

  // Authority ends at path or query. file:///etc/hosts has an empty host:
  // hostStart already points at the path slash.
  size_type authEnd = uri.find_first_of("/?", hostStart);
  if (authEnd == std::string::npos)
    authEnd = uri.size();

  const std::string authority = uri.substr(hostStart, authEnd - hostStart);
  const size_type colon = authority.find(':');
  if (colon != std::string::npos) {
    // Unbracketed IPv6 is not a valid URI host; refuse rather than guess the port.
    if (authority.find(':', colon + 1) != std::string::npos)
      return false;
    host = authority.substr(0, colon);
    port = parsePortDigits(authority.substr(colon + 1));
  } else {
    host = authority;
  }

  parsePathAndQuery(*this, uri, authEnd);
  return true;
}

void URL::reset()
{
  host = {};
  port = 0;
  path = {};
  scheme = {};
  query = {};
}

bool URL::empty() const
{
  return host.empty();
}

std::string URL::str() const
{
  std::stringstream ss;
  ss << scheme;
  if (hostNeedsBrackets(host))
    ss << '[' << host << ']';
  else
    ss << host;
  if (port)
    ss << ":" << port;
  if (!path.empty()) {
    assert(path[0] == '/');
    ss << path;
  }
  if (!query.empty()) {
    ss << "?" << query;
  }
  return ss.str();
}

bool operator < (const URL& a, const URL& b)
{
  if (a.scheme != b.scheme)
    return a.scheme < b.scheme;
  if (a.host != b.host)
    return a.host < b.host;
  if (a.port != b.port)
    return a.port < b.port;
  if (a.path != b.path)
    return a.path < b.path;
  return a.query < b.query;
}

bool operator == (const URL& a, const URL& b)
{
  return a.port == b.port && a.host == b.host && a.path == b.path && a.query == b.query && a.scheme == b.scheme;
}

bool operator != (const URL& a, const URL& b)
{
  return !(a == b);
}

} // namespace network
} // namespace miniros
