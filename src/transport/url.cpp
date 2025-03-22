//
// Created by dkargin on 3/16/25.
//

#include <cassert>
#include <sstream>

#include "miniros/transport/url.h"


namespace miniros {
namespace network {


URL::URL()
{
}

bool URL::fromString(const std::string& uri, bool defaultPort)
{
  reset();
  // http://192.156.54.23:11223/RPC2/request?.......
  constexpr char schemeMarker[] = "://";

  std::string::size_type hostStart = 0;
  std::string::size_type schemePos = uri.find_first_of(schemeMarker);
  if (schemePos != std::string::npos) {
    hostStart = schemePos + sizeof(schemeMarker) - 1;
    scheme = uri.substr(0, hostStart);
  }

  std::string::size_type pathStart = uri.find_first_of('/', hostStart);
  // Port is somewhere between hostname and resource part.
  std::string::size_type portPos = uri.find_first_of(':', hostStart);

  std::string::size_type querySearchStart = hostStart;
  if (pathStart != std::string::npos) {
    querySearchStart = pathStart;
  } else if (portPos != std::string::npos) {
    querySearchStart = portPos;
  }
  std::string::size_type queryPos = uri.find_first_of('?',  querySearchStart);

  std::string::size_type portEnd = uri.size();
  if (queryPos != std::string::npos) {
    query = uri.substr(queryPos+1, uri.size() - queryPos - 1);
    portEnd = queryPos;
  }

  if (pathStart != std::string::npos) {
    std::string::size_type pathEnd = queryPos != std::string::npos ? queryPos : uri.size();
    path = uri.substr(pathStart, pathEnd - pathStart);
    portEnd = pathStart;
  }

  std::string::size_type hostEnd = uri.size();
  if (portPos != std::string::npos) {
    hostEnd = portPos;
    std::string port_str = uri.substr(portPos + 1, portEnd - portPos - 1);
    port = atoi(port_str.c_str());
  } else if (pathStart != std::string::npos) {
    hostEnd = pathStart;
  } else if (queryPos != std::string::npos) {
    hostEnd = queryPos;
  }

  host = uri.substr(hostStart, hostEnd - hostStart);
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
  ss << scheme << host;
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

} // namespace network
} // namespace miniros