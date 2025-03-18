//
// Created by dkargin on 3/16/25.
//

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

  // http://192.156.54.23/:11223/RPC2/request?.......

  constexpr char schemeMarker[] = "://";
  std::string::size_type schemePos = uri.find_first_of(schemeMarker);
  if (schemePos == std::string::npos)
    return false;

  schemePos += sizeof(schemeMarker) - 1;
  scheme = uri.substr(0, schemePos);

  std::string::size_type partPos = uri.find_first_of('/', schemePos);

  // Port is somewhere between hostname and resource part.
  std::string::size_type portPos = uri.find_first_of(':', schemePos);
  if (portPos == std::string::npos) {

  } else {

  }

  host = uri.substr(schemePos);

  // split out the port
  std::string::size_type colon_pos = host.find_first_of(":");
  if (colon_pos == std::string::npos)
    return false;
  std::string port_str = host.substr(colon_pos + 1);
  std::string::size_type slash_pos = port_str.find_first_of("/");
  if (slash_pos != std::string::npos)
    port_str = port_str.erase(slash_pos);
  port = atoi(port_str.c_str());
  host = host.erase(colon_pos);
  return true;
}

void URL::reset()
{
  host = {};
  port = 0;
  path = {};
  scheme = {};
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
  if (path.empty())
    ss << "/" << path;
  return ss.str();
}

} // namespace network
} // namespace miniros