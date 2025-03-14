//
// Created by dkargin on 2/28/25.
//

#include <cstring>

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

#else
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "miniros/transport/http_tools.h"

namespace miniros {
namespace network {


void HttpFrame::reset()
{
  header = "";
  request = "";
  fields.clear();
  requestType = {};
  requestUrl = {};
  requestHttpVersion = {};
}

} // namespace network
} // namespace miniros
