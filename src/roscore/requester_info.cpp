//
// Created by dkargin on 3/16/25.
//

#include "requester_info.h"

namespace miniros {
namespace master {

Error RequesterInfo::assign(const std::string& caller, const network::ClientInfo& client)
{
  callerId = caller;
  if (!client.sameProcess) {
    if (!network::readRemoteAddress(client.fd, clientAddress))
      return Error::SystemError;
    if (!network::readLocalAddress(client.fd, localAddress))
      return Error::SystemError;
  }
  return Error::Ok;
}
} // namespace master
} // namespace miniros