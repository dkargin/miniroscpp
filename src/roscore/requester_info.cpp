//
// Created by dkargin on 3/16/25.
//

#include "requester_info.h"

namespace miniros {
namespace master {

Error RequesterInfo::assign(const std::string& callerId, int connectionFd)
{
  this->callerId = callerId;
  if (!network::readRemoteAddress(connectionFd, clientAddress))
    return Error::SystemError;
  if (!network::readLocalAddress(connectionFd, localAddress))
    return Error::SystemError;
  return Error::Ok;
}
} // namespace master
} // namespace miniros