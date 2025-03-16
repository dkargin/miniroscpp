//
// Created by dkargin on 3/16/25.
//

#ifndef MINIROS_REQUESTER_INFO_H
#define MINIROS_REQUESTER_INFO_H

#include <string>

#include "miniros/errors.h"
#include "miniros/transport/net_address.h"

namespace miniros {
namespace master {

struct MINIROS_DECL RequesterInfo {
  /// ID of a requester. It is typically name of a node.
  std::string callerId;
  /// URL of node API of requester. Can be empty for some non-node requests.
  std::string callerApi;
  /// Endpoint address of requester.
  /// Address is used to provide accessible URI in the response.
  network::NetAddress clientAddress;
  /// Address of a local endpoint.
  network::NetAddress localAddress;

  RequesterInfo() = default;

  Error assign(const std::string& callerId, int connectionFd);
};

}
} // namespace miniros

#endif //MINIROS_REQUESTER_INFO_H
