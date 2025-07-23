//
// Created by dkargin on 8/19/25.
//

#ifndef MINIROS_MASTER_INTERNAL_H
#define MINIROS_MASTER_INTERNAL_H

#include <string_view>

#include "master.h"

#include "registration_manager.h"
#include "parameter_storage.h"
#include "master_handler.h"
#include "discovery.h"

#include "miniros/steady_timer.h"

namespace miniros {
namespace master {

class MasterRootEndpoint;
class NodeInfoEndpoint;
class TopicInfoEndpoint;

struct Master::Internal {
  int port = -1;
  std::string host;

  std::shared_ptr<RPCManager> rpcManager;

  RegistrationManager regManager;

  AddressResolver resolver;

  MasterHandler handler;
  ParameterStorage parameterStorage;

  /// Endpoint for accessing "GET /".
  std::shared_ptr<MasterRootEndpoint> httpRootEndpoint;

  /// Endpoint for accessing /node/<name>
  std::shared_ptr<NodeInfoEndpoint> httpNodeInfoEndpoint;

  /// Endpoint for accessing /topic/<name>
  std::shared_ptr<TopicInfoEndpoint> httpTopicInfoEndpoint;

  /// Discovery service for companion masters.
  Discovery discovery;

  /// Timer for periodic broadcasts.
  SteadyTimer timerBroadcasts;

  Internal(const std::shared_ptr<RPCManager>& manager);
  ~Internal();

  /// Render status of master as HTML page.
  void renderMasterStatus(std::string& output) const;

  /// Render information about specific topic.
  Error renderTopicInfo(const std::string_view& name, std::string& output) const;

  /// Render information about specific node.
  Error renderNodeInfo(const std::string_view& name, std::string& output) const;

  /// Callback for broadcasting timer.
  void onBroadcast(const SteadyTimerEvent& evt);
};

}
}

#endif // MINIROS_MASTER_INTERNAL_H
