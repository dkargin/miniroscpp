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
class PublishedTopicsEndpoint;
class TopicTypesEndpoint;

struct Master::Internal {
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

  /// Endpoint for accessing /api2/published_topics
  std::shared_ptr<PublishedTopicsEndpoint> httpPublishedTopicsEndpoint;

  /// Endpoint for accessing /api2/topic_types
  std::shared_ptr<TopicTypesEndpoint> httpTopicTypesEndpoint;

  /// Discovery service for companion masters.
  std::unique_ptr<Discovery> discovery;

  /// Timer for periodic broadcasts.
  SteadyTimer timerBroadcasts;

  /// Some unique UUID of this instance.
  UUID uuid;

  Internal(const std::shared_ptr<RPCManager>& manager);
  ~Internal();

  /// Render status of master as HTML page.
  void renderMasterStatus(std::string& output) const;

  /// Render information about specific topic.
  Error renderTopicInfo(const std::string_view& name, std::string& output) const;

  /// Render information about specific node.
  Error renderNodeInfo(const std::string_view& name, std::string& output, bool showInternalInfo) const;

  /// Callback for broadcasting timer.
  /// It is called from main thread.
  void onBroadcast(const SteadyTimerEvent& evt);

  /// Callback for discovery event.
  /// It is called from PollSet thread.
  void onDiscovery(const DiscoveryEvent& evt);
};

}
}

#endif // MINIROS_MASTER_INTERNAL_H
