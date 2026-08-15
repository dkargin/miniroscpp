#ifndef MINIROS_MASTER_INTERNAL_H
#define MINIROS_MASTER_INTERNAL_H

#include <atomic>
#include <map>
#include <set>
#include <string_view>

#include "master.h"

#include "registration_manager.h"
#include "parameter_storage.h"
#include "master_handler.h"
#include "master_cache.h"
#include "multimaster.h"

#include "miniros/rostime.h"

namespace miniros {
namespace master {

class MasterRootEndpoint;
class NodeInfoEndpoint;
class TopicInfoEndpoint;
class PublishedTopicsEndpoint;
class TopicTypesEndpoint;
class MultimasterApiEndpoint;
class DebugApiEndpoint;
class MasterLogEndpoint;

struct Master::Internal {
  std::shared_ptr<RPCManager> rpcManager;

  RegistrationManager regManager;

  AddressResolver resolver;

  MasterHandler handler;
  ParameterStorage parameterStorage;

  /// Persistent GUID/node cache and async restore session.
  MasterCache cache;

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

  /// Endpoint for accessing /api2/multimaster/...
  std::shared_ptr<MultimasterApiEndpoint> httpMultimasterApiEndpoint;

  /// Endpoint for accessing /debugAPI/... (only when debugApiEnabled).
  std::shared_ptr<DebugApiEndpoint> httpDebugApiEndpoint;

  /// Endpoint for accessing GET /log (rosout.log).
  std::shared_ptr<MasterLogEndpoint> httpLogEndpoint;

  /// Multimaster discovery + registration sync over UDP.
  std::unique_ptr<MultimasterManager> multimaster;

  /// Foreign node names imported from each peer UUID string.
  std::map<std::string, std::set<std::string>> foreignNodesByPeer;

  std::shared_ptr<CallbackQueue> callbackQueue;

  /// Some unique UUID of this instance.
  UUID uuid;

  /// Period between node liveness checks (0 disables periodic checks).
  WallDuration nodeCheckPeriod{5.0};

  /// Timestamp of the last periodic node liveness check.
  SteadyTime lastNodeCheck;

  /// Timestamp of the last AddressResolver adapter rescan (late NIC bring-up).
  SteadyTime lastAdapterScan;

  /// Process/master start time (for uptime on the status page).
  SteadyTime startTime{SteadyTime::now()};

  /// When true, /api2/debug/* endpoints are registered.
  bool debugApiEnabled = false;

  /// Set by debug shutdown endpoint; makes Master::ok() return false.
  std::atomic_bool shutdownRequested{false};

  Internal(const std::shared_ptr<RPCManager>& manager);
  ~Internal();

  /// Local hostname used in the status-page caption.
  std::string localHostname() const;

  /// Absolute path of rosout.log, or empty if the log directory is unknown.
  std::string rosoutLogPath() const;

  /// True when rosout.log exists on disk.
  bool rosoutLogConfigured() const;

  /// Render status of master as HTML page.
  void renderMasterStatus(std::string& output) const;

  /// Render information about specific topic.
  Error renderTopicInfo(const std::string_view& name, std::string& output) const;

  /// Render information about specific node.
  Error renderNodeInfo(const std::string_view& name, std::string& output, bool showInternalInfo) const;

  /// Probe registered nodes via getPid and queue unreachable ones for shutdown.
  void checkNodesAlive();

  /// Drop registrations for a node, notify remaining subscribers, and optionally
  /// send a Slave API shutdown request when the connection is still usable.
  void shutdownNode(const std::shared_ptr<NodeRef>& node, const std::string& reason);

  /// Collect local registrations for multimaster snapshot (excludes local-only topics).
  std::vector<miniros_msgs::RegistrationRecord> collectMultimasterSnapshot() const;

  /// Apply inbound multimaster registration records from a peer.
  void applyMultimasterRecords(const UUID& peer, const std::vector<miniros_msgs::RegistrationRecord>& records, bool snapshot);

  /// Drop all foreign registrations imported from a peer.
  void dropMultimasterPeer(const UUID& peer);

  /// Ensure a discovered peer master appears as NODE_MASTER in the node list.
  void registerPeerMasterNode(const PeerInfo& peer);
};

}
}

#endif // MINIROS_MASTER_INTERNAL_H
