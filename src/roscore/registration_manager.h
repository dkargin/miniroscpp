//
// Created by dkargin on 2/12/25.
//

#ifndef MINIROS_REGISTRATION_MANAGER_H
#define MINIROS_REGISTRATION_MANAGER_H

#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "node_ref.h"
#include "requester_info.h"

namespace miniros {

namespace master {

/// Stores registrations for Master.
/// RegistrationManager is not threadsafe, so access must be externally locked as appropriate.
class MINIROS_DECL RegistrationManager {
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  using Lock = std::unique_lock<const RegistrationManager>;

  RegistrationManager();

  Registrations publishers;
  Registrations subscribers;
  Registrations services;

  /// Get a list of URIs for specified topic and caller.
  std::vector<std::string> getTopicSubscribersUri(const std::string& topic, const std::string& caller_api);

  /// Get nodes which publish specified topic.
  std::vector<std::shared_ptr<NodeRef>> getTopicPublishers(const std::string& topic) const;

  using NodeIterator = std::function<bool (const std::shared_ptr<NodeRef>& ptr)>;

  /// Iterate over all publishers.
  size_t iteratePublishers(const std::string_view& topic, const NodeIterator& iterator) const;

  /// Iterate over all subscribers.
  size_t iterateSubscribers(const std::string_view& topic, const NodeIterator& iterator) const;

  /// Get nodes which subscribe to specified topic.
  std::vector<std::shared_ptr<NodeRef>> getTopicSubscribers(const std::string& topic) const;

  /// Find node by its name.
  /// @returns reference to a node or nullptr if no node is found.
  std::shared_ptr<NodeRef> getNodeByName(const std::string& name) const;

  /// Find node by its name.
  /// @returns reference to a node or nullptr if no node is found.
  std::shared_ptr<NodeRef> getNodeByName(const std::string_view& name) const;

  /// Find node by an API URI.
  /// @returns reference to a node or nullptr if no node is found.
  std::shared_ptr<NodeRef> getNodeByAPI(const std::string& nodeApi) const;

  /// Register or update node API.
  /// @returns a pair with {newNode, replacedNode}
  std::pair<NodeRefPtr, NodeRefPtr> registerNodeApi(const std::string& caller_id, const std::string& caller_api);

  /// Internal method for registering an object.
  /// It can allocate new NodeRef for an object, or update an existing one.
  /// @param r - a container for registrations
  /// @param key - name of an object being registered
  /// @param caller_id - name of a node
  /// @param caller_api - API of a node
  /// @param service_api - ?
  /// @returns reference to a node.
  std::shared_ptr<NodeRef> _register(Registrations& r, const std::string& key, const std::string& caller_id, const std::string& caller_api,
    const std::string& service_api = "");

  ReturnStruct unregisterObject(Registrations& r, const std::string& key, const std::string& caller_id,
    const std::string& caller_api, const std::string& service_api = "");

  std::shared_ptr<NodeRef> register_service(const std::string& service, const std::string& caller_id, const std::string& caller_api,
    const std::string& service_api);

  std::shared_ptr<NodeRef> register_publisher(const std::string& topic, const std::string& topic_type,
    const std::string& caller_id, const std::string& caller_api);

  std::shared_ptr<NodeRef> register_subscriber(const std::string& topic, const std::string& topic_type,
    const std::string& caller_id, const std::string& caller_api);

  std::shared_ptr<NodeRef> register_param_subscriber(const std::string& param, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_service(const std::string& service, const std::string& caller_id, const std::string& service_api);

  ReturnStruct unregister_subscriber(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_publisher(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_param_subscriber(const std::string& param, const std::string& caller_id, const std::string& caller_api);

  /// Take collection of nodes for shutdown.
  std::set<std::shared_ptr<NodeRef>> pullShutdownNodes();

  std::ostream& writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings) const;

  /// Drop all registrations for specified node.
  void dropRegistrations(const std::shared_ptr<NodeRef>& node);

  /// List all known nodes.
  std::vector<std::shared_ptr<NodeRef>> listAllNodes() const;

  std::string getTopicType(const std::string_view& name) const;

  std::map<std::string, std::string, std::less<>> getTopicTypes(const std::string& caller_id) const;

  /// Get reference to internal container with topics.
  /// It is thread-unsafe and should be locked by external lock.
  const std::map<std::string, std::string, std::less<>>& getTopicTypesUnsafe(const Lock&) const;

  std::vector<std::vector<std::string>> getPublishedTopics(const std::string& prefix) const;

  void lock() const;
  void unlock() const;

protected:

  /// Find node by its name.
  /// This version does not lock any mutex.
  /// @returns reference to a node or nullptr if no node is found.
  std::shared_ptr<NodeRef> getNodeByNameUnsafe(const std::string_view& name) const;

  /// Find node by an API URI.
  /// This version does not lock any mutex.
  /// @returns reference to a node or nullptr if no node is found.
  std::shared_ptr<NodeRef> getNodeByAPIUnsafe(const std::string& nodeApi) const;

  /// Unregister node and all references to it.
  void unregisterNode(const std::string& nodeApi);

protected:
  mutable std::mutex m_guard;

  /// Maps node name/id to a ref.
  std::map<std::string, std::shared_ptr<NodeRef>, std::less<>> m_nodes;

  /// A collection of nodes queued for shutdown.
  std::set<std::shared_ptr<NodeRef>> m_nodesToShutdown;

  Registrations param_subscribers;

  /// Maps topicName to type md5.
  std::map<std::string, std::string, std::less<>> m_topicTypes;
};

} // namespace master
} // namespace miniros

#endif // MINIROS_REGISTRATION_MANAGER_H
