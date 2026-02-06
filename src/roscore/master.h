//
// Created by dkargin on 2/11/25.
//

#ifndef MINIROS_MASTER_H
#define MINIROS_MASTER_H

#include <string>
#include <memory>

#include "miniros/macros.h"
#include "miniros/errors.h"

/*
More info at:

http://wiki.ros.org/ROS/Master_API

http://docs.ros.org/en/melodic/api/rosmaster/html/rosmaster.master_api-module.html

API return convention: (statusCode, statusMessage, returnValue)

    statusCode: an integer indicating the completion condition of the method.
    statusMessage: a human-readable string message for debugging
    returnValue: the return value of the method; method-specific.

Current status codes:

    -1: ERROR: Error on the part of the caller, e.g. an invalid parameter
    0: FAILURE: Method was attempted but failed to complete correctly.
    1: SUCCESS: Method completed successfully.
*/

namespace XmlRpc {
class XmlRpcValue;
}
namespace miniros {

class PollSet;
class RPCManager;
class NodeHandle;
class CallbackQueue;

namespace network {
struct ClientInfo;
}

namespace master {

class NodeRef;

/// Core class of rosmaster/roscore.
class MINIROS_DECL Master {
public:
  struct Internal;

  using RpcValue = XmlRpc::XmlRpcValue;
  using ClientInfo = network::ClientInfo;

  Master(std::shared_ptr<RPCManager> manager);
  ~Master();

  bool start(PollSet* poll_set, int port);
  void stop();
  bool ok() const;

  /// Setup RPC callbacks.
  void setupBindings(const std::shared_ptr<CallbackQueue>& cb);

  std::string getUri() const;
  int getPort() const;

  void setResolveNodeIP(bool resolv);

  /// Enable automatic discovery of other masters.
  void enableDiscoveryBroadcasts(int port);

  /// Set multicast address for service discovery.
  /// Can return an error if multicast address is invalid.
  Error setDiscoveryGroup(const std::string& group);

  /// Update queued tasks.
  void update();

  /// Init periodic events.
  void initEvents(NodeHandle& nh);

  std::shared_ptr<NodeRef> registerNodeApi(const std::string& nodeId, const std::string& nodeApi) const;
  std::shared_ptr<NodeRef> getNodeByName(const std::string& nodeId) const;

public: /// Request handlers
  RpcValue lookupService(const std::string& caller_id, const std::string& service, const ClientInfo& ci);

  /// Register the caller as a provider of the specified service.
  /// Parameters
  ///  - caller_id (str)   - ROS caller ID
  ///  - service (str)     - Fully-qualified name of service
  ///  - service_api (str) - ROSRPC Service URI
  ///  - caller_api (str)  - XML-RPC URI of caller node
  /// Returns (int, str, int) - (code, statusMessage, ignore)
  RpcValue registerService(const std::string& caller_id, const std::string& service,
    const std::string& service_api, const std::string& caller_api, const ClientInfo&);

  RpcValue unregisterService(const std::string& caller_id, const std::string& service,
    const std::string& service_api, const ClientInfo&);

  RpcValue getTopicTypes(const std::string& caller_id, const ClientInfo&);

  /// Callback from master with updated value of subscribed parameter.
  /// Parameters
  ///  - caller_id (str) ROS caller ID.
  ///  - parameter_key (str) Parameter name, globally resolved.
  ///  - parameter_value (!XMLRPCLegalValue) New parameter value.
  /// Returns (int, str, int) (code, statusMessage, ignore)
  RpcValue paramUpdate(const std::string& caller_id, const std::string& parameter_key,
    const RpcValue& value, const ClientInfo&);

  /// Retrieve list representation of system state (i.e. publishers, subscribers, and services).
  /// Parameters:
  /// - caller_id (str) ROS caller ID
  /// Returns (int, str, [ [str,[str] ], [str,[str] ], [str,[str] ] ]) - (code, statusMessage, systemState)
  ///  System state is in list representation [publishers, subscribers, services]
  ///   - publishers is of the form [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]
  ///   - subscribers is of the form [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
  ///   - services is of the form [ [service1, [service1Provider1...service1ProviderN]] ... ]
  RpcValue getSystemState(const std::string& caller_id, const ClientInfo&);

  /// Get list of topics that can be subscribed to. This does not return topics that have no publishers.
  /// See getSystemState() to get more comprehensive list.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - subgraph (str) - Restrict topic names to match within the specified subgraph.
  ///    Subgraph namespace is resolved relative to the caller's namespace. Use empty string to specify all names.
  /// Returns (int, str, [[str, str],]) - (code, statusMessage, [ [topic1, type1]...[topicN, typeN] ])
  RpcValue getPublishedTopics(const std::string& caller_id, const std::string& subgraph, const ClientInfo&);

  /// Register a new publisher to a topic
  RpcValue registerPublisher(const std::string& caller_id, const std::string& topic, const std::string& type,
    const std::string& caller_api, const ClientInfo& /*conn*/);

  /// Unregister an existing publisher
  RpcValue unregisterPublisher(
    const std::string& caller_id, const std::string& topic, const std::string& caller_api, const ClientInfo& /*conn*/);

  /// Register a new subscriber
  RpcValue registerSubscriber(const std::string& caller_id, const std::string& topic, const std::string& type,
    const std::string& caller_api, const ClientInfo& /*conn*/);

  /// Unregister an existing subscriber
  RpcValue unregisterSubscriber(
    const std::string& caller_id, const std::string& topic, const std::string& caller_api, const ClientInfo& /*conn*/);

  /// Get the XML-RPC URI of the node with the associated name/caller_id.
  /// This API is for looking information about publishers and subscribers.
  /// Use lookupService instead to lookup ROS-RPC URIs.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - node (str) - Name of node to lookup
  /// Returns (int, str, str) (code, statusMessage, URI)
  RpcValue lookupNode(const std::string& caller_id, const std::string& node, const ClientInfo& conn);

  /// Parameter API

  /// Check whether a parameter exists
  RpcValue hasParam(const std::string& caller_id, const std::string& param, const ClientInfo& /*conn*/);

  /// Set parameter. NOTE: if value is a dictionary it will be treated as a parameter tree, where key is the parameter namespace. For example
  /// {'x':1,'y':2,'sub':{'z':3}}
  /// will set key/x=1, key/y=2, and key/sub/z=3. Furthermore, it will replace all existing parameters in the key parameter namespace with the parameters in value. You must set parameters individually if you wish to perform a union update.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - key (str) - Parameter name.
  ///  - value (XMLRPCLegalValue) - Parameter value.
  /// Returns (int, str, int) (code, statusMessage, ignore)
  RpcValue setParam(const std::string& caller_api, const std::string& key, const RpcValue& value, const ClientInfo& /*conn*/);

  /// Retrieve a value for an existing parameter, if it exists.
  RpcValue getParam(const std::string& caller_id, const std::string& key, const ClientInfo&);

  /// Delete a parameter, if it exists
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - key (str) - Parameter name.
  /// Returns (int, str, int) - (code, statusMessage, ignore)
  RpcValue deleteParam(const std::string& caller_id, const std::string& key, const ClientInfo&);

  RpcValue searchParam(const std::string& caller_id, const std::string& key, const ClientInfo&);

  RpcValue subscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key, const ClientInfo&);
  RpcValue unsubscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key, const ClientInfo&);

  RpcValue getParamNames(const std::string& caller_id, const ClientInfo&);
  void setDumpParameters(bool dump);

protected:
  std::unique_ptr<Internal> internal_;
};

} // namespace master

} // namespace miniros

#endif // MINIROS_MASTER_H
