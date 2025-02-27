//
// Created by dkargin on 2/11/25.
//

#ifndef MINIROS_MASTER_H
#define MINIROS_MASTER_H

#include "master_handler.h"
#include "parameter_storage.h"

#include "miniros/transport/rpc_manager.h"

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

namespace miniros {
namespace master {

/// Core class of rosmaster/roscore.
class MINIROS_DECL Master {

public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using Connection = XmlRpc::XmlRpcServerConnection;

  Master(std::shared_ptr<RPCManager> manager);
  ~Master();

  bool start();
  void stop();
  bool ok() const;

  /// Setup RPC callbacks.
  void setupBindings();

  RpcValue lookupService(const std::string& caller_id, const std::string& service, Connection*);

  /// Register the caller as a provider of the specified service.
  /// Parameters
  ///  - caller_id (str)   - ROS caller ID
  ///  - service (str)     - Fully-qualified name of service
  ///  - service_api (str) - ROSRPC Service URI
  ///  -  caller_api (str)  - XML-RPC URI of caller node
  /// Returns (int, str, int) - (code, statusMessage, ignore)
  RpcValue registerService(const std::string& caller_id, const std::string& service, const std::string& caller_api,
    const std::string& service_api, Connection*);

  RpcValue unregisterService(
    const std::string& caller_id, const std::string& service, const std::string& service_api, Connection*);

  RpcValue getTopicTypes(const std::string& topic, Connection*);

  /// Callback from master with updated value of subscribed parameter.
  /// Parameters
  ///  - caller_id (str) ROS caller ID.
  ///  - parameter_key (str) Parameter name, globally resolved.
  ///  - parameter_value (!XMLRPCLegalValue) New parameter value.
  /// Returns (int, str, int) (code, statusMessage, ignore)
  RpcValue paramUpdate(
    const std::string& caller_id, const std::string& parameter_key, const RpcValue& value, Connection*);

  /// Retrieve list representation of system state (i.e. publishers, subscribers, and services).
  /// Parameters:
  /// - caller_id (str) ROS caller ID
  /// Returns (int, str, [ [str,[str] ], [str,[str] ], [str,[str] ] ]) - (code, statusMessage, systemState)
  ///  System state is in list representation [publishers, subscribers, services]
  ///   - publishers is of the form [ [topic1, [topic1Publisher1...topic1PublisherN]] ... ]
  ///   - subscribers is of the form [ [topic1, [topic1Subscriber1...topic1SubscriberN]] ... ]
  ///   - services is of the form [ [service1, [service1Provider1...service1ProviderN]] ... ]
  RpcValue getSystemState(const std::string& caller_id, Connection*);

  /// Get list of topics that can be subscribed to. This does not return topics that have no publishers.
  /// See getSystemState() to get more comprehensive list.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - subgraph (str) - Restrict topic names to match within the specified subgraph.
  ///    Subgraph namespace is resolved relative to the caller's namespace. Use empty string to specify all names.
  /// Returns (int, str, [[str, str],]) - (code, statusMessage, [ [topic1, type1]...[topicN, typeN] ])
  RpcValue getPublishedTopics(const std::string& caller_id, const std::string& subgraph, Connection*);

  /// Register a new publisher to a topic
  RpcValue registerPublisher(const std::string& caller_id, const std::string& topic, const std::string& type,
    const std::string& caller_api, Connection* /*conn*/);

  /// Unregister an existing publisher
  RpcValue unregisterPublisher(
    const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* /*conn*/);

  /// Register a new subscriber
  RpcValue registerSubscriber(const std::string& caller_id, const std::string& topic, const std::string& type,
    const std::string& caller_api, Connection* /*conn*/);

  /// Unregister an existing subscriber
  RpcValue unregisterSubscriber(
    const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* /*conn*/);

  /// Get the XML-RPC URI of the node with the associated name/caller_id.
  /// This API is for looking information about publishers and subscribers.
  /// Use lookupService instead to lookup ROS-RPC URIs.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - node (str) - Name of node to lookup
  /// Returns (int, str, str) (code, statusMessage, URI)
  RpcValue lookupNode(const std::string& caller_id, const std::string& node, Connection* conn);

  RpcValue getTime(Connection*);

  /// Parameter API

  /// Check whether a parameter exists
  RpcValue hasParam(const std::string& caller_id, const std::string& param, Connection* /*conn*/);

  /// Set parameter. NOTE: if value is a dictionary it will be treated as a parameter tree, where key is the parameter namespace. For example
  /// {'x':1,'y':2,'sub':{'z':3}}
  /// will set key/x=1, key/y=2, and key/sub/z=3. Furthermore, it will replace all existing parameters in the key parameter namespace with the parameters in value. You must set parameters individually if you wish to perform a union update.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - key (str) - Parameter name.
  ///  - value (XMLRPCLegalValue) - Parameter value.
  /// Returns (int, str, int) (code, statusMessage, ignore)
  RpcValue setParam(const std::string& caller_api, const std::string& key, const RpcValue& value, Connection* /*conn*/);

  /// Retrieve a value for an existing parameter, if it exists.
  RpcValue getParam(const std::string& caller_id, const std::string& key, Connection*);

  /// Delete a parameter, if it exists
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID
  ///  - key (str) - Parameter name.
  /// Returns (int, str, int) - (code, statusMessage, ignore)
  RpcValue deleteParam(const std::string& caller_id, const std::string& key, Connection*);

  RpcValue searchParam(const std::string& caller_id, const std::string& key, Connection*);

  RpcValue subscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key, Connection*);
  RpcValue unsubscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key, Connection*);


  RpcValue getParamNames(const std::string& caller_id, Connection*);

  /// SlaveAPI

  /// Retrieve a list of topics that this node publishes.
  /// Parameters
  ///  - caller_id (str) - ROS caller ID.
  /// Returns (int, str, [ [str, str] ]) - (code, statusMessage, topicList)
  ///  topicList is a list of topics published by this node and is of the form
  ///  [ [topic1, topicType1]...[topicN, topicTypeN] ]
  RpcValue getPublications(const std::string& caller_id, Connection*);

  /// Retrieve a list of topics that this node subscribes to
  ///  getSubscriptions(caller_id)
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID.
  /// Returns (int, str, [ [str, str] ]) - (code, statusMessage, topicList)
  /// topicList is a list of topics this node subscribes to and is of the form
  /// [ [topic1, topicType1]...[topicN, topicTypeN] ]
  RpcValue getSubscriptions(const std::string& caller_id, Connection*);

  /// Publisher node API method called by a subscriber node. This requests that source allocate a channel
  /// for communication. Subscriber provides a list of desired protocols for communication.
  /// Publisher returns the selected protocol along with any additional params required for establishing connection.
  /// For example, for a TCP/IP-based connection, the source node may return a port number of TCP/IP server.
  ///
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID.
  ///  - topic (str) Topic name.
  ///  - protocols ([ [str, !XMLRPCLegalValue*] ]) List of desired protocols for communication in order of preference.
  ///    Each protocol is a list of the form [ProtocolName, ProtocolParam1, ProtocolParam2...N]
  /// Returns (int, str, [str, !XMLRPCLegalValue*] ) (code, statusMessage, protocolParams)
  /// protocolParams may be an empty list if there are no compatible protocols.
  RpcValue requestTopic(const std::string& caller_id, const std::string& topic, const RpcValue& protocols, Connection*);

  /// Callback from master of current publisher list for specified topic.
  /// Parameters:
  /// - caller_id (str) ROS caller ID.
  /// - topic (str) Topic name.
  /// - publishers ([str]) List of current publishers for topic in the form of XMLRPC URIs
  /// Returns (int, str, int) (code, statusMessage, ignore)
  RpcValue publisherUpdate(
    const std::string& caller_id, const std::string& topic, const RpcValue& publishers, Connection*);

  /// Retrieve transport/topic statistics.
  /// Parameters:
  ///  - caller_id (str) ROS caller ID.
  /// Returns (int, str, [XMLRPCLegalValue*]) (code, statusMessage, stats)
  ///  stats is of the form [publishStats, subscribeStats, serviceStats] where
  ///  publishStats: [[topicName, messageDataSent, pubConnectionData]...]
  ///    \ pubConnectionData: [connectionId, bytesSent, numSent, connected]*
  ///  subscribeStats: [[topicName, subConnectionData]...]
  ///    \ subConnectionData: [connectionId, bytesReceived, numReceived, dropEstimate, connected]*
  ///  serviceStats: (proposed) [numRequests, bytesReceived, bytesSent]
  ///  dropEstimate: -1 if no estimate.
  RpcValue getBusStats(const std::string& caller_id, Connection* conn);

  /// Retrieve transport/topic connection information.
  /// Parameters:
  ///  - caller_id (str) - ROS caller ID.
  /// Returns (int, str, [XMLRPCLegalValue*]) (code, statusMessage, busInfo)
  /// busInfo is of the form: [[connectionId1, destinationId1, direction1, transport1, topic1, connected1]... ]
  ///  connectionId is defined by the node and is opaque.
  ///  destinationId is the XMLRPC URI of the destination.
  ///  direction is one of 'i', 'o', or 'b' (in, out, both).
  ///  transport is the transport type (e.g. 'TCPROS').
  ///  topic is the topic name.
  ///  connected1 indicates connection status. Note that this field is only provided by slaves written
  ///   in Python at the moment (cf. rospy/masterslave.py in _TopicImpl.get_stats_info() vs. roscpp/publication.cpp
  /// in Publication::getInfo()).
  RpcValue getBusInfo(const std::string& caller_id, Connection* conn);

  RpcValue getPid(const std::string& caller_id, Connection* conn);

protected:
  int m_port = -1;
  std::string m_host;

  std::shared_ptr<RPCManager> m_manager;

  RegistrationManager m_regManager;

  MasterHandler m_handler;
  ParameterStorage m_parameterStorage;
};

} // namespace master

} // namespace miniros

#endif // MINIROS_MASTER_H
