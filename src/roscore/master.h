//
// Created by dkargin on 2/11/25.
//

#ifndef MINIROS_MASTER_H
#define MINIROS_MASTER_H

#include "miniros/console.h"
#include "miniros/transport/xmlrpc_manager.h"
#include "master_api.h"

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

/// Core class of rosmaster/roscore.
class MINIROS_DECL Master
{
  protected:
    int _port=-1;
    std::string _host;
    ROSMasterHandler handler;
    std::unique_ptr<XMLRPCManager> manager;

  public:
    using XmlRpcValue = XmlRpc::XmlRpcValue;
    using Connection = XmlRpc::XmlRpcServerConnection;

    Master();

    bool start();
    void stop();
    bool ok() const;

    XMLRPCFuncEx wrap0(Master* master,
      XmlRpcValue (Master::*method)(Connection* conn))
    {
      return [=] (const XmlRpcValue& param, XmlRpcValue& result, Connection* conn) {
        result = (master->*method)(conn);
        return 0;
      };
    }

    template <class T0>
    XMLRPCFuncEx wrap1(Master* master,
      XmlRpcValue (Master::*method)(const T0& arg0, Connection* conn))
    {
      return [=] (const XmlRpcValue& param, XmlRpcValue& result, Connection* conn) {
        T0 arg0 = param[0].as<T0>();
        result = (master->*method)(arg0, conn);
        return 0;
      };
    }

    template <class T0, class T1>
    XMLRPCFuncEx wrap2(Master* master,
      XmlRpcValue (Master::*method)(const T0& arg0, const T1& arg1, Connection* conn))
    {
      return [=] (const XmlRpcValue& param, XmlRpcValue& result, Connection* conn) {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        result = (master->*method)(arg0, arg1, conn);
        return 0;
      };
    }

    template <class T0, class T1, class T2>
    XMLRPCFuncEx wrap3(Master* master,
      XmlRpcValue (Master::*method)(const T0& arg0, const T1& arg1, const T2& arg2, Connection* conn))
    {
      return [=] (const XmlRpcValue& param, XmlRpcValue& result, Connection* conn) {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        result = (master->*method)(arg0, arg1, arg2, conn);
        return 0;
      };
    }

    template <class T0, class T1, class T2, class T3>
    XMLRPCFuncEx wrap4(Master* master,
      XmlRpcValue (Master::*method)(const T0& arg0, const T1& arg1, const T2& arg2, const T3& arg3, Connection* conn))
    {
      return [=] (const XmlRpcValue& param, XmlRpcValue& result, Connection* conn) {
        T0 arg0 = param[0].as<T0>();
        T1 arg1 = param[1].as<T1>();
        T2 arg2 = param[2].as<T2>();
        T3 arg3 = param[3].as<T3>();
        result = (master->*method)(arg0, arg1, arg2, arg3, conn);
        return 0;
      };
    }

    /// Setup RPC callbacks.
    void setupBindings();

    XmlRpcValue lookupService(const std::string& caller_id, const std::string& service, Connection*);

    /// Register the caller as a provider of the specified service.
    /// Parameters
    ///  - caller_id (str)   - ROS caller ID
    ///  - service (str)     - Fully-qualified name of service
    ///  - service_api (str) - ROSRPC Service URI
    ///  -  caller_api (str)  - XML-RPC URI of caller node
    /// Returns (int, str, int) - (code, statusMessage, ignore)
    XmlRpcValue registerService(const std::string& caller_id, const std::string& service,
      const std::string& caller_api, const std::string& service_api, Connection*);

    XmlRpcValue unregisterService(const std::string& caller_id, const std::string& service,
      const std::string& service_api, Connection*);

    XmlRpcValue getTopicTypes(const std::string& topic, const std::string& caller_id, Connection*);

    /// Retrieve a list of topics that this node publishes.
    /// Parameters
    ///  - caller_id (str) - ROS caller ID.
    /// Returns (int, str, [ [str, str] ]) - (code, statusMessage, topicList)
    ///  topicList is a list of topics published by this node and is of the form
    ///  [ [topic1, topicType1]...[topicN, topicTypeN] ]
    XmlRpcValue getPublications(const std::string& caller_id, Connection*);

    /// Retrieve a list of topics that this node subscribes to
    ///  getSubscriptions(caller_id)
    /// Parameters:
    ///  - caller_id (str) - ROS caller ID.
    /// Returns (int, str, [ [str, str] ]) - (code, statusMessage, topicList)
    /// topicList is a list of topics this node subscribes to and is of the form
    /// [ [topic1, topicType1]...[topicN, topicTypeN] ]
    XmlRpcValue getSubscriptions(const std::string& caller_id, Connection*);

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
    XmlRpcValue requestTopic(const std::string& caller_id, const std::string& topic, const XmlRpcValue& protocols, Connection*);

    /// Callback from master of current publisher list for specified topic.
    /// Parameters:
    /// - caller_id (str) ROS caller ID.
    /// - topic (str) Topic name.
    /// - publishers ([str]) List of current publishers for topic in the form of XMLRPC URIs
    /// Returns (int, str, int) (code, statusMessage, ignore)
    XmlRpcValue publisherUpdate(const std::string& caller_id, const std::string& topic, const XmlRpcValue& publishers, Connection*);

    /// Callback from master with updated value of subscribed parameter.
    /// Parameters
    ///  - caller_id (str) ROS caller ID.
    ///  - parameter_key (str) Parameter name, globally resolved.
    ///  - parameter_value (!XMLRPCLegalValue) New parameter value.
    /// Returns (int, str, int) (code, statusMessage, ignore)
    XmlRpcValue paramUpdate(const std::string& caller_id, const std::string& parameter_key, const XmlRpcValue& value,Connection*);

    /// Returns list of all, publishers, subscribers, and services.
    XmlRpcValue getSystemState(Connection*);

    /// Get list of topics that can be subscribed to. This does not return topics that have no publishers.
    /// See getSystemState() to get more comprehensive list.
    /// Parameters:
    ///  - caller_id (str) - ROS caller ID
    ///  - subgraph (str) - Restrict topic names to match within the specified subgraph.
    ///    Subgraph namespace is resolved relative to the caller's namespace. Use empty string to specify all names.
    /// Returns (int, str, [[str, str],]) - (code, statusMessage, [ [topic1, type1]...[topicN, typeN] ])
    XmlRpcValue getPublishedTopics(const std::string& caller_id, const std::string& subgraph, Connection*);

    /// Register a new publisher to a topic
    XmlRpcValue registerPublisher(const std::string& caller_id, const std::string& topic,
      const std::string& type, const std::string& caller_api, Connection* /*conn*/);

    /// Unregister an existing publisher
    XmlRpcValue unregisterPublisher(const std::string& caller_id, const std::string& topic, const std::string& caller_api,
      Connection* /*conn*/);

    /// Register a new subscriber
    XmlRpcValue registerSubscriber(const std::string& caller_id, const std::string& topic,
      const std::string& type, const std::string& caller_api, Connection* /*conn*/);

    /// Unregister an existing subscriber
    XmlRpcValue unregisterSubscriber(const std::string& caller_id, const std::string& topic,
      const std::string& caller_api, Connection* /*conn*/);

    XmlRpcValue lookupNode(const std::string& topic, const std::string& caller_id, Connection* conn);

    XmlRpcValue getBusStatus(/*[In] [Out]*/const XmlRpcValue& parms, /*[In] [Out]*/XmlRpcValue& result);
    XmlRpcValue getBusInfo(/*[In] [Out]*/const XmlRpcValue& parms, /*[In] [Out]*/XmlRpcValue& result);

    XmlRpcValue getTime(Connection*);

    /// Parameter API

    /// Check whether a parameter exists
    XmlRpcValue hasParam(const std::string& caller_id, const std::string& topic, Connection* /*conn*/);

    /// Set a new parameter
    XmlRpcValue setParam(const std::string& caller_api, const std::string& topic, const XmlRpcValue& value, Connection* /*conn*/);

    /// Retrieve a value for an existing parameter, if it exists.
    XmlRpcValue getParam(const std::string& caller_id, const std::string& topic, Connection*);

    /// Delete a parameter, if it exists
    /// Parameters:
    ///  - caller_id (str) - ROS caller ID
    ///  - key (str) - Parameter name.
    /// Returns (int, str, int) - (code, statusMessage, ignore)
    XmlRpcValue deleteParam(const std::string& caller_d, const std::string& key, Connection*);

    XmlRpcValue getParamNames(const std::string& caller_id, Connection*);

    /// Subscribe to a param value
    XmlRpcValue Param(const XmlRpcValue& parms, XmlRpcValue& result, Connection*);
};

} // namespace miniros

#endif //MINIROS_MASTER_H
