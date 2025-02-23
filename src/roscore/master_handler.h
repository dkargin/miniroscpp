//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_MASTER_API_H
#define MINIROS_MASTER_API_H

#include <string>
#include <vector>
#include <list>
#include <functional>

#include "xmlrpcpp/XmlRpcValue.h"

#include "miniros/names.h"
#include "registration_manager.h"

namespace XmlRpc {
class XmlRpcServerConnection;
}
namespace miniros {

namespace master {
bool startsWith(const std::string& str, const std::string& prefix);
bool endsWith(const std::string& str, const std::string& suffix);

/// Stores state of rosmaster and handles part of requests.
class MINIROS_DECL MasterHandler
{
protected:
  std::string uri;
  bool done = false;

public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using RpcConnection = XmlRpc::XmlRpcServerConnection;

  /// Instance of parameter.
  struct Parameter {
    RpcValue value;
    bool save = false;

    Parameter() = default;
    explicit Parameter(const RpcValue& _value) : value(_value) {}
  };

  MasterHandler(RPCManagerPtr rpcManager);

  std::list<std::string> publisher_update_task(const std::string& api, const std::string& topic, const std::vector<std::string>& pub_uris);

  void _shutdown(const std::string& reason="");
  void _ready(const std::string& _uri);

  bool _ok() const;

  void shutdown(const std::string& caller_id, const std::string& msg = "");

  std::string getUri(const std::string& caller_id) const;

  int getPid(const std::string& caller_id) const;

  int deleteParam(const std::string& caller_id, const std::string& key);

  void setParam(const std::string& caller_id, const std::string& key, const RpcValue& value);

  RpcValue getParam(const std::string& caller_id, const std::string& key) const;

  std::string searchParam(const std::string& caller_id, const std::string& key) const;

  RpcValue subscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key);

  ReturnStruct unsubscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string &key);

  bool hasParam(const std::string& caller_id, const std::string& key) const;

  std::vector<std::string> getParamNames(const std::string& caller_id);

  int _notify_param_subscribers(const std::map<std::string, std::pair<std::string, RpcValue>>& updates);

  void _param_update_task(const std::string& caller_id, const std::string& caller_api,
      const std::string& param_key, const RpcValue& param_value);

  void _notify_topic_subscribers(const std::string& topic,
    const std::vector<std::string>& pub_uris,
    const std::vector<std::string>& sub_uris);

  void _notify_service_update(const std::string& service, const std::string& service_api);

  ReturnStruct registerService(const std::string& caller_id, const std::string &service,
    const std::string& caller_api, const std::string& service_api, RpcConnection* conn);

  std::string lookupService(const std::string& caller_id, const std::string& service) const;

  ReturnStruct unregisterService(const std::string& caller_id, const std::string& service, const std::string& service_api);

  ReturnStruct registerSubscriber(const std::string& caller_id, const std::string& topic, const std::string& topic_type,
    const std::string& caller_api, RpcConnection* conn);

  int unregisterSubscriber(const std::string& caller_id, const std::string& topic, const std::string& caller_api);

  ReturnStruct registerPublisher(const std::string& caller_id, const std::string& topic, const std::string& topic_type,
    const std::string& caller_api, RpcConnection* conn);

  int unregisterPublisher(const std::string& caller_id, const std::string& topic, const std::string& caller_api);

  std::string lookupNode(const std::string& caller_id, const std::string& node_name) const;

  /// Get list of piblished topics.
  /// @param caller_id - name of requesting node.
  /// @param subgraph - Optional std::string, only returns topics that start with that name
  std::vector<std::vector<std::string>> getPublishedTopics(const std::string& caller_id, const std::string& subgraph) const;

  std::map<std::string,std::string> getTopicTypes(const std::string& caller_id) const;

  struct SystemState {
    std::map<std::string, std::vector<std::string>> publishers;
    std::map<std::string, std::vector<std::string>> subscribers;
    std::map<std::string, std::vector<std::string>> services;
  };

  SystemState getSystemState(const std::string& caller_id) const;

protected:
  RegistrationManager m_regManager;

  RPCManagerPtr m_rpcManager;

  /// Collection of all parameters.
  std::map<std::string, Parameter> m_parameters;

  /// Maps topicName to type md5.
  std::map<std::string, std::string> m_topicTypes;

};

} // namespace master
} // namespace miniros

#endif //MINIROS_MASTER_API_H
