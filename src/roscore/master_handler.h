//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_MASTER_API_H
#define MINIROS_MASTER_API_H

#include <string>
#include <vector>
#include <mutex>

#include "xmlrpcpp/XmlRpcValue.h"
#include "miniros/names.h"

#include "registration_manager.h"
#include "parameter_storage.h"

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

  MasterHandler(RPCManagerPtr rpcManager, RegistrationManager* regManager);

  std::vector<std::string> publisher_update_task(const std::string& api, const std::string& topic, const std::vector<std::string>& pub_uris);

  void _shutdown(const std::string& reason="");
  void _ready(const std::string& _uri);

  std::string getUri(const std::string& caller_id) const;

  int getPid(const std::string& caller_id) const;

  void _notify_topic_subscribers(const std::string& topic,
    const std::vector<std::string>& pub_uris,
    const std::vector<std::string>& sub_uris);

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
  RegistrationManager* m_regManager;

  RPCManagerPtr m_rpcManager;

  /// Maps topicName to type md5.
  std::map<std::string, std::string> m_topicTypes;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_MASTER_API_H
