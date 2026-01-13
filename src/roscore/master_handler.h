//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_MASTER_API_H
#define MINIROS_MASTER_API_H

#include <string>
#include <vector>

#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "miniros/names.h"

#include "registration_manager.h"
#include "parameter_storage.h"
#include "requester_info.h"
#include "resolver.h"

namespace XmlRpc {
class XmlRpcServerConnection;
}
namespace miniros {

namespace master {

/// Stores state of rosmaster and handles part of requests.
class MINIROS_DECL MasterHandler
{
public:
  using RpcValue = XmlRpc::XmlRpcValue;
  using RpcConnection = XmlRpc::XmlRpcServerConnection;

  MasterHandler(RPCManagerPtr rpcManager, RegistrationManager* regManager, AddressResolver* resolver);

  /// Sends immediate command/update to a node.
  /// This is blocking call. It will wait until response is obtained.
  Error sendToNode(const std::shared_ptr<NodeRef>& nr, const char* method,
    const RpcValue& arg1, const RpcValue& arg2 = {});

  void notifyTopicSubscribers(const std::string& topic, const std::vector<std::shared_ptr<NodeRef>>& subscribers);

  ReturnStruct registerService(const RequesterInfo& requesterInfo, const std::string& service, const std::string& service_api);

  std::string lookupService(const RequesterInfo& requesterInfo, const std::string& service) const;

  ReturnStruct unregisterService(const RequesterInfo& requesterInfo, const std::string& service, const std::string& service_api);

  ReturnStruct registerSubscriber(const RequesterInfo& requesterInfo, const std::string& topic, const std::string& topic_type);

  ReturnStruct unregisterSubscriber(const RequesterInfo& requesterInfo, const std::string& topic);

  ReturnStruct registerPublisher(const RequesterInfo& requesterInfo, const std::string& topic, const std::string& topic_type);

  ReturnStruct unregisterPublisher(const RequesterInfo& requesterInfo, const std::string& topic);

  std::string lookupNode(const RequesterInfo& requesterInfo, const std::string& node_name) const;

  /// Get list of published topics.
  /// @param requesterInfo - information about requester.
  /// @param subgraph - Optional std::string, only returns topics that start with that name
  std::vector<std::vector<std::string>> getPublishedTopics(const RequesterInfo& requesterInfo, const std::string& subgraph) const;

  struct SystemState {
    std::map<std::string, std::vector<std::string>> publishers;
    std::map<std::string, std::vector<std::string>> subscribers;
    std::map<std::string, std::vector<std::string>> services;
  };

  SystemState getSystemState(const RequesterInfo& requesterInfo) const;

  /// Updates internal tasks.
  /// It is called from the main thread, out of any RPC handlers.
  void update();

protected:
  AddressResolver* m_resolver;
  RegistrationManager* m_regManager;

  RPCManagerPtr m_rpcManager;

  /// Internal guard for m_topicTypes.
  mutable std::mutex m_guard;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_MASTER_API_H
