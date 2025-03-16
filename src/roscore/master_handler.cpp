//
// Created by dkargin on 2/9/25.
//

#include <algorithm>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "master_handler.h"

#include "miniros/transport/network.h"
#include "miniros/transport/rpc_manager.h"

#include <xmlrpcpp/XmlRpcServerConnection.h>

namespace miniros {
namespace master {

bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.find(prefix) == 0;
}

MasterHandler::MasterHandler(RPCManagerPtr rpcManager, RegistrationManager* regManager)
  :m_rpcManager(rpcManager), m_regManager(regManager)
{
}

Error MasterHandler::sendToNode(const std::shared_ptr<NodeRef>& nr, const char* method, const RpcValue& arg1, const RpcValue& arg2)
{
  if (!nr)
    return Error::InvalidValue;

  RpcValue args;
  args[0] = "master";
  args[1] = arg1;
  args[2] = arg2;

  uint32_t peer_port = 0;
  std::string peer_host;
  std::string nodeApi = nr->getApi();
  if (!miniros::network::splitURI(nodeApi, peer_host, peer_port)) {
    MINIROS_ERROR_NAMED("handler", "Failed to splitURI of node \"%s\"", nodeApi.c_str());
    return Error::InvalidURI;
  }

  // It is expected that client is in idle state, so we can send request immediately without waiting for a response for
  // some previous request.
  XmlRpc::XmlRpcClient* client = m_rpcManager->getXMLRPCClient(peer_host, peer_port, nodeApi);
  if (!client) {
    MINIROS_ERROR_NAMED("handler", "Failed to create client to notify node \"%s\"", nodeApi.c_str());
    return Error::SystemError;
  }
  if (!client->isReady()) {
    MINIROS_FATAL_NAMED("handler", "RPC client is not in idle state");
    return Error::SystemError;
  }

  RpcValue result;
  if (!client->executeNonBlock(method, args))
    return Error::SystemError;
  return Error::Ok;
}

std::string MasterHandler::getUri(const std::string& caller_id) const
{
  return uri;
}

int MasterHandler::getPid(const std::string& caller_id) const
{
#ifdef _WIN32
  return static_cast<int>(GetCurrentProcessId());
#else
  return getpid();
#endif
}

void MasterHandler::notifyTopicSubscribers(const std::string& topic, const std::vector<std::shared_ptr<NodeRef>>& subscribers)
{
  if (subscribers.empty())
    return;

  // Note that list of URI can be different for different clients because of IP resolution and configuration of the network.
  std::vector<std::shared_ptr<NodeRef>> publishers = m_regManager->getTopicPublishers(topic);
  RpcValue l = RpcValue::Array(publishers.size() + 1);

  for (std::shared_ptr<NodeRef> sub: subscribers) {

    l[0] = sub->getApi();
    l[1] = "";
    for (int i = 0; i < publishers.size(); i++) {
      if (publishers[i]) {
        l[i + 1] = publishers[i]->getResolvedApiFor(m_resolveIp, sub);
      }
    }
    sendToNode(sub, "publisherUpdate", topic, l);
  }
}

ReturnStruct MasterHandler::registerService(const std::string& caller_id, const std::string& service,
    const std::string& service_api, const std::string& caller_api, RpcConnection*)
{
  m_regManager->register_service(service, caller_id, caller_api, service_api);
  return ReturnStruct(1, "Registered [" + caller_id + "] as provider of [" + service + "]", RpcValue(1));
}

std::string MasterHandler::lookupService(const RequesterInfo& requesterInfo, const std::string& service) const
{
  return m_regManager->getServiceUri(requesterInfo, service, m_resolveIp);
}

ReturnStruct MasterHandler::unregisterService(const RequesterInfo& requesterInfo, const std::string& service,
  const std::string& service_api)
{
  return m_regManager->unregister_service(service, requesterInfo.callerId, service_api);
}

ReturnStruct MasterHandler::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& topic_type, const std::string& caller_api, RpcConnection* conn)
{
  std::shared_ptr<NodeRef> ref = m_regManager->register_subscriber(topic, caller_id, caller_api);
  if (!ref)
    return ReturnStruct(0, "Internal error");
  network::NetAddress address = conn->getClientAddress();
  ref->updateDirectAddress(address);

  if (!m_topicTypes.count(topic_type))
    m_topicTypes[topic] = topic_type;

  std::vector<std::shared_ptr<NodeRef>> publishers = m_regManager->getTopicPublishers(topic);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Subscribed to [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;

  rtn.value = RpcValue::Array(publishers.size());
  for (int i = 0; i < publishers.size(); i++) {
    if (publishers[i])
      rtn.value[i] = publishers[i]->getResolvedApiFor(m_resolveIp, ref);
  }
  return rtn;
}

int MasterHandler::unregisterSubscriber(const RequesterInfo& requesterInfo, const std::string& topic)
{
  // Subscriber can be unregistered either by a direct call of actual subscriber,
  // or as a part of cleanup procedure from rosnode/rostopic utility. So we do not need to check whether
  // the topic actually belongs to specified caller_id.
  m_regManager->unregister_subscriber(topic, requesterInfo.callerId, requesterInfo.callerApi);
  return 1;
}

ReturnStruct MasterHandler::registerPublisher(const std::string& caller_id, const std::string& topic,
  const std::string& topic_type, const std::string& caller_api, RpcConnection* conn)
{
  MINIROS_INFO_NAMED("handler", "registerPublisher topic=%s caller_id=%s caller_api=%s",
    topic.c_str(), caller_id.c_str(), caller_api.c_str());
  if (!m_topicTypes.count(topic_type))
    m_topicTypes[topic] = topic_type;

  std::shared_ptr<NodeRef> ref = m_regManager->register_publisher(topic, caller_id, caller_api);
  if (!ref) {
    return ReturnStruct(0, "Internal error");
  }

  network::NetAddress address = conn->getClientAddress();
  ref->updateDirectAddress(address);

  std::vector<std::shared_ptr<NodeRef>> subscribers = m_regManager->getTopicSubscribers(topic);
  notifyTopicSubscribers(topic, subscribers);
  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Registered [" << caller_id << "] as publisher of [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;
  rtn.value = RpcValue::Array(subscribers.size());
  for (int i = 0; i < subscribers.size(); i++) {
    if (!subscribers[i])
      continue;
    rtn.value[i] = subscribers[i]->getResolvedApiFor(m_resolveIp, ref);
  }
  return rtn;
}

int MasterHandler::unregisterPublisher(const RequesterInfo& requesterInfo, const std::string& topic)
{
  MINIROS_INFO_NAMED("handler", "unregisterPublisher topic=%s caller_id=%s caller_api=%s",
    topic.c_str(), requesterInfo.callerId.c_str(), requesterInfo.callerApi.c_str());
  // Publisher can be unregistered either by a direct call of actual subscriber,
  // or as a part of cleanup procedure from rosnode/rostopic utility. So we do not need to check whether
  // the topic actually belongs to specified caller_id.
  auto ret = m_regManager->unregister_publisher(topic, requesterInfo.callerId, requesterInfo.callerApi);

  if (ret.statusCode) {
    std::vector<std::shared_ptr<NodeRef>> subscribers = m_regManager->getTopicSubscribers(topic);
    notifyTopicSubscribers(topic, subscribers);
  }

  return 1;
}

std::string MasterHandler::lookupNode(const RequesterInfo& requesterInfo, const std::string& node_name) const
{
  MINIROS_INFO_NAMED("handler", "lookupNode node=%s caller_id=%s", node_name.c_str(), requesterInfo.callerId.c_str());
  // This is typically a call from "rosnode". So there will be no NodeRef for this caller.
  std::shared_ptr<NodeRef> node = m_regManager->getNodeByName(node_name);
  if (!node)
    return "";
  return node->getResolvedApiFor(m_resolveIp, requesterInfo.clientAddress);
}

std::vector<std::vector<std::string>> MasterHandler::getPublishedTopics(
  const RequesterInfo& requesterInfo, const std::string& subgraph) const
{
  MINIROS_INFO_NAMED("handler", "getPublishedTopics from %s subgraph=%s",
    requesterInfo.callerId.c_str(), subgraph.c_str());

  std::string prefix;
  if (!subgraph.empty() && subgraph.back() != '/')
    prefix = subgraph + "/";
  else
    prefix = subgraph;

  const auto& e = m_regManager->publishers.map;

  std::vector<std::vector<std::string>> rtn;

  for (const auto& [Key, Value] : e) {
    if (startsWith(Key, prefix)) {
      for (const auto& s : Value) {
        auto it = m_topicTypes.find(Key);
        if (it != m_topicTypes.end()) {
          std::vector<std::string> value = {Key, it->second};
          rtn.push_back(value);
        }
      }
    }
  }
  return rtn;
}

std::map<std::string, std::string> MasterHandler::getTopicTypes(const std::string& caller_id) const
{
  MINIROS_INFO_NAMED("handler", "getTopicTypes from %s", caller_id.c_str());

  return m_topicTypes;
}

MasterHandler::SystemState MasterHandler::getSystemState(const RequesterInfo& requesterInfo) const
{
  MINIROS_INFO_NAMED("handler", "getSystemState from %s", requesterInfo.callerId.c_str());
  SystemState result;

  // Each topic is mapped to a list of NodeIds. Node API is not used here, so there is nothing to resolve.
  result.publishers = m_regManager->publishers.getState();
  result.subscribers = m_regManager->subscribers.getState();
  result.services = m_regManager->services.getState();
  return result;
}

void MasterHandler::setResolveNodeIP(bool resolve)
{
  m_resolveIp = resolve;
}

void MasterHandler::update()
{
  auto shutdownNodes = m_regManager->pullShutdownNodes();
  for (std::shared_ptr<NodeRef> nr: shutdownNodes) {
    RpcValue msg;
    std::stringstream ss;
    ss << "[" << nr->id() << "] Reason: new node registered with same name";
    msg = ss.str();
    sendToNode(nr, "shutdown", msg);
  }
}

} // namespace master
} // namespace miniros
