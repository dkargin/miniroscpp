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

  XmlRpc::XmlRpcClient* client = m_rpcManager->getXMLRPCClient(peer_host, peer_port, nodeApi);
  if (!client) {
    MINIROS_ERROR_NAMED("handler", "Failed to create client to notify node \"%s\"", nodeApi.c_str());
    return Error::SystemError;
  }

  RpcValue result;
  if (!client->execute(method, args, result))
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

void MasterHandler::_notify_topic_subscribers(const std::string& topic,
  const std::vector<std::string>& pub_uris,
  const std::vector<std::string>& sub_uris)
{
  if (sub_uris.empty())
    return;

  for (const std::string& node_api: sub_uris) {
    std::shared_ptr<NodeRef> nr = m_regManager->getNodeByAPI(node_api);

    RpcValue l;
    l[0] = nr->getApi();
    l[1] = "";
    for (int i = 0; i < pub_uris.size(); i++) {
      RpcValue ll(pub_uris[i]);
      l[i + 1] = ll;
    }

    sendToNode(nr, "publisherUpdate", topic, l);
  }
}

ReturnStruct MasterHandler::registerService(const std::string& caller_id, const std::string& service,
    const std::string& service_api, const std::string& caller_api, RpcConnection*)
{
  m_regManager->register_service(service, caller_id, caller_api, service_api);
  return ReturnStruct(1, "Registered [" + caller_id + "] as provider of [" + service + "]", RpcValue(1));
}

std::string MasterHandler::lookupService(const std::string& caller_id, const std::string& service) const
{
  return m_regManager->services.get_service_api(service);
}

ReturnStruct MasterHandler::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api)
{
  return m_regManager->unregister_service(service, caller_id, service_api);
}

ReturnStruct MasterHandler::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& topic_type, const std::string& caller_api, RpcConnection* conn)
{
  std::shared_ptr<NodeRef> ref = m_regManager->register_subscriber(topic, caller_id, caller_api);
  if (ref) {
    network::NetAddress address = conn->getClientAddress();
    ref->updateDirectAddress(address);
  }

  if (!m_topicTypes.count(topic_type))
    m_topicTypes[topic] = topic_type;

  std::vector<std::string> pubUris = m_regManager->publishers.get_apis(topic);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Subscribed to [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;

  rtn.value = RpcValue::Array(pubUris.size());
  for (int i = 0; i < pubUris.size(); i++) {
    rtn.value[i] = pubUris[i];
  }
  return rtn;
}

int MasterHandler::unregisterSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api)
{
  m_regManager->unregister_subscriber(topic, caller_id, caller_api);
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
  if (ref) {
    network::NetAddress address = conn->getClientAddress();
    ref->updateDirectAddress(address);
  }

  std::vector<std::string> pub_uris = m_regManager->publishers.get_apis(topic);
  std::vector<std::string> sub_uris = m_regManager->subscribers.get_apis(topic);
  _notify_topic_subscribers(topic, pub_uris, sub_uris);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Registered [" << caller_id << "] as publisher of [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;
  rtn.value = RpcValue::Array(sub_uris.size());
  for (int i = 0; i < sub_uris.size(); i++) {
    // This looks very strange.
    rtn.value[i] = sub_uris[i];
  }
  return rtn;
}

int MasterHandler::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api)
{
  MINIROS_INFO_NAMED("handler", "unregisterPublisher topic=%s caller_id=%s caller_api=%s",
    topic.c_str(), caller_id.c_str(), caller_api.c_str());
  auto ret = m_regManager->unregister_publisher(topic, caller_id, caller_api);

  if (true) {
    std::vector<std::string> pub_uris = m_regManager->publishers.get_apis(topic);
    std::vector<std::string> sub_uris = m_regManager->subscribers.get_apis(topic);
    _notify_topic_subscribers(topic, pub_uris, sub_uris);
  }
  return 1;
}

std::string MasterHandler::lookupNode(const std::string& caller_id, const std::string& node_name) const
{
  MINIROS_INFO_NAMED("handler", "lookupNode node=%s caller_id=%s", node_name.c_str(), caller_id.c_str());
  std::shared_ptr<NodeRef> node = m_regManager->getNodeByName(node_name);
  if (!node)
    return "";
  return node->getApi();
}

/// <param name="subgraph">Optional std::string, only returns topics that start with that name</param>
std::vector<std::vector<std::string>> MasterHandler::getPublishedTopics(
  const std::string& caller_id, const std::string& subgraph) const
{
  MINIROS_DEBUG_NAMED("handler", "getPublishedTopics from %s subgraph=%s",
    caller_id.c_str(), subgraph.c_str());

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
  MINIROS_DEBUG_NAMED("handler", "getTopicTypes from %s", caller_id.c_str());

  return m_topicTypes;
}

MasterHandler::SystemState MasterHandler::getSystemState(const std::string& caller_id) const
{
  MINIROS_DEBUG_NAMED("handler", "getSystemState from %s", caller_id.c_str());
  SystemState result;

  result.publishers = m_regManager->publishers.getState();
  result.subscribers = m_regManager->subscribers.getState();
  result.services = m_regManager->services.getState();
  return result;
}

} // namespace master
} // namespace miniros
