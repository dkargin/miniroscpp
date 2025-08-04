//
// Created by dkargin on 2/9/25.
//

#include <algorithm>
#include <cassert>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <unistd.h>
#endif
#include "master_handler.h"

#include "miniros/transport/network.h"
#include "miniros/transport/rpc_manager.h"
#include "miniros/internal/at_exit.h"

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
  m_resolver.scanAdapters();
}

Error MasterHandler::enqueueNodeCommand(const std::shared_ptr<NodeRef>& nr, const char* method, const RpcValue& arg1, const RpcValue& arg2)
{
  if (!nr)
    return Error::InvalidValue;

  std::scoped_lock<std::mutex> lock(m_guard);
  m_asyncCommands.emplace_back(AsyncCommand{nr, std::string(method), arg1, arg2});
  return Error::Ok;
}

Error MasterHandler::sendToNode(const std::shared_ptr<NodeRef>& nr, const char* method,
  const RpcValue& arg1, const RpcValue& arg2)
{
  if (!nr)
    return Error::InvalidValue;

  RpcValue args;
  args[0] = "master";
  args[1] = arg1;
  args[2] = arg2;


  network::URL url;
  std::string nodeApi = nr->getApi();
  if (!url.fromString(nodeApi, false)) {
    MINIROS_ERROR_NAMED("handler", "Failed to splitURI of node \"%s\"", nodeApi.c_str());
    return Error::InvalidURI;
  }

  // It is expected that client is in idle state, so we can send request immediately without waiting for a response for
  // some previous request.
  XmlRpc::XmlRpcClient* client = m_rpcManager->getXMLRPCClient(url.host, url.port, url.path);
  if (!client) {
    MINIROS_ERROR_NAMED("handler", "Failed to create client to notify node \"%s\"", nodeApi.c_str());
    return Error::SystemError;
  }

  AtExit clientDispose([this, client]() {
    m_rpcManager->releaseXMLRPCClient(client);
  });


  if (!client->isReady()) {
    MINIROS_FATAL_NAMED("handler", "RPC client is not in idle state");
    return Error::SystemError;
  }

  /*
  /// This notification can happen inside RPC callback. So blocking call will not work here at all.
  if (!client->executeNonBlock(method, args)) {
    MINIROS_WARN("Failed to execute node request %s", method);
    return Error::SystemError;
  }*/

  RpcValue result;
  if (!client->execute(method, args, result)) {
    MINIROS_WARN("Failed to execute node request %s", method);
    return Error::SystemError;
  }
  if (result.size() > 2) {
    MINIROS_INFO("Response status=%d, msg=%s", result[0].as<int>(), result[1].as<std::string>().c_str());
  } else {
    MINIROS_WARN("Unexpected response");
  }
  return Error::Ok;
}

std::string MasterHandler::getUri(const std::string& caller_id) const
{
  return uri;
}

void MasterHandler::notifyTopicSubscribers(const std::string& topic, const std::vector<std::shared_ptr<NodeRef>>& subscribers)
{
  if (subscribers.empty())
    return;

  // Note that list of URI can be different for different clients because of IP resolution and configuration of the network.
  std::vector<std::shared_ptr<NodeRef>> publishers = m_regManager->getTopicPublishers(topic);
  RpcValue l = RpcValue::Array(0);
  for (std::shared_ptr<NodeRef> sub: subscribers) {
    if (sub) {
      int j = 0;
      for (int i = 0; i < publishers.size(); i++) {
        if (publishers[i]) {
          network::URL url = m_resolver.resolveAddressFor(publishers[i], sub);
          l[j++] = url.str();
        }
      }
      Error err = this->enqueueNodeCommand(sub, "publisherUpdate", topic, l);
      if (err != Error::Ok) {
        MINIROS_WARN("Failed send publisherUpdate(%s) to node \"%s\"", topic.c_str(), sub->id().c_str());
      }
    }
  }
}

ReturnStruct MasterHandler::registerService(const RequesterInfo& requesterInfo, const std::string& service,
    const std::string& service_api)
{
  std::shared_ptr<NodeRef> ref = m_regManager->register_service(service, requesterInfo.callerId, requesterInfo.callerApi, service_api);
  if (!ref)
    return ReturnStruct(0, "Internal error");

  if (auto hostInfo = m_resolver.updateHost(requesterInfo))
    ref->updateHost(hostInfo);

  return ReturnStruct(1, "Registered [" + requesterInfo.callerId + "] as provider of [" + service + "]", RpcValue(1));
}

std::string MasterHandler::lookupService(const RequesterInfo& requesterInfo, const std::string& service) const
{
  // service_api looks like "rosrpc://hostname:port". It differs from ClientAPI URL.
  std::string service_api = m_regManager->services.get_service_api(service);
  std::shared_ptr<NodeRef> node = m_regManager->getNodeByAPI(service_api);
  if (node && requesterInfo.clientAddress.valid()) {
    network::URL url = m_resolver.resolveAddressFor(node, requesterInfo.clientAddress, requesterInfo.localAddress);
    return url.str();
  }
  return service_api;
}

ReturnStruct MasterHandler::unregisterService(const RequesterInfo& requesterInfo, const std::string& service,
  const std::string& service_api)
{
  return m_regManager->unregister_service(service, requesterInfo.callerId, service_api);
}

ReturnStruct MasterHandler::registerSubscriber(const RequesterInfo& requesterInfo, const std::string& topic,
  const std::string& topic_type)
{
  std::shared_ptr<NodeRef> ref = m_regManager->register_subscriber(topic, requesterInfo.callerId, requesterInfo.callerApi);
  if (!ref)
    return ReturnStruct(0, "Internal error");
  if (topic.empty() || topic_type.empty()) {
    ReturnStruct(-1, "Request error");
  }

  MINIROS_INFO_NAMED("handler", "registerSubscriber(\"%s\") caller_id=%s caller_api=%s, type=%s",
    topic.c_str(), requesterInfo.callerId.c_str(), requesterInfo.callerApi.c_str(), topic_type.c_str());

  if (auto hostInfo = m_resolver.updateHost(requesterInfo))
    ref->updateHost(hostInfo);

  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!m_topicTypes.count(topic))
      m_topicTypes[topic] = topic_type;
  }

  std::vector<std::shared_ptr<NodeRef>> publishers = m_regManager->getTopicPublishers(topic);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Subscribed to [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;

  rtn.value = RpcValue::Array(publishers.size());
  for (int i = 0; i < publishers.size(); i++) {
    if (publishers[i]) {
      std::string strUrl;
      if (publishers[i] != ref) {
        network::URL url = m_resolver.resolveAddressFor(publishers[i], requesterInfo.clientAddress, requesterInfo.localAddress);
        strUrl = url.str();
      } else {
        // The publisher is the same as the subscriber. Do not resolve IP address in this case.
        strUrl = publishers[i]->getApi();
      }
      MINIROS_INFO_NAMED("handler", "registerSubscriber(\"%s\") - pub=%s", topic.c_str(), strUrl.c_str());
      rtn.value[i] = strUrl;
    }
  }
  return rtn;
}

ReturnStruct MasterHandler::unregisterSubscriber(const RequesterInfo& requesterInfo, const std::string& topic)
{
  // Subscriber can be unregistered either by a direct call of actual subscriber,
  // or as a part of cleanup procedure from rosnode/rostopic utility. So we do not need to check whether
  // the topic actually belongs to specified caller_id.
  return m_regManager->unregister_subscriber(topic, requesterInfo.callerId, requesterInfo.callerApi);
}

ReturnStruct MasterHandler::registerPublisher(const RequesterInfo& requesterInfo, const std::string& topic,
  const std::string& topic_type)
{
  MINIROS_INFO_NAMED("handler", "registerPublisher(\"%s\") caller_id=%s caller_api=%s type=%s",
    topic.c_str(), requesterInfo.callerId.c_str(), requesterInfo.callerApi.c_str(), topic_type.c_str());

  if (topic.empty() || topic_type.empty()) {
    return ReturnStruct(-1, "Request error");
  }

  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!m_topicTypes.count(topic))
      m_topicTypes[topic] = topic_type;
  }

  std::shared_ptr<NodeRef> ref = m_regManager->register_publisher(topic, requesterInfo.callerId, requesterInfo.callerApi);
  if (!ref) {
    return ReturnStruct(0, "Internal error");
  }

  if (auto hostInfo = m_resolver.updateHost(requesterInfo))
    ref->updateHost(hostInfo);

  std::vector<std::shared_ptr<NodeRef>> subscribers = m_regManager->getTopicSubscribers(topic);
  notifyTopicSubscribers(topic, subscribers);
  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Registered [" << requesterInfo.callerId << "] as publisher of [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;
  rtn.value = RpcValue::Array(subscribers.size());
  for (int i = 0; i < subscribers.size(); i++) {
    if (!subscribers[i])
      continue;
    network::URL url = m_resolver.resolveAddressFor(subscribers[i], requesterInfo.clientAddress, requesterInfo.localAddress);
    std::string strUrl = url.str();
    MINIROS_INFO_NAMED("handler", "registerPublisher(\"%s\") - sub=%s", topic.c_str(), strUrl.c_str());
    rtn.value[i] = strUrl;
  }
  return rtn;
}

ReturnStruct MasterHandler::unregisterPublisher(const RequesterInfo& requesterInfo, const std::string& topic)
{
  MINIROS_INFO_NAMED("handler", "unregisterPublisher(\"%s\") caller_id=%s caller_api=%s",
    topic.c_str(), requesterInfo.callerId.c_str(), requesterInfo.callerApi.c_str());
  // Publisher can be unregistered either by a direct call of actual subscriber,
  // or as a part of cleanup procedure from rosnode/rostopic utility. So we do not need to check whether
  // the topic actually belongs to specified caller_id.
  auto ret = m_regManager->unregister_publisher(topic, requesterInfo.callerId, requesterInfo.callerApi);

  if (ret.statusCode) {
    std::vector<std::shared_ptr<NodeRef>> subscribers = m_regManager->getTopicSubscribers(topic);
    notifyTopicSubscribers(topic, subscribers);
  }

  return ret;
}

std::string MasterHandler::lookupNode(const RequesterInfo& requesterInfo, const std::string& node_name) const
{
  MINIROS_INFO_NAMED("handler", "lookupNode node=%s caller_id=%s", node_name.c_str(), requesterInfo.callerId.c_str());
  // This is typically a call from "rosnode". So there will be no NodeRef for this caller.
  std::shared_ptr<NodeRef> node = m_regManager->getNodeByName(node_name);
  if (!node)
    return "";
  network::URL url = m_resolver.resolveAddressFor(node, requesterInfo.clientAddress, requesterInfo.localAddress);
  return url.str();
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

  std::scoped_lock<std::mutex> lock(m_guard);
  for (const auto& [Key, Value] : e) {
    if (startsWith(Key, prefix)) {
      for (const auto& s : Value) {
        auto it = m_topicTypes.find(Key);
        if (it != m_topicTypes.end()) {
          std::vector<std::string> value = {Key, it->second};
          rtn.push_back(value);
          break;
        }
      }
    }
  }
  return rtn;
}

std::map<std::string, std::string> MasterHandler::getTopicTypes(const std::string& caller_id) const
{
  MINIROS_INFO_NAMED("handler", "getTopicTypes from %s", caller_id.c_str());
  std::scoped_lock<std::mutex> lock(m_guard);
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
  m_resolver.setResolveIp(resolve);
}

void MasterHandler::update()
{
  auto shutdownNodes = m_regManager->pullShutdownNodes();

  std::vector<AsyncCommand> commands;

  {
    std::unique_lock<std::mutex> lock(m_guard);
    std::swap(commands, m_asyncCommands);
  }

  for (const auto& command: commands) {
    if (shutdownNodes.count(command.node)) {
      continue;
    }
    sendToNode(command.node, command.command.c_str(), command.arg1, command.arg2);
  }

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
