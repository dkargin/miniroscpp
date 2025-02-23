//
// Created by dkargin on 2/9/25.
//

#include <algorithm>

#include <unistd.h>

#include "master_handler.h"

#include <transport/rpc_manager.h>
#include "miniros/transport/network.h"

namespace miniros {
namespace master {
bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.find(prefix) == 0;
}

MasterHandler::MasterHandler(RPCManagerPtr rpcManager)
  :m_rpcManager(rpcManager)
{}

std::list<std::string> MasterHandler::publisher_update_task(
  const std::string& api, const std::string& topic, const std::vector<std::string>& pub_uris)
{
  MINIROS_DEBUG_NAMED("rosmaster", "publisher_update_task");
  RpcValue l;
  l[0] = api;
  l[1] = "";
  for (int i = 0; i < pub_uris.size(); i++) {
    RpcValue ll(pub_uris[i]);
    l[i + 1] = ll;
  }

  RpcValue args;
  args[0] = "master";
  args[1] = topic;
  args[2] = l;

  uint32_t peer_port = 0;
  std::string peer_host;
  if (!miniros::network::splitURI(api, peer_host, peer_port)) {
    MINIROS_ERROR_NAMED("handler", "Failed to splitURI of node \"%s\"", api.c_str());
    return {};
  }

  XmlRpc::XmlRpcClient* client = m_rpcManager->getXMLRPCClient(peer_host, peer_port, api);
  if (!client) {
    MINIROS_ERROR_NAMED("handler", "Failed to create client to notify node \"%s\"", api.c_str());
    return {};
  }

  RpcValue result;
  client->execute("publisherUpdate", args, result);
  // TODO: Do we need any return value?
  return {};
}

void MasterHandler::_shutdown(const std::string& reason)
{
  // TODO:THREADING
  done = true;
}
void MasterHandler::_ready(const std::string& _uri)
{
  uri = _uri;
}

bool MasterHandler::_ok() const
{
  return !done;
}

void MasterHandler::shutdown(const std::string& caller_id, const std::string& msg)
{
}

std::string MasterHandler::getUri(const std::string& caller_id) const
{
  return uri;
}

int MasterHandler::getPid(const std::string& caller_id) const
{
  return getpid();
}

int MasterHandler::deleteParam(const std::string& caller_id, const std::string& key)
{
  MINIROS_DEBUG_NAMED("rosparam", "deleteParam %s from %s", key.c_str(), caller_id.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  auto it = m_parameters.find(fullKey);
  if (it == m_parameters.end())
    return 0;
  m_parameters.erase(fullKey);
  return 1;
  // TODO: Notify.
  /*
  try
  {
      key = Names::resolve_name(key, caller_id);
      param_server.delete_param(key, _notify_param_subscribers);
      return 1;
  }
  catch (KeyNotFoundException e) { return -1; }*/
}

void MasterHandler::setParam(const std::string& caller_id, const std::string& key, const RpcValue& value)
{
  std::stringstream ss;
  ss << key << "=";
  value.write(ss);
  ss << " from " << caller_id;

  MINIROS_DEBUG_NAMED("rosparam", "setParam %s", ss.str().c_str());

  if (value.getType() == RpcValue::TypeStruct) {
    // TODO: Implement
  } else {
    std::string fullKey = miniros::names::resolve(caller_id, key, false);

    auto it = m_parameters.find(fullKey);
    if (it != m_parameters.end()) {
      it->second.value = value;
    } else {
      it = m_parameters.emplace(fullKey, RpcValue()).first;
    }
  }
  // TODO: Notify
}

MasterHandler::RpcValue MasterHandler::getParam(const std::string& caller_id, const std::string& key) const
{
  MINIROS_DEBUG_NAMED("rosparam", "getParam %s from %s", key.c_str(), caller_id.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  auto it = m_parameters.find(fullKey);
  if (it == m_parameters.end())
    return {};
  return it->second.value;
}

std::string MasterHandler::searchParam(const std::string& caller_id, const std::string& key) const
{
  // TODO: Implement
  /*
  std::string search_key = param_server.search_param(caller_id, key);
  return search_key;
  */
  return {};
}

MasterHandler::RpcValue MasterHandler::subscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_DEBUG_NAMED(
    "rosparam", "subscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  // TODO: Implement
  /*
  key = Names::resolve_name(key,caller_id);
  try
  {
      return param_server.subscribe_param(key, caller_id, caller_api);
  }catch(Exception e)
  {
      return {};
  }
  */
  return {};
}

ReturnStruct MasterHandler::unsubscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_DEBUG_NAMED(
    "rosparam", "unsubscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());
  // TODO: Implement
  /*
  key = Names::resolve_name(key, caller_id, {});
  return param_server.unsubscribe_param(key, caller_id, caller_api);
  */
  return {};
}

bool MasterHandler::hasParam(const std::string& caller_id, const std::string& key) const
{
  std::string fullKey = miniros::names::resolve(key, false);
  const auto it = m_parameters.find(fullKey);
  return it != m_parameters.end();
}

std::vector<std::string> MasterHandler::getParamNames(const std::string& caller_id)
{
  std::vector<std::string> names(m_parameters.size());
  for (const auto& p : m_parameters)
    names.push_back(p.first);
  return names;
}

int MasterHandler::_notify_param_subscribers(const std::map<std::string, std::pair<std::string, RpcValue>>& updates)
{
  return 1;
}

void MasterHandler::_param_update_task(const std::string& caller_id, const std::string& caller_api,
  const std::string& param_key, const RpcValue& param_value)
{
}

void MasterHandler::_notify_topic_subscribers(const std::string& topic,
  const std::vector<std::string>& pub_uris,
  const std::vector<std::string>& sub_uris)
{
  if (sub_uris.empty())
    return;

  // This is how it looks in python.
  // pub_uris = self.publishers.get_apis(topic)
  // sub_uris = self.subscribers.get_apis(topic)
  // self._notify_topic_subscribers(topic, pub_uris, sub_uris)
  for (const std::string& node_api: sub_uris) {
    publisher_update_task(node_api, topic, pub_uris);
  }
  /*
  notify(reg_manager.subscribers, publisher_update_task, topic, pub_uris, sub_uris);
  self._notify(self.subscribers, publisher_update_task, topic, pub_uris, sub_uris)

  def _notify(self, registrations, task, key, value, node_apis):
      try:
          for node_api in node_apis:
              # use the api as a marker so that we limit one thread per subscriber
              thread_pool.queue_task(node_api, task, (node_api, key, value))
      except KeyError:
          _logger.warn('subscriber data stale (key [%s], listener [%s]): node API unknown'%(key, s))
  */
}

void MasterHandler::_notify_service_update(const std::string& service, const std::string& service_api)
{
}

ReturnStruct MasterHandler::registerService(const std::string& caller_id, const std::string& service,
  const std::string& caller_api, const std::string& service_api, RpcConnection*)
{
  m_regManager.register_service(service, caller_id, caller_api, service_api);
  return ReturnStruct(1, "Registered [" + caller_id + "] as provider of [" + service + "]", RpcValue(1));
}

std::string MasterHandler::lookupService(const std::string& caller_id, const std::string& service) const
{
  return m_regManager.services.get_service_api(service);
}

ReturnStruct MasterHandler::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api)
{
  return m_regManager.unregister_service(service, caller_id, service_api);
}

ReturnStruct MasterHandler::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& topic_type, const std::string& caller_api, RpcConnection*)
{
  m_regManager.register_subscriber(topic, caller_id, caller_api);

  if (!m_topicTypes.count(topic_type))
    m_topicTypes[topic] = topic_type;

  std::vector<std::string> puburis = m_regManager.publishers.get_apis(topic);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Subscribed to [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;

  rtn.value = RpcValue::Array(puburis.size());
  for (int i = 0; i < puburis.size(); i++) {
    rtn.value[i] = puburis[i];
  }
  return rtn;
}

int MasterHandler::unregisterSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api)
{
  m_regManager.unregister_subscriber(topic, caller_id, caller_api);
  return 1;
}

ReturnStruct MasterHandler::registerPublisher(const std::string& caller_id, const std::string& topic,
  const std::string& topic_type, const std::string& caller_api, RpcConnection*)
{
  MINIROS_DEBUG_NAMED("handler", "registerPublisher topic=%s caller_id=%s caller_api=%s",
    topic.c_str(), caller_id.c_str(), caller_api.c_str());
  if (!m_topicTypes.count(topic_type))
    m_topicTypes[topic] = topic_type;

  m_regManager.register_publisher(topic, caller_id, caller_api);

  std::vector<std::string> pub_uris = m_regManager.publishers.get_apis(topic);
  std::vector<std::string> sub_uris = m_regManager.subscribers.get_apis(topic);
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
  MINIROS_DEBUG_NAMED("handler", "unregisterPublisher topic=%s caller_id=%s caller_api=%s",
    topic.c_str(), caller_id.c_str(), caller_api.c_str());
  auto ret = m_regManager.unregister_publisher(topic, caller_id, caller_api);

  if (true) {
    std::vector<std::string> pub_uris = m_regManager.publishers.get_apis(topic);
    std::vector<std::string> sub_uris = m_regManager.subscribers.get_apis(topic);
    _notify_topic_subscribers(topic, pub_uris, sub_uris);
  }
  return 1;
}

std::string MasterHandler::lookupNode(const std::string& caller_id, const std::string& node_name) const
{
  MINIROS_DEBUG_NAMED("handler", "lookupNode node=%s caller_id=%s", node_name.c_str(), caller_id.c_str());
  auto node = m_regManager.getNode(node_name);
  if (!node)
    return "";
  return node->api;
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

  const auto& e = m_regManager.publishers.map;

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

  result.publishers = m_regManager.publishers.getState();
  result.subscribers = m_regManager.subscribers.getState();
  result.services = m_regManager.services.getState();
  return result;
}

} // namespace master
} // namespace miniros
