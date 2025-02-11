//
// Created by dkargin on 2/9/25.
//

#include <algorithm>

#include "master_handler.h"

namespace miniros {

bool startsWith(const std::string& str, const std::string& prefix)
{
  return str.find(prefix) == 0;
}

MasterHandler::MasterHandler()
{
}

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

#ifdef WTF
  XmlRpcValue result = new XmlRpcValue(new XmlRpcValue(), new XmlRpcValue(), new XmlRpcValue(new XmlRpcValue())),
              payload = new XmlRpcValue();

  // WTF is that?
  Ros_CSharp.master.host = api.Replace("http://", "").Replace("/", "").Split(':')[0];
  Ros_CSharp.master.port = int.Parse(api.Replace("http://", "").Replace("/", "").Split(':')[1]);
  Ros_CSharp.master.execute("publisherUpdate", args, result, payload, false);
#endif
  return std::list<std::string>{"http://ERIC:1337"};
}

void MasterHandler::service_update_task(const std::string& api, const std::string& service, const std::string& uri)
{
  RpcValue args;
  args[0] = "master";
  args[1] = service;
  args[2] = uri;
#ifdef WTF
  XmlRpcValue result = new XmlRpcValue(new XmlRpcValue(), new XmlRpcValue(), new XmlRpcValue(new XmlRpcValue())),
              payload = new XmlRpcValue();

  Ros_CSharp.master.host = api.Replace("http://", "").Replace("/", "").Split(':')[0];
  Ros_CSharp.master.port = int.Parse(api.Replace("http://", "").Replace("/", "").Split(':')[1]);
  Ros_CSharp.master.execute("publisherUpdate", args, result, payload, false);
#endif
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

  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  auto it = m_parameters.find(fullKey);
  if (it != m_parameters.end()) {
    it->second.value = value;
  } else {
    it = m_parameters.emplace(fullKey, RpcValue()).first;
  }
  // TODO: Notify
  /*
  key = Names::resolve_name(key,caller_id);
  param_server.set_param(key, value, _notify_param_subscribers);*/
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

void MasterHandler::_notify(Registrations& r,
  std::function<std::vector<std::string>(std::string, std::string, std::vector<std::string>)> task,
  const std::string& key, const std::vector<std::string>& value, const std::vector<std::string>& node_apis)
{
  for (const auto& node_api : node_apis) {
    // if (node_api != null && node_uris.Count > 0)
    //{
    task(node_api, key, value);
    //}
  }
}
int MasterHandler::_notify_param_subscribers(const std::map<std::string, std::pair<std::string, RpcValue>>& updates)
{
  return 1;
}

void MasterHandler::_param_update_task(const std::string& caller_id, const std::string& caller_api,
  const std::string& param_key, const RpcValue& param_value)
{
}

void MasterHandler::_notify_topic_subscribers(
  const std::string& topic, const std::vector<std::string>& pub_uris, const std::vector<std::string>& sub_uris)
{
  // TODO: Implement
  /*
  notify(reg_manager.subscribers, publisher_update_task, topic, pub_uris, sub_uris);
  */
}

void MasterHandler::_notify_service_update(const std::string& service, const std::string& service_api)
{
}

ReturnStruct MasterHandler::registerService(const std::string& caller_id, const std::string& service,
  const std::string& service_api, const std::string& caller_api)
{
  reg_manager.register_service(service, caller_id, caller_api, service_api);
  return ReturnStruct(1, "Registered [" + caller_id + "] as provider of [" + service + "]", RpcValue(1));
}

ReturnStruct MasterHandler::lookupService(const std::string& caller_id, const std::string& service) const
{
  std::string service_url = reg_manager.services.get_service_api(service);

  if (!service_url.empty())
    return ReturnStruct(1, "rosrpc URI: [" + service_url + "]", RpcValue(service_url));

  return ReturnStruct(-1, "No provider");
}

ReturnStruct MasterHandler::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api)
{
  return reg_manager.unregister_service(service, caller_id, service_api);
  // return new ReturnStruct(1, "Registered [" + caller_id + "] as provider of [" + service + "]", new XmlRpcValue(1));
}

ReturnStruct MasterHandler::registerSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& topic_type, const std::string& caller_api)
{
  reg_manager.register_subscriber(topic, caller_id, caller_api);

  if (!topic_types.count(topic_type))
    topic_types[topic] = topic_type;

  std::vector<std::string> puburis = reg_manager.publishers.get_apis(topic);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Subscribed to [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;

  rtn.value[0] = RpcValue();
  for (int i = 0; i < puburis.size(); i++) {
    RpcValue tmp = new RpcValue(puburis[i]);
    rtn.value[i] = tmp;
  }
  return rtn;
}

int MasterHandler::unregisterSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api)
{
  reg_manager.unregister_subscriber(topic, caller_id, caller_api);
  return 1;
}

ReturnStruct MasterHandler::registerPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& topic_type, const std::string& caller_api)
{
  reg_manager.register_publisher(topic, caller_id, caller_api);
  if (!topic_types.count(topic_type))
    topic_types[topic] = topic_type;

  std::vector<std::string> pub_uris = reg_manager.publishers.get_apis(topic);
  std::vector<std::string> sub_uris = reg_manager.subscribers.get_apis(topic);
  _notify_topic_subscribers(topic, pub_uris, sub_uris);

  ReturnStruct rtn;
  std::stringstream ss;
  ss << "Registered [" << caller_id << "] as publisher of [" << topic << "]";
  rtn.statusMessage = ss.str();
  rtn.statusCode = 1;
  // rtn.value = new XmlRpcValue();
  rtn.value[0] = RpcValue();
  for (int i = 0; i < sub_uris.size(); i++) {
    // This looks very strange.
    // XmlRpcValue tmp = new XmlRpcValue(sub_uris[0]);
    rtn.value[i] = sub_uris[i];
  }
  return rtn;
}

int MasterHandler::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api)
{
  reg_manager.unregister_publisher(topic, caller_id, caller_api);
  return 1;
}

std::string MasterHandler::lookupNode(const std::string& caller_id, const std::string& node_name) const
{
  NodeRef node = reg_manager.get_node(caller_id);
  if (node.is_empty())
    return "";
  return node.api;
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

  const auto& e = reg_manager.publishers.map;

  std::vector<std::vector<std::string>> rtn;

  for (const auto& [Key, Value] : e) {
    if (startsWith(Key, prefix)) {
      for (const auto& s : Value) {
        auto it = topic_types.find(Key);
        if (it != topic_types.end()) {
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

  return topic_types;
}

MasterHandler::SystemState MasterHandler::getSystemState(const std::string& caller_id) const
{
  MINIROS_DEBUG_NAMED("handler", "getSystemState from %s", caller_id.c_str());
  SystemState result;

  result.publishers = reg_manager.publishers.getState();
  result.subscribers = reg_manager.subscribers.getState();
  result.services = reg_manager.services.getState();
  return result;
}

} // namespace miniros
