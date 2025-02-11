//
// Created by dkargin on 2/12/25.
//

#include "registration_manager.h"

namespace miniros {
namespace master {

RegistrationManager::RegistrationManager()
    : publishers(Registrations::TOPIC_PUBLICATIONS), subscribers(Registrations::TOPIC_SUBSCRIPTIONS),
      services(Registrations::SERVICE), param_subscribers(Registrations::PARAM_SUBSCRIPTIONS)
{
}

bool RegistrationManager::reverse_lookup(const std::string& caller_api) const
{
  return true;
}

NodeRef RegistrationManager::get_node(const std::string& caller_id) const
{
  auto it = nodes.find(caller_id);
  if (it != nodes.end())
    return it->second;
  return {};
}

void RegistrationManager::_register(Registrations& r, const std::string& key, const std::string& caller_id, const std::string& caller_api,
  const std::string& service_api)
{
  bool changed = false;
  NodeRef node_ref = _register_node_api(caller_id, caller_api, changed);
  node_ref.add(r.type(), key);

  if (changed) {
    publishers.unregister_all(caller_id);
    subscribers.unregister_all(caller_id);
    services.unregister_all(caller_id);
    param_subscribers.unregister_all(caller_id);
  }
  r.registerObj(key, caller_id, caller_api, service_api);
}

ReturnStruct RegistrationManager::_unregister(Registrations& r, const std::string& key,
  const std::string& caller_id, const std::string& caller_api, const std::string& service_api)
{
  ReturnStruct ret;
  if (nodes.count(caller_id)) {
    NodeRef node_ref = nodes[caller_id];
    ret = r.unregisterObj(key, caller_id, caller_api, service_api);
    if (ret.statusCode == 1) {
      node_ref.remove(r.type(), key);
    }
    if (node_ref.is_empty()) {
      nodes.erase(caller_id);
    }
  } else {
    std::stringstream ss;
    ss << "[" << caller_id << "] is not a registered node";
    ret = ReturnStruct(0, ss.str(), RpcValue(1));
  }
  return ret;
}

void RegistrationManager::register_service(const std::string& service, const std::string& caller_id,
  const std::string& caller_api, const std::string& service_api)
{
  _register(services, service, caller_id, caller_api, service_api);
}

void RegistrationManager::register_publisher(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  _register(publishers, topic, caller_id, caller_api);
}

void RegistrationManager::register_subscriber(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  _register(subscribers, topic, caller_id, caller_api);
}

void RegistrationManager::register_param_subscriber(const std::string& param, const std::string& caller_id,
  const std::string& caller_api)
{
  _register(param_subscribers, param, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_service(const std::string& service, const std::string& caller_id,
  const std::string& service_api)
{
  std::string caller_api = "";
  return _unregister(services, service, caller_id, caller_api, service_api);
  // return new ReturnStruct(ret, msg);
}

ReturnStruct RegistrationManager::unregister_subscriber(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return _unregister(subscribers, topic, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_publisher(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return _unregister(publishers, topic, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_param_subscriber(const std::string& param, const std::string& caller_id,
  const std::string& caller_api)
{
  return _unregister(param_subscribers, param, caller_id, caller_api);
}

NodeRef RegistrationManager::_register_node_api(const std::string& caller_id, const std::string& caller_api, bool& rtn)
{
  NodeRef node_ref;
  if (nodes.count(caller_id))
    node_ref = nodes[caller_id];

  std::string bumped_api = "";
  if (!node_ref.is_empty()) {
    if (node_ref.api == caller_api) {
      rtn = false;
      return node_ref;
    } else {
      bumped_api = node_ref.api;
      // thread_pool.queue_task(bumped_api, shutdown_node_task, (bumped_api, caller_id, "new node registered with same
      // name"))
    }
  }

  node_ref = NodeRef(caller_id, caller_api);
  nodes[caller_id] = node_ref;

  rtn = !bumped_api.empty();
  return node_ref;
}

} // namespace master
} // namespace miniros
