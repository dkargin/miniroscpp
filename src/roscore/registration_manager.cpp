//
// Created by dkargin on 2/12/25.
//

#include "registration_manager.h"

#include "miniros/console.h"

namespace miniros {
namespace master {

RegistrationManager::RegistrationManager()
  : publishers(Registrations::TOPIC_PUBLICATIONS)
  , subscribers(Registrations::TOPIC_SUBSCRIPTIONS)
  , services(Registrations::SERVICE)
  , param_subscribers(Registrations::PARAM_SUBSCRIPTIONS)
{
}

bool RegistrationManager::reverse_lookup(const std::string& caller_api) const
{
  return true;
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByName(const std::string& node) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  auto it = m_nodes.find(node);
  if (it != m_nodes.end())
    return it->second;
  return {};
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByAPI(const std::string& api) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  for (auto it = m_nodes.begin(); it != m_nodes.end(); it++) {
    if (it->second->getApi() == api)
      return it->second;
  }
  return {};
}

std::shared_ptr<NodeRef> RegistrationManager::_register(Registrations& r, const std::string& key, const std::string& caller_id, const std::string& caller_api,
  const std::string& service_api)
{
  bool changed = false;
  std::shared_ptr<NodeRef> node_ref = registerNodeApi(caller_id, caller_api, changed);
  if (!node_ref)
    return {};

  node_ref->add(r.type(), key);

  if (changed) {
    publishers.unregister_all(caller_id);
    subscribers.unregister_all(caller_id);
    services.unregister_all(caller_id);
    param_subscribers.unregister_all(caller_id);
  }
  r.registerObj(key, caller_id, caller_api, service_api);
  return node_ref;
}

ReturnStruct RegistrationManager::unregisterObject(Registrations& r, const std::string& key,
  const std::string& caller_id, const std::string& caller_api, const std::string& service_api)
{
  std::scoped_lock<std::mutex> lock(m_guard);

  ReturnStruct ret;
  if (m_nodes.count(caller_id)) {
    auto node_ref = m_nodes[caller_id];
    ret = r.unregisterObj(key, caller_id, caller_api, service_api);
    if (ret.statusCode == 1) {
      node_ref->remove(r.type(), key);
    }
    if (node_ref->is_empty()) {
      m_nodes.erase(caller_id);
    }
  } else {
    std::stringstream ss;
    ss << "[" << caller_id << "] is not a registered node";
    ret = ReturnStruct(0, ss.str(), RpcValue(1));
  }
  return ret;
}

std::shared_ptr<NodeRef> RegistrationManager::register_service(const std::string& service, const std::string& caller_id,
  const std::string& caller_api, const std::string& service_api)
{
  return _register(services, service, caller_id, caller_api, service_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_publisher(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return _register(publishers, topic, caller_id, caller_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_subscriber(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return _register(subscribers, topic, caller_id, caller_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_param_subscriber(const std::string& param, const std::string& caller_id,
  const std::string& caller_api)
{
  return _register(param_subscribers, param, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_service(const std::string& service, const std::string& caller_id,
  const std::string& service_api)
{
  std::string caller_api = "";
  return unregisterObject(services, service, caller_id, caller_api, service_api);
}

ReturnStruct RegistrationManager::unregister_subscriber(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return unregisterObject(subscribers, topic, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_publisher(const std::string& topic, const std::string& caller_id,
  const std::string& caller_api)
{
  return unregisterObject(publishers, topic, caller_id, caller_api);
}

ReturnStruct RegistrationManager::unregister_param_subscriber(const std::string& param, const std::string& caller_id,
  const std::string& caller_api)
{
  return unregisterObject(param_subscribers, param, caller_id, caller_api);
}

std::shared_ptr<NodeRef> RegistrationManager::registerNodeApi(const std::string& caller_id, const std::string& caller_api, bool& rtn)
{
  std::scoped_lock<std::mutex> lock(m_guard);

  std::shared_ptr<NodeRef> node_ref;
  if (m_nodes.count(caller_id))
    node_ref = m_nodes[caller_id];

  std::string bumped_api = "";
  if (node_ref) {
    if (node_ref->getApi() == caller_api) {
      rtn = false;
      return node_ref;
    } else {
      bumped_api = node_ref->getApi();
      MINIROS_WARN_NAMED("reg", "New node registered with name=\"%s\" api=%s", caller_id.c_str(), caller_api.c_str());
      // TODO: Send signal to make this node shut down.
      // thread_pool.queue_task(bumped_api, shutdown_node_task, (bumped_api, caller_id, "new node registered with same
      // name"))
    }
  }

  node_ref.reset(new NodeRef(caller_id, caller_api));
  m_nodes[caller_id] = node_ref;

  rtn = !bumped_api.empty();
  return node_ref;
}

} // namespace master
} // namespace miniros
