//
// Created by dkargin on 2/12/25.
//

#include "registration_manager.h"

#include "miniros/console.h"
#include "miniros/names.h"


namespace miniros {
namespace master {

RegistrationManager::RegistrationManager()
  : publishers(Registrations::TOPIC_PUBLICATIONS)
  , subscribers(Registrations::TOPIC_SUBSCRIPTIONS)
  , services(Registrations::SERVICE)
  , param_subscribers(Registrations::PARAM_SUBSCRIPTIONS)
{
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByName(const std::string& name) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  return getNodeByNameUnsafe(name);
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByName(const std::string_view& name) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  return getNodeByNameUnsafe(name);
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByNameUnsafe(const std::string_view& name) const
{
  if (name.empty())
    return {};

  std::string sname{name};
  // TODO: Some requests can start with a full path. Some without "/". Need to address that.
  auto it = m_nodes.find(name);
  if (it != m_nodes.end()) {
    return it->second;
    MINIROS_INFO_NAMED("reg", "getNodeByName(\"%s\")", sname.c_str());
  }
  MINIROS_WARN_NAMED("reg", "getNodeByName(\"%s\") - no such node", sname.c_str());
  return {};
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByAPI(const std::string& api) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  return getNodeByAPIUnsafe(api);
}

std::shared_ptr<NodeRef> RegistrationManager::getNodeByAPIUnsafe(const std::string& api) const
{
  for (auto it = m_nodes.begin(); it != m_nodes.end(); it++) {
    if (it->second->getApi() == api)
      return it->second;
  }
  return {};
}

std::shared_ptr<NodeRef> RegistrationManager::_register(Registrations& r, const std::string& key, const std::string& caller_id, const std::string& caller_api,
  const std::string& service_api)
{
  RegistrationReport report = registerNodeApi(caller_id, caller_api);
  if (!report.node) {
    MINIROS_ERROR("Failed to register NodeRef(node=%s api=%s)", caller_id.c_str(), caller_api.c_str());
    return {};
  }

  report.node->add(r.type(), key);

  if (report.previous) {
    dropRegistrations(report.previous);
  }
  r.registerObj(key, caller_id, caller_api, service_api);
  return report.node;
}

void RegistrationManager::dropRegistrations(const std::shared_ptr<NodeRef>& node)
{
  if (!node)
    return;
  std::string name = node->id();
  MINIROS_INFO("Unregistering everything from superseded node \"%s\" at %s", name.c_str(), node->getApi().c_str());
  publishers.unregister_all(name);
  subscribers.unregister_all(name);
  services.unregister_all(name);
  param_subscribers.unregister_all(name);
}

ReturnStruct RegistrationManager::unregisterObject(Registrations& r, const std::string& key,
  const std::string& caller_id, const std::string& caller_api, const std::string& service_api)
{
  std::shared_ptr<NodeRef> node_ref;
  ReturnStruct ret;

  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (m_nodes.count(caller_id)) {
      node_ref = m_nodes[caller_id];
      ret = r.unregisterObj(key, caller_id, caller_api, service_api);
      if (ret.statusCode == 1) {
        node_ref->remove(r.type(), key);
      }
    } else {
      std::stringstream ss;
      ss << "[" << caller_id << "] is not a registered node";
      ret = ReturnStruct(0, ss.str(), RpcValue(1));
    }
  }

  if (node_ref) {
    if (node_ref->is_empty()) {
      unregisterNode(caller_id);
    }
  }
  return ret;
}

void RegistrationManager::unregisterNode(const std::string& nodeApi)
{
  std::scoped_lock<std::mutex> lock(m_guard);
  auto it = m_nodes.find(nodeApi);
  if (it == m_nodes.end())
    return;
  if (!it->second->is_empty())
    return;

  std::shared_ptr<NodeRef> node_ref = it->second;
  m_nodes.erase(it);
}

std::shared_ptr<NodeRef> RegistrationManager::register_service(const std::string& service, const std::string& caller_id,
  const std::string& caller_api, const std::string& service_api)
{
  return _register(services, service, caller_id, caller_api, service_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_publisher(const std::string& topic, const std::string& topic_type,
  const std::string& caller_id, const std::string& caller_api)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!m_topicTypes.count(topic))
      m_topicTypes[topic] = topic_type;
  }
  return _register(publishers, topic, caller_id, caller_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_subscriber(const std::string& topic, const std::string& topic_type,
  const std::string& caller_id, const std::string& caller_api)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!m_topicTypes.count(topic))
      m_topicTypes[topic] = topic_type;
  }
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

RegistrationManager::RegistrationReport
RegistrationManager::registerNodeApi(const std::string& nodeName, const std::string& nodeApi, int flags)
{
  std::scoped_lock<std::mutex> lock(m_guard);

  RegistrationReport report;

  if (m_nodes.count(nodeName))
    report.node = m_nodes[nodeName];

  if (report.node) {
    if (report.node->getApi() == nodeApi) {
      return report;
    }
    // TODO: Need to check PID of the new node and verify that it has changed.
    NodeRefPtr prevNode;

    report.previous = report.node;
    MINIROS_WARN_NAMED("reg", "New node registered with name=\"%s\" api=%s", nodeName.c_str(), nodeApi.c_str());
    m_nodesToShutdown.insert(report.node);
  }

  report.node.reset(new NodeRef(nodeName, nodeApi));
  report.created = true;

  m_nodes[nodeName] = report.node;

  if (flags & REG_MASTER) {
    report.node->setMaster();
  }
  return report;
}

std::set<std::shared_ptr<NodeRef>> RegistrationManager::pullShutdownNodes()
{
  std::set<std::shared_ptr<NodeRef>> result;
  std::scoped_lock<std::mutex> lock(m_guard);
  std::swap(result, m_nodesToShutdown);
  return result;
}

std::vector<std::shared_ptr<NodeRef>> RegistrationManager::getTopicPublishers(const std::string& topic) const
{
  std::scoped_lock<std::mutex> lock(m_guard);

  std::vector<std::shared_ptr<NodeRef>> result;
  std::vector<std::string> sub_api = publishers.getApis(topic);
  for (const auto& api: sub_api) {
    result.push_back(getNodeByAPIUnsafe(api));
  }
  return result;
}

std::vector<std::shared_ptr<NodeRef>> RegistrationManager::getTopicSubscribers(const std::string& topic) const
{
  std::scoped_lock<std::mutex> lock(m_guard);

  std::vector<std::shared_ptr<NodeRef>> result;
  std::vector<std::string> sub_api = subscribers.getApis(topic);
  for (const auto& api: sub_api) {
    auto node = getNodeByAPIUnsafe(api);
    if (node) {
      result.push_back(node);
    }
  }
  return result;
}

size_t RegistrationManager::iteratePublishers(const std::string_view& topic,
  const NodeIterator& iterator) const
{
  size_t counter = 0;
  publishers.iterateRecords(topic, [&](const Registrations::Record& rec) {
    if (auto node = getNodeByAPIUnsafe(rec.api)) {
      if (!iterator(node))
        return false;
    }
    counter++;
    return true;
  });
  return counter;
}

size_t RegistrationManager::iterateSubscribers(const std::string_view& topic, const NodeIterator& iterator) const
{
  size_t counter = 0;
  subscribers.iterateRecords(topic, [&](const Registrations::Record& rec) {
      if (auto node = getNodeByAPIUnsafe(rec.api)) {
        if (!iterator(node))
          return false;
      }
      counter++;
      return true;
    });
  return counter;
}

std::vector<std::shared_ptr<NodeRef>> RegistrationManager::listAllNodes() const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  std::vector<std::shared_ptr<NodeRef>> result;
  result.reserve(m_nodes.size());
  for (const auto& [name, node]: m_nodes) {
    result.push_back(node);
  }
  return result;
}

std::map<std::string, std::string, std::less<>> RegistrationManager::getTopicTypes(const std::string& caller_id) const
{
  MINIROS_DEBUG_NAMED("reg", "getTopicTypes from %s", caller_id.c_str());
  std::scoped_lock<std::mutex> lock(m_guard);
  return m_topicTypes;
}

std::string RegistrationManager::getTopicType(const std::string_view& name) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  auto it = m_topicTypes.find(name);
  if (it != m_topicTypes.end()) {
    return it->second;
  }
  return {};
}

const std::map<std::string, std::string, std::less<>>& RegistrationManager::getTopicTypesUnsafe(const Lock&) const
{
  return m_topicTypes;
}

std::vector<std::vector<std::string>> RegistrationManager::getPublishedTopics(const std::string& prefix) const
{
  std::vector<std::vector<std::string>> rtn;

  std::scoped_lock<std::mutex> lock(m_guard);
  for (const auto& [Key, Value] : publishers.map) {
    if (names::startsWith(Key, prefix)) {
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

void RegistrationManager::lock() const
{
  m_guard.lock();
}

void RegistrationManager::unlock() const
{
  m_guard.unlock();
}

} // namespace master
} // namespace miniros
