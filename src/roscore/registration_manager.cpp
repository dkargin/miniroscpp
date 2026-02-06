//
// Created by dkargin on 2/12/25.
//

#include "registration_manager.h"

#include "miniros/console.h"
#include "miniros/names.h"

#include <cassert>

namespace miniros {
namespace master {

RegistrationManager::RegistrationManager(const std::string& name)
  : publishers(Registrations::TOPIC_PUBLICATIONS)
  , subscribers(Registrations::TOPIC_SUBSCRIPTIONS)
  , services(Registrations::SERVICE)
  , name_(name)
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

void RegistrationManager::setPollSet(PollSet* ps)
{
  assert(ps);
  poll_set_ = ps;
}

std::shared_ptr<NodeRef> RegistrationManager::_register(Registrations& r, const std::string& key, const std::string& nodeName, const std::string& nodeApi,
  const std::string& service_api)
{
  std::string nameError;
  if (!names::validate(key, nameError)) {
    MINIROS_ERROR("_register(node=%s,  key=%s) - invalid key: \"%s\"", nodeName.c_str(), key.c_str(), nameError.c_str());
    return {};
  }

  RegistrationReport report = registerNodeApi(nodeName, nodeApi, 0);
  if (!report.node) {
    MINIROS_ERROR("Failed to register NodeRef(node=%s api=%s)", nodeName.c_str(), nodeApi.c_str());
    return {};
  }

  report.node->add(r.type(), key);

  if (report.previous) {
    dropRegistrations(*report.previous);
  }
  r.registerObj(key, nodeName, nodeApi, service_api);
  return report.node;
}

void RegistrationManager::dropRegistrations(const NodeRef& node)
{
  std::string name = node.id();
  MINIROS_INFO("Unregistering everything from superseded node \"%s\" at %s", name.c_str(), node.getApi().c_str());
  publishers.unregisterAll(name);
  subscribers.unregisterAll(name);
  services.unregisterAll(name);
}

ReturnStruct RegistrationManager::unregisterObject(Registrations& r, const std::string& key,
  const std::string& nodeName, const std::string& nodeApi, const std::string& service_api)
{
  std::shared_ptr<NodeRef> node_ref;
  ReturnStruct ret;

  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (m_nodes.count(nodeName)) {
      node_ref = m_nodes[nodeName];
      ret = r.unregisterObj(key, nodeName, nodeApi, service_api);
      if (ret.statusCode == 1) {
        node_ref->remove(r.type(), key);
      }
    } else {
      std::stringstream ss;
      ss << "[" << nodeName << "] is not a registered node";
      ret = ReturnStruct(0, ss.str(), RpcValue(1));
    }
  }

  if (node_ref && node_ref->is_empty()) {
    unregisterNode(nodeName);
  }
  return ret;
}

void RegistrationManager::unregisterNode(const std::string& nodeName)
{
  MINIROS_INFO("RegistrationManager::unregisterNode(%s)", nodeName.c_str());
  std::scoped_lock<std::mutex> lock(m_guard);

  auto it = m_nodes.find(nodeName);
  if (it == m_nodes.end())
    return;
  if (!it->second->is_empty())
    return;

  std::shared_ptr<NodeRef> node_ref = it->second;
  m_nodes.erase(it);
}

std::shared_ptr<NodeRef> RegistrationManager::register_service(const std::string& service, const std::string& caller_id,
  const std::string& nodeName, const std::string& service_api)
{
  return _register(services, service, caller_id, nodeName, service_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_publisher(const std::string& topic, const std::string& topic_type,
  const std::string& caller_id, const std::string& caller_api)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!topic_types_.count(topic))
      topic_types_[topic] = topic_type;
  }
  return _register(publishers, topic, caller_id, caller_api);
}

std::shared_ptr<NodeRef> RegistrationManager::register_subscriber(const std::string& topic, const std::string& topic_type,
  const std::string& nodeName, const std::string& nodeApi)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!topic_types_.count(topic))
      topic_types_[topic] = topic_type;
  }
  return _register(subscribers, topic, nodeName, nodeApi);
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

RegistrationManager::RegistrationReport
RegistrationManager::registerNodeApi(const std::string& nodeName, const std::string& nodeApi, int flags)
{
  std::scoped_lock<std::mutex> lock(m_guard);

  RegistrationReport report;

  auto it = m_nodes.find(nodeName);
  if (it != m_nodes.end()) {
    report.node = it->second;
    if (report.node->getApi() == nodeApi) {
      return report;
    }

    // TODO: Need to check PID of the new node and verify that it has changed.
    // TODO: Need to check some alternative IP addresses to verify this node is really new.
    NodeRefPtr prevNode;

    report.previous = report.node;
    MINIROS_WARN_NAMED("reg", "New node registered with name=\"%s\" api=%s", nodeName.c_str(), nodeApi.c_str());
    m_nodesToShutdown.insert(report.node);
  }

  report.node.reset(new NodeRef(nodeName, nodeApi));
  report.created = true;

  assert(poll_set_);
  if (Error err = report.node->activateConnection(name_, poll_set_); !err) {
    MINIROS_ERROR("RegistrationManager::registerNodeApi(%s) - failed to activate connection", nodeName.c_str());
  }

  m_nodes[nodeName] = report.node;

  report.node->setNodeFlags(flags);

  return report;
}

std::set<std::shared_ptr<NodeRef>> RegistrationManager::pullShutdownNodes()
{
  std::set<std::shared_ptr<NodeRef>> result;
  std::scoped_lock<std::mutex> lock(m_guard);
  std::swap(result, m_nodesToShutdown);
  return result;
}

std::vector<NodeRefPtr> RegistrationManager::checkDeadNodes()
{
  std::vector<NodeRefPtr> graveyard;
  {
    std::scoped_lock<std::mutex> lock(m_guard);

    for (auto& [key, node]: m_nodes) {
      assert(node);
      if (node && node->getState() == NodeRef::State::Dead) {
        graveyard.push_back(node);
      }
    }
  }

  // Drop registrations for dead nodes.
  for (auto& node: graveyard) {
    dropRegistrations(*node);
  }
  return graveyard;
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
  return topic_types_;
}

std::string RegistrationManager::getTopicType(const std::string_view& name) const
{
  std::scoped_lock<std::mutex> lock(m_guard);
  auto it = topic_types_.find(name);
  if (it != topic_types_.end()) {
    return it->second;
  }
  return {};
}

const std::map<std::string, std::string, std::less<>>& RegistrationManager::getTopicTypesUnsafe(const Lock&) const
{
  return topic_types_;
}

std::vector<std::vector<std::string>> RegistrationManager::getPublishedTopics(const std::string& prefix) const
{
  std::vector<std::vector<std::string>> rtn;

  std::scoped_lock<std::mutex> lock(m_guard);
  for (const auto& [Key, Value] : publishers.map) {
    if (names::startsWith(Key, prefix)) {
      for (const auto& s : Value) {
        auto it = topic_types_.find(Key);
        if (it != topic_types_.end()) {
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
