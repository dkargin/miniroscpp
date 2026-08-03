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
  const std::string& service_api, int flags)
{
  std::string nameError;
  if (!names::validate(key, nameError)) {
    MINIROS_ERROR("_register(node=%s,  key=%s) - invalid key: \"%s\"", nodeName.c_str(), key.c_str(), nameError.c_str());
    return {};
  }

  RegistrationReport report = registerNodeApi(nodeName, nodeApi, flags);
  if (!report.node) {
    MINIROS_ERROR("Failed to register NodeRef(node=%s api=%s)", nodeName.c_str(), nodeApi.c_str());
    return {};
  }

  report.node->add(r.type(), key);

  if (report.previous && report.previous != report.node) {
    // Clears stale (topic,name,api) rows for this node name. registerObj below
    // re-adds the key being registered; the new process re-advertises the rest.
    dropRegistrations(*report.previous);
  }
  r.registerObj(key, nodeName, nodeApi, service_api);
  return report.node;
}

void RegistrationManager::dropRegistrations(const NodeRef& node)
{
  std::string name = node.id();
  MINIROS_INFO("Unregistering everything from node \"%s\" at %s", name.c_str(), node.getApi().c_str());
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

  return ret;
}

bool RegistrationManager::unregisterNode(const std::shared_ptr<NodeRef>& node)
{
  assert(node);
  if (!node)
    return false;
  MINIROS_INFO("RegistrationManager::unregisterNode(%s)", node->id().c_str());
  std::scoped_lock<std::mutex> lock(m_guard);

  auto it = m_nodes.find(node->id());
  if (it == m_nodes.end())
    return false;
  if (!it->second->isEmpty())
    return false;
  assert(it->second == node);
  m_nodes.erase(it);
  return true;
}

std::shared_ptr<NodeRef> RegistrationManager::register_service(const std::string& service, const std::string& caller_id,
  const std::string& nodeName, const std::string& service_api, int flags)
{
  return _register(services, service, caller_id, nodeName, service_api, flags);
}

std::shared_ptr<NodeRef> RegistrationManager::register_publisher(const std::string& topic, const std::string& topic_type,
  const std::string& caller_id, const std::string& caller_api, int flags)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!topic_types_.count(topic))
      topic_types_[topic] = topic_type;
  }
  return _register(publishers, topic, caller_id, caller_api, "", flags);
}

std::shared_ptr<NodeRef> RegistrationManager::register_subscriber(const std::string& topic, const std::string& topic_type,
  const std::string& nodeName, const std::string& nodeApi, int flags)
{
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    if (!topic_types_.count(topic))
      topic_types_[topic] = topic_type;
  }
  return _register(subscribers, topic, nodeName, nodeApi, "", flags);
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
    const bool sameApi = report.node->getApi() == nodeApi;
    const bool dead = report.node->getState() == NodeRef::State::Dead;

    // Reuse a live node with the same API. Dead nodes must be replaced: multimaster
    // snapshot apply dropMultimasterPeer()'s markDead() then re-registers the same
    // name/API; keeping the Dead object leaves pubs on a GC victim.
    if (sameApi && !dead) {
      if (flags & NodeRef::NODE_LOCAL)
        report.node->setLocal();
      report.node->setNodeFlags(flags);
      return report;
    }

    // A surviving node may re-advertise while MasterCache is still restoring it,
    // often with a differently spelled URI (hostname vs IP). Superseding would
    // abort restore (drop getPublications) and drop the only live peer.
    if (!dead && report.node->isRestoreInProgress()) {
      MINIROS_WARN_NAMED("reg",
        "Keeping restoring node \"%s\" (cached api=%s, live api=%s)",
        nodeName.c_str(), report.node->getApi().c_str(), nodeApi.c_str());
      if (flags & NodeRef::NODE_LOCAL)
        report.node->setLocal();
      report.node->setNodeFlags(flags);
      return report;
    }

    // Local/in-process master identity must not be displaced by a peer's
    // identically named node (classic multimaster /miniroscore collision).
    const int existingFlags = report.node->getNodeFlags();
    if (!dead && (existingFlags & (NodeRef::NODE_LOCAL | NodeRef::NODE_MASTER))) {
      MINIROS_WARN_NAMED("reg",
        "Refusing to supersede protected node \"%s\" (api=%s flags=0x%x) with api=%s",
        nodeName.c_str(), report.node->getApi().c_str(), existingFlags, nodeApi.c_str());
      report.node.reset();
      return report;
    }

    // TODO: Need to check PID of the new node and verify that it has changed.
    // TODO: Need to check some alternative IP addresses to verify this node is really new.
    report.previous = report.node;
    if (!dead) {
      MINIROS_WARN_NAMED("reg", "New node registered with name=\"%s\" api=%s", nodeName.c_str(), nodeApi.c_str());
    }
    m_nodesToShutdown.insert(report.node);
  }

  report.node.reset(new NodeRef(nodeName, nodeApi));
  report.created = true;

  // Peer masters are not Slave API endpoints — do not open HttpClient / getPid.
  // Foreign mirrors are owned by the peer master; local nodes connect to them
  // directly via TCPROS. Probing foreign Slave APIs from this master races with
  // snapshot replace and fails on filtered cross-host XML-RPC.
  // Local master (NODE_LOCAL|NODE_MASTER) still gets a client for in-process use.
  const bool skipHttp =
    (flags & NodeRef::NODE_FOREIGN) != 0 ||
    ((flags & NodeRef::NODE_MASTER) != 0 && (flags & NodeRef::NODE_LOCAL) == 0);
  if (!skipHttp) {
    assert(poll_set_);
    if (Error err = report.node->activateConnection(name_, poll_set_); !err) {
      MINIROS_ERROR("RegistrationManager::registerNodeApi(%s) - failed to activate connection", nodeName.c_str());
    }
  }

  // Apply NODE_LOCAL after activateConnection so its isLocal() early-out does not skip the client.
  if (flags & NodeRef::NODE_LOCAL)
    report.node->setLocal();
  report.node->setNodeFlags(flags);

  m_nodes[nodeName] = report.node;

  return report;
}

std::set<std::shared_ptr<NodeRef>> RegistrationManager::pullShutdownNodes()
{
  std::set<std::shared_ptr<NodeRef>> result;
  std::scoped_lock<std::mutex> lock(m_guard);
  std::swap(result, m_nodesToShutdown);
  return result;
}

void RegistrationManager::scheduleShutdown(const std::shared_ptr<NodeRef>& node)
{
  assert(node);
  if (!node)
    return;
  std::scoped_lock<std::mutex> lock(m_guard);
  m_nodesToShutdown.insert(node);
}

void RegistrationManager::scheduleDeadNodesForShutdown()
{
  std::vector<std::shared_ptr<NodeRef>> nodes;
  {
    std::scoped_lock<std::mutex> lock(m_guard);
    nodes.reserve(m_nodes.size());
    for (const auto& [key, node] : m_nodes) {
      assert(node);
      nodes.push_back(node);
    }
  }

  std::vector<std::shared_ptr<NodeRef>> dead;
  for (const auto& node : nodes) {
    if (node->getState() == NodeRef::State::Dead)
      dead.push_back(node);
  }
  if (dead.empty())
    return;

  std::scoped_lock<std::mutex> lock(m_guard);
  for (const auto& node : dead)
    m_nodesToShutdown.insert(node);
}

std::vector<NodeRefPtr> RegistrationManager::checkNodesForRemoval()
{
  std::vector<NodeRefPtr> graveyard;
  {
    std::scoped_lock<std::mutex> lock(m_guard);

    for (auto& [key, node]: m_nodes) {
      assert(node);
      // Cache-restored nodes start with no topic/service registrations until
      // getPublications / service restore finishes. Do not treat that empty
      // window as "gone" or restore never completes.
      if (node->isRestoreInProgress())
        continue;
      // Peer masters have no pub/sub/service registrations; keep them for UI / pairing.
      if (node->getNodeFlags() & NodeRef::NODE_MASTER)
        continue;
      if (node->getState() == NodeRef::State::Dead || node->isEmpty()) {
        graveyard.push_back(node);
      }
    }
  }

  // Drop registrations for dead nodes.
  for (auto& node: graveyard) {
    dropRegistrations(*node);
    unregisterNode(node);
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
