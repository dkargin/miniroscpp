//
// Persistent state for miniroscore across restarts.
//

#include "master_cache.h"

#include "master_handler.h"
#include "node_ref.h"
#include "registration_manager.h"

#include "miniros/common.h"
#include "miniros/console.h"
#include "miniros/internal/nlohmann_json.hpp"

#include <fstream>
#include <memory>
#include <system_error>
#include <utility>

namespace miniros {
namespace master {

using json = nlohmann::json;

void MasterCache::bind(RegistrationManager* regs, MasterHandler* handler)
{
  regs_ = regs;
  handler_ = handler;
}

void MasterCache::setEnabled(bool enabled)
{
  enabled_ = enabled;
  MINIROS_INFO("Master cache %s (file cache.<port> in cwd)", enabled ? "enabled" : "disabled");
}

void MasterCache::markDirty()
{
  dirty_.store(true, std::memory_order_release);
}

std::filesystem::path MasterCache::pathForPort(const std::filesystem::path& dir, int port)
{
  return dir / ("cache." + std::to_string(port));
}

Error MasterCache::load(int port)
{
  data_ = MasterCacheData{};
  loaded_ = false;
  if (!enabled_)
    return Error::Ok;

  path_ = pathForPort(".", port);
  Error err = loadFile(path_, data_);
  if (!err)
    return err;

  loaded_ = true;
  return Error::Ok;
}

void MasterCache::beginRestore(int port, const std::string& runtimeGuid)
{
  if (!enabled_ || !regs_)
    return;

  // Prefer the bound port for subsequent writes.
  if (port > 0)
    path_ = pathForPort(".", port);

  if (!loaded_) {
    dirty_.store(true, std::memory_order_relaxed);
    return;
  }

  if (data_.guid.empty() || data_.guid != runtimeGuid)
    dirty_.store(true, std::memory_order_relaxed);

  if (data_.nodes.empty()) {
    MINIROS_INFO("MasterCache: no nodes to restore from \"%s\"", path_.string().c_str());
    dirty_.store(true, std::memory_order_relaxed);
    saveIfNeeded(port, runtimeGuid);
    return;
  }

  restoring_ = true;
  restoreDeadline_ = SteadyTime::now() + WallDuration(15.0);
  size_t started = 0;

  for (const CachedNode& cn : data_.nodes) {
    if (cn.name.empty() || cn.api.empty())
      continue;

    NodeRef::State cachedState;
    if (!cn.state.empty() && cachedState.fromString(cn.state)) {
      if (cachedState == NodeRef::State::Dead || cachedState == NodeRef::State::ShuttingDown) {
        MINIROS_INFO("MasterCache: skipping %s (cached state=%s)",
                     cn.name.c_str(), cn.state.c_str());
        continue;
      }
    }

    if (cn.pid > 0 && !isProcessAlive(cn.pid)) {
      MINIROS_INFO("MasterCache: skipping dead process %s pid=%d", cn.name.c_str(), cn.pid);
      continue;
    }

    auto report = regs_->registerNodeApi(cn.name, cn.api, 0);
    if (!report.node) {
      MINIROS_WARN("MasterCache: failed to register restored node %s", cn.name.c_str());
      continue;
    }

    std::vector<std::pair<std::string, std::string>> services;
    services.reserve(cn.services.size());
    for (const CachedService& svc : cn.services)
      services.emplace_back(svc.name, svc.service_api);
    report.node->beginRestore(std::move(services));
    ++started;
    MINIROS_INFO("MasterCache: restoring node %s at %s (pid=%d cached_state=%s)",
                 cn.name.c_str(), cn.api.c_str(), cn.pid,
                 cn.state.empty() ? "?" : cn.state.c_str());
  }

  // Snapshot nodes are consumed; keep guid for dirty comparisons until next save.
  data_.nodes.clear();

  if (started == 0)
    finalizeRestore();
}

void MasterCache::update(int port, const std::string& guid)
{
  processRestore();
  saveIfNeeded(port, guid);
}

void MasterCache::flush(int port, const std::string& guid)
{
  dirty_.store(true, std::memory_order_release);
  saveIfNeeded(port, guid);
}

void MasterCache::processRestore()
{
  if (!restoring_ || !regs_ || !handler_)
    return;

  std::vector<PendingTopicRestore> pending;
  {
    std::lock_guard lock(restoreMutex_);
    pending.swap(pendingTopicRestores_);
  }
  for (auto& item : pending) {
    applyTopicList(item.nodeName, item.nodeApi, item.publications, item.code, item.data);
  }

  bool anyInProgress = false;
  for (const auto& node : regs_->listAllNodes()) {
    if (!node || !node->isRestoreInProgress())
      continue;

    const auto state = node->getState();
    if (state == NodeRef::State::Dead || state == NodeRef::State::ShuttingDown)
      continue;

    anyInProgress = true;

    if (state == NodeRef::State::Restoring)
      continue;
    if (state == NodeRef::State::Recovering && node->restoreQueriesLeft() > 0)
      continue;
    if (state != NodeRef::State::Recovering)
      continue;

    const std::string name = node->id();
    const std::string api = node->getApi();
    node->beginTopicRecovery(2);
    restoreServices(node);

    int left = 2;
    if (Error err = node->sendGetPublications("/master",
      [this, name, api](int code, const std::string& /*msg*/, const RpcValue& data) {
        queueTopicList(name, api, true, code, data);
      }); !err) {
      MINIROS_WARN("MasterCache: getPublications(%s) failed: %s", name.c_str(), err.toString());
      --left;
      node->notifyRestoreQueryDone();
    }

    if (Error err = node->sendGetSubscriptions("/master",
      [this, name, api](int code, const std::string& /*msg*/, const RpcValue& data) {
        queueTopicList(name, api, false, code, data);
      }); !err) {
      MINIROS_WARN("MasterCache: getSubscriptions(%s) failed: %s", name.c_str(), err.toString());
      --left;
      node->notifyRestoreQueryDone();
    }

    if (left <= 0)
      anyInProgress = node->isRestoreInProgress();
  }

  if (!anyInProgress) {
    for (const auto& node : regs_->listAllNodes()) {
      if (node && node->isRestoreInProgress()) {
        anyInProgress = true;
        break;
      }
    }
  }

  const bool timedOut = !restoreDeadline_.isZero() && SteadyTime::now() >= restoreDeadline_;
  if (!anyInProgress || timedOut) {
    if (timedOut && anyInProgress)
      MINIROS_WARN("MasterCache: restore timed out");
    finalizeRestore();
  }
}

void MasterCache::queueTopicList(const std::string& nodeName, const std::string& nodeApi,
  bool publications, int code, const RpcValue& data)
{
  PendingTopicRestore item;
  item.nodeName = nodeName;
  item.nodeApi = nodeApi;
  item.publications = publications;
  item.code = code;
  item.data = data;
  std::lock_guard lock(restoreMutex_);
  pendingTopicRestores_.push_back(std::move(item));
}

void MasterCache::applyTopicList(const std::string& nodeName, const std::string& nodeApi,
  bool publications, int code, const RpcValue& data)
{
  if (!restoring_ || !regs_ || !handler_)
    return;

  auto node = regs_->getNodeByName(nodeName);
  if (!node)
    return;

  if (code == 1 && data.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    for (int i = 0; i < data.size(); ++i) {
      const RpcValue& entry = data[i];
      if (entry.getType() != XmlRpc::XmlRpcValue::TypeArray || entry.size() < 2)
        continue;
      const std::string topic = static_cast<std::string>(entry[0]);
      const std::string type = static_cast<std::string>(entry[1]);
      if (topic.empty() || type.empty())
        continue;

      if (publications)
        regs_->register_publisher(topic, type, nodeName, nodeApi);
      else
        regs_->register_subscriber(topic, type, nodeName, nodeApi);

      auto subscribers = regs_->getTopicSubscribers(topic);
      handler_->notifyTopicSubscribers(topic, subscribers);

      MINIROS_INFO("MasterCache: restored %s %s type=%s on %s",
                   publications ? "publisher" : "subscriber",
                   topic.c_str(), type.c_str(), nodeName.c_str());
    }
  } else {
    MINIROS_WARN("MasterCache: %s response for %s failed code=%d",
                 publications ? "getPublications" : "getSubscriptions",
                 nodeName.c_str(), code);
  }

  node->notifyRestoreQueryDone();
}

void MasterCache::restoreServices(const std::shared_ptr<NodeRef>& node)
{
  if (!node || !regs_)
    return;
  const std::string nodeName = node->id();
  const std::string nodeApi = node->getApi();
  for (const auto& [svcName, serviceApi] : node->takePendingRestoreServices()) {
    if (svcName.empty() || serviceApi.empty())
      continue;
    regs_->register_service(svcName, nodeName, nodeApi, serviceApi);
    MINIROS_INFO("MasterCache: restored service %s (%s) on %s",
                 svcName.c_str(), serviceApi.c_str(), nodeName.c_str());
  }
}

void MasterCache::finalizeRestore()
{
  if (!restoring_)
    return;

  restoring_ = false;
  if (!regs_)
    return;

  for (const auto& node : regs_->listAllNodes()) {
    if (!node || !node->isRestoreInProgress())
      continue;
    if (node->getState() == NodeRef::State::Restoring)
      regs_->scheduleShutdown(node);
    else {
      while (!node->notifyRestoreQueryDone()) {
      }
    }
  }

  MINIROS_INFO("MasterCache: restore complete");
  dirty_.store(true, std::memory_order_relaxed);
}

MasterCacheData MasterCache::collect(int port, const std::string& guid) const
{
  MasterCacheData data;
  data.guid = guid;
  data.port = port;
  if (!regs_)
    return data;

  for (const auto& node : regs_->listAllNodes()) {
    if (!node)
      continue;
    const int flags = node->getNodeFlags();
    if (flags & (NodeRef::NODE_LOCAL | NodeRef::NODE_MASTER))
      continue;
    if (node->getState() == NodeRef::State::Dead)
      continue;

    CachedNode cn;
    cn.name = node->id();
    cn.api = node->getApi();
    cn.pid = node->pid();
    cn.state = node->getState().toString();

    std::unique_lock nodeLock(*node);
    const auto& services = node->getServicesUnsafe();
    for (const std::string& svcName : services) {
      CachedService svc;
      svc.name = svcName;
      svc.service_api = regs_->services.get_service_api(svcName);
      if (!svc.service_api.empty())
        cn.services.push_back(std::move(svc));
    }

    data.nodes.push_back(std::move(cn));
  }
  return data;
}

void MasterCache::saveIfNeeded(int port, const std::string& guid)
{
  // Skip while disabled or mid-restore; leave dirty_ set so a later update flushes.
  if (!enabled_ || restoring_)
    return;

  // Clear the flag before IO so concurrent markDirty() during save is not lost.
  if (!dirty_.exchange(false, std::memory_order_acq_rel))
    return;

  if (port <= 0 && !path_.empty()) {
    // Keep previous path if port is temporarily unavailable.
  } else {
    path_ = pathForPort(".", port);
  }

  if (path_.empty()) {
    dirty_.store(true, std::memory_order_relaxed);
    return;
  }

  if (!saveFile(path_, collect(port, guid)))
    dirty_.store(true, std::memory_order_relaxed);
}

Error MasterCache::loadFile(const std::filesystem::path& path, MasterCacheData& out)
{
  out = MasterCacheData{};

  std::error_code ec;
  if (!std::filesystem::exists(path, ec) || ec) {
    return Error::Ok;
  }

  std::ifstream in(path);
  if (!in.is_open()) {
    MINIROS_WARN("MasterCache: failed to open \"%s\" for reading", path.string().c_str());
    return Error::SystemError;
  }

  json root;
  try {
    in >> root;
  } catch (const json::exception& e) {
    MINIROS_WARN("MasterCache: failed to parse \"%s\": %s", path.string().c_str(), e.what());
    return Error::InvalidValue;
  }

  if (!root.is_object()) {
    MINIROS_WARN("MasterCache: root is not an object in \"%s\"", path.string().c_str());
    return Error::InvalidValue;
  }

  if (root.contains("guid") && root["guid"].is_string())
    out.guid = root["guid"].get<std::string>();
  if (root.contains("port") && root["port"].is_number_integer())
    out.port = root["port"].get<int>();

  if (root.contains("nodes") && root["nodes"].is_array()) {
    for (const auto& nodeJson : root["nodes"]) {
      if (!nodeJson.is_object())
        continue;
      CachedNode node;
      if (nodeJson.contains("name") && nodeJson["name"].is_string())
        node.name = nodeJson["name"].get<std::string>();
      if (nodeJson.contains("api") && nodeJson["api"].is_string())
        node.api = nodeJson["api"].get<std::string>();
      if (nodeJson.contains("pid") && nodeJson["pid"].is_number_integer())
        node.pid = nodeJson["pid"].get<int>();
      if (nodeJson.contains("state") && nodeJson["state"].is_string())
        node.state = nodeJson["state"].get<std::string>();
      if (nodeJson.contains("services") && nodeJson["services"].is_array()) {
        for (const auto& svcJson : nodeJson["services"]) {
          if (!svcJson.is_object())
            continue;
          CachedService svc;
          if (svcJson.contains("name") && svcJson["name"].is_string())
            svc.name = svcJson["name"].get<std::string>();
          if (svcJson.contains("api") && svcJson["api"].is_string())
            svc.service_api = svcJson["api"].get<std::string>();
          if (!svc.name.empty() && !svc.service_api.empty())
            node.services.push_back(std::move(svc));
        }
      }
      if (!node.name.empty() && !node.api.empty())
        out.nodes.push_back(std::move(node));
    }
  }

  MINIROS_INFO("MasterCache: loaded \"%s\" (guid=%s nodes=%zu)",
               path.string().c_str(), out.guid.c_str(), out.nodes.size());
  return Error::Ok;
}

Error MasterCache::saveFile(const std::filesystem::path& path, const MasterCacheData& data)
{
  std::error_code ec;
  const auto parent = path.parent_path();
  if (!parent.empty() && !std::filesystem::exists(parent, ec)) {
    if (!std::filesystem::create_directories(parent, ec) && ec) {
      MINIROS_WARN("MasterCache: failed to create \"%s\": %s",
                   parent.string().c_str(), ec.message().c_str());
      return Error::SystemError;
    }
  }

  json root;
  root["guid"] = data.guid;
  root["port"] = data.port;
  root["nodes"] = json::array();
  for (const CachedNode& node : data.nodes) {
    json nodeJson;
    nodeJson["name"] = node.name;
    nodeJson["api"] = node.api;
    nodeJson["pid"] = node.pid;
    nodeJson["state"] = node.state;
    nodeJson["services"] = json::array();
    for (const CachedService& svc : node.services) {
      json svcJson;
      svcJson["name"] = svc.name;
      svcJson["api"] = svc.service_api;
      nodeJson["services"].push_back(std::move(svcJson));
    }
    root["nodes"].push_back(std::move(nodeJson));
  }

  const std::filesystem::path tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    if (!out.is_open()) {
      MINIROS_WARN("MasterCache: failed to open \"%s\" for writing", tmp.string().c_str());
      return Error::SystemError;
    }
    out << root.dump(2) << '\n';
    if (!out.good()) {
      MINIROS_WARN("MasterCache: write failed for \"%s\"", tmp.string().c_str());
      return Error::SystemError;
    }
  }

  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    MINIROS_WARN("MasterCache: rename \"%s\" → \"%s\" failed: %s",
                 tmp.string().c_str(), path.string().c_str(), ec.message().c_str());
    std::filesystem::remove(tmp, ec);
    return Error::SystemError;
  }

  MINIROS_DEBUG("MasterCache: saved \"%s\" (nodes=%zu)", path.string().c_str(), data.nodes.size());
  return Error::Ok;
}

} // namespace master
} // namespace miniros
