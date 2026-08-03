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
#include "miniros/network/network.h"

#include <cctype>
#include <cerrno>
#include <cstring>
#include <fstream>
#include <memory>
#include <system_error>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <fcntl.h>
#include <unistd.h>
#endif

namespace miniros {
namespace master {

using json = nlohmann::json;

namespace {

bool hostnamesEqual(const std::string& a, const std::string& b)
{
  if (a.size() != b.size())
    return false;
  for (size_t i = 0; i < a.size(); ++i) {
    if (std::tolower(static_cast<unsigned char>(a[i])) !=
        std::tolower(static_cast<unsigned char>(b[i])))
      return false;
  }
  return true;
}

bool looksLikeLoopbackHost(const std::string& host)
{
  if (host.empty() || host == "localhost" || host == "::1")
    return true;
  return host.rfind("127.", 0) == 0;
}

/// Host that originally wrote this cache, or inferred from a non-loopback node API.
std::string recordedCacheHost(const MasterCacheData& data)
{
  if (!data.host.empty())
    return data.host;
  for (const CachedNode& node : data.nodes) {
    std::string host;
    uint32_t port = 0;
    if (!network::splitURI(node.api, host, port) || host.empty() || looksLikeLoopbackHost(host))
      continue;
    return host;
  }
  return {};
}

} // namespace

void MasterCache::bind(RegistrationManager* regs, MasterHandler* handler)
{
  regs_ = regs;
  handler_ = handler;
}

void MasterCache::setPeerPersistence(CollectPeersFn collect, RestorePeersFn restore)
{
  collectPeers_ = std::move(collect);
  restorePeers_ = std::move(restore);
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

  const std::string here = network::getHost();
  const std::string recorded = recordedCacheHost(data_);
  if (!recorded.empty() && !here.empty() && !hostnamesEqual(recorded, here)) {
    MINIROS_WARN("MasterCache: \"%s\" was written on host \"%s\", this process is \"%s\". "
                 "Ignoring stored GUID/nodes/peers (copied cache?).",
                 path_.string().c_str(), recorded.c_str(), here.c_str());
    data_ = MasterCacheData{};
    loaded_ = false;
    return Error::Ok;
  }

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

  // Snapshot nodes are consumed; keep guid/peers for dirty comparisons and
  // restorePeers() (called after MultimasterManager::start).
  data_.nodes.clear();

  if (started == 0)
    finalizeRestore();
}

void MasterCache::restorePeers()
{
  if (!enabled_ || !restorePeers_ || data_.peers.empty()) {
    data_.peers.clear();
    return;
  }

  const std::vector<CachedPeer> peers = std::move(data_.peers);
  data_.peers.clear();
  MINIROS_INFO("MasterCache: restoring %zu multimaster peer pairing(s)", peers.size());
  restorePeers_(peers);
  dirty_.store(true, std::memory_order_relaxed);
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
  data.host = network::getHost();
  data.port = port;
  if (!regs_)
    return data;

  for (const auto& node : regs_->listAllNodes()) {
    if (!node)
      continue;
    const int flags = node->getNodeFlags();
    // Local master, peer-master UI entries, and foreign (mirrored) nodes are not
    // restored via Slave API — pairing + sync re-imports them after restart.
    if (flags & (NodeRef::NODE_LOCAL | NodeRef::NODE_MASTER | NodeRef::NODE_FOREIGN))
      continue;
    if (node->getState() == NodeRef::State::Dead)
      continue;

    CachedNode cn;
    cn.name = node->id();
    cn.api = node->getApi();
    cn.pid = node->pid();
    cn.state = node->getState().toString();

    // Copy service names under the node lock, then resolve APIs under the
    // RegistrationManager lock. Do not hold both at once (register takes
    // m_guard then the node lock). Reading services.service_api_map without
    // m_guard races with register/unregister and can yield torn UTF-8 strings
    // that abort nlohmann::json::dump().
    std::vector<std::string> serviceNames;
    {
      NodeRef::Lock nodeLock(*node);
      const auto& services = node->getServicesLocked(nodeLock);
      serviceNames.assign(services.begin(), services.end());
    }
    {
      RegistrationManager::Lock regLock(*regs_);
      for (const std::string& svcName : serviceNames) {
        CachedService svc;
        svc.name = svcName;
        svc.service_api = regs_->services.get_service_api(svcName);
        if (!svc.service_api.empty())
          cn.services.push_back(std::move(svc));
      }
    }

    data.nodes.push_back(std::move(cn));
  }

  if (collectPeers_)
    data.peers = collectPeers_();

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
    noteSaveResult(Error::InvalidValue, "empty cache path");
    return;
  }

  std::string detail;
  const Error err = saveFile(path_, collect(port, guid), &detail);
  if (!err) {
    // Keep dirty so we retry after disk-full / read-only / power-loss style failures.
    dirty_.store(true, std::memory_order_relaxed);
    noteSaveResult(err, detail.empty() ? std::string(err.toString()) : detail);
    return;
  }
  noteSaveResult(Error::Ok, detail);
}

void MasterCache::noteSaveResult(Error err, const std::string& detail)
{
  if (err) {
    if (saveFailCount_ > 0) {
      MINIROS_INFO("MasterCache: save recovered after %d failure(s)%s%s",
                   saveFailCount_,
                   detail.empty() ? "" : ": ",
                   detail.c_str());
    }
    saveFailCount_ = 0;
    lastSaveError_.clear();
    return;
  }

  ++saveFailCount_;
  lastSaveError_ = detail;

  // Log the first failure immediately, then at most once every 30s while it persists.
  const SteadyTime now = SteadyTime::now();
  const bool first = (saveFailCount_ == 1);
  const bool due = first || (now - lastSaveErrorLog_ > WallDuration(30.0));
  if (!due)
    return;

  lastSaveErrorLog_ = now;
  MINIROS_ERROR("MasterCache: failed to persist state (attempt %d): %s — will retry; "
                "graph updates may be lost across an unclean master restart",
                saveFailCount_,
                detail.empty() ? err.toString() : detail.c_str());
}

Error MasterCache::loadFile(const std::filesystem::path& path, MasterCacheData& out)
{
  out = MasterCacheData{};

  std::error_code ec;
  const std::filesystem::path tmpPath = path.string() + ".tmp";
  if (std::filesystem::exists(tmpPath, ec) && !ec) {
    MINIROS_WARN("MasterCache: found leftover \"%s\" (previous save may have been "
                 "interrupted by power loss or a full/read-only disk); ignoring it",
                 tmpPath.string().c_str());
  }

  if (!std::filesystem::exists(path, ec) || ec) {
    return Error::Ok;
  }

  const auto fileSize = std::filesystem::file_size(path, ec);
  if (ec) {
    MINIROS_ERROR("MasterCache: cannot stat \"%s\": %s", path.string().c_str(), ec.message().c_str());
    return Error::SystemError;
  }
  if (fileSize == 0) {
    MINIROS_ERROR("MasterCache: \"%s\" is empty (truncated or interrupted write?); starting without cache",
                  path.string().c_str());
    return Error::InvalidValue;
  }

  std::ifstream in(path);
  if (!in.is_open()) {
    const int saved = errno;
    MINIROS_ERROR("MasterCache: failed to open \"%s\" for reading: %s (errno=%d)",
                  path.string().c_str(), std::strerror(saved), saved);
    return Error::SystemError;
  }

  json root;
  try {
    in >> root;
  } catch (const json::exception& e) {
    MINIROS_ERROR("MasterCache: corrupt/incomplete \"%s\" (%zu bytes): %s — "
                  "starting without cache (power loss mid-write?)",
                  path.string().c_str(), static_cast<size_t>(fileSize), e.what());
    return Error::InvalidValue;
  }

  if (!root.is_object()) {
    MINIROS_ERROR("MasterCache: root is not an object in \"%s\"; starting without cache",
                  path.string().c_str());
    return Error::InvalidValue;
  }

  if (root.contains("guid") && root["guid"].is_string())
    out.guid = root["guid"].get<std::string>();
  if (root.contains("host") && root["host"].is_string())
    out.host = root["host"].get<std::string>();
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

  if (root.contains("peers") && root["peers"].is_array()) {
    for (const auto& peerJson : root["peers"]) {
      if (!peerJson.is_object())
        continue;
      CachedPeer peer;
      if (peerJson.contains("uuid") && peerJson["uuid"].is_string())
        peer.uuid = peerJson["uuid"].get<std::string>();
      if (peerJson.contains("uri") && peerJson["uri"].is_string())
        peer.uri = peerJson["uri"].get<std::string>();
      if (peerJson.contains("sync_host") && peerJson["sync_host"].is_string())
        peer.sync_host = peerJson["sync_host"].get<std::string>();
      if (peerJson.contains("sync_port") && peerJson["sync_port"].is_number_integer())
        peer.sync_port = peerJson["sync_port"].get<int>();
      if (peerJson.contains("state") && peerJson["state"].is_string())
        peer.state = peerJson["state"].get<std::string>();
      if (!peer.uuid.empty() && !peer.sync_host.empty() && peer.sync_port > 0)
        out.peers.push_back(std::move(peer));
    }
  }

  MINIROS_INFO("MasterCache: loaded \"%s\" (guid=%s host=%s nodes=%zu peers=%zu)",
               path.string().c_str(), out.guid.c_str(),
               out.host.empty() ? "?" : out.host.c_str(),
               out.nodes.size(), out.peers.size());
  return Error::Ok;
}

namespace {

std::string errnoDetail(const char* op, const std::filesystem::path& path, int err)
{
  std::string msg = std::string(op) + " \"" + path.string() + "\": " + std::strerror(err)
                    + " (errno=" + std::to_string(err) + ")";
  if (err == ENOSPC
#ifdef EDQUOT
      || err == EDQUOT
#endif
  ) {
    msg += " [disk full or quota exceeded]";
  } else if (err == EROFS) {
    msg += " [filesystem is read-only]";
  } else if (err == EACCES || err == EPERM) {
    msg += " [permission denied]";
  }
  return msg;
}

Error writeAtomically(const std::filesystem::path& path, const std::string& payload, std::string* detail)
{
  auto setDetail = [&](std::string msg) {
    if (detail)
      *detail = std::move(msg);
  };

  const std::filesystem::path tmp = path.string() + ".tmp";
  std::error_code ec;
  // Drop a stale temp from a previous interrupted attempt before rewriting.
  std::filesystem::remove(tmp, ec);

#if defined(_WIN32)
  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
      setDetail("open \"" + tmp.string() + "\" for writing failed");
      return Error::SystemError;
    }
    out.write(payload.data(), static_cast<std::streamsize>(payload.size()));
    out.flush();
    if (!out.good()) {
      setDetail("write \"" + tmp.string() + "\" failed");
      out.close();
      std::filesystem::remove(tmp, ec);
      return Error::SystemError;
    }
  }
#else
  // POSIX: open/write/fsync/close so a power cut cannot leave a half-written final file
  // (final replace is rename below). ENOSPC/EROFS surface with clear errno tags.
  const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
  if (fd < 0) {
    setDetail(errnoDetail("open", tmp, errno));
    return Error::SystemError;
  }

  const char* buf = payload.data();
  size_t remaining = payload.size();
  while (remaining > 0) {
    const ssize_t n = ::write(fd, buf, remaining);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      const int saved = errno;
      ::close(fd);
      std::filesystem::remove(tmp, ec);
      setDetail(errnoDetail("write", tmp, saved));
      return Error::SystemError;
    }
    buf += n;
    remaining -= static_cast<size_t>(n);
  }

  if (::fsync(fd) != 0) {
    const int saved = errno;
    ::close(fd);
    std::filesystem::remove(tmp, ec);
    setDetail(errnoDetail("fsync", tmp, saved));
    return Error::SystemError;
  }
  if (::close(fd) != 0) {
    const int saved = errno;
    std::filesystem::remove(tmp, ec);
    setDetail(errnoDetail("close", tmp, saved));
    return Error::SystemError;
  }
#endif

  std::filesystem::rename(tmp, path, ec);
  if (ec) {
#if defined(_WIN32)
    setDetail("rename \"" + tmp.string() + "\" → \"" + path.string() + "\": " + ec.message());
#else
    // Prefer errno-tagged message when the generic category carries an errno value.
    const int err = (ec.category() == std::system_category()) ? ec.value() : EIO;
    setDetail(errnoDetail("rename", path, err) + " (from \"" + tmp.string() + "\")");
#endif
    std::filesystem::remove(tmp, ec);
    return Error::SystemError;
  }

#if !defined(_WIN32)
  // Best-effort: durable directory entry for the rename itself.
  const auto parent = path.parent_path().empty() ? std::filesystem::path(".") : path.parent_path();
  const int dirfd = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
  if (dirfd >= 0) {
    if (::fsync(dirfd) != 0) {
      MINIROS_WARN("MasterCache: %s", errnoDetail("fsync(dir)", parent, errno).c_str());
    }
    ::close(dirfd);
  }
#endif

  return Error::Ok;
}

} // namespace

Error MasterCache::saveFile(const std::filesystem::path& path, const MasterCacheData& data,
                            std::string* detail)
{
  auto setDetail = [&](std::string msg) {
    if (detail)
      *detail = std::move(msg);
  };

  std::error_code ec;
  const auto parent = path.parent_path();
  if (!parent.empty() && !std::filesystem::exists(parent, ec)) {
    if (!std::filesystem::create_directories(parent, ec) && ec) {
      setDetail("create_directories \"" + parent.string() + "\": " + ec.message());
      return Error::SystemError;
    }
  }

  json root;
  root["guid"] = data.guid;
  root["host"] = data.host;
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

  root["peers"] = json::array();
  for (const CachedPeer& peer : data.peers) {
    json peerJson;
    peerJson["uuid"] = peer.uuid;
    peerJson["uri"] = peer.uri;
    peerJson["sync_host"] = peer.sync_host;
    peerJson["sync_port"] = peer.sync_port;
    peerJson["state"] = peer.state;
    root["peers"].push_back(std::move(peerJson));
  }

  std::string payload;
  try {
    // replace: never abort the master on a stray non-UTF-8 byte in a name/URI.
    payload = root.dump(2, ' ', false, json::error_handler_t::replace) + "\n";
  } catch (const json::exception& e) {
    setDetail(std::string("json dump failed: ") + e.what());
    return Error::InvalidValue;
  }

  if (Error err = writeAtomically(path, payload, detail); !err)
    return err;

  MINIROS_DEBUG("MasterCache: saved \"%s\" (nodes=%zu peers=%zu)",
                path.string().c_str(), data.nodes.size(), data.peers.size());
  setDetail("saved " + path.string());
  return Error::Ok;
}

} // namespace master
} // namespace miniros
