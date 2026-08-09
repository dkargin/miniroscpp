//
// Persistent state for miniroscore across restarts.
//

#ifndef MINIROS_MASTER_CACHE_H
#define MINIROS_MASTER_CACHE_H

#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "miniros/errors.h"
#include "miniros/rostime.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

namespace miniros {
namespace master {

class RegistrationManager;
class MasterHandler;
class NodeRef;

/// One service advertisement known to master at the time of the snapshot.
struct CachedService {
  std::string name;
  std::string service_api;
};

/// One registered node identity. Topic registrations are re-queried from the
/// node on restore; services are restored from this snapshot because the ROS
/// Slave API has no getServices method.
struct CachedNode {
  std::string name;
  std::string api;
  int pid = 0;
  /// NodeRef::State name at the time of the snapshot (see State::toString).
  std::string state;
  std::vector<CachedService> services;
};

struct MasterCacheData {
  std::string guid;
  int port = 0;
  std::vector<CachedNode> nodes;
};

/// Owns on-disk master state (cache.<port> in cwd) and the async restore session.
class MasterCache {
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  MasterCache() = default;

  /// Wire graph dependencies used during restore / snapshot.
  void bind(RegistrationManager* regs, MasterHandler* handler);

  void setEnabled(bool enabled);
  bool enabled() const { return enabled_; }

  /// Mark graph dirty so the next idle update() flushes to disk.
  void markDirty();

  /// Read cache.<port> once into memory. Safe to call before RPC bind.
  Error load(int port);

  /// GUID from the last successful load() (empty if none).
  const std::string& guid() const { return data_.guid; }

  /// Snapshot loaded by load(); used by beginRestore().
  const MasterCacheData& data() const { return data_; }

  /// Register cached nodes from the already-loaded snapshot and start restore.
  /// @param port - bound server port (updates path for later saves if it differs).
  /// @param runtimeGuid - GUID of this master instance (/run_id).
  void beginRestore(int port, const std::string& runtimeGuid);

  /// Drive restore progress and flush dirty state when idle.
  void update(int port, const std::string& guid);

  /// Write cache immediately (e.g. on shutdown), unless disabled or mid-restore.
  void flush(int port, const std::string& guid);

  static std::filesystem::path pathForPort(const std::filesystem::path& dir, int port);

private:
  struct PendingTopicRestore {
    std::string nodeName;
    std::string nodeApi;
    bool publications = false;
    int code = 0;
    RpcValue data;
  };

  static Error loadFile(const std::filesystem::path& path, MasterCacheData& out);
  /// Atomically write via `path.tmp` + fsync + rename. Safe against power loss mid-write
  /// when the filesystem honors POSIX rename semantics; reports disk-full / read-only.
  static Error saveFile(const std::filesystem::path& path, const MasterCacheData& data,
                        std::string* detail = nullptr);

  MasterCacheData collect(int port, const std::string& guid) const;
  void saveIfNeeded(int port, const std::string& guid);
  void noteSaveResult(Error err, const std::string& detail);
  void processRestore();
  void finalizeRestore();
  void queueTopicList(const std::string& nodeName, const std::string& nodeApi,
    bool publications, int code, const RpcValue& data);
  void applyTopicList(const std::string& nodeName, const std::string& nodeApi,
    bool publications, int code, const RpcValue& data);
  void restoreServices(const std::shared_ptr<NodeRef>& node);

  RegistrationManager* regs_ = nullptr;
  MasterHandler* handler_ = nullptr;

  bool enabled_ = false;
  /// Written from RPC/callback threads via markDirty(); read/cleared on the
  /// master update thread in saveIfNeeded(). Must be atomic for TSan/correctness.
  std::atomic<bool> dirty_{false};
  bool restoring_ = false;
  bool loaded_ = false;
  SteadyTime restoreDeadline_;
  std::filesystem::path path_;
  MasterCacheData data_;

  /// Persist-failure tracking (main / update thread only).
  int saveFailCount_ = 0;
  SteadyTime lastSaveErrorLog_;
  std::string lastSaveError_;

  mutable std::mutex restoreMutex_;
  std::vector<PendingTopicRestore> pendingTopicRestores_;
};

} // namespace master
} // namespace miniros

#endif // MINIROS_MASTER_CACHE_H
