//
// Created by dkargin on 2/25/25.
//

#ifndef MINIROS_PARAMETER_STORAGE_H
#define MINIROS_PARAMETER_STORAGE_H

#include <mutex>

#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "miniros/names.h"

#include "registrations.h"

namespace miniros {
namespace master {

class RegistrationManager;

class MINIROS_DECL ParameterStorage {
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  ParameterStorage();
  virtual ~ParameterStorage();

  bool deleteParam(const std::string& caller_id, const std::string& key);

  Error setParam(const std::string& caller_id, const std::string& key, const RpcValue& value);

  RpcValue getParam(const std::string& caller_id, const std::string& key) const;

  /// Notify all listeners about change in parameter tree.
  /// @param fullPath - path to node with change.
  /// @param ptr - reference value. It will be null if value is removed.
  /// @returns error code
  Error checkParamUpdates(const std::string& fullPath, const RpcValue* ptr);

  /// Searches for parameter.
  std::string searchParam(const std::string& ns, const std::string& key);

  /// Subscribes to a parameter or a subtree.
  /// Called from Master::unsubscribeParam handler.
  /// @returns pointer to corresponding value.
  const RpcValue* subscribeParam(const std::shared_ptr<NodeRef>& node, const std::string& key);

  /// Unsubscribe node from parameter updates.
  /// Called from Master::unsubscribeParam handler.
  bool unsubscribeParam(const std::shared_ptr<NodeRef>& node, const std::string &key);

  /// Drop all parameter subscriptions from listener.
  void dropSubscriptions(const std::shared_ptr<NodeRef>& node);

  bool hasParam(const std::string& caller_id, const std::string& key) const;

  std::vector<std::string> getParamNames(const std::string& caller_id);

  /// Dumps parameters as a json to a file.
  /// Thread unsafe.
  void dumpParamStateUnsafe(const char* file) const;

  /// Look for a parameter.
  /// No thread locks are used.
  /// @param name - annotated path to parameter
  /// @param create - create parameters if they not exist.
  /// @returns a pair with pointer to parameter and pointer to its owner.
  std::pair<RpcValue*, RpcValue*> findParameter(const names::Path& name, bool create) const;

  void setDumpParameters(bool dump);

protected:
  /// Collection of all parameters.
  RpcValue m_parameterRoot;

  /// Maps parameter path to a subscriber node.
  //std::map<names::Path, std::set<std::weak_ptr<NodeRef>>> m_parameterListeners;

  std::map<names::Path, std::set<std::weak_ptr<NodeRef>, std::owner_less<std::weak_ptr<NodeRef>>>> m_parameterListeners;

  /// Guards access to parameter data.
  mutable std::mutex m_parameterLock;

  /// Dump all parameters to JSON on each update.
  bool m_dumpParameters = false;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_PARAMETER_STORAGE_H
