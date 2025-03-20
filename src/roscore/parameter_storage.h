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

  ParameterStorage(RegistrationManager* regManager);
  virtual ~ParameterStorage();

  bool deleteParam(const std::string& caller_id, const std::string& key);

  Error setParam(const std::string& caller_id, const std::string& key, const RpcValue& value);

  RpcValue getParam(const std::string& caller_id, const std::string& key) const;

  /// Notify all listeners about change in parameter tree.
  /// @param fullPath - path to node with change.
  /// @param ptr - reference value. It will be null if value is removed.
  /// @returns error code
  Error checkParamUpdates(const std::string& fullPath, const RpcValue* ptr);

  /// Callback is invoked when parameter is updated.
  std::function<void (const std::shared_ptr<NodeRef>& nr, const std::string& fullPath, const RpcValue* value)> paramUpdateFn;

  /// Searches for parameter.
  std::string searchParam(const std::string& ns, const std::string& key);

  /// Subscribes to a parameter or a subtree.
  /// @returns pointer to corresponding value.
  const RpcValue* subscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key);

  bool unsubscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string &key);

  bool hasParam(const std::string& caller_id, const std::string& key) const;

  std::vector<std::string> getParamNames(const std::string& caller_id);

  void dropListener(std::shared_ptr<NodeRef> listener);

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
  std::map<names::Path, std::set<std::shared_ptr<NodeRef>>> m_parameterListeners;

  /// Guards access to parameter data.
  mutable std::mutex m_parameterLock;

  RegistrationManager* m_regManager;

  /// Dump all parameters to JSON on each update.
  bool m_dumpParameters = false;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_PARAMETER_STORAGE_H
