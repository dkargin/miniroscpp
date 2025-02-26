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
  ~ParameterStorage();

  int deleteParam(const std::string& caller_id, const std::string& key);

  Error setParam(const std::string& caller_id, const std::string& key, const RpcValue& value);

  RpcValue getParam(const std::string& caller_id, const std::string& key) const;

  void computeParamUpdates(const Registrations& paramSubscribers, const std::string& caller_id, const std::string& key, const RpcValue& value);

  //std::string searchParam(const std::string& caller_id, const std::string& key) const;

  RpcValue subscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string& key);

  ReturnStruct unsubscribeParam(const std::string& caller_id, const std::string& caller_api, const std::string &key);

  bool hasParam(const std::string& caller_id, const std::string& key) const;

  std::vector<std::string> getParamNames(const std::string& caller_id);

  void dropListener(std::shared_ptr<NodeRef> listener);

  /// Look for a parameter.
  /// No thread locks are used.
  /// @param name - annotated path to parameter
  /// @param create - create parameters if they not exist.
  /// @returns pointer to parameter.
  RpcValue* findParameter(const names::Name& name, bool create) const;

protected:
  /// Collection of all parameters.
  RpcValue m_parameterRoot;

  /// Maps parameter path to a subscriber node.
  std::map<std::string, std::set<std::shared_ptr<NodeRef>>> m_parameterListeners;

  /// Guards access to parameter data.
  mutable std::mutex m_parameterLock;

  RegistrationManager* m_regManager;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_PARAMETER_STORAGE_H
