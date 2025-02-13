//
// Created by dkargin on 2/12/25.
//

#ifndef MINIROS_REGISTRATION_MANAGER_H
#define MINIROS_REGISTRATION_MANAGER_H

#include <memory>

#include "registrations.h"

namespace miniros {

namespace master {

/// Stores registrations for Master.
/// RegistrationManager is not threadsafe, so access must be externally locked as appropriate.
class MINIROS_DECL RegistrationManager {
public:
  using RpcValue = XmlRpc::XmlRpcValue;

  Registrations publishers;
  Registrations subscribers;
  Registrations services;
  Registrations param_subscribers;

  /// <summary>
  /// Param 1 = caller_id
  /// Param 2 = NodeRef
  /// </summary>
  std::map<std::string, std::shared_ptr<NodeRef>> nodes;

  RegistrationManager();

  bool reverse_lookup(const std::string& caller_api) const;

  std::shared_ptr<NodeRef> getNode(const std::string& nodeName) const;

  void _register(Registrations& r, const std::string& key, const std::string& caller_id, const std::string& caller_api,
    const std::string& service_api = "");

  ReturnStruct _unregister(Registrations& r, const std::string& key, const std::string& caller_id,
    const std::string& caller_api, const std::string& service_api = "");

  void register_service(const std::string& service, const std::string& caller_id, const std::string& caller_api,
    const std::string& service_api);

  void register_publisher(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  void register_subscriber(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  void register_param_subscriber(const std::string& param, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_service(const std::string& service, const std::string& caller_id, const std::string& service_api);

  ReturnStruct unregister_subscriber(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_publisher(const std::string& topic, const std::string& caller_id, const std::string& caller_api);

  ReturnStruct unregister_param_subscriber(const std::string& param, const std::string& caller_id, const std::string& caller_api);

  std::shared_ptr<NodeRef> registerNodeApi(const std::string& caller_id, const std::string& caller_api, bool& rtn);
};

} // namespace master
} // namespace miniros

#endif // MINIROS_REGISTRATION_MANAGER_H
