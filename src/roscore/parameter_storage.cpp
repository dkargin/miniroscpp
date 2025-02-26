//
// Created by dkargin on 2/25/25.
//

#include "registration_manager.h"
#include "parameter_storage.h"

namespace miniros {
namespace master {

ParameterStorage::ParameterStorage(RegistrationManager* regManager)
  :m_regManager(regManager)
{
    assert(regManager);
    m_parameterRoot = RpcValue::Dict();
}

ParameterStorage::~ParameterStorage()
{
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  m_regManager = nullptr;
}

int ParameterStorage::deleteParam(const std::string& caller_id, const std::string& key)
{
  MINIROS_DEBUG_NAMED("rosparam", "deleteParam %s from %s", key.c_str(), caller_id.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  if (fullKey == "/") {
    m_parameterRoot = RpcValue::Dict();
    return 1;
  }

  names::Name name;
  name.fromPath(fullKey);

  // TODO: Implement
  // TODO: Notify.

  return 1;
  /*
  try
  {
      key = Names::resolve_name(key, caller_id);
      param_server.delete_param(key, _notify_param_subscribers);
      return 1;
  }
  catch (KeyNotFoundException e) { return -1; }*/
}

/*  def set_param(self, key, value, notify_task=None, caller_id=None):
        Set the parameter in the parameter dictionary.
        """
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @param notify_task: function to call with
        subscriber updates. updates is of the form
        [(subscribers, param_key, param_value)*]. The empty dictionary
        represents an unset parameter.
        @type  notify_task: fn(updates)
        @param caller_id: the caller id
        @type caller_id: str
        """
*/
Error ParameterStorage::setParam(const std::string& caller_id, const std::string& key, const RpcValue& value)
{
  std::stringstream ss;
  ss << key << "=";
  value.write(ss);
  ss << " from " << caller_id;

  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);

  if (fullKey == "/") {
    if (value.getType() == RpcValue::TypeStruct) {
      m_parameterRoot = value;
      // TODO: Notify all listeners.
    } else {
      MINIROS_ERROR_NAMED("rosparam", "setParam %s - cannot set root of parameter tree to non-dictionary", ss.str().c_str());
      return Error::InvalidValue;
    }
  } else {
    names::Name name;
    name.fromPath(fullKey);
    RpcValue* param = this->findParameter(name, true);
    if (!value)
      return Error::InvalidValue;
    *param = value;
    return Error::Ok;
  }
  // TODO: Notify
  return Error::Ok;
}

void ParameterStorage::computeParamUpdates(const Registrations& paramSubscribers,
  const std::string& caller_id, const std::string& key, const RpcValue& value)
{
/*
def compute_param_updates(subscribers, param_key, param_value, caller_id_to_ignore=None):
    """
    Compute subscribers that should be notified based on the parameter update
    @param subscribers: parameter subscribers
    @type  subscribers: Registrations
    @param param_key: parameter key
    @type  param_key: str
    @param param_value: parameter value
    @type  param_value: str
    @param caller_id_to_ignore: the caller to ignore
    @type caller_id_to_ignore: str
    """

    # logic correct for both updates and deletions

    if not subscribers:
        return []

    # end with a trailing slash to optimize startswith check from
    # needing an extra equals check
    if param_key != SEP:
        param_key = canonicalize_name(param_key) + SEP

    # compute all the updated keys
    if type(param_value) == dict:
        all_keys = _compute_all_keys(param_key, param_value)
    else:
        all_keys = None

    updates = []

    # subscriber gets update if anything in the subscribed namespace is updated or if its deleted
    for sub_key in subscribers.iterkeys():
        ns_key = sub_key
        if ns_key[-1] != SEP:
            ns_key = sub_key + SEP
        if param_key.startswith(ns_key):
            node_apis = subscribers[sub_key]
            if caller_id_to_ignore is not None:
                node_apis = [
                    (caller_id, caller_api)
                    for (caller_id, caller_api) in node_apis
                    if caller_id != caller_id_to_ignore]
            updates.append((node_apis, param_key, param_value))
        elif all_keys is not None and ns_key.startswith(param_key) \
             and not sub_key in all_keys:
            # parameter was deleted
            node_apis = subscribers[sub_key]
            updates.append((node_apis, sub_key, {}))

    # add updates for exact matches within tree
    if all_keys is not None:
        # #586: iterate over parameter tree for notification
        for key in all_keys:
            if key in subscribers:
                # compute actual update value
                sub_key = key[len(param_key):]
                namespaces = [x for x in sub_key.split(SEP) if x]
                val = param_value
                for ns in namespaces:
                    val = val[ns]

                updates.append((subscribers[key], key, val))

    return updates
*/
}

ParameterStorage::RpcValue* ParameterStorage::findParameter(const names::Name& name, bool create) const
{
  if (name.size() == 0)
    return nullptr;

  RpcValue* current = const_cast<RpcValue*>(&m_parameterRoot);
  size_t i = 0;
  for (; i < name.size(); i++) {
    std::string key = name.str(i);
    if (current->hasMember(key)) {
      RpcValue* next = &(*current)[key];
      current = next;
    }
    else
      break;
  }

  if (i == name.size()) {
    return current;
  }
  return nullptr;
}


ParameterStorage::RpcValue ParameterStorage::getParam(const std::string& caller_id, const std::string& key) const
{
  MINIROS_DEBUG_NAMED("rosparam", "getParam %s from %s", key.c_str(), caller_id.c_str());

  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  names::Name fullPath;
  fullPath.fromPath(fullKey);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  RpcValue* param = this->findParameter(fullPath, false);
  if (param == nullptr)
    return {};

  return *param;
}

ParameterStorage::RpcValue ParameterStorage::subscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_DEBUG_NAMED(
    "rosparam", "subscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  if (m_regManager) {
    std::shared_ptr<NodeRef> nodeRef = m_regManager->getNodeByName(caller_id);
    auto& listeners = m_parameterListeners[fullKey];
    listeners.insert(nodeRef);
  }
  return {};
}

ReturnStruct ParameterStorage::unsubscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_DEBUG_NAMED(
    "rosparam", "unsubscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());

  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  if (m_regManager) {
    auto it = m_parameterListeners.find(fullKey);
    if (it != m_parameterListeners.end()) {
      std::shared_ptr<NodeRef> nodeRef = m_regManager->getNodeByName(caller_id);
      it->second.erase(nodeRef);
    }
  }
  return {};
}

bool ParameterStorage::hasParam(const std::string& caller_id, const std::string& key) const
{
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  names::Name fullPath;
  fullPath.fromPath(fullKey);

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  RpcValue* param = this->findParameter(fullPath, false);
  return param != nullptr;
}

std::string makeName(const std::vector<std::string>& path, const std::string& final)
{
  std::stringstream ss;
  for (const auto& p: path) {
    ss << "/" << p;
  }
  ss << "/" << final;
  return ss.str();
}

void depthFirstWalk(const XmlRpc::XmlRpcValue& node, std::vector<std::string>& outNames, std::vector<std::string>& stack) {
  for (const auto [key, val]: node) {
    if (val.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
      stack.push_back(key);
      depthFirstWalk(val, outNames, stack);
    } else if (val.isPrimitive()) {
      outNames.push_back(makeName(stack, key));
    } else {
      // Nothing to do here.
    }
  }
  stack.pop_back();
}

std::vector<std::string> ParameterStorage::getParamNames(const std::string& caller_id)
{
  // Output names.
  std::vector<std::string> names;
  // Current stack for depth first walk.
  std::vector<std::string> stack;

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  depthFirstWalk(m_parameterRoot, names, stack);
  return names;
}


} // namespace master
} // namespace miniros