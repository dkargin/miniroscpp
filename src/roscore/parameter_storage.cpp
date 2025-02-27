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
  MINIROS_INFO_NAMED("rosparam", "deleteParam %s from %s", key.c_str(), caller_id.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  if (fullKey == "/") {
    m_parameterRoot = RpcValue::Dict();
    return 1;
  }

  names::Path path;
  path.fromString(fullKey);

  auto p = findParameter(path, false);
  if (p.first && p.second) {
    p.second->eraseMember(path.name());
  }

  // TODO: Notify.
  return 1;
}

std::string ParameterStorage::searchParam(const std::string& ns, const std::string& key)
{
  /* Search for parameter key on the Parameter Server. Search starts in caller's namespace and proceeds upwards
   * through parent namespaces until Parameter Server finds a matching key. searchParam()'s behavior is
   * to search for the first partial match. For example, imagine that there are two 'robot_description' parameters:
   *  /robot_description
   *  /robot_description/arm
   *  /robot_description/base
   *  /pr2/robot_description
   *  /pr2/robot_description/base
   * If I start in the namespace /pr2/foo and search for robot_description, searchParam() will match
   * /pr2/robot_description. If I search for robot_description/arm it will return /pr2/robot_description/arm,
   * even though that parameter does not exist (yet).
   * Parameters:
   *  - caller_id (str) ROS caller ID
   *  - key (str) Parameter name to search for.
   * Returns (int, str, str) (code, statusMessage, foundKey) If code is not 1, foundKey should be ignored.
   */

  // caller_id seems like a "namespace".
  if (!names::isGlobal(ns))
    return {};
  if (key.empty() || names::isPrivate(key))
    return {};

  if (names::isGlobal(key)) {
    if (hasParam("", key))
      return key;
    return {};
  }

  /*
    if not key or is_private(key):
        raise ValueError("invalid key")
    if not is_global(ns):
        raise ValueError("namespace must be global")
    if is_global(key):
        if self.has_param(key):
            return key
        else:
            return None
   */

  // there are more efficient implementations, but our hiearchy
  // is not very deep and this is fairly clean code to read.
  // - we only search for the first namespace in the key to check for a match

  names::Path keyPath;
  keyPath.fromString(key);
  std::string key_ns = keyPath.str(0);
  if (hasParam("",  ns + "/" + key_ns)) {
    return ns + "/" + key;
  }

  std::vector<std::string> allParams = this->getParamNames("");
  /*
    key_namespaces = [x for x in key.split(SEP) if x]
    key_ns = key_namespaces[0]

    #  - corner case: have to test initial namespace first as
    #    negative indices won't work with 0
    search_key = ns_join(ns, key_ns)
    if self.has_param(search_key):
        # resolve to full key
        return ns_join(ns, key)
   */
  names::Path nsPath;
  nsPath.fromString(ns);

  for (int i = 0; i < nsPath.size() ; i++) {
    std::string left = nsPath.left(i);
    std::string searchKey = left + key_ns;
    if (hasParam("", searchKey)) {
      std::string fullKey = left + key;
      return fullKey;
    }
  }
  /*
    namespaces = [x for x in ns.split(SEP) if x]
    for i in range(1, len(namespaces)+1):
        search_key = SEP + SEP.join(namespaces[0:-i] + [key_ns])
        if self.has_param(search_key):
            # we have a match on the namespace of the key, so
            # compose the full key and return it
            full_key = SEP + SEP.join(namespaces[0:-i] + [key])
            return full_key
    return None
   */
  return {};
}


/*  def set_param(self, key, value, notify_task=None, caller_id=None):
        Set the parameter in the parameter dictionary.
        """
        @param key: parameter key
        @type  key: str
        @param value: parameter value
        @param notify_task: function to call with subscriber updates. updates is of the form
        [(subscribers, param_key, param_value)*]. The empty dictionary represents an unset parameter.
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
  MINIROS_INFO_NAMED("rosparam", "setParam \"%s\" (\"%s\", \"%s\")", fullKey.c_str(), caller_id.c_str(), key.c_str());
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
    names::Path name;
    name.fromString(fullKey);
    RpcValue* param = findParameter(name, true).first;
    if (!param)
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
   * key = /node/param2/strValue1
   * storage:
   *  /node
   *    param1
   *    param2
   *      strValue1
   *      strValue2
   *  subscribers:
   *    /node/param2
   */

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

std::pair<ParameterStorage::RpcValue*, ParameterStorage::RpcValue*> ParameterStorage::findParameter(const names::Path& name, bool create) const
{
  if (name.size() == 0)
    return {};

  RpcValue* current = const_cast<RpcValue*>(&m_parameterRoot);
  RpcValue* parent = nullptr;

  size_t i = 0;
  for (; i < name.size(); i++) {
    std::string key = name.str(i);
    if (current->hasMember(key)) {
      RpcValue* next = &(*current)[key];
      parent = current;
      current = next;
    }
    else if (create) {
      if (current->getType() != RpcValue::TypeStruct)
        *current = RpcValue::Dict();
      // New value. We are making it dict by default.
      // It is ok for final new value to be dict: setParam will override it anyway.
      (*current)[key] = RpcValue::Dict();
      RpcValue* next = &(*current)[key];

      parent = current;
      current = next;
    } else
      break;
  }

  if (i == name.size()) {
    // current contains a pointer to the needed element.
    return {current, parent};
  }
  return {nullptr, parent};
}


ParameterStorage::RpcValue ParameterStorage::getParam(const std::string& caller_id, const std::string& key) const
{
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  MINIROS_INFO_NAMED("rosparam", "getParam \"%s\" (client=\"%s\", \"%s\"", fullKey.c_str(), caller_id.c_str(), key.c_str());

  names::Path fullPath;
  fullPath.fromString(fullKey);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  RpcValue* param = this->findParameter(fullPath, false).first;
  if (param == nullptr)
    return {};

  return *param;
}

const ParameterStorage::RpcValue* ParameterStorage::subscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_INFO_NAMED("rosparam", "subscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  names::Path path;
  path.fromString(fullKey);
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);

  if (m_regManager) {
    std::shared_ptr<NodeRef> nodeRef = m_regManager->getNodeByName(caller_id);
    auto& listeners = m_parameterListeners[path];
    listeners.insert(nodeRef);
  }

  return nullptr;
}

bool ParameterStorage::unsubscribeParam(
  const std::string& caller_id, const std::string& caller_api, const std::string& key)
{
  MINIROS_INFO_NAMED("rosparam", "unsubscribeParam %s from %s, api=%s", key.c_str(), caller_id.c_str(), caller_api.c_str());

  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  names::Path path;
  path.fromString(fullKey);

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  if (m_regManager) {
    auto it = m_parameterListeners.find(path);
    if (it != m_parameterListeners.end()) {
      std::shared_ptr<NodeRef> nodeRef = m_regManager->getNodeByName(caller_id);
      it->second.erase(nodeRef);
    }
  }
  return true;
}

bool ParameterStorage::hasParam(const std::string& caller_id, const std::string& key) const
{
  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  names::Path fullPath;
  fullPath.fromString(fullKey);

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  RpcValue* param = this->findParameter(fullPath, false).first;
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
  if (!stack.empty())
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