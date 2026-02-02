//
// Created by dkargin on 2/25/25.
//

#include <cassert>
#include <fstream>

#include "registration_manager.h"
#include "parameter_storage.h"

namespace miniros {
namespace master {

ParameterStorage::ParameterStorage()
{
    m_parameterRoot = RpcValue::Dict();
}

ParameterStorage::~ParameterStorage()
{
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
}

bool ParameterStorage::deleteParam(const std::string& caller_id, const std::string& key)
{
  MINIROS_INFO_NAMED("rosparam", "deleteParam %s from %s", key.c_str(), caller_id.c_str());
  std::string fullKey = miniros::names::resolve(caller_id, key, false);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  if (fullKey == "/") {
    m_parameterRoot = RpcValue::Dict();
    checkParamUpdates(fullKey, nullptr);
    return true;
  }

  names::Path path;
  path.fromString(fullKey);

  auto p = findParameter(path, false);
  if (p.first && p.second) {
    p.second->eraseMember(path.name());
    checkParamUpdates(fullKey, nullptr);
    return true;
  }

  return false;
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

  // there are more efficient implementations, but our hierarchy
  // is not very deep and this is fairly clean code to read.
  // - we only search for the first namespace in the key to check for a match

  names::Path keyPath;
  keyPath.fromString(key);
  std::string key_ns = keyPath.str(0);
  if (hasParam(ns, key_ns)) {
    return names::resolve(ns, key, false);
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

Error ParameterStorage::setParam(const std::string& caller_id, const std::string& key, const RpcValue& value)
{
  std::stringstream ss;
  ss << key << "=";
  value.write(ss);
  ss << " from " << caller_id;

  std::string fullKey = miniros::names::resolve(caller_id, key, false);
  MINIROS_INFO_NAMED("rosparam", "setParam(\"%s\") from \"%s\", key=\"%s\"", fullKey.c_str(), caller_id.c_str(), key.c_str());
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);

  if (fullKey == "/") {
    if (value.getType() == RpcValue::TypeStruct) {
      m_parameterRoot = value;
      checkParamUpdates(fullKey, &m_parameterRoot);
    } else {
      MINIROS_ERROR_NAMED("rosparam", "setParam(\"%s\") - cannot set root of parameter tree to non-dictionary", ss.str().c_str());
      return Error::InvalidValue;
    }
  } else {
    names::Path name;
    if (auto err = name.fromString(fullKey); !err)
      return err;
    RpcValue* param = findParameter(name, true).first;
    if (!param)
      return Error::ParameterNotFound;
    *param = value;
    checkParamUpdates(fullKey, param);
  }

  return Error::Ok;
}

Error ParameterStorage::checkParamUpdates(const std::string& fullKey, const RpcValue* ptr)
{
  /*
   * Case1: simple param was changed. key = /node/param2/strValue1
   * storage:
   *  /node
   *    param1
   *    param2
   *      strValue1
   *      strValue2
   *  subscribers:
   *    /node/param2
   */
  if (m_dumpParameters)
    dumpParamStateUnsafe("params.json");

  if (m_parameterListeners.empty())
    return Error::Ok;
  names::Path path;
  if (auto err = path.fromString(fullKey); err != Error::Ok)
    return err;

  // Subscriber /a/b
  // Updated:  /a/b/c/d
  if (ptr && ptr->getType() == RpcValue::TypeStruct) {
    // This is big dictionary update.
    // Case1: subscriber listens to internal variable.
    //  Updated: /a/b/c
    //  Subscriber /a/b/c/d/e
    // Case2: subscriber listens to variable upper in the hierarchy.
    for (auto& [subPath, nodes]: m_parameterListeners) {
      if (subPath.startsWith(path) || path.startsWith(subPath)) {
        std::vector<std::weak_ptr<NodeRef>> dead;
        for (auto& wnode: nodes) {
          auto node = wnode.lock();
          if (node) {
            node->sendParameterUpdate("/master", subPath.fullPath(), ptr);
          } else {
            dead.push_back(wnode);
          }
        }
        for (auto wnode: dead) {
          nodes.erase(wnode);
        }
      }
    }
  }
  else {
    // Parameter is removed/updated, or it was an elementary value.
    for (auto& [subPath, nodes]: m_parameterListeners) {
      if (subPath.startsWith(path) || path.startsWith(subPath)) {
        std::vector<std::weak_ptr<NodeRef>> dead;
        for (auto& wnode: nodes) {
          auto node = wnode.lock();
          if (node) {
            node->sendParameterUpdate("/master", subPath.fullPath(), ptr);
          } else {
            dead.push_back(wnode);
          }
        }
        for (auto wnode: dead) {
          nodes.erase(wnode);
        }
      }
    }
  }

  return Error::Ok;
}

std::pair<ParameterStorage::RpcValue*, ParameterStorage::RpcValue*> ParameterStorage::findParameter(const names::Path& name, bool create) const
{
  if (name.size() == 0)
    return {};

  RpcValue* current = const_cast<RpcValue*>(&m_parameterRoot);
  RpcValue* parent = nullptr;

  size_t i = 0;
  for (; i < name.size(); i++) {
    std::string key = name.str(static_cast<int>(i));
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
  MINIROS_DEBUG_NAMED("rosparam", "getParam(\"%s\") client=\"%s\", key=\"%s\"", fullKey.c_str(), caller_id.c_str(), key.c_str());

  names::Path fullPath;
  fullPath.fromString(fullKey);

  std::scoped_lock<std::mutex> m_lock(m_parameterLock);
  RpcValue* param = findParameter(fullPath, false).first;
  if (param == nullptr)
    return {};

  return *param;
}

const ParameterStorage::RpcValue* ParameterStorage::subscribeParam(const std::shared_ptr<NodeRef>& node, const std::string& key)
{
  assert(node);
  if (!node)
    return nullptr;
  MINIROS_DEBUG_NAMED("rosparam", "subscribeParam(\"%s\") from %s", key.c_str(), node->debugName().c_str());
  std::string fullKey = miniros::names::resolve(node->id(), key, false);

  names::Path path;
  path.fromString(fullKey);
  std::scoped_lock<std::mutex> m_lock(m_parameterLock);

  auto& listeners = m_parameterListeners[path];
  listeners.insert(node);
  node->addParamSubscription(key);

  return findParameter(path, false).first;
}

bool ParameterStorage::unsubscribeParam(const std::shared_ptr<NodeRef>& node, const std::string& key)
{
  assert(node);
  if (!node)
    return false;
  MINIROS_INFO_NAMED("rosparam", "unsubscribeParam(\"%s\") from %s", key.c_str(), node->debugName().c_str());

  std::string fullKey = miniros::names::resolve(node->id(), key, false);

  names::Path path;
  path.fromString(fullKey);

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  auto it = m_parameterListeners.find(path);
  if (it != m_parameterListeners.end()) {
    it->second.erase(node);
  }
  node->removeParamSubscription(key);
  return true;
}

void ParameterStorage::dropSubscriptions(const std::shared_ptr<NodeRef>& node)
{
  assert(node);
  if (!node)
    return;

  std::scoped_lock<std::mutex> lock(m_parameterLock);
  for (const auto& param: node->getParamSubscriptions()) {
    names::Path path;
    path.fromString(param);

    auto it = m_parameterListeners.find(path);
    if (it != m_parameterListeners.end()) {
      it->second.erase(node);
    }
  }
  node->removeAllParamSubscriptions();
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

void ParameterStorage::dumpParamStateUnsafe(const char* file) const
{
  std::ofstream out(file);

  JsonState jstate;
  JsonSettings jsettings;
  m_parameterRoot.writeJson(out, jstate, jsettings);
}

void ParameterStorage::setDumpParameters(bool dump)
{
  m_dumpParameters = dump;
}

} // namespace master
} // namespace miniros