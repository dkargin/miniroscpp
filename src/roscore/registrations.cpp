//
// Created by dkargin on 2/9/25.
//

#include <algorithm>
#include <cassert>
#include <sstream>

#include "registrations.h"

#include <errors.h>

namespace miniros {
namespace master {

Registrations::Registrations(Type type_) : m_type(type_)
{
  assert(type_ == TOPIC_SUBSCRIPTIONS || type_ == TOPIC_PUBLICATIONS
    || type_ == SERVICE);
}

/// Get URL to service.
std::string Registrations::get_service_api(const std::string& service) const
{
  auto it = service_api_map.find(service);

  if (it != service_api_map.end())
    return it->second.api;
  return {};
}

std::vector<std::string> Registrations::getApis(const std::string& key) const
{
  std::vector<std::string> rtn;
  auto it = map.find(key);
  if (it != map.end()) {
    for (const Record& obj : it->second)
      rtn.push_back(obj.api);
  }
  return rtn;
}

bool Registrations::has_key(const std::string& key) const
{
  return map.count(key);
}

std::map<std::string, std::vector<std::string>> Registrations::getState() const
{
  // It must return a collection of node names.
  // Also, node duplicates must be cleaned up.
  std::map<std::string, std::vector<std::string>> result;

  for (const auto& pair : map) {
    std::vector<std::string>& providers = result[pair.first];
    for (const Record& obj : pair.second) {
      providers.push_back(obj.nodeName);
    }
  }
  return result;
}

Error Registrations::registerObj(const std::string& key, const std::string& nodeName,
  const std::string& nodeApi, const std::string& service_api)
{
  if (key.empty() || nodeName.empty() || nodeApi.empty())
    return Error::InvalidValue;

  Record record{nodeName, nodeApi};

  assert(!key.empty());
  auto providerIt = map.find(key);
  if (providerIt != map.end() && service_api.empty()) {
    auto& providers = providerIt->second;

    auto it = std::find(providers.begin(), providers.end(), record);
    if (it == providers.end()) {
      providers.push_back(record);
    }
  } else {
    map[key].push_back(record);
  }

  if (!service_api.empty()) {
    record.api = service_api;
    service_api_map[key] = record;
  } else if (m_type == Registrations::SERVICE) {
    throw std::runtime_error("service_api must be specified for Registrations.SERVICE");
  }
  return Error::Ok;
}

ReturnStruct Registrations::unregisterObj(const std::string& key, const std::string& nodeName,
  const std::string& nodeApi, const std::string& service_api)
{
  std::stringstream ss;
  if (!service_api.empty()) {
    if (service_api_map.empty()) {
      ss << "[" << nodeName << "] is not a provider of [" << key << "]";
      return ReturnStruct(1, ss.str(), XmlRpcValue(0));
    }

    Record tmplist = {nodeName, service_api};

    if (service_api_map[key] != tmplist) {
      ss << "[" << nodeName << "] is no longer the current service api handle for [" << key << "]";
      return ReturnStruct(1, ss.str(), XmlRpcValue(0));
    } else {
      service_api_map.erase(key);
      map.erase(key);
    }
    ss << "Unregistered [" << nodeName << "] as provider of [" << key << "]";
    return ReturnStruct(1, ss.str(), XmlRpcValue(1));
  } else if (m_type == Registrations::SERVICE) {
    throw std::runtime_error("service_api must be specified for Registrations.SERVICE");
    // RAISE THE ROOF
  } else {
    // providers = map[key];
    Record tmplist{nodeName, nodeApi};
    auto& providers = map[key];
    for (auto it = providers.begin(); it != providers.end(); ++it) {
      if (it->nodeName == nodeName) {
        providers.erase(it);
        ss << "Unregistered [" << nodeName << "] as provider of [" << key << "]";
        return ReturnStruct(1, ss.str(), XmlRpcValue(1));
      }
    }
    ss << "[" << nodeName << "] is not a known provider of [" << key << "]";
    return ReturnStruct(1, ss.str(), XmlRpcValue(0));
  }
}

void Registrations::unregisterAll(const std::string& nodeName)
{
  std::vector<std::string> dead_keys;

  for (auto& [key, providers] : map) {
    auto newEnd = std::remove_if(providers.begin(), providers.end(),
      [&](const Record& r) {
        return r.nodeName == nodeName;
      });
    providers.erase(newEnd, providers.end());

    if (providers.empty())
      dead_keys.push_back(key);
  }
  for (std::string k : dead_keys)
    map.erase(k);

  if (m_type == Registrations::SERVICE && !service_api_map.empty()) {
    dead_keys.clear();

    for (const auto& [key, record] : service_api_map) {
      if (record.nodeName == nodeName)
        dead_keys.push_back(key);
    }
    for (const std::string& key : dead_keys)
      service_api_map.erase(key);
  }
}

} // namespace master
} // namespace miniros
