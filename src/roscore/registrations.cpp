//
// Created by dkargin on 2/9/25.
//

#include "registrations.h"

namespace miniros {

Registrations::Registrations(Type type_) : m_type(type_)
{
  assert(type_ == TOPIC_SUBSCRIPTIONS || type_ == TOPIC_PUBLICATIONS
    || type_ == SERVICE || type_ == PARAM_SUBSCRIPTIONS);
}

/// Get URL to service.
std::string Registrations::get_service_api(const std::string& service) const
{
  auto it = service_api_map.find(service);

  if (it != service_api_map.end())
    return it->second.api;
  return {};
}

std::vector<std::string> Registrations::get_apis(const std::string& key)
{
  std::vector<std::string> rtn;
  if (map.count(key)) {
    for (const Record& obj : map[key])
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
  std::map<std::string, std::vector<std::string>> result;

  for (const auto& pair : map) {
    std::vector<std::string>& providers = result[pair.first];
    for (const Record& obj : pair.second) {
      providers.push_back(obj.api);
    }
  }
  return result;
}

void Registrations::registerObj(const std::string& key, const std::string& caller_id,
  const std::string& caller_api, const std::string& service_api)
{
  Record record{caller_id, caller_api};

  auto providerIt = map.find(key);
  if (providerIt != map.end() && service_api.empty()) {
    auto& providers = providerIt->second;

    auto it = std::find(providers.begin(), providers.end(), record);
    if (it == providers.end()) {
      providers.push_back(record);
    }
  } else if (service_api.empty()) {
    map[key].push_back(record);
  }

  if (!service_api.empty()) {
    record.api = service_api;
    service_api_map[key] = record;
  } else if (m_type == Registrations::SERVICE) {
    throw std::runtime_error("service_api must be specified for Registrations.SERVICE");
  }
}

ReturnStruct Registrations::unregisterObj(
  std::string key, std::string caller_id, std::string caller_api, std::string service_api)
{
  std::stringstream ss;
  if (!service_api.empty()) {
    if (service_api_map.empty()) {
      ss << "[" << caller_id << "] is not a provider of [" << key << "]";
      return ReturnStruct(1, ss.str(), XmlRpcValue(0));
    }

    Record tmplist = {caller_id, service_api};

    if (service_api_map[key] != tmplist) {
      ss << "[" << caller_id << "] is no longer the current service api handle for [" << key << "]";
      return ReturnStruct(1, ss.str(), XmlRpcValue(0));
    } else {
      service_api_map.erase(key);
      map.erase(key);
    }
    ss << "Unregistered [" << caller_id << "] as provider of [" << key << "]";
    return ReturnStruct(1, ss.str(), XmlRpcValue(1));
  } else if (m_type == Registrations::SERVICE) {
    throw std::runtime_error("service_api must be specified for Registrations.SERVICE");
    // RAISE THE ROOF
  } else {
    // providers = map[key];
    Record tmplist{caller_id, caller_api};
    auto& providers = map[key];
    for (auto it = providers.begin(); it != providers.end(); ++it) {
      if (it->caller_id == caller_id) {
        providers.erase(it);
        ss << "Unregistered [" << caller_id << "] as provider of [" << key << "]";
        return ReturnStruct(1, ss.str(), XmlRpcValue(1));
      }
    }
    ss << "[" << caller_id << "] is not a known provider of [" << key << "]";
    return ReturnStruct(1, ss.str(), XmlRpcValue(0));
  }
}

void Registrations::unregister_all(const std::string& caller_id)
{
  std::vector<std::string> dead_keys;

  for (auto& [key, providers] : map) {
    std::remove_if(providers.begin(), providers.end(), [&](const Record& r) { return r.caller_id == caller_id; });

    if (providers.empty())
      dead_keys.push_back(key);
  }
  for (std::string k : dead_keys)
    map.erase(k);

  if (m_type == Registrations::SERVICE && !service_api_map.empty()) {
    dead_keys.clear();

    for (const auto& [key, record] : service_api_map) {
      if (record.caller_id == caller_id)
        dead_keys.push_back(key);
    }
    for (const std::string& key : dead_keys)
      service_api_map.erase(key);
  }
}

} // namespace miniros