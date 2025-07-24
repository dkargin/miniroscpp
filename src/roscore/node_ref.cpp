//
// Created by dkargin on 3/10/25.
//

#include "node_ref.h"

#include <console.h>

namespace miniros {
namespace master {

NodeRef::NodeRef(const std::string& _id, const std::string& _api)
{
  m_id = _id;
  m_api = _api;
  m_apiUrl.fromString(_api, false);
}

NodeRef::~NodeRef()
{
  MINIROS_INFO("NodeRef::~NodeRef(%s) api=%s", m_id.c_str(), m_api.c_str());
}

void NodeRef::clear()
{
  m_paramSubscriptions.clear();
  m_topicPublications.clear();
  m_topicSubscriptions.clear();
  m_services.clear();
}

bool NodeRef::is_empty() const
{
  return m_paramSubscriptions.empty() && m_topicSubscriptions.empty() &&
         m_topicPublications.empty() && m_services.empty();
}

bool NodeRef::add(Registrations::Type type_, const std::string& key)
{
  if (type_ == Registrations::TOPIC_SUBSCRIPTIONS) {
    if (!m_topicSubscriptions.count(key)) {
      m_topicSubscriptions.insert(key);
    }
  } else if (type_ == Registrations::TOPIC_PUBLICATIONS) {
    if (!m_topicPublications.count(key)) {
      m_topicPublications.insert(key);
    }
  } else if (type_ == Registrations::SERVICE) {
    if (!m_services.count(key)) {
      m_services.insert(key);
    }
  } else if (type_ == Registrations::PARAM_SUBSCRIPTIONS) {
    if (!m_paramSubscriptions.count(key)) {
      m_paramSubscriptions.insert(key);
    }
  } else
    return false;
  return true;
}

bool NodeRef::remove(Registrations::Type type_, const std::string& key)
{
  if (type_ == Registrations::TOPIC_SUBSCRIPTIONS) {
    m_topicSubscriptions.erase(key);
  } else if (type_ == Registrations::TOPIC_PUBLICATIONS) {
    m_topicPublications.erase(key);
  } else if (type_ == Registrations::SERVICE) {
    m_services.erase(key);
  } else if (type_ == Registrations::PARAM_SUBSCRIPTIONS) {
    m_paramSubscriptions.erase(key);
  } else {
    return false;
  }
  return true;
}

network::URL NodeRef::getUrl() const
{
  return m_apiUrl;
}

std::string NodeRef::getApi() const
{
  return m_api;
}

std::string NodeRef::getHost() const
{
  return m_apiUrl.host;
}

void NodeRef::writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings)
{
  // TODO: Implement
}

void NodeRef::updateHost(const std::shared_ptr<HostInfo>& hostInfo)
{
  m_hostInfo = hostInfo;
}

std::weak_ptr<const HostInfo> NodeRef::hostInfo() const
{
  return m_hostInfo;
}

} // namespace master
} // namespace miniros