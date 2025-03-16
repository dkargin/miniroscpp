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
         m_topicPublications.empty() + m_services.empty();
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

std::string NodeRef::getApi() const
{
  return m_api;
}

std::string NodeRef::getHost() const
{
  return m_apiUrl.host;
}

void NodeRef::updateDirectAddress(const network::NetAddress& address)
{
  if (address.type == network::NetAddress::AddressIPv4 && !address.isLocal()) {
    // TODO: we can get local address, like 127.0.0.1.
    // This address is not very practical for other subscribers.
    m_resolvedIp = address.address;
    m_apiUrl.host = address.address;
    MINIROS_INFO("NodeRef(%s) updated address to %s", m_id.c_str(), address.address.c_str());
  }
}


void NodeRef::writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings)
{
  // TODO: Implement
}

} // namespace master
} // namespace miniros