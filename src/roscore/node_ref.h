//
// Created by dkargin on 3/10/25.
//

#ifndef MINIROS_NODE_REF_H
#define MINIROS_NODE_REF_H

#include <memory>
#include <set>
#include <string>

#include <miniros/macros.h>
#include <miniros/transport/net_address.h>
#include <miniros/transport/url.h>

#include "registrations.h"


namespace miniros {

namespace master {

/// NodeRef contains specific data about a node: publishers, subscribers, services, etc.
/// It also stores information about ip addresses of the node.
///
/// IP resolution:
/// Each node reports its own api key an id in most requests to master. Master tries to ensure that _id is unique.
/// It will send a shutdown request to other node if new node with the same `id` appears. In this particular situation `id`
/// will not be unique if we consider not only master's database, but all the network. For this reason API address
/// should be used as part of the identification as well.
///
class MINIROS_DECL NodeRef
{
public:
    NodeRef() = default;

    NodeRef(const std::string& _id, const std::string& _api);

    void clear();

    /// Check if there are any registrations.
    bool is_empty() const;

    /// Add registration.
    bool add(Registrations::Type type_, const std::string& key);

    /// Remove registration.
    bool remove(Registrations::Type type_, const std::string& key);

    const std::string& id() const
    {
        return m_id;
    }

    /// Get default API address.
    std::string getApi() const;

    /// Get resolved IP url, which is accessible by specified Node.
    std::string getResolvedApiFor(bool useIP, const std::shared_ptr<NodeRef>& other) const;

    std::string getResolvedApiFor(bool useIP, const network::NetAddress& requesterIp) const;


    /// Update direct IP address of a node.
    void updateDirectAddress(const network::NetAddress& address);

    /// Save state in a json form.
    void writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings);

protected:
    std::set<std::string> m_paramSubscriptions;
    std::set<std::string> m_topicSubscriptions;
    std::set<std::string> m_topicPublications;
    std::set<std::string> m_services;

    std::string m_id;
    std::string m_api;

    network::URL m_apiUrl;

    /// Resolved IP of a node.
    std::string m_resolvedIp;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_NODE_REF_H
