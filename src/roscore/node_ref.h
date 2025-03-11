//
// Created by dkargin on 3/10/25.
//

#ifndef MINIROS_NODE_REF_H
#define MINIROS_NODE_REF_H

#include <set>
#include <string>

#include <miniros/macros.h>
#include <miniros/transport/network.h>

#include "registrations.h"


namespace miniros {

namespace master {

/// Container for node registration information. Used in master's
/// self.nodes data structure.  This is effectively a reference
/// counter for the node registration information: when the
/// subscriptions and publications are empty the node registration can be deleted.
class MINIROS_DECL NodeRef
{
public:
    NodeRef() = default;

    NodeRef(const std::string& _id, const std::string& _api);

    void clear();

    bool is_empty() const;

    /// Add registration.
    bool add(Registrations::Type type_, const std::string& key);

    /// Remove registration.
    bool remove(Registrations::Type type_, const std::string& key);

    void shutdown_node_task(const std::string& api, int caller_id, const std::string& reason);

    /// Get default API address.
    std::string getApi() const;

    /// Update direct IP address of a node.
    void updateDirectAddress(const network::NetAddress& address);

protected:
    std::set<std::string> m_paramSubscriptions;
    std::set<std::string> m_topicSubscriptions;
    std::set<std::string> m_topicPublications;
    std::set<std::string> m_services;

    std::string id;

    /// Default URL of a node, as reported by node itself.
    std::string m_api;

    /// Resolved IP of a node.
    std::string m_resolvedIp;
};

} // namespace master
} // namespace miniros

#endif //MINIROS_NODE_REF_H
