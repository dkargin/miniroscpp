//
// Created by dkargin on 3/10/25.
//

#ifndef MINIROS_NODE_REF_H
#define MINIROS_NODE_REF_H

#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "miniros/network/url.h"
#include "miniros/macros.h"

#include "registrations.h"


namespace miniros {

namespace network {
struct HostInfo;
}

namespace http {
class HttpClient;
class HttpRequest;
}

class PollSet;

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
    NodeRef(const std::string& _id, const std::string& _api);
    ~NodeRef();

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

    /// Get annotated URL to ClientAPI interface.
    network::URL getUrl() const;

    /// Get default API address.
    std::string getApi() const;

    /// Get hostname.
    /// Hostname is often determined by API URL. In some cases hostname is a direct IP address.
    std::string getHost() const;

    void updateHost(const std::shared_ptr<network::HostInfo>& hostInfo);

    std::weak_ptr<const network::HostInfo> hostInfo() const;

    /// Save state in a json form.
    void writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings);

    /// Lock internal mutex.
    void lock() const;

    /// Unlock internal mutex.
    void unlock() const;

    /// Get access to a subscription set.
    /// It is thread-unsafe method and should be done inside external lock.
    const std::set<std::string>& getSubscriptionsUnsafe() const;

    /// Get access to a publication set.
    /// It is thread-unsafe method and should be done inside external lock.
    const std::set<std::string>& getPublicationsUnsafe() const;

    /// Get access to a service set.
    /// It is thread-unsafe method and should be done inside external lock.
    const std::set<std::string>& getServicesUnsafe() const;

    /// Mark this node as master.
    void setMaster();
    bool isMaster() const;

    /// Enable connection to this node.
    void activateConnection(PollSet* ps);

    Error sendRequest(const std::shared_ptr<http::HttpRequest>& request);

    /// Serialize state into intermediate value.
    void saveState(XmlRpc::XmlRpcValue& storage);

    /// Load state from intermediate value.
    void loadState(const XmlRpc::XmlRpcValue& storage);

protected:
    std::set<std::string> m_paramSubscriptions;
    std::set<std::string> m_topicSubscriptions;
    std::set<std::string> m_topicPublications;
    std::set<std::string> m_services;

    /// Name of a node.
    std::string m_id;
    /// API path.
    /// TODO: maybe it should be removed.
    std::string m_api;

    /// Should be the same as m_api.
    network::URL m_apiUrl;

    std::weak_ptr<network::HostInfo> m_hostInfo;

    /// PID of a node. Checked by separate request to a node.
    int m_pid = 0;

    /// Checked if this node is also master node.
    bool m_isMaster = false;

    mutable std::mutex m_guard;

    std::shared_ptr<http::HttpClient> m_client;
};

using NodeRefPtr = std::shared_ptr<NodeRef>;

} // namespace master
} // namespace miniros

#endif //MINIROS_NODE_REF_H
