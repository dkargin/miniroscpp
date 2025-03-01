//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_REGISTRATIONS_H
#define MINIROS_REGISTRATIONS_H

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <cassert>
#include <sstream>

#include "miniros/xmlrpcpp/XmlRpcValue.h"

namespace XmlRpc {
class XmlRpcClient;
}
namespace miniros {
namespace master {

struct ReturnStruct
{
    int statusCode;
    std::string statusMessage;
    XmlRpc::XmlRpcValue value;

    ReturnStruct(int _statusCode = 1, std::string _statusMessage = "", XmlRpc::XmlRpcValue _value = {})
    {
        statusCode = _statusCode;
        statusMessage = _statusMessage;
        value = _value;
    }
};

/// A collection of registrations.
class Registrations
{
public:
    enum Type {
        TOPIC_SUBSCRIPTIONS = 1,
        TOPIC_PUBLICATIONS,
        SERVICE = 3,
        PARAM_SUBSCRIPTIONS = 4
      };

    /// Some registration record about network resource.
    struct Record {
        std::string caller_id;
        std::string api;

        friend bool operator == (const Record& lhs, const Record& rhs) {
            return lhs.caller_id == rhs.caller_id && lhs.api == rhs.api;
        }

        friend bool operator != (const Record& lhs, const Record& rhs) {
            return lhs.caller_id != rhs.caller_id || lhs.api != rhs.api;
        }
    };

    using XmlRpcValue = XmlRpc::XmlRpcValue;
    std::map<std::string, std::vector<Record> > map;
    std::map<std::string, Record> service_api_map;

    Registrations(Type type_);

    /// Get URL to service.
    std::string get_service_api(const std::string& service) const;

    std::vector<std::string> get_apis(const std::string& key);

    bool has_key(const std::string& key) const;

    std::map<std::string, std::vector<std::string>> getState() const;

    void registerObj(const std::string& key, const std::string& caller_id,
      const std::string& caller_api, const std::string& service_api="");

    ReturnStruct unregisterObj(std::string key, std::string caller_id, std::string caller_api, std::string service_api);

    void unregister_all(const std::string& caller_id);

    Type type() const
    {
        return m_type;
    }
protected:
    const Type m_type;
};


/// Container for node registration information. Used in master's
/// self.nodes data structure.  This is effectively a reference
/// counter for the node registration information: when the
/// subscriptions and publications are empty the node registration can be deleted.
class NodeRef
{
protected:
    std::set<std::string> param_subscriptions;
    std::set<std::string> topic_subscriptions;
    std::set<std::string> topic_publications;
    std::set<std::string> services;

public:
    using RpcValue = XmlRpc::XmlRpcValue;

    std::string id;

    /// URL of a node.
    std::string api;

    NodeRef() = default;

    NodeRef(const std::string& _id, const std::string& _api)
    {
        id = _id;
        api = _api;
    }

    void clear()
    {
        param_subscriptions.clear();
        topic_publications.clear();
        topic_subscriptions.clear();
        services.clear();
    }

    bool is_empty() const
    {
        return param_subscriptions.empty() && topic_subscriptions.empty() && topic_publications.empty() + services.empty();
    }

    bool add(Registrations::Type type_, const std::string& key)
    {
        if (type_ == Registrations::TOPIC_SUBSCRIPTIONS)
        {
            if (!topic_subscriptions.count(key))
            {
                topic_subscriptions.insert(key);
            }
        }
        else if (type_ == Registrations::TOPIC_PUBLICATIONS)
        {
            if (!topic_publications.count(key))
            {
                topic_publications.insert(key);
            }
        }
        else if (type_ == Registrations::SERVICE)
        {
            if (!services.count(key))
            {
                services.insert(key);
            }
        }
        else if (type_ == Registrations::PARAM_SUBSCRIPTIONS)
        {
            if (!param_subscriptions.count(key))
            {
                param_subscriptions.insert(key);
            }
        }
        else
            return false;
        return true;
    }

    bool remove(Registrations::Type type_, const std::string& key)
    {
        if (type_ == Registrations::TOPIC_SUBSCRIPTIONS)
        {
            topic_subscriptions.erase(key);
        }
        else if (type_ == Registrations::TOPIC_PUBLICATIONS)
        {
            topic_publications.erase(key);
        }
        else if (type_ == Registrations::SERVICE)
        {
            services.erase(key);
        }
        else if (type_ == Registrations::PARAM_SUBSCRIPTIONS)
        {
            param_subscriptions.erase(key);
        }
        else
        {
            return false;
        }
        return true;
    }

    void shutdown_node_task(const std::string& api, int caller_id, const std::string& reason)
    {
        //XmlRpcManager m = new XmlRpcManager();
        //m.shutdown();
    }
};

} // namespace master
} // namespace miniros

#endif //MINIROS_REGISTRATIONS_H
