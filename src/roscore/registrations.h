//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_REGISTRATIONS_H
#define MINIROS_REGISTRATIONS_H

#include <map>
#include <string>
#include <vector>

#include "miniros/errors.h"
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

    ReturnStruct(int _statusCode = 1, const std::string& _statusMessage = "", XmlRpc::XmlRpcValue _value = {})
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

    std::vector<std::string> getApis(const std::string& key) const;

    bool has_key(const std::string& key) const;

    /// Returns a map: resourceName -> array of URIs who provide/need this resource (subscription, publication, service, ...)
    std::map<std::string, std::vector<std::string>> getState() const;

    Error registerObj(const std::string& key, const std::string& caller_id,
      const std::string& caller_api, const std::string& service_api="");

    ReturnStruct unregisterObj(const std::string& key, const std::string& caller_id,
        const std::string& caller_api, const std::string& service_api);

    void unregister_all(const std::string& caller_id);

    Type type() const
    {
        return m_type;
    }
protected:
    const Type m_type;
};


} // namespace master
} // namespace miniros

#endif //MINIROS_REGISTRATIONS_H
