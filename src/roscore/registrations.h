//
// Created by dkargin on 2/8/25.
//

#ifndef MINIROS_REGISTRATIONS_H
#define MINIROS_REGISTRATIONS_H

#include <map>
#include <string>
#include <vector>
#include <functional>

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
        //PARAM_SUBSCRIPTIONS = 4
      };

    /// Some registration record about network resource.
    struct Record {
        std::string nodeName;
        std::string api;

        friend bool operator == (const Record& lhs, const Record& rhs) {
            return lhs.nodeName == rhs.nodeName && lhs.api == rhs.api;
        }

        friend bool operator != (const Record& lhs, const Record& rhs) {
            return lhs.nodeName != rhs.nodeName || lhs.api != rhs.api;
        }
    };

    using XmlRpcValue = XmlRpc::XmlRpcValue;
    std::map<std::string, std::vector<Record>, std::less<> > map;
    std::map<std::string, Record> service_api_map;

    Registrations(Type type_);

    /// Get URL to service.
    std::string get_service_api(const std::string& service) const;

    std::vector<std::string> getApis(const std::string& key) const;

    /// Iterate over all records with specified key.
    /// @param key - key to search. It is name of topic or service.
    /// @param cb - callback object.
    /// @returns number of objects iterated.
    template <class Callback>
    size_t iterateRecords(const std::string_view& key, const Callback& cb) const
    {
      size_t result = 0;
      auto it = map.find(key);
      if (it != map.end()) {
        for (const Record& obj: it->second) {
          result++;
          if (!cb(obj))
            break;
        }
      }
      return result;
    }

    bool has_key(const std::string& key) const;

    /// Returns a map: resourceName -> array of URIs who provide/need this resource (subscription, publication, service, ...)
    std::map<std::string, std::vector<std::string>> getState() const;

    /// Register new object.
    /// @param key - name of topic or service
    /// @param nodeName - full name of providing node
    /// @param nodeApi - API link of providing node
    /// @param service_api - API link of service.
    Error registerObj(const std::string& key, const std::string& nodeName,
      const std::string& nodeApi, const std::string& service_api="");

    /// Remove registration.
    /// @param key - name of topic or service
    /// @param nodeName - full name of providing node
    /// @param nodeApi - API link of providing node
    /// @param service_api - API link of service.
    ReturnStruct unregisterObj(const std::string& key, const std::string& nodeName,
        const std::string& nodeApi, const std::string& service_api);

    void unregisterAll(const std::string& nodeName);

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
