//
// Created by dkargin on 3/10/25.
//

#ifndef MINIROS_NODE_REF_H
#define MINIROS_NODE_REF_H

#include <memory>
#include <mutex>
#include <set>
#include <string>

#include "miniros/macros.h"
#include "miniros/network/url.h"

#include "miniros/http/http_client.h"

#include "registrations.h"

namespace miniros {

namespace network {
struct HostInfo;
class NetSocket;
}

namespace http {
class HttpClient;
class HttpRequest;
class XmlRpcRequest;
} // namespace http

class PollSet;

namespace master {

/// NodeRef contains specific data about a node:
///  - publishers, subscribers, services, etc.
///  - ip addresses of the node.
/// It also serves as a wrapper for Client XMLRPC API.
///
/// IP resolution:
/// Each node reports its own api key an id in most requests to master. Master tries to ensure that _id is unique.
/// It will send a shutdown request to other node if new node with the same `id` appears. In this particular situation
/// `id` will not be unique if we consider not only master's database, but all the network. For this reason API address
/// should be used as part of the identification as well.
///
class MINIROS_DECL NodeRef : public std::enable_shared_from_this<NodeRef> {
public:
  /// Representation of high-level state of node.
  struct State {
    enum State_t {
      /// Node has just appeared and no additional information about this node is known.
      Initial,
      /// Lost connection to node or failed to send request.
      Connecting,
      /// Connected to remote node.
      Connected,
      /// Master has managed to contact node by "getPid" request.
      Verified,
      /// Sent shutdown request.
      ShuttingDown,
      /// Node is considered dead. It can be caused either by shutdown request
      /// or by failed attempt to reconnect. Node can stay in this state for some
      /// time before complete removal.
      Dead,
    };

    State() = default;
    State(State_t val) : value_(val) {}

    operator State_t() const
    {
      return value_;
    }

    const char* toString() const;

  protected:
    State_t value_ = State::Initial;
  };

  using RpcValue = XmlRpc::XmlRpcValue;

  enum NodeFlags {
    /// Node is located in the same process.
    NODE_LOCAL = 1 << 1,
    /// This node belongs to another master.
    NODE_FOREIGN = 1 << 2,
    /// This is miniros-based node.
    NODE_MINIROS = 1 << 3,
    /// This node is master.
    NODE_MASTER = 1 << 4,
  };

  NodeRef(const std::string& _id, const std::string& _api);
  ~NodeRef();

  State getState() const;

  void clear();

  /// Check if there are any registrations.
  bool is_empty() const;

  /// Add registration.
  bool add(Registrations::Type type_, const std::string& key);

  /// Remove registration.
  bool remove(Registrations::Type type_, const std::string& key);

  void addParamSubscription(const std::string& key);
  void removeParamSubscription(const std::string& key);
  void removeAllParamSubscriptions();

  const std::string& id() const;

  /// Get annotated URL to ClientAPI interface.
  network::URL getUrl() const;

  /// Get default API address.
  std::string getApi() const;

  /// Get hostname.
  /// Hostname is often determined by API URL. In some cases hostname is a direct IP address.
  std::string getHost() const;

  std::string debugName() const;

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

  /// It is thread-unsafe method and should be done inside external lock.
  std::set<std::string> getParamSubscriptions() const;

  /// Mark this node as local.
  void setLocal();
  bool isLocal() const;

  /// Set local flags of node.
  /// It should be a combination of NodeFlags.
  void setNodeFlags(int flags);

  /// Check if node is suitable for RPC requests.
  bool needRequests() const;

  /// Enable connection to this node.
  /// It will create local m_client instance and initiate connection.
  /// @return Error::Ok if connection was initiated successfully, error code otherwise.
  Error activateConnection(const std::string& callerId, PollSet* ps);

  /// Release HTTP client and drop all related objects.
  void deactivateConnectionUnsafe();

  /// Send shutdown request to remote node.
  /// @param msg - message sent to remote node.
  /// @return Error::Ok if request was queued successfully, error code otherwise.
  Error sendShutdown(const std::string& msg);

  /// Send request to read PID value of remote process.
  Error sendGetPid(const std::string& callerId);

  /// Response method for GetPid request.
  void responseGetPid(int code, const std::string& msg, const RpcValue& data);

  /// Notify client about updated publishers.
  /// @param callerId - name of requester node.
  /// @param topic - name of updated topic
  /// @param update - an array with publishers.
  Error sendPublisherUpdate(const std::string& callerId, const std::string& topic, const RpcValue& update);

  /// Notify node about change in parameters.
  Error sendParameterUpdate(const std::string& callerId, const std::string& param, const RpcValue* value);

  /// Get number of queued requests to this node.
  size_t getQueuedRequests() const;

  /// Get client object.
  std::shared_ptr<http::HttpClient> getClient();

protected:
  using Lock = std::unique_lock<std::mutex>;
  void updateState(State newState, Lock& lock);

  /// Creates client object.
  std::shared_ptr<http::HttpClient> makeClient(PollSet* ps);

  /// Handler for disconnect events from HttpClient.
  void handleDisconnect(const std::weak_ptr<http::HttpClient>& client);

protected:
  std::set<std::string> m_paramSubscriptions;
  std::set<std::string> m_topicSubscriptions;
  std::set<std::string> m_topicPublications;
  std::set<std::string> m_services;

  /// Name of a node.
  const std::string m_id;

  /// API path.
  const std::string m_api;

  /// Should be the same as m_api.
  network::URL m_apiUrl;

  std::weak_ptr<network::HostInfo> m_hostInfo;

  /// PID of a node. Checked by separate request to a node.
  int m_pid = 0;

  int m_flags = 0;

  /// Generic guard for data and state objects.
  mutable std::mutex m_guard;

  std::shared_ptr<http::HttpClient> m_client;
  /// Guard for interaction with http client.
  mutable std::mutex m_clientGuard;

  /// Request to shut down node.
  std::shared_ptr<http::XmlRpcRequest> m_reqShutdown;

  /// Request to get PID of a process.
  std::shared_ptr<http::XmlRpcRequest> m_reqGetPid;

  State m_state = State::Initial;

  /// Collection of active requests.
  std::set<std::shared_ptr<http::HttpRequest>> m_activeRequests;
};

using NodeRefPtr = std::shared_ptr<NodeRef>;

} // namespace master
} // namespace miniros

#endif // MINIROS_NODE_REF_H
