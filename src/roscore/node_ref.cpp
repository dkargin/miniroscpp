//
// Created by dkargin on 3/10/25.
//

#include "node_ref.h"

#include "miniros/http/xmlrpc_request.h"
#include "miniros/http/http_client.h"
#include "miniros/errors.h"

#include <cassert>
#include <console.h>

namespace miniros {
namespace master {

NodeRef::NodeRef(const std::string& _id, const std::string& _api)
  :m_id(_id), m_api(_api)
{
  m_apiUrl.fromString(_api, false);
}

NodeRef::~NodeRef()
{
  std::unique_lock lock(m_guard);
  MINIROS_INFO("NodeRef::~NodeRef(\"%s\") api=%s", m_id.c_str(), m_api.c_str());
}

void NodeRef::clear()
{
  std::unique_lock lock(m_guard);
  m_paramSubscriptions.clear();
  m_topicPublications.clear();
  m_topicSubscriptions.clear();
  m_services.clear();
}

NodeRef::State NodeRef::getState() const
{
  std::unique_lock lock(m_guard);
  return m_state;
}


bool NodeRef::is_empty() const
{
  std::unique_lock lock(m_guard);
  return m_paramSubscriptions.empty() && m_topicSubscriptions.empty() &&
         m_topicPublications.empty() && m_services.empty();
}

bool NodeRef::add(Registrations::Type type_, const std::string& key)
{
  std::unique_lock lock(m_guard);
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
  std::unique_lock lock(m_guard);
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
  std::unique_lock lock(m_guard);
  return m_apiUrl;
}

std::string NodeRef::getApi() const
{
  std::unique_lock lock(m_guard);
  return m_api;
}

std::string NodeRef::getHost() const
{
  std::unique_lock lock(m_guard);
  return m_apiUrl.host;
}

void NodeRef::writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings)
{
  // TODO: Implement
}

void NodeRef::updateHost(const std::shared_ptr<network::HostInfo>& hostInfo)
{
  std::unique_lock lock(m_guard);
  m_hostInfo = hostInfo;
}

std::weak_ptr<const network::HostInfo> NodeRef::hostInfo() const
{
  std::unique_lock lock(m_guard);
  return m_hostInfo;
}

void NodeRef::lock() const
{
  m_guard.lock();
}

void NodeRef::unlock() const
{
  m_guard.unlock();
}

const std::set<std::string>& NodeRef::getSubscriptionsUnsafe() const
{
  return m_topicSubscriptions;
}

const std::set<std::string>& NodeRef::getPublicationsUnsafe() const
{
  return m_topicPublications;
}

const std::set<std::string>& NodeRef::getServicesUnsafe() const
{
  return m_services;
}

void NodeRef::setLocal()
{
  std::unique_lock lock(m_guard);
  m_state = State::Local;
}

bool NodeRef::isLocal() const
{
  std::unique_lock lock(m_guard);
  return m_state == State::Local;
}

Error NodeRef::activateConnection(PollSet* ps)
{
  if (!ps) {
    return Error::InvalidValue;
  }

  if (isLocal())
    return Error::Ok;

  {
    std::unique_lock lock(m_guard);

    // Create client if it doesn't exist
    if (!m_client) {
      m_client = std::make_shared<http::HttpClient>(ps);
      m_client->onDisconnect = [this](std::shared_ptr<network::NetSocket> socket) {
        http::HttpClient::DisconnectResponse dr;
        dr.reconnect = true;
        dr.reconnectTimeout = 5;
        std::unique_lock lock(m_guard);
        MINIROS_INFO("NodeRef(%s) - disconnected. Initiating reconnect", m_id.c_str());
        m_state = State::Connecting;
        return dr;
      };

      m_client->onConnect = [this](std::shared_ptr<network::NetSocket> socket) {
        MINIROS_INFO("NodeRef(%s) - connected", m_id.c_str());
        m_state = State::Connected;
        return true;
      };

      m_client->onResponse = [this](const std::shared_ptr<http::HttpRequest>& req) {
        m_activeRequests.erase(req);
      };
    }

    // Check if URL is valid
    if (m_apiUrl.host.empty() || m_apiUrl.port == 0) {
      return Error::InvalidAddress;
    }

    // Initiate connection to the remote host
    Error connectErr = m_client->connect(m_apiUrl.host, static_cast<int>(m_apiUrl.port));
    if (connectErr != Error::Ok && connectErr != Error::WouldBlock) {
      // WouldBlock is expected for non-blocking connections, so only log unexpected errors
      MINIROS_WARN("NodeRef::activateConnection: Unexpected error initiating connection for node \"%s\" to %s:%u: %s",
                   m_id.c_str(), m_apiUrl.host.c_str(), m_apiUrl.port, connectErr.toString());
      return connectErr;
    }

    m_state = State::Connecting;
  }
  //sendGetPid();
  return Error::Ok;
}

bool NodeRef::needRequests() const
{
  std::unique_lock lock(m_guard);
  if (m_state == State::ShuttingDown)
    return false;
  if (m_state == State::Local)
    return false;
  return true;
}

Error NodeRef::sendPublisherUpdate(const std::string& callerId, const std::string& topic, const RpcValue& update)
{
  if (!needRequests())
    return Error::Ok;

  // Generate the request body (no parameters needed for shutdown)
  // Check if client exists
  assert(m_client);
  if (!m_client) {
    MINIROS_WARN("NodeRef::sendPublisherUpdate: No client available for node \"%s\"", m_id.c_str());
    return Error::NotConnected;
  }

  auto request = makeRequest("publisherUpdate");
  request->setParams(callerId, topic, update);
  m_activeRequests.insert(request);
  MINIROS_INFO("NodeRef(%s)::sendPublisherUpdate(%s)", this->getApi().c_str(), topic.c_str());

  return m_client->enqueueRequest(request);
}


Error NodeRef::sendParameterUpdate(const std::string& callerId, const std::string& param, const XmlRpc::XmlRpcValue* value)
{
  if (!needRequests())
    return Error::Ok;

  // Generate the request body (no parameters needed for shutdown)
  // Check if client exists
  assert(m_client);
  if (!m_client) {
    MINIROS_WARN("NodeRef::sendShutdown: No client available for node \"%s\"", m_id.c_str());
    return Error::NotConnected;
  }

  auto request = makeRequest("paramUpdate");
  request->setParams(callerId, param, value ? *value : RpcValue::Dict());
  m_activeRequests.insert(request);

  MINIROS_INFO("NodeRef(%s)::sendParameterUpdate(%s)", this->getApi().c_str(), param.c_str());
  return m_client->enqueueRequest(request);
}


Error NodeRef::sendShutdown(const std::string& msg)
{
  if (!needRequests())
    return Error::Ok;

  // Generate the request body (no parameters needed for shutdown)
  // Check if client exists
  if (!m_client) {
    MINIROS_WARN("NodeRef::sendShutdown: No client available for node \"%s\"", m_id.c_str());
    return Error::NotConnected;
  }

  {
    std::unique_lock lock(m_guard);
    if (!m_reqShutdown) {
      // Create XML-RPC request for shutdown
      m_reqShutdown = makeRequest("shutdown");
    } else if (m_reqShutdown->state() != http::HttpRequest::State::Idle) {
      // Already sent request.
      return Error::Ok;
    }
  }

  m_reqShutdown->setParams(msg);
  if (Error err = m_client->enqueueRequest(m_reqShutdown); !err) {
    return err;
  }
  m_state = State::ShuttingDown;
  return Error::Ok;
}

Error NodeRef::sendGetPid()
{
  if (!needRequests())
    return Error::Ok;

  // Check if client exists
  assert(m_client);
  if (!m_client) {
    MINIROS_ERROR("NodeRef::sendGetPid: No client available for node \"%s\"", m_id.c_str());
    return Error::NotConnected;
  }

  {
    std::unique_lock lock(m_guard);
    if (!m_reqGetPid) {

      m_reqGetPid  = makeRequest("getPid");
      m_reqGetPid->setParamArray(XmlRpc::XmlRpcValue::Array(0));
      m_reqGetPid->generateRequestBody();

      m_reqGetPid->onComplete = [this] (int code, const XmlRpc::XmlRpcValue& data, const std::string& msg) {
        if (code && data.getType() == XmlRpc::XmlRpcValue::TypeInt) {
          m_pid = data.as<int>();
          if (m_state == State::Connecting)
            m_state = State::Verified;
        }
      };
    } else if (m_reqShutdown->state() != http::HttpRequest::State::Idle) {
      // Already sent request.
      return Error::Ok;
    }
  }

  MINIROS_INFO("NodeRef(%s)::sendGetPid()", getApi().c_str());

  // Generate the request body (no parameters needed for shutdown)
  return m_client->enqueueRequest(m_reqGetPid);
}

std::shared_ptr<http::XmlRpcRequest> NodeRef::makeRequest(const std::string& function)
{
  // Determine the path for XML-RPC endpoint
  std::string path = m_apiUrl.path;
  if (path.empty() || path == "/") {
    path = "/RPC2";
  }

  auto request = std::make_shared<http::XmlRpcRequest>("getPid", path.c_str());
  return request;
}

size_t NodeRef::getQueuedRequests() const
{
  std::unique_lock lock(m_guard);
  if (!m_client)
    return 0;
  return m_client->getQueuedRequests();
}

} // namespace master
} // namespace miniros