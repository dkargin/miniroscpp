//
// Created by dkargin on 3/10/25.
//

#include "node_ref.h"

#include "miniros/http/xmlrpc_request.h"
#include "miniros/http/http_client.h"
#include "miniros/errors.h"

#include <cassert>
#include <console.h>
#include <cstring>
#include <utility>
#include <vector>

namespace miniros {
namespace master {

const char* NodeRef::State::toString() const
{
  switch (value_) {
    case Initial:
      return "Initial";
    case Connecting:
      return "Connecting";
    case Connected:
      return "Connected";
    case Verified:
      return "Verified";
    case Restoring:
      return "Restoring";
    case Recovering:
      return "Recovering";
    case ShuttingDown:
      return "ShuttingDown";
    case Dead:
      return "Dead";
  }
  return "Unknown";
}

bool NodeRef::State::fromString(const char* name)
{
  if (!name)
    return false;
  if (std::strcmp(name, "Initial") == 0) { value_ = Initial; return true; }
  if (std::strcmp(name, "Connecting") == 0) { value_ = Connecting; return true; }
  if (std::strcmp(name, "Connected") == 0) { value_ = Connected; return true; }
  if (std::strcmp(name, "Verified") == 0) { value_ = Verified; return true; }
  if (std::strcmp(name, "Restoring") == 0) { value_ = Restoring; return true; }
  if (std::strcmp(name, "Recovering") == 0) { value_ = Recovering; return true; }
  if (std::strcmp(name, "ShuttingDown") == 0) { value_ = ShuttingDown; return true; }
  if (std::strcmp(name, "Dead") == 0) { value_ = Dead; return true; }
  return false;
}

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

bool NodeRef::isEmpty() const
{
  std::unique_lock lock(m_guard);
  /// m_paramSubscriptions are not checked here, since node is not guaranteed to properly unsubscribe from all its
  /// parameters on exit.
  return m_topicSubscriptions.empty() &&
         m_topicPublications.empty() && m_services.empty() /* && m_paramSubscriptions.empty() */;
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
  } else
    return false;
  return true;
}

void NodeRef::addParamSubscription(const std::string& key)
{
  std::unique_lock lock(m_guard);
  m_paramSubscriptions.insert(key);
}

void NodeRef::removeParamSubscription(const std::string& key)
{
  std::unique_lock lock(m_guard);
  m_paramSubscriptions.erase(key);
}

void NodeRef::removeAllParamSubscriptions()
{
  std::unique_lock lock(m_guard);
  m_paramSubscriptions.clear();
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

const std::set<std::string>& NodeRef::getSubscriptionsLocked(const Lock&) const
{
  return m_topicSubscriptions;
}

const std::set<std::string>& NodeRef::getPublicationsLocked(const Lock&) const
{
  return m_topicPublications;
}

const std::set<std::string>& NodeRef::getServicesLocked(const Lock&) const
{
  return m_services;
}

std::set<std::string> NodeRef::getParamSubscriptions() const
{
  return m_paramSubscriptions;
}

void NodeRef::setLocal()
{
  std::unique_lock lock(m_guard);
  m_flags |= NODE_LOCAL;
}

bool NodeRef::isLocal() const
{
  std::unique_lock lock(m_guard);
  return m_flags & NODE_LOCAL;
}

void NodeRef::setNodeFlags(int flags)
{
  std::unique_lock lock(m_guard);
  m_flags |= flags & (NODE_LOCAL | NODE_MASTER | NODE_FOREIGN | NODE_MINIROS);
}

int NodeRef::getNodeFlags() const
{
  std::unique_lock lock(m_guard);
  return m_flags;
}

std::string NodeRef::debugName() const
{
  std::stringstream ss;
  ss << "NodeRef[" << id();

  if (!m_api.empty()) {
    ss << " " << m_api;
  }
  if (m_pid) {
    ss << " pid=" << m_pid;
  }
  ss << "]";
  return ss.str();
}

std::shared_ptr<http::HttpClient> NodeRef::makeClient(PollSet* ps)
{
  assert(ps);
  if (!ps)
    return {};
  auto client = std::make_shared<http::HttpClient>(ps);
  std::weak_ptr<NodeRef> wnode = weak_from_this();
  std::weak_ptr<http::HttpClient> wclient = client;
  client->setDisconnectHandler(
    [wnode, wclient](std::shared_ptr<network::NetSocket> socket, http::HttpClient::State state, Error err)
    {
      if (auto node = wnode.lock()) {
        node->handleDisconnect(wclient, err);
      }
    });

  client->setConnectHandler([wnode](std::shared_ptr<network::NetSocket> socket) {
    if (auto node = wnode.lock()) {
      MINIROS_INFO("%s - connected", node->debugName().c_str());
      std::string callerId;
      {
        std::unique_lock<std::mutex> lock(node->m_guard);
        // Keep Restoring through TCP connect; getPid advances it to Recovering.
        if (node->m_state != State::Restoring)
          node->updateState(State::Connected, lock);
        callerId = node->m_callerId;
      }
      // Verify remote nodes after (re)connect; in-process nodes skip getPid.
      if (!callerId.empty() && !node->isLocal())
        node->sendGetPid(callerId);
      return true;
    }
    return false;
  });

  client->setResponseHandler([wnode](const std::shared_ptr<http::HttpRequest>& req) {
    if (auto node = wnode.lock()) {
      node->m_activeRequests.erase(req);
    }
  });
  return client;
}


Error NodeRef::activateConnection(const std::string& callerId, PollSet* ps)
{
  if (!ps) {
    return Error::InvalidValue;
  }

  if (isLocal())
    return Error::Ok;

  network::URL url;

  // Check if URL is valid
  {
    std::unique_lock lock(m_guard);

    if (m_apiUrl.host.empty() || m_apiUrl.port == 0) {
      return Error::InvalidAddress;
    }
    url = m_apiUrl;
    m_callerId = callerId;
  }

  std::shared_ptr<http::HttpClient> client;
  {
    // Create client if it doesn't exist
    std::unique_lock clock(m_clientGuard);
    if (m_client)
      return Error::Ok;

    client = makeClient(ps);
    // Initiate connection to the remote host
    Error connectErr = client->connect(url.host, static_cast<int>(url.port));
    if (connectErr != Error::Ok && connectErr != Error::WouldBlock) {
      // WouldBlock is expected for non-blocking connections, so only log unexpected errors
      MINIROS_WARN("%s::activateConnection: Unexpected error initiating connection to %s:%u: %s",
                   debugName().c_str(), url.host.c_str(), url.port, connectErr.toString());
      return connectErr;
    }
    m_client = client;

    std::unique_lock lock(m_guard);
    // Restoring nodes stay Restoring until getPid; ordinary nodes go Connecting.
    if (m_state != State::Restoring)
      updateState(State::Connecting, lock);
  }

  // getPid is sent from the connect handler once the socket is up.
  return Error::Ok;
}

void NodeRef::handleDisconnect(const std::weak_ptr<http::HttpClient>& wclient, Error disconnectError)
{
  auto client = wclient.lock();
  if (!client) {
    MINIROS_ERROR("%s - http client is lost during disconnect at state=%s", debugName().c_str(), m_state.toString());
    return;
  }

  // Lock order matches activateConnection: m_clientGuard then m_guard.
  ClientLock clientLock(m_clientGuard);
  GuardLock lock(m_guard);

  if (m_state == State::ShuttingDown || m_state == State::Dead) {
    deactivateConnectionUnsafe(lock, clientLock);
    MINIROS_INFO("%s - disconnected at state=%s err=%s, HttpClient state=%s",
                 debugName().c_str(), m_state.toString(), disconnectError.toString(),
                 client->getState().toString());
    updateState(State::Dead, lock);
    return;
  }

  // Connection refused means nothing is listening on the node's Slave API port —
  // typically the process exited. Do not keep retrying.
  // Cap other transient failures so we cannot spin reconnect forever.
  constexpr int kMaxReconnectAttempts = 3;
  const int attempts = client->getReconnectAttempts();
  const bool permanentFailure =
    disconnectError == Error::ConnectionRefused ||
    disconnectError == Error::InvalidAddress ||
    attempts >= kMaxReconnectAttempts;

  if (permanentFailure) {
    MINIROS_WARN("%s - giving up reconnect (err=%s attempts=%d); marking node dead",
                 debugName().c_str(), disconnectError.toString(), attempts);
    deactivateConnectionUnsafe(lock, clientLock);
    updateState(State::Dead, lock);
    return;
  }

  if (Error err = client->reconnect(5)) {
    MINIROS_INFO("%s - disconnected at state %s err=%s. Initiating reconnect (attempt %d)",
                 debugName().c_str(), m_state.toString(), disconnectError.toString(),
                 client->getReconnectAttempts());
    if (m_state != State::Restoring && m_state != State::Recovering)
      updateState(State::Connecting, lock);
  } else {
    MINIROS_INFO("%s - disconnected at state %s. Failed to initiate reconnect: %s",
                 debugName().c_str(), m_state.toString(), err.toString());
    deactivateConnectionUnsafe(lock, clientLock);
    updateState(State::Dead, lock);
  }
}


void NodeRef::deactivateConnectionUnsafe(GuardLock& /*lock*/, ClientLock& /*clientLock*/)
{
  m_client.reset();
  m_reqGetPid.reset();
  m_reqShutdown.reset();
  m_reqGetPublications.reset();
  m_reqGetSubscriptions.reset();
  m_activeRequests.clear();
  m_pid = 0;
}


bool NodeRef::needRequests() const
{
  std::unique_lock lock(m_guard);
  if (m_state == State::ShuttingDown)
    return false;
  return true;
}


void NodeRef::updateState(State newState, GuardLock& /*lock*/)
{
  if (newState == m_state)
    return;

  if (m_state == State::ShuttingDown && newState != State::Dead) {
    MINIROS_ERROR("%s::updateState(%s) from %s - unexpected transition", debugName().c_str(), newState.toString(), m_state.toString());
  } else {
    MINIROS_INFO("%s::updateState(%s) from %s", debugName().c_str(), newState.toString(), m_state.toString());
  }
  m_state = newState;
}

const std::string& NodeRef::id() const
{
  return m_id;
}

Error NodeRef::sendPublisherUpdate(const std::string& callerId, const std::string& topic, const RpcValue& update)
{
  if (!needRequests())
    return Error::Ok;


  auto client = getClient();
  if (!client) {
    MINIROS_WARN("NodeRef::sendPublisherUpdate: No client available for node \"%s\"", m_id.c_str());
    return Error::NotConnected;
  }

  auto request = http::makeRequest(m_apiUrl.path, "publisherUpdate");
  request->setParams(callerId, topic, update);
  MINIROS_INFO("%s::sendPublisherUpdate(%s)", debugName().c_str(), topic.c_str());
  m_activeRequests.insert(request);
  return client->enqueueRequest(request);
}

Error NodeRef::sendParameterUpdate(const std::string& callerId, const std::string& param, const XmlRpc::XmlRpcValue* value)
{
  if (!needRequests())
    return Error::Ok;

  auto client = getClient();
  if (!client) {
    MINIROS_WARN("%s::sendParameterUpdate: No client available", debugName().c_str());
    return Error::NotConnected;
  }

  auto request = http::makeRequest(m_apiUrl.path, "paramUpdate");
  request->setParams(callerId, param, value ? *value : RpcValue::Dict());

  MINIROS_INFO("%s::sendParameterUpdate(%s)", debugName().c_str(), param.c_str());
  m_activeRequests.insert(request);
  return client->enqueueRequest(request);
}

Error NodeRef::sendShutdown(const std::string& msg)
{
  if (!needRequests())
    return Error::Ok;

  auto client = getClient();
  if (!client) {
    MINIROS_WARN("%s::sendShutdown: No client available", debugName().c_str());
    return Error::NotConnected;
  }

  {
    std::unique_lock lock(m_guard);
    if (!m_reqShutdown) {
      // Create XML-RPC request for shutdown
      m_reqShutdown = http::makeRequest(m_apiUrl.path, "shutdown");
    } else if (m_reqShutdown->state() != http::HttpRequest::State::Idle) {
      // Already sent request.
      return Error::Ok;
    }
    m_reqShutdown->setParams(msg);
  }

  m_activeRequests.insert(m_reqShutdown);
  client->enqueueRequest(m_reqShutdown);
  std::unique_lock lock(m_guard);
  updateState(State::ShuttingDown, lock);
  return Error::Ok;
}

std::shared_ptr<http::HttpClient> NodeRef::getClient()
{
  std::unique_lock lock(m_clientGuard);
  return m_client;
}

Error NodeRef::sendGetPid(const std::string& callerId)
{
  if (!needRequests())
    return Error::Ok;

  // Same-process nodes are always alive; no Slave API round-trip.
  if (isLocal())
    return Error::Ok;

  auto client = getClient();
  if (!client) {
    MINIROS_ERROR("%s::sendGetPid: No client available", debugName().c_str());
    return Error::NotConnected;
  }

  std::shared_ptr<http::XmlRpcRequest> reqGetPid;
  {
    std::unique_lock lock(m_guard);
    if (!callerId.empty())
      m_callerId = callerId;

    if (!m_reqGetPid) {
      m_reqGetPid = http::makeRequest(m_apiUrl.path, "getPid");
      m_reqGetPid->setParams(m_callerId);
      m_reqGetPid->generateRequestBody();
      std::weak_ptr<NodeRef> wnode = this->shared_from_this();
      m_reqGetPid->onComplete = [wnode] (int code, const std::string& msg, const RpcValue& data) {
        if (auto node = wnode.lock())
          node->responseGetPid(code, msg, data);
      };
      m_reqGetPid->setFailureCallback([wnode]() {
        if (auto node = wnode.lock()) {
          std::shared_ptr<http::XmlRpcRequest> req;
          {
            std::unique_lock lock(node->m_guard);
            req = node->m_reqGetPid;
          }
          if (req)
            node->m_activeRequests.erase(req);
        }
      });
    } else {
      auto s = m_reqGetPid->state();
      if (s != http::HttpRequest::State::Idle && s != http::HttpRequest::State::Done) {
        // Already in flight.
        return Error::Ok;
      }
      // Reuse completed request object.
      m_reqGetPid->resetResponse();
      m_reqGetPid->updateState(http::HttpRequest::State::Idle);
    }
    reqGetPid = m_reqGetPid;
  }

  MINIROS_INFO("%s::sendGetPid()", debugName().c_str());
  m_activeRequests.insert(reqGetPid);
  return client->enqueueRequest(reqGetPid);
}

void NodeRef::responseGetPid(int code, const std::string& msg, const RpcValue& data)
{
  if (code && data.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    std::unique_lock lock(m_guard);
    m_pid = data.as<int>();
    MINIROS_INFO("%s::responseGetPid() pid=%d", debugName().c_str(), m_pid);
    if (m_state == State::Restoring)
      updateState(State::Recovering, lock);
    else if (m_state == State::Connecting || m_state == State::Connected)
      updateState(State::Verified, lock);
  } else {
    MINIROS_ERROR("%s::responseGetPid unexpected response code=%d data=%s msg=%s",
                  debugName().c_str(), code, data.toJsonStr().c_str(), msg.c_str());
  }
}

Error NodeRef::sendGetPublications(const std::string& callerId,
  std::function<void(int, const std::string&, const RpcValue&)> onComplete)
{
  if (!needRequests())
    return Error::Ok;

  auto client = getClient();
  if (!client) {
    MINIROS_ERROR("%s::sendGetPublications: No client available", debugName().c_str());
    return Error::NotConnected;
  }

  std::shared_ptr<http::XmlRpcRequest> req;
  {
    std::unique_lock lock(m_guard);
    if (!callerId.empty())
      m_callerId = callerId;

    if (!m_reqGetPublications) {
      m_reqGetPublications = http::makeRequest(m_apiUrl.path, "getPublications");
    } else {
      auto s = m_reqGetPublications->state();
      if (s != http::HttpRequest::State::Idle && s != http::HttpRequest::State::Done)
        return Error::Ok;
      m_reqGetPublications->resetResponse();
      m_reqGetPublications->updateState(http::HttpRequest::State::Idle);
    }

    m_reqGetPublications->setParams(m_callerId);
    m_reqGetPublications->generateRequestBody();
    m_reqGetPublications->onComplete = std::move(onComplete);
    std::weak_ptr<NodeRef> wnode = shared_from_this();
    m_reqGetPublications->setFailureCallback([wnode]() {
      if (auto node = wnode.lock()) {
        std::shared_ptr<http::XmlRpcRequest> r;
        {
          std::unique_lock lock(node->m_guard);
          r = node->m_reqGetPublications;
        }
        if (r)
          node->m_activeRequests.erase(r);
      }
    });
    req = m_reqGetPublications;
  }

  MINIROS_INFO("%s::sendGetPublications()", debugName().c_str());
  m_activeRequests.insert(req);
  return client->enqueueRequest(req);
}

Error NodeRef::sendGetSubscriptions(const std::string& callerId,
  std::function<void(int, const std::string&, const RpcValue&)> onComplete)
{
  if (!needRequests())
    return Error::Ok;

  auto client = getClient();
  if (!client) {
    MINIROS_ERROR("%s::sendGetSubscriptions: No client available", debugName().c_str());
    return Error::NotConnected;
  }

  std::shared_ptr<http::XmlRpcRequest> req;
  {
    std::unique_lock lock(m_guard);
    if (!callerId.empty())
      m_callerId = callerId;

    if (!m_reqGetSubscriptions) {
      m_reqGetSubscriptions = http::makeRequest(m_apiUrl.path, "getSubscriptions");
    } else {
      auto s = m_reqGetSubscriptions->state();
      if (s != http::HttpRequest::State::Idle && s != http::HttpRequest::State::Done)
        return Error::Ok;
      m_reqGetSubscriptions->resetResponse();
      m_reqGetSubscriptions->updateState(http::HttpRequest::State::Idle);
    }

    m_reqGetSubscriptions->setParams(m_callerId);
    m_reqGetSubscriptions->generateRequestBody();
    m_reqGetSubscriptions->onComplete = std::move(onComplete);
    std::weak_ptr<NodeRef> wnode = shared_from_this();
    m_reqGetSubscriptions->setFailureCallback([wnode]() {
      if (auto node = wnode.lock()) {
        std::shared_ptr<http::XmlRpcRequest> r;
        {
          std::unique_lock lock(node->m_guard);
          r = node->m_reqGetSubscriptions;
        }
        if (r)
          node->m_activeRequests.erase(r);
      }
    });
    req = m_reqGetSubscriptions;
  }

  MINIROS_INFO("%s::sendGetSubscriptions()", debugName().c_str());
  m_activeRequests.insert(req);
  return client->enqueueRequest(req);
}

size_t NodeRef::getQueuedRequests() const
{
  std::unique_lock lock(m_clientGuard);
  if (!m_client)
    return 0;
  return m_client->getQueuedRequests();
}

int NodeRef::pid() const
{
  std::unique_lock lock(m_guard);
  return m_pid;
}

void NodeRef::beginRestore(std::vector<std::pair<std::string, std::string>> services)
{
  std::unique_lock lock(m_guard);
  m_pendingRestoreServices = std::move(services);
  m_restoreQueriesLeft = 0;
  updateState(State::Restoring, lock);
}

bool NodeRef::isRestoreInProgress() const
{
  std::unique_lock lock(m_guard);
  return m_state.isRestoreInProgress();
}

int NodeRef::restoreQueriesLeft() const
{
  std::unique_lock lock(m_guard);
  return m_restoreQueriesLeft;
}

void NodeRef::beginTopicRecovery(int queryCount)
{
  std::unique_lock lock(m_guard);
  if (m_state != State::Recovering)
    updateState(State::Recovering, lock);
  m_restoreQueriesLeft = queryCount;
}

bool NodeRef::notifyRestoreQueryDone()
{
  std::unique_lock lock(m_guard);
  if (m_restoreQueriesLeft > 0)
    --m_restoreQueriesLeft;
  if (m_restoreQueriesLeft > 0)
    return false;
  if (m_state == State::Recovering)
    updateState(State::Verified, lock);
  return true;
}

std::vector<std::pair<std::string, std::string>> NodeRef::takePendingRestoreServices()
{
  std::unique_lock lock(m_guard);
  std::vector<std::pair<std::string, std::string>> out;
  out.swap(m_pendingRestoreServices);
  return out;
}

void NodeRef::markDead()
{
  // Lock order matches activateConnection: m_clientGuard then m_guard.
  ClientLock clientLock(m_clientGuard);
  GuardLock lock(m_guard);
  if (m_state == State::Dead)
    return;
  MINIROS_WARN("%s::markDead()", debugName().c_str());
  deactivateConnectionUnsafe(lock, clientLock);
  m_pendingRestoreServices.clear();
  m_restoreQueriesLeft = 0;
  updateState(State::Dead, lock);
}

} // namespace master
} // namespace miniros