//
// Created by dkargin on 2/11/25.
//

#include <cassert>

#include "master.h"
#include "master_internal.h"
#include "master_endpoints.h"

#include "master_handler.h"
#include "master_link.h"
#include "node_handle.h"
#include "parameter_storage.h"
#include "multimaster_protocol.h"

#include "miniros/transport/rpc_manager.h"
#include "miniros/this_node.h"

#include "internal_config.h"

#include "miniros/http/http_server.h"

#include "miniros/http/endpoints/filesystem.h"
#include "miniros/http/http_filters.h"
#include "miniros/callback_queue.h"
#include "miniros/rostime.h"
#include "miniros/common.h"

namespace miniros {

/// Implemented in transport/init.cpp
CallbackQueuePtr getInternalCallbackQueue();

namespace master {

Master::Internal::Internal(const std::shared_ptr<RPCManager>& manager)
  : regManager("/master")
  , handler(manager, &regManager, &resolver)
{
  rpcManager = manager;
  resolver.scanAdapters();
  cache.bind(&regManager, &handler);
  multimaster = std::make_unique<MultimasterManager>(&resolver, &regManager);
  cache.setPeerPersistence(
    [this]() { return multimaster ? multimaster->collectCachedPeers() : std::vector<CachedPeer>{}; },
    [this](const std::vector<CachedPeer>& peers) {
      if (multimaster)
        multimaster->restoreCachedPeers(peers);
    });
  multimaster->setTopologyChanged([this]() { cache.markDirty(); });
}

Master::Internal::~Internal() = default;

Master::Master(std::shared_ptr<RPCManager> manager)
{
  internal_ = std::make_unique<Internal>(manager);
}

Master::~Master()
{
  if (internal_->rpcManager) {
    internal_->rpcManager->unbind(this);
  }
}

std::string Master::getUri() const
{
  return internal_->rpcManager->getServerUrlStr();
}

int Master::getPort() const
{
  return internal_ && internal_->rpcManager ? internal_->rpcManager->getServerPort() : 0;
}

bool Master::start(PollSet* poll_set, int port)
{
  if (!internal_)
    return false;
  if (!internal_->rpcManager) {
    MINIROS_ERROR("No RPC Manager was attached");
    return false;
  }

  MINIROS_DEBUG("Starting RPC module");

  auto cb = getInternalCallbackQueue();

  setupBindings(cb);
  internal_->callbackQueue = cb;

  // Prefer a persisted instance GUID when caching is enabled.
  bool guidFromCache = false;
  if (internal_->cache.enabled()) {
    if (Error err = internal_->cache.load(port); !err) {
      MINIROS_WARN("MasterCache: load failed: %s", err.toString());
    } else if (!internal_->cache.guid().empty()) {
      if (internal_->uuid.fromString(internal_->cache.guid())) {
        guidFromCache = true;
        MINIROS_INFO("Restored master GUID %s from cache", internal_->cache.guid().c_str());
      } else {
        MINIROS_WARN("Ignoring invalid GUID in cache: %s", internal_->cache.guid().c_str());
      }
    }
  }
  if (!guidFromCache) {
    // It was done in roslaunch by calling generate_run_id() function.
    internal_->uuid.generate();
  }
  internal_->parameterStorage.setParam("master", "/run_id", internal_->uuid.toString());

  internal_->rpcManager->setPollSet(poll_set);
  internal_->regManager.setPollSet(poll_set);

  if (!internal_->rpcManager->start(cb, port)) {
    return false;
  }

  // Use the already-loaded snapshot; bound port only updates the save path.
  const int boundPort = internal_->rpcManager->getServerPort();
  if (internal_->cache.enabled()) {
    internal_->cache.beginRestore(boundPort > 0 ? boundPort : port, internal_->uuid.toString());
  }

  if (internal_->multimaster) {
    auto url = internal_->rpcManager->getServerUrl();
    internal_->multimaster->setCollectSnapshot([this]() {
      return internal_->collectMultimasterSnapshot();
    });
    internal_->multimaster->setApplyRecords([this](const UUID& peer, const std::vector<miniros_msgs::RegistrationRecord>& records, bool snapshot) {
      internal_->applyMultimasterRecords(peer, records, snapshot);
    });
    internal_->multimaster->setDropPeer([this](const UUID& peer) {
      internal_->dropMultimasterPeer(peer);
    });
    if (Error err = internal_->multimaster->start(poll_set, internal_->uuid, url); !err) {
      MINIROS_ERROR("Failed to start multimaster UDP: %s", err.toString());
      stop();
      return false;
    }
    // Re-pair peers from cache before (or alongside) fresh discovery.
    if (internal_->cache.enabled())
      internal_->cache.restorePeers();
    // Discovery is always active; token only gates pairing.
    internal_->multimaster->sendDiscover();
  }

  MINIROS_DEBUG("Master startup is complete.");
  return true;
}

void Master::stop()
{
  if (!internal_)
    return;
  const int port = internal_->rpcManager ? internal_->rpcManager->getServerPort() : 0;
  internal_->cache.flush(port, internal_->uuid.toString());
  if (internal_->multimaster)
    internal_->multimaster->stop();
  if (internal_->rpcManager)
    internal_->rpcManager->shutdown();
}

bool Master::ok() const
{
  if (!internal_ || !internal_->rpcManager || internal_->rpcManager->isShuttingDown())
    return false;
  if (internal_->shutdownRequested.load())
    return false;
  return true;
}

void Master::setDebugApi(bool enabled)
{
  if (internal_)
    internal_->debugApiEnabled = enabled;
}

void Master::requestShutdown()
{
  if (!internal_)
    return;
  internal_->shutdownRequested.store(true);
  MINIROS_INFO("Master shutdown requested");
}

void Master::setupBindings(const std::shared_ptr<CallbackQueue>& cb)
{
  if (!internal_)
    return;
  RPCManager* rpcManager = internal_->rpcManager.get();
  if (!rpcManager)
    return;
  // Core master part.
  rpcManager->bindEx4("registerPublisher", this, &Master::registerPublisher);
  rpcManager->bindEx3("unregisterPublisher", this, &Master::unregisterPublisher);
  rpcManager->bindEx4("registerSubscriber", this, &Master::registerSubscriber);
  rpcManager->bindEx3("unregisterSubscriber", this, &Master::unregisterSubscriber);
  rpcManager->bindEx2("getPublishedTopics", this, &Master::getPublishedTopics);
  rpcManager->bindEx1("getTopicTypes", this, &Master::getTopicTypes);
  rpcManager->bindEx1("getSystemState", this, &Master::getSystemState);

  rpcManager->bindEx2("lookupService", this, &Master::lookupService);
  rpcManager->bindEx3("unregisterService", this, &Master::unregisterService);
  rpcManager->bindEx4("registerService", this, &Master::registerService);
  rpcManager->bindEx2("lookupNode", this, &Master::lookupNode);

  // Rosparam part.
  rpcManager->bindEx2("hasParam", this, &Master::hasParam);
  rpcManager->bindEx3("setParam", this, &Master::setParam);
  rpcManager->bindEx2("getParam", this, &Master::getParam);
  rpcManager->bindEx2("deleteParam", this, &Master::deleteParam);
  rpcManager->bindEx2("searchParam", this, &Master::searchParam);
  rpcManager->bindEx3("subscribeParam", this, &Master::subscribeParam);
  rpcManager->bindEx3("unsubscribeParam", this, &Master::unsubscribeParam);
  rpcManager->bindEx1("getParamNames", this, &Master::getParamNames);

  if (http::HttpServer* server = internal_->rpcManager->getHttpServer()) {
    internal_->httpRootEndpoint.reset(new MasterRootEndpoint(internal_.get()));
    internal_->httpNodeInfoEndpoint.reset(new NodeInfoEndpoint(internal_.get()));
    internal_->httpTopicInfoEndpoint.reset(new TopicInfoEndpoint(internal_.get()));
    internal_->httpPublishedTopicsEndpoint.reset(new PublishedTopicsEndpoint(internal_.get()));
    internal_->httpTopicTypesEndpoint.reset(new TopicTypesEndpoint(internal_.get()));
    internal_->httpMultimasterApiEndpoint.reset(new MultimasterApiEndpoint(internal_.get()));

    server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/"), internal_->httpRootEndpoint, cb);
    server->registerEndpoint(
      std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/node/", http::SimpleFilter::CheckType::Prefix),
      internal_->httpNodeInfoEndpoint, cb);
    server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/topic/", http::SimpleFilter::CheckType::Prefix),
      internal_->httpTopicInfoEndpoint, cb);
    server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/api2/published_topics"),
      internal_->httpPublishedTopicsEndpoint, cb);
    server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/api2/topic_types"),
      internal_->httpTopicTypesEndpoint, cb);
    server->registerEndpoint(
      std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/api2/multimaster", http::SimpleFilter::CheckType::Prefix),
      internal_->httpMultimasterApiEndpoint, cb);
    if (internal_->debugApiEnabled) {
      internal_->httpDebugApiEndpoint.reset(new DebugApiEndpoint(internal_.get()));
      server->registerEndpoint(
        std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/debugAPI", http::SimpleFilter::CheckType::Prefix),
        internal_->httpDebugApiEndpoint, cb);
      MINIROS_WARN("Debug HTTP API enabled (GET /debugAPI/...)");
    }
    server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/favicon.ico"),
      std::make_shared<MasterFaviconEndpoint>(), cb);

    // This endpoint is only for testing purposes. Clients could create endpoints by themselves if needed.
    //auto fsEndpoint = std::make_shared<http::FilesystemEndpoint>("/files/", ".");
    //server->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/files/", http::SimpleFilter::CheckType::Prefix), fsEndpoint);
  }
}

void Master::setResolveNodeIP(bool resolv)
{
  if (!internal_)
    return;
  internal_->resolver.setResolveIp(resolv);
  internal_->parameterStorage.setParam("master", "/resolve_ip", resolv);
}

void Master::setNodeCheckPeriod(double seconds)
{
  if (!internal_)
    return;
  if (seconds < 0)
    seconds = 0;
  internal_->nodeCheckPeriod = WallDuration(seconds);
  internal_->lastNodeCheck = SteadyTime();
  MINIROS_INFO("Node liveness check period set to %.3fs%s",
               seconds, seconds <= 0 ? " (disabled)" : "");
}

void Master::setCacheEnabled(bool enabled)
{
  if (!internal_)
    return;
  internal_->cache.setEnabled(enabled);
}

void Master::Internal::checkNodesAlive()
{
  auto nodes = regManager.listAllNodes();
  for (const std::shared_ptr<NodeRef>& node : nodes) {
    if (!node)
      continue;

    if (node->isLocal())
      continue;

    if (node->getNodeFlags() & (NodeRef::NODE_MASTER | NodeRef::NODE_FOREIGN))
      continue;

    if (node->getState() == NodeRef::State::Dead) {
      regManager.scheduleShutdown(node);
      continue;
    }

    if (!node->needRequests())
      continue;

    // Refresh PID / probe reachability via Slave API.
    if (Error err = node->sendGetPid("/master"); !err) {
      // NotConnected is expected while reconnecting; disconnect handler marks
      // the node Dead when reconnect fails, then scheduleDeadNodesForShutdown
      // picks it up.
      MINIROS_DEBUG("sendGetPid(%s) returned %s", node->id().c_str(), err.toString());
    }
  }
}

void Master::Internal::shutdownNode(const std::shared_ptr<NodeRef>& node, const std::string& reason)
{
  assert(node);
  if (!node)
    return;

  std::set<std::string> publishedTopics;
  {
    NodeRef::Lock nodeLock(*node);
    publishedTopics = node->getPublicationsLocked(nodeLock);
  }

  // Ask the node to shut down if the HTTP client is still usable.
  if (Error err = node->sendShutdown(reason); !err) {
    MINIROS_DEBUG("sendShutdown(%s) returned %s", node->id().c_str(), err.toString());
  }

  regManager.dropRegistrations(*node);
  node->clear();

  // Tell remaining subscribers that publishers for these topics have changed.
  for (const std::string& topic : publishedTopics) {
    auto subscribers = regManager.getTopicSubscribers(topic);
    handler.notifyTopicSubscribers(topic, subscribers);
  }

  node->markDead();
  cache.markDirty();
}

void Master::update()
{
  // Queue unreachable nodes discovered via disconnect / failed reconnect.
  internal_->regManager.scheduleDeadNodesForShutdown();

  if (internal_->nodeCheckPeriod.toSec() > 0) {
    const SteadyTime now = SteadyTime::now();
    if (internal_->lastNodeCheck.isZero() ||
        (now - internal_->lastNodeCheck) >= internal_->nodeCheckPeriod) {
      internal_->lastNodeCheck = now;
      internal_->checkNodesAlive();
    }
  }

  auto shutdownNodes = internal_->regManager.pullShutdownNodes();
  for (std::shared_ptr<NodeRef> nr : shutdownNodes) {
    if (!nr)
      continue;
    // Never kill the in-process master or peer-master UI entries.
    if (nr->getNodeFlags() & (NodeRef::NODE_LOCAL | NodeRef::NODE_MASTER)) {
      MINIROS_WARN("Skipping shutdown of protected node %s", nr->id().c_str());
      continue;
    }
    // Superseded NodeRef: a newer registration already owns this name in
    // RegistrationManager. dropRegistrations() keys by name, so shutting down
    // the old object would wipe the live node's pubs/subs/services — which is
    // exactly what broke multimaster snapshot replace (pair sends two snapshots).
    auto current = internal_->regManager.getNodeByName(nr->id());
    if (current && current != nr) {
      nr->clear();
      if (nr->getState() != NodeRef::State::Dead)
        nr->markDead();
      continue;
    }
    // Foreign mirrors are owned by multimaster sync (dropMultimasterPeer), not
    // by Slave-API liveness. If one is queued while still current, dispose
    // without pretending we can shutdown() a remote process.
    if (nr->getNodeFlags() & NodeRef::NODE_FOREIGN) {
      internal_->regManager.dropRegistrations(*nr);
      nr->clear();
      if (nr->getState() != NodeRef::State::Dead)
        nr->markDead();
      continue;
    }
    std::stringstream ss;
    ss << "[" << nr->id() << "] Reason: node unreachable";
    internal_->shutdownNode(nr, ss.str());
  }

  // Drive cache restore before empty-node GC: restored NodeRefs are empty until
  // getPublications / services are reapplied.
  const int port = internal_->rpcManager ? internal_->rpcManager->getServerPort() : 0;
  internal_->cache.update(port, internal_->uuid.toString());

  auto graveyard = internal_->regManager.checkNodesForRemoval();
  if (!graveyard.empty()) {
    MINIROS_INFO("Dropping parameter subscriptions from %d nodes", static_cast<int>(graveyard.size()));
    for (auto node: graveyard) {
      internal_->parameterStorage.dropSubscriptions(node);
    }
    internal_->cache.markDirty();
  }

  if (internal_->multimaster)
    internal_->multimaster->update();

  if (internal_->multimaster) {
    for (const PeerInfo& peer : internal_->multimaster->listPeers())
      internal_->registerPeerMasterNode(peer);
  }
}

Master::RpcValue Master::lookupService(
  const std::string& caller_id, const std::string& service, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  std::string uri = internal_->handler.lookupService(requesterInfo, service);

  RpcValue res = RpcValue::Array(3);
  if (uri.empty()) {
    res[0] = 0;
    res[1] = std::string("Failed to lookup service '" + service + "'");
    res[2] = RpcValue();
  } else {
    res[0] = 1;
    res[1] = std::string("rosrpc URI: [") + uri + "]";
    res[2] = uri;
  }
  return res;
}

Master::RpcValue Master::registerService(const std::string& caller_id, const std::string& service,
  const std::string& service_api, const std::string& caller_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct r = internal_->handler.registerService(requesterInfo, service, service_api);
  if (r.statusCode == 1)
    internal_->cache.markDirty();

  if (r.statusCode == 1 && internal_->multimaster) {
    miniros_msgs::RegistrationRecord rec;
    rec.kind = miniros_msgs::RegistrationRecord::KIND_SRV_REGISTER;
    rec.name = service;
    rec.node_name = caller_id;
    rec.node_api = caller_api;
    rec.service_api = service_api;
    internal_->multimaster->announceLocalChange(rec);
  }

  RpcValue res = RpcValue::Array(3);
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::unregisterService(const std::string& caller_id, const std::string& service,
  const std::string& service_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  ReturnStruct r = internal_->handler.unregisterService(requesterInfo, service, service_api);
  if (r.statusCode == 1)
    internal_->cache.markDirty();

  if (r.statusCode == 1 && internal_->multimaster) {
    miniros_msgs::RegistrationRecord rec;
    rec.kind = miniros_msgs::RegistrationRecord::KIND_SRV_UNREGISTER;
    rec.name = service;
    rec.node_name = caller_id;
    rec.service_api = service_api;
    internal_->multimaster->announceLocalChange(rec);
  }

  RpcValue res = RpcValue::Array(3);
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::getTopicTypes(const std::string& /*caller_id*/, const ClientInfo&)
{
  std::unique_lock<const RegistrationManager> lock(internal_->regManager);

  const auto& types = internal_->regManager.getTopicTypesUnsafe(lock);

  RpcValue xmlTopics = RpcValue::Array(types.size());
  int index = 0;
  for (auto [key, val] : types) {
    RpcValue payload;
    payload[0] = key;
    payload[1] = val;
    xmlTopics[index++] = payload;
  }

  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "current system state";
  res[2] = xmlTopics;
  return res;
}

Master::RpcValue Master::getSystemState(const std::string& caller_id, const ClientInfo& clientInfo)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "current system state";

  auto writeXml = [&](const std::map<std::string, std::vector<std::string>>& providers, RpcValue& result) {
    int index = 0;
    result.setSize(providers.size());
    for (const auto& [key, apis] : providers) {
      RpcValue xmlApis;
      xmlApis.setSize(apis.size());
      for (size_t i = 0; i < apis.size(); i++) {
        xmlApis[static_cast<int>(i)] = apis[i];
      }

      RpcValue group;
      group.setSize(2);
      group[0] = key;
      group[1] = xmlApis;
      result[index++] = group;
    }
  };

  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }

  MasterHandler::SystemState state = internal_->handler.getSystemState(requesterInfo);

  RpcValue listoftypes = RpcValue::Array(3);

  writeXml(state.publishers, listoftypes[0]);
  writeXml(state.subscribers, listoftypes[1]);
  writeXml(state.services, listoftypes[2]);

  res[2] = listoftypes;
  return res;
}

Master::RpcValue Master::getPublishedTopics(const std::string& caller_id, const std::string& subgraph, const ClientInfo& clientInfo)
{
  RpcValue res = RpcValue::Array(3);

  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  auto topics = internal_->handler.getPublishedTopics(requesterInfo, subgraph);
  res[0] = 1;
  res[1] = "current system state";

  RpcValue xmlTopics = RpcValue::Array(topics.size());
  int index = 0;
  for (const auto& l : topics) {
    RpcValue value = RpcValue::Array(2);
    value[0] = l[0]; // Topic Name
    value[1] = l[1]; // Topic type
    xmlTopics[index] = value;
    index++;
  }
  res[2] = xmlTopics;
  return res;
}

Master::RpcValue Master::registerPublisher(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = internal_->handler.registerPublisher(requesterInfo, topic, type);
  if (st.statusCode == 1) {
    internal_->cache.markDirty();
    if (internal_->multimaster) {
      miniros_msgs::RegistrationRecord rec;
      rec.kind = miniros_msgs::RegistrationRecord::KIND_PUB_REGISTER;
      rec.name = topic;
      rec.type = type;
      rec.node_name = caller_id;
      rec.node_api = caller_api;
      internal_->multimaster->announceLocalChange(rec);
    }
  }
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = internal_->handler.unregisterPublisher(requesterInfo, topic);
  if (st.statusCode == 1) {
    internal_->cache.markDirty();
    if (internal_->multimaster) {
      miniros_msgs::RegistrationRecord rec;
      rec.kind = miniros_msgs::RegistrationRecord::KIND_PUB_UNREGISTER;
      rec.name = topic;
      rec.node_name = caller_id;
      rec.node_api = caller_api;
      internal_->multimaster->announceLocalChange(rec);
    }
  }
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = internal_->handler.registerSubscriber(requesterInfo, topic, type);
  if (st.statusCode == 1) {
    internal_->cache.markDirty();
    if (internal_->multimaster) {
      miniros_msgs::RegistrationRecord rec;
      rec.kind = miniros_msgs::RegistrationRecord::KIND_SUB_REGISTER;
      rec.name = topic;
      rec.type = type;
      rec.node_name = caller_id;
      rec.node_api = caller_api;
      internal_->multimaster->announceLocalChange(rec);
    }
  }
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& caller_api, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = internal_->handler.unregisterSubscriber(requesterInfo, topic);
  if (st.statusCode == 1) {
    internal_->cache.markDirty();
    if (internal_->multimaster) {
      miniros_msgs::RegistrationRecord rec;
      rec.kind = miniros_msgs::RegistrationRecord::KIND_SUB_UNREGISTER;
      rec.name = topic;
      rec.node_name = caller_id;
      rec.node_api = caller_api;
      internal_->multimaster->announceLocalChange(rec);
    }
  }
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::lookupNode(const std::string& caller_id, const std::string& node, const ClientInfo& clientInfo)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, clientInfo)) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }

  std::string api = internal_->handler.lookupNode(requesterInfo, node);
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "lookupNode";
  res[2] = api;
  return res;
}

Master::RpcValue Master::hasParam(const std::string& caller_id, const std::string& key, const ClientInfo& /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  bool found = internal_->parameterStorage.hasParam(caller_id, key);
  res[0] = 1;
  res[1] = key;
  res[2] = found;
  return res;
}

Master::RpcValue Master::setParam(
  const std::string& caller_api, const std::string& key, const RpcValue& value, const ClientInfo& /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "setParam";
  internal_->parameterStorage.setParam(caller_api, key, value);
  res[2] = std::string("parameter ") + key + std::string(" set");
  return res;
}

Master::RpcValue Master::getParam(const std::string& caller_id, const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  RpcValue value = internal_->parameterStorage.getParam(caller_id, key);
  if (!value.valid()) {
    res[0] = -1;
    res[1] = std::string("Parameter [") + key + std::string("] is not set");
    res[2] = 0;
  } else {
    res[0] = 1;
    res[1] = std::string("Parameter [") + key + std::string("]");
    res[2] = value;
  }
  return res;
}

Master::RpcValue Master::deleteParam(const std::string& caller_id, const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[2] = 0;
  if (internal_->parameterStorage.deleteParam(caller_id, key)) {
    res[1] = "deleteParam success";
  } else {
    res[1] = "deleteParam param not found";
  }
  return res;
}

Master::RpcValue Master::searchParam(const std::string& caller_id, const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  std::string foundKey = internal_->parameterStorage.searchParam(caller_id, key);
  if (!foundKey.empty()) {
    res[0] = 1;
    res[1] = "searchParam success";
  } else {
    res[0] = 0;
    res[1] = "searchParam param not found";
  }
  res[2] = foundKey;
  return res;
}

std::shared_ptr<NodeRef> Master::registerNodeApi(const std::string& nodeId, const std::string& nodeApi) const
{
  if (!internal_)
    return {};

  auto report = internal_->regManager.registerNodeApi(nodeId, nodeApi, 0);
  return report.node;
}

void Master::registerSelfRef()
{
  if (!internal_ || !internal_->rpcManager)
    return;

  const std::string& name = this_node::getName();
  const std::string& api = internal_->rpcManager->getServerUrlStr();
  if (name.empty() || api.empty()) {
    MINIROS_WARN("registerSelfRef: missing node name or RPC URL");
    return;
  }

  auto report = internal_->regManager.registerNodeApi(
    name, api, NodeRef::NODE_LOCAL | NodeRef::NODE_MASTER | NodeRef::NODE_MINIROS);
  if (report.created) {
    MINIROS_INFO("Registered local master node %s at %s", name.c_str(), api.c_str());
  }
}

std::shared_ptr<NodeRef> Master::getNodeByName(const std::string& nodeId) const
{
  if (!internal_)
    return {};

  return internal_->regManager.getNodeByName(nodeId);
}

Master::RpcValue Master::subscribeParam(const std::string& caller_id, const std::string& caller_api,
  const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  auto node = registerNodeApi(caller_id, caller_api);
  if (node) {
    res[0] = 1;
    res[1] = "subscribeParam done";
    const RpcValue* val = internal_->parameterStorage.subscribeParam(node, key);
    res[2] = val ? *val : RpcValue::Dict();
  } else {
    res[0] = 0;
    res[1] = "Failed to find node";
    res[2] = 0;
  }
  return res;
}

Master::RpcValue Master::unsubscribeParam(const std::string& caller_id, const std::string& caller_api,
  const std::string& key, const ClientInfo&)
{
  assert(internal_);
  RpcValue res = RpcValue::Array(3);
  auto node = getNodeByName(caller_id);
  if (node) {
    res[0] = 1;
    res[1] = "unsubscribeParam done";
    if (internal_->parameterStorage.unsubscribeParam(node, key)) {
      res[2] = 1;
    }
  } else {
    MINIROS_ERROR("Master::unsubscribeParam(%s) from %s - no such node", key.c_str(), caller_id.c_str());
    res[0] = 0;
    res[1] = "Failed to find node";
    res[2] = 0;
  }
  return res;
}

Master::RpcValue Master::getParamNames(const std::string& caller_id, const ClientInfo&)
{
  assert(internal_);
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "getParamNames";

  RpcValue response;
  int index = 0;
  for (std::string s : internal_->parameterStorage.getParamNames(caller_id)) {
    response[index++] = s;
  }

  res[2] = response;
  return res;
}

void Master::setDumpParameters(bool dump)
{
  if (internal_)
    internal_->parameterStorage.setDumpParameters(dump);
}

void Master::initEvents(NodeHandle& /*nh*/)
{
}

void Master::setMultimasterToken(const std::string& token)
{
  if (!internal_ || !internal_->multimaster)
    return;
  internal_->multimaster->setToken(token);
  if (!token.empty())
    internal_->parameterStorage.setParam("master", "/multimaster/token_set", true);
}

void Master::setMultimasterUdpPort(int port)
{
  if (!internal_ || !internal_->multimaster)
    return;
  internal_->multimaster->setUdpPort(port);
}

void Master::setMultimasterMulticast(const std::string& addr, int port)
{
  if (!internal_ || !internal_->multimaster)
    return;
  internal_->multimaster->setMulticast(addr, port);
}

Error Master::addMultimasterPeer(const std::string& host, int udpPort)
{
  if (!internal_ || !internal_->multimaster)
    return Error::InternalError;
  if (host.empty() || udpPort <= 0)
    return Error::InvalidValue;
  network::NetAddress addr = network::NetAddress::fromIp4String(host, udpPort);
  if (!addr.valid())
    addr = network::NetAddress::fromString(network::NetAddress::AddressUnspecified, host, udpPort);
  if (!addr.valid())
    return Error::InvalidAddress;
  return internal_->multimaster->addPeerProbe(addr);
}

std::vector<miniros_msgs::RegistrationRecord> Master::Internal::collectMultimasterSnapshot() const
{
  std::vector<miniros_msgs::RegistrationRecord> out;
  // Keep rosout traffic local to each master; never advertise the in-process
  // master node (/miniroscore) itself.
  const std::set<std::string> localOnly{"/rosout", "/rosout_agg"};

  for (const std::shared_ptr<NodeRef>& node : regManager.listAllNodes()) {
    if (!node)
      continue;
    const int flags = node->getNodeFlags();
    // LOCAL = in-process master/rosout; MASTER = peer or self master UI entry;
    // FOREIGN = already mirrored from another master.
    if (flags & (NodeRef::NODE_FOREIGN | NodeRef::NODE_MASTER | NodeRef::NODE_LOCAL))
      continue;

    const std::string nodeName = node->id();
    const std::string nodeApi = node->getApi();
    std::set<std::string> pubs, subs, srvs;
    {
      NodeRef::Lock nodeLock(*node);
      pubs = node->getPublicationsLocked(nodeLock);
      subs = node->getSubscriptionsLocked(nodeLock);
      srvs = node->getServicesLocked(nodeLock);
    }

    // Hold RegistrationManager while reading topic types / service APIs — they
    // share m_guard with register/unregister (TSan race otherwise).
    RegistrationManager::Lock regLock(regManager);
    const auto& topicTypes = regManager.getTopicTypesUnsafe(regLock);
    for (const std::string& topic : pubs) {
      if (localOnly.count(topic))
        continue;
      miniros_msgs::RegistrationRecord r;
      r.kind = miniros_msgs::RegistrationRecord::KIND_PUB_REGISTER;
      r.name = topic;
      auto tit = topicTypes.find(topic);
      r.type = (tit != topicTypes.end()) ? tit->second : std::string{};
      r.node_name = nodeName;
      r.node_api = nodeApi;
      out.push_back(std::move(r));
    }
    for (const std::string& topic : subs) {
      if (localOnly.count(topic))
        continue;
      miniros_msgs::RegistrationRecord r;
      r.kind = miniros_msgs::RegistrationRecord::KIND_SUB_REGISTER;
      r.name = topic;
      auto tit = topicTypes.find(topic);
      r.type = (tit != topicTypes.end()) ? tit->second : std::string{};
      r.node_name = nodeName;
      r.node_api = nodeApi;
      out.push_back(std::move(r));
    }
    for (const std::string& service : srvs) {
      miniros_msgs::RegistrationRecord r;
      r.kind = miniros_msgs::RegistrationRecord::KIND_SRV_REGISTER;
      r.name = service;
      r.node_name = nodeName;
      r.node_api = nodeApi;
      r.service_api = regManager.services.get_service_api(service);
      out.push_back(std::move(r));
    }
  }
  return out;
}

void Master::Internal::registerPeerMasterNode(const PeerInfo& peer)
{
  if (peer.state == PeerState::GuidCollision)
    return;
  if (!peer.uuid.valid())
    return;
  std::string name = "/master_" + peer.uuid.toString();
  std::string URI = peer.masterUri.str();
  if (URI.empty() && peer.lastAddress.valid()) {
    network::URL u;
    u.scheme = "http://";
    u.host = peer.lastAddress.address;
    u.port = peer.masterUri.port ? peer.masterUri.port : static_cast<uint32_t>(peer.lastAddress.port());
    URI = u.str();
  }
  if (URI.empty())
    return;

  if (auto existing = regManager.getNodeByName(name)) {
    if (existing->getApi() == URI)
      return;
  }

  auto report = regManager.registerNodeApi(name, URI, NodeRef::NODE_MASTER | NodeRef::NODE_MINIROS);
  if (report.created) {
    MINIROS_INFO("Registered peer master=%s at %s", name.c_str(), URI.c_str());
  }
}

void Master::Internal::dropMultimasterPeer(const UUID& peer)
{
  const std::string key = peer.toString();
  auto it = foreignNodesByPeer.find(key);
  if (it == foreignNodesByPeer.end())
    return;

  std::set<std::string> topicsToNotify;
  for (const std::string& nodeName : it->second) {
    auto node = regManager.getNodeByName(nodeName);
    if (!node)
      continue;
    {
      NodeRef::Lock nodeLock(*node);
      const auto& pubs = node->getPublicationsLocked(nodeLock);
      topicsToNotify.insert(pubs.begin(), pubs.end());
    }
    regManager.dropRegistrations(*node);
    node->clear();
    node->markDead();
    regManager.scheduleShutdown(node);
  }
  foreignNodesByPeer.erase(it);

  for (const std::string& topic : topicsToNotify) {
    handler.notifyTopicSubscribers(topic, regManager.getTopicSubscribers(topic));
  }
}

void Master::Internal::applyMultimasterRecords(const UUID& peer, const std::vector<miniros_msgs::RegistrationRecord>& records, bool snapshot)
{
  const std::string peerKey = peer.toString();
  if (snapshot)
    dropMultimasterPeer(peer);

  auto& foreignNodes = foreignNodesByPeer[peerKey];
  std::set<std::string> topicsToNotify;
  constexpr int kForeignFlags = NodeRef::NODE_FOREIGN | NodeRef::NODE_MINIROS;

  for (const miniros_msgs::RegistrationRecord& r : records) {
    if (r.name == "/rosout" || r.name == "/rosout_agg")
      continue;

    switch (r.kind) {
    case miniros_msgs::RegistrationRecord::KIND_PUB_REGISTER: {
      auto node = regManager.register_publisher(r.name, r.type, r.node_name, r.node_api, kForeignFlags);
      if (node) {
        foreignNodes.insert(r.node_name);
        topicsToNotify.insert(r.name);
      }
      break;
    }
    case miniros_msgs::RegistrationRecord::KIND_PUB_UNREGISTER: {
      regManager.unregister_publisher(r.name, r.node_name, r.node_api);
      topicsToNotify.insert(r.name);
      break;
    }
    case miniros_msgs::RegistrationRecord::KIND_SUB_REGISTER: {
      auto node = regManager.register_subscriber(r.name, r.type, r.node_name, r.node_api, kForeignFlags);
      if (node) {
        foreignNodes.insert(r.node_name);
      }
      break;
    }
    case miniros_msgs::RegistrationRecord::KIND_SUB_UNREGISTER: {
      regManager.unregister_subscriber(r.name, r.node_name, r.node_api);
      break;
    }
    case miniros_msgs::RegistrationRecord::KIND_SRV_REGISTER: {
      auto node = regManager.register_service(r.name, r.node_name, r.node_api,
        r.service_api.empty() ? r.node_api : r.service_api, kForeignFlags);
      if (node) {
        foreignNodes.insert(r.node_name);
      }
      break;
    }
    case miniros_msgs::RegistrationRecord::KIND_SRV_UNREGISTER: {
      regManager.unregister_service(r.name, r.node_name, r.service_api);
      break;
    }
    }
  }

  for (const std::string& topic : topicsToNotify) {
    handler.notifyTopicSubscribers(topic, regManager.getTopicSubscribers(topic));
  }
}

} // namespace master
} // namespace miniros
