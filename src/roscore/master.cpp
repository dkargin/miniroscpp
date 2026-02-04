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

#include "miniros/transport/rpc_manager.h"

#include "miniros/http/http_server.h"

#include "miniros/http/endpoints/filesystem.h"
#include "miniros/http/http_filters.h"

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
}

Master::Internal::~Internal()
{
  if (timerBroadcasts)
    timerBroadcasts = {};
}

void Master::Internal::onBroadcast(const SteadyTimerEvent& evt)
{
  if (!discovery)
    return;
  discovery->doBroadcast();
}

void Master::Internal::onDiscovery(const DiscoveryEvent& evt)
{
  if (uuid == evt.uuid)
    return;
  std::string name = std::string("/") + evt.uuid.toString();
  std::string URI = evt.masterAddress.str();
  auto report = regManager.registerNodeApi(name, URI);
  if (report.created) {
    MINIROS_INFO("Registered new master=%s at %s", name.c_str(), URI.c_str());
  }
  RequesterInfo reqInfo;
  reqInfo.callerId = name;
  reqInfo.callerApi = URI;
  reqInfo.clientAddress = evt.masterAddress;
  auto hostInfo = resolver.updateHost(reqInfo);
  report.node->updateHost(hostInfo);
}

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
  return internal_->rpcManager->getServerURI();
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

  // It was done in roslaunch by calling generate_run_id() function.
  // It should be uuid.uuid1()
  internal_->uuid.generate();
  internal_->parameterStorage.setParam("master", "/run_id", internal_->uuid.toString());

  internal_->rpcManager->setPollSet(poll_set);
  internal_->regManager.setPollSet(poll_set);

  if (!internal_->rpcManager->start(cb, port)) {
    return false;
  }

  if (internal_->discovery) {
    MINIROS_DEBUG("Starting discovery module");
    internal_->discovery->setDiscoveryCallback(
        // This is invoked at poll queue thread.
      [this](const DiscoveryEvent& event) {
          internal_->onDiscovery(event);
        });

    int realPort = internal_->rpcManager->getServerPort();
    if (!internal_->discovery->start(poll_set, internal_->uuid, realPort)) {
      stop();
      return false;
    }
  } else {
    MINIROS_DEBUG("Starting without discovery module");
  }

  MINIROS_DEBUG("Master startup is complete.");
  return true;
}

void Master::stop()
{
  if (internal_ && internal_->rpcManager)
    internal_->rpcManager->shutdown();
}

bool Master::ok() const
{
  return internal_ && internal_->rpcManager && !internal_->rpcManager->isShuttingDown();
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

void Master::update()
{
  auto shutdownNodes = internal_->regManager.pullShutdownNodes();

  for (std::shared_ptr<NodeRef> nr: shutdownNodes) {
    RpcValue msg;
    std::stringstream ss;
    ss << "[" << nr->id() << "] Reason: new node registered with same name";
    msg = ss.str();
    nr->sendShutdown(msg);
  }

  auto graveyard = internal_->regManager.checkDeadNodes();
  if (!graveyard.empty()) {
    MINIROS_INFO("Dropping parameter subscriptions from %zu nodes", graveyard.size());
    for (auto node: graveyard) {
      internal_->parameterStorage.dropSubscriptions(node);
    }
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
        xmlApis[i] = apis[i];
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

  auto report = internal_->regManager.registerNodeApi(nodeId, nodeApi);
  return report.node;
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

void Master::initEvents(NodeHandle& nh)
{
  if (!internal_)
    return;
  WallDuration period(0.5);
  internal_->timerBroadcasts = nh.createSteadyTimer(period, &Internal::onBroadcast, internal_.get());
}

void Master::registerSelfRef()
{
  internal_->regManager.registerNodeApi("/miniroscore", internal_->rpcManager->getServerURI(), NodeRef::NODE_MASTER | NodeRef::NODE_LOCAL);
}

void enableDiscoveryBroadcasts(bool flag);

void Master::enableDiscoveryBroadcasts(bool flag)
{
  if (!internal_)
    return;

  if (flag && !internal_->discovery) {
    internal_->discovery.reset(new Discovery(&internal_->resolver));
    internal_->discovery->setAdapterBroadcasts(flag);
  }
  else if (!flag && internal_->discovery) {
    internal_->discovery.reset();
  }
}

Error Master::setDiscoveryGroup(const std::string& group)
{
  if (!internal_)
    return Error::InternalError;

  if (!internal_->discovery) {
    internal_->discovery.reset(new Discovery(&internal_->resolver));
  }

  return internal_->discovery->setMulticast(group);
}


} // namespace master
} // namespace miniros
