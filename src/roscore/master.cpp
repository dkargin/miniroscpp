//
// Created by dkargin on 2/11/25.
//

#include <cassert>

#include "master.h"

#include "master_handler.h"
#include "miniros_favicon.h"
#include "parameter_storage.h"

#include "miniros/transport/rpc_manager.h"

#include "http/http_server.h"
#include "miniros/transport/network.h"

#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"

#include <xmlrpcpp/XmlRpcServerConnection.h>

namespace miniros {
namespace master {

struct Master::Internal {
  int port = -1;
  std::string host;

  std::shared_ptr<RPCManager> rpcManager;

  RegistrationManager regManager;

  MasterHandler handler;
  ParameterStorage parameterStorage;

  class MasterHttpEndpoint : public http::EndpointHandler {
  public:
    MasterHttpEndpoint(Internal* internal) : internal(internal)
    {
    }

    Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
      http::HttpResponseHeader& responseHeader, std::string& body);

    Internal* internal = nullptr;
  };

  /// Handles GET /favicon.ico.
  class MasterFaviconEndpoint : public http::EndpointHandler {
    Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
      http::HttpResponseHeader& responseHeader, std::string& body);
  };

  std::shared_ptr<MasterHttpEndpoint> httpEndpoint;

  Internal(std::shared_ptr<RPCManager> manager);

  /// Render status of master as HTML page.
  void renderMasterStatus(std::string& output);
};

Error Master::Internal::MasterHttpEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
  http::HttpResponseHeader& responseHeader, std::string& body)
{
  if (!internal)
    return Error::InternalError;

  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = "text/html";
  body = "<!doctype html><html><title>Mini ROS master</title><body>";
  internal->renderMasterStatus(body);
  body += "</body></html>";
  return Error::Ok;
}

Error Master::Internal::MasterFaviconEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body)
{
  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = " image/x-icon";

  std::string_view vFavicon((const char*)(favicon), sizeof(favicon));
  body += vFavicon;

  return Error::Ok;
}

Master::Internal::Internal(std::shared_ptr<RPCManager> manager)
    : handler(manager, &regManager), parameterStorage(&regManager)
{
  rpcManager = manager;
  manager->setMaster();
  // TODO: Read environment.
  // split the URI (if it's valid) into host and port
  // if (!network.splitURI(ROS.ROS_MASTER_URI, ref _host, ref _port))
  {
    host = "localhost";
    port = 11311;
    MINIROS_WARN("Invalid XMLRPC uri. Using ROS_MASTER_URI=http://%s:%d", host.c_str(), port);
  }

  parameterStorage.paramUpdateFn =
    [this](const std::shared_ptr<NodeRef>& nr, const std::string& fullPath, const RpcValue* value)
    {
      this->handler.sendToNode(nr, "paramUpdate", fullPath, value ? *value : RpcValue::Dict());
    };
}

void Master::Internal::renderMasterStatus(std::string& output)
{
  std::stringstream ss;
  ss << "<h>Nodes</h>\n";
  ss << "<ul>";
  for (const std::shared_ptr<NodeRef>& r : regManager.listAllNodes()) {
    const std::string& name = r->id();
    std::string url = r->getApi();
    ss << "<li>";
    ss << name;
    ss << ": <a href=\"" << url << "\">" << url << "</a>";
    ss << "</li>";
  }
  ss << "</ul>\n";

  ss << "<h>Topics</h>\n";
  ss << "<ul>";
  std::map<std::string, std::string> types = handler.getTopicTypes("root");
  for (const auto& [topic, type] : types) {
    ss << "<li>";
    ss << topic << " (" << type << ")";
    ss << "</li>";
  }
  ss << "</ul>\n";
  output += ss.str();
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
  return std::string("http://") + internal_->host + ":" + std::to_string(internal_->port);
}

int Master::getPort() const
{
  return internal_ ? internal_->port : 0;
}

bool Master::start(PollSet* poll_set)
{
  if (!internal_)
    return false;
  if (!internal_->rpcManager) {
    MINIROS_ERROR("No RPC Manager was attached");
    return false;
  }

  MINIROS_DEBUG("Creating XmlRpc server");

  setupBindings();

  http::HttpServer* server = internal_->rpcManager->getHttpServer();
  if (server) {
    internal_->httpEndpoint.reset(new Internal::MasterHttpEndpoint(internal_.get()));
    server->registerEndpoint(std::make_unique<http::SimpleFilter>("/", http::HttpMethod::Get), internal_->httpEndpoint);
    server->registerEndpoint(std::make_unique<http::SimpleFilter>("/favicon.ico", http::HttpMethod::Get),
      std::make_shared<Internal::MasterFaviconEndpoint>());
  }

  // It was done in roslaunch by calling generate_run_id() function.
  // It should be uuid.uuid1()
  std::string uuid = generatePseudoUuid();
  internal_->parameterStorage.setParam("master", "/run_id", uuid);

  if (!internal_->rpcManager->start(poll_set, internal_->port))
    return false;

  MINIROS_DEBUG("Master startup complete.");
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

void Master::setupBindings()
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

  rpcManager->getServerURI();
}

void Master::setResolveNodeIP(bool resolv)
{
  if (!internal_)
    return;
  internal_->handler.setResolveNodeIP(resolv);
  internal_->parameterStorage.setParam("master", "/resolve_ip", resolv);
}

void Master::update()
{
  internal_->handler.update();
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

Master::RpcValue Master::getTopicTypes(const std::string& caller_id, const ClientInfo&)
{
  std::map<std::string, std::string> types = internal_->handler.getTopicTypes(caller_id);

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

Master::RpcValue Master::subscribeParam(const std::string& caller_id, const std::string& caller_api,
  const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "subscribeParam done";
  const RpcValue* val = internal_->parameterStorage.subscribeParam(caller_id, caller_api, key);
  res[2] = val ? *val : RpcValue::Dict();
  return res;
}

Master::RpcValue Master::unsubscribeParam(const std::string& caller_id, const std::string& caller_api,
  const std::string& key, const ClientInfo&)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "unsubscribeParam done";
  if (internal_->parameterStorage.unsubscribeParam(caller_id, caller_api, key))
    res[2] = 1;
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

} // namespace master
} // namespace miniros
