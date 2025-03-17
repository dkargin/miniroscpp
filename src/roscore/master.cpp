//
// Created by dkargin on 2/11/25.
//

#include "master.h"
#include "miniros/transport/network.h"

#include <xmlrpcpp/XmlRpcServerConnection.h>

namespace miniros {
namespace master {

Master::Master(std::shared_ptr<RPCManager> manager)
  : m_handler(manager, &m_regManager), m_parameterStorage(&m_regManager)
{
  m_rpcManager = manager;
  // TODO: Read environment.
  // split the URI (if it's valid) into host and port
  // if (!network.splitURI(ROS.ROS_MASTER_URI, ref _host, ref _port))
  {
    m_host = "localhost";
    m_port = 11311;
    MINIROS_WARN("Invalid XMLRPC uri. Using ROS_MASTER_URI=http://%s:%d", m_host.c_str(), m_port);
  }

  m_parameterStorage.paramUpdateFn =
    [this] (const std::shared_ptr<NodeRef>& nr, const std::string& fullPath, const RpcValue* value) {
      this->m_handler.sendToNode(nr, "paramUpdate", fullPath, value ? *value : RpcValue::Dict());
    };
}

Master::~Master()
{
  if (m_rpcManager) {
    m_rpcManager->unbind(this);
  }
}

std::string Master::getUri() const
{
  return std::string("http://") + m_host + ":" + std::to_string(m_port);
}

bool Master::start()
{
  if (!m_rpcManager) {
    MINIROS_ERROR("No RPC Manager was attached");
    return false;
  }

  MINIROS_DEBUG("Creating XmlRpc server");

  setupBindings();

  if (!m_rpcManager->start(m_port))
    return false;

  // It was done in roslaunch by calling generate_run_id() function.
  // It should be uuid.uuid1()
  std::string uuid = generatePseudoUuid();
  m_parameterStorage.setParam("master", "/run_id", uuid);

  MINIROS_DEBUG("Master startup complete.");
  return true;
}

void Master::stop()
{
  if (m_rpcManager)
    m_rpcManager->shutdown();
}

bool Master::ok() const
{
  return m_rpcManager && !m_rpcManager->isShuttingDown();
}

void Master::setupBindings()
{
  // Core master part.
  m_rpcManager->bindEx4("registerPublisher", this, &Master::registerPublisher);
  m_rpcManager->bindEx3("unregisterPublisher", this, &Master::unregisterPublisher);
  m_rpcManager->bindEx4("registerSubscriber", this, &Master::registerSubscriber);
  m_rpcManager->bindEx3("unregisterSubscriber", this, &Master::unregisterSubscriber);
  m_rpcManager->bindEx2("getPublishedTopics", this, &Master::getPublishedTopics);
  m_rpcManager->bindEx1("getTopicTypes", this, &Master::getTopicTypes);
  m_rpcManager->bindEx1("getSystemState", this, &Master::getSystemState);

  m_rpcManager->bindEx2("lookupService", this, &Master::lookupService);
  m_rpcManager->bindEx3("unregisterService", this, &Master::unregisterService);
  m_rpcManager->bindEx4("registerService", this, &Master::registerService);
  m_rpcManager->bindEx2("lookupNode", this, &Master::lookupNode);

  // Rosparam part.
  m_rpcManager->bindEx2("hasParam", this, &Master::hasParam);
  m_rpcManager->bindEx3("setParam", this, &Master::setParam);
  m_rpcManager->bindEx2("getParam", this, &Master::getParam);
  m_rpcManager->bindEx2("deleteParam", this, &Master::deleteParam);
  m_rpcManager->bindEx2("searchParam", this, &Master::searchParam);
  m_rpcManager->bindEx3("subscribeParam", this, &Master::subscribeParam);
  m_rpcManager->bindEx3("unsubscribeParam", this, &Master::unsubscribeParam);
  m_rpcManager->bindEx1("getParamNames", this, &Master::getParamNames);
}

void Master::setResolveNodeIP(bool resolv)
{
  m_handler.setResolveNodeIP(resolv);
}

void Master::update()
{
  m_handler.update();
}

Master::RpcValue Master::lookupService(const std::string& caller_id, const std::string& service, Connection* connection)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  std::string uri = m_handler.lookupService(requesterInfo, service);

  RpcValue res = RpcValue::Array(3);;
  if (uri.empty()) {
    res[0] = -1;
    res[1] = std::string("Failed to lookup service '" + service + "'");
  } else {
    res[0] = 1;
    res[1] = std::string("rosrpc URI: [") + uri + "]";
    res[3] = uri;
  }
  return res;
}

Master::RpcValue Master::registerService(const std::string& caller_id, const std::string& service,
  const std::string& service_api, const std::string& caller_api, Connection* connection)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct r = m_handler.registerService(requesterInfo, service, service_api);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api, Connection* connection)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  ReturnStruct r = m_handler.unregisterService(requesterInfo, service, service_api);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::getTopicTypes(const std::string& topic, Connection*)
{
  std::map<std::string, std::string> types = m_handler.getTopicTypes(topic);

  RpcValue xmlTopics = RpcValue::Array(types.size());
  int index = 0;
  for (auto [key, val] : types) {
    RpcValue payload;
    payload[0] = key;
    payload[1] = val;
    xmlTopics[index++] = payload;
  }

  RpcValue res = RpcValue::Array(3);;
  res[0] = 1;
  res[1] = "getTopicTypes";
  res[2] = xmlTopics;
  return res;
}

Master::RpcValue Master::getSystemState(const std::string& caller_id, Connection* connection)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "getSystemState";

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
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }

  MasterHandler::SystemState state = m_handler.getSystemState(requesterInfo);

  RpcValue listoftypes = RpcValue::Array(3);

  writeXml(state.publishers, listoftypes[0]);
  writeXml(state.subscribers, listoftypes[1]);
  writeXml(state.services, listoftypes[2]);

  res[2] = listoftypes;
  return res;
}

Master::RpcValue Master::getPublishedTopics(const std::string& caller_id, const std::string& subgraph, Connection* connection)
{
  RpcValue res = RpcValue::Array(3);

  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  auto topics = m_handler.getPublishedTopics(requesterInfo, subgraph);
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
  const std::string& type, const std::string& caller_api, Connection* connection)
{
  MINIROS_INFO("PUBLISHING: caller_id=\"%s\" caller_api=%s topic=\"%s\"", caller_id.c_str(), caller_api.c_str(), topic.c_str());

  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = m_handler.registerPublisher(requesterInfo, topic, type);
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* connection)
{
  MINIROS_INFO("UNPUBLISHING caller_id=\"%s\" caller_api=%s topic=\"%s\"", caller_id.c_str(), caller_api.c_str(), topic.c_str());

  RpcValue res = RpcValue::Array(3);
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;


  int ret = m_handler.unregisterPublisher(requesterInfo, topic);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, Connection* connection)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  ReturnStruct st = m_handler.registerSubscriber(requesterInfo, topic, type);
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* connection)
{
  RpcValue res = RpcValue::Array(3);
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }
  requesterInfo.callerApi = caller_api;

  int ret = m_handler.unregisterSubscriber(requesterInfo, topic);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::lookupNode(const std::string& caller_id, const std::string& node, Connection* connection)
{
  RequesterInfo requesterInfo;
  if (!requesterInfo.assign(caller_id, connection->getfd())) {
    MINIROS_WARN("Failed to read network address of caller %s", caller_id.c_str());
  }

  std::string api = m_handler.lookupNode(requesterInfo, node);
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "lookupNode";
  res[2] = api;
  return res;
}

Master::RpcValue Master::getTime(Connection*)
{
  throw std::runtime_error("NOT IMPLEMENTED YET!");
}

Master::RpcValue Master::hasParam(const std::string& caller_id, const std::string& topic, Connection* /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "hasParam";
  res[2] = m_parameterStorage.hasParam(caller_id, topic);
  return res;
}

Master::RpcValue Master::setParam(
  const std::string& caller_api, const std::string& key, const RpcValue& value, Connection* /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "setParam";
  m_parameterStorage.setParam(caller_api, key, value);
  res[2] = std::string("parameter ") + key + std::string(" set");
  return res;
}

Master::RpcValue Master::getParam(const std::string& caller_id, const std::string& topic, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  RpcValue value = m_parameterStorage.getParam(caller_id, topic);
  if (!value.valid()) {
    res[0] = 0;
    res[1] = std::string("Parameter ") + topic + std::string(" is not set");
  } else {
    res[0] = 1;
    res[1] = "getParam";
    res[2] = value;
  }
  return res;
}

Master::RpcValue Master::deleteParam(const std::string& caller_id, const std::string& key, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[2] = 0;
  if (m_parameterStorage.deleteParam(caller_id, key)) {
    res[1] = "deleteParam success";
  } else {
    res[1] = "deleteParam param not found";
  }
  return res;
}

Master::RpcValue Master::searchParam(const std::string& caller_id, const std::string& key, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  std::string foundKey = m_parameterStorage.searchParam(caller_id, key);
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
  const std::string& key, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "subscribeParam done";
  const RpcValue* val = m_parameterStorage.subscribeParam(caller_id, caller_api, key);
  res[2] = val ? *val : RpcValue::Dict();
  return res;
}

Master::RpcValue Master::unsubscribeParam(const std::string& caller_id, const std::string& caller_api,
  const std::string& key, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "unsubscribeParam done";
  if (m_parameterStorage.unsubscribeParam(caller_id, caller_api, key))
    res[2] = 1;
  return res;
}


Master::RpcValue Master::getParamNames(const std::string& caller_id, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "getParamNames";

  RpcValue response;
  int index = 0;
  for (std::string s : m_parameterStorage.getParamNames(caller_id)) {
    response[index++] = s;
  }

  res[2] = response;
  return res;
}

} // namespace master
} // namespace miniros
