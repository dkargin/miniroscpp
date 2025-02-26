//
// Created by dkargin on 2/11/25.
//

#include "master.h"

namespace miniros {
namespace master {

Master::Master(std::shared_ptr<RPCManager> manager)
  : m_handler(manager, &m_regManager), m_parameterStorage(&m_regManager)
{
  m_manager = manager;
  // TODO: Read environment.
  // split the URI (if it's valid) into host and port
  // if (!network.splitURI(ROS.ROS_MASTER_URI, ref _host, ref _port))
  {
    m_host = "localhost";
    m_port = 11311;
    MINIROS_WARN("Invalid XMLRPC uri. Using ROS_MASTER_URI=http://%s:%d", m_host.c_str(), m_port);
  }
}

Master::~Master()
{
  if (m_manager) {
    m_manager->unbind(this);
  }
}

bool Master::start()
{
  if (!m_manager) {
    MINIROS_ERROR("No RPC Manager was attached");
    return false;
  }

  MINIROS_DEBUG("Creating XmlRpc server");

  setupBindings();

  if (!m_manager->start(m_port))
    return false;

  m_parameterStorage.setParam("master", "/run_id", RpcValue());

  MINIROS_DEBUG("Master startup complete.");
  return true;
}

void Master::stop()
{
  if (m_manager)
    m_manager->shutdown();
}

bool Master::ok() const
{
  return m_manager && !m_manager->isShuttingDown();
}

void Master::setupBindings()
{
  m_manager->bindEx4("registerPublisher", this, &Master::registerPublisher);
  m_manager->bindEx3("unregisterPublisher", this, &Master::unregisterPublisher);
  m_manager->bindEx4("registerSubscriber", this, &Master::registerSubscriber);
  m_manager->bindEx3("unregisterSubscriber", this, &Master::unregisterSubscriber);
  m_manager->bindEx2("getPublishedTopics", this, &Master::getPublishedTopics);
  m_manager->bindEx1("getTopicTypes", this, &Master::getTopicTypes);
  m_manager->bindEx1("getSystemState", this, &Master::getSystemState);

  m_manager->bindEx2("lookupService", this, &Master::lookupService);
  m_manager->bindEx3("unregisterService", this, &Master::unregisterService);
  m_manager->bindEx4("registerService", this, &Master::registerService);

  m_manager->bindEx2("hasParam", this, &Master::hasParam);
  m_manager->bindEx3("setParam", this, &Master::setParam);
  m_manager->bindEx2("getParam", this, &Master::getParam);
  m_manager->bindEx2("deleteParam", this, &Master::deleteParam);
  // master_node.bind("subscribeParam", tobind(new Func<std::string, std::string, std::string, std::string,
  // RpcValue>(subscribeParam)));
  m_manager->bindEx1("getParamNames", this, &Master::getParamNames);

  m_manager->bindEx2("lookupNode", this, &Master::lookupNode);

  // master_node.bind("Time", tobind(new Func<std::string, std::string, std::string, std::string, RpcValue>(Time)));
  // master_node.bind("Duration", tobind(new Func<std::string, std::string, std::string, std::string,
  // RpcValue>(Duration))); master_node.bind("get_rostime", tobind(new Func<std::string, std::string, std::string,
  // std::string, RpcValue>(get_rostime))); master_node.bind("get_time", tobind(new Func<std::string, std::string,
  // std::string, std::string, RpcValue>(get_time)));

  // roscore is also a publisher/subsriber.
  // TODO: Probably it should be moved elsewhere.
  m_manager->bindEx1("getPublications", this, &Master::getPublications);
  m_manager->bindEx1("getSubscriptions", this, &Master::getSubscriptions);
  m_manager->bindEx3("requestTopic", this, &Master::requestTopic);
  m_manager->bindEx3("publisherUpdate", this, &Master::publisherUpdate);
  m_manager->bindEx3("paramUpdate", this, &Master::paramUpdate);
  m_manager->bindEx1("getBusStats", this, &Master::getBusStats);
  m_manager->bindEx1("getBusInfo", this, &Master::getBusInfo);
  m_manager->bindEx1("getPid", this, &Master::getPid);
}

Master::RpcValue Master::lookupService(const std::string& caller_id, const std::string& service, Connection*)
{
  std::string uri = m_handler.lookupService(caller_id, service);

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
  const std::string& caller_api, const std::string& service_api, Connection* conn)
{
  ReturnStruct r = m_handler.registerService(caller_id, service, service_api, caller_api, conn);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api, Connection*)
{
  ReturnStruct r = m_handler.unregisterService(caller_id, service, service_api);

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

Master::RpcValue Master::getSystemState(const std::string& caller_id, Connection*)
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

  MasterHandler::SystemState state = m_handler.getSystemState(caller_id);

  RpcValue listoftypes = RpcValue::Array(3);

  writeXml(state.publishers, listoftypes[0]);
  writeXml(state.subscribers, listoftypes[1]);
  writeXml(state.services, listoftypes[2]);

  res[2] = listoftypes;
  return res;
}

Master::RpcValue Master::getPublishedTopics(const std::string& caller_id, const std::string& subgraph, Connection*)
{
  RpcValue res = RpcValue::Array(3);

  auto topics = m_handler.getPublishedTopics("", "");
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
  const std::string& type, const std::string& caller_api, Connection* conn)
{
  MINIROS_INFO("PUBLISHING: caller_id=\"%s\" caller_api=%s topic=\"%s\"", caller_id.c_str(), caller_api.c_str(), topic.c_str());

  ReturnStruct st = m_handler.registerPublisher(caller_id, topic, type, caller_api, conn);
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* /*conn*/)
{
  MINIROS_INFO("UNPUBLISHING caller_id=\"%s\" caller_api=%s topic=\"%s\"", caller_id.c_str(), caller_api.c_str(), topic.c_str());

  RpcValue res = RpcValue::Array(3);
  int ret = m_handler.unregisterPublisher(caller_id, topic, caller_api);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, Connection* conn)
{
  ReturnStruct st = m_handler.registerSubscriber(caller_id, topic, type, caller_api, conn);
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterSubscriber(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  int ret = m_handler.unregisterSubscriber(caller_id, topic, caller_api);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::lookupNode(const std::string& caller_id, const std::string& node, Connection* /*conn*/)
{
  std::string api = m_handler.lookupNode(caller_id, node);
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
  if (!value) {
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


// This is SlaveAPI request to node.
Master::RpcValue Master::getPublications(const std::string& caller_id, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "publications";

  RpcValue response;

  // response.Size = 0;
  auto current = m_handler.getPublishedTopics(caller_id, "");

  for (int i = 0; i < current.size(); i++) {
    RpcValue pub;
    // TODO: Expect problems here
    pub[0] = current[i][0];
    pub[1] = current[i][1];
    response[i] = pub;
  }
  res[2] = response;
  return res;
}

// This is SlaveAPI request to node.
Master::RpcValue Master::getSubscriptions(const std::string& caller_id, Connection*)
{
  throw std::runtime_error("NOT IMPLEMENTED YET!");
}

// This is SlaveAPI request to node.
Master::RpcValue Master::requestTopic(
  const std::string& caller_id, const std::string& topic, const RpcValue& protocols, Connection*)
{
  throw std::runtime_error("NOT IMPLEMENTED YET!");
  return {};
}

// This is SlaveAPI request to node.
Master::RpcValue Master::publisherUpdate(
  const std::string& caller_id, const std::string& topic, const RpcValue& publishers, Connection*)
{
  return {};
}

// This is SlaveAPI request to node.
Master::RpcValue Master::paramUpdate(
  const std::string& caller_id, const std::string& key, const RpcValue& value, Connection*)
{
  // TODO: Implement
  RpcValue result = RpcValue::Array(3);
  result[0] = 0;
  result[1] = "Not implemented yet";
  result[2] = 0;
  return result;
}

// This is SlaveAPI request to node.
Master::RpcValue Master::getBusStats(const std::string& caller_id, Connection* conn)
{
  // TODO: Implement
  RpcValue result = RpcValue::Array(3);
  result[0] = 0;
  result[1] = "Not implemented yet";
  result[2] = 0;
  return result;
}

// This is SlaveAPI request to node.
Master::RpcValue Master::getBusInfo(const std::string& caller_id, Connection* conn)
{
  // TODO: Implement
  RpcValue result = RpcValue::Array(3);
  result[0] = 0;
  result[1] = "Not implemented yet";
  result[2] = 0;
  return result;
}

// This is SlaveAPI request to node.
Master::RpcValue Master::getPid(const std::string& caller_id, Connection* conn)
{
  MINIROS_INFO("getPid from %s", caller_id.c_str());
  RpcValue result = RpcValue::Array(3);
  result[0] = 1;
  result[1] = "Master PID";
  result[2] = m_handler.getPid(caller_id);
  return result;
}

} // namespace master
} // namespace miniros
