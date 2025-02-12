//
// Created by dkargin on 2/11/25.
//

#include "master.h"

namespace miniros {
namespace master {
Master::Master()
{
  // split the URI (if it's valid) into host and port
  // if (!network.splitURI(ROS.ROS_MASTER_URI, ref _host, ref _port))
  {
    MINIROS_WARN("Invalid XMLRPC uri. Using WHATEVER I WANT instead.");
    _host = "localhost";
    _port = 11311;
  }
}

bool Master::start()
{
  // creatre handler??
  MINIROS_DEBUG("Creating XmlRpc server");

  manager.reset(new RPCManager(_port));
  setupBindings();

  if (!manager->start())
    return false;

  handler.setParam("master", "/run_id", RpcValue());

#ifdef FIX_ROSOUT
  RosOut.start();
#endif

  MINIROS_DEBUG("Master startup complete.");
  return true;
}

void Master::stop()
{
  if (manager)
    manager->shutdown();
}

bool Master::ok() const
{
  return manager && !manager->isShuttingDown();
}

void Master::setupBindings()
{
  manager->bindEx("registerPublisher", wrap4(this, &Master::registerPublisher));
  manager->bindEx("unregisterPublisher", wrap3(this, &Master::unregisterPublisher));
  manager->bindEx("registerSubscriber", wrap4(this, &Master::registerSubscriber));
  manager->bindEx("unregisterSubscriber", wrap3(this, &Master::unregisterSubscriber));
  manager->bindEx("getPublishedTopics", wrap2(this, &Master::getPublishedTopics));
  manager->bindEx("getTopicTypes", wrap1(this, &Master::getTopicTypes));
  manager->bindEx("getSystemState", wrap1(this, &Master::getSystemState));

  manager->bindEx("lookupService", wrap2(this, &Master::lookupService));
  manager->bindEx("unregisterService", wrap3(this, &Master::unregisterService));
  manager->bindEx("registerService", wrap4(this, &Master::registerService));

  manager->bindEx("hasParam", wrap2(this, &Master::hasParam));
  manager->bindEx("setParam", wrap3(this, &Master::setParam));
  manager->bindEx("getParam", wrap2(this, &Master::getParam));
  manager->bindEx("deleteParam", wrap2(this, &Master::deleteParam));
  // master_node.bind("subscribeParam", tobind(new Func<std::string, std::string, std::string, std::string,
  // RpcValue>(subscribeParam)));
  manager->bindEx("getParamNames", wrap1(this, &Master::getParamNames));

  manager->bindEx("lookupNode", wrap2(this, &Master::lookupNode));
  // std::string, RpcValue>(getBusInfo)));

  // master_node.bind("Time", tobind(new Func<std::string, std::string, std::string, std::string, RpcValue>(Time)));
  // master_node.bind("Duration", tobind(new Func<std::string, std::string, std::string, std::string,
  // RpcValue>(Duration))); master_node.bind("get_rostime", tobind(new Func<std::string, std::string, std::string,
  // std::string, RpcValue>(get_rostime))); master_node.bind("get_time", tobind(new Func<std::string, std::string,
  // std::string, std::string, RpcValue>(get_time)));

  // roscore is also a publisher/subsriber.
  // TODO: Probably it should be moved elsewhere.
  manager->bindEx("getPublications", wrap1(this, &Master::getPublications));
  manager->bindEx("getSubscriptions", wrap1(this, &Master::getSubscriptions));
  manager->bindEx("requestTopic", wrap3(this, &Master::requestTopic));
  manager->bindEx("publisherUpdate", wrap3(this, &Master::publisherUpdate));
  manager->bindEx("paramUpdate", wrap3(this, &Master::paramUpdate));
  manager->bindEx("getBusStats", wrap1(this, &Master::getBusStats));
  manager->bindEx("getBusInfo", wrap1(this, &Master::getBusInfo));
}

Master::RpcValue Master::lookupService(const std::string& caller_id, const std::string& service, Connection*)
{
  ReturnStruct r = handler.lookupService(caller_id, service);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  if (r.value)
    res[2] = r.value;
  return res;
}

Master::RpcValue Master::registerService(const std::string& caller_id, const std::string& service,
  const std::string& caller_api, const std::string& service_api, Connection*)
{
  ReturnStruct r = handler.registerService(caller_id, service, service_api, caller_api);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::unregisterService(
  const std::string& caller_id, const std::string& service, const std::string& service_api, Connection*)
{
  ReturnStruct r = handler.unregisterService(caller_id, service, service_api);

  RpcValue res = RpcValue::Array(3);;
  res[0] = r.statusCode;
  res[1] = r.statusMessage;
  res[2] = r.value;
  return res;
}

Master::RpcValue Master::getTopicTypes(const std::string& topic, Connection*)
{
  std::map<std::string, std::string> types = handler.getTopicTypes(topic);

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

  MasterHandler::SystemState state = handler.getSystemState(caller_id);

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

  auto topics = handler.getPublishedTopics("", "");
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
  const std::string& type, const std::string& caller_api, Connection* /*conn*/)
{
  MINIROS_INFO("PUBLISHING: %s : %s : %s", caller_id.c_str(), caller_api.c_str(), topic.c_str());

  ReturnStruct st = handler.registerPublisher(caller_id, topic, type, caller_api);
  RpcValue res = RpcValue::Array(3);
  res[0] = st.statusCode;
  res[1] = st.statusMessage;
  res[2] = st.value;
  return res;
}

Master::RpcValue Master::unregisterPublisher(
  const std::string& caller_id, const std::string& topic, const std::string& caller_api, Connection* /*conn*/)
{
  MINIROS_INFO("UNPUBLISHING: %s : %s", caller_id.c_str(), caller_api.c_str());

  RpcValue res = RpcValue::Array(3);
  int ret = handler.unregisterPublisher(caller_id, topic, caller_api);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, Connection* /*conn*/)
{
  ReturnStruct st = handler.registerSubscriber(caller_id, topic, type, caller_api);
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
  int ret = handler.unregisterSubscriber(caller_id, topic, caller_api);
  res[0] = 1;
  res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
  res[2] = ret;
  return res;
}

Master::RpcValue Master::lookupNode(const std::string& topic, const std::string& caller_id, Connection* conn)
{
  std::string api = handler.lookupNode(caller_id, topic);
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
  res[2] = handler.hasParam(caller_id, topic);
  return res;
}

Master::RpcValue Master::setParam(
  const std::string& caller_api, const std::string& key, const RpcValue& value, Connection* /*conn*/)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "setParam";
  handler.setParam(caller_api, key, value);
  res[2] = std::string("parameter ") + key + std::string(" set");
  return res;
}

Master::RpcValue Master::getParam(const std::string& caller_id, const std::string& topic, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  RpcValue value = handler.getParam(caller_id, topic);
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

Master::RpcValue Master::deleteParam(const std::string& caller_d, const std::string& key, Connection*)
{
  throw std::runtime_error("NOT IMPLEMENTED YET!");
  // RpcValue parm = new RpcValue(), result = new RpcValue(), payload = new RpcValue();
  // parm.Set(0, this_node.Name);
  // parm.Set(1, mapped_key);
  // if (!master.execute("deleteParam", parm, ref result, ref payload, false))
  //     return false;
  // return true;
}

Master::RpcValue Master::getParamNames(const std::string& caller_id, Connection*)
{
  RpcValue res = RpcValue::Array(3);
  res[0] = 1;
  res[1] = "getParamNames";

  RpcValue response;
  int index = 0;
  for (std::string s : handler.getParamNames(caller_id)) {
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
  auto current = handler.getPublishedTopics(caller_id, "");

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
  // throw std::runtime_error("NOT IMPLEMENTED YET!");
  // mlRpcValue parm = RpcValue.Create(ref parms);
  // List<string> pubs = new List<string>();
  // for (int idx = 0; idx < parm[2].Size; idx++)
  //     pubs.Add(parm[2][idx].Get<string>());
  // if (pubUpdate(parm[1].Get<string>(), pubs))
  //     manager->responseInt(1, "", 0)(result);
  // else
  //{
  //     EDB.WriteLine("Unknown Error");
  //     manager->responseInt(0, "Unknown Error or something", 0)(result);
  // }

  // EDB.WriteLine("TopicManager is updating publishers for " + topic);
  // Subscription sub = null;
  // lock (subs_mutex)
  //{
  //     if (shutting_down) return false;
  //     foreach (Subscription s in subscriptions)
  //     {
  //         if (s.name != topic || s.IsDropped)
  //             continue;
  //         sub = s;
  //         break;
  //     }
  // }
  // if (sub != null)
  //     return sub.pubUpdate(pubs);
  // else
  //     EDB.WriteLine("got a request for updating publishers of topic " + topic +
  //                   ", but I don't have any subscribers to that topic.");
  // return false;
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

} // namespace master
} // namespace miniros
