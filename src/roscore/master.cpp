//
// Created by dkargin on 2/11/25.
//

#include "master.h"

namespace miniros {


Master::Master()
{
    // split the URI (if it's valid) into host and port
    //if (!network.splitURI(ROS.ROS_MASTER_URI, ref _host, ref _port))
    {
        MINIROS_WARN("Invalid XMLRPC uri. Using WHATEVER I WANT instead.");
        _host = "localhost";
        _port = 11311;
    }
}

bool Master::start()
{
    //creatre handler??
    MINIROS_DEBUG("Creating XmlRpc server");

    manager.reset(new RPCManager(_port));
    setupBindings();

    if (!manager->start())
      return false;

    handler.setParam("master", "/run_id", XmlRpcValue());

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
    manager->bindEx("getTopicTypes", wrap2(this, &Master::getTopicTypes));
    manager->bindEx("getSystemState", wrap0(this, &Master::getSystemState));

    manager->bindEx("lookupService", wrap2(this, &Master::lookupService));
    manager->bindEx("unregisterService", wrap3(this, &Master::unregisterService));
    manager->bindEx("registerService", wrap4(this, &Master::registerService));

    manager->bindEx("hasParam", wrap2(this, &Master::hasParam));
    manager->bindEx("setParam", wrap3(this, &Master::setParam));
    manager->bindEx("getParam", wrap2(this, &Master::getParam));
    manager->bindEx("deleteParam", wrap2(this, &Master::deleteParam));
    //master_node.bind("subscribeParam", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(subscribeParam)));
    manager->bindEx("getParamNames", wrap1(this, &Master::getParamNames));

    manager->bindEx("lookupNode", wrap2(this, &Master::lookupNode));
    //master_node.bind("getBusStats", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(getBusStats)));
    //master_node.bind("getBusInfo", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(getBusInfo)));

    //master_node.bind("Time", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(Time)));
    //master_node.bind("Duration", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(Duration)));
    //master_node.bind("get_rostime", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(get_rostime)));
    //master_node.bind("get_time", tobind(new Func<std::string, std::string, std::string, std::string, XmlRpcValue>(get_time)));

    // roscore is also a publisher/subsriber.
    // TODO: Probably it should be moved elsewhere.
    manager->bindEx("getPublications", wrap1(this, &Master::getPublications));
    manager->bindEx("getSubscriptions", wrap1(this, &Master::getSubscriptions));
    manager->bindEx("requestTopic", wrap3(this, &Master::requestTopic));
    manager->bindEx("publisherUpdate", wrap3(this, &Master::publisherUpdate));
    manager->bindEx("paramUpdate", wrap3(this, &Master::paramUpdate));
}

Master::XmlRpcValue Master::lookupService(const std::string& caller_id, const std::string& service, Connection*)
{
    ReturnStruct r = handler.lookupService(caller_id, service);

    XmlRpcValue res;
    res[0] = r.statusCode;
    res[1] = r.statusMessage;
    if (r.value)
        res[2] = r.value;
    return res;
}

Master::XmlRpcValue Master::registerService(const std::string& caller_id, const std::string& service,
  const std::string& caller_api, const std::string& service_api, Connection*)
{
    ReturnStruct r = handler.registerService(caller_id, service, service_api, caller_api);

    XmlRpcValue res;
    res[0] = r.statusCode;
    res[1] = r.statusMessage;
    res[2] = r.value;
    return res;
}

Master::XmlRpcValue Master::unregisterService(const std::string& caller_id, const std::string& service,
  const std::string& service_api, Connection*)
{
    ReturnStruct r = handler.unregisterService(caller_id, service, service_api);

    XmlRpcValue res;
    res[0] = r.statusCode;
    res[1] = r.statusMessage;
    res[2] = r.value;
    return res;
}

Master::XmlRpcValue Master::getTopicTypes(const std::string& topic, const std::string& caller_id, Connection*)
{
    std::map<std::string, std::string> types = handler.getTopicTypes(topic);

    XmlRpcValue value;
    int index = 0;
    for(auto [key, val]: types)
    {
        XmlRpcValue payload;
        payload[0] = key;
        payload[1] = val;
        value[index++] = payload;
    }

    XmlRpcValue res;
    res[0] = 1;
    res[1] = "getTopicTypes";
    res[2] = value;
    return res;
}

Master::XmlRpcValue Master::getPublications(const std::string& caller_id, Connection*)
{
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "publications";

    XmlRpcValue response;

    //response.Size = 0;
    auto current = handler.getPublishedTopics("", "");

    for (int i = 0; i < current.size(); i ++)
    {
        XmlRpcValue pub;
        // TODO: Expect problems here
        pub[0] = current[i][0];
        pub[1] = current[i][1];
        response[i] =  pub;
    }
    res[2] = response;
    return res;
}

Master::XmlRpcValue Master::getSubscriptions(const std::string& caller_id, Connection*)
{
  throw std::runtime_error("NOT IMPLEMENTED YET!");
}

Master::XmlRpcValue Master::requestTopic(const std::string& caller_id, const std::string& topic, const XmlRpcValue& protocols, Connection*)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
    return {};
}

Master::XmlRpcValue Master::publisherUpdate(const std::string& caller_id, const std::string& topic, const XmlRpcValue& publishers, Connection*)
{
    //throw std::runtime_error("NOT IMPLEMENTED YET!");
    //mlRpcValue parm = XmlRpcValue.Create(ref parms);
    //List<string> pubs = new List<string>();
    //for (int idx = 0; idx < parm[2].Size; idx++)
    //    pubs.Add(parm[2][idx].Get<string>());
    //if (pubUpdate(parm[1].Get<string>(), pubs))
    //    manager->responseInt(1, "", 0)(result);
    //else
    //{
    //    EDB.WriteLine("Unknown Error");
    //    manager->responseInt(0, "Unknown Error or something", 0)(result);
    //}

    //EDB.WriteLine("TopicManager is updating publishers for " + topic);
    //Subscription sub = null;
    //lock (subs_mutex)
    //{
    //    if (shutting_down) return false;
    //    foreach (Subscription s in subscriptions)
    //    {
    //        if (s.name != topic || s.IsDropped)
    //            continue;
    //        sub = s;
    //        break;
    //    }
    //}
    //if (sub != null)
    //    return sub.pubUpdate(pubs);
    //else
    //    EDB.WriteLine("got a request for updating publishers of topic " + topic +
    //                  ", but I don't have any subscribers to that topic.");
    //return false;
    return {};
}

Master::XmlRpcValue Master::paramUpdate(const std::string& caller_id, const std::string& parameter_key, const XmlRpcValue& value,Connection*)
{
  // TODO: Implement
  XmlRpcValue result;
  result[0] = 0;
  result[1] = "Not implemented yet";
  result[2] = 0;
  return result;
}

Master::XmlRpcValue Master::getSystemState(Connection*)
{
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "getSystemState";

    auto systemstatelist = handler.getSystemState("");

    XmlRpcValue listoftypes;
    XmlRpcValue listofvalues;

    int index = 0;

    for (const auto& types: systemstatelist) //publisher, subscriber, services
    {
        int bullshitindex = 0;
        XmlRpcValue bullshit = new XmlRpcValue();
        if (types.size() > 0)
        {
            for (const auto& l: types)
            {
                int typeindex = 0;
                XmlRpcValue typelist;
                //XmlRpcValue value = new XmlRpcValue();
                typelist[typeindex++] =  l[0];
                XmlRpcValue payload;
                for (int i = 1; i < l.size(); i++)
                {
                    payload[i - 1] = l[i];
                }

                typelist[typeindex++] = payload;
                bullshit[bullshitindex++] = typelist;
            }
        }
        else
        {
            bullshit[bullshitindex++] = XmlRpcValue();
        }

        listoftypes[index++] = bullshit;
    }

    res[2] = listoftypes;
    return res;
}

Master::XmlRpcValue Master::getPublishedTopics(const std::string& caller_id, const std::string& subgraph, Connection*)
{
    XmlRpcValue res;
    auto publishedtopics = handler.getPublishedTopics("", "");
    res[0] = 1;
    res[1] = "current system state";

    XmlRpcValue listofvalues;
    int index = 0;
    for (const auto& l: publishedtopics)
    {
        XmlRpcValue value;
        value[0] = l[0]; //Topic Name
        value[1] = l[1]; // Topic type
        listofvalues[index] = value;
        index++;
    }
    res[2] = listofvalues;
    return res;
}

Master::XmlRpcValue Master::registerPublisher(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, Connection* /*conn*/)
{
    MINIROS_INFO("PUBLISHING: %s : %s : %s",  caller_id.c_str(), caller_api.c_str(), topic.c_str());

    ReturnStruct st =  handler.registerPublisher(caller_id, topic, type, caller_api);
    XmlRpcValue res;
    res[0] = st.statusCode;
    res[1] = st.statusMessage;
    res[2] = st.value;
    return res;
}

Master::XmlRpcValue Master::unregisterPublisher(const std::string& caller_id, const std::string& topic, const std::string& caller_api,
  Connection* /*conn*/)
{
    MINIROS_INFO("UNPUBLISHING: %s : %s", caller_id.c_str(), caller_api.c_str());

    XmlRpcValue res = new XmlRpcValue();
    int ret = handler.unregisterPublisher(caller_id, topic, caller_api);
    res[0] = ret;
    res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
    return res;
}

Master::XmlRpcValue Master::registerSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& type, const std::string& caller_api, Connection* /*conn*/)
{
    ReturnStruct st =  handler.registerSubscriber(caller_id, topic, type, caller_api);
    XmlRpcValue res;
    res[0] = st.statusCode;
    res[1] = st.statusMessage;
    res[2] = st.value;
    return res;
}

Master::XmlRpcValue Master::unregisterSubscriber(const std::string& caller_id, const std::string& topic,
  const std::string& caller_api, Connection* /*conn*/)
{
    XmlRpcValue res;
    res[0] = handler.unregisterSubscriber(caller_id, topic, caller_api);
    res[1] = std::string("unregistered ") + caller_id + std::string("as provder of ") + topic;
    return res;
}

Master::XmlRpcValue Master::lookupNode(const std::string& topic, const std::string& caller_id, Connection* conn)
{
    std::string api = handler.lookupNode(caller_id, topic);
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "lookupNode";
    res[2] = api;
    return res;
}

Master::XmlRpcValue Master::getBusStatus(/*[In] [Out]*/const XmlRpcValue& parms, /*[In] [Out]*/XmlRpcValue& result)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
}

Master::XmlRpcValue Master::getBusInfo(/*[In] [Out]*/const XmlRpcValue& parms, /*[In] [Out]*/XmlRpcValue& result)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
}

Master::XmlRpcValue Master::getTime(Connection*)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
}

Master::XmlRpcValue Master::hasParam(const std::string& caller_id, const std::string& topic, Connection* /*conn*/)
{
    XmlRpcValue res = new XmlRpcValue();//XmlRpcValue.Create(ref result), parm = XmlRpcValue.Create(ref parms);
    res[0] = 1;
    res[1] = "hasParam";
    //std::string caller_id = parm[0].Getstd::string();
    //std::string topic = parm[1].Getstd::string();
    res[2] = handler.hasParam(caller_id, topic);
    return res;
}

Master::XmlRpcValue Master::setParam(const std::string& caller_api, const std::string& topic, const XmlRpcValue& value, Connection* /*conn*/)
{
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "setParam";
    handler.setParam(caller_api, topic,value);
    res[2] = std::string("parameter ") + topic + std::string(" set");
    return res;
}

Master::XmlRpcValue Master::getParam(const std::string& caller_id, const std::string& topic, Connection*)
{
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "getParam";

    XmlRpcValue value = handler.getParam(caller_id, topic);
    if (!value)
    {
        res[0] = 0;
        res[1] = std::string("Parameter ") + topic + std::string(" is not set");
        value = std::string("");
    }
    res[2] = value;
    return res;
}

Master::XmlRpcValue Master::deleteParam(const std::string& caller_d, const std::string& key, Connection*)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
    //XmlRpcValue parm = new XmlRpcValue(), result = new XmlRpcValue(), payload = new XmlRpcValue();
    //parm.Set(0, this_node.Name);
    //parm.Set(1, mapped_key);
    //if (!master.execute("deleteParam", parm, ref result, ref payload, false))
    //    return false;
    //return true;
}

Master::XmlRpcValue Master::getParamNames(const std::string& caller_id, Connection*)
{
    XmlRpcValue res;
    res[0] = 1;
    res[1] = "getParamNames";

    XmlRpcValue response;
    int index = 0;
    for (std::string s: handler.getParamNames(caller_id)) {
        response[index++] = s;
    }

    res[2] = response;
    return res;
}

Master::XmlRpcValue Master::Param(const XmlRpcValue& parms, XmlRpcValue& result, Connection*)
{
    throw std::runtime_error("NOT IMPLEMENTED YET!");
    //return new XmlRpcValue;
}

} // namespace miniros