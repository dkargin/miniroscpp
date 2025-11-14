//
// Created by dkargin on 8/19/25.
//

#include "master_endpoints.h"
#include "miniros_favicon.h"

#include "http/http_server.h"
#include "http/http_filters.h"
#include "requester_info.h"
#include "miniros/internal/json_tools.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"



namespace miniros {
namespace master {

Error MasterRootEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
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

Error MasterFaviconEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body)
{
  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = "image/x-icon";

  std::string_view vFavicon((const char*)(favicon), sizeof(favicon));
  body += vFavicon;

  return Error::Ok;
}

Error NodeInfoEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
  http::HttpResponseHeader& responseHeader, std::string& body)
{
  std::string_view name = http::getNameFromUrlPath(frame.getPath(), "/node/", true);

  responseHeader.contentType = "text/html";
  body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || !internal->renderNodeInfo(name, body)) {
    responseHeader.statusCode = 404;
    responseHeader.status = "Unknown node";
  } else {
    responseHeader.statusCode = 200;
    responseHeader.status = "OK";
  }
  body += "</body></html>";
  return Error::Ok;
}

Error TopicInfoEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
  http::HttpResponseHeader& responseHeader, std::string& body)
{
  std::string_view name = http::getNameFromUrlPath(frame.getPath(), "/topic/", true);

  responseHeader.contentType = "text/html";
  body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || internal->renderTopicInfo(name, body)) {
    responseHeader.statusCode = 404;
    responseHeader.status = "Unknown topic";
  } else {
    responseHeader.statusCode = 200;
    responseHeader.status = "OK";
  }
  body += "</body></html>";
  return Error::Ok;
}

Error PublishedTopicsEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
  http::HttpResponseHeader& responseHeader, std::string& body)
{
  if (!internal)
    return Error::InternalError;

  // Create RequesterInfo for the HTTP client
  RequesterInfo requesterInfo;
  requesterInfo.assign("http_client", clientInfo);

  // Get published topics
  auto topics = internal->handler.getPublishedTopics(requesterInfo, "");

  // Build JSON structure as a simple object: {"/topic1": "std_msgs/String", ...}
  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue jsonResponse;
  for (size_t i = 0; i < topics.size(); i++) {
    jsonResponse[topics[i][0]] = topics[i][1];  // topic name -> topic type
  }

  // Serialize to JSON
  std::ostringstream oss;
  miniros::JsonState state;
  jsonResponse.writeJson(oss, state, {});
  body = oss.str();

  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = "application/json";

  return Error::Ok;
}

Error TopicTypesEndpoint::handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
  http::HttpResponseHeader& responseHeader, std::string& body)
{
  if (!internal)
    return Error::InternalError;

  // Get all topic types
  auto topicTypes = internal->regManager.getTopicTypes("http_client");

  // Build JSON structure as a simple object: {"/topic1": "std_msgs/String", ...}
  using RpcValue = XmlRpc::XmlRpcValue;
  RpcValue jsonResponse;
  for (const auto& [topicName, topicType] : topicTypes) {
    jsonResponse[topicName] = topicType;
  }

  // Serialize to JSON
  std::ostringstream oss;
  miniros::JsonState state;
  jsonResponse.writeJson(oss, state, {});
  body = oss.str();

  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = "application/json";

  return Error::Ok;
}

}
}