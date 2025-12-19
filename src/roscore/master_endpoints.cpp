//
// Created by dkargin on 8/19/25.
//

#include "master_endpoints.h"
#include "miniros_favicon.h"

#include "http/http_filters.h"
#include "http/http_request.h"
#include "http/http_server.h"
#include "miniros/internal/json_tools.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "requester_info.h"

namespace miniros {
namespace master {

Error MasterRootEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  if (!internal)
    return Error::InternalError;

  std::string body = "<!doctype html><html><title>Mini ROS master</title><body>";
  internal->renderMasterStatus(body);
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error MasterFaviconEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view vFavicon((const char*)(favicon), sizeof(favicon));
  request->setResponseBody(std::string(vFavicon), "image/x-icon");
  request->setResponseStatusOk();
  return Error::Ok;
}

Error NodeInfoEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view name = http::getNameFromUrlPath(request->path(), "/node/", true);

  std::string body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || !internal->renderNodeInfo(name, body)) {
    request->setResponseStatus(404, "Node not found");
  } else {
    request->setResponseStatusOk();
  }
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  return Error::Ok;
}

Error TopicInfoEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
{
  std::string_view name = http::getNameFromUrlPath(request->path(), "/topic/", true);

  std::string body = "<!doctype html><html><title>Mini ROS master</title><body>";

  if (name.empty() || internal->renderTopicInfo(name, body)) {
    request->setResponseStatus(404, "Topic not found");
  } else {
    request->setResponseStatusOk();
  }
  body += "</body></html>";
  request->setResponseBody(body, "text/html");
  return Error::Ok;
}

Error PublishedTopicsEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
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

  request->setResponseBody(oss.str(), "application/json");
  request->setResponseStatusOk();

  return Error::Ok;
}

Error TopicTypesEndpoint::handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request)
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
  request->setResponseStatusOk();
  request->setResponseBody(oss.str(), "application/json");

  return Error::Ok;
}

}
}