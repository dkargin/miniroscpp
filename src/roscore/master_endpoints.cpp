//
// Created by dkargin on 8/19/25.
//

#include "master_endpoints.h"
#include "miniros_favicon.h"

#include "http/http_server.h"
#include "http/http_filters.h"


namespace miniros {
namespace master {

Error MasterRootEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
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

Error MasterFaviconEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body)
{
  responseHeader.statusCode = 200;
  responseHeader.status = "OK";
  responseHeader.contentType = "image/x-icon";

  std::string_view vFavicon((const char*)(favicon), sizeof(favicon));
  body += vFavicon;

  return Error::Ok;
}

Error NodeInfoEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
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

Error TopicInfoEndpoint::handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
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

}
}