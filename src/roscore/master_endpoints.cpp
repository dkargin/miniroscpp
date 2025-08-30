//
// Created by dkargin on 8/19/25.
//

#include "master_endpoints.h"
#include "miniros_favicon.h"
#include "http/http_server.h"


namespace miniros {
namespace master {

/// Get final name from path: "/node/ws1/node1" returns "/ws1/node1"
std::string_view getNameFromUrlPath(const std::string_view& path)
{
  size_t pos = path.find_first_of("/\\");
  if(pos != std::string_view::npos) {
    return path.substr(pos);
  }
  return path;
}

/// Get final name from path: "/node/ws1/node1" returns "/ws1/node1"
std::string_view getNameFromUrlPath(const std::string_view& path, const std::string_view& prefix)
{
  // Even if path == prefix, it is still invalid input, because output name will be empty.
  if (path.size() <= prefix.size()) {
    return {};
  }

  for (size_t i = 0; i < prefix.size(); i++) {
    if (path[i] != prefix[i]) {
      return {};
    }
  }
  // Ensure trailing slash is added.
  return path.substr(prefix.size() - 1);
}

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
  std::string_view name = getNameFromUrlPath(frame.getPath(), "/node/");

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
  std::string_view name = getNameFromUrlPath(frame.getPath(), "/topic/");

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