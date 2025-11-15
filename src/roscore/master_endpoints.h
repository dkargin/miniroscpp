//
// Created by dkargin on 8/19/25.
//

#ifndef MINIROS_MASTER_ENDPOINTS_H
#define MINIROS_MASTER_ENDPOINTS_H

#include "miniros/http/http_endpoint.h"
#include "master_internal.h"

namespace miniros {
namespace master {

class MasterRootEndpoint : public http::EndpointHandler {
public:
  MasterRootEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body);

  Master::Internal* internal = nullptr;
};

class NodeInfoEndpoint : public http::EndpointHandler {
public:
  NodeInfoEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body);

  Master::Internal* internal = nullptr;
};

class TopicInfoEndpoint : public http::EndpointHandler {
public:
  TopicInfoEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body);

  Master::Internal* internal = nullptr;
};

class PublishedTopicsEndpoint : public http::EndpointHandler {
public:
  PublishedTopicsEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body);

  Master::Internal* internal = nullptr;
};

/// Handles GET /favicon.ico.
class MasterFaviconEndpoint : public http::EndpointHandler {
  Error handle(const http::HttpFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body);
};

}
}

#endif // MINIROS_MASTER_ENDPOINTS_H
