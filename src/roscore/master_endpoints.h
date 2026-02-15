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

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

class NodeInfoEndpoint : public http::EndpointHandler {
public:
  NodeInfoEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

class TopicInfoEndpoint : public http::EndpointHandler {
public:
  TopicInfoEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

class PublishedTopicsEndpoint : public http::EndpointHandler {
public:
  PublishedTopicsEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

class TopicTypesEndpoint : public http::EndpointHandler {
public:
  TopicTypesEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

/// Handles GET /favicon.ico.
class MasterFaviconEndpoint : public http::EndpointHandler {
  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;
};

class MultimasterConnectEndpoint : public http::EndpointHandler {
public:
  MultimasterConnectEndpoint(Master::Internal* internal) : internal(internal) {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override;

  Master::Internal* internal = nullptr;
};

}
}

#endif // MINIROS_MASTER_ENDPOINTS_H
