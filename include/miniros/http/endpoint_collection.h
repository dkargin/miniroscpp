//
// Created by dkargin on 3/21/26.
//

#ifndef MINIROS_ENDPOINT_COLLECTION_H
#define MINIROS_ENDPOINT_COLLECTION_H

#include <mutex>
#include <memory>

#include "miniros/macros.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/callback_queue.h"

namespace miniros {
namespace http {
namespace internal {

/// A collection of http endpoints for server.
/// This object is shared between http::Server and http::ServerConnection instances.
class MINIROS_DECL EndpointCollection {
public:
  struct Binding {
    std::unique_ptr<EndpointFilter> filter;
    std::shared_ptr<EndpointHandler> endpoint;
    CallbackQueuePtr callbackQueue;

    Binding(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& endpoint, const CallbackQueuePtr& cb)
      : filter(std::move(filter)), endpoint(endpoint), callbackQueue(cb)
    {}
  };


  Error registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const CallbackQueuePtr& cb)
  {
    std::unique_lock lock(guard_);
    endpoints_.emplace_back(std::move(filter), handler, cb);
    return Error::Ok;
  }

  std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> findEndpoint(const HttpParserFrame& frame) const
  {
    std::unique_lock<std::mutex> lock(guard_);

    for (const Binding& binding: endpoints_) {
      if (binding.filter->check(frame)) {
        return {binding.endpoint, binding.callbackQueue};
      }
    }

    return {};
  }

protected:
  /// A collection of endpoints.
  std::vector<Binding> endpoints_;

  /// A mutex for endpoints.
  mutable std::mutex guard_;
};

}
}
}
#endif // MINIROS_ENDPOINT_COLLECTION_H
