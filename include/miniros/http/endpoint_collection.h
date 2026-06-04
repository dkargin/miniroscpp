//
// Created by dkargin on 3/21/26.
//

#ifndef MINIROS_ENDPOINT_COLLECTION_H
#define MINIROS_ENDPOINT_COLLECTION_H

#include <mutex>
#include <memory>
#include <list>
#include <algorithm>

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
    std::shared_ptr<EndpointHandler> handler;
    CallbackQueuePtr callbackQueue;

    Binding(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const CallbackQueuePtr& cb)
      : filter(std::move(filter)), handler(handler), callbackQueue(cb)
    {}
  };


  void registerEndpoint(std::unique_ptr<EndpointFilter>&& filter, const std::shared_ptr<EndpointHandler>& handler, const CallbackQueuePtr& cb)
  {
    std::unique_lock lock(guard_);
    endpoints_.emplace_back(std::move(filter), handler, cb);
  }

  void unregisterAll(const std::shared_ptr<EndpointHandler>& handler)
  {
    std::unique_lock lock(guard_);
    std::remove_if(endpoints_.begin(), endpoints_.end(), [handler](const Binding& bind) {
      return bind.handler == handler;
    });
  }

  std::pair<std::shared_ptr<EndpointHandler>, std::shared_ptr<CallbackQueue>> findEndpoint(const HttpParserFrame& frame) const
  {
    std::unique_lock<std::mutex> lock(guard_);

    for (const Binding& binding: endpoints_) {
      if (binding.filter->check(frame)) {
        return {binding.handler, binding.callbackQueue};
      }
    }

    return {};
  }

protected:
  /// A collection of endpoints.
  std::list<Binding> endpoints_;

  /// A mutex for endpoints.
  mutable std::mutex guard_;
};

}
}
}
#endif // MINIROS_ENDPOINT_COLLECTION_H
