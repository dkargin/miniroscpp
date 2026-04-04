//
// Created for WebSocket endpoint support.
//

#ifndef MINIROS_HTTP_ENDPOINTS_WEBSOCKET_H
#define MINIROS_HTTP_ENDPOINTS_WEBSOCKET_H

#include "miniros/http/http_endpoint.h"
#include "miniros/http/websocket.h"
#include <functional>
#include <memory>

namespace miniros {
namespace network {
  class NetSocket;
}

namespace http {

/// Handler for WebSocket upgrade requests.
/// Validates WebSocket handshake and creates WebSocket object upon successful upgrade.
class WebSocketHandler : public EndpointHandler {
public:
  /// Callback type for when a WebSocket connection is established.
  /// @param ws - the WebSocket object that was created
  using OnWebSocketCreated = std::function<void(std::shared_ptr<WebSocket> ws)>;

  WebSocketHandler(OnWebSocketCreated onCreated = nullptr);
  virtual ~WebSocketHandler();

  /// Handle WebSocket upgrade request.
  /// Validates the handshake and creates WebSocket object if successful.
  /// @param clientInfo - connection information
  /// @param request - HTTP request containing WebSocket upgrade headers
  /// @returns Error::Ok if handshake is successful and WebSocket is created,
  ///          Error::InvalidValue if handshake is invalid
  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request) override;

  /// Handle completion of WebSocket upgrade.
  /// Extracts the socket from the connection and creates WebSocket object.
  /// @param connection - the HTTP server connection that is being upgraded
  /// @param clientInfo - connection information
  /// @param request - the original request that triggered the upgrade
  /// @returns Error::Ok if upgrade was handled successfully
  Error upgradeComplete(const std::shared_ptr<network::NetSocket>& connection, const network::ClientInfo& clientInfo, const std::shared_ptr<HttpRequest>& request) override;

  /// Set callback for when WebSocket is created.
  void setOnWebSocketCreated(OnWebSocketCreated onCreated);

  /// Get the callback (for internal use).
  OnWebSocketCreated getOnWebSocketCreated() const { return onCreated_; }

private:
  OnWebSocketCreated onCreated_;
};

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_ENDPOINTS_WEBSOCKET_H
