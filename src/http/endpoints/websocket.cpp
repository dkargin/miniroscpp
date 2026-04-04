//
// Created for WebSocket endpoint support.
//

#include "miniros/console.h"
#include "miniros/http/endpoints/websocket.h"
#include "miniros/http/websocket.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server_connection.h"
#include "miniros/http/http_tools.h"
#include "miniros/b64/encode.h"
#include "miniros/network/socket.h"
#include "miniros/internal/sha1.h"

#include <sstream>
#include <iomanip>
#include <cstring>
#include <algorithm>

#define MINIROS_PACKAGE_NAME "http"

namespace miniros {
namespace http {

namespace {

// Base64 encode a string
std::string base64Encode(const std::string& input) {
  base64::Encoder encoder;
  // Calculate output size: (input_size + 2) / 3 * 4
  size_t outputSize = ((input.size() + 2) / 3) * 4;
  std::string output;
  output.resize(outputSize + 10); // Extra space for safety
  char* outPtr = &output[0];
  int written = encoder.encode(input.data(), static_cast<int>(input.size()), outPtr);
  outPtr += written;
  written += encoder.encode_end(outPtr);
  output.resize(static_cast<size_t>(written));
  return output;
}

// Generate WebSocket accept key from client key
std::string generateWebSocketAccept(const std::string& clientKey) {
  const std::string magic = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
  std::string combined = clientKey + magic;
  std::string hash = miniros::internal::SHA1::compute(combined);
  return base64Encode(hash);
}

// Case-insensitive header comparison
bool headerEquals(const std::string& a, const std::string& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (std::tolower(a[i]) != std::tolower(b[i])) return false;
  }
  return true;
}

} // anonymous namespace

WebSocketHandler::WebSocketHandler(OnWebSocketCreated onCreated)
  : onCreated_(onCreated)
{
}

WebSocketHandler::~WebSocketHandler()
{
}

void WebSocketHandler::setOnWebSocketCreated(OnWebSocketCreated onCreated)
{
  onCreated_ = onCreated;
}

Error WebSocketHandler::handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request)
{
  // Validate WebSocket upgrade request
  if (request->method() != HttpMethod::Get) {
    LOCAL_ERROR("WebSocketHandler: Invalid method, expected GET");
    request->setResponseStatus(405, "Method Not Allowed");
    return Error::InvalidValue;
  }

  // Check Upgrade header
  std::string upgrade = request->getHeader("Upgrade");
  if (!headerEquals(upgrade, "websocket")) {
    LOCAL_ERROR("WebSocketHandler: Missing or invalid Upgrade header");
    request->setResponseStatus(400, "Bad Request");
    return Error::InvalidValue;
  }

  // Check Connection header
  std::string connection = request->getHeader("Connection");
  std::string connectionLower = connection;
  std::transform(connectionLower.begin(), connectionLower.end(), connectionLower.begin(), ::tolower);
  if (connectionLower.find("upgrade") == std::string::npos) {
    LOCAL_ERROR("WebSocketHandler: Missing or invalid Connection header");
    request->setResponseStatus(400, "Bad Request");
    return Error::InvalidValue;
  }

  // Check Sec-WebSocket-Key header
  std::string secKey = request->getHeader("Sec-WebSocket-Key");
  if (secKey.empty()) {
    LOCAL_ERROR("WebSocketHandler: Missing Sec-WebSocket-Key header");
    request->setResponseStatus(400, "Bad Request");
    return Error::InvalidValue;
  }

  // Check Sec-WebSocket-Version header (should be 13)
  std::string secVersion = request->getHeader("Sec-WebSocket-Version");
  if (secVersion != "13") {
    LOCAL_ERROR("WebSocketHandler: Invalid Sec-WebSocket-Version, expected 13");
    request->setResponseStatus(400, "Bad Request");
    request->setResponseHeader("Sec-WebSocket-Version", "13");
    return Error::InvalidValue;
  }

  // Generate accept key
  std::string acceptKey = generateWebSocketAccept(secKey);

  // Prepare WebSocket handshake response
  request->setResponseStatus(101, "Switching Protocols");
  request->setResponseHeader("Upgrade", "websocket");
  request->setResponseHeader("Connection", "Upgrade");
  request->setResponseHeader("Sec-WebSocket-Accept", acceptKey);
  request->setResponseHeader("X-Upgrade-Handler", "true");

  LOCAL_DEBUG("WebSocketHandler: Handshake successful for client fd=%d", clientInfo.fd);

  return Error::Ok;
}

Error WebSocketHandler::upgradeComplete(const std::shared_ptr<network::NetSocket>& socket, const network::ClientInfo& clientInfo, const std::shared_ptr<HttpRequest>& request)
{
  (void)request; // Not used but kept for interface consistency

  // Extract socket from connection
  if (!socket) {
    LOCAL_ERROR("WebSocketHandler::upgradeComplete: failed to extract socket");
    return Error::InternalError;
  }

  // Create WebSocket object
  auto ws = std::make_shared<WebSocket>(socket);

  // Call callback if set
  if (onCreated_) {
    onCreated_(ws);
  }

  LOCAL_DEBUG("WebSocketHandler::upgradeComplete: WebSocket object created for client fd=%d", clientInfo.fd);

  return Error::Ok;
}

} // namespace http
} // namespace miniros
