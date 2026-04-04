//
// WebSocket connection class.
// Manages a WebSocket connection after handshake is complete.
// Includes frame encoding/decoding and event handling.
//

#ifndef MINIROS_HTTP_WEBSOCKET_H
#define MINIROS_HTTP_WEBSOCKET_H

#include "miniros/errors.h"
#include <memory>
#include <functional>
#include <string>
#include <atomic>

namespace miniros {

class PollSet;
namespace network {
  class NetSocket;
}

namespace http {

/// WebSocket connection object.
/// Manages a WebSocket connection after handshake is complete.
/// Handles frame encoding/decoding, message sending/receiving, and event management.
class MINIROS_DECL WebSocket {
public:
  /// Callback type for when a message is received.
  /// @param message - the received message
  using OnMessage = std::function<void(const std::string& message)>;

  /// Create WebSocket from a socket (after handshake).
  /// @param socket - the socket to use for WebSocket communication
  explicit WebSocket(std::shared_ptr<network::NetSocket> socket);
  ~WebSocket();

  /// Get the underlying socket.
  std::shared_ptr<network::NetSocket> socket() const { return socket_; }

  /// Close the WebSocket connection.
  void close();

  /// Register this WebSocket with a PollSet for event handling.
  /// This will automatically handle incoming messages and call the OnMessage callback.
  /// @param pollSet - the PollSet to register with
  /// @param onMessage - callback to call when a message is received
  /// @returns Error::Ok if registration was successful
  Error registerWithPollSet(PollSet* pollSet, OnMessage onMessage = nullptr);

  /// Unregister this WebSocket from the PollSet.
  void unregisterFromPollSet();

  /// Send a text message over the WebSocket.
  /// @param message - the message to send
  /// @returns Error::Ok if message was sent successfully
  Error sendMessage(const std::string& message);

  /// Set callback for when messages are received.
  void setOnMessage(OnMessage onMessage);

  /// Check if the WebSocket is closed.
  bool isClosed() const { return closed_; }

private:
  /// Handle events from PollSet.
  int handleEvents(int flags);

  /// Handle a received message.
  void handleMessage(const std::string& message);

  /// Encode a text frame.
  static std::string encodeTextFrame(const std::string& message);

  /// Decode a WebSocket frame (simple implementation for text frames).
  static bool decodeFrame(const std::string& data, size_t& offset, std::string& message);

  std::shared_ptr<network::NetSocket> socket_;
  PollSet* pollSet_;
  OnMessage onMessage_;
  std::string buffer_;
  std::atomic<bool> closed_;
  int socketFd_;
};

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_WEBSOCKET_H
