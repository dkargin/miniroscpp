//
// WebSocket connection implementation.
// Manages a WebSocket connection after handshake is complete.
// Includes frame encoding/decoding and event handling.
//

#include "miniros/http/websocket.h"
#include "miniros/network/socket.h"
#include "miniros/io/poll_set.h"
#include "miniros/internal/code_location.h"
#include "miniros/console.h"

namespace miniros {
namespace http {

WebSocket::WebSocket(std::shared_ptr<network::NetSocket> socket)
  : socket_(socket), pollSet_(nullptr), onMessage_(nullptr), closed_(false), socketFd_(-1)
{
  if (socket_) {
    socketFd_ = socket_->fd();
  }
}

WebSocket::~WebSocket()
{
  close();
}

void WebSocket::close()
{
  if (closed_) return;
  closed_ = true;

  unregisterFromPollSet();

  if (socket_) {
    socket_->close();
    socket_.reset();
  }
}

Error WebSocket::registerWithPollSet(PollSet* pollSet, OnMessage onMessage)
{
  if (!pollSet || !socket_ || closed_) {
    return Error::InvalidValue;
  }

  pollSet_ = pollSet;
  onMessage_ = onMessage;

  if (socketFd_ < 0) {
    socketFd_ = socket_->fd();
  }

  if (socketFd_ >= 0) {
    // Use a lambda that captures 'this' to handle events
    // We need to keep a shared_ptr to this WebSocket to prevent it from being deleted
    // while the callback is active. We'll use a shared_from_this pattern or track it differently.
    // For now, we'll use a shared_ptr<void> to track the socket itself.
    std::shared_ptr<network::NetSocket> trackedSocket = socket_;
    bool added = pollSet_->addSocket(
      socketFd_,
      PollSet::EventIn,
      [this](int flags) {
        return this->handleEvents(flags);
      },
      trackedSocket,
      internal::CodeLocation::make(__FILE__, __LINE__)
    );

    if (!added) {
      pollSet_ = nullptr;
      return Error::SystemError;
    }
  }

  return Error::Ok;
}

void WebSocket::unregisterFromPollSet()
{
  if (pollSet_ && socketFd_ >= 0) {
    pollSet_->delSocket(socketFd_);
    pollSet_ = nullptr;
  }
}

Error WebSocket::sendMessage(const std::string& message)
{
  if (closed_ || !socket_) {
    return Error::InvalidValue;
  }

  std::string frame = encodeTextFrame(message);
  auto [written, err] = socket_->send(frame.c_str(), frame.size(), nullptr);

  if (err != Error::Ok && err != Error::WouldBlock) {
    close();
    return err;
  }

  return Error::Ok;
}

void WebSocket::setOnMessage(OnMessage onMessage)
{
  onMessage_ = onMessage;
}

int WebSocket::handleEvents(int flags)
{
  if (closed_ || !socket_) {
    return 0;
  }

  if (flags & PollSet::EventIn) {
    // Read data from socket
    auto [transferred, err] = socket_->recv(buffer_, nullptr);

    if (err == Error::EndOfFile || err == Error::SystemError) {
      close();
      return 0;
    }

    if (err == Error::Ok || err == Error::WouldBlock) {
      // Try to decode frames
      size_t offset = 0;
      std::string message;
      while (offset < buffer_.size() && decodeFrame(buffer_, offset, message)) {
        handleMessage(message);
        // Remove processed data from buffer
        if (offset < buffer_.size()) {
          buffer_ = buffer_.substr(offset);
          offset = 0;
        } else {
          buffer_.clear();
          break;
        }
      }
    }
  }

  if (closed_) {
    return 0;
  }

  return PollSet::EventIn;
}

void WebSocket::handleMessage(const std::string& message)
{
  if (onMessage_) {
    onMessage_(message);
  }
}

std::string WebSocket::encodeTextFrame(const std::string& message)
{
  std::string frame;
  frame.reserve(message.size() + 10);

  // First byte: FIN=1, RSV=0, Opcode=1 (text frame)
  frame.push_back(0x81);

  // Second byte: MASK=0 (server doesn't mask), payload length
  size_t len = message.size();
  if (len < 126) {
    frame.push_back(static_cast<char>(len));
  } else if (len < 65536) {
    frame.push_back(126);
    frame.push_back(static_cast<char>((len >> 8) & 0xFF));
    frame.push_back(static_cast<char>(len & 0xFF));
  } else {
    frame.push_back(127);
    for (int i = 7; i >= 0; i--) {
      frame.push_back(static_cast<char>((len >> (i * 8)) & 0xFF));
    }
  }

  // Payload
  frame += message;

  return frame;
}

bool WebSocket::decodeFrame(const std::string& data, size_t& offset, std::string& message)
{
  if (data.size() < offset + 2) {
    return false; // Not enough data
  }

  size_t pos = offset;
  uint8_t firstByte = static_cast<uint8_t>(data[pos++]);
  uint8_t secondByte = static_cast<uint8_t>(data[pos++]);

  bool fin = (firstByte & 0x80) != 0;
  uint8_t opcode = firstByte & 0x0F;
  bool masked = (secondByte & 0x80) != 0;
  uint64_t payloadLen = secondByte & 0x7F;

  if (!fin || opcode != 1) {
    // Only handle simple text frames for this implementation
    return false;
  }

  // Read extended payload length if needed
  if (payloadLen == 126) {
    if (data.size() < pos + 2) return false;
    payloadLen = (static_cast<uint8_t>(data[pos]) << 8) | static_cast<uint8_t>(data[pos + 1]);
    pos += 2;
  } else if (payloadLen == 127) {
    if (data.size() < pos + 8) return false;
    payloadLen = 0;
    for (int i = 0; i < 8; i++) {
      payloadLen = (payloadLen << 8) | static_cast<uint8_t>(data[pos + i]);
    }
    pos += 8;
  }

  // Read masking key if present
  uint8_t maskingKey[4] = {0};
  if (masked) {
    if (data.size() < pos + 4) return false;
    for (int i = 0; i < 4; i++) {
      maskingKey[i] = static_cast<uint8_t>(data[pos + i]);
    }
    pos += 4;
  }

  // Read payload
  if (data.size() < pos + payloadLen) return false;

  message.resize(payloadLen);
  for (size_t i = 0; i < payloadLen; i++) {
    if (masked) {
      message[i] = data[pos + i] ^ maskingKey[i % 4];
    } else {
      message[i] = data[pos + i];
    }
  }

  offset = pos + payloadLen;
  return true;
}

} // namespace http
} // namespace miniros
