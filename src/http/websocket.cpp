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

#include <cstdint>

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
      size_t offset = 0;
      DecodedFrame frame;
      while (offset < buffer_.size() && decodeFrame(buffer_, offset, frame)) {
        if (frame.opcode == 0x1) {
          handleMessage(frame.payload);
        } else if (frame.opcode == 0x8) {
          close();
          return 0;
        } else if (frame.opcode == 0x9 && socket_ && !closed_) {
          const std::string pong = encodeControlFrame(0xA, frame.payload);
          socket_->send(pong.c_str(), pong.size(), nullptr);
        }
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
  return encodeControlFrame(0x1, message);
}

std::string WebSocket::encodeControlFrame(uint8_t opcode, const std::string& payload)
{
  std::string frame;
  frame.reserve(payload.size() + 10);
  frame.push_back(static_cast<char>(0x80 | (opcode & 0x0F)));

  const size_t len = payload.size();
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
  frame += payload;
  return frame;
}

bool WebSocket::decodeFrame(const std::string& data, size_t& offset, DecodedFrame& frame)
{
  if (data.size() < offset + 2) {
    return false;
  }

  size_t pos = offset;
  const uint8_t firstByte = static_cast<uint8_t>(data[pos++]);
  const uint8_t secondByte = static_cast<uint8_t>(data[pos++]);

  frame.fin = (firstByte & 0x80) != 0;
  frame.opcode = firstByte & 0x0F;
  const bool masked = (secondByte & 0x80) != 0;
  uint64_t payloadLen = secondByte & 0x7F;

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

  uint8_t maskingKey[4] = {0};
  if (masked) {
    if (data.size() < pos + 4) return false;
    for (int i = 0; i < 4; i++) {
      maskingKey[i] = static_cast<uint8_t>(data[pos + i]);
    }
    pos += 4;
  }

  if (data.size() < pos + payloadLen) return false;

  frame.payload.resize(static_cast<size_t>(payloadLen));
  for (size_t i = 0; i < payloadLen; i++) {
    if (masked) {
      frame.payload[i] = data[pos + i] ^ maskingKey[i % 4];
    } else {
      frame.payload[i] = data[pos + i];
    }
  }

  offset = pos + static_cast<size_t>(payloadLen);
  return true;
}

} // namespace http
} // namespace miniros
