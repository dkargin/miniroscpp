//
// Reverse HTTP proxy tunnel. Websocket upgrade is intentionally rejected
// until a later pass. The splice never blocks the poll thread.
//

#ifndef MINIROS_HTTP_PROXY_H
#define MINIROS_HTTP_PROXY_H

#include <functional>
#include <memory>
#include <string>
#include <string_view>

#include "miniros/network/net_address.h"

namespace miniros {

class PollSet;

namespace network {
class NetSocket;
}

namespace http {

struct HttpParserFrame;

/// True for `/p` and `/p/...`. Other paths stay on the normal HTTP server.
inline bool isHttpProxyPath(std::string_view path)
{
  return path == "/p" || (path.size() > 2 && path[0] == '/' && path[1] == 'p' && path[2] == '/');
}

/// Installed by the master on its HttpServer. Called on the poll thread once
/// request headers for `/p/...` are complete, before the body is buffered.
class HttpProxyAcceptor {
public:
  virtual ~HttpProxyAcceptor() = default;

  /// Take ownership of @p client. The socket is already detached from HttpServer.
  /// @return true if the acceptor now owns the socket (including after writing an error).
  virtual bool acceptHttpProxy(const HttpParserFrame& frame,
    const std::shared_ptr<network::NetSocket>& client) = 0;
};

/// Where one accepted `/p/{node}` request is forwarded.
struct HttpProxyTarget {
  network::NetAddress address;
  /// Literal host used in Host and in Location rewriting. No DNS.
  std::string host;
  int port = 0;
  /// `/p/camera/grabber`
  std::string publicPrefix;
  /// Path on the node, always starts with `/`.
  std::string path;
  /// Query without the leading `?`.
  std::string query;
  /// Request Content-Length, or -1 when the request has no body.
  int64_t contentLength = -1;
};

/// One client socket spliced to one upstream socket.
class HttpProxyTunnel {
public:
  using DoneFn = std::function<void()>;

  /// Start a non-blocking tunnel. On immediate failure the client socket is closed
  /// and nullptr is returned (an HTTP error was written when possible).
  static std::shared_ptr<HttpProxyTunnel> open(PollSet* poll,
    std::shared_ptr<network::NetSocket> client,
    const HttpParserFrame& inbound,
    HttpProxyTarget target,
    DoneFn onDone);

  ~HttpProxyTunnel();

  /// Ask the poll thread to close both sockets. Safe from any thread.
  void requestClose();

  /// True after both sockets have been dropped.
  bool finished() const;

private:
  static constexpr size_t kMaxHeader = 16 * 1024;
  static constexpr size_t kBufCap = 16 * 1024;
  static constexpr int kConnectTimeoutMs = 2000;
  static constexpr int kIdleTimeoutMs = 60000;
  static constexpr int kMaxPumps = 8;

  struct Impl;
  explicit HttpProxyTunnel(const std::shared_ptr<Impl>& impl);
  std::shared_ptr<Impl> impl_;
};

/// Write a tiny response with an empty body and close the socket.
void writeHttpStatusAndClose(const std::shared_ptr<network::NetSocket>& socket, int code, const char* reason);

} // namespace http
} // namespace miniros

#endif
