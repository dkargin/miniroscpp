//
// Master's HTTP reverse-proxy route table. The URL mount is chosen by the caller.
//

#ifndef MINIROS_ROSCORE_HTTP_PROXY_H
#define MINIROS_ROSCORE_HTTP_PROXY_H

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "miniros/http/http_proxy.h"
#include "miniros/network/net_address.h"

namespace miniros {
namespace http {
class HttpServer;
struct HttpParserFrame;
class HttpProxyTunnel;
}

namespace master {

class RegistrationManager;

class HttpProxyHub : public http::HttpProxyAcceptor,
                       public std::enable_shared_from_this<HttpProxyHub> {
public:
  /// @p mount is an absolute path with no trailing slash, for example `/p`.
  /// It is fixed for the life of the hub. An empty mount matches nothing.
  HttpProxyHub(RegistrationManager* registrations, std::string mount);

  void attach(http::HttpServer* server);
  void detach();

  void setEnabled(bool enabled);
  bool enabled() const;

  /// The mount passed to the constructor, without a trailing slash.
  const std::string& mount() const { return mount_; }

  /// Public path for one node, for example `/p/camera`. @p nodeId may start with `/`.
  std::string publicPrefix(const std::string& nodeId) const;

  bool acceptHttpProxy(const http::HttpParserFrame& frame,
    const std::shared_ptr<network::NetSocket>& client) override;

  /// Close tunnels that dial nodes imported from @p peerKey.
  void closePeer(const std::string& peerKey);

  void closeAll();

private:
  struct Reservation {
    std::string node;
    std::string peer;
  };

  struct Live {
    std::shared_ptr<http::HttpProxyTunnel> tunnel;
    std::string peer;
  };

  void release(const Reservation& slot);
  bool tryReserve(const Reservation& slot);

  RegistrationManager* registrations_ = nullptr;
  http::HttpServer* server_ = nullptr;
  std::string mount_;
  bool enabled_ = true;

  mutable std::mutex mu_;
  int total_ = 0;
  std::vector<std::pair<std::string, int>> perNode_;
  std::vector<std::pair<std::string, int>> perPeer_;
  std::vector<Live> tunnels_;
};

/// Resolve a host to a literal address. DNS runs only when @p allowDns is set,
/// and that must not be the poll thread.
network::NetAddress dialAddressForHost(const std::string& host, int port, bool allowDns);

} // namespace master
} // namespace miniros

#endif
