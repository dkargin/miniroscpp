//
// Route lookup is a short scan of the node table. Byte movement lives in
// http::HttpProxyTunnel and returns as soon as a socket would block.
//

#include "http_proxy.h"

#include <algorithm>
#include <cctype>
#include <string_view>

#define MINIROS_PACKAGE_NAME "master"

#include "registration_manager.h"
#include "node_ref.h"

#include "miniros/http/http_server.h"
#include "miniros/http/http_tools.h"
#include "miniros/network/url.h"
#include "rosconsole/local_log.h"

namespace miniros {
namespace master {
namespace {

constexpr int kMaxTunnels = 64;
constexpr int kMaxPerNode = 8;
constexpr int kMaxPerPeer = 16;

bool badSegment(std::string_view path)
{
  size_t start = 0;
  while (start <= path.size()) {
    const size_t slash = path.find('/', start);
    const size_t end = slash == std::string_view::npos ? path.size() : slash;
    const std::string_view seg = path.substr(start, end - start);
    if (seg == "." || seg == "..")
      return true;
    if (seg.empty() && end != path.size() && start != 0)
      return true;
    if (end == path.size())
      break;
    start = end + 1;
  }
  return false;
}

bool prefixAllows(const std::vector<std::string>& prefixes, const std::string& remainder)
{
  if (remainder == "/RPC2" || remainder.rfind("/RPC2/", 0) == 0)
    return false;
  for (const std::string& prefix : prefixes) {
    if (prefix == "/")
      return true;
    if (remainder == prefix)
      return true;
    if (!prefix.empty() && prefix.back() == '/') {
      if (remainder.rfind(prefix, 0) == 0)
        return true;
    } else if (remainder.rfind(prefix + "/", 0) == 0) {
      return true;
    }
  }
  return false;
}

std::string nodeKey(const std::string& id)
{
  if (!id.empty() && id[0] == '/')
    return id.substr(1);
  return id;
}

bool headerHasUpgrade(const http::HttpParserFrame& frame)
{
  for (const auto& field : frame.fields) {
    if (field.name.size() == 7) {
      const char* n = field.name.c_str();
      if ((n[0] == 'U' || n[0] == 'u') && (n[1] == 'P' || n[1] == 'p') &&
          (n[2] == 'G' || n[2] == 'g') && (n[3] == 'R' || n[3] == 'r') &&
          (n[4] == 'A' || n[4] == 'a') && (n[5] == 'D' || n[5] == 'd') &&
          (n[6] == 'E' || n[6] == 'e'))
        return true;
    }
  }
  return false;
}

int bump(std::vector<std::pair<std::string, int>>& rows, const std::string& key, int delta, int limit)
{
  for (auto& row : rows) {
    if (row.first == key) {
      if (delta > 0 && row.second >= limit)
        return -1;
      row.second += delta;
      const int now = row.second;
      if (now <= 0) {
        row.swap(rows.back());
        rows.pop_back();
      }
      return now;
    }
  }
  if (delta > 0) {
    if (limit <= 0)
      return -1;
    rows.emplace_back(key, delta);
    return delta;
  }
  return 0;
}

} // namespace

network::NetAddress dialAddressForHost(const std::string& host, int port, bool allowDns)
{
  if (host.empty() || port <= 0 || port > 65535)
    return {};
  network::NetAddress v4 = network::NetAddress::fromIp4String(host, port);
  if (v4.valid())
    return v4;
  network::NetAddress v6 = network::NetAddress::fromIp6String(host, port);
  if (v6.valid())
    return v6;
  if (!allowDns)
    return {};
  return network::NetAddress::fromString(network::NetAddress::AddressUnspecified, host, port);
}

HttpProxyHub::HttpProxyHub(RegistrationManager* registrations, std::string mount)
  : registrations_(registrations)
  , mount_(std::move(mount))
{
  while (!mount_.empty() && mount_.back() == '/')
    mount_.pop_back();
}

std::string HttpProxyHub::publicPrefix(const std::string& nodeId) const
{
  std::string key = nodeId;
  if (!key.empty() && key.front() == '/')
    key.erase(key.begin());
  std::string prefix = mount_;
  prefix.push_back('/');
  prefix += key;
  return prefix;
}

void HttpProxyHub::attach(http::HttpServer* server)
{
  server_ = server;
  if (server_)
    server_->setHttpProxyAcceptor(this, mount_);
}

void HttpProxyHub::detach()
{
  if (server_)
    server_->setHttpProxyAcceptor(nullptr, {});
  server_ = nullptr;
  closeAll();
}

void HttpProxyHub::setEnabled(bool enabled)
{
  enabled_ = enabled;
}

bool HttpProxyHub::enabled() const
{
  return enabled_;
}

void HttpProxyHub::release(const Reservation& slot)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (total_ > 0)
    --total_;
  bump(perNode_, slot.node, -1, kMaxPerNode);
  if (!slot.peer.empty())
    bump(perPeer_, slot.peer, -1, kMaxPerPeer);
  tunnels_.erase(std::remove_if(tunnels_.begin(), tunnels_.end(),
    [](const Live& live) {
      return !live.tunnel || live.tunnel->finished();
    }), tunnels_.end());
}

bool HttpProxyHub::tryReserve(const Reservation& slot)
{
  std::lock_guard<std::mutex> lock(mu_);
  if (total_ >= kMaxTunnels)
    return false;
  if (bump(perNode_, slot.node, 1, kMaxPerNode) < 0)
    return false;
  if (!slot.peer.empty() && bump(perPeer_, slot.peer, 1, kMaxPerPeer) < 0) {
    bump(perNode_, slot.node, -1, kMaxPerNode);
    return false;
  }
  ++total_;
  return true;
}

bool HttpProxyHub::acceptHttpProxy(const http::HttpParserFrame& frame,
  const std::shared_ptr<network::NetSocket>& client)
{
  if (!enabled_ || !registrations_) {
    http::writeHttpStatusAndClose(client, 404, "Not Found");
    return true;
  }
  if (headerHasUpgrade(frame)) {
    http::writeHttpStatusAndClose(client, 501, "Not Implemented");
    return true;
  }

  const std::string_view path = frame.getPath();
  if (!http::isHttpProxyPath(path, mount_)) {
    http::writeHttpStatusAndClose(client, 404, "Not Found");
    return true;
  }
  std::string_view rest = path.substr(mount_.size());
  if (!rest.empty() && rest.front() == '/')
    rest.remove_prefix(1);
  if (rest.empty()) {
    http::writeHttpStatusAndClose(client, 404, "Not Found");
    return true;
  }
  if (badSegment(rest)) {
    http::writeHttpStatusAndClose(client, 400, "Bad Request");
    return true;
  }

  const bool mayHaveBody = frame.requestMethod == http::HttpMethod::Post
    || frame.requestMethod == http::HttpMethod::Put
    || frame.requestMethod == http::HttpMethod::Patch;
  if (mayHaveBody && !frame.hasContentLength()) {
    http::writeHttpStatusAndClose(client, 411, "Length Required");
    return true;
  }

  std::shared_ptr<NodeRef> best;
  size_t bestLen = 0;
  for (const std::shared_ptr<NodeRef>& node : registrations_->listAllNodes()) {
    if (!node || !node->hasHttpProxy())
      continue;
    if (node->getState() == NodeRef::State::Dead)
      continue;
    const std::string key = nodeKey(node->id());
    if (key.empty())
      continue;
    const bool match = rest == key ||
      (rest.size() > key.size() && rest.substr(0, key.size()) == key && rest[key.size()] == '/');
    if (match && key.size() > bestLen) {
      best = node;
      bestLen = key.size();
    }
  }
  if (!best) {
    http::writeHttpStatusAndClose(client, 404, "Not Found");
    return true;
  }

  std::string remainder = rest.size() == bestLen ? std::string("/") : std::string(rest.substr(bestLen));
  const std::vector<std::string> prefixes = best->httpProxyPrefixes();
  if (!prefixAllows(prefixes, remainder)) {
    http::writeHttpStatusAndClose(client, 404, "Not Found");
    return true;
  }

  http::HttpProxyTarget target;
  target.address = best->httpProxyDial();
  target.host = target.address.address;
  target.port = target.address.port();
  target.publicPrefix = publicPrefix(best->id());
  target.path = remainder;
  target.query = std::string(frame.getQuery());
  target.contentLength = mayHaveBody ? frame.contentLength() : -1;
  if (!target.address.valid() || target.host.empty() || target.port <= 0) {
    http::writeHttpStatusAndClose(client, 502, "Bad Gateway");
    return true;
  }

  Reservation slot;
  slot.node = best->id();
  slot.peer = best->httpProxyPeer();
  if (!tryReserve(slot)) {
    http::writeHttpStatusAndClose(client, 503, "Service Unavailable");
    return true;
  }

  http::HttpServer* server = server_;
  PollSet* poll = server ? server->getPollSet() : nullptr;
  std::weak_ptr<HttpProxyHub> weak = weak_from_this();
  auto tunnel = http::HttpProxyTunnel::open(poll, client, frame, std::move(target),
    [weak, slot]() {
      if (auto self = weak.lock())
        self->release(slot);
    });
  if (!tunnel) {
    release(slot);
    return true;
  }
  if (tunnel->finished())
    return true;
  {
    std::lock_guard<std::mutex> lock(mu_);
    tunnels_.push_back(Live{std::move(tunnel), slot.peer});
  }
  return true;
}

void HttpProxyHub::closePeer(const std::string& peerKey)
{
  if (peerKey.empty())
    return;
  std::vector<std::shared_ptr<http::HttpProxyTunnel>> closing;
  {
    std::lock_guard<std::mutex> lock(mu_);
    for (const Live& live : tunnels_) {
      if (live.peer == peerKey && live.tunnel)
        closing.push_back(live.tunnel);
    }
  }
  for (const auto& tunnel : closing)
    tunnel->requestClose();
}

void HttpProxyHub::closeAll()
{
  std::vector<std::shared_ptr<http::HttpProxyTunnel>> closing;
  {
    std::lock_guard<std::mutex> lock(mu_);
    closing.reserve(tunnels_.size());
    for (const Live& live : tunnels_) {
      if (live.tunnel)
        closing.push_back(live.tunnel);
    }
    tunnels_.clear();
  }
  for (const auto& tunnel : closing)
    tunnel->requestClose();
}

} // namespace master
} // namespace miniros
