//
// Non-blocking HTTP/1.1 reverse proxy. One callback moves at most a fixed
// buffer in each direction and returns, so the poll thread can service the
// upstream node (often on the same PollSet).
//

#include "miniros/http/http_proxy.h"

#include <cstdio>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#define MINIROS_PACKAGE_NAME "http"

#include "internal/threading.h"
#include "miniros/http/http_tools.h"
#include "miniros/io/io.h"
#include "miniros/io/poll_set.h"
#include "miniros/network/socket.h"
#include "miniros/network/url.h"
#include "rosconsole/local_log.h"

namespace miniros {
namespace http {

bool isHttpProxyPath(std::string_view path, std::string_view prefix)
{
  if (prefix.empty() || prefix.front() != '/')
    return false;
  if (prefix.back() == '/')
    prefix.remove_suffix(1);
  if (prefix.empty() || path.empty())
    return false;
  if (path == prefix)
    return true;
  return path.size() > prefix.size()
    && path.compare(0, prefix.size(), prefix) == 0
    && path[prefix.size()] == '/';
}

namespace {

bool containsIgnoreCase(std::string_view hay, std::string_view needle)
{
  if (needle.empty() || needle.size() > hay.size())
    return false;
  for (size_t i = 0; i + needle.size() <= hay.size(); ++i) {
    if (HttpParserFrame::nameEquals(hay.substr(i, needle.size()), needle))
      return true;
  }
  return false;
}

bool isHopByHop(std::string_view name)
{
  return HttpParserFrame::nameEquals(name, "connection")
    || HttpParserFrame::nameEquals(name, "keep-alive")
    || HttpParserFrame::nameEquals(name, "proxy-connection")
    || HttpParserFrame::nameEquals(name, "te")
    || HttpParserFrame::nameEquals(name, "trailer")
    || HttpParserFrame::nameEquals(name, "upgrade")
    || HttpParserFrame::nameEquals(name, "transfer-encoding")
    || HttpParserFrame::nameEquals(name, "host")
    || HttpParserFrame::nameEquals(name, "content-length");
}

/// A byte-stream request must deliver Upgrade and Connection. Other hop-by-hop fields still drop.
bool dropRequestHeader(std::string_view name, bool byteStream)
{
  if (byteStream && (HttpParserFrame::nameEquals(name, "upgrade") || HttpParserFrame::nameEquals(name, "connection")))
    return false;
  return isHopByHop(name);
}

void appendHeader(std::string& out, std::string_view name, std::string_view value)
{
  out.append(name);
  out.append(": ");
  out.append(value);
  out.append("\r\n");
}

/// Rewrite an upstream Location into the public `/p/{node}` prefix.
std::string rewriteLocation(std::string_view value, const std::string& publicPrefix,
  const std::string& upstreamHost, int upstreamPort)
{
  if (value.empty())
    return {};
  if (!publicPrefix.empty() && (value == publicPrefix ||
      (value.size() > publicPrefix.size() && value.substr(0, publicPrefix.size()) == publicPrefix &&
        value[publicPrefix.size()] == '/')))
    return std::string(value);
  if (value.front() == '/')
    return publicPrefix + std::string(value);

  network::URL url;
  if (!url.fromString(std::string(value), false) || !HttpParserFrame::nameEquals(url.scheme, "http://"))
    return std::string(value);
  // fromString leaves the port at 0 when the URI omits it. HTTP's default is 80.
  const int port = url.port == 0 ? 80 : static_cast<int>(url.port);
  if (!HttpParserFrame::nameEquals(url.host, upstreamHost) || port != upstreamPort)
    return std::string(value);

  std::string rewritten = publicPrefix;
  rewritten += url.path.empty() ? "/" : url.path;
  if (!url.query.empty()) {
    rewritten.push_back('?');
    rewritten += url.query;
  }
  return rewritten;
}

struct ResponseHead {
  std::string bytes;
  /// -1: read until EOF. >= 0: that many body bytes remain, including bytes already past the head.
  int64_t bodyLeft = -1;
  /// Upstream switched protocols. The following bytes are a raw bidirectional stream.
  bool switching = false;
};

ResponseHead rewriteResponseHead(const HttpParserFrame& frame, const HttpProxyTarget& target)
{
  ResponseHead out;
  const bool switching = target.byteStream && frame.responseCode == 101;
  const std::string_view raw = frame.header();
  const auto lineEnd = raw.find("\r\n");
  std::string rebuilt;
  rebuilt.reserve(raw.size() + target.publicPrefix.size() + 32);
  if (lineEnd == std::string_view::npos)
    rebuilt.append(raw);
  else
    rebuilt.append(raw.data(), lineEnd);
  rebuilt.append("\r\n");

  bool sawConnection = false;
  bool chunked = false;
  for (const auto& field : frame.fields) {
    if (HttpParserFrame::nameEquals(field.name, "Location")) {
      appendHeader(rebuilt, "Location", rewriteLocation(field.value, target.publicPrefix, target.host, target.port));
    } else if (HttpParserFrame::nameEquals(field.name, "Connection")) {
      appendHeader(rebuilt, "Connection", switching ? field.value : "close");
      sawConnection = true;
    } else if (HttpParserFrame::nameEquals(field.name, "Transfer-Encoding")) {
      if (containsIgnoreCase(field.value, "chunked"))
        chunked = true;
      appendHeader(rebuilt, field.name, field.value);
    } else {
      appendHeader(rebuilt, field.name, field.value);
    }
  }
  if (!sawConnection)
    appendHeader(rebuilt, "Connection", switching ? "Upgrade" : "close");
  rebuilt.append("\r\n");
  out.bytes = std::move(rebuilt);
  out.switching = switching;
  if (switching || chunked || !frame.hasContentLength())
    out.bodyLeft = -1;
  else
    out.bodyLeft = frame.contentLength() < 0 ? 0 : frame.contentLength();
  return out;
}

std::string buildRequestHead(const HttpParserFrame& frame, const HttpProxyTarget& target, const std::string& clientIp)
{
  std::string head;
  head.reserve(512 + frame.fields.size() * 32);
  head += frame.requestMethod.toString();
  head.push_back(' ');
  head += target.path.empty() ? "/" : target.path;
  if (!target.query.empty()) {
    head.push_back('?');
    head += target.query;
  }
  head += " HTTP/1.1\r\nHost: ";
  network::URL hostUrl;
  hostUrl.host = target.host;
  hostUrl.port = static_cast<uint32_t>(target.port);
  head += hostUrl.str();
  head += "\r\n";

  for (const auto& field : frame.fields) {
    if (dropRequestHeader(field.name, target.byteStream))
      continue;
    head += field.name;
    head += ": ";
    head += field.value;
    head += "\r\n";
  }
  if (target.contentLength >= 0) {
    head += "Content-Length: ";
    head += std::to_string(target.contentLength);
    head += "\r\n";
  }
  if (!clientIp.empty()) {
    head += "X-Forwarded-For: ";
    head += clientIp;
    head += "\r\n";
  }
  head += "X-Forwarded-Proto: http\r\nX-Forwarded-Prefix: ";
  head += target.publicPrefix;
  head += "\r\n";
  if (!target.byteStream)
    head += "Connection: close\r\n";
  head += "\r\n";
  return head;
}

} // namespace

void writeHttpStatusAndClose(const std::shared_ptr<network::NetSocket>& socket, int code, const char* reason)
{
  if (!socket)
    return;
  char buf[192];
  const int n = std::snprintf(buf, sizeof(buf),
    "HTTP/1.1 %d %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
    code, reason ? reason : "Error");
  if (n > 0)
    (void)socket->send(buf, static_cast<size_t>(n), nullptr);
  socket->close();
}

struct HttpProxyTunnel::Impl {
  PollSet* poll GUARDED_BY(mu) = nullptr;
  std::shared_ptr<network::NetSocket> client;
  std::shared_ptr<network::NetSocket> upstream;
  HttpProxyTarget target;
  DoneFn onDone;

  bool connecting = true;
  /// Guards stop, finished, and the fd numbers requestClose() signals.
  mutable std::mutex mu;
  bool stop GUARDED_BY(mu) = false;
  bool finished GUARDED_BY(mu) = false;
  /// -1 once finish() drops that socket.
  int clientFd GUARDED_BY(mu) = -1;
  int upstreamFd GUARDED_BY(mu) = -1;
  bool responseHeadDone = false;

  /// Head and body stay apart so one write2() can gather them.
  struct Pending {
    std::string head;
    std::string body;
    size_t written = 0;

    size_t size() const { return head.size() + body.size(); }
    size_t unsent() const { return written < size() ? size() - written : 0; }
    bool pending() const { return unsent() > 0; }

    void reset()
    {
      head.clear();
      body.clear();
      written = 0;
    }
  };

  Pending up;
  int64_t requestLeft = 0;

  Pending down;
  HttpParserFrame responseFrame;
  int64_t responseLeft = -1;
  bool responseEof = false;

  char scratch[kBufCap];

  void armIdle()
  {
    PollSet* ps = pollSet();
    if (!ps)
      return;
    if (client && client->valid())
      ps->setTimerEvent(client->fd(), kIdleTimeoutMs);
    if (upstream && upstream->valid())
      ps->setTimerEvent(upstream->fd(), connecting ? kConnectTimeoutMs : kIdleTimeoutMs);
  }

  void refreshInterest()
  {
    PollSet* ps = pollSet();
    if (!ps)
      return;
    if (client && client->valid()) {
      int ev = 0;
      if (down.pending())
        ev |= PollSet::EventOut;
      if (requestLeft > 0 && up.unsent() < kBufCap)
        ev |= PollSet::EventIn;
      ps->setEvents(client->fd(), ev);
    }
    if (upstream && upstream->valid()) {
      int ev = 0;
      if (connecting || up.pending())
        ev |= PollSet::EventOut;
      const bool wantRead = !responseEof && (responseLeft != 0) && down.unsent() < kBufCap;
      if (!connecting && wantRead)
        ev |= PollSet::EventIn;
      ps->setEvents(upstream->fd(), ev);
    }
  }

  bool writePending(network::NetSocket& sock, Pending& pending)
  {
    if (!sock.valid() || !pending.pending())
      return false;
    const auto [n, err] = sock.write2(pending.head.data(), pending.head.size(),
      pending.body.data(), pending.body.size(), pending.written);
    if (n > 0)
      pending.written += n;
    if (!pending.pending())
      pending.reset();
    if (err == Error::EndOfFile || err == Error::SystemError)
      requestStop();
    return n > 0;
  }

  bool readRequestBody()
  {
    if (requestLeft <= 0 || !client || !client->valid())
      return false;
    if (up.unsent() >= kBufCap)
      return false;
    const size_t cap = static_cast<size_t>(std::min<int64_t>(requestLeft, static_cast<int64_t>(kBufCap)));
    const int n = read_socket(static_cast<socket_fd_t>(client->fd()), scratch, cap);
    if (n > 0) {
      up.body.append(scratch, n);
      requestLeft -= static_cast<int64_t>(n);
      return true;
    }
    if (n == 0 || !last_socket_error_is_would_block())
      requestStop();
    return false;
  }

  bool readResponse()
  {
    if (responseEof || responseLeft == 0 || !upstream || !upstream->valid())
      return false;
    if (!responseHeadDone) {
      if (responseFrame.data.size() >= kMaxHeader) {
        requestStop();
        return false;
      }
      const size_t cap = std::min(kBufCap, kMaxHeader - responseFrame.data.size());
      const int n = read_socket(static_cast<socket_fd_t>(upstream->fd()), scratch, cap);
      if (n > 0) {
        responseFrame.data.append(scratch, n);
        responseFrame.incrementalParse();
        if (responseFrame.state() == HttpParserFrame::ParseInvalid) {
          requestStop();
          return false;
        }
        if (responseFrame.hasHeader()) {
          ResponseHead rewritten = rewriteResponseHead(responseFrame, target);
          responseLeft = rewritten.bodyLeft;
          down.head = std::move(rewritten.bytes);
          const std::string_view extra = responseFrame.pastHeader();
          if (!extra.empty()) {
            if (responseLeft >= 0) {
              const size_t take = std::min(extra.size(), static_cast<size_t>(responseLeft));
              down.body.append(extra.data(), take);
              responseLeft -= static_cast<int64_t>(take);
            } else {
              down.body.append(extra.data(), extra.size());
            }
          }
          responseFrame.data.clear();
          responseFrame.resetParseState(false);
          responseHeadDone = true;
        }
        return true;
      }
      if (n == 0 || !last_socket_error_is_would_block())
        requestStop();
      return false;
    }

    if (down.unsent() >= kBufCap)
      return false;
    size_t cap = kBufCap;
    if (responseLeft >= 0)
      cap = static_cast<size_t>(std::min<int64_t>(responseLeft, static_cast<int64_t>(kBufCap)));
    if (cap == 0)
      return false;
    const int n = read_socket(static_cast<socket_fd_t>(upstream->fd()), scratch, cap);
    if (n > 0) {
      down.body.append(scratch, n);
      if (responseLeft >= 0)
        responseLeft -= static_cast<int64_t>(n);
      return true;
    }
    if (n == 0) {
      responseEof = true;
      if (responseLeft > 0)
        requestStop();
      return false;
    }
    if (!last_socket_error_is_would_block())
      requestStop();
    return false;
  }

  bool responseComplete() const
  {
    if (!responseHeadDone)
      return false;
    if (down.pending())
      return false;
    if (responseLeft == 0)
      return true;
    return responseEof;
  }

  bool requestComplete() const
  {
    return !up.pending() && requestLeft <= 0;
  }

  bool isFinished() const
  {
    std::lock_guard<std::mutex> lock(mu);
    return finished;
  }

  bool stopRequested() const
  {
    std::lock_guard<std::mutex> lock(mu);
    return stop;
  }

  void requestStop()
  {
    std::lock_guard<std::mutex> lock(mu);
    stop = true;
  }

  PollSet* pollSet() const
  {
    std::lock_guard<std::mutex> lock(mu);
    return poll;
  }

  void finish()
  {
    DoneFn done;
    int closingClient = -1;
    int closingUpstream = -1;
    PollSet* ps = nullptr;
    {
      std::lock_guard<std::mutex> lock(mu);
      if (finished)
        return;
      finished = true;
      closingClient = clientFd;
      closingUpstream = upstreamFd;
      clientFd = -1;
      upstreamFd = -1;
      ps = poll;
      done = std::move(onDone);
    }
    if (ps && closingClient >= 0)
      ps->delSocket(closingClient);
    if (ps && closingUpstream >= 0)
      ps->delSocket(closingUpstream);
    if (client)
      client->close();
    if (upstream)
      upstream->close();
    if (done)
      done();
  }

  void pump()
  {
    for (int i = 0; i < kMaxPumps; ++i) {
      bool progress = false;
      if (upstream)
        progress = writePending(*upstream, up) || progress;
      if (client)
        progress = writePending(*client, down) || progress;
      progress = readRequestBody() || progress;
      progress = readResponse() || progress;
      if (!progress || stopRequested())
        break;
    }
  }

  void onConnected()
  {
    connecting = false;
    if (upstream)
      upstream->setNoDelay(true);
    armIdle();
    pump();
  }

  int onEvent(int fd, int flags)
  {
    (void)fd;
    if (isFinished())
      return 0;
    if (stopRequested() || (flags & PollSet::EventTimer)) {
      if (connecting && client && client->valid())
        writeHttpStatusAndClose(client, 502, "Bad Gateway");
      finish();
      return 0;
    }
    if (flags & (PollSet::EventError)) {
      if (connecting && client && client->valid())
        writeHttpStatusAndClose(client, 502, "Bad Gateway");
      finish();
      return 0;
    }

    if (connecting) {
      if (!upstream) {
        finish();
        return 0;
      }
      const Error err = upstream->checkConnected();
      if (!err) {
        if (client && client->valid())
          writeHttpStatusAndClose(client, 502, "Bad Gateway");
        finish();
        return 0;
      }
      onConnected();
      if (stop || (requestComplete() && responseComplete()))
        finish();
      else
        refreshInterest();
      return 0;
    }

    pump();
    if (stop || (requestComplete() && responseComplete()))
      finish();
    else {
      armIdle();
      refreshInterest();
    }
    return 0;
  }
};

HttpProxyTunnel::HttpProxyTunnel(const std::shared_ptr<Impl>& impl)
  : impl_(impl)
{
}

HttpProxyTunnel::~HttpProxyTunnel() = default;

bool HttpProxyTunnel::finished() const
{
  return !impl_ || impl_->isFinished();
}

void HttpProxyTunnel::requestClose()
{
  if (!impl_)
    return;
  int fd = -1;
  PollSet* poll = nullptr;
  {
    std::lock_guard<std::mutex> lock(impl_->mu);
    impl_->stop = true;
    if (impl_->finished)
      return;
    poll = impl_->poll;
    fd = impl_->clientFd >= 0 ? impl_->clientFd : impl_->upstreamFd;
  }
  if (poll && fd >= 0)
    poll->signalFd(fd);
}

std::shared_ptr<HttpProxyTunnel> HttpProxyTunnel::open(PollSet* poll,
  std::shared_ptr<network::NetSocket> client,
  const HttpParserFrame& inbound,
  HttpProxyTarget target,
  DoneFn onDone)
{
  if (!poll || !client || !target.address.valid()) {
    writeHttpStatusAndClose(client, 502, "Bad Gateway");
    return nullptr;
  }

  auto impl = std::make_shared<Impl>();
  impl->client = std::move(client);
  impl->target = std::move(target);
  impl->onDone = std::move(onDone);
  impl->responseFrame.finishResponse();

  const std::string clientIp = impl->client->peerAddress().address;
  impl->up.head = buildRequestHead(inbound, impl->target, clientIp);

  const std::string_view prefetched = inbound.pastHeader();
  if (impl->target.contentLength < 0) {
    impl->requestLeft = 0;
  } else {
    const size_t take = std::min(prefetched.size(), static_cast<size_t>(impl->target.contentLength));
    impl->up.body.append(prefetched.data(), take);
    impl->requestLeft = impl->target.contentLength - static_cast<int64_t>(take);
  }

  impl->upstream = std::make_shared<network::NetSocket>();
  const Error conn = impl->upstream->tcpConnect(impl->target.address, true);
  if (!conn && conn != Error::WouldBlock) {
    LOCAL_DEBUG("HttpProxy connect to %s:%d failed: %s", impl->target.host.c_str(), impl->target.port, conn.toString());
    writeHttpStatusAndClose(impl->client, 502, "Bad Gateway");
    impl->upstream->close();
    return nullptr;
  }

  const int clientFd = impl->client->fd();
  const int upFd = impl->upstream->fd();
  {
    std::lock_guard<std::mutex> lock(impl->mu);
    impl->poll = poll;
    impl->clientFd = clientFd;
    impl->upstreamFd = upFd;
  }
  if (!poll->addSocket(clientFd, 0,
        [impl, clientFd](int flags) { return impl->onEvent(clientFd, flags); }, impl, {})) {
    writeHttpStatusAndClose(impl->client, 502, "Bad Gateway");
    impl->upstream->close();
    return nullptr;
  }
  if (!poll->addSocket(upFd, PollSet::EventOut,
        [impl, upFd](int flags) { return impl->onEvent(upFd, flags); }, impl, {})) {
    poll->delSocket(clientFd);
    writeHttpStatusAndClose(impl->client, 502, "Bad Gateway");
    impl->upstream->close();
    return nullptr;
  }

  if (conn)
    impl->onConnected();
  else
    impl->armIdle();
  impl->refreshInterest();
  return std::shared_ptr<HttpProxyTunnel>(new HttpProxyTunnel(impl));
}

} // namespace http
} // namespace miniros
