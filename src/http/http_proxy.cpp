//
// Non-blocking HTTP/1.1 reverse proxy. One callback moves at most a fixed
// buffer in each direction and returns, so the poll thread can service the
// upstream node (often on the same PollSet).
//

#include "miniros/http/http_proxy.h"

#include <atomic>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#define MINIROS_PACKAGE_NAME "http"

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

std::string lowerCopy(std::string_view in)
{
  std::string out(in);
  for (char& c : out)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return out;
}

bool startsWithIgnoreCase(std::string_view text, std::string_view prefix)
{
  if (text.size() < prefix.size())
    return false;
  for (size_t i = 0; i < prefix.size(); ++i) {
    const auto a = static_cast<unsigned char>(text[i]);
    const auto b = static_cast<unsigned char>(prefix[i]);
    if (std::tolower(a) != std::tolower(b))
      return false;
  }
  return true;
}

bool equalsIgnoreCase(std::string_view a, std::string_view b)
{
  return a.size() == b.size() && startsWithIgnoreCase(a, b);
}

bool headerNameIs(std::string_view line, std::string_view name)
{
  if (line.size() < name.size() + 1)
    return false;
  if (!startsWithIgnoreCase(line, name))
    return false;
  return line[name.size()] == ':';
}

std::string_view headerValue(std::string_view line)
{
  const auto colon = line.find(':');
  if (colon == std::string_view::npos)
    return {};
  size_t i = colon + 1;
  while (i < line.size() && (line[i] == ' ' || line[i] == '\t'))
    ++i;
  return line.substr(i);
}

bool isHopByHop(std::string_view name)
{
  const std::string lower = lowerCopy(name);
  return lower == "connection" || lower == "keep-alive" || lower == "proxy-connection"
    || lower == "te" || lower == "trailer" || lower == "upgrade" || lower == "transfer-encoding"
    || lower == "host" || lower == "content-length";
}

/// A byte-stream request must deliver Upgrade and Connection. Other hop-by-hop fields still drop.
bool dropRequestHeader(std::string_view name, bool byteStream)
{
  if (byteStream) {
    const std::string lower = lowerCopy(name);
    if (lower == "upgrade" || lower == "connection")
      return false;
  }
  return isHopByHop(name);
}

int responseStatusCode(std::string_view line)
{
  const auto sp = line.find(' ');
  if (sp == std::string_view::npos)
    return 0;
  int code = 0;
  for (size_t i = sp + 1; i < line.size() && line[i] >= '0' && line[i] <= '9'; ++i)
    code = code * 10 + (line[i] - '0');
  return code;
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
  if (!url.fromString(std::string(value), false) || !equalsIgnoreCase(url.scheme, "http://"))
    return std::string(value);
  // fromString leaves the port at 0 when the URI omits it. HTTP's default is 80.
  const int port = url.port == 0 ? 80 : static_cast<int>(url.port);
  if (!equalsIgnoreCase(url.host, upstreamHost) || port != upstreamPort)
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

ResponseHead rewriteResponseHead(std::string_view raw, const HttpProxyTarget& target)
{
  ResponseHead out;
  std::string rebuilt;
  rebuilt.reserve(raw.size() + target.publicPrefix.size() + 32);
  bool first = true;
  bool sawConnection = false;
  bool chunked = false;
  bool sawLength = false;
  bool switching = false;
  int64_t length = -1;

  size_t lineStart = 0;
  while (lineStart < raw.size()) {
    size_t lineEnd = raw.find("\r\n", lineStart);
    if (lineEnd == std::string_view::npos)
      lineEnd = raw.size();
    const std::string_view line = raw.substr(lineStart, lineEnd - lineStart);
    if (first) {
      switching = target.byteStream && responseStatusCode(line) == 101;
      rebuilt.append(line);
      rebuilt.append("\r\n");
      first = false;
    } else if (!line.empty()) {
      if (headerNameIs(line, "Location")) {
        const std::string rewritten = rewriteLocation(headerValue(line), target.publicPrefix, target.host, target.port);
        rebuilt.append("Location: ");
        rebuilt.append(rewritten);
        rebuilt.append("\r\n");
      } else if (headerNameIs(line, "Connection")) {
        if (switching)
          rebuilt.append(line.begin(), line.end()).append("\r\n");
        else
          rebuilt.append("Connection: close\r\n");
        sawConnection = true;
      } else if (headerNameIs(line, "Content-Length")) {
        length = std::atoll(std::string(headerValue(line)).c_str());
        sawLength = true;
        rebuilt.append(line);
        rebuilt.append("\r\n");
      } else if (headerNameIs(line, "Transfer-Encoding")) {
        if (lowerCopy(headerValue(line)).find("chunked") != std::string::npos)
          chunked = true;
        rebuilt.append(line);
        rebuilt.append("\r\n");
      } else {
        rebuilt.append(line);
        rebuilt.append("\r\n");
      }
    }
    if (lineEnd >= raw.size())
      break;
    lineStart = lineEnd + 2;
  }
  if (!sawConnection)
    rebuilt.append(switching ? "Connection: Upgrade\r\n" : "Connection: close\r\n");
  rebuilt.append("\r\n");
  out.bytes = std::move(rebuilt);
  out.switching = switching;
  if (switching || chunked || !sawLength)
    out.bodyLeft = -1;
  else
    out.bodyLeft = length < 0 ? 0 : length;
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
  PollSet* poll = nullptr;
  std::shared_ptr<network::NetSocket> client;
  std::shared_ptr<network::NetSocket> upstream;
  HttpProxyTarget target;
  DoneFn onDone;

  bool connecting = true;
  std::atomic<bool> stop{false};
  bool finished = false;
  bool responseHeadDone = false;

  std::string upPending;
  size_t upOff = 0;
  int64_t requestLeft = 0;

  std::string downPending;
  size_t downOff = 0;
  std::string respAccum;
  int64_t responseLeft = -1;
  bool responseEof = false;

  char scratch[kBufCap];

  void armIdle()
  {
    if (!poll)
      return;
    if (client && client->valid())
      poll->setTimerEvent(client->fd(), kIdleTimeoutMs);
    if (upstream && upstream->valid())
      poll->setTimerEvent(upstream->fd(), connecting ? kConnectTimeoutMs : kIdleTimeoutMs);
  }

  void refreshInterest()
  {
    if (!poll)
      return;
    if (client && client->valid()) {
      int ev = 0;
      if (downPending.size() > downOff)
        ev |= PollSet::EventOut;
      if (requestLeft > 0 && (upPending.size() - upOff) < kBufCap)
        ev |= PollSet::EventIn;
      poll->setEvents(client->fd(), ev);
    }
    if (upstream && upstream->valid()) {
      int ev = 0;
      if (connecting || upPending.size() > upOff)
        ev |= PollSet::EventOut;
      const bool wantRead = !responseEof && (responseLeft != 0) && (downPending.size() - downOff) < kBufCap;
      if (!connecting && wantRead)
        ev |= PollSet::EventIn;
      poll->setEvents(upstream->fd(), ev);
    }
  }

  bool writePending(const std::shared_ptr<network::NetSocket>& sock, std::string& buf, size_t& off)
  {
    if (!sock || !sock->valid() || off >= buf.size())
      return false;
    const auto [n, err] = sock->send(buf.data() + off, buf.size() - off, nullptr);
    if (n > 0)
      off += n;
    if (off >= buf.size()) {
      buf.clear();
      off = 0;
    }
    if (err == Error::EndOfFile || err == Error::SystemError)
      stop = true;
    return n > 0;
  }

  bool readRequestBody()
  {
    if (requestLeft <= 0 || !client || !client->valid())
      return false;
    if (upPending.size() - upOff >= kBufCap)
      return false;
    const size_t cap = static_cast<size_t>(std::min<int64_t>(requestLeft, static_cast<int64_t>(kBufCap)));
    const int n = read_socket(static_cast<socket_fd_t>(client->fd()), scratch, cap);
    if (n > 0) {
      upPending.append(scratch, n);
      requestLeft -= static_cast<int64_t>(n);
      return true;
    }
    if (n == 0 || !last_socket_error_is_would_block())
      stop = true;
    return false;
  }

  bool readResponse()
  {
    if (responseEof || responseLeft == 0 || !upstream || !upstream->valid())
      return false;
    if (!responseHeadDone) {
      if (respAccum.size() >= kMaxHeader) {
        stop = true;
        return false;
      }
      const size_t cap = std::min(kBufCap, kMaxHeader - respAccum.size());
      const int n = read_socket(static_cast<socket_fd_t>(upstream->fd()), scratch, cap);
      if (n > 0) {
        respAccum.append(scratch, n);
        const auto mark = respAccum.find("\r\n\r\n");
        if (mark != std::string::npos) {
          const size_t headEnd = mark + 4;
          ResponseHead rewritten = rewriteResponseHead(std::string_view(respAccum.data(), mark), target);
          responseLeft = rewritten.bodyLeft;
          downPending.append(rewritten.bytes);
          if (headEnd < respAccum.size()) {
            const size_t extra = respAccum.size() - headEnd;
            if (responseLeft >= 0) {
              const size_t take = std::min(extra, static_cast<size_t>(responseLeft));
              downPending.append(respAccum.data() + headEnd, take);
              responseLeft -= static_cast<int64_t>(take);
            } else {
              downPending.append(respAccum.data() + headEnd, extra);
            }
          }
          respAccum.clear();
          responseHeadDone = true;
        }
        return true;
      }
      if (n == 0 || !last_socket_error_is_would_block())
        stop = true;
      return false;
    }

    if (downPending.size() - downOff >= kBufCap)
      return false;
    size_t cap = kBufCap;
    if (responseLeft >= 0)
      cap = static_cast<size_t>(std::min<int64_t>(responseLeft, static_cast<int64_t>(kBufCap)));
    if (cap == 0)
      return false;
    const int n = read_socket(static_cast<socket_fd_t>(upstream->fd()), scratch, cap);
    if (n > 0) {
      downPending.append(scratch, n);
      if (responseLeft >= 0)
        responseLeft -= static_cast<int64_t>(n);
      return true;
    }
    if (n == 0) {
      responseEof = true;
      if (responseLeft > 0)
        stop = true;
      return false;
    }
    if (!last_socket_error_is_would_block())
      stop = true;
    return false;
  }

  bool responseComplete() const
  {
    if (!responseHeadDone)
      return false;
    if (downPending.size() > downOff)
      return false;
    if (responseLeft == 0)
      return true;
    return responseEof;
  }

  bool requestComplete() const
  {
    return upPending.size() == upOff && requestLeft <= 0;
  }

  void finish()
  {
    if (finished)
      return;
    finished = true;
    DoneFn done = std::move(onDone);
    const int clientFd = client && client->valid() ? client->fd() : -1;
    const int upFd = upstream && upstream->valid() ? upstream->fd() : -1;
    if (poll && clientFd >= 0)
      poll->delSocket(clientFd);
    if (poll && upFd >= 0)
      poll->delSocket(upFd);
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
      progress = writePending(upstream, upPending, upOff) || progress;
      progress = writePending(client, downPending, downOff) || progress;
      progress = readRequestBody() || progress;
      progress = readResponse() || progress;
      if (!progress || stop)
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
    if (finished)
      return 0;
    if (stop || (flags & PollSet::EventTimer)) {
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
  return !impl_ || impl_->finished;
}

void HttpProxyTunnel::requestClose()
{
  if (!impl_ || impl_->finished)
    return;
  impl_->stop = true;
  if (impl_->poll && impl_->client && impl_->client->valid())
    impl_->poll->signalFd(impl_->client->fd());
  else if (impl_->poll && impl_->upstream && impl_->upstream->valid())
    impl_->poll->signalFd(impl_->upstream->fd());
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
  impl->poll = poll;
  impl->client = std::move(client);
  impl->target = std::move(target);
  impl->onDone = std::move(onDone);

  const std::string clientIp = impl->client->peerAddress().address;
  impl->upPending = buildRequestHead(inbound, impl->target, clientIp);

  const std::string_view header = inbound.header();
  std::string_view prefetched;
  if (header.size() < inbound.data.size())
    prefetched = std::string_view(inbound.data.data() + header.size(), inbound.data.size() - header.size());
  if (impl->target.contentLength < 0) {
    impl->requestLeft = 0;
  } else {
    const size_t take = std::min(prefetched.size(), static_cast<size_t>(impl->target.contentLength));
    impl->upPending.append(prefetched.data(), take);
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
