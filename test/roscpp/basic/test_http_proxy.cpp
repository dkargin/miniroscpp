//
// Reverse HTTP proxy: /p/{node}/... on the master, spliced to the node's HTTP server.
//

#include <chrono>
#include <map>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <poll.h>

#include "miniros/roscore/master.h"

#include "miniros/common.h"
#include "miniros/io/io.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/io/poll_manager.h"
#include "miniros/transport/rpc_manager.h"

using namespace miniros;

namespace {

class NodeHandler : public http::EndpointHandler {
public:
  std::string absLocation;

  Error handle(const network::ClientInfo&, std::shared_ptr<http::HttpRequest> request) override
  {
    const std::string& path = request->path();
    if (path == "/ui/go") {
      request->setResponseStatus(302, "Found");
      request->setResponseHeader("Location", "/ui/next");
      request->setResponseBody("", "text/plain");
      return Error::Ok;
    }
    if (path == "/ui/abs") {
      request->setResponseStatus(302, "Found");
      request->setResponseHeader("Location", absLocation);
      request->setResponseBody("", "text/plain");
      return Error::Ok;
    }
    if (path == "/ui/echo") {
      request->setResponseStatusOk();
      request->setResponseBody(request->requestBody(), "application/octet-stream");
      return Error::Ok;
    }
    if (path == "/onlygrab/x") {
      request->setResponseStatusOk();
      request->setResponseBody("grabber", "text/plain");
      return Error::Ok;
    }
    if (path.rfind("/ui/", 0) == 0) {
      request->setResponseStatusOk();
      std::string body = path;
      const std::string q = request->getParameter("x");
      if (!q.empty())
        body += " x=" + q;
      request->setResponseBody(body, "text/plain");
      return Error::Ok;
    }
    request->setResponseStatus(404, "Not Found");
    request->setResponseBody("upstream-miss", "text/plain");
    return Error::Ok;
  }
};

struct Exchange {
  int status = 0;
  std::map<std::string, std::string> headers;
  std::string body;
};

std::string lower(std::string s)
{
  for (char& c : s)
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  return s;
}

bool sendAll(int fd, const std::string& data)
{
  size_t off = 0;
  while (off < data.size()) {
    const ssize_t n = ::send(fd, data.data() + off, data.size() - off, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    off += static_cast<size_t>(n);
  }
  return true;
}

Exchange httpExchange(int port, const std::string& raw, double timeoutSec = 3.0)
{
  Exchange out;
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    return out;
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr);
  if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    ::close(fd);
    return out;
  }
  if (!sendAll(fd, raw)) {
    ::close(fd);
    return out;
  }

  std::string buf;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
  bool sawHead = false;
  size_t headEnd = 0;
  int contentLength = -1;
  while (std::chrono::steady_clock::now() < deadline) {
    if (sawHead && contentLength >= 0 && buf.size() >= headEnd + static_cast<size_t>(contentLength))
      break;
    pollfd pfd{};
    pfd.fd = fd;
    pfd.events = POLLIN;
    const int pr = ::poll(&pfd, 1, 50);
    if (pr < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    if (pr == 0)
      continue;
    char tmp[4096];
    const ssize_t n = ::recv(fd, tmp, sizeof(tmp), 0);
    if (n == 0)
      break;
    if (n < 0) {
      if (errno == EINTR)
        continue;
      break;
    }
    buf.append(tmp, static_cast<size_t>(n));
    if (!sawHead) {
      headEnd = buf.find("\r\n\r\n");
      if (headEnd != std::string::npos) {
        sawHead = true;
        headEnd += 4;
        const std::string head = buf.substr(0, headEnd);
        const auto lineEnd = head.find("\r\n");
        const std::string statusLine = head.substr(0, lineEnd);
        const auto sp1 = statusLine.find(' ');
        if (sp1 != std::string::npos)
          out.status = std::atoi(statusLine.c_str() + sp1 + 1);
        size_t line = lineEnd + 2;
        while (line + 2 < head.size()) {
          const auto next = head.find("\r\n", line);
          if (next == std::string::npos || next == line)
            break;
          const std::string row = head.substr(line, next - line);
          const auto colon = row.find(':');
          if (colon != std::string::npos) {
            std::string name = lower(row.substr(0, colon));
            std::string value = row.substr(colon + 1);
            while (!value.empty() && (value.front() == ' ' || value.front() == '\t'))
              value.erase(value.begin());
            out.headers[name] = value;
            if (name == "content-length")
              contentLength = std::atoi(value.c_str());
          }
          line = next + 2;
        }
      }
    }
  }
  ::close(fd);
  if (sawHead && headEnd <= buf.size())
    out.body = buf.substr(headEnd);
  return out;
}

std::string request(const std::string& method, const std::string& path, const std::string& extra = {},
  const std::string& body = {})
{
  std::string raw = method + " " + path + " HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n";
  raw += extra;
  if (!body.empty() || method == "POST" || method == "PUT" || method == "PATCH")
    raw += "Content-Length: " + std::to_string(body.size()) + "\r\n";
  raw += "\r\n";
  raw += body;
  return raw;
}

master::Master::RpcValue prefixList(std::initializer_list<const char*> items)
{
  master::Master::RpcValue value = master::Master::RpcValue::Array(items.size());
  int i = 0;
  for (const char* item : items)
    value[i++] = std::string(item);
  return value;
}

} // namespace

class HttpProxyTest : public ::testing::Test {
public:
  PollManager poll_;
  std::shared_ptr<RPCManager> rpc_;
  std::unique_ptr<master::Master> master_;
  std::unique_ptr<http::HttpServer> node_;
  std::shared_ptr<NodeHandler> handler_;
  int nodePort_ = 0;

  void SetUp() override
  {
    poll_.start();
    rpc_ = std::make_shared<RPCManager>();
    master_ = std::make_unique<master::Master>(rpc_);
    master_->setCacheEnabled(false);
    master_->setMultimasterEnabled(false);
    master_->setNodeCheckPeriod(0);
    master_->setHttpProxyEnabled(true);
    ASSERT_TRUE(master_->start(&poll_.getPollSet(), 0, "127.0.0.1"));

    handler_ = std::make_shared<NodeHandler>();
    node_ = std::make_unique<http::HttpServer>(&poll_.getPollSet());
    node_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/ui", http::SimpleFilter::CheckType::Prefix),
      handler_, {});
    node_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/ui/echo"),
      handler_, {});
    node_->registerEndpoint(std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/onlygrab", http::SimpleFilter::CheckType::Prefix),
      handler_, {});
    ASSERT_EQ(node_->start(0), Error::Ok);
    nodePort_ = node_->getPort();
    handler_->absLocation = "http://127.0.0.1:" + std::to_string(nodePort_) + "/ui/next";
  }

  void TearDown() override
  {
    if (master_)
      master_->stop();
    if (node_)
      node_->stop();
    poll_.shutdown();
  }

  std::string nodeApi() const
  {
    return "http://127.0.0.1:" + std::to_string(nodePort_) + "/";
  }

  void enable(const std::string& name, std::initializer_list<const char*> prefixes)
  {
    const auto res = master_->rpc2EnableHttpProxy(name, nodeApi(), prefixList(prefixes), {});
    ASSERT_EQ(static_cast<int>(res[0]), 1) << static_cast<std::string>(res[1]);
  }
};

TEST_F(HttpProxyTest, ForwardsGetPostAndRewritesLocation)
{
  enable("/camera", {"/ui/"});

  const int port = master_->getPort();
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/hello?x=1"));
    EXPECT_EQ(got.status, 200);
    EXPECT_EQ(got.body, "/ui/hello x=1");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/secret"));
    EXPECT_EQ(got.status, 404);
    EXPECT_NE(got.body, "upstream-miss");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/RPC2"));
    EXPECT_EQ(got.status, 404);
  }
  {
    const std::string payload(80 * 1024, 'Q');
    const Exchange got = httpExchange(port, request("POST", "/p/camera/ui/echo", {}, payload));
    EXPECT_EQ(got.status, 200);
    EXPECT_EQ(got.body, payload);
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/go"));
    EXPECT_EQ(got.status, 302);
    EXPECT_EQ(got.headers.at("location"), "/p/camera/ui/next");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/abs"));
    EXPECT_EQ(got.status, 302);
    EXPECT_EQ(got.headers.at("location"), "/p/camera/ui/next");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/hello", "Upgrade: websocket\r\n"));
    EXPECT_EQ(got.status, 501);
  }
  {
    const std::string raw =
      "POST /p/camera/ui/echo HTTP/1.1\r\nHost: 127.0.0.1\r\n\r\n";
    const Exchange got = httpExchange(port, raw);
    EXPECT_EQ(got.status, 411);
  }

  enable("/camera/grabber", {"/onlygrab/"});
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/grabber/onlygrab/x"));
    EXPECT_EQ(got.status, 200);
    EXPECT_EQ(got.body, "grabber");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/grabber/ui/hello"));
    EXPECT_EQ(got.status, 404);
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/hello"));
    EXPECT_EQ(got.status, 200);
    EXPECT_EQ(got.body, "/ui/hello");
  }

  const auto disabled = master_->rpc2DisableHttpProxy("/camera", nodeApi(), {});
  ASSERT_EQ(static_cast<int>(disabled[0]), 1);
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/hello"));
    EXPECT_EQ(got.status, 404);
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/grabber/onlygrab/x"));
    EXPECT_EQ(got.status, 200);
  }
}

TEST_F(HttpProxyTest, DisabledMasterRefusesRegistration)
{
  master_->setHttpProxyEnabled(false);
  const auto res = master_->rpc2EnableHttpProxy("/camera", nodeApi(), prefixList({"/ui/"}), {});
  EXPECT_EQ(static_cast<int>(res[0]), 0);
  const Exchange got = httpExchange(master_->getPort(), request("GET", "/p/camera/ui/hello"));
  EXPECT_EQ(got.status, 404);
}

TEST_F(HttpProxyTest, WholeOriginExceptRpc)
{
  enable("/camera", {"/"});
  const int port = master_->getPort();
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/ui/hello"));
    EXPECT_EQ(got.status, 200);
    EXPECT_EQ(got.body, "/ui/hello");
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/RPC2"));
    EXPECT_EQ(got.status, 404);
  }
  {
    const Exchange got = httpExchange(port, request("GET", "/p/camera/../ui/hello"));
    EXPECT_EQ(got.status, 400);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();
  miniros::ensureNetworkInitialized();
  return RUN_ALL_TESTS();
}
