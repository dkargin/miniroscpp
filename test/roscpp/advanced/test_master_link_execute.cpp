//
// MasterLink::execute tests against a fake XML-RPC peer (HttpServer + own PollSet).
//

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "miniros/callback_queue.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/io/poll_manager.h"
#include "miniros/master_link.h"
#include "miniros/transport/rpc_manager.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

using namespace miniros;
using RpcValue = XmlRpc::XmlRpcValue;

namespace {

std::string wrapMethodResponse(const RpcValue& result)
{
  // Mirrors generateResponseBody() in src/http/endpoints/xmlrpc.cpp
  return std::string("<?xml version=\"1.0\"?>\r\n"
                     "<methodResponse><params><param>\r\n\t") +
         result.toXml() +
         "\r\n</param></params></methodResponse>\r\n";
}

RpcValue makeMasterResult(int status, const std::string& msg, const RpcValue& payload)
{
  RpcValue v;
  v[0] = status;
  v[1] = msg;
  v[2] = payload;
  return v;
}

/// Minimal XML-RPC peer used as a stand-in for rosmaster.
class FakeXmlRpcMaster {
public:
  FakeXmlRpcMaster() = default;

  ~FakeXmlRpcMaster() { stop(); }

  Error start()
  {
    callback_done_ = false;
    callback_queue_ = std::make_shared<CallbackQueue>();
    callback_thread_ = std::thread([this]() {
      while (!callback_done_.load()) {
        callback_queue_->callAvailable(WallDuration(0.05));
      }
    });

    poll_manager_.start();
    server_ = std::make_unique<http::HttpServer>(&poll_manager_.getPollSet());
    if (Error err = server_->start(0); !err)
      return err;

    handler_ = std::make_shared<Handler>(this);
    server_->registerEndpoint(
      std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/RPC2"),
      handler_,
      callback_queue_);
    return Error::Ok;
  }

  void stop()
  {
    if (server_) {
      server_->stop();
      server_.reset();
    }
    poll_manager_.shutdown();

    callback_done_ = true;
    if (callback_queue_)
      callback_queue_->disable();
    if (callback_thread_.joinable())
      callback_thread_.join();
    callback_queue_.reset();
    handler_.reset();
  }

  int port() const { return server_ ? server_->getPort() : 0; }

  void setDelay(WallDuration delay) { delay_ = delay; }
  void setStatusCode(int code) { status_code_ = code; }
  void setPayload(const RpcValue& payload)
  {
    std::scoped_lock lock(mutex_);
    payload_ = payload;
  }

  int callCount() const { return call_count_.load(); }

  std::string lastMethod() const
  {
    std::scoped_lock lock(mutex_);
    return last_method_;
  }

private:
  class Handler : public http::EndpointHandler {
  public:
    explicit Handler(FakeXmlRpcMaster* owner) : owner_(owner) {}

    Error handle(const network::ClientInfo&, std::shared_ptr<http::HttpRequest> request) override
    {
      owner_->onRequest(request);
      return Error::Ok;
    }

    FakeXmlRpcMaster* owner_;
  };

  void onRequest(const std::shared_ptr<http::HttpRequest>& request)
  {
    {
      std::scoped_lock lock(mutex_);
      // Body is a methodCall; method name is enough for assertions.
      const std::string& body = request->requestBody();
      const std::string open = "<methodName>";
      const std::string close = "</methodName>";
      const size_t a = body.find(open);
      const size_t b = body.find(close);
      if (a != std::string::npos && b != std::string::npos && b > a)
        last_method_ = body.substr(a + open.size(), b - a - open.size());
      else
        last_method_.clear();
    }
    call_count_.fetch_add(1);

    const WallDuration delay = delay_;
    if (delay > WallDuration(0))
      delay.sleep();

    RpcValue payload;
    {
      std::scoped_lock lock(mutex_);
      payload = payload_;
    }
    RpcValue result = makeMasterResult(status_code_.load(), "", payload);
    request->setResponseStatusOk();
    request->setResponseBody(wrapMethodResponse(result), "text/xml");
  }

  PollManager poll_manager_;
  std::unique_ptr<http::HttpServer> server_;
  std::shared_ptr<Handler> handler_;

  std::shared_ptr<CallbackQueue> callback_queue_;
  std::thread callback_thread_;
  std::atomic<bool> callback_done_{false};

  std::atomic<int> call_count_{0};
  std::atomic<int> status_code_{1};
  WallDuration delay_{0};
  RpcValue payload_;

  mutable std::mutex mutex_;
  std::string last_method_;
};

class MasterLinkExecuteTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    ASSERT_EQ(fake_.start(), Error::Ok);
    ASSERT_GT(fake_.port(), 0);

    client_poll_.start();
    rpc_ = std::make_shared<RPCManager>();
    rpc_->setPollSet(&client_poll_.getPollSet());

    M_string remaps;
    remaps["__master"] = "http://127.0.0.1:" + std::to_string(fake_.port());
    ASSERT_EQ(link_.initLink(remaps, rpc_, /*local=*/false), Error::Ok);
  }

  void TearDown() override
  {
    link_.disconnect();
    if (rpc_)
      rpc_->shutdown();
    rpc_.reset();
    client_poll_.shutdown();
    fake_.stop();
  }

  FakeXmlRpcMaster fake_;
  PollManager client_poll_;
  RPCManagerPtr rpc_;
  MasterLink link_;
};

} // namespace

TEST_F(MasterLinkExecuteTest, ExecuteSucceeds)
{
  fake_.setPayload(RpcValue(true));

  RpcValue args, response, payload;
  args[0] = "test_node";
  args[1] = "/some_param";

  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(link_.execute("hasParam", args, response, payload, /*wait_for_master=*/false), Error::Ok);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_LT(elapsed, std::chrono::milliseconds(500));
  EXPECT_EQ(fake_.callCount(), 1);
  EXPECT_EQ(fake_.lastMethod(), "hasParam");
  EXPECT_TRUE(static_cast<bool>(payload));
}

TEST_F(MasterLinkExecuteTest, ExecuteSurvivesSlowResponse)
{
  // Regression: waitForState(Done, 0.3s) must not treat an in-flight request as finished.
  fake_.setDelay(WallDuration(0.7));
  fake_.setPayload(RpcValue(42));

  RpcValue args, response, payload;
  args[0] = "test_node";

  const auto t0 = std::chrono::steady_clock::now();
  ASSERT_EQ(link_.execute("getPid", args, response, payload, /*wait_for_master=*/true), Error::Ok);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_GE(elapsed, std::chrono::milliseconds(600));
  EXPECT_EQ(fake_.callCount(), 1);
  EXPECT_EQ(static_cast<int>(payload), 42);
}

TEST_F(MasterLinkExecuteTest, ExecuteInvalidStatus)
{
  fake_.setStatusCode(0);
  fake_.setPayload(RpcValue(false));

  RpcValue args, response, payload;
  args[0] = "test_node";
  args[1] = "/missing";

  EXPECT_EQ(link_.execute("hasParam", args, response, payload, /*wait_for_master=*/false),
            Error::InvalidResponse);
  EXPECT_EQ(fake_.callCount(), 1);
}

TEST(MasterLinkExecuteNoPeer, ExecuteNoMaster)
{
  PollManager client_poll;
  client_poll.start();
  auto rpc = std::make_shared<RPCManager>();
  rpc->setPollSet(&client_poll.getPollSet());

  MasterLink link;
  M_string remaps;
  // Nothing listens here.
  remaps["__master"] = "http://127.0.0.1:1";
  ASSERT_EQ(link.initLink(remaps, rpc, /*local=*/false), Error::Ok);
  link.setRetryTimeout(WallDuration(0.5));

  RpcValue args, response, payload;
  args[0] = "test_node";

  EXPECT_EQ(link.execute("getPid", args, response, payload, /*wait_for_master=*/false), Error::NoMaster);

  link.disconnect();
  rpc->shutdown();
  client_poll.shutdown();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
