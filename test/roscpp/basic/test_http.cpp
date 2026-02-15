//
// Created by dkargin on 11/22/25.
//

#include <thread>
#include <chrono>
#include <atomic>
#include <set>
#include <mutex>

#include <gtest/gtest.h>

#include "miniros/http/http_client.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"

#include "miniros/io/poll_manager.h"
#include "miniros/callback_queue.h"

#include "gtest_printers.h"

using namespace miniros;

class HttpServerTest : public ::testing::Test {
public:
  std::unique_ptr<http::HttpServer> server_;

  PollManager poll_manager_;

  std::shared_ptr<CallbackQueue> callback_queue_;
  std::thread callback_thread_;
  std::atomic<bool> callback_thread_done_{false};


  /// Get port of HTTP server.
  int getServerPort() const
  {
    return server_ ? server_->getPort() : 0;
  }

  // Helper function to process callback queue in a separate thread
  static void processCallbackQueue(CallbackQueue* queue, std::atomic<bool>* done)
  {
    while (!done->load())
    {
      queue->callAvailable(WallDuration(50));
    }
  }

  void SetUp() override
  {
    // Create a callback queue and start a thread to process it
    callback_queue_ = std::make_shared<CallbackQueue>();
    callback_thread_ = std::thread(processCallbackQueue, callback_queue_.get(), &callback_thread_done_);

    poll_manager_.start();
    PollSet* ps = &poll_manager_.getPollSet();
    server_.reset(new http::HttpServer(ps));
    Error err = server_->start(0);
    ASSERT_EQ(err, Error::Ok);
  }

  void TearDown() override
  {
    if (server_) {
      server_->stop();
      server_.reset();
    }
    poll_manager_.shutdown();

    // Stop callback thread
    callback_thread_done_ = true;
    callback_queue_.reset();
    callback_thread_.join();
  }
};

// Simple endpoint handler that returns JSON data
class DataEndpointHandler : public http::EndpointHandler {
public:
  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override
  {
    // Return example JSON data
    std::string body = std::string(R"({"name": "test_data", "counter": )") + std::to_string(counter)
      + R"(, "items": ["item1", "item2", "item3"]})";
    request->setResponseStatusOk();
    request->setResponseBody(body, "application/json");
    counter++;
    return Error::Ok;
  }
  int counter = 0;
};


/// Global timeouts for tests. Can be changed to larger values for debug purposes.
constexpr double connectionTimeoutSec = 50;
constexpr double responseTimeoutSec = 50;

TEST(BasicSanity, WaitIsReal)
{
  http::HttpRequest emptyRequest;

  auto start = SteadyTime::now();
  WallDuration waitDuration{0.5};
  emptyRequest.waitForResponse(waitDuration);
  WallDuration elapsed = SteadyTime::now() - start;
  /// Expecting it to actually wait.
  ASSERT_GE(elapsed, waitDuration*0.5)
      << "It seems condition_variable::wait_for does not properly wait." << std::endl
      << " It can be caused by missing proper implementation for std threads " << std::endl;
}

TEST_F(HttpServerTest, ConnectNoServer)
{
  http::HttpClient client(&poll_manager_.getPollSet());

  client.connect("127.0.0.1", 19999);
  Error conErr = client.waitConnected(WallDuration(connectionTimeoutSec));
  EXPECT_EQ(conErr, Error::NotConnected);
}

TEST_F(HttpServerTest, SimpleGet)
{
  // Test sends two GET requests using the same request object.

  // Register the /data endpoint
  auto dataHandler = std::make_shared<DataEndpointHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/data"),
    dataHandler, {});
  ASSERT_EQ(regErr, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  MINIROS_INFO("Started connection");
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

  MINIROS_INFO("Test connected to local server");

  // Create and configure GET request to /data
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/data");

  // Enqueue the request
  Error enqErr = client.enqueueRequest(request);
  ASSERT_EQ(enqErr, Error::Ok);

  // Wait for response (poll until status becomes HasResponse)
  EXPECT_EQ(request->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);

  // Verify we actually got a response
  ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
  {
    const auto& responseHeader = request->responseHeader();
    EXPECT_EQ(responseHeader.statusCode, 200);
    EXPECT_EQ(responseHeader.contentType, "application/json");
    // Verify response body contains JSON
    const std::string& responseBody = request->responseBody();
    ASSERT_FALSE(responseBody.empty());
    ASSERT_NE(responseBody.find("counter"), std::string::npos);
    ASSERT_NE(responseBody.find("0"), std::string::npos);
    ASSERT_NE(responseBody.find("items"), std::string::npos);
  }

  // Send second request.
  Error err2 = client.enqueueRequest(request);
  ASSERT_EQ(err2, Error::Ok);

  EXPECT_EQ(request->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);
  // Verify we actually got a response
  ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
  {
    const auto& responseHeader = request->responseHeader();
    EXPECT_EQ(responseHeader.statusCode, 200);
    EXPECT_EQ(responseHeader.contentType, "application/json");
    // Verify response body contains JSON
    const std::string& responseBody = request->responseBody();
    ASSERT_FALSE(responseBody.empty());
    ASSERT_NE(responseBody.find("counter"), std::string::npos);
    ASSERT_NE(responseBody.find("1"), std::string::npos);
    ASSERT_NE(responseBody.find("items"), std::string::npos);
  }
}

// POST endpoint handler that processes JSON request and returns JSON response
class JsonPostEndpointHandler : public http::EndpointHandler {
public:
  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override
  {

    // Read JSON from request body
    std::string_view requestBody = request->requestBody();
    
    // Simple JSON processing: echo back the request with a processed field
    // In a real scenario, you would parse the JSON here
    // For this test, we'll create a simple response that includes the request data
    std::string body = R"({"status": "success", "received": )" + std::string(requestBody) +
           R"(, "processed": true, "message": "Request processed successfully"})";

    request->setResponseStatus(200, "OK");
    request->setResponseBody(body, "application/json");
    return Error::Ok;
  }
};

TEST_F(HttpServerTest, PostWithJson)
{
  // Register the /api/data POST endpoint
  auto jsonHandler = std::make_shared<JsonPostEndpointHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Post, "/api/data"),
    jsonHandler, {});
  ASSERT_EQ(regErr, Error::Ok);

  // Create and configure POST request to /api/data
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Post, "/api/data");

  {
    // Create HTTP client
    http::HttpClient client(&poll_manager_.getPollSet());

    int port = getServerPort();
    Error conErr = client.connect("127.0.0.1", port);
    MINIROS_INFO("Started connection for POST test");
    ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
    ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

    MINIROS_INFO("Connected to local server for POST test");


    // Set JSON body
    std::string jsonRequest = R"({"name": "test_user", "value": 42, "active": true})";
    request->setRequestBody(jsonRequest);

    // Set Content-Type header
    request->setHeader("Content-Type", "application/json");

    // Enqueue the request
    Error enqErr = client.enqueueRequest(request);
    ASSERT_EQ(enqErr, Error::Ok);

    // Wait for response (poll until status becomes HasResponse)
    EXPECT_EQ(request->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);

    // Verify we actually got a response
    ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
    {
      const auto& responseHeader = request->responseHeader();
      EXPECT_EQ(responseHeader.statusCode, 200);
      EXPECT_EQ(responseHeader.status, "OK");
      EXPECT_EQ(responseHeader.contentType, "application/json");

      // Verify response body contains JSON
      const std::string& responseBody = request->responseBody();
      ASSERT_FALSE(responseBody.empty());

      // Verify response contains expected JSON fields
      ASSERT_NE(responseBody.find("status"), std::string::npos);
      ASSERT_NE(responseBody.find("success"), std::string::npos);
      ASSERT_NE(responseBody.find("processed"), std::string::npos);
      ASSERT_NE(responseBody.find("true"), std::string::npos);

      // Verify response contains the original request data
      ASSERT_NE(responseBody.find("test_user"), std::string::npos);
      ASSERT_NE(responseBody.find("42"), std::string::npos);
    }
  }
  // Expecting that we are holding the last reference.
  // Client should be already destroyed here. There could be some dangling callbacks in PollSet, which can keep reference.
  ASSERT_EQ(request.use_count(), 1);
}

// Endpoint handler that tracks which thread it runs in
class ThreadTrackingHandler : public http::EndpointHandler {
public:
  ThreadTrackingHandler()
    : call_count(0)
  {}

  Error handle(const network::ClientInfo& clientInfo, std::shared_ptr<http::HttpRequest> request) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    call_count++;
    thread_ids_.insert(std::this_thread::get_id());
    
    std::string body = R"({"thread_id": ")" + std::to_string(std::hash<std::thread::id>{}(std::this_thread::get_id())) +
                       R"(", "call_count": )" + std::to_string(call_count) + "}";
    request->setResponseStatusOk();
    request->setResponseBody(body, "application/json");
    return Error::Ok;
  }

  size_t getCallCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return call_count;
  }

  size_t getThreadCount() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return thread_ids_.size();
  }

  std::set<std::thread::id> getThreadIds() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return thread_ids_;
  }

private:
  mutable std::mutex mutex_;
  size_t call_count;
  std::set<std::thread::id> thread_ids_;
};

TEST_F(HttpServerTest, GetWithCallbackQueue)
{
  // Test sends two GET requests using the same request object, but processed via CallbackQueue.

  // Register the /data endpoint with callback queue
  auto dataHandler = std::make_shared<DataEndpointHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/data"),
    dataHandler, callback_queue_);
  ASSERT_EQ(regErr, Error::Ok);

  // Create and configure GET request to /data
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/data");

  {
    // Create HTTP client
    http::HttpClient client(&poll_manager_.getPollSet());

    int port = getServerPort();
    Error conErr = client.connect("127.0.0.1", port);
    MINIROS_INFO("Started connection");
    ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
    ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

    MINIROS_INFO("Test connected to local server");

    // Enqueue the request
    Error enqErr = client.enqueueRequest(request);
    ASSERT_EQ(enqErr, Error::Ok);

    // Wait for response (poll until status becomes HasResponse)
    auto ret = request->waitForResponse(WallDuration(responseTimeoutSec));
    LOCAL_INFO("waitForResponse returned %s", ret.toString());
    EXPECT_EQ(ret, Error::Ok);

    // Verify we actually got a response
    ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
    {
      const auto& responseHeader = request->responseHeader();
      EXPECT_EQ(responseHeader.statusCode, 200);
      EXPECT_EQ(responseHeader.contentType, "application/json");
      // Verify response body contains JSON
      const std::string& responseBody = request->responseBody();
      ASSERT_FALSE(responseBody.empty());
      ASSERT_NE(responseBody.find("counter"), std::string::npos);
      ASSERT_NE(responseBody.find("0"), std::string::npos);
      ASSERT_NE(responseBody.find("items"), std::string::npos);
    }

    // Send second request.
    Error err2 = client.enqueueRequest(request);
    ASSERT_EQ(err2, Error::Ok);

    EXPECT_EQ(request->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);
    // Verify we actually got a response
    ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
    {
      const auto& responseHeader = request->responseHeader();
      EXPECT_EQ(responseHeader.statusCode, 200);
      EXPECT_EQ(responseHeader.contentType, "application/json");
      // Verify response body contains JSON
      const std::string& responseBody = request->responseBody();
      ASSERT_FALSE(responseBody.empty());
      ASSERT_NE(responseBody.find("counter"), std::string::npos);
      ASSERT_NE(responseBody.find("1"), std::string::npos);
      ASSERT_NE(responseBody.find("items"), std::string::npos);
    }
  }
  // Expecting that we are holding the last reference.
  ASSERT_EQ(request.use_count(), 1);
}

TEST_F(HttpServerTest, MultipleRequestsWithCallbackQueue)
{
  // Register endpoint with callback queue
  auto handler = std::make_shared<ThreadTrackingHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/async-multi"),
    handler, callback_queue_);
  ASSERT_EQ(regErr, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

  // Send multiple requests
  const size_t num_requests = 5;
  std::vector<std::shared_ptr<http::HttpRequest>> requests;
  
  for (size_t i = 0; i < num_requests; ++i)
  {
    auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/async-multi");
    Error enqErr = client.enqueueRequest(request);
    ASSERT_EQ(enqErr, Error::Ok);
    requests.push_back(request);
  }

  // Wait for all responses
  for (auto& request : requests)
  {
    EXPECT_EQ(request->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);
    ASSERT_EQ(request->state(), http::HttpRequest::State::Done);
    
    const auto& responseHeader = request->responseHeader();
    EXPECT_EQ(responseHeader.statusCode, 200);
  }

  // Verify all requests were processed
  EXPECT_EQ(handler->getCallCount(), num_requests);
}

TEST_F(HttpServerTest, CallbackQueueThreadIsolation)
{
  // Get the callback thread ID
  std::thread::id callback_thread_id = callback_thread_.get_id();

  // Register endpoint with callback queue
  auto handler = std::make_shared<ThreadTrackingHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/thread-check"),
    handler, callback_queue_);
  ASSERT_EQ(regErr, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

  // Send request
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/thread-check");
  Error enqErr = client.enqueueRequest(request);
  ASSERT_EQ(enqErr, Error::Ok);

  // Wait for response
  auto ret = request->waitForResponse(WallDuration(responseTimeoutSec));
  LOCAL_INFO("waitForResponse returned %s", ret.toString());
  EXPECT_EQ(ret, Error::Ok);
  ASSERT_EQ(request->state(), http::HttpRequest::State::Done);

  // Verify handler was called in callback thread (not in PollSet thread)
  // The handler should have recorded a thread ID different from the main test thread
  std::set<std::thread::id> thread_ids = handler->getThreadIds();
  EXPECT_EQ(thread_ids.size(), 1U);
  
  // The thread ID should be the callback thread (or at least not the main thread)
  std::thread::id main_thread_id = std::this_thread::get_id();
  EXPECT_NE(*thread_ids.begin(), main_thread_id);
}

TEST_F(HttpServerTest, MixedSyncAndAsyncEndpoints)
{
  // Register async endpoint with callback queue
  auto asyncHandler = std::make_shared<ThreadTrackingHandler>();
  Error regErr1 = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/async"),
    asyncHandler, callback_queue_);
  ASSERT_EQ(regErr1, Error::Ok);

  // Register sync endpoint without callback queue
  auto syncHandler = std::make_shared<DataEndpointHandler>();
  Error regErr2 = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/sync"),
    syncHandler, {});
  ASSERT_EQ(regErr2, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(connectionTimeoutSec)), Error::Ok);

  // Send async request
  auto asyncRequest = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/async");
  Error enqErr1 = client.enqueueRequest(asyncRequest);
  ASSERT_EQ(enqErr1, Error::Ok);

  // Send sync request
  auto syncRequest = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/sync");
  Error enqErr2 = client.enqueueRequest(syncRequest);
  ASSERT_EQ(enqErr2, Error::Ok);

  // Wait for both responses
  EXPECT_EQ(asyncRequest->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);
  EXPECT_EQ(syncRequest->waitForResponse(WallDuration(responseTimeoutSec)), Error::Ok);

  // Verify both responses
  ASSERT_EQ(asyncRequest->state(), http::HttpRequest::State::Done);
  ASSERT_EQ(syncRequest->state(), http::HttpRequest::State::Done);
  
  EXPECT_EQ(asyncRequest->responseHeader().statusCode, 200);
  EXPECT_EQ(syncRequest->responseHeader().statusCode, 200);

  // Verify handlers were called
  EXPECT_EQ(asyncHandler->getCallCount(), 1U);
  EXPECT_EQ(syncHandler->counter, 1);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();

  miniros::console::set_logger_level("destructor", console::Level::Debug);
  miniros::console::set_logger_level("miniros.http", console::Level::Debug);
  return RUN_ALL_TESTS();
}
