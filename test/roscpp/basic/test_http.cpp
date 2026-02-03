//
// Created by dkargin on 11/22/25.
//

#include <thread>
#include <chrono>

#include <gtest/gtest.h>

#include "miniros/http/http_client.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"

#include "miniros/io/poll_manager.h"

using namespace miniros;

class HttpServerTest : public ::testing::Test {
public:
  std::unique_ptr<http::HttpServer> server_;

  PollManager poll_manager_;

  /// Get port of HTTP server.
  int getServerPort() const
  {
    return server_ ? server_->getPort() : 0;
  }

  void SetUp() override
  {
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

TEST_F(HttpServerTest, ConnectNoServer)
{
  http::HttpClient client(&poll_manager_.getPollSet());

  client.connect("127.0.0.1", 19999);
  Error conErr = client.waitConnected(WallDuration(5));
  EXPECT_EQ(conErr, Error::NotConnected);
}

TEST_F(HttpServerTest, SimpleGet)
{
  // Test sends two GET requests using the same request object.

  // Register the /data endpoint
  auto dataHandler = std::make_shared<DataEndpointHandler>();
  Error regErr = server_->registerEndpoint(
    std::make_unique<http::SimpleFilter>(http::HttpMethod::Get, "/data"),
    dataHandler);
  ASSERT_EQ(regErr, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  MINIROS_INFO("Started connection");
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(20)), Error::Ok);

  MINIROS_INFO("Test connected to local server");

  // Create and configure GET request to /data
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Get, "/data");

  // Enqueue the request
  Error enqErr = client.enqueueRequest(request);
  ASSERT_EQ(enqErr, Error::Ok);

  // Wait for response (poll until status becomes HasResponse)
  EXPECT_EQ(request->waitForResponse(WallDuration(5.0)), Error::Ok);

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
  client.enqueueRequest(request);

  EXPECT_EQ(request->waitForResponse(WallDuration(5.0)), Error::Ok);
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
    jsonHandler);
  ASSERT_EQ(regErr, Error::Ok);

  // Create HTTP client
  http::HttpClient client(&poll_manager_.getPollSet());

  int port = getServerPort();
  Error conErr = client.connect("127.0.0.1", port);
  MINIROS_INFO("Started connection for POST test");
  ASSERT_TRUE(conErr == Error::Ok || conErr == Error::Timeout);
  ASSERT_EQ(client.waitConnected(WallDuration(20)), Error::Ok);

  MINIROS_INFO("Connected to local server for POST test");

  // Create and configure POST request to /api/data
  auto request = std::make_shared<http::HttpRequest>(http::HttpMethod::Post, "/api/data");
  
  // Set JSON body
  std::string jsonRequest = R"({"name": "test_user", "value": 42, "active": true})";
  request->setRequestBody(jsonRequest);
  
  // Set Content-Type header
  request->setHeader("Content-Type", "application/json");

  // Enqueue the request
  Error enqErr = client.enqueueRequest(request);
  ASSERT_EQ(enqErr, Error::Ok);

  // Wait for response (poll until status becomes HasResponse)
  EXPECT_EQ(request->waitForResponse(WallDuration(5.0)), Error::Ok);

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

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();
  miniros::console::set_logger_level("destructor", console::Level::Debug);
  return RUN_ALL_TESTS();
}
