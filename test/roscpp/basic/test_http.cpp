//
// Created by dkargin on 11/22/25.
//

#include <thread>
#include <chrono>

#include "miniros/http/http_client.h"
#include "miniros/http/http_request.h"
#include "miniros/http/http_server.h"
#include "miniros/http/http_tools.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"

#include <gtest/gtest.h>

#include "miniros/transport/poll_manager.h"
#include "miniros/transport/poll_set.h"

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

  void startServer()
  {

  }
};

// Simple endpoint handler that returns JSON data
class DataEndpointHandler : public http::EndpointHandler {
public:
  Error handle(const http::HttpParserFrame& frame, const network::ClientInfo& clientInfo,
    http::HttpResponseHeader& responseHeader, std::string& body) override
  {
    // Return example JSON data
    body = R"({"name": "test_data", "value": 42, "items": ["item1", "item2", "item3"]})";
    responseHeader.statusCode = 200;
    responseHeader.status = "OK";
    responseHeader.contentType = "application/json";
    return Error::Ok;
  }
};

TEST_F(HttpServerTest, HttpServer)
{
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

  MINIROS_INFO("Connected to local server");

  // Create and configure GET request to /data
  auto request = std::make_shared<http::HttpRequest>();
  request->setMethod(http::HttpMethod::Get);
  request->setPath("/data");

  // Enqueue the request
  Error enqErr = client.enqueueRequest(request);
  ASSERT_EQ(enqErr, Error::Ok);

  // Wait for response (poll until status becomes HasResponse)
  const int maxWaitMs = 5000;
  const int pollIntervalMs = 10;
  int waited = 0;
  while (request->status() != http::HttpRequest::Status::HasResponse && waited < maxWaitMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(pollIntervalMs));
    waited += pollIntervalMs;
  }

  // Verify we got a response
  ASSERT_EQ(request->status(), http::HttpRequest::Status::HasResponse);
  const auto& responseHeader = request->responseHeader();
  EXPECT_EQ(responseHeader.statusCode, 200);
  EXPECT_EQ(responseHeader.contentType, "application/json");

  // Verify response body contains JSON
  const std::string& responseBody = request->responseBody();
  ASSERT_FALSE(responseBody.empty());
  ASSERT_NE(responseBody.find("test_data"), std::string::npos);
  ASSERT_NE(responseBody.find("42"), std::string::npos);
  ASSERT_NE(responseBody.find("items"), std::string::npos);
  MINIROS_INFO("Received response: %s", responseBody.c_str());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();

  return RUN_ALL_TESTS();
}
