//
// WebSocket server example.
// This example demonstrates how to set up a WebSocket endpoint and handle WebSocket connections.
//

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <mutex>
#include <atomic>

#include "miniros/http/http_server.h"
#include "miniros/http/http_endpoint.h"
#include "miniros/http/http_filters.h"
#include "miniros/http/endpoints/websocket.h"
#include "miniros/http/websocket.h"
#include "miniros/http/endpoints/filesystem.h"
#include "miniros/io/poll_manager.h"
#include "miniros/io/poll_set.h"
#include "miniros/network/socket.h"
#include "miniros/console.h"
#include "miniros/errors.h"
#include <filesystem>

using namespace miniros;
using namespace miniros::http;

namespace {

// Global connection storage
std::vector<std::shared_ptr<WebSocket>> g_connections;
std::mutex g_connections_mutex;

void onWebSocketCreated(std::shared_ptr<WebSocket> ws) {
  std::cout << "WebSocket connection established" << std::endl;
  
  PollManagerPtr pm = PollManager::instance();
  if (pm) {
    // Register WebSocket with PollSet and set message handler
    Error err = ws->registerWithPollSet(&pm->getPollSet(), 
      [ws](const std::string& message) {
        std::cout << "Received WebSocket message: " << message << std::endl;
        
        // Echo back with a response
        std::string response = "Echo: " + message;
        ws->sendMessage(response);
      });
    
    if (err == Error::Ok) {
      std::lock_guard<std::mutex> lock(g_connections_mutex);
      g_connections.push_back(ws);
    } else {
      std::cerr << "Failed to register WebSocket with PollSet: " << err.toString() << std::endl;
    }
  }
}

} // anonymous namespace

int main(int argc, char** argv)
{
  miniros::console::set_logger_level("miniros.http", console::Level::Debug);
  miniros::console::set_logger_level("miniros.poll_set", console::Level::Debug);

  console::initializeSafe();
  
  std::cout << "WebSocket Server Example" << std::endl;
  std::cout << "========================" << std::endl;
  
  // Initialize PollManager
  PollManagerPtr pm = PollManager::instance();
  pm->start();
  
  // Create HTTP server
  HttpServer server(&pm->getPollSet());
  
  // Create WebSocket handler
  auto wsHandler = std::make_shared<WebSocketHandler>(onWebSocketCreated);
  
  // Register WebSocket endpoint first (more specific route)
  Error err = server.registerEndpoint(
    std::make_unique<SimpleFilter>(HttpMethod::Get, "/ws"),
    wsHandler,
    nullptr  // No callback queue - handle synchronously
  );
  
  if (err != Error::Ok) {
    std::cerr << "Failed to register WebSocket endpoint: " << err.toString() << std::endl;
    return 1;
  }
  
  // Get the examples directory path (where this source file is located)
  std::filesystem::path examplesDir = std::filesystem::path(__FILE__).parent_path();
  
  // Create filesystem endpoint to serve static files from examples directory
  auto fsEndpoint = std::make_shared<FilesystemEndpoint>("/", examplesDir.string());
  
  // Register filesystem endpoint to serve static files (including websocket_test.html)
  // This is registered after WebSocket endpoint so exact matches take precedence
  err = server.registerEndpoint(
    std::make_unique<SimpleFilter>(HttpMethod::Get, "/", SimpleFilter::CheckType::Prefix),
    fsEndpoint,
    nullptr
  );
  
  if (err != Error::Ok) {
    std::cerr << "Failed to register filesystem endpoint: " << err.toString() << std::endl;
    return 1;
  }
  
  // Start server on port 8080
  err = server.start(8080);
  if (err != Error::Ok) {
    std::cerr << "Failed to start HTTP server: " << err.toString() << std::endl;
    return 1;
  }
  
  int port = server.getPort();
  std::cout << "HTTP server started on port " << port << std::endl;
  std::cout << "WebSocket endpoint available at: ws://localhost:" << port << "/ws" << std::endl;
  std::cout << "Test page available at: http://localhost:" << port << "/websocket_test.html" << std::endl;
  std::cout << "Press Ctrl+C to stop..." << std::endl;
  
  // Keep running
  std::atomic<bool> running(true);
  while (running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  
  // Cleanup
  server.stop();
  pm->shutdown();
  
  {
    std::lock_guard<std::mutex> lock(g_connections_mutex);
    for (auto& ws : g_connections) {
      if (ws) {
        ws->close();
      }
    }
    g_connections.clear();
  }
  
  return 0;
}
