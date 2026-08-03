//
// Multimaster test helper: subscribes to /mm/request, replies on /mm/response.
//

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>

#include "miniros/ros.h"
#include "std_msgs/String.hxx"

namespace {

std::atomic_bool g_done{false};
miniros::Publisher g_pub;

void onRequest(const std_msgs::String::ConstPtr& msg)
{
  if (!msg || g_done)
    return;
  std_msgs::String resp;
  resp.data = std::string("ack:") + msg->data;
  g_pub.publish(resp);
  g_done = true;
}

} // namespace

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "mm_responder");
  miniros::NodeHandle nh;
  miniros::NodeHandle pnh("~");

  const double timeoutSec = pnh.param("timeout", 30.0);

  g_pub = nh.advertise<std_msgs::String>("/mm/response", 1, true);
  miniros::Subscriber sub = nh.subscribe<std_msgs::String>("/mm/request", 10, onRequest);

  miniros::Rate rate(20);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
  // Stay alive after the first reply so multimaster tests can observe /mm/response
  // (exiting immediately unregisters the topic and races waitPublishedTopic).
  while (miniros::ok() && std::chrono::steady_clock::now() < deadline) {
    miniros::spinOnce();
    rate.sleep();
  }

  return g_done ? EXIT_SUCCESS : EXIT_FAILURE;
}
