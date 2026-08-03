//
// Multimaster test helper: publishes to /mm/request, waits for /mm/response.
// Republishes periodically so late peer masters / responders can still complete
// the exchange after this node has already started.
//

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <string>

#include "miniros/ros.h"
#include "std_msgs/String.hxx"

namespace {

std::atomic_bool g_got{false};
std::string g_expected;

void onResponse(const std_msgs::String::ConstPtr& msg)
{
  if (msg && msg->data == g_expected)
    g_got = true;
}

} // namespace

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "mm_requester");
  miniros::NodeHandle nh;
  miniros::NodeHandle pnh("~");

  const std::string nonce = pnh.param<std::string>("nonce", "42");
  const double timeoutSec = pnh.param("timeout", 15.0);
  g_expected = std::string("ack:") + nonce;

  miniros::Subscriber sub = nh.subscribe<std_msgs::String>("/mm/response", 10, onResponse);
  miniros::Publisher pub = nh.advertise<std_msgs::String>("/mm/request", 1, true);

  miniros::Rate rate(10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
  auto nextPublish = std::chrono::steady_clock::now();

  while (miniros::ok() && !g_got && std::chrono::steady_clock::now() < deadline) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= nextPublish) {
      std_msgs::String req;
      req.data = nonce;
      pub.publish(req);
      nextPublish = now + std::chrono::seconds(1);
    }
    miniros::spinOnce();
    rate.sleep();
  }

  return g_got ? EXIT_SUCCESS : EXIT_FAILURE;
}
