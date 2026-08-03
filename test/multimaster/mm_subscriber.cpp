//
// Multimaster test helper: waits for an expected std_msgs/String on /mm/chatter.
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

void onMsg(const std_msgs::String::ConstPtr& msg)
{
  if (msg && msg->data == g_expected)
    g_got = true;
}

} // namespace

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "mm_subscriber");
  miniros::NodeHandle nh;
  miniros::NodeHandle pnh("~");

  g_expected = pnh.param<std::string>("payload", "hello-from-c");
  const double timeoutSec = pnh.param("timeout", 20.0);

  miniros::Subscriber sub = nh.subscribe<std_msgs::String>("/mm/chatter", 10, onMsg);

  miniros::Rate rate(10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
  while (miniros::ok() && !g_got && std::chrono::steady_clock::now() < deadline) {
    miniros::spinOnce();
    rate.sleep();
  }

  return g_got ? EXIT_SUCCESS : EXIT_FAILURE;
}
