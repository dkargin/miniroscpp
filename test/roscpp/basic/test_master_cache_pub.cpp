//
// Long-lived latching publisher for master-cache restore tests.
// Advertises, publishes one latched message, notifies readiness, then spins
// so a restarted master can re-query getPublications and a later subscriber
// can still connect.
//

#include <chrono>
#include <cstdlib>
#include <string>

#include "miniros/common.h"
#include "miniros/ros.h"
#include "std_msgs/String.hxx"

namespace {

constexpr const char* kTopic = "cache_test_chatter";
constexpr const char* kPayload = "hello-from-cached-publisher";

double spinTimeoutSec()
{
  if (const char* env = std::getenv("MASTER_CACHE_PUB_TIMEOUT")) {
    char* end = nullptr;
    const double v = std::strtod(env, &end);
    if (end != env && v > 0.0)
      return v;
  }
  return 60.0;
}

} // namespace

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "cache_test_pub",
                miniros::init_options::NoRosout | miniros::init_options::NoSimTime);

  miniros::NodeHandle nh;
  miniros::Publisher pub = nh.advertise<std_msgs::String>(kTopic, 1, true /* latch */);

  std_msgs::String msg;
  msg.data = kPayload;
  pub.publish(msg);

  miniros::NodeNotifyInfo info;
  if (miniros::Error err = miniros::notifyNodeStarted(info); !err) {
    MINIROS_ERROR("cache_test_pub: notifyNodeStarted failed: %s", err.toString());
    return EXIT_FAILURE;
  }

  miniros::Rate rate(20);
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(spinTimeoutSec());
  while (miniros::ok() && std::chrono::steady_clock::now() < deadline) {
    miniros::spinOnce();
    rate.sleep();
  }

  return EXIT_SUCCESS;
}
