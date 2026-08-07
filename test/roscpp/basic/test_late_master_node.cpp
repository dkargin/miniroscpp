//
// Helper for late-master startup test.
// Started before miniroscore: the first NodeHandle calls miniros::start(), which
// registerService()'s with wait_for_master=true and blocks until the master appears.
// After that, advertise + notify prove the node finished coming up.
//

#include <chrono>
#include <cstdlib>

#include "miniros/common.h"
#include "miniros/ros.h"
#include "std_msgs/String.hxx"

namespace {

double spinTimeoutSec()
{
  if (const char* env = std::getenv("LATE_MASTER_NODE_TIMEOUT")) {
    char* end = nullptr;
    const double v = std::strtod(env, &end);
    if (end != env && v > 0.0)
      return v;
  }
  return 30.0;
}

} // namespace

int main(int argc, char** argv)
{
  // Avoid command-line `_param:=...` remaps: initParam() uses setParam(wait_for_master=true)
  // and would also block here — fine functionally, but NH/start is the intentional wait point.
  miniros::init(argc, argv, "late_master_node",
                miniros::init_options::NoRosout | miniros::init_options::NoSimTime);

  // Blocks here until master is up (start → registerService wait_for_master=true).
  miniros::NodeHandle nh;

  miniros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1);
  (void)pub;

  miniros::NodeNotifyInfo info;
  if (miniros::Error err = miniros::notifyNodeStarted(info); !err) {
    MINIROS_ERROR("late_master_node: notifyNodeStarted failed: %s", err.toString());
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
