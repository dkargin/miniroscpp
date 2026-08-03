//
// Multimaster test helper: publishes std_msgs/String on /mm/chatter.
//

#include <chrono>
#include <cstdlib>
#include <string>

#include "miniros/ros.h"
#include "std_msgs/String.hxx"

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "mm_publisher");
  miniros::NodeHandle nh;
  miniros::NodeHandle pnh("~");

  std::string payload = pnh.param<std::string>("payload", "hello-from-c");
  const double timeoutSec = pnh.param("timeout", 30.0);

  miniros::Publisher pub = nh.advertise<std_msgs::String>("/mm/chatter", 10);

  miniros::Rate rate(10);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeoutSec);
  while (miniros::ok() && std::chrono::steady_clock::now() < deadline) {
    std_msgs::String msg;
    msg.data = payload;
    pub.publish(msg);
    miniros::spinOnce();
    rate.sleep();
  }
  return EXIT_SUCCESS;
}
