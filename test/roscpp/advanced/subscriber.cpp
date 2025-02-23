// Test subscriber
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <miniros/publisher.h>
#include <miniros/init.h>
#include <miniros/node_handle.h>

#include <std_msgs/Int8MultiArray.hxx>

#include <stdio.h>

void handleMsg(const std_msgs::Int8MultiArray& msg)
{
  static uint8_t previous = 0;
  uint8_t current = msg.data[0];
  uint8_t diff = current - previous;
  if (diff != 1)
  {
    MINIROS_WARN("Missing %u message(s)", diff - 1);
  }
  MINIROS_INFO("Got message %u", current);
  previous = msg.data[0];
}

int main(int argc, char** argv)
{
  miniros::init(argc, argv, "subscriber");
  miniros::NodeHandle n;
  miniros::Subscriber sub = n.subscribe("data", 1, &handleMsg, miniros::TransportHints().udp());
  miniros::spin();
  return 0;
}
