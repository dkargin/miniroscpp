//
// Created by dkargin on 3/23/25.
//
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 * It is adaptation of official ROS tutorial at http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
 */
#include "miniros/ros.h"

// Messages for miniros have .hxx file extension to make them distinguishable from original ROS messages.
#include "std_msgs/String.hxx"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  MINIROS_INFO("I heard: [%s]", msg->data.c_str());
}

void timerCallback(const miniros::TimerEvent& e)
{
  static int counter = 0;
  MINIROS_INFO("INFO - timer callback %d", counter);
  MINIROS_DEBUG("DEBUG - timer callback %d", counter);
  MINIROS_WARN("WARN - timer callback %d", counter);
  MINIROS_ERROR("ERROR - timer callback %d", counter);

  MINIROS_INFO_NAMED("timer", "INFO - named timer callback %d", counter);
  MINIROS_DEBUG_NAMED("timer", "DEBUG - named timer callback %d", counter);
  MINIROS_WARN_NAMED("timer", "WARN - named timer callback %d", counter);
  MINIROS_ERROR_NAMED("timer", "ERROR - named timer callback %d", counter);
  counter++;
}

int main(int argc, char **argv)
{
  /**
   * The miniros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of miniros::init() before using any other
   * part of the ROS system.
   */
  miniros::init(argc, argv, "log_demo");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  miniros::NodeHandle n;

  miniros::console::set_logger_level("timer", miniros::console::Level::Debug);

  auto timer = n.createTimer(miniros::Duration(1), timerCallback, false, true);
  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  miniros::spin();

  return 0;
}