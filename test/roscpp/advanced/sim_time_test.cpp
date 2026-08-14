/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Tony Pratkanis */

/*
 * Subscribe to a topic multiple times
 */

#include <string>
#include <gtest/gtest.h>
#include <time.h>
#include <stdlib.h>
#include <atomic>
#include <thread>
#include "miniros/ros.h"
#include "miniros/master_link.h"
#include <rosgraph_msgs/Clock.hxx>

#include "../../require_master.h"

int g_argc;
char** g_argv;

class RosClockTest : public testing::Test
{
public:
  void setTime(miniros::Time t)
  {
    rosgraph_msgs::Clock message;
    message.clock = t;
    pub_.publish(message);
  }

protected:
  RosClockTest()
  {
    pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
    while (pub_.getNumSubscribers() == 0)
    {
      miniros::WallDuration(0.01).sleep();
    }
  }

  miniros::NodeHandle nh_;
  miniros::Publisher pub_;

};

TEST_F(RosClockTest, SimClockTest)
{
  //Get the start time.
  miniros::Time start = miniros::Time::now();

  //The start time should be zero before a message is published.
  ASSERT_TRUE(start.isZero());

  //Publish a rostime of 42.
  setTime(miniros::Time(42, 0));

  //Wait half a second to get the message.
  miniros::WallDuration(0.5).sleep();

  //Make sure that it is really set
  ASSERT_EQ(42.0, miniros::Time::now().toSec());
}

void sleepThread(std::atomic<bool>* done)
{
  bool ok = miniros::Duration(1.0).sleep();
  if (!ok)
  {
    MINIROS_ERROR("!OK");
  }
  done->store(true, std::memory_order_release);
}

TEST(Clock, sleepFromZero)
{
  miniros::Time::setNow(miniros::Time());
  std::atomic<bool> done{false};
  std::thread t(sleepThread, &done);

  miniros::WallDuration(1.0).sleep();
  miniros::WallTime start = miniros::WallTime::now();
  miniros::Time::setNow(miniros::Time(miniros::WallTime::now().sec, miniros::WallTime::now().nsec));
  while (!done.load(std::memory_order_acquire))
  {
    miniros::WallDuration(0.001).sleep();
    miniros::WallTime now = miniros::WallTime::now();
    miniros::Time::setNow(miniros::Time(now.sec, now.nsec));
  }
  t.join();
  miniros::WallTime end = miniros::WallTime::now();
  EXPECT_GE(end - start, miniros::WallDuration(1.0));
}

TEST(Clock, isTimeValid)
{
  miniros::Time::setNow(miniros::Time());
  ASSERT_FALSE(miniros::Time::isValid());
  miniros::Time::setNow(miniros::TIME_MIN);
  ASSERT_TRUE(miniros::Time::isValid());
}

void waitThread(std::atomic<bool>* done)
{
  miniros::Time::waitForValid();
  done->store(true, std::memory_order_release);
}

TEST(Clock, waitForValid)
{
  miniros::Time::setNow(miniros::Time());

  // Test timeout
  miniros::WallTime start = miniros::WallTime::now();
  ASSERT_FALSE(miniros::Time::waitForValid(miniros::WallDuration(1.0)));
  miniros::WallTime end = miniros::WallTime::now();
  ASSERT_GT(end - start, miniros::WallDuration(1.0));

  std::atomic<bool> done{false};
  std::thread t(waitThread, &done);

  miniros::WallDuration(1.0).sleep();
  EXPECT_FALSE(done.load(std::memory_order_acquire));
  miniros::Time::setNow(miniros::TIME_MIN);
  while (!done.load(std::memory_order_acquire))
  {
    miniros::WallDuration(0.01).sleep();
  }
  t.join();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "sim_time_test");
  miniros::test::requireMasterOrExit("advanced-sim_time_test");

  // Former sim_time_test.xml: /use_sim_time=true. Set before start() (first NodeHandle).
  miniros::MasterLinkPtr master = miniros::getMasterLink();
  if (!master || !master->set("/use_sim_time", true)) {
    return EXIT_FAILURE;
  }

  miniros::NodeHandle nh;
  g_argc = argc;
  g_argv = argv;
  const int rc = RUN_ALL_TESTS();
  master->set("/use_sim_time", false);
  return rc;
}
