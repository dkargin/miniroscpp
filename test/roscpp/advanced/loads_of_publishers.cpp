/*
 * Copyright (c) 2009, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Josh Faust */

#include <gtest/gtest.h>
#include <miniros/ros.h>
#include <miniros/init.h>

#include <miniros/transport/callback_queue.h>

#include "test_roscpp/TestArray.hxx"

uint32_t g_pub_count = 0;

void callback(const test_roscpp::TestArrayConstPtr&)
{
}

TEST(LoadsOfPublishers, waitForAll)
{
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 1000, callback);

  while (sub.getNumPublishers() < g_pub_count)
  {
    miniros::getGlobalCallbackQueue()->callAvailable(miniros::WallDuration(0.1));
  }

  miniros::WallDuration(10).sleep();
  miniros::spinOnce();
  ASSERT_EQ(sub.getNumPublishers(), g_pub_count);
}

struct Helper
{
  void callback(const miniros::MessageEvent<test_roscpp::TestArray>& msg_event)
  {
    alive[msg_event.getPublisherName()] = true;
  }

  std::map<std::string, bool> alive;
};

TEST(LoadsOfPublishers, receiveFromAll)
{
  miniros::NodeHandle nh;
  Helper helper;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 1000, &Helper::callback, &helper);

  while (helper.alive.size() < g_pub_count)
  {
    miniros::getGlobalCallbackQueue()->callAvailable(miniros::WallDuration(0.1));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "loads_of_publishers");

  if (argc < 2)
  {
    MINIROS_ERROR("Not enough arguments (usage: loads_of_publishers num_publishers)");
    return 1;
  }

  g_pub_count = atoi(argv[1]);

  miniros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
