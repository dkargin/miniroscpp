/*
 * Copyright (c) 2010, Willow Garage, Inc.
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

/*
 * Author: Josh Faust
 */

#include <gtest/gtest.h>

#include <miniros/ros.h>
#include <miniros/connection_manager.h>

#include "test_roscpp/TestArray.h"
#include "roscpp/Empty.h"

int32_t g_count = 0;
void callback(const test_roscpp::TestArrayConstPtr&)
{
  ++g_count;
}

TEST(SubscribeRetryTCP, localDisconnect)
{
  g_count = 0;
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 0, callback);
  // wait for initial messages to arrive
  miniros::WallTime start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_GT(g_count, 0);

  miniros::ConnectionManager::instance()->clear(miniros::Connection::TransportDisconnect);

  // spin to make sure all previous messages have arrived
  miniros::spinOnce();

  g_count = 0;
  // wait for reconnect/new messages to arrive
  start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_GT(g_count, 0);
}

TEST(SubscribeRetryTCP, localDisconnectNonTransportDisconnect)
{
  g_count = 0;
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 0, callback);
  // wait for initial messages to arrive
  miniros::WallTime start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_GT(g_count, 0);

  miniros::ConnectionManager::instance()->clear(miniros::Connection::HeaderError);

  // spin for a bit to make sure all previous messages have arrived
  start = miniros::WallTime::now();
  while (miniros::WallTime::now() - start < miniros::WallDuration(1.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  g_count = 0;
  // make sure we did not reconnect
  start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_EQ(g_count, 0);
}

TEST(SubscribeRetryTCP, remoteDisconnect)
{
  g_count = 0;
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 0, callback);
  // wait for initial messages to arrive
  miniros::WallTime start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_GT(g_count, 0);

  roscpp::Empty::Request req;
  roscpp::Empty::Response resp;
  miniros::service::call("/publish_constantly/debug/close_all_connections", req, resp);

  // spin to make sure all previous messages have arrived
  miniros::spinOnce();

  g_count = 0;
  // wait for reconnect/new messages to arrive
  start = miniros::WallTime::now();
  while (g_count == 0 && miniros::WallTime::now() - start < miniros::WallDuration(5.0))
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_GT(g_count, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "subscribe_retry_tcp");
  miniros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
