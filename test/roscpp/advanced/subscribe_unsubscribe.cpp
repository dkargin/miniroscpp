/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

/* Author: Brian Gerkey */

/*
 * Subscribe to a topic, expecting to get a single message.
 */

#include <string>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"
#include <test_roscpp/TestArray.hxx>

#include "../basic/barrier.h"

#include <thread>

int g_argc;
char** g_argv;

class Subscriptions : public testing::Test
{
public:
  miniros::NodeHandle nh_;
  miniros::Subscriber sub_;

  void messageCallback(const test_roscpp::TestArrayConstPtr&)
  {
    MINIROS_INFO("in callback");

    if(!sub_)
    {
      MINIROS_INFO("but not subscribed!");
      FAIL();
    }
  }

  void autoUnsubscribeCallback(const test_roscpp::TestArrayConstPtr&)
  {
    MINIROS_INFO("in autounsub callback");
    sub_.shutdown();
  }

protected:
  Subscriptions() {}
};

TEST_F(Subscriptions, subUnsub)
{
  sub_.shutdown();

  for(int i=0;i<100;i++)
  {
    if(!sub_)
    {
      sub_ = nh_.subscribe("roscpp/pubsub_test", 0, &Subscriptions::messageCallback, (Subscriptions*)this);
      ASSERT_TRUE(sub_);
    }
    else
    {
      sub_.shutdown();
      ASSERT_FALSE(sub_);
    }

    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }
}

TEST_F(Subscriptions, unsubInCallback)
{
  sub_ = nh_.subscribe("roscpp/pubsub_test", 0, &Subscriptions::autoUnsubscribeCallback, (Subscriptions*)this);

  while (sub_ && miniros::ok())
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }
}

void spinThread(bool volatile* cont)
{
  while (*cont)
  {
    miniros::spinOnce();
    miniros::Duration(0.001).sleep();
  }
}

void unsubscribeAfterBarrierWait(Barrier* barrier, miniros::Subscriber& sub)
{
  barrier->wait();

  sub.shutdown();
}

TEST_F(Subscriptions, unsubInCallbackAndOtherThread)
{
  Barrier barrier(2);
  for (int i = 0; i < 100; ++i)
  {
    miniros::Subscriber sub;
    sub_ = nh_.subscribe<test_roscpp::TestArray>("roscpp/pubsub_test", 1,
      [&barrier, &sub](const test_roscpp::TestArray& array) {
      unsubscribeAfterBarrierWait(&barrier, sub);
      // ()boost::bind(unsubscribeAfterBarrierWait, &barrier, boost::ref(sub))
      });
    sub = sub_;

    bool cont = true;
    std::thread t(spinThread, &cont);

    barrier.wait();

    sub_.shutdown();
    cont = false;
    t.join();
  }
}

int
main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  g_argc = argc;
  g_argv = argv;

  miniros::init(g_argc, g_argv, "subscribe_unsubscribe");

  if (g_argc != 1)
  {
    puts("Too many arguments\n");
    return -1;
  }

  return RUN_ALL_TESTS();
}
