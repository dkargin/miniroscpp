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
#include <miniros/transport/callback_queue.h>
#include <test_roscpp/TestArray.hxx>

int g_msg_count;
miniros::Duration g_dt;
uint32_t g_options;

class Subscriptions : public testing::Test
{
  public:
    bool success;
    bool failure;
    int msg_i;
    miniros::Publisher pub_;

    void messageCallback(const test_roscpp::TestArrayConstPtr& msg)
    {
      if(failure || success)
        return;

      //printf("received message %d\n", msg.counter);
      if (msg_i == -1)
      {
        msg_i = msg->counter - 1;
      }

      msg_i++;
      //MINIROS_INFO("msg_i=%d, counter=%d", msg_i, msg->counter);
      if(msg_i != msg->counter)
      {
        failure = true;
        MINIROS_INFO("failed");
      }
      else if(msg_i == (g_msg_count-1))
      {
        success = true;
        MINIROS_INFO("success");
      }
      else
      {
        pub_.publish(msg);
      }
    }

    void subscriberCallback(const miniros::SingleSubscriberPublisher&)
    {
      test_roscpp::TestArray msg;
      msg.counter = 0;
      pub_.publish(msg);
    }

  protected:
    void SetUp()
    {
      success = false;
      failure = false;

      msg_i = -1;
    }
    void TearDown()
    {

    }
};

TEST_F(Subscriptions, subPub)
{
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe("roscpp/pubsub_test", 0, &Subscriptions::messageCallback, (Subscriptions*)this);
  ASSERT_TRUE(sub);
  //pub_ = nh.advertise<test_roscpp::TestArray>("roscpp/subpub_test", 0, boost::bind(&Subscriptions::subscriberCallback, this, boost::placeholders::_1));
  pub_ = nh.advertise<test_roscpp::TestArray>("roscpp/subpub_test", 0,
    [this](const miniros::SingleSubscriberPublisher& sp) {
      this->subscriberCallback(sp);
    });
  ASSERT_TRUE(pub_);
  miniros::Time t1(miniros::Time::now()+g_dt);

  while(miniros::Time::now() < t1 && !success && !failure)
  {
    miniros::getGlobalCallbackQueue()->callAvailable(miniros::WallDuration(0.1));
  }

  if(success) {
    SUCCEED();
  } else if (miniros::Time::now() >= t1) {
    FAIL() << "timed out after receiving " << msg_i << " of " << g_msg_count << " messages";
  } else {
    FAIL() << "message counter did not match";
  }
}

#define USAGE "USAGE: sub_pub <count> <time>"

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "sub_pub");

  if(argc != 3)
  {
    puts(USAGE);
    return -1;
  }
  g_msg_count = atoi(argv[1]);
  g_dt.fromSec(atof(argv[2]));

  miniros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
