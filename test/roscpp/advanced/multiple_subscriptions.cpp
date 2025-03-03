/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

/* Author: Morgan Quigley */

/*
 * Subscribe to a topic multiple times
 */

#include <string>
#include <gtest/gtest.h>
#include <time.h>
#include <stdlib.h>
#include "miniros/ros.h"
#include <test_roscpp/TestArray.hxx>
#include <test_roscpp/TestEmpty.hxx>

int g_argc;
char** g_argv;

class Subscriptions : public testing::Test
{
  public:
    miniros::NodeHandle nh_;
    bool got_it[4], should_have_it[4];
    miniros::Subscriber subs_[4];
    miniros::Subscriber verify_sub_;
    miniros::Subscriber reset_sub_;
    bool test_ready;
    int n_test;

    void cb0(const test_roscpp::TestArrayConstPtr&) { if (!test_ready) return; got_it[0] = true; }
    void cb1(const test_roscpp::TestArrayConstPtr&) { if (!test_ready) return; got_it[1] = true; }
    void cb2(const test_roscpp::TestArrayConstPtr&) { if (!test_ready) return; got_it[2] = true; }
    void cb3(const test_roscpp::TestArrayConstPtr&) { if (!test_ready) return; got_it[3] = true; }
    void cb_verify(const test_roscpp::TestArrayConstPtr&)
    {
      if (!test_ready)
        return;
      n_test++;
      /*
      ASSERT_TRUE(((should_have_it[0] ? got_it[0] : true) &&
                   (should_have_it[1] ? got_it[1] : true) &&
                   (should_have_it[2] ? got_it[2] : true) &&
                   (should_have_it[3] ? got_it[3] : true)));
      */
    }
    void cb_reset(const test_roscpp::TestArrayConstPtr&)
    {
      got_it[0] = got_it[1] = got_it[2] = got_it[3] = false; test_ready = true;
    }

  protected:
    bool sub(int cb_num)
    { 
      MINIROS_INFO("Subscribing %d", cb_num);
      std::function<void(const test_roscpp::TestArrayConstPtr&)> funcs[4] =
      {
        [this](const test_roscpp::TestArrayConstPtr& msg) { this->cb0(msg);},
        [this](const test_roscpp::TestArrayConstPtr& msg) { this->cb1(msg);},
        [this](const test_roscpp::TestArrayConstPtr& msg) { this->cb2(msg);},
        [this](const test_roscpp::TestArrayConstPtr& msg) { this->cb3(msg);},
      };

      subs_[cb_num] = nh_.subscribe("roscpp/pubsub_test", 10, funcs[cb_num]);

      return subs_[cb_num];
    }
    bool sub_wrappers()
    {
      MINIROS_INFO("sub_wrappers");
      verify_sub_ = nh_.subscribe("roscpp/pubsub_test", 10, &Subscriptions::cb_verify, this);
      reset_sub_ = nh_.subscribe("roscpp/pubsub_test", 10, &Subscriptions::cb_reset, this);
      return verify_sub_ && reset_sub_;
    }
    bool unsub(int cb_num)
    {
      MINIROS_INFO("unsubscribing %d", cb_num);
      subs_[cb_num].shutdown();

      return true;
    }
    bool unsub_wrappers()
    {
      MINIROS_INFO("unsub wrappers");
      verify_sub_.shutdown();
      reset_sub_.shutdown();
      return true;
    }
};

TEST_F(Subscriptions, multipleSubscriptions)
{
  test_ready = false;

  for (int i = 0; i < 0x10; i++)
  {
    for (int j = 0; j < 4; j++)
      should_have_it[j] = (i & (1 << j) ? true : false);

    MINIROS_INFO(" testing: %d, %d, %d, %d\n",
           should_have_it[0],
           should_have_it[1],
           should_have_it[2],
           should_have_it[3]);

    for (int j = 0; j < 4; j++)
      if (should_have_it[j]) {
        ASSERT_TRUE(sub(j));
      }
    ASSERT_TRUE(sub_wrappers());

    miniros::Time t_start = miniros::Time::now();
    n_test = 0;
    while (n_test < 10 && miniros::Time::now() - t_start < miniros::Duration(5000.0))
    {
      static int count = 0;
      if (count++ % 10 == 0)
        MINIROS_INFO("%d/100 tests completed...\n", n_test);

      miniros::spinOnce();
      miniros::Duration(0.01).sleep();
    }
    
    for (int j = 0; j < 4; j++)
      if (should_have_it[j]) {
        ASSERT_TRUE(unsub(j));
      }
    ASSERT_TRUE(unsub_wrappers());
  }
  SUCCEED();
}

void callback1(const test_roscpp::TestArrayConstPtr&)
{

}

void callback2(const test_roscpp::TestEmptyConstPtr&)
{

}

TEST(Subscriptions2, multipleDifferentMD5Sums)
{
  miniros::NodeHandle nh;
  miniros::Subscriber sub1 = nh.subscribe("test", 0, callback1);

  try
  {
    miniros::Subscriber sub2 = nh.subscribe("test", 0, callback2);
    FAIL();
  }
  catch (miniros::ConflictingSubscriptionException&)
  {
    SUCCEED();
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "multiple_subscriptions");
  miniros::NodeHandle nh;
  g_argc = argc;
  g_argv = argv;
  return RUN_ALL_TESTS();
}

