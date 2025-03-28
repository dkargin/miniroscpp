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

/* Author: Josh Faust */

/*
 * Test handles
 */
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <thread>

#include <gtest/gtest.h>

#include <time.h>
#include <stdlib.h>

#include "miniros/ros.h"
#include "miniros/transport/callback_queue.h"

#include <test_roscpp/TestArray.hxx>
#include <test_roscpp/TestStringString.hxx>



using namespace miniros;
using namespace test_roscpp;

TEST(RoscppHandles, nodeHandleConstructionDestruction)
{
  {
    ASSERT_FALSE(miniros::isStarted());

    miniros::NodeHandle n1;
    ASSERT_TRUE(miniros::isStarted());

    {
      miniros::NodeHandle n2;
      ASSERT_TRUE(miniros::isStarted());

      {
        miniros::NodeHandle n3(n2);
        ASSERT_TRUE(miniros::isStarted());

        {
          miniros::NodeHandle n4 = n3;
          ASSERT_TRUE(miniros::isStarted());
        }
      }
    }

    ASSERT_TRUE(miniros::isStarted());
  }

  ASSERT_FALSE(miniros::isStarted());

  {
    miniros::NodeHandle n;
    ASSERT_TRUE(miniros::isStarted());
  }

  ASSERT_FALSE(miniros::isStarted());
}

TEST(RoscppHandles, nodeHandleParentWithRemappings)
{
  miniros::M_string remappings;
  remappings["a"] = "b";
  remappings["c"] = "d";
  miniros::NodeHandle n1("", remappings);

  // sanity checks
  EXPECT_STREQ(n1.resolveName("a").c_str(), "/b");
  EXPECT_STREQ(n1.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n1.resolveName("c").c_str(), "/d");
  EXPECT_STREQ(n1.resolveName("/c").c_str(), "/d");

  miniros::NodeHandle n2(n1, "my_ns");
  EXPECT_STREQ(n2.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n2.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n2.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n2.resolveName("/c").c_str(), "/d");

  miniros::NodeHandle n3(n2);
  EXPECT_STREQ(n3.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n3.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n3.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n3.resolveName("/c").c_str(), "/d");

  miniros::NodeHandle n4;
  n4 = n3;
  EXPECT_STREQ(n4.resolveName("a").c_str(), "/my_ns/a");
  EXPECT_STREQ(n4.resolveName("/a").c_str(), "/b");
  EXPECT_STREQ(n4.resolveName("c").c_str(), "/my_ns/c");
  EXPECT_STREQ(n4.resolveName("/c").c_str(), "/d");
}

int32_t g_recv_count = 0;
void subscriberCallback(const test_roscpp::TestArray::ConstPtr&)
{
  ++g_recv_count;
}

class SubscribeHelper
{
public:
  SubscribeHelper()
  : recv_count_(0)
  {}

  void callback(const test_roscpp::TestArray::ConstPtr&)
  {
    ++recv_count_;
  }

  int32_t recv_count_;
};

TEST(RoscppHandles, subscriberValidity)
{
  miniros::NodeHandle n;

  miniros::Subscriber sub;
  ASSERT_FALSE(sub);

  sub = n.subscribe("test", 0, subscriberCallback);
  ASSERT_TRUE(sub);
}

TEST(RoscppHandles, subscriberDestructionMultipleCallbacks)
{
  miniros::NodeHandle n;
  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("test", 0);
  test_roscpp::TestArray msg;

  {
    SubscribeHelper helper;
    miniros::Subscriber sub_class = n.subscribe("test", 0, &SubscribeHelper::callback, &helper);

    miniros::Duration d(0.05);
    int32_t last_class_count = helper.recv_count_;
    while (last_class_count == helper.recv_count_)
    {
      pub.publish(msg);
      miniros::spinOnce();
      d.sleep();
    }

    int32_t last_fn_count = g_recv_count;
    {
      miniros::Subscriber sub_fn = n.subscribe("test", 0, subscriberCallback);

      ASSERT_TRUE(sub_fn != sub_class);

      last_fn_count = g_recv_count;
      while (last_fn_count == g_recv_count)
      {
        pub.publish(msg);
        miniros::spinOnce();
        d.sleep();
      }
    }

    last_fn_count = g_recv_count;
    last_class_count = helper.recv_count_;
    while (last_class_count == helper.recv_count_)
    {
      pub.publish(msg);
      miniros::spinOnce();
      d.sleep();
    }
    d.sleep();

    ASSERT_EQ(last_fn_count, g_recv_count);
  }
}

TEST(RoscppHandles, subscriberSpinAfterSubscriberShutdown)
{
  miniros::NodeHandle n;
  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("test", 0);
  test_roscpp::TestArray msg;

  int32_t last_fn_count = g_recv_count;

  {
    miniros::Subscriber sub_fn = n.subscribe("test", 0, subscriberCallback);

    last_fn_count = g_recv_count;
    for (int i = 0; i < 10; ++i)
    {
      pub.publish(msg);
    }

    miniros::WallDuration(0.1).sleep();
  }

  miniros::spinOnce();

  ASSERT_EQ(last_fn_count, g_recv_count);
}

TEST(RoscppHandles, subscriberGetNumPublishers)
{
	miniros::NodeHandle n;
	miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("test", 0);

	miniros::Subscriber sub = n.subscribe("test", 0, subscriberCallback);

	miniros::WallTime begin = miniros::WallTime::now();
	while (sub.getNumPublishers() < 1 && (miniros::WallTime::now() - begin < miniros::WallDuration(1)))
	{
		miniros::spinOnce();
		miniros::WallDuration(0.1).sleep();
	}

	ASSERT_EQ(sub.getNumPublishers(), 1ULL);
}

TEST(RoscppHandles, subscriberCopy)
{
  miniros::NodeHandle n;

  g_recv_count = 0;

  {
    miniros::Subscriber sub1 = n.subscribe("/test", 0, subscriberCallback);

    {
      miniros::Subscriber sub2 = sub1;

      {
        miniros::Subscriber sub3(sub2);

        ASSERT_TRUE(sub3 == sub2);

        V_string topics;
        this_node::getSubscribedTopics(topics);
        ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
      }

      ASSERT_TRUE(sub2 == sub1);

      V_string topics;
      this_node::getSubscribedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getSubscribedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getSubscribedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

TEST(RoscppHandles, publisherCopy)
{
  miniros::NodeHandle n;

  g_recv_count = 0;

  {
    miniros::Publisher pub1 = n.advertise<test_roscpp::TestArray>("/test", 0);

    {
      miniros::Publisher pub2 = pub1;

      {
        miniros::Publisher pub3(pub2);

        ASSERT_TRUE(pub3 == pub2);

        V_string topics;
        this_node::getAdvertisedTopics(topics);
        ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
      }

      ASSERT_TRUE(pub2 == pub1);

      V_string topics;
      this_node::getAdvertisedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getAdvertisedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getAdvertisedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

TEST(RoscppHandles, publisherMultiple)
{
  miniros::NodeHandle n;

  g_recv_count = 0;

  {
    miniros::Publisher pub1 = n.advertise<test_roscpp::TestArray>("/test", 0);

    {
      miniros::Publisher pub2 = n.advertise<test_roscpp::TestArray>("/test", 0);

      ASSERT_TRUE(pub1 != pub2);

      V_string topics;
      this_node::getAdvertisedTopics(topics);
      ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
    }

    V_string topics;
    this_node::getAdvertisedTopics(topics);
    ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") != topics.end());
  }

  V_string topics;
  this_node::getAdvertisedTopics(topics);
  ASSERT_TRUE(std::find(topics.begin(), topics.end(), "/test") == topics.end());
}

bool serviceCallback(TestStringString::Request&, TestStringString::Response&)
{
  return true;
}

void pump(miniros::CallbackQueue* queue)
{
  while (queue->isEnabled())
  {
    queue->callAvailable(miniros::WallDuration(0.1));
  }
}

TEST(RoscppHandles, serviceAdv)
{
  miniros::NodeHandle n;
  TestStringString t;

  miniros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  std::thread th(pump, &queue);
  {
    miniros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);

    EXPECT_TRUE(miniros::service::call("/test_srv", t));
  }

  queue.disable();
  th.join();

  ASSERT_FALSE(miniros::service::call("/test_srv", t));
}

TEST(RoscppHandles, serviceAdvCopy)
{
  miniros::NodeHandle n;
  TestStringString t;

  miniros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  std::thread th(pump, &queue);

  {
    miniros::ServiceServer srv1 = n.advertiseService("/test_srv", serviceCallback);

    {
      miniros::ServiceServer srv2 = srv1;

      {
        miniros::ServiceServer srv3(srv2);

        ASSERT_TRUE(srv3 == srv2);

        EXPECT_TRUE(miniros::service::call("/test_srv", t));
      }

      ASSERT_TRUE(srv2 == srv1);

      EXPECT_TRUE(miniros::service::call("/test_srv", t));
    }

    EXPECT_TRUE(miniros::service::call("/test_srv", t));
  }

  ASSERT_FALSE(miniros::service::call("/test_srv", t));

  queue.disable();
  th.join();
}

TEST(RoscppHandles, serviceAdvMultiple)
{
  miniros::NodeHandle n;

  miniros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);
  miniros::ServiceServer srv2 = n.advertiseService("/test_srv", serviceCallback);
  ASSERT_TRUE(srv);
  ASSERT_FALSE(srv2);

  ASSERT_TRUE(srv != srv2);
}

int32_t g_sub_count = 0;
void connectedCallback(const miniros::SingleSubscriberPublisher&)
{
  ++g_sub_count;
}

TEST(RoscppHandles, trackedObjectWithAdvertiseSubscriberCallback)
{
  miniros::NodeHandle n;

  std::shared_ptr<char> tracked(std::make_shared<char>());

  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback(), tracked);

  g_recv_count = 0;
  g_sub_count = 0;
  miniros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  Duration d(0.01);
  while (g_sub_count == 0)
  {
    d.sleep();
    miniros::spinOnce();
  }
  ASSERT_EQ(g_sub_count, 1);

  sub.shutdown();

  tracked.reset();
  sub = n.subscribe("/test", 0, subscriberCallback);

  Duration d2(0.01);
  for (int i = 0; i < 10; ++i)
  {
    d2.sleep();
    miniros::spinOnce();
  }

  ASSERT_EQ(g_sub_count, 1);
}

TEST(RoscppHandles, spinAfterHandleShutdownWithAdvertiseSubscriberCallback)
{
  miniros::NodeHandle n;
  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());

  g_sub_count = 0;
  miniros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  while (pub.getNumSubscribers() == 0)
  {
    miniros::WallDuration(0.01).sleep();
  }

  pub.shutdown();

  miniros::spinOnce();

  ASSERT_EQ(g_sub_count, 0);
}

TEST(RoscppHandles, multiplePublishersWithSubscriberConnectCallback)
{
  miniros::NodeHandle n;
  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());

  g_sub_count = 0;
  miniros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);

  while (g_sub_count == 0)
  {
    miniros::WallDuration(0.01).sleep();
    miniros::spinOnce();
  }

  ASSERT_EQ(g_sub_count, 1);
  g_sub_count = 0;

  miniros::Publisher pub2 = n.advertise<test_roscpp::TestArray>("/test", 0, connectedCallback, SubscriberStatusCallback());
  miniros::spinOnce();

  ASSERT_EQ(g_sub_count, 1);
}

class ServiceClass
{
public:
  bool serviceCallback(TestStringString::Request&, TestStringString::Response&)
  {
    return true;
  }
};

TEST(RoscppHandles, trackedObjectWithServiceCallback)
{
  miniros::NodeHandle n;

  miniros::CallbackQueue queue;
  n.setCallbackQueue(&queue);
  std::thread th(pump, &queue);

  std::shared_ptr<ServiceClass> tracked(std::make_shared<ServiceClass>());
  miniros::ServiceServer srv = n.advertiseService("/test_srv", &ServiceClass::serviceCallback, tracked);

  TestStringString t;
  EXPECT_TRUE(miniros::service::call("/test_srv", t));

  tracked.reset();

  ASSERT_FALSE(miniros::service::call("/test_srv", t));

  queue.disable();
  th.join();
}

TEST(RoscppHandles, trackedObjectWithSubscriptionCallback)
{
  miniros::NodeHandle n;

  std::shared_ptr<SubscribeHelper> tracked(std::make_shared<SubscribeHelper>());

  g_recv_count = 0;
  miniros::Subscriber sub = n.subscribe("/test", 0, &SubscribeHelper::callback, tracked);

  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("/test", 0);

  test_roscpp::TestArray msg;
  Duration d(0.01);
  while (tracked->recv_count_ == 0)
  {
    pub.publish(msg);
    d.sleep();
    miniros::spinOnce();
  }
  ASSERT_GE(tracked->recv_count_, 1);

  tracked.reset();

  pub.publish(msg);
  Duration d2(0.01);
  for (int i = 0; i < 10; ++i)
  {
    d2.sleep();
    miniros::spinOnce();
  }
}

TEST(RoscppHandles, nodeHandleNames)
{
  miniros::NodeHandle n1;
  EXPECT_STREQ(n1.resolveName("blah").c_str(), "/blah");

  try
  {
    n1.resolveName("~blah");
    FAIL();
  }
  catch (miniros::InvalidNameException&)
  {
  }

  miniros::NodeHandle n2("internal_ns");
  EXPECT_STREQ(n2.resolveName("blah").c_str(), "/internal_ns/blah");

  miniros::NodeHandle n3(n2, "2");
  EXPECT_STREQ(n3.resolveName("blah").c_str(), "/internal_ns/2/blah");

  miniros::NodeHandle n4("~");
  EXPECT_STREQ(n4.resolveName("blah").c_str(), (miniros::this_node::getName() + "/blah").c_str());
  
  try {
    miniros::NodeHandle n5(n2, "illegal_name!!!");
    FAIL();
  } catch (miniros::InvalidNameException&) { }

}

TEST(RoscppHandles, nodeHandleShutdown)
{
  miniros::NodeHandle n;

  miniros::Subscriber sub = n.subscribe("/test", 0, subscriberCallback);
  miniros::Publisher pub = n.advertise<test_roscpp::TestArray>("/test", 0);
  miniros::ServiceServer srv = n.advertiseService("/test_srv", serviceCallback);

  n.shutdown();

  ASSERT_FALSE(pub);
  ASSERT_FALSE(sub);
  ASSERT_FALSE(srv);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "test_handles");

  return RUN_ALL_TESTS();
}

