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

#include "test_roscpp/TestEmpty.h"
#include "test_roscpp/TestArray.h"

#include <std_srvs/Empty.h>

class AnyMessage
{
};
typedef boost::shared_ptr<AnyMessage> AnyMessagePtr;
typedef boost::shared_ptr<AnyMessage const> AnyMessageConstPtr;

namespace ros
{
namespace message_traits
{

template<>
struct MD5Sum<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct DataType<AnyMessage>
{
  static const char* value() { return "*"; }
  static const char* value(const AnyMessage&) { return "*"; }
};

template<>
struct Definition<AnyMessage>
{
};

}

namespace serialization
{
template<>
struct Serializer<AnyMessage>
{
  template<typename Stream, typename T>
  static void allInOne(Stream, T)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
};
}
}

struct AnyHelper
{
  AnyHelper()
  : count(0)
  {
  }

  void cb(const AnyMessageConstPtr&)
  {
    ++count;
  }

  uint32_t count;
};


TEST(SubscribeStar, simpleSubFirstIntra)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  miniros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  AnyMessagePtr msg(boost::make_shared<AnyMessage>());
  pub.publish(msg);
  miniros::spinOnce();
  EXPECT_EQ(h.count, 1U);
}

TEST(SubscribeStar, simplePubFirstIntra)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  miniros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);

  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  AnyMessagePtr msg(boost::make_shared<AnyMessage>());
  pub.publish(msg);
  miniros::spinOnce();
  EXPECT_EQ(h.count, 1U);
}

void emptyCallback(const test_roscpp::TestEmptyConstPtr&)
{

}

TEST(SubscribeStar, multipleSubsStarFirstIntra)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  miniros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);

  miniros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(sub2.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
  EXPECT_EQ(sub2.getNumPublishers(), 0U);
}

TEST(SubscribeStar, multipleSubsConcreteFirstIntra)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);
  miniros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);

  miniros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_EQ(sub2.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
  EXPECT_EQ(sub2.getNumPublishers(), 0U);
}

TEST(SubscribeStar, multipleShutdownConcreteIntra)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_intra", 0, &AnyHelper::cb, &h);
  miniros::Subscriber sub2 = nh.subscribe("test_star_intra", 0, emptyCallback);
  sub2.shutdown();

  miniros::Publisher pub = nh.advertise<test_roscpp::TestEmpty>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 1U);
  EXPECT_EQ(sub.getNumPublishers(), 1U);

  pub.shutdown();
  pub = nh.advertise<test_roscpp::TestArray>("test_star_intra", 0);
  EXPECT_EQ(pub.getNumSubscribers(), 0U);
  EXPECT_EQ(sub.getNumPublishers(), 0U);
}

TEST(SubscribeStar, simpleInter)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h);

  miniros::WallDuration(1.0).sleep();
  miniros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_GT(h.count, 0U);
}

TEST(SubscribeStar, simpleInterUDP)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h, miniros::TransportHints().udp());

  miniros::WallDuration(1.0).sleep();
  miniros::spinOnce();

  EXPECT_EQ(sub.getNumPublishers(), 1U);
  EXPECT_GT(h.count, 0U);
}

// must be the last test as it makes the service node exit
TEST(SubscribeStar, switchTypeInter)
{
  miniros::NodeHandle nh;
  AnyHelper h;
  miniros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h);
  miniros::Subscriber sub2 = nh.subscribe("test_star_inter", 0, emptyCallback);

  miniros::WallDuration(1.0).sleep();
  miniros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 1U);
  ASSERT_EQ(sub2.getNumPublishers(), 1U);

  std_srvs::Empty srv;
  // by invoking the service call the service node will exit with FATAL
  ASSERT_TRUE(miniros::service::call("switch_publisher_type", srv));

  miniros::WallDuration(1.0).sleep();
  miniros::spinOnce();

  ASSERT_EQ(sub.getNumPublishers(), 0U);
  ASSERT_EQ(sub2.getNumPublishers(), 0U);
}

// disabled because switchTypeInter will stop the other node intentionally
//TEST(SubscribeStar, switchTypeInterUDP)
//{
//  miniros::NodeHandle nh;
//  AnyHelper h;
//  miniros::Subscriber sub = nh.subscribe("test_star_inter", 0, &AnyHelper::cb, &h, miniros::TransportHints().udp());
//  miniros::Subscriber sub2 = nh.subscribe("test_star_inter", 0, emptyCallback, miniros::TransportHints().udp());
//
//  miniros::WallDuration(1.0).sleep();
//  miniros::spinOnce();
//
//  ASSERT_EQ(sub.getNumPublishers(), 1U);
//  ASSERT_EQ(sub2.getNumPublishers(), 1U);
//
//  std_srvs::Empty srv;
//  // by invoking the service call the service node will exit with FATAL
//  ASSERT_TRUE(miniros::service::call("switch_publisher_type", srv));
//
//  miniros::WallDuration(1.0).sleep();
//  miniros::spinOnce();
//
//  ASSERT_EQ(sub.getNumPublishers(), 0U);
//  ASSERT_EQ(sub2.getNumPublishers(), 0U);
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "subscribe_star");
  miniros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
