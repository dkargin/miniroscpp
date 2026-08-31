/*
 * Test Publisher::getQueueLength() and Subscriber::getQueueLength().
 */

#include <gtest/gtest.h>

#include "miniros/ros.h"
#include "miniros/callback_queue.h"
#include <test_roscpp/TestArray.hxx>

#include "../../require_master.h"

using namespace miniros;
using namespace test_roscpp;

namespace {

void waitForSubscriber(const Publisher& pub)
{
  miniros::WallTime begin = miniros::WallTime::now();
  while (pub.getNumSubscribers() == 0 &&
         (miniros::WallTime::now() - begin < miniros::WallDuration(5.0)))
  {
    miniros::WallDuration(0.01).sleep();
  }
  ASSERT_GE(pub.getNumSubscribers(), 1u);
}

void waitForQueueLength(const Subscriber& sub, int expected)
{
  miniros::WallTime begin = miniros::WallTime::now();
  while (sub.getQueueLength() != expected &&
         (miniros::WallTime::now() - begin < miniros::WallDuration(5.0)))
  {
    miniros::WallDuration(0.01).sleep();
  }
  ASSERT_EQ(sub.getQueueLength(), expected);
}

} // namespace

TEST(QueueLength, invalidHandlesReturnZero)
{
  Publisher pub;
  Subscriber sub;
  EXPECT_EQ(pub.getQueueLength(), 0);
  EXPECT_EQ(sub.getQueueLength(), 0);
}

TEST(QueueLength, subscriberQueueFillsAndDrains)
{
  NodeHandle nh;
  CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  int received = 0;
  Publisher pub = nh.advertise<TestArray>("queue_length_fill", 10);
  Subscriber sub = nh.subscribe<TestArray>("queue_length_fill", 10,
    [&received](const TestArrayConstPtr&) { ++received; });

  waitForSubscriber(pub);

  EXPECT_EQ(sub.getQueueLength(), 0);
  EXPECT_EQ(pub.getQueueLength(), 0);

  // shared_ptr publish delivers intraprocess immediately into SubscriptionQueue.
  for (int i = 0; i < 5; ++i)
  {
    auto msg = std::make_shared<TestArray>();
    msg->counter = i;
    pub.publish(msg);
  }

  ASSERT_EQ(sub.getQueueLength(), 5);
  EXPECT_EQ(received, 0);

  queue.callAvailable();

  EXPECT_EQ(sub.getQueueLength(), 0);
  EXPECT_EQ(received, 5);

  // Staging plus intraprocess outbox (always 0) should be drained.
  miniros::WallTime begin = miniros::WallTime::now();
  while (pub.getQueueLength() != 0 &&
         (miniros::WallTime::now() - begin < miniros::WallDuration(2.0)))
  {
    miniros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(pub.getQueueLength(), 0);
}

TEST(QueueLength, subscriberQueueIsCapped)
{
  NodeHandle nh;
  CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  int received = 0;
  Publisher pub = nh.advertise<TestArray>("queue_length_cap", 10);
  Subscriber sub = nh.subscribe<TestArray>("queue_length_cap", 2,
    [&received](const TestArrayConstPtr&) { ++received; });

  waitForSubscriber(pub);

  for (int i = 0; i < 5; ++i)
  {
    auto msg = std::make_shared<TestArray>();
    msg->counter = i;
    pub.publish(msg);
  }

  ASSERT_EQ(sub.getQueueLength(), 2);

  queue.callAvailable();

  EXPECT_EQ(sub.getQueueLength(), 0);
  EXPECT_EQ(received, 2);
}

TEST(QueueLength, publisherStagingDrainsToSubscriber)
{
  NodeHandle nh;
  CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  int received = 0;
  // Advertise with a small send queue so a remote outbox would cap; intraprocess
  // still uses SubscriptionQueue instead of TransportSubscriberLink::outbox_.
  Publisher pub = nh.advertise<TestArray>("queue_length_pub", 4);
  Subscriber sub = nh.subscribe<TestArray>("queue_length_pub", 16,
    [&received](const TestArrayConstPtr&) { ++received; });

  waitForSubscriber(pub);

  TestArray msg;
  for (int i = 0; i < 8; ++i)
  {
    msg.counter = i;
    pub.publish(msg);
  }

  waitForQueueLength(sub, 8);

  miniros::WallTime begin = miniros::WallTime::now();
  while (pub.getQueueLength() != 0 &&
         (miniros::WallTime::now() - begin < miniros::WallDuration(2.0)))
  {
    miniros::WallDuration(0.01).sleep();
  }
  EXPECT_EQ(pub.getQueueLength(), 0);

  queue.callAvailable();
  EXPECT_EQ(sub.getQueueLength(), 0);
  EXPECT_EQ(received, 8);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::init(argc, argv, "queue_length");
  miniros::test::requireMasterOrExit("advanced-queue_length");

  miniros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
