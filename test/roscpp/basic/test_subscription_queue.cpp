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

/*
 * Test subscription queue
 */

#include <gtest/gtest.h>
#include "miniros/transport/subscription_queue.h"
#include "miniros/transport/message_deserializer.h"
#include "miniros/transport/callback_queue_interface.h"
#include "miniros/transport/subscription_callback_helper.h"
#include "miniros/init.h"

#include <thread>
#include <mutex>

#include "barrier.h"

#ifdef MINIROS_INTERNAL_HEADER
#error "Internal header has leaked to user headers"
#endif

using namespace miniros;

class FakeMessage
{
public:
  virtual const std::string __getDataType() const { return ""; }
  virtual const std::string __getMD5Sum() const { return ""; }
  virtual const std::string __getMessageDefinition() const { return ""; }
  virtual uint32_t serializationLength() const { return 0; }
  virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const { (void)seq; return write_ptr; }
  virtual uint8_t *deserialize(uint8_t *read_ptr) { return read_ptr; }
};

class FakeSubHelper : public SubscriptionCallbackHelper
{
public:
  FakeSubHelper()
  : calls_(0)
  {}

  virtual VoidConstPtr deserialize(const SubscriptionCallbackHelperDeserializeParams&)
  {
    return std::make_shared<FakeMessage>();
  }

  virtual std::string getMD5Sum() { return ""; }
  virtual std::string getDataType() { return ""; }

  virtual void call(SubscriptionCallbackHelperCallParams& params)
  {
    (void)params;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      ++calls_;
    }

    if (cb_)
    {
      cb_();
    }
  }

  virtual const std::type_info& getTypeInfo() { return typeid(FakeMessage); }
  virtual bool isConst() { return true; }
  virtual bool hasHeader() { return false; }

  std::mutex mutex_;
  int32_t calls_;

  std::function<void(void)> cb_;
};
typedef std::shared_ptr<FakeSubHelper> FakeSubHelperPtr;

TEST(SubscriptionQueue, queueSize)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 1, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  ASSERT_FALSE(queue->full());

  queue->push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue->full());

  ASSERT_EQ(queue->call(), CallbackInterface::Success);

  ASSERT_FALSE(queue->full());

  queue->push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue->full());

  ASSERT_TRUE(queue->ready());

  queue->push(helper, des, false, VoidConstWPtr(), true);

  ASSERT_TRUE(queue->full());

  ASSERT_EQ(queue->call(), CallbackInterface::Success);
  ASSERT_EQ(queue->call(), CallbackInterface::Invalid);

  ASSERT_EQ(helper->calls_, 2);
}

TEST(SubscriptionQueue, infiniteQueue)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 0, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  ASSERT_FALSE(queue->full());

  queue->push(helper, des, false, VoidConstWPtr(), true);
  ASSERT_EQ(queue->call(), CallbackInterface::Success);

  ASSERT_FALSE(queue->full());

  for (int i = 0; i < 10000; ++i)
  {
    queue->push(helper, des, false, VoidConstWPtr(), true);
  }

  ASSERT_FALSE(queue->full());

  for (int i = 0; i < 10000; ++i)
  {
    ASSERT_EQ(queue->call(), CallbackInterface::Success);
  }

  ASSERT_EQ(queue->call(), CallbackInterface::Invalid);

  ASSERT_EQ(helper->calls_, 10001);
}

TEST(SubscriptionQueue, clearCall)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 1, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  queue->push(helper, des, false, VoidConstWPtr(), true);
  queue->clear();
  ASSERT_EQ(queue->call(), CallbackInterface::Invalid);
}

TEST(SubscriptionQueue, clearThenAddAndCall)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 1, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  queue->push(helper, des, false, VoidConstWPtr(), true);
  queue->clear();
  queue->push(helper, des, false, VoidConstWPtr(), true);
  ASSERT_EQ(queue->call(), CallbackInterface::Success);
}

void clearInCallbackCallback(SubscriptionQueue& queue)
{
  queue.clear();
}

TEST(SubscriptionQueue, clearInCallback)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 1, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  helper->cb_ = [queue](){ clearInCallbackCallback(*queue);};
  queue->push(helper, des, false, VoidConstWPtr(), true);
  queue->call();
}

void clearWhileThreadIsBlockingCallback(bool* done, Barrier* barrier)
{
  barrier->wait();
  miniros::WallDuration(.1).sleep();
  *done = true;
}

void callThread(SubscriptionQueue& queue)
{
  queue.call();
}

TEST(SubscriptionQueue, clearWhileThreadIsBlocking)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 1, false);

  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  bool done = false;
  Barrier barrier(2);
  helper->cb_ = [&barrier, &done](){clearWhileThreadIsBlockingCallback(&done, &barrier);};
  queue->push(helper, des, false, VoidConstWPtr(), true);
  std::thread t(callThread, std::ref(*queue));
  barrier.wait();

  queue->clear();

  ASSERT_TRUE(done);
  t.join();
}

void waitForBarrier(Barrier* bar)
{
  bar->wait();
}

TEST(SubscriptionQueue, concurrentCallbacks)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 0, true);
  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  Barrier bar(2);
  helper->cb_ = [&bar](){waitForBarrier(&bar);};
  queue->push(helper, des, false, VoidConstWPtr(), true);
  queue->push(helper, des, false, VoidConstWPtr(), true);
  std::thread t1(callThread, std::ref(*queue));
  std::thread t2(callThread, std::ref(*queue));
  t1.join();
  t2.join();

  ASSERT_EQ(helper->calls_, 2);
}

void waitForASecond()
{
  miniros::WallDuration(1.0).sleep();
}

TEST(SubscriptionQueue, nonConcurrentOrdering)
{
  auto queue = std::make_shared<miniros::SubscriptionQueue>("blah", 0, false);
  FakeSubHelperPtr helper(std::make_shared<FakeSubHelper>());
  MessageDeserializerPtr des(std::make_shared<MessageDeserializer>(helper, SerializedMessage(), std::shared_ptr<M_string>()));

  helper->cb_ = waitForASecond;
  queue->push(helper, des, false, VoidConstWPtr(), true);
  queue->push(helper, des, false, VoidConstWPtr(), true);
  std::thread t1(callThread, std::ref(*queue));
  std::thread t2(callThread, std::ref(*queue));
  t1.join();
  t2.join();

  ASSERT_EQ(helper->calls_, 1);
  queue->call();
  ASSERT_EQ(helper->calls_, 2);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();
  miniros::init(argc, argv, "blah");
  return RUN_ALL_TESTS();
}


