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

/* Author: Josh Faust */

/*
 * Test callback queue
 */

#include <gtest/gtest.h>
#include <miniros/transport/callback_queue.h>
#include <miniros/console.h>
#include <miniros/timer.h>

#include <atomic>
#include <thread>
#include <functional>

using namespace miniros;

class CountingCallback : public CallbackInterface
{
public:
  CountingCallback()
  : count(0)
  {}

  virtual CallResult call()
  {
    std::unique_lock<std::mutex> lock(mutex);
    ++count;

    return Success;
  }

  std::mutex mutex;
  size_t count;
};
typedef std::shared_ptr<CountingCallback> CountingCallbackPtr;

TEST(CallbackQueue, singleCallback)
{
  CountingCallbackPtr cb(std::make_shared<CountingCallback>());
  CallbackQueue queue;
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 1U);

  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 2U);

  queue.callOne();
  EXPECT_EQ(cb->count, 2U);

  queue.callAvailable();
  EXPECT_EQ(cb->count, 2U);
}

TEST(CallbackQueue, multipleCallbacksCallAvailable)
{
  CountingCallbackPtr cb(std::make_shared<CountingCallback>());
  CallbackQueue queue;
  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.addCallback(cb, 1);
  }

  queue.callAvailable();

  EXPECT_EQ(cb->count, 1000U);
}

TEST(CallbackQueue, multipleCallbacksCallOne)
{
  CountingCallbackPtr cb(std::make_shared<CountingCallback>());
  CallbackQueue queue;
  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.addCallback(cb, 1);
  }

  for (uint32_t i = 0; i < 1000; ++i)
  {
    queue.callOne();
    EXPECT_EQ(cb->count, i + 1);
  }
}

TEST(CallbackQueue, remove)
{
  CountingCallbackPtr cb1(std::make_shared<CountingCallback>());
  CountingCallbackPtr cb2(std::make_shared<CountingCallback>());
  CallbackQueue queue;
  queue.addCallback(cb1, 1);
  queue.addCallback(cb2, 2);
  queue.removeByID(1);
  queue.callAvailable();

  EXPECT_EQ(cb1->count, 0U);
  EXPECT_EQ(cb2->count, 1U);
}

class SelfRemovingCallback : public CallbackInterface
{
public:
  SelfRemovingCallback(CallbackQueue* queue, uint64_t id)
  : count(0)
  , queue(queue)
  , id(id)
  {}

  virtual CallResult call()
  {
    ++count;

    queue->removeByID(id);

    return Success;
  }

  size_t count;

  CallbackQueue* queue;
  uint64_t id;
};

typedef std::shared_ptr<SelfRemovingCallback> SelfRemovingCallbackPtr;

TEST(CallbackQueue, removeSelf)
{
  CallbackQueue queue;
  SelfRemovingCallbackPtr cb1(std::make_shared<SelfRemovingCallback>(&queue, 1));
  CountingCallbackPtr cb2(std::make_shared<CountingCallback>());
  queue.addCallback(cb1, 1);
  queue.addCallback(cb2, 1);
  queue.addCallback(cb2, 1);

  queue.callOne();

  queue.addCallback(cb2, 1);
  
  queue.callAvailable();

  EXPECT_EQ(cb1->count, 1U);
  EXPECT_EQ(cb2->count, 1U);
}

class RecursiveCallback : public CallbackInterface
{
public:
  RecursiveCallback(CallbackQueue* queue, bool use_available)
  : count(0)
  , queue(queue)
  , use_available(use_available)
  {}

  virtual CallResult call()
  {
    ++count;

    if (count < 3)
    {
      if (use_available)
      {
        queue->callAvailable();
      }
      else
      {
        queue->callOne();
      }
    }

    return Success;
  }

  size_t count;

  CallbackQueue* queue;
  bool use_available;
};
typedef std::shared_ptr<RecursiveCallback> RecursiveCallbackPtr;

TEST(CallbackQueue, recursive1)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(std::make_shared<RecursiveCallback>(&queue, true));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive2)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(std::make_shared<RecursiveCallback>(&queue, false));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive3)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(std::make_shared<RecursiveCallback>(&queue, false));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callAvailable();

  EXPECT_EQ(cb->count, 3U);
}

TEST(CallbackQueue, recursive4)
{
  CallbackQueue queue;
  RecursiveCallbackPtr cb(std::make_shared<RecursiveCallback>(&queue, true));
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.addCallback(cb, 1);
  queue.callOne();

  EXPECT_EQ(cb->count, 3U);
}

void callAvailableThread(CallbackQueue* queue, bool& done)
{
  while (!done)
  {
    queue->callAvailable(miniros::WallDuration(0.1));
  }
}

size_t runThreadedTest(const CountingCallbackPtr& cb, const std::function<void(CallbackQueue*, bool&)>& threadFunc)
{
  CallbackQueue queue;
  std::vector<std::thread> tg;
  bool done = false;

  for (uint32_t i = 0; i < 10; ++i)
  {
    tg.emplace_back(threadFunc, &queue, std::ref(done));
  }

  miniros::WallTime start = miniros::WallTime::now();
  size_t i = 0;
  while (miniros::WallTime::now() - start < miniros::WallDuration(5))
  {
    queue.addCallback(cb);
    ++i;
  }

  while (!queue.isEmpty())
  {
    miniros::WallDuration(0.01).sleep();
  }

  done = true;
  for (auto& t: tg)
    t.join();

  return i;
}

TEST(CallbackQueue, threadedCallAvailable)
{
  CountingCallbackPtr cb(std::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callAvailableThread);
  MINIROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

void callOneThread(CallbackQueue* queue, bool& done)
{
  while (!done)
  {
    queue->callOne(miniros::WallDuration(0.1));
  }
}

TEST(CallbackQueue, threadedCallOne)
{
  CountingCallbackPtr cb(std::make_shared<CountingCallback>());
  size_t i = runThreadedTest(cb, callOneThread);
  MINIROS_INFO_STREAM(i);
  EXPECT_EQ(cb->count, i);
}

// this class is just an ugly hack
// to access the constructor Timer(TimerOptions)
namespace miniros
{
class NodeHandle
{
public:
  static Timer createTimer(const TimerOptions& ops)
  {
    return Timer(ops);
  }
};
}

void dummyTimer(const miniros::TimerEvent&)
{
}

CallbackQueueInterface* recursiveTimerQueue;

void recursiveTimer(const miniros::TimerEvent&)
{
  // wait until the timer is TimerRecreationCallback is garbaged
  WallDuration(2).sleep();

  TimerOptions ops(Duration(0.1), dummyTimer, recursiveTimerQueue, false, false);
  Timer t = miniros::NodeHandle::createTimer(ops);
  t.start();
}

class TimerRecursionCallback : public CallbackInterface
{
public:
  TimerRecursionCallback(CallbackQueueInterface* _queue)
  : queue(_queue)
  {}

  virtual CallResult call()
  {
    TimerOptions ops(Duration(0.1), recursiveTimer, queue, false, false);
    Timer t = miniros::NodeHandle::createTimer(ops);
    t.start();

    // wait until the recursiveTimer has been fired
    WallDuration(1).sleep();

    return Success;
  }

  CallbackQueueInterface* queue;
};
typedef std::shared_ptr<TimerRecursionCallback> TimerRecursionCallbackPtr;

TEST(CallbackQueue, recursiveTimer)
{
  // ensure that the test does not dead-lock, see #3867
  miniros::Time::init();
  CallbackQueue queue;
  recursiveTimerQueue = &queue;
  TimerRecursionCallbackPtr cb(std::make_shared<TimerRecursionCallback>(&queue));
  queue.addCallback(cb, 1);

  std::vector<std::thread> tg;
  bool done = false;

  for (uint32_t i = 0; i < 2; ++i)
  {
    tg.emplace_back(callOneThread, &queue, std::ref(done));
  }

  while (!queue.isEmpty())
  {
    miniros::WallDuration(0.01).sleep();
  }

  done = true;
  for (auto& t: tg)
    t.join();
}

class ConditionObject
{
public:
  ConditionObject(CallbackQueue * _queue)
  : id(0), queue(_queue) {
    condition_sync.store(true);
    condition_one.store(false);
    condition_stop.store(false);
  }

  void add();

  unsigned long id;
  CallbackQueue * queue;
  std::atomic<bool> condition_one;
  std::atomic<bool> condition_sync;
  std::atomic<bool> condition_stop;
};

class RaceConditionCallback : public CallbackInterface
{
public:
  RaceConditionCallback(ConditionObject * _condition_object, unsigned long * _id)
  : condition_object(_condition_object), id(_id)
  {}

  virtual CallResult call()
  {
    condition_object->condition_one.store(false);
    return Success;
  }

  ConditionObject * condition_object;
  unsigned long * id;
};

void ConditionObject::add()
{
  while(!condition_stop.load())
  {
    if(condition_sync.load() && queue->isEmpty())
    {
      condition_one.store(true);
      id++;
      queue->addCallback(std::make_shared<RaceConditionCallback>(this, &id), id);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(1));
  }
}

TEST(CallbackQueue, raceConditionCallback)
{
  CallbackQueue queue;
  ConditionObject condition_object(&queue);

  std::thread t(&ConditionObject::add, &condition_object);
  for(unsigned int i = 0; i < 1000000; ++i)
  {
    condition_object.condition_sync.store(false);
    if (queue.callOne() == CallbackQueue::Called)
    {
      if(condition_object.condition_one.load())
      {
        condition_object.condition_stop.store(true);
        ASSERT_FALSE(condition_object.condition_one.load());
      }
    }

    queue.clear();
    condition_object.condition_sync.store(true);
  }
  condition_object.condition_stop.store(true);
  t.join();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::handleCrashes();

  return RUN_ALL_TESTS();
}



