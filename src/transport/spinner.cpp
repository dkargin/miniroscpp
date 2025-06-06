/*
 * Copyright (C) 2009, Willow Garage, Inc.
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

#include "miniros/spinner.h"
#include "miniros/ros.h"
#include "miniros/transport/callback_queue.h"

#include <thread>
#include <mutex>

namespace {

/** class to monitor running single-threaded spinners.
 *
 *  Calling the callbacks of a callback queue _in order_, requires a unique SingleThreadedSpinner
 *  spinning on the queue. Other threads accessing the callback queue will probably intercept execution order.

 *  To avoid multiple SingleThreadedSpinners (started from different threads) to operate on the same callback queue,
 *  this class stores a map of all spinned callback queues.
 *  If the spinner is single threaded, the corresponding thread-id is stored in the map
 *  and if other threads will try to spin the same queue, an error message is issued.
 *
 *  If the spinner is multi-threaded, the stored thread-id is NULL and future SingleThreadedSpinners
 *  should not spin this queue. However, other multi-threaded spinners are allowed.
 */
struct SpinnerMonitor
{
  /* store spinner information per callback queue:
     Only alike spinners (single-threaded or multi-threaded) are allowed on a callback queue.
     For single-threaded spinners we store their thread id.
     We store the number of alike spinners operating on the callback queue.
  */
  struct Entry
  {
    Entry(const std::thread::id &tid) : tid(tid), num(0) {}

    std::thread::id tid; // proper thread id of single-threaded spinner
    unsigned int num; // number of (alike) spinners serving this queue
  };

  /// add a queue to the list
  bool add(miniros::CallbackQueue* queue, bool single_threaded)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    std::thread::id tid; // current thread id for single-threaded spinners, zero for multi-threaded ones
    if (single_threaded)
      tid = std::this_thread::get_id();

    std::map<miniros::CallbackQueue*, Entry>::iterator it = spinning_queues_.find(queue);
    bool can_spin = ( it == spinning_queues_.end() || // we will spin on any new queue
                      it->second.tid == tid ); // otherwise spinner must be alike (all multi-threaded: 0, or single-threaded on same thread id)

    if (!can_spin)
      return false;

    if (it == spinning_queues_.end())
      it = spinning_queues_.insert(it, std::make_pair(queue, Entry(tid)));

    // increment number of active spinners
    it->second.num += 1;

    return true;
  }

  /// remove a queue from the list
  void remove(miniros::CallbackQueue* queue)
  {
    std::scoped_lock<std::mutex> lock(mutex_);
    std::map<miniros::CallbackQueue*, Entry>::iterator it = spinning_queues_.find(queue);
    MINIROS_ASSERT_MSG(it != spinning_queues_.end(), "Call to SpinnerMonitor::remove() without matching call to add().");

    if (it->second.tid != std::thread::id() && it->second.tid != std::this_thread::get_id())
    {
      // This doesn't harm, but isn't good practice?
      // It was enforced by the previous implementation.
      MINIROS_WARN("SpinnerMonitor::remove() called from different thread than add().");
    }

    MINIROS_ASSERT_MSG(it->second.num > 0, "SpinnerMonitor::remove(): Invalid spinner count (0) encountered.");
    it->second.num -= 1;
    if (it->second.num == 0)
      spinning_queues_.erase(it); // erase queue entry to allow future queues with same pointer
  }

  std::map<miniros::CallbackQueue*, Entry> spinning_queues_;
  std::mutex mutex_;
};

SpinnerMonitor spinner_monitor;
const std::string DEFAULT_ERROR_MESSAGE =
    "Attempt to spin a callback queue from two spinners, one of them being single-threaded.";
}

namespace miniros
{


void SingleThreadedSpinner::spin(CallbackQueue* queue)
{
  if (!queue)
  {
    queue = getGlobalCallbackQueue();
  }

  if (!spinner_monitor.add(queue, true))
  {
    std::string errorMessage = "SingleThreadedSpinner: " + DEFAULT_ERROR_MESSAGE + " You might want to use a MultiThreadedSpinner instead.";
    MINIROS_FATAL_STREAM(errorMessage);
    throw std::runtime_error(errorMessage);
  }

  miniros::WallDuration timeout(0.1f);
  miniros::NodeHandle n;
  while (n.ok())
  {
    queue->callAvailable(timeout);
  }
  spinner_monitor.remove(queue);
}

MultiThreadedSpinner::MultiThreadedSpinner(uint32_t thread_count)
: thread_count_(thread_count)
{
}

void MultiThreadedSpinner::spin(CallbackQueue* queue)
{
  AsyncSpinner s(thread_count_, queue);
  s.start();

  miniros::waitForShutdown();
}

class AsyncSpinnerImpl
{
public:
  AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue);
  ~AsyncSpinnerImpl();

  bool canStart();
  void start();
  void stop();

private:
  void threadFunc();

  std::mutex mutex_;
  std::list<std::thread> threads_;

  uint32_t thread_count_;
  CallbackQueue* callback_queue_;

  volatile bool continue_;

  miniros::NodeHandle nh_;
};

AsyncSpinnerImpl::AsyncSpinnerImpl(uint32_t thread_count, CallbackQueue* queue)
: thread_count_(thread_count)
, callback_queue_(queue)
, continue_(false)
{
  if (thread_count == 0)
  {
    thread_count_ = std::thread::hardware_concurrency();

    if (thread_count_ == 0)
    {
      thread_count_ = 1;
    }
  }

  if (!queue)
  {
    callback_queue_ = getGlobalCallbackQueue();
  }
}

AsyncSpinnerImpl::~AsyncSpinnerImpl()
{
  stop();
}

bool AsyncSpinnerImpl::canStart()
{
  return true;
}

void AsyncSpinnerImpl::start()
{
  std::scoped_lock<std::mutex> lock(mutex_);

  if (continue_)
    return; // already spinning

  if (!spinner_monitor.add(callback_queue_, false))
  {
    std::string errorMessage = "AsyncSpinnerImpl: " + DEFAULT_ERROR_MESSAGE;
    MINIROS_FATAL_STREAM(errorMessage);
    throw std::runtime_error(errorMessage);
  }

  continue_ = true;

  for (uint32_t i = 0; i < thread_count_; ++i)
  {
    threads_.emplace_back(&AsyncSpinnerImpl::threadFunc, this);
  }
}

void AsyncSpinnerImpl::stop()
{
  std::scoped_lock<std::mutex> lock(mutex_);
  if (!continue_)
    return;

  continue_ = false;
  for (auto& thread: threads_)
      thread.join();

  spinner_monitor.remove(callback_queue_);
}

void AsyncSpinnerImpl::threadFunc()
{
  disableAllSignalsInThisThread();

  CallbackQueue* queue = callback_queue_;
  bool use_call_available = thread_count_ == 1;
  WallDuration timeout(0.1);

  while (continue_ && nh_.ok())
  {
    if (use_call_available)
    {
      queue->callAvailable(timeout);
    }
    else
    {
      queue->callOne(timeout);
    }
  }
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count)
: impl_(new AsyncSpinnerImpl(thread_count, 0))
{
}

AsyncSpinner::AsyncSpinner(uint32_t thread_count, CallbackQueue* queue)
: impl_(new AsyncSpinnerImpl(thread_count, queue))
{
}

bool AsyncSpinner::canStart()
{
  return impl_->canStart();
}

void AsyncSpinner::start()
{
  impl_->start();
}

void AsyncSpinner::stop()
{
  impl_->stop();
}

}
