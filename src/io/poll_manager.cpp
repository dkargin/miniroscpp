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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#include <mutex>
#include <signal.h>

// ROS log will write to the channel "miniros.http[.server]"
#define MINIROS_PACKAGE_NAME "poll_manager"

#include "miniros/common.h"
#include "miniros/io/poll_manager.h"
#include "rosconsole/local_log.h"

namespace miniros
{

const PollManagerPtr& PollManager::instance()
{
  static PollManagerPtr poll_manager = std::make_shared<PollManager>();
  return poll_manager;
}

PollManager::PollManager()
  : shutting_down_(false), running_(false)
{
}

PollManager::~PollManager()
{
  shutdown();
}

void PollManager::start()
{
  shutting_down_ = false;
  if (!running_) {
    thread_ = std::thread(&PollManager::threadFunc, this);
    running_ = true;
  }
}

void PollManager::shutdown()
{
  if (shutting_down_) return;

  LOCAL_DEBUG("PollManager::shutdown()");

  if (thread_.joinable() && thread_.get_id() != std::this_thread::get_id())
  {
    shutting_down_ = true;
    thread_.join();
    poll_watchers_.disconnectAll();
    running_ = false;
    LOCAL_DEBUG("PollManager::shutdown() - complete");
  } else {
    LOCAL_DEBUG("PollManager::shutdown() - skipped from other thread");
  }
}

// Defined in init.cpp
void checkForShutdown();

void PollManager::threadFunc()
{
  setThreadName("PollManager");
  disableAllSignalsInThisThread();

  while (!shutting_down_)
  {

    {
      std::scoped_lock<PollWatchers> lock(poll_watchers_);
      auto it = poll_watchers_.begin();
      for (; it != poll_watchers_.end(); it++)
      {
        if (it)
          it->onPollEvents();
      }
    }

    if (shutting_down_)
    {
      break;
    }

    // While it breaks abstraction, it reduces number of mutexes and threading challenges.
    checkForShutdown();

    constexpr int updatePeriodMS = 50;

    poll_set_.update(updatePeriodMS);
  }

  LOCAL_DEBUG("PollManager::threadFunc() - exit");
}

void PollManager::addPollThreadWatcher(PollWatcher* watcher) {
  MINIROS_ASSERT(watcher);
  if (watcher) {
    this->poll_watchers_.attach(watcher);
  }
}

}
