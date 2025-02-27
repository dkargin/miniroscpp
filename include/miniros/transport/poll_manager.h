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

#ifndef ROSCPP_POLL_MANAGER_H
#define ROSCPP_POLL_MANAGER_H

#include <atomic>
#include <thread>

#include "miniros/macros.h"
#include "miniros/internal/forwards.h"
#include "miniros/internal/observer.h"

#include "poll_set.h"
#include "miniros/common.h"


namespace miniros
{

class PollManager;
typedef std::shared_ptr<PollManager> PollManagerPtr;

class MINIROS_DECL PollManager
{
public:
  class PollWatcher : public observer::Connection {
  public:
    virtual void onPollEvents() = 0;
  };

  static const PollManagerPtr& instance();

  PollManager();
  ~PollManager();

  PollSet& getPollSet() { return poll_set_; }

  void addPollThreadWatcher(PollWatcher* watcher);

  void start();
  void shutdown();

private:
  void threadFunc();

  PollSet poll_set_;
  std::atomic_bool shutting_down_;

  /// Signal raised when this connection is dropped.
  using PollWatchers = observer::Target<PollWatcher>;
  PollWatchers poll_watchers_;

  std::thread thread_;
};

}

#endif
