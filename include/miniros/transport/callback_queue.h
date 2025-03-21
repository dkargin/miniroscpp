/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MINIROS_CALLBACK_QUEUE_H
#define MINIROS_CALLBACK_QUEUE_H

#include "miniros/transport/callback_queue_interface.h"
#include "miniros/internal/thread_local_ptr.h"
#include "miniros/rostime.h"
#include "miniros/common.h"

#include <mutex>
#include <shared_mutex>
#include <condition_variable>

#include <list>
#include <deque>
#include <map>

namespace miniros
{

/**
 * \brief This is the default implementation of the miniros::CallbackQueueInterface
 */
class MINIROS_DECL CallbackQueue : public CallbackQueueInterface
{
public:
  CallbackQueue(bool enabled = true);
  ~CallbackQueue() override;

  virtual void addCallback(const CallbackInterfacePtr& callback, uint64_t removal_id = 0) override;
  virtual void removeByID(uint64_t removal_id) override;

  enum CallOneResult
  {
    Called,
    TryAgain,
    Disabled,
    Empty,
  };

  /**
   * \brief Pop a single callback off the front of the queue and invoke it.  If the callback was not ready to be called,
   * pushes it back onto the queue.
   */
  CallOneResult callOne()
  {
    return callOne(miniros::WallDuration());
  }

  /**
   * \brief Pop a single callback off the front of the queue and invoke it.  If the callback was not ready to be called,
   * pushes it back onto the queue.  This version includes a timeout which lets you specify the amount of time to wait for
   * a callback to be available before returning.
   *
   * \param timeout The amount of time to wait for a callback to be available.  If there is already a callback available,
   * this parameter does nothing.
   */
  CallOneResult callOne(miniros::WallDuration timeout);

  /**
   * \brief Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.
   */
  void callAvailable()
  {
    callAvailable(miniros::WallDuration());
  }
  /**
   * \brief Invoke all callbacks currently in the queue.  If a callback was not ready to be called, pushes it back onto the queue.  This version
   * includes a timeout which lets you specify the amount of time to wait for a callback to be available before returning.
   *
   * \param timeout The amount of time to wait for at least one callback to be available.  If there is already at least one callback available,
   * this parameter does nothing.
   */
  void callAvailable(miniros::WallDuration timeout);

  /**
   * \brief returns whether or not the queue is empty
   */
  bool empty() { return isEmpty(); }
  /**
   * \brief returns whether or not the queue is empty
   */
  bool isEmpty();
  /**
   * \brief Removes all callbacks from the queue.  Does \b not wait for calls currently in progress to finish.
   */
  void clear();

  /**
   * \brief Enable the queue (queue is enabled by default)
   */
  void enable();
  /**
   * \brief Disable the queue, meaning any calls to addCallback() will have no effect
   */
  void disable();
  /**
   * \brief Returns whether or not this queue is enabled
   */
  bool isEnabled();

protected:
  void setupTLS();

  struct TLS;
  CallOneResult callOneCB(TLS* tls);

  struct IDInfo
  {
    uint64_t id;
    std::shared_mutex calling_rw_mutex;
  };

  typedef std::shared_ptr<IDInfo> IDInfoPtr;
  typedef std::map<uint64_t, IDInfoPtr> M_IDInfo;

  IDInfoPtr getIDInfo(uint64_t id);

  struct CallbackInfo
  {
    CallbackInfo()
    : removal_id(0)
    , marked_for_removal(false)
    {}
    CallbackInterfacePtr callback;
    uint64_t removal_id;
    bool marked_for_removal;
  };
  typedef std::list<CallbackInfo> L_CallbackInfo;
  typedef std::deque<CallbackInfo> D_CallbackInfo;
  D_CallbackInfo callbacks_;
  size_t calling_;
  std::mutex mutex_;
  std::condition_variable condition_;

  std::mutex id_info_mutex_;
  M_IDInfo id_info_;

  struct TLS
  {
    TLS()
    : calling_in_this_thread(0xffffffffffffffffULL)
    , cb_it(callbacks.end())
    {}
    uint64_t calling_in_this_thread;
    D_CallbackInfo callbacks;
    D_CallbackInfo::iterator cb_it;
  };

  ThreadLocalPointer<TLS> tls_;

  bool enabled_;
};
typedef std::shared_ptr<CallbackQueue> CallbackQueuePtr;

} // namespace miniros

#endif // MINIROS_CALLBACK_QUEUE_H
