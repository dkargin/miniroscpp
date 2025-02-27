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

#ifndef MINIROS_SUBSCRIPTION_QUEUE_H
#define MINIROS_SUBSCRIPTION_QUEUE_H

#include "miniros/internal/forwards.h"
#include "miniros/common.h"
#include "miniros/message_event.h"
#include "callback_queue_interface.h"

#include <memory>
#include <mutex>
#include <deque>

namespace miniros
{

class MessageDeserializer;
typedef std::shared_ptr<MessageDeserializer> MessageDeserializerPtr;

class SubscriptionCallbackHelper;
typedef std::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

class MINIROS_DECL SubscriptionQueue : public CallbackInterface, public std::enable_shared_from_this<SubscriptionQueue>
{
private:
  struct Item
  {
    SubscriptionCallbackHelperPtr helper;
    MessageDeserializerPtr deserializer;

    bool has_tracked_object;
    VoidConstWPtr tracked_object;

    bool nonconst_need_copy;
    miniros::Time receipt_time;
  };
  typedef std::deque<Item> D_Item;

public:
  SubscriptionQueue(const std::string& topic, int32_t queue_size, bool allow_concurrent_callbacks);
  ~SubscriptionQueue();

  void push(const SubscriptionCallbackHelperPtr& helper, const MessageDeserializerPtr& deserializer, 
	    bool has_tracked_object, const VoidConstWPtr& tracked_object, bool nonconst_need_copy, 
	    miniros::Time receipt_time = miniros::Time(), bool* was_full = 0);
  void clear();

  virtual CallbackInterface::CallResult call();
  virtual bool ready();
  bool full();

private:
  bool fullNoLock();
  std::string topic_;
  int32_t size_;
  bool full_;

  std::mutex queue_mutex_;
  D_Item queue_;
  uint32_t queue_size_;
  bool allow_concurrent_callbacks_;

  std::recursive_mutex callback_mutex_;
};

}

#endif // MINIROS_SUBSCRIPTION_QUEUE_H
