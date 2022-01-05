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

#ifndef ROSCPP_TOPIC_H
#define ROSCPP_TOPIC_H

#include "common.h"
#include "node_handle.h"

namespace miniros
{
namespace topic
{

/**
 * \brief Internal method, do not use
 */
MINIROS_DECL void waitForMessageImpl(SubscribeOptions& ops, const std::function<bool(void)>& ready_pred, NodeHandle& nh, miniros::Duration timeout);

template<class M>
class SubscribeHelper
{
public:
  typedef std::shared_ptr<M const> MConstPtr;
  void callback(const MConstPtr& message)
  {
    message_ = message;
  }

  bool hasMessage()
  {
    return static_cast<bool>(message_);
  }

  MConstPtr getMessage()
  {
    return message_;
  }

private:
  MConstPtr message_;
};

/**
 * \brief Wait for a single message to arrive on a topic, with timeout
 *
 * \param M <template> The message type
 * \param topic The topic to subscribe on
 * \param nh The NodeHandle to use to do the subscription
 * \param timeout The amount of time to wait before returning if no message is received
 * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the node shutting down
 */
template<class M>
std::shared_ptr<M const> waitForMessage(const std::string& topic, NodeHandle& nh, miniros::Duration timeout)
{
  SubscribeHelper<M> helper;
  SubscribeOptions ops;
  using MConstPtr = typename SubscribeHelper<M>::MConstPtr;
  auto wrapCallback =
    [&helper](const MConstPtr& msg)
    {
      helper.callback(msg);
    };
  ops.template init<M>(topic, 1, wrapCallback);

  auto wrapHasMessage =
    [&helper]() -> bool
    {
      return helper.hasMessage();
    };
  waitForMessageImpl(ops, wrapHasMessage, nh, timeout);

  return helper.getMessage();
}

/**
 * \brief Wait for a single message to arrive on a topic
 *
 * \param M <template> The message type
 * \param topic The topic to subscribe on
 * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the node shutting down
 */
template<class M>
std::shared_ptr<M const> waitForMessage(const std::string& topic)
{
  miniros::NodeHandle nh;
  return waitForMessage<M>(topic, nh, miniros::Duration());
}

/**
 * \brief Wait for a single message to arrive on a topic, with timeout
 *
 * \param M <template> The message type
 * \param topic The topic to subscribe on
 * \param timeout The amount of time to wait before returning if no message is received
 * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the node shutting down
 */
template<class M>
std::shared_ptr<M const> waitForMessage(const std::string& topic, miniros::Duration timeout)
{
  miniros::NodeHandle nh;
  return waitForMessage<M>(topic, nh, timeout);
}

/**
 * \brief Wait for a single message to arrive on a topic
 *
 * \param M <template> The message type
 * \param topic The topic to subscribe on
 * \param nh The NodeHandle to use to do the subscription
 * \return The message.  Empty std::shared_ptr if waitForMessage is interrupted by the node shutting down
 */
template<class M>
std::shared_ptr<M const> waitForMessage(const std::string& topic, miniros::NodeHandle& nh)
{
  return waitForMessage<M>(topic, nh, miniros::Duration());
}

} // namespace topic
} // namespace miniros

#endif // ROSCPP_TOPIC_H
