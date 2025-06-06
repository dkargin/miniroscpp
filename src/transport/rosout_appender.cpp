/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "miniros/transport/rosout_appender.h"
#include "miniros/this_node.h"
#include "miniros/node_handle.h"
#include "miniros/master_link.h"
#include "miniros/transport/topic_manager.h"
#include "miniros/transport/advertise_options.h"
#include "miniros/names.h"

#include <rosgraph_msgs/Log.hxx>

namespace miniros
{

ROSOutAppender::ROSOutAppender(const TopicManagerPtr& tm)
: shutting_down_(false)
, disable_topics_(false)
, publish_thread_(&ROSOutAppender::logThread, this)
, topic_manager_(tm)
{

}

ROSOutAppender::~ROSOutAppender()
{
  shutting_down_ = true;

  {
    std::scoped_lock<std::mutex> lock(queue_mutex_);
    queue_condition_.notify_all();
  }

  publish_thread_.join();
}

Error ROSOutAppender::init()
{
  if (!topic_manager_) {
    return Error::InternalError;
  }
  AdvertiseOptions ops;
  ops.init<rosgraph_msgs::Log>(names::resolve("/rosout"), 0);
  ops.latch = true;
  SubscriberCallbacksPtr cbs(std::make_shared<SubscriberCallbacks>());
  if (!topic_manager_->advertise(ops, cbs)) {
    MINIROS_ERROR("Failed to advertise /rosout");
    return Error::InvalidResponse;
  }
  return Error::Ok;
}


const std::string&  ROSOutAppender::getLastError() const
{
  return last_error_;
}

void ROSOutAppender::log(::miniros::console::Level level, const char* str, const char* file, const char* function, int line)
{
  rosgraph_msgs::LogPtr msg(std::make_shared<rosgraph_msgs::Log>());

  msg->header.stamp = miniros::Time::now();
  if (level == miniros::console::Level::Debug)
  {
    msg->level = rosgraph_msgs::Log::DEBUG;
  }
  else if (level == miniros::console::Level::Info)
  {
    msg->level = rosgraph_msgs::Log::INFO;
  }
  else if (level == miniros::console::Level::Warn)
  {
    msg->level = rosgraph_msgs::Log::WARN;
  }
  else if (level == miniros::console::Level::Error)
  {
    msg->level = rosgraph_msgs::Log::ERROR;
  }
  else if (level == miniros::console::Level::Fatal)
  {
    msg->level = rosgraph_msgs::Log::FATAL;
  }
  msg->name = this_node::getName();
  msg->msg = str;
  msg->file = file;
  msg->function = function;
  msg->line = line;

  MasterLinkPtr master = topic_manager_ ? topic_manager_->getMasterLink() : MasterLinkPtr();
  // check parameter server/cache for omit_topics flag
  // the same parameter is checked in rosout.py for the same purpose
  if (master)
    master->getCached("/rosout_disable_topics_generation", disable_topics_);

  if (!disable_topics_) {
    this_node::getAdvertisedTopics(msg->topics);
  }

  if (level == ::miniros::console::Level::Fatal || level == ::miniros::console::Level::Error)
  {
    last_error_ = str;
  }

  std::scoped_lock<std::mutex> lock(queue_mutex_);
  log_queue_.push_back(msg);
  queue_condition_.notify_all();
}

void ROSOutAppender::logThread()
{
  setThreadName("ROSOutAppender");
  while (!shutting_down_)
  {
    V_Log local_queue;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);

      if (shutting_down_)
      {
        break;
      }

      queue_condition_.wait(lock);

      if (shutting_down_)
      {
        break;
      }

      local_queue.swap(log_queue_);
    }

    if (topic_manager_) {
      for (const auto& msg: local_queue)
      {
        topic_manager_->publish(names::resolve("/rosout"), *msg);
      }
    }
  }
}

} // namespace miniros
