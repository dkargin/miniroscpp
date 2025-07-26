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

#ifndef MINIROS_ROSOUT_APPENDER_H
#define MINIROS_ROSOUT_APPENDER_H

#include <miniros/message_forward.h>
#include "miniros/common.h"

#include <mutex>
#include <condition_variable>
#include <thread>

namespace rosgraph_msgs
{
MINIROS_DECLARE_MESSAGE(Log)
}

namespace miniros
{

class MasterLink;
using MasterLinkPtr = std::shared_ptr<MasterLink>;

class TopicManager;
using TopicManagerPtr = std::shared_ptr<TopicManager>;

class MINIROS_DECL ROSOutAppender : public console::LogAppender
{
public:
  explicit ROSOutAppender(const TopicManagerPtr& tm);
  ~ROSOutAppender() override;

  /// Initialize appender.
  /// It tries to advertise /rosout topic.
  Error init();

  const std::string& getLastError() const;

  void log(console::Level level, const char* str, const char* file, const char* function, int line) override;

protected:
  void logThread();

  std::string last_error_;

  typedef std::vector<rosgraph_msgs::LogPtr> V_Log;
  V_Log log_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_condition_;
  bool shutting_down_;
  bool disable_topics_;

  std::thread publish_thread_;
  TopicManagerPtr topic_manager_;
};

} // namespace miniros

#endif
