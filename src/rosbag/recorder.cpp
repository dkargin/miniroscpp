/*********************************************************************
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
 ********************************************************************/

#include <algorithm>
#include <filesystem>
#include <sys/stat.h>

#include <ctime>
#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>

#include <regex>
#include "minibag/recorder.h"

#include <miniros/topic_tools/shape_shifter.h>

#include "miniros/network/network.h"
#include "miniros/transport/rpc_manager.h"

using std::cout;
using std::endl;
using std::set;
using std::shared_ptr;
using std::string;
using std::vector;

namespace minibag {

// OutgoingMessage

OutgoingMessage::OutgoingMessage(string const& _topic, miniros::topic_tools::ShapeShifter::ConstPtr _msg,
  shared_ptr<miniros::M_string> _connection_header, miniros::Time _time)
    : topic(_topic), msg(_msg), connection_header(_connection_header), time(_time)
{
}

// OutgoingQueue

OutgoingQueue::OutgoingQueue(string const& _filename, std::queue<OutgoingMessage>* _queue, miniros::Time _time)
    : filename(_filename), queue(_queue), time(_time)
{
}

// RecorderOptions

RecorderOptions::RecorderOptions()
    : trigger(false), record_all(false), regex(false), do_exclude(false), quiet(false), append_date(true),
      snapshot(false), verbose(false), publish(false), compression(compression::Uncompressed), exclude_regex(),
      buffer_size(1048576 * 256), chunk_size(1024 * 768), limit(0), split(false), max_size(0), max_splits(0),
      max_duration(-1.0), min_space(1024 * 1024 * 1024), min_space_str("1G")
{
}

// Recorder

Recorder::Recorder(RecorderOptions const& options)
    : options_(options), num_subscribers_(0), exit_code_(0), queue_size_(0), split_count_(0), writing_enabled_(true)
{
}

int Recorder::run()
{
  if (options_.trigger) {
    doTrigger();
    return 0;
  }

  if (options_.topics.empty()) {
    // Make sure limit is not specified with automatic topic subscription
    if (options_.limit > 0) {
      fprintf(stderr, "Specifying a count is not valid with automatic topic subscription.\n");
      return 1;
    }

    // Make sure topics are specified
    if (!options_.record_all && (options_.node == std::string(""))) {
      fprintf(stderr, "No topics specified.\n");
      return 1;
    }
  }

  miniros::NodeHandle nh;
  if (!nh.ok())
    return 0;

  if (options_.publish) {
    pub_begin_write = nh.advertise<std_msgs::String>("begin_write", 1, true);
  }

  last_buffer_warn_ = miniros::Time();
  queue_ = new std::queue<OutgoingMessage>;

  // Subscribe to each topic
  if (!options_.regex) {
    for (string const& topic : options_.topics)
      subscribe(topic);
  }

  if (!miniros::Time::waitForValid(miniros::WallDuration(2.0)))
    MINIROS_WARN("/use_sim_time set to true and no clock published.  Still waiting for valid time...");

  miniros::Time::waitForValid();

  start_time_ = miniros::Time::now();

  // Don't bother doing anything if we never got a valid time
  if (!nh.ok())
    return 0;

  miniros::Subscriber trigger_sub;

  // Spin up a thread for writing to the file
  std::thread record_thread;
  if (options_.snapshot) {
    record_thread = std::thread(&Recorder::doRecordSnapshotter, this);

    // Subscribe to the snapshot trigger
    trigger_sub = nh.subscribe<std_msgs::Empty>(
      "snapshot_trigger", 100, [this](std_msgs::Empty::ConstPtr trigger) { this->snapshotTrigger(trigger); });
  } else
    record_thread = std::thread(&Recorder::doRecord, this);

  miniros::Timer check_master_timer;
  if (options_.record_all || options_.regex || (options_.node != std::string(""))) {
    // check for master first
    doCheckMaster(miniros::TimerEvent(), nh);
    check_master_timer =
      nh.createTimer(miniros::Duration(1.0), [this, &nh](miniros::TimerEvent const& e) { this->doCheckMaster(e, nh); });
  }

  miniros::AsyncSpinner s(10);
  s.start();

  record_thread.join();
  queue_condition_.notify_all();
  delete queue_;

  return exit_code_;
}

shared_ptr<miniros::Subscriber> Recorder::subscribe(string const& topic)
{
  MINIROS_INFO("Subscribing to %s", topic.c_str());

  miniros::NodeHandle nh;
  shared_ptr<int> count(std::make_shared<int>(options_.limit));
  shared_ptr<miniros::Subscriber> sub(std::make_shared<miniros::Subscriber>());

  miniros::SubscribeOptions ops;
  ops.topic = topic;
  ops.queue_size = 100;
  ops.md5sum = miniros::message_traits::md5sum<miniros::topic_tools::ShapeShifter>();
  ops.datatype = miniros::message_traits::datatype<miniros::topic_tools::ShapeShifter>();
  ops.helper = std::make_shared<
    miniros::SubscriptionCallbackHelperT<const miniros::MessageEvent<miniros::topic_tools::ShapeShifter const>&>>(
    [this, topic, sub, count](const miniros::MessageEvent<miniros::topic_tools::ShapeShifter const>& mev) {
      this->doQueue(mev, topic, sub, count);
    });
  ops.transport_hints = options_.transport_hints;
  *sub = nh.subscribe(ops);

  currently_recording_.insert(topic);
  num_subscribers_++;

  return sub;
}

bool Recorder::isSubscribed(string const& topic) const
{
  return currently_recording_.find(topic) != currently_recording_.end();
}

bool Recorder::shouldSubscribeToTopic(std::string const& topic, bool from_node)
{
  // ignore already known topics
  if (isSubscribed(topic)) {
    return false;
  }

  // subtract exclusion regex, if any
  if (options_.do_exclude && std::regex_match(topic, options_.exclude_regex)) {
    return false;
  }

  if (options_.record_all || from_node) {
    return true;
  }

  if (options_.regex) {
    // Treat the topics as regular expressions
    return std::any_of(std::begin(options_.topics), std::end(options_.topics), [&topic](string const& regex_str) {
      std::regex re(regex_str);
      return std::regex_match(topic, re);
    });
  }

  return std::find(std::begin(options_.topics), std::end(options_.topics), topic) != std::end(options_.topics);
}

template <class T> std::string Recorder::timeToStr(T ros_t)
{
  (void)ros_t;

  char timeString[1024];
  
  // Example of the very popular RFC 3339 format UTC time
  std::time_t time = std::time({});
  std::strftime(timeString, sizeof(timeString), "%Y-%m-%d-%H-%M-%S", std::gmtime(&time));
  
  return std::string(timeString);
}

//! Callback to be invoked to save messages into a queue
void Recorder::doQueue(const miniros::MessageEvent<miniros::topic_tools::ShapeShifter const>& msg_event,
  string const& topic, shared_ptr<miniros::Subscriber> subscriber, shared_ptr<int> count)
{
  miniros::Time rectime = miniros::Time::now();

  if (options_.verbose)
    cout << "Received message on topic " << subscriber->getTopic() << endl;

  OutgoingMessage out(topic, msg_event.getMessage(), msg_event.getConnectionHeaderPtr(), rectime);

  {
    std::scoped_lock<std::mutex> lock(queue_mutex_);

    queue_->push(out);
    queue_size_ += out.msg->size();

    // Check to see if buffer has been exceeded
    while (options_.buffer_size > 0 && queue_size_ > options_.buffer_size) {
      OutgoingMessage drop = queue_->front();
      queue_->pop();
      queue_size_ -= drop.msg->size();

      if (!options_.snapshot) {
        miniros::Time now = miniros::Time::now();
        if (now > last_buffer_warn_ + miniros::Duration(5.0)) {
          MINIROS_WARN("rosbag record buffer exceeded.  Dropping oldest queued message.");
          last_buffer_warn_ = now;
        }
      }
    }
  }

  if (!options_.snapshot)
    queue_condition_.notify_all();

  // If we are book-keeping count, decrement and possibly shutdown
  if ((*count) > 0) {
    (*count)--;
    if ((*count) == 0) {
      subscriber->shutdown();

      num_subscribers_--;

      if (num_subscribers_ == 0)
        miniros::shutdown();
    }
  }
}

void Recorder::updateFilenames()
{
  vector<string> parts;

  std::string prefix = options_.prefix;
  size_t ind = prefix.rfind(".bag");

  if (ind != std::string::npos && ind == prefix.size() - 4) {
    prefix.erase(ind);
  }

  if (prefix.length() > 0)
    parts.push_back(prefix);
  if (options_.append_date)
    parts.push_back(timeToStr(miniros::WallTime::now()));
  if (options_.split)
    parts.push_back(std::to_string(split_count_));

  if (parts.size() == 0) {
    throw BagException("Bag filename is empty (neither of these was specified: prefix, append_date, split)");
  }

  target_filename_ = parts[0];
  for (unsigned int i = 1; i < parts.size(); i++)
    target_filename_ += string("_") + parts[i];

  target_filename_ += string(".bag");
  write_filename_ = target_filename_ + string(".active");
}

//! Callback to be invoked to actually do the recording
void Recorder::snapshotTrigger(std_msgs::Empty::ConstPtr trigger)
{
  (void)trigger;
  updateFilenames();

  MINIROS_INFO("Triggered snapshot recording with name %s.", target_filename_.c_str());

  {
    std::scoped_lock<std::mutex> lock(queue_mutex_);
    queue_queue_.push(OutgoingQueue(target_filename_, queue_, miniros::Time::now()));
    queue_ = new std::queue<OutgoingMessage>;
    queue_size_ = 0;
  }

  queue_condition_.notify_all();
}

void Recorder::startWriting()
{
  bag_.setCompression(options_.compression);
  bag_.setChunkThreshold(options_.chunk_size);

  updateFilenames();
  try {
    bag_.open(write_filename_, bagmode::Write);
  } catch (minibag::BagException e) {
    MINIROS_ERROR("Error writing: %s", e.what());
    exit_code_ = 1;
    miniros::shutdown();
  }
  MINIROS_INFO("Recording to %s.", target_filename_.c_str());

  if (options_.publish) {
    std_msgs::String msg;
    msg.data = target_filename_.c_str();
    pub_begin_write.publish(msg);
  }
}

void Recorder::stopWriting()
{
  MINIROS_INFO("Closing %s.", target_filename_.c_str());
  bag_.close();
  rename(write_filename_.c_str(), target_filename_.c_str());
}

void Recorder::checkNumSplits()
{
  if (options_.max_splits > 0) {
    current_files_.push_back(target_filename_);
    if (current_files_.size() > options_.max_splits) {
      int err = unlink(current_files_.front().c_str());
      if (err != 0) {
        MINIROS_ERROR("Unable to remove %s: %s", current_files_.front().c_str(), strerror(errno));
      }
      current_files_.pop_front();
    }
  }
}

bool Recorder::checkSize()
{
  if (options_.max_size > 0) {
    if (bag_.getSize() > options_.max_size) {
      if (options_.split) {
        stopWriting();
        split_count_++;
        checkNumSplits();
        startWriting();
      } else {
        miniros::shutdown();
        return true;
      }
    }
  }
  return false;
}

bool Recorder::checkDuration(const miniros::Time& t)
{
  if (options_.max_duration > miniros::Duration(0)) {
    if (t - start_time_ > options_.max_duration) {
      if (options_.split) {
        while (start_time_ + options_.max_duration < t) {
          stopWriting();
          split_count_++;
          checkNumSplits();
          start_time_ += options_.max_duration;
          startWriting();
        }
      } else {
        miniros::shutdown();
        return true;
      }
    }
  }
  return false;
}

//! Thread that actually does writing to file.
void Recorder::doRecord()
{
  // Open bag file for writing
  startWriting();

  // Schedule the disk space check
  warn_next_ = miniros::WallTime();

  try {
    checkDisk();
  } catch (minibag::BagException& ex) {
    MINIROS_ERROR("BagException: %s", ex.what());
    exit_code_ = 1;
    stopWriting();
    return;
  }

  check_disk_next_ = miniros::WallTime::now() + miniros::WallDuration().fromSec(20.0);

  // Technically the queue_mutex_ should be locked while checking empty.
  // Except it should only get checked if the node is not ok, and thus
  // it shouldn't be in contention.
  miniros::NodeHandle nh;
  while (nh.ok() || !queue_->empty()) {
    std::unique_lock<std::mutex> lock(queue_mutex_);

    bool finished = false;
    while (queue_->empty()) {
      if (!nh.ok()) {
        lock.release()->unlock();
        finished = true;
        break;
      }
      std::chrono::duration dur = std::chrono::milliseconds(250);
      queue_condition_.wait_for(lock, dur);
      if (checkDuration(miniros::Time::now())) {
        finished = true;
        break;
      }
    }
    if (finished)
      break;

    OutgoingMessage out = queue_->front();
    queue_->pop();
    queue_size_ -= out.msg->size();

    lock.release()->unlock();

    if (checkSize())
      break;

    if (checkDuration(out.time))
      break;

    try {
      if (scheduledCheckDisk() && checkLogging())
        bag_.write(out.topic, out.time, *out.msg, out.connection_header);
    } catch (minibag::BagException& ex) {
      MINIROS_ERROR("BagException: %s", ex.what());
      exit_code_ = 1;
      break;
    }
  }

  stopWriting();
}

void Recorder::doRecordSnapshotter()
{
  miniros::NodeHandle nh;

  while (nh.ok() || !queue_queue_.empty()) {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    while (queue_queue_.empty()) {
      if (!nh.ok())
        return;
      queue_condition_.wait(lock);
    }

    OutgoingQueue out_queue = queue_queue_.front();
    queue_queue_.pop();

    lock.release()->unlock();

    string target_filename = out_queue.filename;
    string write_filename = target_filename + string(".active");

    try {
      bag_.open(write_filename, bagmode::Write);
    } catch (minibag::BagException& ex) {
      MINIROS_ERROR("Error writing: %s", ex.what());
      return;
    }

    while (!out_queue.queue->empty()) {
      OutgoingMessage out = out_queue.queue->front();
      out_queue.queue->pop();

      bag_.write(out.topic, out.time, *out.msg);
    }

    stopWriting();
  }
}

bool Recorder::doCheckMaster(miniros::TimerEvent const& e, miniros::NodeHandle& node_handle)
{
  (void)e;
  (void)node_handle;
  auto master = node_handle.getMasterLink();
  MINIROS_ASSERT(master);
  if (!master)
    return false;
  std::vector<miniros::TopicInfo> topics;
  master->getTopics(topics);
  if (master->getTopics(topics)) {
    for (const auto& t : topics) {
      if (shouldSubscribeToTopic(t.name))
        subscribe(t.name);
    }
  }

  if (options_.node != std::string("")) {
    XmlRpc::XmlRpcValue req;
    req[0] = miniros::this_node::getName();
    req[1] = options_.node;
    XmlRpc::XmlRpcValue resp;
    XmlRpc::XmlRpcValue payload;

    if (master->execute("lookupNode", req, resp, payload, true)) {
      std::string peer_host;
      uint32_t peer_port;

      if (!miniros::network::splitURI(static_cast<std::string>(resp[2]), peer_host, peer_port)) {
        MINIROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
      } else {
        XmlRpc::XmlRpcClient c(peer_host.c_str(), peer_port, "/");
        XmlRpc::XmlRpcValue req2;
        XmlRpc::XmlRpcValue resp2;
        req2[0] = miniros::this_node::getName();
        c.execute("getSubscriptions", req2, resp2);

        if (!c.isFault() && resp2.valid() && resp2.size() > 0 && static_cast<int>(resp2[0]) == 1) {
          for (int i = 0; i < resp2[2].size(); i++) {
            if (shouldSubscribeToTopic(resp2[2][i][0], true))
              subscribe(resp2[2][i][0]);
          }
        } else {
          MINIROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
          return false;
        }
      }
    }
  }
  return true;
}

void Recorder::doTrigger()
{
  miniros::NodeHandle nh;
  miniros::Publisher pub = nh.advertise<std_msgs::Empty>("snapshot_trigger", 1, true);
  pub.publish(std_msgs::Empty());

  miniros::Timer terminate_timer =
    nh.createTimer(miniros::Duration(1.0), [](const miniros::TimerEvent&) { miniros::shutdown(); });
  miniros::spin();
}

bool Recorder::scheduledCheckDisk()
{
  std::scoped_lock<std::mutex> lock(check_disk_mutex_);

  if (miniros::WallTime::now() < check_disk_next_)
    return true;

  check_disk_next_ += miniros::WallDuration().fromSec(20.0);
  return checkDisk();
}

bool Recorder::checkDisk()
{
  std::filesystem::path p(std::filesystem::absolute(bag_.getFileName().c_str()));
  p = p.parent_path();
  std::filesystem::space_info info;
  try {
    info = std::filesystem::space(p);
  } catch (std::filesystem::filesystem_error& e) {
    MINIROS_WARN("Failed to check filesystem stats [%s].", e.what());
    writing_enabled_ = false;
    return false;
  }
  if (info.available < options_.min_space) {
    writing_enabled_ = false;
    throw BagException("Less than " + options_.min_space_str + " of space free on disk with " + bag_.getFileName() +
                       ". Disabling recording.");
  } else if (info.available < 5 * options_.min_space) {
    MINIROS_WARN(
      "Less than 5 x %s of space free on disk with %s.", options_.min_space_str.c_str(), bag_.getFileName().c_str());
    writing_enabled_ = true;
  } else {
    writing_enabled_ = true;
  }
  return true;
}

bool Recorder::checkLogging()
{
  if (writing_enabled_)
    return true;

  miniros::WallTime now = miniros::WallTime::now();
  if (now >= warn_next_) {
    warn_next_ = now + miniros::WallDuration().fromSec(5.0);
    MINIROS_WARN("Not logging message because logging disabled.  Most likely cause is a full disk.");
  }
  return false;
}

} // namespace minibag
