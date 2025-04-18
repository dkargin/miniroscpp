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

#ifndef ROSBAG_RECORDER_H
#define ROSBAG_RECORDER_H

#include <sys/stat.h>
#if !defined(_WIN32)
  #include <termios.h>
  #include <unistd.h>
#endif
#include <time.h>

#include <queue>
#include <string>
#include <vector>
#include <list>

#include <condition_variable>
#include <mutex>
#include <regex>

#include <miniros/ros.h>
#include <miniros/rostime.h>

#include <std_msgs/Empty.hxx>
#include <std_msgs/String.hxx>

#include <miniros/topic_tools/shape_shifter.h>

#include "minibag/bag.h"
#include "minibag/stream.h"
#include "minibag/macros.h"


namespace minibag {

class OutgoingMessage
{
public:
    OutgoingMessage(std::string const& _topic, miniros::topic_tools::ShapeShifter::ConstPtr _msg, std::shared_ptr<miniros::M_string> _connection_header, miniros::Time _time);

    std::string                         topic;
    miniros::topic_tools::ShapeShifter::ConstPtr msg;
    std::shared_ptr<miniros::M_string>    connection_header;
    miniros::Time                           time;
};

class OutgoingQueue
{
public:
    OutgoingQueue(std::string const& _filename, std::queue<OutgoingMessage>* _queue, miniros::Time _time);

    std::string                  filename;
    std::queue<OutgoingMessage>* queue;
    miniros::Time                    time;
};

struct RecorderOptions
{
    RecorderOptions();

    bool            trigger;
    bool            record_all;
    bool            regex;
    bool            do_exclude;
    bool            quiet;
    bool            append_date;
    bool            snapshot;
    bool            verbose;
    bool            publish;
    CompressionType compression;
    std::string     prefix;
    std::string     name;
    std::regex    exclude_regex;
    uint32_t        buffer_size;
    uint32_t        chunk_size;
    uint32_t        limit;
    bool            split;
    uint64_t        max_size;
    uint32_t        max_splits;
    miniros::Duration   max_duration;
    std::string     node;
    unsigned long long min_space;
    std::string min_space_str;
    miniros::TransportHints transport_hints;

    std::vector<std::string> topics;
};

class Recorder
{
public:
    Recorder(RecorderOptions const& options);

    void doTrigger();

    bool isSubscribed(std::string const& topic) const;

    std::shared_ptr<miniros::Subscriber> subscribe(std::string const& topic);

    int run();

private:
    void printUsage();

    void updateFilenames();
    void startWriting();
    void stopWriting();

    bool checkLogging();
    bool scheduledCheckDisk();
    bool checkDisk();

    void snapshotTrigger(std_msgs::Empty::ConstPtr trigger);
    void doQueue(
        const miniros::MessageEvent<miniros::topic_tools::ShapeShifter const>& msg_event,
        std::string const& topic,
        std::shared_ptr<miniros::Subscriber> subscriber,
        std::shared_ptr<int> count);
    void doRecord();
    void checkNumSplits();
    bool checkSize();
    bool checkDuration(const miniros::Time&);
    void doRecordSnapshotter();
    bool doCheckMaster(miniros::TimerEvent const& e, miniros::NodeHandle& node_handle);

    bool shouldSubscribeToTopic(std::string const& topic, bool from_node = false);

    template<class T>
    static std::string timeToStr(T MINIROS_t);

private:
    RecorderOptions               options_;

    Bag                           bag_;

    std::string                   target_filename_;
    std::string                   write_filename_;
    std::list<std::string>        current_files_;

    std::set<std::string>         currently_recording_;  //!< set of currenly recording topics
    int                           num_subscribers_;      //!< used for book-keeping of our number of subscribers

    int                           exit_code_;            //!< eventual exit code

    std::condition_variable_any queue_condition_;      //!< conditional variable for queue
    std::mutex                  queue_mutex_;          //!< mutex for queue
    std::queue<OutgoingMessage>*  queue_;                //!< queue for storing
    uint64_t                      queue_size_;           //!< queue size
    uint64_t                      max_queue_size_;       //!< max queue size

    uint64_t                      split_count_;          //!< split count

    std::queue<OutgoingQueue>     queue_queue_;          //!< queue of queues to be used by the snapshot recorders

    miniros::Time                     last_buffer_warn_;

    miniros::Time                     start_time_;

    bool                          writing_enabled_;
    std::mutex                    check_disk_mutex_;
    miniros::WallTime                 check_disk_next_;
    miniros::WallTime                 warn_next_;

    miniros::Publisher                pub_begin_write;
};

} // namespace minibag

#endif
