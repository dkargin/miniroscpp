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

#include <algorithm>
#include <vector>
#include <map>
#include <mutex>
#include <optional>

#include "internal/at_exit.h"

#include "miniros/io/poll_set.h"

#include "internal/profiling.h"
#include "miniros/io/io.h"

#include "miniros/internal/local_log.h"

//#define POLL_SET_SERIOUS_LOG

namespace miniros
{

/// Use this flag to subscribe to "input" events. They are equal to POLLIN.
const int PollSet::EventIn = POLLIN;

/// Use this flag to get notified when output is possible. It is equal to POLLOUT.
const int PollSet::EventOut = POLLOUT;

const int PollSet::EventError = POLLERR;

struct PollSet::Internal {
  struct SocketInfo
  {
    TrackedObject object_;
    SocketUpdateFunc func_;
    int fd_;
    int events_;
    bool updateEvents_;
    int timeLeftMs_ = -1;
  };

  std::map<int, SocketInfo> socket_info_;
  std::map<int, SteadyTime> socket_timers_;
  mutable std::mutex socket_info_mutex_;
  bool sockets_changed_ = false;

  std::mutex just_deleted_mutex_;
  std::vector<int> just_deleted_;

  std::vector<socket_pollfd> ufds_;

  /// Storage for results from poll.
  std::vector<socket_pollfd> ofds_;


  std::mutex signal_mutex_;
  signal_fd_t signal_pipe_[2] = {MINIROS_INVALID_SOCKET, MINIROS_INVALID_SOCKET};

  int epfd_;

  Internal()
    :epfd_(create_socket_watcher())
  {

  }

  bool isInternalFd(int fd) const
  {
    return fd == signal_pipe_[0] || fd == signal_pipe_[1];
  }

  /// It returns a copy of SocketInfo to make sure that shared pointer to tracked object having additional reference.
  bool findSocketInfo(int fd, SocketInfo& out) const
  {
    std::scoped_lock<std::mutex> lock(socket_info_mutex_);
    auto it = socket_info_.find(fd);
    // the socket has been entirely deleted
    if (it == socket_info_.end())
      return false;
    out = it->second;
    return true;
  }
};

PollSet::PollSet()
{
  internal_.reset(new Internal());

  if ( create_signal_pair(internal_->signal_pipe_) != 0 ) {
    // Really no idea how to recover from this kind of error.
    perror("PollSet::PollSet - create_signal_pair() failed");
  }
  addSocket(internal_->signal_pipe_[0], POLLIN, [this](int events){return onLocalPipeEvents(events);});
}

PollSet::~PollSet()
{
  close_signal_pair(internal_->signal_pipe_);
  close_socket_watcher(internal_->epfd_);
  internal_.reset();
}

bool PollSet::addSocket(int fd, int events, const SocketUpdateFunc& update_func, const TrackedObject& object)
{
  if (fd < 0)
    return false;

  MINIROS_PROFILE_SCOPE2("PollSet", "addSocket");

  Internal::SocketInfo info;
  info.fd_ = fd;
  if (events & EventUpdate) {
    info.updateEvents_ = true;
    events &= ~EventUpdate;
  } else {
    info.updateEvents_ = false;
  }
  info.events_ = events;
  info.object_ = object;
  info.func_ = update_func;

  {
    std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);

    bool b = internal_->socket_info_.insert(std::make_pair(fd, info)).second;
    if (!b)
    {
      printf("PollSet: Tried to add duplicate fd [%d]", fd);
      return false;
    }

    add_socket_to_watcher(internal_->epfd_, fd, events);

    internal_->sockets_changed_ = true;
  }
  LOCAL_DEBUG("PollSet::addSocket(%d) evt=%s", fd, eventToString(events).c_str());

  signal();

  return true;
}

bool PollSet::delSocket(int fd)
{
  if(fd < 0)
  {
    return false;
  }
  MINIROS_PROFILE_SCOPE2("PollSet", "delSocket");

  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);
  auto it = internal_->socket_info_.find(fd);
  if (it != internal_->socket_info_.end())
  {
    internal_->socket_info_.erase(it);

    {
      std::scoped_lock<std::mutex> lock(internal_->just_deleted_mutex_);
      internal_->just_deleted_.push_back(fd);
    }

    del_socket_from_watcher(internal_->epfd_, fd);

    auto it2 = internal_->socket_timers_.find(fd);
    if (it2 != internal_->socket_timers_.end())
      internal_->socket_timers_.erase(it2);

    internal_->sockets_changed_ = true;
    signal();

    LOCAL_DEBUG("PollSet::delSocket(%d)", fd);
    return true;
  }

  LOCAL_WARN("PollSet: Tried to delete fd [%d] which is not being tracked", fd);

  return false;
}


bool PollSet::addEvents(int sock, int events)
{
  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);

  auto it = internal_->socket_info_.find(sock);

  if (it == internal_->socket_info_.end())
  {
    LOCAL_DEBUG("PollSet: Tried to add events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  it->second.events_ |= events;

  set_events_on_socket(internal_->epfd_, sock, it->second.events_);

  internal_->sockets_changed_ = true;
  signal();

  return true;
}

bool PollSet::delEvents(int sock, int events)
{
  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);

  auto it = internal_->socket_info_.find(sock);
  if (it != internal_->socket_info_.end())
  {
    it->second.events_ &= ~events;
  }
  else
  {
    LOCAL_DEBUG("PollSet: Tried to delete events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  set_events_on_socket(internal_->epfd_, sock, it->second.events_);

  internal_->sockets_changed_ = true;
  signal();

  return true;
}

bool PollSet::setEvents(int sock, int events)
{
  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);

  auto it = internal_->socket_info_.find(sock);

  if (it == internal_->socket_info_.end())
  {
    LOCAL_DEBUG("PollSet: Tried to set events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  if (it->second.events_ != events) {
    it->second.events_ = events;
    set_events_on_socket(internal_->epfd_, sock, it->second.events_);
    internal_->sockets_changed_ = true;
    LOCAL_DEBUG("PollSet::setEvents(%d, %s)", sock, eventToString(events).c_str());
    signal();
  }

  return true;
}

bool PollSet::setTimerEvent(int sock, int timeoutMs)
{
  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);
  auto it = internal_->socket_info_.find(sock);

  if (it == internal_->socket_info_.end())
  {
    LOCAL_DEBUG("PollSet::setTimerEvent(%d) - fd does not exist", sock);
    return false;
  }
  internal_->socket_timers_[sock] = SteadyTime::now() + WallDuration(timeoutMs*0.001);
  signal();
  LOCAL_DEBUG("PollSet::setTimerEvent(%d) timeout=%d", sock, timeoutMs);
  return true;
}

void PollSet::signal()
{
  if (internal_->signal_mutex_.try_lock())
  {
    std::lock_guard<std::mutex> lock(internal_->signal_mutex_, std::adopt_lock);
    char b = 0;
    if (write_signal(internal_->signal_pipe_[1], &b, 1) < 0)
    {
      // do nothing... this prevents warnings on gcc 4.3
    }
  }
}


void PollSet::update(int poll_timeout)
{
  MINIROS_PROFILE_SCOPE2("PollSet", "update");

  static SteadyTime lastUpdateFinish = SteadyTime::now();

  SteadyTime updateStart = SteadyTime::now();
  auto gap = updateStart - lastUpdateFinish;
  createNativePollset();

  AtExit ae([this]() {
    std::scoped_lock<std::mutex> lock(internal_->just_deleted_mutex_);
    internal_->just_deleted_.clear();
  });

  // This function does not lock socket info for most of the time.
  // So it should be expected that configuration of sockets and events can change.
  // delSocket also can be called during poll and its processing.

  const nfds_t numFd = static_cast<nfds_t>(internal_->ufds_.size());
  {
    MINIROS_PROFILE_SCOPE("poll");
    Error err = poll_sockets(internal_->epfd_, &internal_->ufds_.front(), numFd, poll_timeout, internal_->ofds_);
    if (!err)
    {
      LOCAL_ERROR("PollSet::update() poll_sockets failed with error %s", err.toString());
      return;
    }
  }

  for (const socket_pollfd& spfd: internal_->ofds_)
  {
    int fd = spfd.fd;
    int revents = spfd.revents;

    if (revents == 0)
      continue;

    Internal::SocketInfo info;
    if (!internal_->findSocketInfo(fd, info))
      continue;

    // Store off the function and transport in case the socket is deleted from another thread
    SocketUpdateFunc func = info.func_;
    // This pointer helps keeping transport alive until we exit this block.
    TrackedObject object = info.object_;
    const int events = info.events_;

    if (!internal_->isInternalFd(fd)) {
#ifdef POLL_SET_SERIOUS_LOG
      std::cout << "poll fd=" << fd << " evt=" << eventToString(spfd.revents) << std::endl;
#endif
    }
    bool hasEvents = events & revents
            || revents & POLLERR
            || revents & POLLHUP
            || revents & POLLNVAL;
    if (!func) {
#ifdef POLL_SET_SERIOUS_LOG
      std::cerr << "poll no event handler for fd=" << fd << std::endl;
#endif
      continue;
    }
    if (!hasEvents) {
      continue;
    }
    // If these are registered events for this socket, OR the events are ERR/HUP/NVAL,
    // call through to the registered function
    bool skip = false;
    if (revents & (POLLNVAL|POLLERR|POLLHUP))
    {
      // If a socket was just closed and then the file descriptor immediately reused, we can
      // get in here with what we think is a valid socket (since it was just re-added to our set)
      // but which is actually referring to the previous fd with the same #.  If this is the case,
      // we ignore the first instance of one of these errors.  If it's a real error we'll
      // hit it again next time through.
      std::scoped_lock<std::mutex> lock(internal_->just_deleted_mutex_);
      auto it = std::find(internal_->just_deleted_.begin(), internal_->just_deleted_.end(), fd);
      if (it != internal_->just_deleted_.end())
        skip = true;
    }
    if (skip)
      continue;

    int ret = func(revents & (events|POLLERR|POLLHUP|POLLNVAL));
    if (ret & ResultDropFD) {
      delSocket(fd);
    }
    else if (info.updateEvents_ && ret != info.events_) {
      std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);
      auto it = internal_->socket_info_.find(fd);
      if (it != internal_->socket_info_.end() && it->second.events_ != ret) {
        it->second.events_ = ret;
        set_events_on_socket(internal_->epfd_, fd, it->second.events_);
        LOCAL_DEBUG("PollSet fd=%d adjust events from %s to %s", fd, eventToString(info.events_).c_str(), eventToString(ret).c_str());
      }
    }
  } // for socket event

  processTimers();

  SteadyTime updateDone = SteadyTime::now();
  auto dur = updateDone - updateStart;
  //LOCAL_DEBUG("PollSet processed in %fs, gap=%fs", dur.toSec(), gap.toSec());
  lastUpdateFinish = updateDone;
}

void PollSet::processTimers()
{
  MINIROS_PROFILE_SCOPE2("PollSet", "processTimers");
  // Fire timer events.
  SteadyTime now = SteadyTime::now();
  std::vector<int> expiredTimers;

  {
    std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);
    // Gather all timer events that fired.
    for (auto [fd, stamp]: internal_->socket_timers_) {
      if (stamp <= now) {
        expiredTimers.push_back(fd);
      }
    }
    for (auto fd: expiredTimers) {
      internal_->socket_timers_.erase(fd);
    }
  }

  for (auto fd: expiredTimers) {
    Internal::SocketInfo info;
    if (!internal_->findSocketInfo(fd, info))
      continue;
    if (!info.func_)
      continue;
    (void)info.func_(EventTimer);
  }
}


void PollSet::createNativePollset()
{
  std::scoped_lock<std::mutex> lock(internal_->socket_info_mutex_);

  if (!internal_->sockets_changed_)
  {
    return;
  }

#ifdef POLL_SET_SERIOUS_LOG
  std::stringstream ss;
  ss << "poll updating PollSet ";
#endif
  // Build the list of structures to pass to poll for the sockets we're servicing
  internal_->ufds_.resize(internal_->socket_info_.size());
  auto sock_it = internal_->socket_info_.begin();
  auto sock_end = internal_->socket_info_.end();
  for (int i = 0; sock_it != sock_end; ++sock_it, ++i)
  {
    const Internal::SocketInfo& info = sock_it->second;
    socket_pollfd& pfd = internal_->ufds_[i];
    pfd.fd = info.fd_;
    pfd.events = info.events_;
    pfd.revents = 0;
#ifdef POLL_SET_SERIOUS_LOG
    ss << info.fd_ << ":" << eventToString(info.events_) << " ";
#endif
  }
#ifdef POLL_SET_SERIOUS_LOG
  std::cout << ss.str() << std::endl;
#endif
  internal_->sockets_changed_ = false;
}

int PollSet::onLocalPipeEvents(int events)
{
  if(events & POLLIN)
  {
    char b;
    while(read_signal(internal_->signal_pipe_[0], &b, 1) > 0)
    {
      //do nothing keep draining
    };
  }
  return 0;
}

std::string PollSet::eventToString(int event)
{
  std::string result;
#define EVT_FLAG(flag, ch) if (event & flag) { result += ch; event &= ~flag; }
  EVT_FLAG(POLLIN, 'I');
  EVT_FLAG(POLLOUT, 'O');
  EVT_FLAG(POLLERR, 'E');
  EVT_FLAG(POLLHUP, 'H');

#ifdef POLLMSG
  EVT_FLAG(POLLMSG, 'M');
#endif
#ifdef POLLREMOVE
  EVT_FLAG(POLLREMOVE, "Rm");
#endif
#ifdef POLLRDHUP
  EVT_FLAG(POLLRDHUP, "Rdh");
#endif
  if (event != 0) {
    result += "_";
    result += std::to_string(event);
  }
#undef EVT_FLAG
  return result;
}


}
