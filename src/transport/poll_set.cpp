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

#define MINIROS_PACKAGE_NAME "poll_set"

#include <algorithm>
#include <fcntl.h>

#include "miniros/transport/poll_set.h"
#include "miniros/file_log.h"

#include "miniros/transport/transport.h"

#include <miniros/rosassert.h>


namespace miniros
{

PollSet::PollSet()
    : sockets_changed_(false), epfd_(create_socket_watcher())
{
  if ( create_signal_pair(signal_pipe_) != 0 ) {
    MINIROS_FATAL("create_signal_pair() failed");
    MINIROS_BREAK();
  }
  addSocket(signal_pipe_[0], [this](int events){this->onLocalPipeEvents(events);});
  addEvents(signal_pipe_[0], POLLIN);
}

PollSet::~PollSet()
{
  close_signal_pair(signal_pipe_);
  close_socket_watcher(epfd_);
}

bool PollSet::addSocket(int fd, const SocketUpdateFunc& update_func, const TransportPtr& transport)
{
  SocketInfo info;
  info.fd_ = fd;
  info.events_ = 0;
  info.transport_ = transport;
  info.func_ = update_func;

  {
    std::scoped_lock<std::mutex> lock(socket_info_mutex_);

    bool b = socket_info_.insert(std::make_pair(fd, info)).second;
    if (!b)
    {
      MINIROS_DEBUG("PollSet: Tried to add duplicate fd [%d]", fd);
      return false;
    }

    add_socket_to_watcher(epfd_, fd);

    sockets_changed_ = true;
  }

  signal();

  return true;
}

bool PollSet::delSocket(int fd)
{
  if(fd < 0)
  {
    return false;
  }

  std::scoped_lock<std::mutex> lock(socket_info_mutex_);
  M_SocketInfo::iterator it = socket_info_.find(fd);
  if (it != socket_info_.end())
  {
    socket_info_.erase(it);

    {
      std::scoped_lock<std::mutex> lock(just_deleted_mutex_);
      just_deleted_.push_back(fd);
    }

    del_socket_from_watcher(epfd_, fd);

    sockets_changed_ = true;
    signal();

    return true;
  }

  MINIROS_DEBUG("PollSet: Tried to delete fd [%d] which is not being tracked", fd);

  return false;
}


bool PollSet::addEvents(int sock, int events)
{
  std::scoped_lock<std::mutex> lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);

  if (it == socket_info_.end())
  {
    MINIROS_DEBUG("PollSet: Tried to add events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  it->second.events_ |= events;

  set_events_on_socket(epfd_, sock, it->second.events_);

  sockets_changed_ = true;
  signal();

  return true;
}

bool PollSet::delEvents(int sock, int events)
{
  std::scoped_lock<std::mutex> lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);
  if (it != socket_info_.end())
  {
    it->second.events_ &= ~events;
  }
  else
  {
    MINIROS_DEBUG("PollSet: Tried to delete events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  set_events_on_socket(epfd_, sock, it->second.events_);

  sockets_changed_ = true;
  signal();

  return true;
}

void PollSet::signal()
{
  if (signal_mutex_.try_lock())
  {
    std::lock_guard<std::mutex> lock(signal_mutex_, std::adopt_lock);
    char b = 0;
    if (write_signal(signal_pipe_[1], &b, 1) < 0)
    {
      // do nothing... this prevents warnings on gcc 4.3
    }
  }
}


void PollSet::update(int poll_timeout)
{
  createNativePollset();

  // Poll across the sockets we're servicing
  std::shared_ptr<std::vector<socket_pollfd> > ofds = poll_sockets(epfd_, &ufds_.front(), ufds_.size(), poll_timeout);
  if (!ofds)
  {
    MINIROS_ERROR("poll failed with error %s", last_socket_error_string());
  }
  else
  {
    for (std::vector<socket_pollfd>::iterator it = ofds->begin() ; it != ofds->end(); ++it)
    {
      int fd = it->fd;
      int revents = it->revents;
      SocketUpdateFunc func;
      TransportPtr transport;
      int events = 0;

      if (revents == 0)
      {
        continue;
      }
      {
        std::scoped_lock<std::mutex> lock(socket_info_mutex_);
        M_SocketInfo::iterator it = socket_info_.find(fd);
        // the socket has been entirely deleted
        if (it == socket_info_.end())
        {
          continue;
        }

        const SocketInfo& info = it->second;

        // Store off the function and transport in case the socket is deleted from another thread
        func = info.func_;
        transport = info.transport_;
        events = info.events_;
      }

      // If these are registered events for this socket, OR the events are ERR/HUP/NVAL,
      // call through to the registered function
      if (func
          && ((events & revents)
              || (revents & POLLERR)
              || (revents & POLLHUP)
              || (revents & POLLNVAL)))
      {
        bool skip = false;
        if (revents & (POLLNVAL|POLLERR|POLLHUP))
        {
          // If a socket was just closed and then the file descriptor immediately reused, we can
          // get in here with what we think is a valid socket (since it was just re-added to our set)
          // but which is actually referring to the previous fd with the same #.  If this is the case,
          // we ignore the first instance of one of these errors.  If it's a real error we'll
          // hit it again next time through.
          std::scoped_lock<std::mutex> lock(just_deleted_mutex_);
          if (std::find(just_deleted_.begin(), just_deleted_.end(), fd) != just_deleted_.end())
          {
            skip = true;
          }
        }

        if (!skip)
        {
          func(revents & (events|POLLERR|POLLHUP|POLLNVAL));
        }
      }
    }
  }

  std::scoped_lock<std::mutex> lock(just_deleted_mutex_);
  just_deleted_.clear();

}

void PollSet::createNativePollset()
{
  std::scoped_lock<std::mutex> lock(socket_info_mutex_);

  if (!sockets_changed_)
  {
    return;
  }

  // Build the list of structures to pass to poll for the sockets we're servicing
  ufds_.resize(socket_info_.size());
  M_SocketInfo::iterator sock_it = socket_info_.begin();
  M_SocketInfo::iterator sock_end = socket_info_.end();
  for (int i = 0; sock_it != sock_end; ++sock_it, ++i)
  {
    const SocketInfo& info = sock_it->second;
    socket_pollfd& pfd = ufds_[i];
    pfd.fd = info.fd_;
    pfd.events = info.events_;
    pfd.revents = 0;
  }
  sockets_changed_ = false;
}

void PollSet::onLocalPipeEvents(int events)
{
  if(events & POLLIN)
  {
    char b;
    while(read_signal(signal_pipe_[0], &b, 1) > 0)
    {
      //do nothing keep draining
    };
  }

}

}
