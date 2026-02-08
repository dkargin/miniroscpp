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

#ifndef MINIROS_POLL_SET_H
#define MINIROS_POLL_SET_H

#include <functional>
#include <memory>

#include "miniros/macros.h"
#include "miniros/internal/code_location.h"

namespace miniros
{

/**
 * \brief Manages a set of sockets being polled through the poll() function call.
 *
 * PollSet provides thread-safe ways of adding and deleting sockets, as well as adding
 * and deleting events.
 *
 * Warning: PollSet must not use rosconsole in any way. This class is used too early
 * and used for all socket interactions, including rosconsole.
 */
class MINIROS_DECL PollSet
{
public:
  PollSet();
  ~PollSet();


  /// Use this flag to subscribe to "input" events. They are equal to POLLIN.
  static const int EventIn;
  /// Use this flag to get notified when output is possible. It is equal to POLLOUT.
  static const int EventOut;
  /// Equal to POLLERR.
  static const int EventError;

  /// Low precision timer event.
  static constexpr int EventTimer = 1<<29;
  /// Add this evt flag to update event flags by return value from SocketUpdateFunc.
  static constexpr int EventUpdate = 1 << 30;

  /// Drop FD from poll set.
  /// It can be returned by event handler.
  static const int ResultDropFD = 1 << 24;

  /// Object to be kept back during active callback.
  typedef std::shared_ptr<void> TrackedObject;

  typedef std::function<int (int)> SocketUpdateFunc;

  typedef std::function<void (int)> SocketUpdateVoidFunc;
  /**
   * \brief Add a socket.
   *
   * addSocket() may be called from any thread.
   * \param sock The socket to add
   * \param events selected events. They are equal to events in poll/epoll, except PollSet::EventUpdate, which
   *      allows to update events by returning new event flags from SocketUpdateFunc call.
   * \param update_func The function to call when a socket has events
   * \param object The (optional) object associated with this socket. Mainly
   * used to prevent the transport from being deleted while we're calling the update function
   */
  bool addSocket(int sock, int events, const SocketUpdateFunc& update_func, const TrackedObject& object = TrackedObject(), const internal::CodeLocation& loc = {});

  /**
   * \brief Delete a socket
   *
   * delSocket() may be called from any thread.
   * \param sock The socket to delete
   */
  bool delSocket(int sock);

  /**
   * \brief Add events to be polled on a socket
   *
   * addEvents() may be called from any thread.
   * \param sock The socket to add events to
   * \param events The events to add
   */
  bool addEvents(int sock, int events);
  /**
   * \brief Delete events to be polled on a socket
   *
   * delEvents() may be called from any thread.
   * \param sock The socket to delete events from
   * \param events The events to delete
   */
  bool delEvents(int sock, int events);


  /**
   * \brief Override the whole set of events with new flags.
   *
   * setEvents() may be called from any thread.
   * \param sock The socket to delete events from
   * \param events The events to delete
   */
  bool setEvents(int sock, int events);

  /**
   * \brief Enable oneshot timer event for a socket.
   * It will overwrite previous timer if this event was already set.
   * @param sock The socket to run event.
   * @param timeoutMs time for event to happen.
   */
  bool setTimerEvent(int sock, int timeoutMs);

  /**
   * \brief Process all socket events
   *
   * This function will actually call poll() on the available sockets, and allow
   * them to do their processing.
   *
   * update() may only be called from one thread at a time
   *
   * \param poll_timeout The time, in milliseconds, for the poll() call to timeout after
   * if there are no events.  Note that this does not provide an upper bound for the entire
   * function, just the call to poll()
   */
  void update(int poll_timeout);

  /**
   * \brief Signal our poll() call to finish if it's blocked waiting (see the poll_timeout
   * option for update()).
   */
  void signal();

  static std::string eventToString(int event);

private:
  /**
   * \brief Creates the native pollset for our sockets, if any have changed
   */
  void createNativePollset();

  /**
   * \brief Called when events have been triggered on our signal pipe
   */
  int onLocalPipeEvents(int events);

  void processTimers();

  struct Internal;
  std::unique_ptr<Internal> internal_;
};

}

#endif // MINIROS_POLL_SET_H
