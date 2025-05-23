/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
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

#ifndef ROSCPP_FORWARDS_H
#define ROSCPP_FORWARDS_H

#include <string>
#include <vector>
#include <map>
#include <set>

#include <memory>
#include <functional>

#include <miniros/rostime.h>
#include <miniros/macros.h>
#include "exceptions.h"
#include "miniros/datatypes.h"

namespace miniros
{

typedef std::shared_ptr<void> VoidPtr;
typedef std::weak_ptr<void> VoidWPtr;
typedef std::shared_ptr<void const> VoidConstPtr;
typedef std::weak_ptr<void const> VoidConstWPtr;

class Header;
class Transport;
typedef std::shared_ptr<Transport> TransportPtr;
class TransportTCP;
typedef std::shared_ptr<TransportTCP> TransportTCPPtr;
class TransportUDP;
typedef std::shared_ptr<TransportUDP> TransportUDPPtr;
class Connection;
typedef std::shared_ptr<Connection> ConnectionPtr;
typedef std::set<ConnectionPtr> S_Connection;
typedef std::vector<ConnectionPtr> V_Connection;
class Publication;
typedef std::shared_ptr<Publication> PublicationPtr;
typedef std::vector<PublicationPtr> V_Publication;
class SubscriberLink;
typedef std::shared_ptr<SubscriberLink> SubscriberLinkPtr;
typedef std::vector<SubscriberLinkPtr> V_SubscriberLink;
class Subscription;
typedef std::shared_ptr<Subscription> SubscriptionPtr;
typedef std::weak_ptr<Subscription> SubscriptionWPtr;
class PublisherLink;
typedef std::shared_ptr<PublisherLink> PublisherLinkPtr;
typedef std::vector<PublisherLinkPtr> V_PublisherLink;
class ServicePublication;
typedef std::shared_ptr<ServicePublication> ServicePublicationPtr;
typedef std::vector<ServicePublicationPtr> V_ServicePublication;
class ServiceServerLink;
typedef std::shared_ptr<ServiceServerLink> ServiceServerLinkPtr;
class Transport;
typedef std::shared_ptr<Transport> TransportPtr;
class NodeHandle;
typedef std::shared_ptr<NodeHandle> NodeHandlePtr;


class SingleSubscriberPublisher;
typedef std::function<void(const SingleSubscriberPublisher&)> SubscriberStatusCallback;

class CallbackQueue;
class CallbackQueueInterface;
class CallbackInterface;
typedef std::shared_ptr<CallbackInterface> CallbackInterfacePtr;

struct SubscriberCallbacks
{
  SubscriberCallbacks(const SubscriberStatusCallback& connect = SubscriberStatusCallback(),
                      const SubscriberStatusCallback& disconnect = SubscriberStatusCallback(),
                      const VoidConstPtr& tracked_object = VoidConstPtr(),
                      CallbackQueueInterface* callback_queue = 0)
  : connect_(connect)
  , disconnect_(disconnect)
  , callback_queue_(callback_queue)
  {
    has_tracked_object_ = false;
    if (tracked_object)
    {
      has_tracked_object_ = true;
      tracked_object_ = tracked_object;
    }
  }
  SubscriberStatusCallback connect_;
  SubscriberStatusCallback disconnect_;

  bool has_tracked_object_;
  VoidConstWPtr tracked_object_;
  CallbackQueueInterface* callback_queue_;
};
typedef std::shared_ptr<SubscriberCallbacks> SubscriberCallbacksPtr;

/**
 * \brief Structure passed as a parameter to the callback invoked by a miniros::Timer
 */
struct TimerEvent
{
  Time last_expected;                     ///< In a perfect world, this is when the last callback should have happened
  Time last_real;                         ///< When the last callback actually happened
  Time last_expired;                      ///< When the last timer actually expired and the callback was added to the queue

  Time current_expected;                  ///< In a perfect world, this is when the current callback should be happening
  Time current_real;                      ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)
  Time current_expired;                   ///< When the current timer actually expired and the callback was added to the queue

  struct
  {
    WallDuration last_duration;           ///< How long the last callback ran for
  } profile;
};
typedef std::function<void(const TimerEvent&)> TimerCallback;

/**
 * \brief Structure passed as a parameter to the callback invoked by a miniros::WallTimer
 */
struct WallTimerEvent
{
  WallTime last_expected;                 ///< In a perfect world, this is when the last callback should have happened
  WallTime last_real;                     ///< When the last callback actually happened
  WallTime last_expired;                  ///< When the last timer actually expired and the callback was added to the queue

  WallTime current_expected;              ///< In a perfect world, this is when the current callback should be happening
  WallTime current_real;                  ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)
  WallTime current_expired;               ///< When the current timer actually expired and the callback was added to the queue

  struct
  {
    WallDuration last_duration;           ///< How long the last callback ran for
  } profile;
};
typedef std::function<void(const WallTimerEvent&)> WallTimerCallback;

/**
 * \brief Structure passed as a parameter to the callback invoked by a miniros::SteadyTimer
 */
struct SteadyTimerEvent
{
  SteadyTime last_expected;            ///< In a perfect world, this is when the last callback should have happened
  SteadyTime last_real;                ///< When the last callback actually happened
  SteadyTime last_expired;             ///< When the last timer actually expired and the callback was added to the queue

  SteadyTime current_expected;         ///< In a perfect world, this is when the current callback should be happening
  SteadyTime current_real;             ///< This is when the current callback was actually called (SteadyTime::now() as of the beginning of the callback)
  SteadyTime current_expired;          ///< When the current timer actually expired and the callback was added to the queue

  struct
  {
    WallDuration last_duration;           ///< How long the last callback ran for
  } profile;
};
typedef std::function<void(const SteadyTimerEvent&)> SteadyTimerCallback;

class ServiceManager;
typedef std::shared_ptr<ServiceManager> ServiceManagerPtr;
class TopicManager;
typedef std::shared_ptr<TopicManager> TopicManagerPtr;
class ConnectionManager;
typedef std::shared_ptr<ConnectionManager> ConnectionManagerPtr;
class RPCManager;
typedef std::shared_ptr<RPCManager> RPCManagerPtr;
class PollManager;
typedef std::shared_ptr<PollManager> PollManagerPtr;
class MasterLink;
typedef std::shared_ptr<MasterLink> MasterLinkPtr;

}

#endif
