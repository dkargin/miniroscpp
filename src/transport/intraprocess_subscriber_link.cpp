
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

#define MINIROS_PACKAGE_NAME "intraprocess_subscriber_link"

#include "miniros/transport/intraprocess_subscriber_link.h"
#include "miniros/transport/intraprocess_publisher_link.h"
#include "miniros/transport/publication.h"
#include "miniros/transport/transport.h"
#include "miniros/this_node.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/topic_manager.h"
#include "miniros/file_log.h"

namespace miniros
{

IntraProcessSubscriberLink::IntraProcessSubscriberLink(const PublicationPtr& parent)
: dropped_(false)
{
  MINIROS_ASSERT(parent);
  parent_ = parent;
  topic_ = parent->getName();
}

IntraProcessSubscriberLink::~IntraProcessSubscriberLink()
{
}

void IntraProcessSubscriberLink::setSubscriber(const IntraProcessPublisherLinkPtr& subscriber)
{
  subscriber_ = subscriber;
  connection_id_ = ConnectionManager::instance()->getNewConnectionID();
  destination_caller_id_ = this_node::getName();
}

bool IntraProcessSubscriberLink::isLatching()
{
  if (PublicationPtr parent = parent_.lock())
  {
    return parent->isLatching();
  }

  return false;
}

void IntraProcessSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  std::scoped_lock<std::recursive_mutex> lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }

  MINIROS_ASSERT(subscriber_);
  subscriber_->handleMessage(m, ser, nocopy);
}

std::string IntraProcessSubscriberLink::getTransportType()
{
  return std::string("INTRAPROCESS");
}

std::string IntraProcessSubscriberLink::getTransportInfo()
{
  // TODO: Check if we can dump more useful information here
  return getTransportType();
}

void IntraProcessSubscriberLink::drop()
{
  {
    std::scoped_lock<std::recursive_mutex> lock(drop_mutex_);
    if (dropped_)
    {
      return;
    }

    dropped_ = true;
  }

  if (subscriber_)
  {
    subscriber_->drop();
    subscriber_.reset();
  }

  if (PublicationPtr parent = parent_.lock())
  {
    MINIROS_DEBUG("Connection to local subscriber on topic [%s] dropped", topic_.c_str());

    parent->removeSubscriberLink(shared_from_this());
  }
}

void IntraProcessSubscriberLink::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti)
{
  std::scoped_lock<std::recursive_mutex> lock(drop_mutex_);
  if (dropped_)
  {
    return;
  }

  subscriber_->getPublishTypes(ser, nocopy, ti);
}

} // namespace miniros
