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

#ifndef ROSCPP_INTRAPROCESS_SUBSCRIBER_LINK_H
#define ROSCPP_INTRAPROCESS_SUBSCRIBER_LINK_H

#include "subscriber_link.h"
#include "miniros/common.h"

#include <mutex>

namespace miniros
{

class IntraProcessPublisherLink;
typedef std::shared_ptr<IntraProcessPublisherLink> IntraProcessPublisherLinkPtr;

/**
 * \brief SubscriberLink handles broadcasting messages to a single subscriber on a single topic
 */
class MINIROS_DECL IntraProcessSubscriberLink : public SubscriberLink
{
public:
  IntraProcessSubscriberLink(const PublicationPtr& parent);
  virtual ~IntraProcessSubscriberLink();

  void setSubscriber(const IntraProcessPublisherLinkPtr& subscriber);
  bool isLatching();

  virtual void enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy);
  virtual void drop();
  virtual std::string getTransportType();
  virtual std::string getTransportInfo();
  virtual bool isIntraprocess() { return true; }
  virtual void getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti);

private:
  IntraProcessPublisherLinkPtr subscriber_;
  bool dropped_;
  std::recursive_mutex drop_mutex_;
};
typedef std::shared_ptr<IntraProcessSubscriberLink> IntraProcessSubscriberLinkPtr;

} // namespace miniros

#endif // ROSCPP_INTRAPROCESS_SUBSCRIBER_LINK_H
