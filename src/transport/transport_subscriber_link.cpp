
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

#define MINIROS_PACKAGE_NAME "transport_subscriber_link"

#include "miniros/transport/transport_subscriber_link.h"
#include "miniros/transport/publication.h"
#include "miniros/header.h"
#include "miniros/transport/connection.h"
#include "miniros/transport/transport.h"
#include "miniros/this_node.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/topic_manager.h"
#include "miniros/file_log.h"

namespace miniros
{

class TransportSubscriberLink::DropWatcher : public Connection::DropWatcher {
public:
    DropWatcher(TransportSubscriberLink& owner) : owner_(owner) {}

    void onConnectionDropped(
        const ConnectionPtr& connection,
        miniros::Connection::DropReason reason) override
    {
        owner_.onConnectionDropped(connection);
    }

    TransportSubscriberLink& owner_;
};

TransportSubscriberLink::TransportSubscriberLink()
: writing_message_(false)
, header_written_(false)
, queue_full_(false)
{
    drop_watcher_ = std::make_unique<DropWatcher>(*this);
}

TransportSubscriberLink::~TransportSubscriberLink()
{
  drop();
}

bool TransportSubscriberLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;

  connection_->addDropWatcher(drop_watcher_.get());

  return true;
}

bool TransportSubscriberLink::handleHeader(const Header& header)
{
  std::string topic;
  if (!header.getValue("topic", topic))
  {
    std::string msg("Header from subscriber did not have the required element: topic");

    MINIROS_ERROR("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  // This will get validated by validateHeader below
  std::string client_callerid;
  header.getValue("callerid", client_callerid);
  PublicationPtr pt = TopicManager::instance()->lookupPublication(topic);
  if (!pt)
  {
    std::string msg = std::string("received a connection for a nonexistent topic [") +
                    topic + std::string("] from [" + connection_->getTransport()->getTransportInfo() + "] [" + client_callerid +"].");

    MINIROS_DEBUG("%s", msg.c_str());
    connection_->sendHeaderError(msg);

    return false;
  }

  std::string error_msg;
  if (!pt->validateHeader(header, error_msg))
  {
    MINIROS_DEBUG("%s", error_msg.c_str());
    connection_->sendHeaderError(error_msg);

    return false;
  }

  destination_caller_id_ = client_callerid;
  connection_id_ = ConnectionManager::instance()->getNewConnectionID();
  topic_ = pt->getName();
  parent_ = PublicationWPtr(pt);

  // Send back a success, with info
  M_string m;
  m["type"] = pt->getDataType();
  m["md5sum"] = pt->getMD5Sum();
  m["message_definition"] = pt->getMessageDefinition();
  m["callerid"] = this_node::getName();
  m["latching"] = pt->isLatching() ? "1" : "0";
  m["topic"] = topic_;
  connection_->writeHeader(m, [this](const ConnectionPtr& conn) {this->onHeaderWritten(conn);});

  pt->addSubscriberLink(shared_from_this());

  return true;
}

void TransportSubscriberLink::onConnectionDropped(const ConnectionPtr& conn)
{
  (void)conn;
  MINIROS_ASSERT(conn == connection_);

  PublicationPtr parent = parent_.lock();

  if (parent)
  {
    MINIROS_DEBUG("Connection to subscriber [%s] to topic [%s] dropped", connection_->getRemoteString().c_str(), topic_.c_str());

    parent->removeSubscriberLink(shared_from_this());
  }
}

void TransportSubscriberLink::onHeaderWritten(const ConnectionPtr& conn)
{
  (void)conn;
  header_written_ = true;
  startMessageWrite(true);
}

void TransportSubscriberLink::onMessageWritten(const ConnectionPtr& conn)
{
  (void)conn;
  writing_message_ = false;
  startMessageWrite(true);
}

void TransportSubscriberLink::startMessageWrite(bool immediate_write)
{
  std::shared_ptr<uint8_t[]> dummy;
  SerializedMessage m(dummy, (uint32_t)0);

  {
    std::scoped_lock<std::mutex> lock(outbox_mutex_);
    if (writing_message_ || !header_written_)
    {
      return;
    }

    if (!outbox_.empty())
    {
      writing_message_ = true;
      m = outbox_.front();
      outbox_.pop();
    }
  }

  if (m.num_bytes > 0)
  {
    connection_->write(m.buf, m.num_bytes,
      [this](const ConnectionPtr& conn){this->onMessageWritten(conn);}, immediate_write);
  }
}

void TransportSubscriberLink::enqueueMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  (void)nocopy;
  if (!ser)
  {
    return;
  }

  {
    std::scoped_lock<std::mutex> lock(outbox_mutex_);

    int max_queue = 0;
    if (PublicationPtr parent = parent_.lock())
    {
      max_queue = parent->getMaxQueue();
    }

    MINIROS_DEBUG_NAMED("superdebug", "TransportSubscriberLink on topic [%s] to caller [%s], queueing message (queue size [%d])", topic_.c_str(), destination_caller_id_.c_str(), (int)outbox_.size());

    if (max_queue > 0 && (int)outbox_.size() >= max_queue)
    {
      if (!queue_full_)
      {
        MINIROS_DEBUG("Outgoing queue full for topic [%s].  "
               "Discarding oldest message\n",
               topic_.c_str());
      }

      outbox_.pop(); // toss out the oldest thing in the queue to make room for us
      queue_full_ = true;
    }
    else
    {
      queue_full_ = false;
    }

    outbox_.push(m);
  }

  startMessageWrite(false);

  stats_.messages_sent_++;
  stats_.bytes_sent_ += m.num_bytes;
  stats_.message_data_sent_ += m.num_bytes;
}

std::string TransportSubscriberLink::getTransportType()
{
  return connection_->getTransport()->getType();
}

std::string TransportSubscriberLink::getTransportInfo()
{
  return connection_->getTransport()->getTransportInfo();
}

void TransportSubscriberLink::drop()
{
  // Only drop the connection if it's not already sending a header error
  // If it is, it will automatically drop itself
  if (connection_->isSendingHeaderError())
  {
    drop_watcher_->disconnect();
  }
  else
  {
    connection_->drop(Connection::Destructing);
  }
}

} // namespace miniros
