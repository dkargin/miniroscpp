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

#define MINIROS_PACKAGE_NAME "transport_publisher_link"

#include <miniros/platform.h>  // platform dependendant requirements

#include "miniros/transport/transport_publisher_link.h"
#include "miniros/transport/subscription.h"
#include "miniros/header.h"
#include "miniros/transport/connection.h"
#include "miniros/transport/transport.h"
#include "miniros/this_node.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/file_log.h"
#include "miniros/transport/poll_manager.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/transport/timer_manager.h"
#include "miniros/transport/callback_queue.h"
#include "miniros/transport/internal_timer_manager.h"


#include <sstream>

namespace miniros
{

class TransportPublisherLink::DropWatcher : public Connection::DropWatcher {
public:
    DropWatcher(TransportPublisherLink& owner) : owner_(owner) {}

    void onConnectionDropped(
        const ConnectionPtr& connection,
        miniros::Connection::DropReason reason) override
    {
        owner_.onConnectionDropped(connection, reason);
    }

    TransportPublisherLink& owner_;
};

TransportPublisherLink::TransportPublisherLink(const SubscriptionPtr& parent, const std::string& xmlrpc_uri, const TransportHints& transport_hints)
: PublisherLink(parent, xmlrpc_uri, transport_hints)
, retry_timer_handle_(-1)
, needs_retry_(false)
, dropping_(false)
{
    drop_watcher_ = std::make_unique<DropWatcher>(*this);
}

TransportPublisherLink::~TransportPublisherLink()
{
  dropping_ = true;

  if (retry_timer_handle_ != -1)
  {
    getInternalTimerManager()->remove(retry_timer_handle_);
  }

  connection_->drop(Connection::Destructing);
}

bool TransportPublisherLink::initialize(const ConnectionPtr& connection)
{
  connection_ = connection;
  // slot_type is used to automatically track the TransporPublisherLink class' existence
  // and disconnect when this class' reference count is decremented to 0. It increments
  // then decrements the shared_from_this reference count around calls to the
  // onConnectionDropped function, preventing a coredump in the middle of execution.
  auto thisptr = std::static_pointer_cast<TransportPublisherLink>(shared_from_this());
  connection_->addDropWatcher(drop_watcher_.get());

  if (connection_->getTransport()->requiresHeader())
  {
    connection_->setHeaderReceivedCallback(
      [this](const ConnectionPtr& conn, const Header& header)
      {
        return this->onHeaderReceived(conn, header);
      });

    SubscriptionPtr parent = parent_.lock();
    if (!parent)
    {
      return false;
    }

    M_string header;
    header["topic"] = parent->getName();
    header["md5sum"] = parent->md5sum();
    header["callerid"] = this_node::getName();
    header["type"] = parent->datatype();
    header["tcp_nodelay"] = transport_hints_.getTCPNoDelay() ? "1" : "0";
    connection_->writeHeader(header, [this](const ConnectionPtr& conn){this->onHeaderWritten(conn);});
  }
  else
  {
    connection_->read(4,
      [this](const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
      {
        this->onMessageLength(conn, buffer, size, success);
      });
  }

  return true;
}

void TransportPublisherLink::drop()
{
  dropping_ = true;
  connection_->drop(Connection::Destructing);

  if (SubscriptionPtr parent = parent_.lock())
  {
    parent->removePublisherLink(shared_from_this());
  }
}

void TransportPublisherLink::onHeaderWritten(const ConnectionPtr& conn)
{
  (void)conn;
  // Do nothing
}

bool TransportPublisherLink::onHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  (void)conn;
  MINIROS_ASSERT(conn == connection_);

  if (!setHeader(header))
  {
    drop();
    return false;
  }

  if (retry_timer_handle_ != -1)
  {
    getInternalTimerManager()->remove(retry_timer_handle_);
    retry_timer_handle_ = -1;
  }

  connection_->read(4,
    [this](const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
    {
      this->onMessageLength(conn, buffer, size, success);
    });

  return true;
}

void TransportPublisherLink::onMessageLength(const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
{
  (void)conn;
  (void)size;
  if (retry_timer_handle_ != -1)
  {
    getInternalTimerManager()->remove(retry_timer_handle_);
    retry_timer_handle_ = -1;
  }

  if (!success)
  {
    if (connection_) {
      connection_->read(4,
        [this](const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
        {
          this->onMessageLength(conn, buffer, size, success);
        });
    }
    return;
  }

  MINIROS_ASSERT(conn == connection_);
  MINIROS_ASSERT(size == 4);

  uint32_t len = *((uint32_t*)buffer.get());

  if (len > 1000000000)
  {
    MINIROS_ERROR("a message of over a gigabyte was " \
                "predicted in tcpros. that seems highly " \
                "unlikely, so I'll assume protocol " \
                "synchronization is lost.");
    drop();
    return;
  }

  connection_->read(len,
    [this](const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
    {
      this->onMessage(conn, buffer, size, success);
    });
}

void TransportPublisherLink::onMessage(const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
{
  if (!success && !conn)
    return;

  MINIROS_ASSERT(conn == connection_);

  if (success)
  {
    handleMessage(SerializedMessage(buffer, size), true, false);
  }

  if (success || !connection_->getTransport()->requiresHeader())
  {
    connection_->read(4,
      [this](const ConnectionPtr& conn, const std::shared_ptr<uint8_t[]>& buffer, uint32_t size, bool success)
      {
        this->onMessageLength(conn, buffer, size, success);
      });
  }
}

void TransportPublisherLink::onRetryTimer(const miniros::SteadyTimerEvent&)
{
  if (dropping_)
  {
    return;
  }

  if (needs_retry_ && SteadyTime::now() > next_retry_)
  {
    retry_period_ = std::min<WallDuration>(retry_period_ * 2, WallDuration(20));
    needs_retry_ = false;
    SubscriptionPtr parent = parent_.lock();
    // TODO: support retry on more than just TCP
    // For now, since UDP does not have a heartbeat, we do not attempt to retry
    // UDP connections since an error there likely means some invalid operation has
    // happened.
    if (connection_->getTransport()->getType() == std::string("TCPROS"))
    {
      std::string topic = parent ? parent->getName() : "unknown";

      TransportTCPPtr old_transport = std::dynamic_pointer_cast<TransportTCP>(connection_->getTransport());
      MINIROS_ASSERT(old_transport);
      const std::string& host = old_transport->getConnectedHost();
      int port = old_transport->getConnectedPort();

      MINIROS_DEBUG("Retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());

      TransportTCPPtr transport(std::make_shared<TransportTCP>(&PollManager::instance()->getPollSet()));
      if (transport->connect(host, port))
      {
        ConnectionPtr connection(std::make_shared<Connection>());
        connection->initialize(transport, false, HeaderReceivedFunc());
        if (initialize(connection))
        {
          ConnectionManager::instance()->addConnection(connection);
        }
      }
      else
      {
        MINIROS_DEBUG("connect() failed when retrying connection to [%s:%d] for topic [%s]", host.c_str(), port, topic.c_str());
      }
    }
    else if (parent)
    {
      parent->removePublisherLink(shared_from_this());
    }
  }
}

CallbackQueuePtr getInternalCallbackQueue();

void TransportPublisherLink::onConnectionDropped(const ConnectionPtr& conn, Connection::DropReason reason)
{
  (void)conn;
  if (dropping_)
  {
    return;
  }

  MINIROS_ASSERT(conn == connection_);

  SubscriptionPtr parent = parent_.lock();

  if (reason == Connection::TransportDisconnect)
  {
    std::string topic = parent ? parent->getName() : "unknown";

    MINIROS_DEBUG("Connection to publisher [%s] to topic [%s] dropped", connection_->getTransport()->getTransportInfo().c_str(), topic.c_str());

    MINIROS_ASSERT(!needs_retry_);
    needs_retry_ = true;
    next_retry_ = SteadyTime::now() + retry_period_;

    if (retry_timer_handle_ == -1)
    {
      retry_period_ = WallDuration(0.1);
      next_retry_ = SteadyTime::now() + retry_period_;
      // shared_from_this() shared_ptr is used to ensure TransportPublisherLink is not
      // destroyed in the middle of onRetryTimer execution
      retry_timer_handle_ = getInternalTimerManager()->add(WallDuration(retry_period_),
        [this](const miniros::SteadyTimerEvent& event){this->onRetryTimer(event);},
        getInternalCallbackQueue().get(),
        shared_from_this(), false);
    }
    else
    {
      getInternalTimerManager()->setPeriod(retry_timer_handle_, retry_period_);
    }
  }
  else
  {
    drop();
  }
}

void TransportPublisherLink::handleMessage(const SerializedMessage& m, bool ser, bool nocopy)
{
  stats_.bytes_received_ += m.num_bytes;
  stats_.messages_received_++;

  SubscriptionPtr parent = parent_.lock();

  if (parent)
  {
    stats_.drops_ += parent->handleMessage(m, ser, nocopy, getConnection()->getHeader().getValues(), shared_from_this());
  }
}

std::string TransportPublisherLink::getTransportType()
{
  return connection_->getTransport()->getType();
}

std::string TransportPublisherLink::getTransportInfo()
{
  return connection_->getTransport()->getTransportInfo();
}

} // namespace miniros

