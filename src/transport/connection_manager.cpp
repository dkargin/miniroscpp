/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#define MINIROS_PACKAGE_NAME "connection_manager"

#include "miniros/transport/connection_manager.h"
#include "miniros/transport/poll_manager.h"
#include "miniros/transport/connection.h"
#include "miniros/transport/transport_subscriber_link.h"
#include "miniros/transport/service_client_link.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/transport/transport_udp.h"
#include "miniros/transport/file_log.h"
#include "miniros/transport/network.h"

#include <miniros/rosassert.h>

namespace miniros
{

class ConnectionManager::PollWatcher : public PollManager::PollWatcher {
public:
    PollWatcher(ConnectionManager& owner) : m_owner(owner) {}

    void onPollEvents() override {
        m_owner.removeDroppedConnections();
    }

protected:
    ConnectionManager& m_owner;
};

const ConnectionManagerPtr& ConnectionManager::instance()
{
  static ConnectionManagerPtr connection_manager = std::make_shared<ConnectionManager>();
  return connection_manager;
}

ConnectionManager::ConnectionManager()
: connection_id_counter_(0)
{
    poll_watcher_.reset(new PollWatcher(*this));
}

ConnectionManager::~ConnectionManager()
{
  shutdown();
}

void ConnectionManager::start()
{
  poll_manager_ = PollManager::instance();

  poll_manager_->addPollThreadWatcher(poll_watcher_.get());

  // Bring up the TCP listener socket
  tcpserver_transport_ = std::make_shared<TransportTCP>(&poll_manager_->getPollSet());
  bool started = tcpserver_transport_->listen(network::getTCPROSPort(), MAX_TCPROS_CONN_QUEUE,
    [this](const TransportTCPPtr& transport)
    {
      this->tcprosAcceptConnection(transport);
    });
  if (!started)
  {
    MINIROS_FATAL("Listen on port [%d] failed", network::getTCPROSPort());
    MINIROS_BREAK();
  }

  // Bring up the UDP listener socket
  udpserver_transport_ = std::make_shared<TransportUDP>(&poll_manager_->getPollSet());
  if (!udpserver_transport_->createIncoming(0, true))
  {
    MINIROS_FATAL("Listen failed");
    MINIROS_BREAK();
  }
}

void ConnectionManager::shutdown()
{
  if (udpserver_transport_)
  {
    udpserver_transport_->close();
    udpserver_transport_.reset();
  }

  if (tcpserver_transport_)
  {
    tcpserver_transport_->close();
    tcpserver_transport_.reset();
  }

  poll_watcher_->disconnect();

  clear(Connection::Destructing);
}

void ConnectionManager::clear(Connection::DropReason reason)
{
  S_Connection local_connections;
  {
    std::scoped_lock<std::mutex> conn_lock(connections_mutex_);
    local_connections.swap(connections_);
  }

  for(S_Connection::iterator itr = local_connections.begin();
      itr != local_connections.end();
      itr++)
  {
    const ConnectionPtr& conn = *itr;
    conn->drop(reason);
  }

  std::scoped_lock<std::mutex> dropped_lock(dropped_connections_mutex_);
  dropped_connections_.clear();
}

uint32_t ConnectionManager::getTCPPort()
{
  return tcpserver_transport_->getServerPort();
}

uint32_t ConnectionManager::getUDPPort()
{
  return udpserver_transport_->getServerPort();
}

uint32_t ConnectionManager::getNewConnectionID()
{
  std::scoped_lock<std::mutex> lock(connection_id_counter_mutex_);
  uint32_t ret = connection_id_counter_++;
  return ret;
}

void ConnectionManager::addConnection(const ConnectionPtr& conn)
{
  std::scoped_lock<std::mutex> lock(connections_mutex_);

  connections_.insert(conn);
  // TODO: Once upon a time here was a registration for dropped connections.
}

void ConnectionManager::onConnectionDropped(const ConnectionPtr& conn)
{
  std::scoped_lock<std::mutex> lock(dropped_connections_mutex_);
  dropped_connections_.push_back(conn);
}

void ConnectionManager::removeDroppedConnections()
{
  V_Connection local_dropped;
  {
      // There are not much connections, so direct iteration should not be that slow.
      for (const ConnectionPtr& connection: connections_) {
          if (connection->isDropped()) {
              local_dropped.push_back(connection);
          }
      }
  }

  std::scoped_lock<std::mutex> conn_lock(connections_mutex_);

  V_Connection::iterator conn_it = local_dropped.begin();
  V_Connection::iterator conn_end = local_dropped.end();
  for (;conn_it != conn_end; ++conn_it)
  {
    const ConnectionPtr& conn = *conn_it;
    connections_.erase(conn);
  }
}

void ConnectionManager::udprosIncomingConnection(const TransportUDPPtr& transport, Header& header)
{
  std::string client_uri = ""; // TODO: transport->getClientURI();
  MINIROS_DEBUG("UDPROS received a connection from [%s]", client_uri.c_str());

  ConnectionPtr conn(std::make_shared<Connection>());
  addConnection(conn);

  conn->initialize(transport, true, NULL);
  onConnectionHeaderReceived(conn, header);
}

void ConnectionManager::tcprosAcceptConnection(const TransportTCPPtr& transport)
{
  std::string client_uri = transport->getClientURI();
  MINIROS_DEBUG("TCPROS received a connection from [%s]", client_uri.c_str());

  ConnectionPtr conn(std::make_shared<Connection>());
  addConnection(conn);

  conn->initialize(transport, true,
    [this](const ConnectionPtr& connection, const Header& header)
    {
      return this->onConnectionHeaderReceived(connection, header);
    });
}

bool ConnectionManager::onConnectionHeaderReceived(const ConnectionPtr& conn, const Header& header)
{
  bool ret = false;
  std::string val;
  if (header.getValue("topic", val))
  {
    MINIROS_DEBUG(
     "Connection: Creating TransportSubscriberLink for topic [%s] connected to [%s]",
     val.c_str(), conn->getRemoteString().c_str());

    TransportSubscriberLinkPtr sub_link(std::make_shared<TransportSubscriberLink>());
    sub_link->initialize(conn);
    ret = sub_link->handleHeader(header);
  }
  else if (header.getValue("service", val))
  {
    MINIROS_DEBUG(
      "Connection: Creating ServiceClientLink for service [%s] connected to [%s]",
      val.c_str(), conn->getRemoteString().c_str());

    ServiceClientLinkPtr link(std::make_shared<ServiceClientLink>());
    link->initialize(conn);
    ret = link->handleHeader(header);
  }
  else
  {
    MINIROS_DEBUG(
      "Got a connection for a type other than 'topic' or 'service' from [%s].  Fail.",
      conn->getRemoteString().c_str());
    return false;
  }

  return ret;
}

}
