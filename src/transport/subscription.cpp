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

#define MINIROS_PACKAGE_NAME "subscription"

#include <algorithm>
#include <sstream>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#include <typeinfo>

#include "miniros/network/network.h"
#include "miniros/this_node.h"
#include "miniros/io/poll_manager.h"

#include "miniros/transport/callback_queue_interface.h"
#include "miniros/transport/connection.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/intraprocess_publisher_link.h"
#include "miniros/transport/intraprocess_subscriber_link.h"
#include "miniros/transport/message_deserializer.h"
#include "miniros/transport/publication.h"
#include "miniros/transport/subscription.h"

#include "http/http_client.h"
#include "http/xmlrpc_request.h"
#include "internal/xml_tools.h"
#include "miniros/transport/subscription_callback_helper.h"
#include "miniros/transport/subscription_queue.h"
#include "miniros/transport/transport_hints.h"
#include "miniros/transport/transport_publisher_link.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/transport/transport_udp.h"

#include "miniros/xmlrpcpp/XmlRpcClient.h"

using XmlRpc::XmlRpcValue;

namespace miniros
{

struct Subscription::PendingConnection
{
  PendingConnection(TransportUDPPtr udp_transport, const SubscriptionWPtr& parent, const std::string& remote_uri);
  ~PendingConnection();

  TransportUDPPtr getUDPTransport() const;
  const std::string& getRemoteURI();

  TransportUDPPtr udp_transport_;
  SubscriptionWPtr parent_;
  std::string remote_uri_;
};

Subscription::PendingConnection::PendingConnection(TransportUDPPtr udp_transport, const SubscriptionWPtr& parent, const std::string& remote_uri)
: udp_transport_(udp_transport)
, parent_(parent)
, remote_uri_(remote_uri)
{
}

Subscription::PendingConnection::~PendingConnection()
{
}

TransportUDPPtr Subscription::PendingConnection::getUDPTransport() const
{
  return udp_transport_;
}

const std::string& Subscription::PendingConnection::getRemoteURI()
{
  return remote_uri_;
}


Subscription::Subscription(const std::string &name, const std::string& md5sum, const std::string& datatype, const TransportHints& transport_hints)
: name_(name)
, md5sum_(md5sum)
, datatype_(datatype)
, nonconst_callbacks_(0)
, dropped_(false)
, shutting_down_(false)
, transport_hints_(transport_hints)
{
}

Subscription::~Subscription()
{
  pending_connections_.clear();
  callbacks_.clear();
}

void Subscription::shutdown()
{
  {
    std::scoped_lock<std::mutex> lock(shutdown_mutex_);
    shutting_down_ = true;
  }

  drop();
}

XmlRpcValue Subscription::getStats()
{
  XmlRpcValue stats;
  stats[0] = name_;
  XmlRpcValue conn_data;
  conn_data.setSize(0);

  std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

  uint32_t cidx = 0;
  for (V_PublisherLink::iterator c = publisher_links_.begin();
       c != publisher_links_.end(); ++c)
  {
    const PublisherLink::Stats& s = (*c)->getStats();
    conn_data[cidx][0] = (*c)->getConnectionID();
    conn_data[cidx][1] = (int)s.bytes_received_;
    conn_data[cidx][2] = (int)s.messages_received_;
    conn_data[cidx][3] = (int)s.drops_;
    conn_data[cidx][4] = 0; // figure out something for this. not sure.
  }

  stats[1] = conn_data;
  return stats;
}

// [(connection_id, publisher_xmlrpc_uri, direction, transport, topic_name, connected, connection_info_string)*]
// e.g. [(1, 'http://host:54893/', 'i', 'TCPROS', '/chatter', 1, 'TCPROS connection on port 59746 to [host:34318 on socket 11]')]
void Subscription::getInfo(XmlRpc::XmlRpcValue& info)
{
  std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

  for (V_PublisherLink::iterator c = publisher_links_.begin();
       c != publisher_links_.end(); ++c)
  {
    XmlRpcValue curr_info;
    curr_info[0] = (int)(*c)->getConnectionID();
    curr_info[1] = (*c)->getPublisherXMLRPCURI();
    curr_info[2] = "i";
    curr_info[3] = (*c)->getTransportType();
    curr_info[4] = name_;
    curr_info[5] = true; // For length compatibility with rospy
    curr_info[6] = (*c)->getTransportInfo();
    info[info.size()] = curr_info;
  }
}

const std::string& Subscription::getName() const
{
  return name_;
}

uint32_t Subscription::getNumCallbacks() const
{
  return static_cast<int32_t>(callbacks_.size());
}


uint32_t Subscription::getNumPublishers() const
{
    std::scoped_lock<std::mutex> lock(publisher_links_mutex_);
    return (uint32_t)publisher_links_.size();
}

void Subscription::drop()
{
  if (!dropped_)
  {
    dropped_ = true;

    dropAllConnections();
  }
}

void Subscription::dropAllConnections()
{
  // Swap our subscribers list with a local one so we can only lock for a short period of time, because a
  // side effect of our calling drop() on connections can be re-locking the subscribers mutex
  V_PublisherLink localsubscribers;

  {
    std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

    localsubscribers.swap(publisher_links_);
  }

  V_PublisherLink::iterator it = localsubscribers.begin();
  V_PublisherLink::iterator end = localsubscribers.end();
  for (;it != end; ++it)
  {
    (*it)->drop();
  }
}

void Subscription::addLocalConnection(const RPCManagerPtr& rpcManager, const PublicationPtr& pub)
{
  std::scoped_lock<std::mutex> lock(publisher_links_mutex_);
  if (dropped_)
  {
    return;
  }

  MINIROS_DEBUG("Creating intraprocess link for topic [%s]", name_.c_str());

  auto pub_link = std::make_shared<IntraProcessPublisherLink>(shared_from_this(), rpcManager->getServerUrlStr(), transport_hints_);
  auto sub_link = std::make_shared<IntraProcessSubscriberLink>(pub);
  pub_link->setPublisher(sub_link);
  sub_link->setSubscriber(pub_link);

  addPublisherLink(pub_link);
  pub->addSubscriberLink(sub_link);
}

bool urisEqual(const std::string& uri1, const std::string& uri2)
{
  std::string host1, host2;
  uint32_t port1 = 0, port2 = 0;
  network::splitURI(uri1, host1, port1);
  network::splitURI(uri2, host2, port2);
  return port1 == port2 && host1 == host2;
}

bool Subscription::pubUpdate(const RPCManagerPtr& rpcManager, const V_string& new_pubs)
{
  std::scoped_lock<std::mutex> slock(shutdown_mutex_);

  if (shutting_down_ || dropped_)
  {
    return false;
  }

  bool retval = true;

  const std::string ownURI = rpcManager->getServerUrlStr();
  {
    std::stringstream ss;

    for (const std::string& up_i: new_pubs)
      ss << up_i << ", ";

    ss << " already have these connections {";
    {
      std::scoped_lock<std::mutex> lock(publisher_links_mutex_);
      for (PublisherLinkPtr& plink: publisher_links_)
        ss << plink->getPublisherXMLRPCURI() << ", ";
    }

    ss << " } pending = { ";

    {
      std::scoped_lock<std::mutex> lock(pending_connections_mutex_);
      for (const PendingConnectionPtr& conn: pending_connections_)
        ss << conn->getRemoteURI() << ", ";
    }
    ss << "}";

    MINIROS_DEBUG("Publisher update for [%s]: %s", name_.c_str(), ss.str().c_str());
  }

  V_string additions;
  V_PublisherLink subtractions;
  V_PublisherLink to_add;
  // could use the STL set operations... but these sets are so small
  // it doesn't really matter.
  {
    std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

    for (V_PublisherLink::iterator spc = publisher_links_.begin();
         spc!= publisher_links_.end(); ++spc)
    {
      bool found = false;
      for (V_string::const_iterator up_i = new_pubs.begin();
           !found && up_i != new_pubs.end(); ++up_i)
      {
        if (urisEqual((*spc)->getPublisherXMLRPCURI(), *up_i))
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        subtractions.push_back(*spc);
      }
    }

    for (V_string::const_iterator up_i  = new_pubs.begin(); up_i != new_pubs.end(); ++up_i)
    {
      bool found = false;
      for (V_PublisherLink::iterator spc = publisher_links_.begin();
           !found && spc != publisher_links_.end(); ++spc)
      {
        if (urisEqual(*up_i, (*spc)->getPublisherXMLRPCURI()))
        {
          found = true;
          break;
        }
      }

      if (!found)
      {
        std::scoped_lock<std::mutex> lock(pending_connections_mutex_);
        for (const PendingConnectionPtr& conn: pending_connections_)
        {
          if (urisEqual(*up_i, conn->getRemoteURI()))
          {
            found = true;
            break;
          }
        }
      }

      if (!found)
      {
        additions.push_back(*up_i);
      }
    }
  }

  for (const PublisherLinkPtr& link: subtractions)
  {
    if (link->getPublisherXMLRPCURI() != ownURI)
    {
      MINIROS_DEBUG("Disconnecting from publisher [%s] of topic [%s] at [%s]",
                  link->getCallerID().c_str(), name_.c_str(), link->getPublisherXMLRPCURI().c_str());
      link->drop();
    }
    else
    {
      MINIROS_DEBUG("Disconnect: skipping myself for topic [%s]", name_.c_str());
    }
  }

  for (const std::string& uri: additions)
  {
    // this function should never negotiate a self-subscription
    if (ownURI != uri)
    {
      retval &= negotiateConnection(rpcManager, uri);
    }
    else
    {
      MINIROS_DEBUG("Skipping myself (%s, %s)", name_.c_str(), ownURI.c_str());
    }
  }

  return retval;
}

bool Subscription::negotiateConnection(const RPCManagerPtr& rpcManager, const std::string& xmlrpc_uri)
{
  XmlRpcValue tcpros_array, protos_array, params;
  XmlRpcValue udpros_array;
  TransportUDPPtr udp_transport;
  int protos = 0;
  V_string transports = transport_hints_.getTransports();
  PollSet* ps = rpcManager->getPollSet();
  if (transports.empty())
  {
    transport_hints_.reliable();
    transports = transport_hints_.getTransports();
  }
  for (const std::string& transport: transports)
  {
    if (transport == "UDP")
    {
      int max_datagram_size = transport_hints_.getMaxDatagramSize();
      assert(ps);
      udp_transport = std::make_shared<TransportUDP>(ps);
      if (!max_datagram_size)
        max_datagram_size = udp_transport->getMaxDatagramSize();
      udp_transport->createIncoming(0, false);
      udpros_array[0] = "UDPROS";
      M_string m;
      m["topic"] = getName();
      m["md5sum"] = md5sum();
      m["callerid"] = this_node::getName();
      m["type"] = datatype();
      std::shared_ptr<uint8_t[]> buffer;
      uint32_t len;
      Header::write(m, buffer, len);
      XmlRpcValue v(buffer.get(), len);
      udpros_array[1] = v;
      udpros_array[2] = network::getHost();
      udpros_array[3] = udp_transport->getServerPort();
      udpros_array[4] = max_datagram_size;

      protos_array[protos++] = udpros_array;
    }
    else if (transport == "TCP")
    {
      tcpros_array[0] = std::string("TCPROS");
      protos_array[protos++] = tcpros_array;
    }
    else
    {
      MINIROS_WARN("Unsupported transport type hinted: %s, skipping", transport.c_str());
    }
  }
  params[0] = this_node::getName();
  params[1] = name_;
  params[2] = protos_array;

  network::URL url;
  if (!url.fromString(xmlrpc_uri, false))
  {
    MINIROS_ERROR("Bad xml-rpc URI: [%s]", xmlrpc_uri.c_str());
    return false;
  }

  // The PendingConnectionPtr takes ownership of c, and will delete it on
  // destruction.
  PendingConnectionPtr conn = std::make_shared<PendingConnection>(udp_transport, shared_from_this(), xmlrpc_uri);

  auto c = rpcManager->getXMLRPCClient(url.host, url.port, "/");
  auto req = http::makeRequest("/", "requestTopic");
  req->setParamArray(params);

  req->onCompleteRaw = [conn, url](Error err, const http::XmlRpcRequest::RpcValue& data, bool ok) {
      if (SubscriptionPtr parent = conn->parent_.lock())
      {
        parent->pendingConnectionDone(conn, url, data);
      }
  };
  req->setRetry(0);

  // Initiate the negotiation.  We'll come back and check on it later.
  if (!c->enqueueRequest(req))
  {
    MINIROS_ERROR("Failed to contact publisher [%s:%d] for topic [%s]", url.host.c_str(), url.port, name_.c_str());
    if (udp_transport)
    {
      udp_transport->close();
    }
    return false;
  }

  MINIROS_INFO("Began asynchronous xmlrpc connection to [%s:%d] fd=%d", url.host.c_str(), url.port, c->fd());
  // Put this connection on the list that we'll look at later.
  {
    std::scoped_lock<std::mutex> pending_connections_lock(pending_connections_mutex_);
    pending_connections_.insert(conn);
  }
  return true;
}

void closeTransport(const TransportUDPPtr& trans)
{
  if (trans)
  {
    trans->close();
  }
}

void Subscription::pendingConnectionDone(const PendingConnectionPtr& conn, const network::URL& url, const XmlRpcValue& result)
{
  std::scoped_lock<std::mutex> lock(shutdown_mutex_);
  if (shutting_down_ || dropped_)
  {
    return;
  }

  {
    std::scoped_lock<std::mutex> pending_connections_lock(pending_connections_mutex_);
    pending_connections_.erase(conn);
  }

  auto cm = ConnectionManager::instance();

  TransportUDPPtr udp_transport;

  std::string xmlrpc_uri = url.str();
  udp_transport = conn->getUDPTransport();

  xml::XmlCodec codec;
  XmlRpcValue proto;
  if(!codec.validateXmlrpcResponse("requestTopic", result, proto))
  {
    MINIROS_DEBUG("Failed to contact publisher [%s] for topic [%s]", xmlrpc_uri.c_str(), name_.c_str());
    closeTransport(udp_transport);
    return;
  }

  if (proto.size() == 0)
  {
    MINIROS_DEBUG("Couldn't agree on any common protocols with [%s] for topic [%s]", xmlrpc_uri.c_str(), name_.c_str());
    closeTransport(udp_transport);
    return;
  }

  if (proto.getType() != XmlRpcValue::TypeArray)
  {
    MINIROS_DEBUG("Available protocol info returned from %s is not a list.", xmlrpc_uri.c_str());
    closeTransport(udp_transport);
    return;
  }
  if (proto[0].getType() != XmlRpcValue::TypeString)
  {
    MINIROS_DEBUG("Available protocol info list doesn't have a string as its first element.");
    closeTransport(udp_transport);
    return;
  }

  std::string proto_name = proto[0];
  if (proto_name == "TCPROS")
  {
    if (proto.size() != 3 ||
        proto[1].getType() != XmlRpcValue::TypeString ||
        proto[2].getType() != XmlRpcValue::TypeInt)
    {
        MINIROS_DEBUG("publisher implements TCPROS, but the " \
                "parameters aren't string,int");
      return;
    }
    std::string pub_host = proto[1];
    int pub_port = proto[2];
    MINIROS_DEBUG("Connecting via tcpros to topic [%s] at host [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);

    TransportTCPPtr transport(std::make_shared<TransportTCP>(&PollManager::instance()->getPollSet()));
    if (transport->connect(pub_host, pub_port))
    {
      ConnectionPtr connection(std::make_shared<Connection>());
      TransportPublisherLinkPtr pub_link(std::make_shared<TransportPublisherLink>(shared_from_this(), xmlrpc_uri, transport_hints_));

      connection->initialize(transport, false, HeaderReceivedFunc());
      pub_link->initialize(connection);

      cm->addConnection(connection);

      std::scoped_lock<std::mutex> lock2(publisher_links_mutex_);
      addPublisherLink(pub_link);

      MINIROS_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
    else
    {
      MINIROS_DEBUG("Failed to connect to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
  }
  else if (proto_name == "UDPROS")
  {
    if (proto.size() != 6 ||
        proto[1].getType() != XmlRpcValue::TypeString ||
        proto[2].getType() != XmlRpcValue::TypeInt ||
        proto[3].getType() != XmlRpcValue::TypeInt ||
        proto[4].getType() != XmlRpcValue::TypeInt ||
        proto[5].getType() != XmlRpcValue::TypeBase64)
    {
      MINIROS_DEBUG("publisher implements UDPROS, but the " \
	    	       "parameters aren't string,int,int,int,base64");
      closeTransport(udp_transport);
      return;
    }
    std::string pub_host = proto[1];
    int pub_port = proto[2];
    int conn_id = proto[3];
    int max_datagram_size = proto[4];
    std::vector<char> header_bytes = proto[5];
    std::shared_ptr<uint8_t[]> buffer(new uint8_t[header_bytes.size()]);
    memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
    Header h;
    std::string err;
    if (!h.parse(buffer, header_bytes.size(), err))
    {
      MINIROS_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
      closeTransport(udp_transport);
      return;
    }
    MINIROS_DEBUG("Connecting via udpros to topic [%s] at host [%s:%d] connection id [%08x] max_datagram_size [%d]", name_.c_str(), pub_host.c_str(), pub_port, conn_id, max_datagram_size);

    std::string error_msg;
    if (h.getValue("error", error_msg))
    {
      MINIROS_DEBUG("Received error message in header for connection to [%s]: [%s]", xmlrpc_uri.c_str(), error_msg.c_str());
      closeTransport(udp_transport);
      return;
    }

    TransportPublisherLinkPtr pub_link(std::make_shared<TransportPublisherLink>(shared_from_this(), xmlrpc_uri, transport_hints_));
    if (pub_link->setHeader(h))
    {
      ConnectionPtr connection(std::make_shared<Connection>());
      connection->initialize(udp_transport, false, NULL);
      connection->setHeader(h);
      pub_link->initialize(connection);

      cm->addConnection(connection);

      std::scoped_lock<std::mutex> lock2(publisher_links_mutex_);
      addPublisherLink(pub_link);

      MINIROS_DEBUG("Connected to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
    }
    else
    {
      MINIROS_DEBUG("Failed to connect to publisher of topic [%s] at [%s:%d]", name_.c_str(), pub_host.c_str(), pub_port);
      closeTransport(udp_transport);
      return;
    }
  }
  else
  {
  	MINIROS_DEBUG("Publisher offered unsupported transport [%s]", proto_name.c_str());
  }
}

uint32_t Subscription::handleMessage(const SerializedMessage& m, bool ser, bool nocopy, const std::shared_ptr<M_string>& connection_header, const PublisherLinkPtr& link)
{
  std::scoped_lock<std::mutex> lock(callbacks_mutex_);

  uint32_t drops = 0;

  // Cache the deserializers by type info.  If all the subscriptions are the same type this has the same performance as before.  If
  // there are subscriptions with different C++ type (but same ROS message type), this now works correctly rather than passing
  // garbage to the messages with different C++ types than the first one.
  cached_deserializers_.clear();

  miniros::Time receipt_time = miniros::Time::now();

  for (V_CallbackInfo::iterator cb = callbacks_.begin();
       cb != callbacks_.end(); ++cb)
  {
    const CallbackInfoPtr& info = *cb;

    MINIROS_ASSERT(info->callback_queue_);

    const std::type_info* ti = &info->helper_->getTypeInfo();

    if ((nocopy && m.type_info && *ti == *m.type_info) || (ser && (!m.type_info || *ti != *m.type_info)))
    {
      MessageDeserializerPtr deserializer;

      V_TypeAndDeserializer::iterator des_it = cached_deserializers_.begin();
      V_TypeAndDeserializer::iterator des_end = cached_deserializers_.end();
      for (; des_it != des_end; ++des_it)
      {
        if (*des_it->first == *ti)
        {
          deserializer = des_it->second;
          break;
        }
      }

      if (!deserializer)
      {
        deserializer = std::make_shared<MessageDeserializer>(info->helper_, m, connection_header);
        cached_deserializers_.push_back(std::make_pair(ti, deserializer));
      }

      bool was_full = false;
      bool nonconst_need_copy = false;
      if (callbacks_.size() > 1)
      {
        nonconst_need_copy = true;
      }

      info->subscription_queue_->push(info->helper_, deserializer, info->has_tracked_object_, info->tracked_object_, nonconst_need_copy, receipt_time, &was_full);

      if (was_full)
      {
        ++drops;
      }
      else
      {
        info->callback_queue_->addCallback(info->subscription_queue_, (uint64_t)info.get());
      }
    }
  }

  // measure statistics
  statistics_.callback(connection_header, name_, link->getCallerID(), m, link->getStats().bytes_received_, receipt_time, drops > 0);

  // If this link is latched, store off the message so we can immediately pass it to new subscribers later
  if (link->isLatched())
  {
    LatchInfo li;
    li.connection_header = connection_header;
    li.link = link;
    li.message = m;
    li.receipt_time = receipt_time;
    latched_messages_[link] = li;
  }

  cached_deserializers_.clear();

  return drops;
}

void Subscription::initStatistics(const SubscriptionCallbackHelperPtr& helper, const MasterLinkPtr& master_link)
{
  MINIROS_ASSERT(helper);
  MINIROS_ASSERT(master_link);

  statistics_.init(helper, master_link);
}

bool Subscription::addCallback(const SubscriptionCallbackHelperPtr& helper, const std::string& md5sum,
  CallbackQueueInterface* queue, int32_t queue_size, const VoidConstPtr& tracked_object, bool allow_concurrent_callbacks)
{
  MINIROS_ASSERT(helper);
  MINIROS_ASSERT(queue);


  // Decay to a real type as soon as we have a subscriber with a real type
  {
    std::scoped_lock<std::mutex> lock(md5sum_mutex_);
    if (md5sum_ == "*" && md5sum != "*")
    {

      md5sum_ = md5sum;
    }
  }

  if (md5sum != "*" && md5sum != this->md5sum())
  {
    return false;
  }

  {
    std::scoped_lock<std::mutex> lock(callbacks_mutex_);

    CallbackInfoPtr info(std::make_shared<CallbackInfo>());
    info->helper_ = helper;
    info->callback_queue_ = queue;
    info->subscription_queue_ = std::make_shared<SubscriptionQueue>(name_, queue_size, allow_concurrent_callbacks);
    info->tracked_object_ = tracked_object;
    info->has_tracked_object_ = false;
    if (tracked_object)
    {
      info->has_tracked_object_ = true;
    }

    if (!helper->isConst())
    {
      ++nonconst_callbacks_;
    }

    callbacks_.push_back(info);
    cached_deserializers_.reserve(callbacks_.size());

    // if we have any latched links, we need to immediately schedule callbacks
    if (!latched_messages_.empty())
    {
      std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

      for (const PublisherLinkPtr& link: publisher_links_)
      {
        if (!link->isLatched())
          continue;

        auto des_it = latched_messages_.find(link);
        if (des_it == latched_messages_.end())
          continue;

        const LatchInfo& latch_info = des_it->second;
        auto des = std::make_shared<MessageDeserializer>(helper, latch_info.message, latch_info.connection_header);
        bool was_full = false;
        info->subscription_queue_->push(info->helper_, des, info->has_tracked_object_, info->tracked_object_, true, latch_info.receipt_time, &was_full);
        if (!was_full)
        {
          info->callback_queue_->addCallback(info->subscription_queue_, (uint64_t)info.get());
        }
      }
    }
  }

  return true;
}

void Subscription::removeCallback(const SubscriptionCallbackHelperPtr& helper)
{
  CallbackInfoPtr info;
  {
    std::scoped_lock<std::mutex> cbs_lock(callbacks_mutex_);
    for (V_CallbackInfo::iterator it = callbacks_.begin();
         it != callbacks_.end(); ++it)
    {
      if ((*it)->helper_ == helper)
      {
        info = *it;
        callbacks_.erase(it);

        if (!helper->isConst())
        {
          --nonconst_callbacks_;
        }

        break;
      }
    }
  }

  if (info)
  {
    info->subscription_queue_->clear();
    info->callback_queue_->removeByID((uint64_t)info.get());
  }
}

void Subscription::headerReceived(const PublisherLinkPtr& link, const Header& h)
{
  (void)h;
  std::scoped_lock<std::mutex> lock(md5sum_mutex_);
  if (md5sum_ == "*")
  {
    md5sum_ = link->getMD5Sum();
  }
}

void Subscription::addPublisherLink(const PublisherLinkPtr& link)
{
  publisher_links_.push_back(link);
}

void Subscription::removePublisherLink(const PublisherLinkPtr& pub_link)
{
  std::scoped_lock<std::mutex> lock(publisher_links_mutex_);

  V_PublisherLink::iterator it = std::find(publisher_links_.begin(), publisher_links_.end(), pub_link);
  if (it != publisher_links_.end())
  {
    publisher_links_.erase(it);
  }

  if (pub_link->isLatched())
  {
    latched_messages_.erase(pub_link);
  }
}

void Subscription::getPublishTypes(bool& ser, bool& nocopy, const std::type_info& ti)
{
  std::scoped_lock<std::mutex> lock(callbacks_mutex_);
  for (V_CallbackInfo::iterator cb = callbacks_.begin();
       cb != callbacks_.end(); ++cb)
  {
    const CallbackInfoPtr& info = *cb;
    if (info->helper_->getTypeInfo() == ti)
    {
      nocopy = true;
    }
    else
    {
      ser = true;
    }

    if (nocopy && ser)
    {
      return;
    }
  }
}

const std::string Subscription::datatype()
{
  return datatype_;
}

const std::string Subscription::md5sum()
{
  std::scoped_lock<std::mutex> lock(md5sum_mutex_);
  return md5sum_;
}

}
