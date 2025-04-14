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

#define MINIROS_PACKAGE_NAME "topic_manager"

#include "miniros/transport/topic_manager.h"
#include "miniros/master_link.h"
#include "miniros/this_node.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/network.h"
#include "miniros/transport/poll_manager.h"
#include "miniros/transport/publication.h"
#include "miniros/transport/rosout_appender.h"
#include "miniros/transport/rpc_manager.h"
#include "miniros/transport/subscribe_options.h"
#include "miniros/transport/subscription.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/transport/transport_udp.h"
#include "miniros/transport/net_address.h"

#include <miniros/console.h>
#include <sstream>
#include <xmlrpcpp/XmlRpcServerConnection.h>

using namespace XmlRpc; // A battle to be fought later

/// \todo Locking can be significantly simplified here once the Node API goes away.

namespace miniros {

class TopicManager::PollWatcher : public PollManager::PollWatcher {
public:
  PollWatcher(TopicManager& owner) : m_owner(owner)
  {
  }

  void onPollEvents() override
  {
    m_owner.processPublishQueues();
  }

protected:
  TopicManager& m_owner;
};

const TopicManagerPtr& TopicManager::instance()
{
  static TopicManagerPtr topic_manager = std::make_shared<TopicManager>();
  return topic_manager;
}

TopicManager::TopicManager() : shutting_down_(false)
{
  poll_watcher_.reset(new PollWatcher(*this));
}

TopicManager::~TopicManager()
{
  shutdown();
}

Error TopicManager::start(PollManagerPtr pm, MasterLinkPtr master_link, ConnectionManagerPtr cm, RPCManagerPtr rpcm)
{
  if (!pm || !cm || !rpcm)
    return Error::InvalidValue;

  std::scoped_lock<std::mutex> shutdown_lock(shutting_down_mutex_);
  shutting_down_ = false;
  resolve_ip_ = false;

  master_link_ = master_link;
  poll_manager_ = pm;
  connection_manager_ = cm;
  rpc_manager_ = rpcm;

  rpc_manager_->bind("publisherUpdate", [this](const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    this->pubUpdateCallback(params, result);
  });

  rpc_manager_->bindEx3("requestTopic", this, &TopicManager::requestTopic);

  rpc_manager_->bind("getBusStats", [this](const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    this->getBusStatsCallback(params, result);
  });
  rpc_manager_->bind("getBusInfo", [this](const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    this->getBusInfoCallback(params, result);
  });
  rpc_manager_->bind("getSubscriptions", [this](const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    this->getSubscriptionsCallback(params, result);
  });
  rpc_manager_->bind("getPublications", [this](const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result) {
    this->getPublicationsCallback(params, result);
  });

  poll_manager_->addPollThreadWatcher(poll_watcher_.get());

  if (master_link_) {
    resolve_ip_ = master_link->param<bool>("/resolve_ip", false);
  }
  return Error::Ok;
}

void TopicManager::shutdown()
{
  std::scoped_lock<std::mutex> shutdown_lock(shutting_down_mutex_);
  if (shutting_down_) {
    return;
  }

  {
    std::lock(subs_mutex_, advertised_topics_mutex_);
    shutting_down_ = true;
    subs_mutex_.unlock();
    advertised_topics_mutex_.unlock();
  }

  poll_watcher_->disconnect();

  rpc_manager_->unbind("publisherUpdate");
  rpc_manager_->unbind("requestTopic");
  rpc_manager_->unbind("getBusStats");
  rpc_manager_->unbind("getBusInfo");
  rpc_manager_->unbind("getSubscriptions");
  rpc_manager_->unbind("getPublications");

  MINIROS_DEBUG("Shutting down topics...");
  MINIROS_DEBUG("  shutting down publishers");
  {
    std::scoped_lock<std::recursive_mutex> adv_lock(advertised_topics_mutex_);

    for (PublicationPtr pub : advertised_topics_) {
      if (!pub->isDropped())
        unregisterPublisher(pub->getName());
      pub->drop();
    }
    advertised_topics_.clear();
  }

  // unregister all of our subscriptions
  MINIROS_DEBUG("  shutting down subscribers");
  {
    std::scoped_lock<std::mutex> subs_lock(subs_mutex_);

    for (SubscriptionPtr s : subscriptions_) {
      // Remove us as a subscriber from the master
      unregisterSubscriber(s->getName());
      // now, drop our side of the connection
      s->shutdown();
    }
    subscriptions_.clear();
  }

  rpc_manager_.reset();
  connection_manager_.reset();
  poll_manager_.reset();
  master_link_.reset();
}

void TopicManager::processPublishQueues()
{
  std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

  for (const PublicationPtr& pub : advertised_topics_)
    pub->processPublishQueue();
}

void TopicManager::getAdvertisedTopics(V_string& topics)
{
  std::scoped_lock<std::mutex> lock(advertised_topic_names_mutex_);
  topics.resize(advertised_topic_names_.size());
  std::copy(advertised_topic_names_.begin(), advertised_topic_names_.end(), topics.begin());
}

void TopicManager::getSubscribedTopics(V_string& topics)
{
  std::scoped_lock<std::mutex> lock(subs_mutex_);

  topics.reserve(subscriptions_.size());
  for (const SubscriptionPtr& sub : subscriptions_)
    topics.push_back(sub->getName());
}

PublicationPtr TopicManager::lookupPublication(const std::string& topic)
{
  std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

  return lookupPublicationWithoutLock(topic);
}

bool md5sumsMatch(const std::string& lhs, const std::string& rhs)
{
  return lhs == "*" || rhs == "*" || lhs == rhs;
}

bool TopicManager::addSubCallback(const SubscribeOptions& ops)
{
  // spin through the subscriptions and see if we find a match. if so, use it.
  bool found = false;
  bool found_topic = false;

  SubscriptionPtr sub;

  {
    if (isShuttingDown()) {
      return false;
    }

    for (auto s = subscriptions_.begin(); s != subscriptions_.end() && !found; ++s) {
      sub = *s;
      if (!sub->isDropped() && sub->getName() == ops.topic) {
        found_topic = true;
        if (md5sumsMatch(ops.md5sum, sub->md5sum())) {
          found = true;
        }
        break;
      }
    }
  }

  if (found_topic && !found) {
    std::stringstream ss;
    ss << "Tried to subscribe to a topic with the same name but different md5sum as a topic that was already "
          "subscribed ["
       << ops.datatype << "/" << ops.md5sum << " vs. " << sub->datatype() << "/" << sub->md5sum() << "]";
    throw ConflictingSubscriptionException(ss.str());
  } else if (found) {
    if (!sub->addCallback(ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object,
          ops.allow_concurrent_callbacks)) {
      return false;
    }
  }

  return found;
}

// this function has the subscription code that doesn't need to be templated.
bool TopicManager::subscribe(const SubscribeOptions& ops)
{
  std::scoped_lock<std::mutex> lock(subs_mutex_);

  if (addSubCallback(ops)) {
    return true;
  }

  if (isShuttingDown()) {
    return false;
  }

  if (ops.md5sum.empty()) {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty md5sum");
  }

  if (ops.datatype.empty()) {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] with an empty datatype");
  }

  if (!ops.helper) {
    throw InvalidParameterException("Subscribing to topic [" + ops.topic + "] without a callback");
  }

  const std::string& md5sum = ops.md5sum;
  std::string datatype = ops.datatype;

  auto s = std::make_shared<Subscription>(ops.topic, md5sum, datatype, ops.transport_hints);
  s->initStatistics(ops.helper, master_link_);
  s->addCallback(
    ops.helper, ops.md5sum, ops.callback_queue, ops.queue_size, ops.tracked_object, ops.allow_concurrent_callbacks);

  if (!registerSubscriber(s, ops.datatype)) {
    MINIROS_WARN("couldn't register subscriber on topic [%s]", ops.topic.c_str());
    s->shutdown();
    return false;
  }

  subscriptions_.push_back(s);

  return true;
}

bool TopicManager::advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks)
{
  if (!master_link_) {
    MINIROS_ERROR("TopicManager::advertise() called with no master link");
    return false;
  }

  if (ops.datatype == "*") {
    std::stringstream ss;
    ss << "Advertising with * as the datatype is not allowed.  Topic [" << ops.topic << "]";
    throw InvalidParameterException(ss.str());
  }

  if (ops.md5sum == "*") {
    std::stringstream ss;
    ss << "Advertising with * as the md5sum is not allowed.  Topic [" << ops.topic << "]";
    throw InvalidParameterException(ss.str());
  }

  if (ops.md5sum.empty()) {
    throw InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty md5sum");
  }

  if (ops.datatype.empty()) {
    throw InvalidParameterException("Advertising on topic [" + ops.topic + "] with an empty datatype");
  }

  if (ops.message_definition.empty()) {
    MINIROS_WARN(
      "Advertising on topic [%s] with an empty message definition.  Some tools (e.g. rosbag) may not work correctly.",
      ops.topic.c_str());
  }

  PublicationPtr pub;

  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

    if (isShuttingDown()) {
      return false;
    }

    pub = lookupPublicationWithoutLock(ops.topic);
    if (pub && pub->getNumCallbacks() == 0) {
      pub.reset();
    }

    if (pub) {
      if (pub->getMD5Sum() != ops.md5sum) {
        MINIROS_ERROR("Tried to advertise on topic [%s] with md5sum [%s] and datatype [%s], but the topic is already "
                      "advertised as md5sum [%s] and datatype [%s]",
          ops.topic.c_str(), ops.md5sum.c_str(), ops.datatype.c_str(), pub->getMD5Sum().c_str(),
          pub->getDataType().c_str());
        return false;
      }

      pub->addCallbacks(callbacks);

      return true;
    }

    pub = PublicationPtr(std::make_shared<Publication>(
      ops.topic, ops.datatype, ops.md5sum, ops.message_definition, ops.queue_size, ops.latch, ops.has_header));
    pub->addCallbacks(callbacks);
    advertised_topics_.push_back(pub);
  }

  {
    std::scoped_lock<std::mutex> lock(advertised_topic_names_mutex_);
    advertised_topic_names_.push_back(ops.topic);
  }

  // Check whether we've already subscribed to this topic.  If so, we'll do
  // the self-subscription here, to avoid the deadlock that would happen if
  // the ROS thread later got the publisherUpdate with its own XMLRPC URI.
  // The assumption is that advertise() is called from somewhere other
  // than the ROS thread.
  bool found = false;
  SubscriptionPtr sub;
  {
    std::scoped_lock<std::mutex> lock(subs_mutex_);

    for (SubscriptionPtr& s : subscriptions_) {
      if (s->getName() == ops.topic && md5sumsMatch(s->md5sum(), ops.md5sum) && !s->isDropped()) {
        found = true;
        sub = s;
        break;
      }
    }
  }

  if (found) {
    sub->addLocalConnection(pub);
  }

  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ops.topic;
  args[2] = ops.datatype;
  args[3] = rpc_manager_->getServerURI();
  return master_link_->execute("registerPublisher", args, result, payload, true);
}

bool TopicManager::unadvertise(const std::string& topic, const SubscriberCallbacksPtr& callbacks)
{
  PublicationPtr pub;
  V_Publication::iterator i;
  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

    if (isShuttingDown()) {
      return false;
    }

    for (i = advertised_topics_.begin(); i != advertised_topics_.end(); ++i) {
      if (((*i)->getName() == topic) && (!(*i)->isDropped())) {
        pub = *i;
        break;
      }
    }
  }

  if (!pub) {
    return false;
  }

  pub->removeCallbacks(callbacks);

  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);
    if (pub->getNumCallbacks() == 0) {
      unregisterPublisher(pub->getName());
      pub->drop();

      advertised_topics_.erase(i);

      {
        std::scoped_lock<std::mutex> lock(advertised_topic_names_mutex_);
        advertised_topic_names_.remove(pub->getName());
      }
    }
  }

  return true;
}

bool TopicManager::unregisterPublisher(const std::string& topic)
{
  if (!master_link_)
    return false;
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = topic;
  args[2] = rpc_manager_->getServerURI();
  return master_link_->execute("unregisterPublisher", args, result, payload, false);
}

bool TopicManager::isTopicAdvertised(const std::string& topic)
{
  for (auto& advertised_topic : advertised_topics_) {
    if ((advertised_topic->getName() == topic) && (!advertised_topic->isDropped())) {
      return true;
    }
  }

  return false;
}

bool TopicManager::registerSubscriber(const SubscriptionPtr& s, const std::string& datatype)
{
  if (!master_link_) {
    MINIROS_ERROR("TopicManager::registerSubscriber: master link is null");
    return false;
  }

  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = s->getName();
  args[2] = datatype;
  args[3] = rpc_manager_->getServerURI();

  if (!master_link_->execute("registerSubscriber", args, result, payload, true)) {
    return false;
  }

  std::vector<std::string> pub_uris;
  for (int i = 0; i < payload.size(); i++) {
    std::string pubUri = payload[i];
    if (pubUri != rpc_manager_->getServerURI()) {
      pub_uris.push_back(pubUri);
    }
  }

  bool self_subscribed = false;
  PublicationPtr pub;
  const std::string& sub_md5sum = s->md5sum();
  // Figure out if we have a local publisher
  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);
    V_Publication::const_iterator it = advertised_topics_.begin();
    V_Publication::const_iterator end = advertised_topics_.end();
    for (; it != end; ++it) {
      pub = *it;
      const std::string& pub_md5sum = pub->getMD5Sum();

      if (pub->getName() == s->getName() && !pub->isDropped()) {
        if (!md5sumsMatch(pub_md5sum, sub_md5sum)) {
          MINIROS_ERROR("md5sum mismatch making local subscription to topic %s.", s->getName().c_str());
          MINIROS_ERROR("Subscriber expects type %s, md5sum %s", s->datatype().c_str(), s->md5sum().c_str());
          MINIROS_ERROR("Publisher provides type %s, md5sum %s", pub->getDataType().c_str(), pub->getMD5Sum().c_str());
          return false;
        }

        self_subscribed = true;
        break;
      }
    }
  }

  s->pubUpdate(pub_uris);
  if (self_subscribed) {
    s->addLocalConnection(pub);
  }

  return true;
}

bool TopicManager::unregisterSubscriber(const std::string& topic)
{
  if (!master_link_)
    return false;
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = topic;
  args[2] = rpc_manager_->getServerURI();

  return master_link_->execute("unregisterSubscriber", args, result, payload, false);
}

bool TopicManager::pubUpdate(const std::string& topic, const std::vector<std::string>& pubs)
{
  SubscriptionPtr sub;
  {
    std::scoped_lock<std::mutex> lock(subs_mutex_);

    if (isShuttingDown()) {
      return false;
    }

    MINIROS_DEBUG("Received update for topic [%s] (%d publishers)", topic.c_str(), (int)pubs.size());
    // find the subscription
    for (auto s = subscriptions_.begin(); s != subscriptions_.end(); ++s) {
      if ((*s)->getName() != topic || (*s)->isDropped())
        continue;

      sub = *s;
      break;
    }
  }

  if (sub) {
    return sub->pubUpdate(pubs);
  } else {
    MINIROS_DEBUG("got a request for updating publishers of topic %s, but I "
                  "don't have any subscribers to that topic.",
      topic.c_str());
  }

  return false;
}

XmlRpc::XmlRpcValue TopicManager::requestTopic(const std::string& caller, const std::string& topic, const XmlRpcValue& protos, XmlRpc::XmlRpcServerConnection* connection)
{
  RpcValue ret = RpcValue::Array(3);
  ret[0] = 0;
  ret[1] = "";
  ret[2] = 0;

  if (!connection_manager_) {
    return ret;
  }

  std::string goodAddress;

  network::NetAddress localAddress;
  if (resolve_ip_ && network::readLocalAddress(connection->getfd(), localAddress)) {
    goodAddress = localAddress.address;
  } else {
    goodAddress = network::getHost();
  }

  for (int proto_idx = 0; proto_idx < protos.size(); proto_idx++) {
    RpcValue proto = protos[proto_idx]; // save typing
    if (proto.getType() != XmlRpcValue::TypeArray) {
      MINIROS_DEBUG("requestTopic protocol list was not a list of lists");
      return ret;
    }

    if (proto[0].getType() != XmlRpcValue::TypeString) {
      MINIROS_DEBUG("requestTopic received a protocol list in which a sublist "
                    "did not start with a string");
      return ret;
    }

    std::string proto_name = proto[0];
    if (proto_name == std::string("TCPROS")) {
      XmlRpcValue tcpros_params;
      tcpros_params[0] = std::string("TCPROS");
      tcpros_params[1] = goodAddress;
      tcpros_params[2] = int(connection_manager_->getTCPPort());
      ret[0] = int(1);
      ret[1] = std::string();
      ret[2] = tcpros_params;
      return ret;
    } else if (proto_name == std::string("UDPROS")) {
      if (proto.size() != 5 || proto[1].getType() != XmlRpcValue::TypeBase64 ||
          proto[2].getType() != XmlRpcValue::TypeString || proto[3].getType() != XmlRpcValue::TypeInt ||
          proto[4].getType() != XmlRpcValue::TypeInt) {
        MINIROS_DEBUG("Invalid protocol parameters for UDPROS");
        return ret;
      }
      std::vector<char> header_bytes = proto[1];
      std::shared_ptr<uint8_t[]> buffer(new uint8_t[header_bytes.size()]);
      memcpy(buffer.get(), &header_bytes[0], header_bytes.size());
      Header h;
      std::string err;
      if (!h.parse(buffer, header_bytes.size(), err)) {
        MINIROS_DEBUG("Unable to parse UDPROS connection header: %s", err.c_str());
        return ret;
      }

      PublicationPtr pub_ptr = lookupPublication(topic);
      if (!pub_ptr) {
        MINIROS_DEBUG("Unable to find advertised topic %s for UDPROS connection", topic.c_str());
        return ret;
      }

      std::string host = proto[2];
      int port = proto[3];

      M_string m;
      std::string error_msg;
      if (!pub_ptr->validateHeader(h, error_msg)) {
        MINIROS_DEBUG("Error validating header from [%s:%d] for topic [%s]: %s", host.c_str(), port, topic.c_str(),
          error_msg.c_str());
        return ret;
      }

      int max_datagram_size = proto[4];
      int conn_id = connection_manager_->getNewConnectionID();
      TransportUDPPtr transport =
        connection_manager_->getUDPServerTransport()->createOutgoing(host, port, conn_id, max_datagram_size);
      if (!transport) {
        MINIROS_DEBUG("Error creating outgoing transport for [%s:%d]", host.c_str(), port);
        return ret;
      }
      connection_manager_->udprosIncomingConnection(transport, h);

      XmlRpcValue udpros_params;
      udpros_params[0] = std::string("UDPROS");
      udpros_params[1] = goodAddress;
      udpros_params[2] = connection_manager_->getUDPServerTransport()->getServerPort();
      udpros_params[3] = conn_id;
      udpros_params[4] = max_datagram_size;
      m["topic"] = topic;
      m["md5sum"] = pub_ptr->getMD5Sum();
      m["type"] = pub_ptr->getDataType();
      m["callerid"] = this_node::getName();
      m["message_definition"] = pub_ptr->getMessageDefinition();
      std::shared_ptr<uint8_t[]> msg_def_buffer;
      uint32_t len;
      Header::write(m, msg_def_buffer, len);
      XmlRpcValue v(msg_def_buffer.get(), len);
      udpros_params[5] = v;
      ret[0] = int(1);
      ret[1] = std::string();
      ret[2] = udpros_params;
      return ret;
    } else {
      MINIROS_DEBUG("an unsupported protocol was offered: [%s]", proto_name.c_str());
    }
  }

  MINIROS_DEBUG("Currently, roscpp only supports TCPROS. The caller to "
                "requestTopic did not support TCPROS, so there are no "
                "protocols in common.");
  return ret;
}

void TopicManager::publish(
  const std::string& topic, const std::function<SerializedMessage(void)>& serfunc, SerializedMessage& m)
{
  std::scoped_lock lock(advertised_topics_mutex_);

  if (isShuttingDown()) {
    return;
  }

  PublicationPtr p = lookupPublicationWithoutLock(topic);
  if (p->hasSubscribers() || p->isLatching()) {
    MINIROS_DEBUG_NAMED("superdebug", "Publishing message on topic [%s] with sequence number [%d]",
      p->getName().c_str(), p->getSequence());

    // Determine what kinds of subscribers we're publishing to.  If they're intraprocess with the same C++ type we can
    // do a no-copy publish.
    bool nocopy = false;
    bool serialize = false;

    // We can only do a no-copy publish if a shared_ptr to the message is provided, and we have type information for it
    if (m.type_info && m.message) {
      p->getPublishTypes(serialize, nocopy, *m.type_info);
    } else {
      serialize = true;
    }

    if (!nocopy) {
      m.message.reset();
      m.type_info = 0;
    }

    if (serialize || p->isLatching()) {
      SerializedMessage m2 = serfunc();
      m.buf = m2.buf;
      m.num_bytes = m2.num_bytes;
      m.message_start = m2.message_start;
    }

    p->publish(m);

    // If we're not doing a serialized publish we don't need to signal the pollset.  The write()
    // call inside signal() is actually relatively expensive when doing a nocopy publish.
    if (serialize) {
      poll_manager_->getPollSet().signal();
    }
  } else {
    p->incrementSequence();
  }
}

void TopicManager::incrementSequence(const std::string& topic)
{
  PublicationPtr pub = lookupPublication(topic);
  if (pub) {
    pub->incrementSequence();
  }
}

bool TopicManager::isLatched(const std::string& topic)
{
  PublicationPtr pub = lookupPublication(topic);
  if (pub) {
    return pub->isLatched();
  }

  return false;
}

PublicationPtr TopicManager::lookupPublicationWithoutLock(const std::string& topic)
{
  PublicationPtr t;
  for (V_Publication::iterator i = advertised_topics_.begin(); !t && i != advertised_topics_.end(); ++i) {
    if (((*i)->getName() == topic) && (!(*i)->isDropped())) {
      t = *i;
      break;
    }
  }

  return t;
}

bool TopicManager::unsubscribe(const std::string& topic, const SubscriptionCallbackHelperPtr& helper)
{
  SubscriptionPtr sub;

  {
    std::scoped_lock<std::mutex> lock(subs_mutex_);

    if (isShuttingDown()) {
      return false;
    }

    for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
      if ((*it)->getName() == topic) {
        sub = *it;
        break;
      }
    }
  }

  if (!sub) {
    return false;
  }

  sub->removeCallback(helper);

  if (sub->getNumCallbacks() == 0) {
    // nobody is left. blow away the subscription.
    {
      std::scoped_lock<std::mutex> lock(subs_mutex_);

      for (auto it = subscriptions_.begin(); it != subscriptions_.end(); ++it) {
        if ((*it)->getName() == topic) {
          subscriptions_.erase(it);
          break;
        }
      }

      if (!unregisterSubscriber(topic)) {
        MINIROS_DEBUG("Couldn't unregister subscriber for topic [%s]", topic.c_str());
      }
    }

    sub->shutdown();
    return true;
  }

  return true;
}

size_t TopicManager::getNumSubscribers(const std::string& topic)
{
  std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

  if (isShuttingDown()) {
    return 0;
  }

  PublicationPtr p = lookupPublicationWithoutLock(topic);
  if (p) {
    return p->getNumSubscribers();
  }

  return 0;
}

size_t TopicManager::getNumSubscriptions()
{
  std::scoped_lock<std::mutex> lock(subs_mutex_);
  return subscriptions_.size();
}

size_t TopicManager::getNumPublishers(const std::string& topic)
{
  std::scoped_lock<std::mutex> lock(subs_mutex_);

  if (isShuttingDown()) {
    return 0;
  }

  for (SubscriptionPtr t: subscriptions_) {
    if (!t->isDropped() && t->getName() == topic) {
      return t->getNumPublishers();
    }
  }

  return 0;
}

void TopicManager::getBusStats(XmlRpcValue& stats)
{
  XmlRpcValue publish_stats, subscribe_stats, service_stats;
  // force these guys to be arrays, even if we don't populate them
  publish_stats.setSize(advertised_topics_.size());
  subscribe_stats.setSize(subscriptions_.size());
  service_stats.setSize(0);

  {
    uint32_t pidx = 0;
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);
    for (PublicationPtr t: advertised_topics_)
      publish_stats[pidx++] = t->getStats();
  }

  {
    uint32_t sidx = 0;
    std::scoped_lock<std::mutex> lock(subs_mutex_);
    for (SubscriptionPtr t: subscriptions_)
      subscribe_stats[sidx++] = t->getStats();
  }

  stats[0] = publish_stats;
  stats[1] = subscribe_stats;
  stats[2] = service_stats;
}

void TopicManager::getBusInfo(XmlRpcValue& info)
{
  // force these guys to be arrays, even if we don't populate them
  info.setSize(0);

  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);
    for (PublicationPtr t: advertised_topics_) {
      t->getInfo(info);
    }
  }

  {
    std::scoped_lock<std::mutex> lock(subs_mutex_);
    for (SubscriptionPtr t: subscriptions_) {
      t->getInfo(info);
    }
  }
}

void TopicManager::getSubscriptions(XmlRpcValue& subs)
{
  // force these guys to be arrays, even if we don't populate them
  subs.setSize(0);

  {
    std::scoped_lock<std::mutex> lock(subs_mutex_);

    uint32_t sidx = 0;

    for (SubscriptionPtr t: subscriptions_) {
      XmlRpcValue sub;
      sub[0] = t->getName();
      sub[1] = t->datatype();
      subs[sidx++] = sub;
    }
  }
}

void TopicManager::getPublications(XmlRpcValue& pubs)
{
  // force these guys to be arrays, even if we don't populate them
  pubs.setSize(0);

  {
    std::scoped_lock<std::recursive_mutex> lock(advertised_topics_mutex_);

    uint32_t sidx = 0;

    for (PublicationPtr t: advertised_topics_) {
      XmlRpcValue pub;
      pub[0] = t->getName();
      pub[1] = t->getDataType();
      pubs[sidx++] = pub;
    }
  }
}

Error TopicManager::pubUpdateCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  std::stringstream ss;
  if (params.size() < 3)
    return Error::InvalidValue;
  if (params[1].getType() != XmlRpc::XmlRpcValue::TypeString)
    return Error::InvalidValue;
  if (params[2].getType() != XmlRpc::XmlRpcValue::TypeArray)
    return Error::InvalidValue;
  std::string topic = params[1];
  std::vector<std::string> pubs;
  for (int idx = 0; idx < params[2].size(); idx++) {
    std::string pub = params[2][idx];
    pubs.push_back(pub);
    if (idx != 0)
      ss << ",";
    ss << pub;
  }

  MINIROS_DEBUG("pubUpdateCallback(%s) publishers={%s}", topic.c_str(), ss.str().c_str());

  if (pubUpdate(topic, pubs)) {
    result = xmlrpc::responseInt(1, "", 0);
  } else {
    result = xmlrpc::responseInt(0, console::g_last_error_message, 0);
  }
  return Error::Ok;
}

void TopicManager::getBusStatsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  (void)params;
  result[0] = 1;
  result[1] = std::string("");
  XmlRpcValue response;
  getBusStats(result);
  result[2] = response;
}

void TopicManager::getBusInfoCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  (void)params;
  result[0] = 1;
  result[1] = std::string("");
  XmlRpcValue response;
  getBusInfo(response);
  result[2] = response;
}

void TopicManager::getSubscriptionsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  (void)params;
  result[0] = 1;
  result[1] = std::string("subscriptions");
  XmlRpcValue response;
  getSubscriptions(response);
  result[2] = response;
}

void TopicManager::getPublicationsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  (void)params;
  result[0] = 1;
  result[1] = std::string("publications");
  XmlRpcValue response;
  getPublications(response);
  result[2] = response;
}

MasterLinkPtr TopicManager::getMasterLink() const
{
  return master_link_;
}

} // namespace miniros
