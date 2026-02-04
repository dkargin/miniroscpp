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

#ifndef MINIROS_TOPIC_MANAGER_H
#define MINIROS_TOPIC_MANAGER_H

#include "../network/network.h"

#include <list>

#include "miniros/internal/forwards.h"
#include "miniros/common.h"
#include "miniros/serialization.h"
#include "rosout_appender.h"

#include "miniros/xmlrpcpp/XmlRpcValue.h"

namespace XmlRpc {
class XmlRpcServerConnection;
}

namespace miniros
{

class Message;
struct SubscribeOptions;
struct AdvertiseOptions;

namespace network {
struct ClientInfo;
}

class TopicManager;
typedef std::shared_ptr<TopicManager> TopicManagerPtr;

class PollManager;
typedef std::shared_ptr<PollManager> PollManagerPtr;

class RPCManager;
typedef std::shared_ptr<RPCManager> RPCManagerPtr;

class ConnectionManager;
typedef std::shared_ptr<ConnectionManager> ConnectionManagerPtr;

class SubscriptionCallbackHelper;
typedef std::shared_ptr<SubscriptionCallbackHelper> SubscriptionCallbackHelperPtr;

class MINIROS_DECL TopicManager
{
public:
  static const TopicManagerPtr& instance();

  using RpcValue = XmlRpc::XmlRpcValue;

  TopicManager();
  ~TopicManager();

  Error start(PollManagerPtr pm, MasterLinkPtr master_link, ConnectionManagerPtr cm, RPCManagerPtr rpcm);
  void shutdown();

  bool subscribe(const SubscribeOptions& ops);
  bool unsubscribe(const std::string &_topic, const SubscriptionCallbackHelperPtr& helper);

  bool advertise(const AdvertiseOptions& ops, const SubscriberCallbacksPtr& callbacks);
  bool unadvertise(const std::string &topic, const SubscriberCallbacksPtr& callbacks);

  /** @brief Get the list of topics advertised by this node
   *
   * @param[out] topics The advertised topics
   */
  void getAdvertisedTopics(V_string& topics);

  /** @brief Get the list of topics subscribed to by this node
   *
   * @param[out] The subscribed topics
   */
  void getSubscribedTopics(V_string& topics);

  /** @brief Lookup an advertised topic.
   *
   * This method iterates over advertised_topics, looking for one with name
   * matching the given topic name.  The advertised_topics_mutex is locked
   * during this search.  This method is only used internally.
   *
   * @param topic The topic name to look for.
   *
   * @returns Pointer to the matching Publication, NULL if none is found.
   */
  PublicationPtr lookupPublication(const std::string& topic);

  /** @brief Return the number of subscribers a node has for a particular topic:
   *
   * @param _topic The topic name to check
   *
   * @return number of subscribers
   */
  uint32_t getNumSubscribers(const std::string &_topic);

  size_t getNumSubscriptions();

  /**
   * \brief Return the number of publishers connected to this node on a particular topic
   *
   * \param _topic the topic name to check
   * \return the number of subscribers
   */
  size_t getNumPublishers(const std::string &_topic);

  template<typename M>
  void publish(const std::string& topic, const M& message)
  {
    using namespace serialization;

    SerializedMessage m;
    publish(topic, [&message](){return serializeMessage<M>(message);}, m);
  }

  void publish(const std::string &_topic, const std::function<SerializedMessage(void)>& serfunc, SerializedMessage& m);

  void incrementSequence(const std::string &_topic);
  bool isLatched(const std::string& topic);

  MasterLinkPtr getMasterLink() const;

private:
  /** if it finds a pre-existing subscription to the same topic and of the
   *  same message type, it appends the Functor to the callback vector for
   *  that subscription. otherwise, it returns false, indicating that a new
   *  subscription needs to be created.
   */
  bool addSubCallback(const SubscribeOptions& ops);

  /** @brief Request a topic
   *
   * Negotiate a subscriber connection on a topic.
   *
   * @param caller
   * @param topic The topic of interest.
   * @param protos List of transport protocols, in preference order
   * @param conn Contains informaton about requesting client
   *
   * @return true on success, false otherwise
   *
   */
  RpcValue requestTopic(const std::string& caller, const std::string &topic, const RpcValue &protos, const network::ClientInfo& conn);

  // Must lock the advertised topics mutex before calling this function
  bool isTopicAdvertised(const std::string& topic);

  /// Send XMLRPC request to register publisher.
  /// It will receive and process a list of subscribers from master.
  bool registerSubscriber(const SubscriptionPtr& s, const std::string& datatype);

  /// Send XMLRPC request to unregister subscriber.
  bool unregisterSubscriber(const std::string& topic);

  /// Send RPC request to unregister publisher.
  bool unregisterPublisher(const std::string& topic);

  PublicationPtr lookupPublicationWithoutLock(const std::string &topic);

  void processPublishQueues();

  /** @brief Compute the statistics for the node's connectivity
   *
   * This is the implementation of the xml-rpc getBusStats function;
   * it populates the XmlRpcValue object sent to it with various statistics
   * about the node's connectivity, bandwidth utilization, etc.
   */
  void getBusStats(XmlRpc::XmlRpcValue &stats);

  /** @brief Compute the info for the node's connectivity
   *
   * This is the implementation of the xml-rpc getBusInfo function;
   * it populates the XmlRpcValue object sent to it with various info
   * about the node's connectivity.
   */
  void getBusInfo(XmlRpc::XmlRpcValue &info);

  /** @brief Return the list of subcriptions for the node
   *
   * This is the implementation of the xml-rpc getSubscriptions
   * function; it populates the XmlRpcValue object sent to it with the
   * list of subscribed topics and their datatypes.
   */
  void getSubscriptions(XmlRpc::XmlRpcValue &subscriptions);

  /** @brief Return the list of advertised topics for the node
   *
   * This is the implementation of the xml-rpc getPublications
   * function; it populates the XmlRpcValue object sent to it with the
   * list of advertised topics and their datatypes.
   */
  void getPublications(XmlRpc::XmlRpcValue &publications);

  /** @brief Update local publisher lists.
   *
   * Use this method to update address information for publishers on a
   * given topic.
   *
   * @param topic The topic of interest
   * @param pubs The list of publishers to update.
   *
   * @return true on success, false otherwise.
   */
  bool pubUpdate(const std::string &topic, const std::vector<std::string> &pubs);

  RpcValue pubUpdateCallback(const std::string& callerId, const std::string& topic, const RpcValue& publishers, const network::ClientInfo& ci);

  void getBusStatsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void getBusInfoCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void getSubscriptionsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);
  void getPublicationsCallback(const XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result);

  bool isShuttingDown() { return shutting_down_; }

  std::mutex subs_mutex_;
  std::list<SubscriptionPtr> subscriptions_;

  std::recursive_mutex advertised_topics_mutex_;
  V_Publication advertised_topics_;
  std::list<std::string> advertised_topic_names_;
  std::mutex advertised_topic_names_mutex_;

  volatile bool shutting_down_;
  std::mutex shutting_down_mutex_;

  PollManagerPtr poll_manager_;
  class PollWatcher;
  std::unique_ptr<PollWatcher> poll_watcher_;

  ConnectionManagerPtr connection_manager_;
  RPCManagerPtr rpc_manager_;

  MasterLinkPtr master_link_;

  /// Provide direct IP for clients.
  bool resolve_ip_;
};

} // namespace miniros

#endif // MINIROS_TOPIC_MANAGER_H
