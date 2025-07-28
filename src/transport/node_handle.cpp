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

#include "miniros/node_handle.h"
#include "miniros/this_node.h"
#include "miniros/service.h"
#include "miniros/transport/callback_queue.h"

#include "miniros/rostime.h"
#include "miniros/timer.h"
#include "miniros/wall_timer.h"
#include "miniros/steady_timer.h"

#include "miniros/transport/rpc_manager.h"
#include "miniros/transport/topic_manager.h"
#include "miniros/transport/service_manager.h"
#include "miniros/master_link.h"
#include "miniros/names.h"
#include "miniros/init.h"
#include "miniros/xmlrpcpp/XmlRpc.h"

#include <thread>
#include <sstream>

namespace miniros
{

std::mutex g_nh_refcount_mutex;
int32_t g_nh_refcount = 0;
bool g_node_started_by_nh = false;

class NodeHandleBackingCollection
{
public:
  typedef std::vector<Publisher::ImplWPtr> V_PubImpl;
  typedef std::vector<ServiceServer::ImplWPtr> V_SrvImpl;
  typedef std::vector<Subscriber::ImplWPtr> V_SubImpl;
  typedef std::vector<ServiceClient::ImplWPtr> V_SrvCImpl;
  V_PubImpl pubs_;
  V_SrvImpl srvs_;
  V_SubImpl subs_;
  V_SrvCImpl srv_cs_;

  std::mutex mutex_;

  // keep shared_ptrs to these managers to avoid assertions. Fixes #838
  TopicManagerPtr keep_alive_topic_manager;
  ServiceManagerPtr keep_alive_service_manager;

  NodeHandleBackingCollection(TopicManagerPtr tm, ServiceManagerPtr sm)
  {
      keep_alive_topic_manager = tm;
      keep_alive_service_manager = sm;
  }
};

NodeHandle::NodeHandle(const std::string& ns, const M_string& remappings)
  : namespace_(this_node::getNamespace())
  , callback_queue_(0)
  , collection_(0)
{
  std::string tilde_resolved_ns;
  if (!ns.empty() && ns[0] == '~')// starts with tilde
    tilde_resolved_ns = names::resolve(ns);
  else
    tilde_resolved_ns = ns;

  construct(tilde_resolved_ns, true);

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns)
: collection_(0)
{
  namespace_ = parent.getNamespace();
  callback_queue_ = parent.callback_queue_;

  remappings_ = parent.remappings_;
  unresolved_remappings_ = parent.unresolved_remappings_;

  construct(ns, false);
}

NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns, const M_string& remappings)
: collection_(0)
{
  namespace_ = parent.getNamespace();
  callback_queue_ = parent.callback_queue_;

  remappings_ = parent.remappings_;
  unresolved_remappings_ = parent.unresolved_remappings_;

  construct(ns, false);

  initRemappings(remappings);
}

NodeHandle::NodeHandle(const NodeHandle& rhs)
: collection_(0)
{
  callback_queue_ = rhs.callback_queue_;
  remappings_ = rhs.remappings_;
  unresolved_remappings_ = rhs.unresolved_remappings_;

  construct(rhs.namespace_, true); 

  unresolved_namespace_ = rhs.unresolved_namespace_;
}

NodeHandle::~NodeHandle()
{
  destruct();
}

NodeHandle& NodeHandle::operator=(const NodeHandle& rhs)
{
  MINIROS_ASSERT(collection_);
  namespace_ = rhs.namespace_;
  callback_queue_ = rhs.callback_queue_;
  remappings_ = rhs.remappings_;
  unresolved_remappings_ = rhs.unresolved_remappings_;

  return *this;
}

void spinThread()
{
  miniros::spin();
}

void NodeHandle::construct(const std::string& ns, bool validate_name)
{
  if (!miniros::isInitialized())
  {
    MINIROS_FATAL("You must call miniros::init() before creating the first NodeHandle");
    MINIROS_BREAK();
  }

  collection_ = new NodeHandleBackingCollection(getTopicManager(), getServiceManager());
  unresolved_namespace_ = ns;
  // if callback_queue_ is nonnull, we are in a non-nullary constructor

  if (validate_name)
    namespace_ = resolveName(ns, true);
  else
    {
      namespace_ = resolveName(ns, true, no_validate());
      // FIXME validate namespace_ now
    }
  ok_ = true;

  std::scoped_lock<std::mutex> lock(g_nh_refcount_mutex);

  if (g_nh_refcount == 0 && !miniros::isStarted())
  {
    g_node_started_by_nh = true;
    miniros::start();
  }

  ++g_nh_refcount;
}

void NodeHandle::destruct()
{
  delete collection_;

  std::scoped_lock<std::mutex> lock(g_nh_refcount_mutex);

  --g_nh_refcount;

  if (g_nh_refcount == 0 && g_node_started_by_nh)
  {
    miniros::shutdown();
  }
}

void NodeHandle::initRemappings(const M_string& remappings)
{
  {
    M_string::const_iterator it = remappings.begin();
    M_string::const_iterator end = remappings.end();
    for (; it != end; ++it)
    {
      const std::string& from = it->first;
      const std::string& to = it->second;

      remappings_.insert(std::make_pair(resolveName(from, false), resolveName(to, false)));
      unresolved_remappings_.insert(std::make_pair(from, to));
    }
  }
}

void NodeHandle::setCallbackQueue(CallbackQueueInterface* queue)
{
  callback_queue_ = queue;
}

std::string NodeHandle::remapName(const std::string& name) const
{
  std::string resolved = resolveName(name, false);

  // First search any remappings that were passed in specifically for this NodeHandle
  M_string::const_iterator it = remappings_.find(resolved);
  if (it != remappings_.end())
  {
    // MINIROS_DEBUG("found 'local' remapping: %s", it->second.c_str());
    return it->second;
  }

  // If not in our local remappings, perhaps in the global ones
  return names::remap(resolved);
}

std::string NodeHandle::resolveName(const std::string& name, bool remap) const
{
  // MINIROS_DEBUG("resolveName(%s, %s)", name.c_str(), remap ? "true" : "false");
  std::string error;
  if (!names::validate(name, error))
  {
    throw InvalidNameException(error);
  }

  return resolveName(name, remap, no_validate());
}

std::string NodeHandle::resolveName(const std::string& name, bool remap, no_validate) const
{
  if (name.empty())
  {
    return namespace_;
  }

  std::string final = name;

  if (final[0] == '~')
  {
    std::stringstream ss;
    ss << "Using ~ names with NodeHandle methods is not allowed.  If you want to use private names with the NodeHandle ";
    ss << "interface, construct a NodeHandle using a private name as its namespace.  e.g. ";
    ss << "miniros::NodeHandle nh(\"~\");  ";
    ss << "nh.getParam(\"my_private_name\");";
    ss << " (name = [" << name << "])";
    throw InvalidNameException(ss.str());
  }
  else if (final[0] == '/')
  {
    // do nothing
  }
  else if (!namespace_.empty())
  {
    // MINIROS_DEBUG("Appending namespace_ (%s)", namespace_.c_str());
    final = names::append(namespace_, final);
  }

  // MINIROS_DEBUG("resolveName, pre-clean: %s", final.c_str());
  final = names::clean(final);
  // MINIROS_DEBUG("resolveName, post-clean: %s", final.c_str());

  if (remap)
  {
    final = remapName(final);
    // MINIROS_DEBUG("resolveName, remapped: %s", final.c_str());
  }

  return names::resolve(final, false);
}

Publisher NodeHandle::advertise(AdvertiseOptions& ops)
{
  ops.topic = resolveName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  SubscriberCallbacksPtr callbacks(std::make_shared<SubscriberCallbacks>(ops.connect_cb, ops.disconnect_cb, 
                                                                           ops.tracked_object, ops.callback_queue));

  if (getTopicManager()->advertise(ops, callbacks))
  {
    Publisher pub(ops.topic, ops.md5sum, ops.datatype, *this, callbacks);

    {
      std::scoped_lock<std::mutex> lock(collection_->mutex_);
      collection_->pubs_.push_back(pub.impl_);
    }

    return pub;
  }

  return Publisher();
}

Subscriber NodeHandle::subscribe(SubscribeOptions& ops)
{
  ops.topic = resolveName(ops.topic);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  if (getTopicManager()->subscribe(ops))
  {
    Subscriber sub(ops.topic, *this, ops.helper);

    {
      std::scoped_lock<std::mutex> lock(collection_->mutex_);
      collection_->subs_.push_back(sub.impl_);
    }

    return sub;
  }

  return Subscriber();
}

ServiceServer NodeHandle::advertiseService(AdvertiseServiceOptions& ops)
{
  ops.service = resolveName(ops.service);
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  if (getServiceManager()->advertiseService(ops))
  {
    ServiceServer srv(ops.service, *this);

    {
      std::scoped_lock<std::mutex> lock(collection_->mutex_);
      collection_->srvs_.push_back(srv.impl_);
    }

    return srv;
  }

  return ServiceServer();
}

ServiceClient NodeHandle::serviceClient(ServiceClientOptions& ops)
{
  ops.service = resolveName(ops.service);
  ServiceClient client(ops.service, ops.persistent, ops.header, ops.md5sum);

  if (client)
  {
    std::scoped_lock<std::mutex> lock(collection_->mutex_);
    collection_->srv_cs_.push_back(client.impl_);
  }

  return client;
}

Timer NodeHandle::createTimer(Duration period, const TimerCallback& callback, 
                              bool oneshot, bool autostart) const
{
  TimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  ops.autostart = autostart;
  return createTimer(ops);
}

Timer NodeHandle::createTimer(TimerOptions& ops) const
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  Timer timer(ops);
  if (ops.autostart)
    timer.start();
  return timer;
}

WallTimer NodeHandle::createWallTimer(WallDuration period, const WallTimerCallback& callback, 
                                      bool oneshot, bool autostart) const
{
  WallTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  ops.autostart = autostart;
  return createWallTimer(ops);
}

WallTimer NodeHandle::createWallTimer(WallTimerOptions& ops) const
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  WallTimer timer(ops);
  if (ops.autostart)
    timer.start();
  return timer;
}

SteadyTimer NodeHandle::createSteadyTimer(WallDuration period, const SteadyTimerCallback& callback,
                                          bool oneshot, bool autostart) const
{
  SteadyTimerOptions ops;
  ops.period = period;
  ops.callback = callback;
  ops.oneshot = oneshot;
  ops.autostart = autostart;
  return createSteadyTimer(ops);
}

SteadyTimer NodeHandle::createSteadyTimer(SteadyTimerOptions& ops) const
{
  if (ops.callback_queue == 0)
  {
    if (callback_queue_)
    {
      ops.callback_queue = callback_queue_;
    }
    else
    {
      ops.callback_queue = getGlobalCallbackQueue();
    }
  }

  SteadyTimer timer(ops);
  if (ops.autostart)
    timer.start();
  return timer;
}

void NodeHandle::shutdown()
{
  for (auto ptr: collection_->subs_)
  {
    if (Subscriber::ImplPtr impl = ptr.lock())
      impl->unsubscribe();
  }

  for (auto ptr : collection_->pubs_)
  {
    if (Publisher::ImplPtr impl = ptr.lock())
      impl->unadvertise();
  }

  for (auto ptr: collection_->srvs_)
  {
    if (ServiceServer::ImplPtr impl = ptr.lock())
      impl->unadvertise();
  }

  for (auto ptr: collection_->srv_cs_)
  {
    if (ServiceClient::ImplPtr impl = ptr.lock())
      impl->shutdown();
  }

  ok_ = false;
}

void NodeHandle::setParam(const std::string& key, const XmlRpc::XmlRpcValue& v) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), v);
  }
}

void NodeHandle::setParam(const std::string& key, const std::string& s) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), s);
  }
}

void NodeHandle::setParam(const std::string& key, const char* s) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), s);
  }
}

void NodeHandle::setParam(const std::string& key, double d) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), d);
  }
}

void NodeHandle::setParam(const std::string& key, int i) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), i);
  }
}

void NodeHandle::setParam(const std::string& key, bool b) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), b);
  }
}

void NodeHandle::setParam(const std::string& key, const std::vector<std::string>& vec) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), vec);
  }
}
void NodeHandle::setParam(const std::string& key, const std::vector<double>& vec) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), vec);
  }
}
void NodeHandle::setParam(const std::string& key, const std::vector<float>& vec) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), vec);
  }
}
void NodeHandle::setParam(const std::string& key, const std::vector<int>& vec) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), vec);
  }
}
void NodeHandle::setParam(const std::string& key, const std::vector<bool>& vec) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), vec);
  }
}

void NodeHandle::setParam(const std::string& key, const std::map<std::string, std::string>& map) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), map);
  }
}
void NodeHandle::setParam(const std::string& key, const std::map<std::string, double>& map) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), map);
  }
}
void NodeHandle::setParam(const std::string& key, const std::map<std::string, float>& map) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), map);
  }
}
void NodeHandle::setParam(const std::string& key, const std::map<std::string, int>& map) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), map);
  }
}
void NodeHandle::setParam(const std::string& key, const std::map<std::string, bool>& map) const
{
  if (auto link = getMasterLink()) {
    link->set(resolveName(key), map);
  }
}

bool NodeHandle::hasParam(const std::string& key) const
{
  if (auto link = getMasterLink()) {
    return link->has(resolveName(key));
  }
  return false;
}

bool NodeHandle::deleteParam(const std::string& key) const
{
  if (auto link = getMasterLink()) {
    return link->del(resolveName(key));
  }
  return false;
}

bool NodeHandle::getParamNames(std::vector<std::string>& keys) const
{
  if (auto link = getMasterLink()) {
    return link->getParamNames(keys);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, XmlRpc::XmlRpcValue& v) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), v);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::string& s) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), s);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, double& d) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), d);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, float& f) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), f);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, int& i) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), i);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, bool& b) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), b);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::vector<std::string>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::vector<double>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::vector<float>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::vector<int>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::vector<bool>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::map<std::string, std::string>& map) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::map<std::string, double>& map) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::map<std::string, float>& map) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::map<std::string, int>& map) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParam(const std::string& key, std::map<std::string, bool>& map) const
{
  if (auto link = getMasterLink()) {
    return link->get(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, XmlRpc::XmlRpcValue& v) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), v);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::string& s) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), s);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, double& d) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), d);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, float& f) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), f);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, int& i) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), i);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, bool& b) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), b);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::vector<std::string>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::vector<double>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::vector<float>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::vector<int>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::vector<bool>& vec) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), vec);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::map<std::string, std::string>& map) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::map<std::string, double>& map) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::map<std::string, float>& map) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::map<std::string, int>& map) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::getParamCached(const std::string& key, std::map<std::string, bool>& map) const
{
  if (auto link = getMasterLink()) {
    return link->getCached(resolveName(key), map);
  }
  return false;
}

bool NodeHandle::searchParam(const std::string& key, std::string& result_out) const
{
  // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
  // resolved one.

  std::string remapped = key;
  M_string::const_iterator it = unresolved_remappings_.find(key);
  // First try our local remappings
  if (it != unresolved_remappings_.end())
  {
    remapped = it->second;
  }

  if (auto link = getMasterLink()) {
    return link->search(resolveName(""), remapped, result_out);
  }
  return false;
}

bool NodeHandle::ok() const
{
  return miniros::ok() && ok_;
}

TopicManagerPtr NodeHandle::getTopicManager() const {
  return TopicManager::instance();
}

ServiceManagerPtr NodeHandle::getServiceManager() const {
  return ServiceManager::instance();
}

MasterLinkPtr NodeHandle::getMasterLink() const
{
  return TopicManager::instance()->getMasterLink();
}

} // namespace miniros
