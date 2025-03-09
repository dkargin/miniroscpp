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

#define MINIROS_PACKAGE_NAME "master_link"

#include "miniros/master_link.h"
#include "miniros/init.h"
#include "miniros/names.h"
#include "miniros/this_node.h"
#include "miniros/transport/network.h"
#include "miniros/transport/rpc_manager.h"
#include <miniros/console.h>
#include <miniros/rosassert.h>

#include "miniros/xmlrpcpp/XmlRpc.h"

namespace miniros {

struct MasterLink::Internal {
  std::shared_ptr<RPCManager> rpcManager;

  uint32_t port = 0;
  std::string host;
  std::string uri;
  WallDuration retry_timeout;

  /// Local cache of parameters.
  std::map<std::string, RpcValue> params;
  std::mutex params_mutex;
  std::set<std::string> subscribed_params;

#if defined(__APPLE__)
  std::mutex xmlrpc_call_mutex;
#endif

  Internal(const std::shared_ptr<RPCManager>& rpcManager) : rpcManager(rpcManager) {}
};

MasterLink::MasterLink(const std::shared_ptr<RPCManager>& rpcManager) : internal_(nullptr)
{
  internal_ = new Internal(rpcManager);
}

MasterLink::~MasterLink()
{
  if (internal_) {
    delete internal_;
    internal_ = nullptr;
  }
}

Error MasterLink::initLink(const M_string& remappings)
{
  if (!internal_)
    return Error::InternalError;

  auto it = remappings.find("__master");
  if (it != remappings.end()) {
    internal_->uri = it->second;
  }

  if (internal_->uri.empty()) {
    char* master_uri_env = NULL;
#ifdef _MSC_VER
    _dupenv_s(&master_uri_env, NULL, "ROS_MASTER_URI");
#else
    master_uri_env = getenv("ROS_MASTER_URI");
#endif
    if (master_uri_env) {
      internal_->uri = master_uri_env;
    } else {
      MINIROS_WARN("Defaulting ROS_MASTER_URI to localhost:11311");
      internal_->uri = "http://localhost:11311";
    }
#ifdef _MSC_VER
    // http://msdn.microsoft.com/en-us/library/ms175774(v=vs.80).aspx
    free(master_uri_env);
#endif
  }

  // Split URI into
  if (!network::splitURI(internal_->uri, internal_->host, internal_->port)) {
    MINIROS_FATAL("Couldn't parse the master URI [%s] into a host:port pair.", internal_->uri.c_str());
    return Error::InvalidURI;
  }

  return Error::Ok;
}

std::string MasterLink::getHost() const
{
  return internal_->host;
}

uint32_t MasterLink::getPort() const
{
  return internal_ ? internal_->port : 0;
}

std::string MasterLink::getURI() const
{
  return internal_ ? internal_->uri : "";
}

void MasterLink::setRetryTimeout(miniros::WallDuration timeout)
{
  if (!internal_)
    return;
  if (timeout < miniros::WallDuration(0)) {
    MINIROS_FATAL("retry timeout must not be negative.");
    MINIROS_BREAK();
  }
  internal_->retry_timeout = timeout;
}

bool MasterLink::check() const
{
  RpcValue args, result, payload;
  args[0] = this_node::getName();
  return execute("getPid", args, result, payload, false);
}

bool MasterLink::getTopics(std::vector<TopicInfo>& topics) const
{
  RpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ""; // TODO: Fix this

  if (!execute("getPublishedTopics", args, result, payload, true)) {
    return false;
  }

  topics.clear();
  for (size_t i = 0; i < payload.size(); i++) {
    topics.push_back(TopicInfo(std::string(payload[i][0]), std::string(payload[i][1])));
  }

  return true;
}

bool MasterLink::getNodes(std::vector<std::string>& nodes) const
{
  RpcValue args, result, payload;
  args[0] = this_node::getName();

  if (!execute("getSystemState", args, result, payload, true)) {
    return false;
  }

  std::set<std::string> node_set;
  for (int i = 0; i < payload.size(); ++i) {
    for (int j = 0; j < payload[i].size(); ++j) {
      RpcValue val = payload[i][j][1];
      for (int k = 0; k < val.size(); ++k) {
        std::string name = payload[i][j][1][k];
        node_set.insert(name);
      }
    }
  }

  nodes.clear();
  nodes.insert(nodes.end(), node_set.begin(), node_set.end());

  return true;
}

Error MasterLink::execute(const std::string& method, const RpcValue& request, RpcValue& response,
  RpcValue& payload, bool wait_for_master) const
{
  if (!internal_)
    return Error::InternalError;
  miniros::SteadyTime start_time = miniros::SteadyTime::now();

  std::string master_host = getHost();
  uint32_t master_port = getPort();
  RPCManagerPtr manager = internal_->rpcManager;
  if (!manager)
    return Error::InternalError;
  XmlRpc::XmlRpcClient* c = manager->getXMLRPCClient(master_host, master_port, "/");
  bool printed = false;
  bool slept = false;
  bool ok = true;
  bool b = false;
  do {
    {
#if defined(__APPLE__)
      std::scoped_lock<std::mutex> lock(internal_->xmlrpc_call_mutex);
#endif

      b = c->execute(method.c_str(), request, response);
    }

    ok = !miniros::isShuttingDown() && !manager->isShuttingDown();

    if (!b && ok) {
      if (!printed && wait_for_master) {
        MINIROS_ERROR("[%s] Failed to contact master at [%s:%d].  %s", method.c_str(), master_host.c_str(), master_port,
          wait_for_master ? "Retrying..." : "");
        printed = true;
      }

      if (!wait_for_master) {
        manager->releaseXMLRPCClient(c);
        return Error::NoMaster;
      }

      if (!internal_->retry_timeout.isZero() && (miniros::SteadyTime::now() - start_time) >= internal_->retry_timeout) {
        MINIROS_ERROR(
          "[%s] Timed out trying to connect to the master after [%f] seconds", method.c_str(), internal_->retry_timeout.toSec());
        manager->releaseXMLRPCClient(c);
        return Error::NoMaster;
      }

      miniros::WallDuration(0.05).sleep();
      slept = true;
    } else {
      if (!manager->validateXmlrpcResponse(method, response, payload)) {
        manager->releaseXMLRPCClient(c);
        return Error::InvalidResponse;
      }
      break;
    }

    ok = !miniros::isShuttingDown() && !manager->isShuttingDown();
  } while (ok);

  if (ok && slept) {
    MINIROS_INFO("Connected to master at [%s:%d]", master_host.c_str(), master_port);
  }

  manager->releaseXMLRPCClient(c);

  if (!ok) {
    MINIROS_ERROR("[%s] Got shutdown request during RPC call", method.c_str());
    return Error::ShutdownInterrupt;
  }
  return Error::Ok;;
}

void MasterLink::invalidateParentParams(const std::string& key)
{
  if (!internal_)
    return;
  std::string ns_key = names::parentNamespace(key);
  while (ns_key != "" && ns_key != "/") {
    if (internal_->subscribed_params.find(ns_key) != internal_->subscribed_params.end()) {
      // by erasing the key the parameter will be re-queried
      internal_->params.erase(ns_key);
    }
    ns_key = names::parentNamespace(ns_key);
  }
}

void MasterLink::set(const std::string& key, const RpcValue& v)
{
  if (!internal_)
    return;

  std::string mapped_key = miniros::names::resolve(key);

  RpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  params[2] = v;

  {
    // Lock around the execute to the master in case we get a parameter update on this value between
    // executing on the master and setting the parameter in the g_params list.
    std::scoped_lock<std::mutex> lock(internal_->params_mutex);

    if (this->execute("setParam", params, result, payload, true)) {
      // Update our cached params list now so that if get() is called immediately after param::set()
      // we already have the cached state and our value will be correct
      if (internal_->subscribed_params.find(mapped_key) != internal_->subscribed_params.end()) {
        internal_->params[mapped_key] = v;
      }
      invalidateParentParams(mapped_key);
    }
  }
}

void MasterLink::set(const std::string& key, const std::string& s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  RpcValue v(s);
  set(key, v);
}

void MasterLink::set(const std::string& key, const char* s)
{
  // construct xmlrpc_c::value object of the std::string and
  // call param::set(key, xmlvalue);
  std::string sxx = std::string(s);
  RpcValue v(sxx);
  set(key, v);
}

void MasterLink::set(const std::string& key, double d)
{
  RpcValue v(d);
  set(key, v);
}

void MasterLink::set(const std::string& key, int i)
{
  RpcValue v(i);
  set(key, v);
}

void MasterLink::set(const std::string& key, bool b)
{
  RpcValue v(b);
  set(key, v);
}

template <class T>
void MasterLink::setParamImpl(const std::string& key, const std::vector<T>& vec)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertArray turns it
  // into an array type with the given size
  RpcValue xml_vec;
  xml_vec.setSize(vec.size());

  // Copy the contents into the XmlRpcValue
  for (size_t i = 0; i < vec.size(); i++) {
    xml_vec[i] = vec.at(i);
  }

  this->set(key, xml_vec);
}

void MasterLink::set(const std::string& key, const std::vector<std::string>& vec)
{
  setParamImpl(key, vec);
}

void MasterLink::set(const std::string& key, const std::vector<double>& vec)
{
  setParamImpl(key, vec);
}

void MasterLink::set(const std::string& key, const std::vector<float>& vec)
{
  setParamImpl(key, vec);
}

void MasterLink::set(const std::string& key, const std::vector<int>& vec)
{
  setParamImpl(key, vec);
}

void MasterLink::set(const std::string& key, const std::vector<bool>& vec)
{
  setParamImpl(key, vec);
}

template <class T>
void MasterLink::setParamImpl(const std::string& key, const std::map<std::string, T>& map)
{
  // Note: the XmlRpcValue starts off as "invalid" and assertStruct turns it
  // into a struct type
  RpcValue xml_value;
  xml_value.begin();

  // Copy the contents into the XmlRpcValue
  for (typename std::map<std::string, T>::const_iterator it = map.begin(); it != map.end(); ++it) {
    xml_value[it->first] = it->second;
  }

  this->set(key, xml_value);
}

void MasterLink::set(const std::string& key, const std::map<std::string, std::string>& map)
{
  setParamImpl(key, map);
}

void MasterLink::set(const std::string& key, const std::map<std::string, double>& map)
{
  setParamImpl(key, map);
}

void MasterLink::set(const std::string& key, const std::map<std::string, float>& map)
{
  setParamImpl(key, map);
}

void MasterLink::set(const std::string& key, const std::map<std::string, int>& map)
{
  setParamImpl(key, map);
}

void MasterLink::set(const std::string& key, const std::map<std::string, bool>& map)
{
  setParamImpl(key, map);
}

bool MasterLink::has(const std::string& key)
{
  RpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = miniros::names::resolve(key);
  // params[1] = key;
  //  We don't loop here, because validateXmlrpcResponse() returns false
  //  both when we can't contact the master and when the master says, "I
  //  don't have that param."
  if (!this->execute("hasParam", params, result, payload, false)) {
    return false;
  }

  return payload;
}

bool MasterLink::del(const std::string& key)
{
  if (!internal_)
    return false;
  std::string mapped_key = miniros::names::resolve(key);

  {
    std::scoped_lock<std::mutex> lock(internal_->params_mutex);

    internal_->subscribed_params.erase(mapped_key);
    internal_->params.erase(mapped_key);
  }

  RpcValue params, result, payload;
  params[0] = this_node::getName();
  params[1] = mapped_key;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!this->execute("deleteParam", params, result, payload, false)) {
    return false;
  }

  return true;
}

bool MasterLink::getParamImpl(const std::string& key, RpcValue& v, bool use_cache)
{
  if (!internal_)
    return false;
  std::string mapped_key = miniros::names::resolve(key);
  if (mapped_key.empty())
    mapped_key = "/";

  if (use_cache) {
    std::scoped_lock<std::mutex> lock(internal_->params_mutex);

    if (internal_->subscribed_params.find(mapped_key) != internal_->subscribed_params.end()) {
      auto it = internal_->params.find(mapped_key);
      if (it != internal_->params.end()) {
        if (it->second.valid()) {
          MINIROS_DEBUG_NAMED("cached_parameters", "Using cached parameter value for key [%s]", mapped_key.c_str());

          v = it->second;
          return true;
        } else {
          MINIROS_DEBUG_NAMED("cached_parameters", "Cached parameter is invalid for key [%s]", mapped_key.c_str());
          return false;
        }
      }
    } else {
      // parameter we've never seen before, register for update from the master
      if (internal_->subscribed_params.insert(mapped_key).second) {
        RpcValue params, result, payload;
        params[0] = this_node::getName();
        params[1] = internal_->rpcManager->getServerURI();
        params[2] = mapped_key;

        if (!this->execute("subscribeParam", params, result, payload, false)) {
          MINIROS_DEBUG_NAMED(
            "cached_parameters", "Subscribe to parameter [%s]: call to the master failed", mapped_key.c_str());
          internal_->subscribed_params.erase(mapped_key);
          use_cache = false;
        } else {
          MINIROS_DEBUG_NAMED("cached_parameters", "Subscribed to parameter [%s]", mapped_key.c_str());
        }
      }
    }
  }

  RpcValue params, result;
  params[0] = this_node::getName();
  params[1] = mapped_key;

  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  bool ret = this->execute("getParam", params, result, v, false);

  if (use_cache) {
    std::scoped_lock<std::mutex> lock(internal_->params_mutex);

    MINIROS_DEBUG_NAMED(
      "cached_parameters", "Caching parameter [%s] with value type [%d]", mapped_key.c_str(), v.getType());
    internal_->params[mapped_key] = v;
  }

  return ret;
}

bool MasterLink::getParamImpl(const std::string& key, std::string& s, bool use_cache)
{
  RpcValue v;
  if (!getParamImpl(key, v, use_cache))
    return false;
  if (v.getType() != RpcValue::TypeString)
    return false;
  s = std::string(v);
  return true;
}

bool MasterLink::getParamImpl(const std::string& key, double& d, bool use_cache)
{
  RpcValue v;
  if (!getParamImpl(key, v, use_cache)) {
    return false;
  }

  if (v.getType() == RpcValue::TypeInt) {
    d = (int)v;
  } else if (v.getType() != RpcValue::TypeDouble) {
    return false;
  } else {
    d = v;
  }

  return true;
}

bool MasterLink::getParamImpl(const std::string& key, float& f, bool use_cache)
{
  double d = static_cast<double>(f);
  bool result = getParamImpl(key, d, use_cache);
  if (result)
    f = static_cast<float>(d);
  return result;
}

bool MasterLink::getParamImpl(const std::string& key, int& i, bool use_cache)
{
  RpcValue v;
  if (!getParamImpl(key, v, use_cache)) {
    return false;
  }

  if (v.getType() == RpcValue::TypeDouble) {
    double d = v;

    if (fmod(d, 1.0) < 0.5) {
      d = floor(d);
    } else {
      d = ceil(d);
    }
    i = d;
  } else if (v.getType() != RpcValue::TypeInt) {
    return false;
  } else {
    i = v;
  }

  return true;
}

bool MasterLink::getParamImpl(const std::string& key, bool& b, bool use_cache)
{
  RpcValue v;
  if (!getParamImpl(key, v, use_cache))
    return false;
  if (v.getType() != RpcValue::TypeBoolean)
    return false;
  b = v;
  return true;
}

bool MasterLink::get(const std::string& key, std::string& s)
{
  return getParamImpl(key, s, false);
}

bool MasterLink::get(const std::string& key, double& d)
{
  return getParamImpl(key, d, false);
}

bool MasterLink::get(const std::string& key, float& f)
{
  return getParamImpl(key, f, false);
}

bool MasterLink::get(const std::string& key, int& i)
{
  return getParamImpl(key, i, false);
}

bool MasterLink::get(const std::string& key, bool& b)
{
  return getParamImpl(key, b, false);
}

bool MasterLink::get(const std::string& key, RpcValue& v)
{
  return getParamImpl(key, v, false);
}

bool MasterLink::getCached(const std::string& key, std::string& s)
{
  return getParamImpl(key, s, true);
}

bool MasterLink::getCached(const std::string& key, double& d)
{
  return getParamImpl(key, d, true);
}

bool MasterLink::getCached(const std::string& key, float& f)
{
  return getParamImpl(key, f, true);
}

bool MasterLink::getCached(const std::string& key, int& i)
{
  return getParamImpl(key, i, true);
}

bool MasterLink::getCached(const std::string& key, bool& b)
{
  return getParamImpl(key, b, true);
}

bool MasterLink::getCached(const std::string& key, RpcValue& v)
{
  return getParamImpl(key, v, true);
}

template <class T> T xml_cast(MasterLink::RpcValue xml_value)
{
  return static_cast<T>(xml_value);
}

template <class T> bool xml_castable(int XmlType)
{
  return false;
}

template <> bool xml_castable<std::string>(int XmlType)
{
  return XmlType == MasterLink::RpcValue::TypeString;
}

template <> bool xml_castable<double>(int XmlType)
{
  return (XmlType == MasterLink::RpcValue::TypeDouble || XmlType == MasterLink::RpcValue::TypeInt ||
          XmlType == MasterLink::RpcValue::TypeBoolean);
}

template <> bool xml_castable<float>(int XmlType)
{
  return (XmlType == MasterLink::RpcValue::TypeDouble || XmlType == MasterLink::RpcValue::TypeInt ||
          XmlType == MasterLink::RpcValue::TypeBoolean);
}

template <> bool xml_castable<int>(int XmlType)
{
  return (XmlType == MasterLink::RpcValue::TypeDouble || XmlType == MasterLink::RpcValue::TypeInt ||
          XmlType == MasterLink::RpcValue::TypeBoolean);
}

template <> bool xml_castable<bool>(int XmlType)
{
  return (XmlType == MasterLink::RpcValue::TypeDouble || XmlType == MasterLink::RpcValue::TypeInt ||
          XmlType == MasterLink::RpcValue::TypeBoolean);
}

template <> double xml_cast(MasterLink::RpcValue xml_value)
{
  using namespace XmlRpc;
  switch (xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<double>(xml_value);
    case XmlRpcValue::TypeInt:
      return static_cast<double>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<double>(static_cast<bool>(xml_value));
    default:
      return 0.0;
  };
}

template <> float xml_cast(MasterLink::RpcValue xml_value)
{
  using namespace XmlRpc;
  switch (xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<float>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<float>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<float>(static_cast<bool>(xml_value));
    default:
      return 0.0f;
  };
}

template <> int xml_cast(MasterLink::RpcValue xml_value)
{
  using namespace XmlRpc;
  switch (xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<int>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<int>(xml_value);
    case XmlRpcValue::TypeBoolean:
      return static_cast<int>(static_cast<bool>(xml_value));
    default:
      return 0;
  };
}

template <> bool xml_cast(MasterLink::RpcValue xml_value)
{
  using namespace XmlRpc;
  switch (xml_value.getType()) {
    case XmlRpcValue::TypeDouble:
      return static_cast<bool>(static_cast<double>(xml_value));
    case XmlRpcValue::TypeInt:
      return static_cast<bool>(static_cast<int>(xml_value));
    case XmlRpcValue::TypeBoolean:
      return static_cast<bool>(xml_value);
    default:
      return false;
  };
}

template <class T> bool MasterLink::getParamImpl(const std::string& key, std::vector<T>& vec, bool cached)
{
  RpcValue xml_array;
  if (!getParamImpl(key, xml_array, cached)) {
    return false;
  }

  // Make sure it's an array type
  if (xml_array.getType() != RpcValue::TypeArray) {
    return false;
  }

  // Resize the target vector (destructive)
  vec.resize(xml_array.size());

  // Fill the vector with stuff
  for (int i = 0; i < xml_array.size(); i++) {
    if (!xml_castable<T>(xml_array[i].getType())) {
      return false;
    }

    vec[i] = xml_cast<T>(xml_array[i]);
  }

  return true;
}

bool MasterLink::get(const std::string& key, std::vector<std::string>& vec)
{
  return getParamImpl(key, vec, false);
}
bool MasterLink::get(const std::string& key, std::vector<double>& vec)
{
  return getParamImpl(key, vec, false);
}
bool MasterLink::get(const std::string& key, std::vector<float>& vec)
{
  return getParamImpl(key, vec, false);
}
bool MasterLink::get(const std::string& key, std::vector<int>& vec)
{
  return getParamImpl(key, vec, false);
}
bool MasterLink::get(const std::string& key, std::vector<bool>& vec)
{
  return getParamImpl(key, vec, false);
}

bool MasterLink::getCached(const std::string& key, std::vector<std::string>& vec)
{
  return getParamImpl(key, vec, true);
}
bool MasterLink::getCached(const std::string& key, std::vector<double>& vec)
{
  return getParamImpl(key, vec, true);
}
bool MasterLink::getCached(const std::string& key, std::vector<float>& vec)
{
  return getParamImpl(key, vec, true);
}
bool MasterLink::getCached(const std::string& key, std::vector<int>& vec)
{
  return getParamImpl(key, vec, true);
}
bool MasterLink::getCached(const std::string& key, std::vector<bool>& vec)
{
  return getParamImpl(key, vec, true);
}

template <class T> bool MasterLink::getParamImpl(const std::string& key, std::map<std::string, T>& map, bool cached)
{
  RpcValue xml_value;
  if (!getParamImpl(key, xml_value, cached)) {
    return false;
  }

  // Make sure it's a struct type
  if (xml_value.getType() != RpcValue::TypeStruct) {
    return false;
  }

  // Fill the map with stuff
  for (auto it = xml_value.begin(); it != xml_value.end(); ++it) {
    // Make sure this element is the right type
    if (!xml_castable<T>(it->second.getType())) {
      return false;
    }
    // Store the element
    map[it->first] = xml_cast<T>(it->second);
  }

  return true;
}

bool MasterLink::get(const std::string& key, std::map<std::string, std::string>& map)
{
  return getParamImpl(key, map, false);
}
bool MasterLink::get(const std::string& key, std::map<std::string, double>& map)
{
  return getParamImpl(key, map, false);
}
bool MasterLink::get(const std::string& key, std::map<std::string, float>& map)
{
  return getParamImpl(key, map, false);
}
bool MasterLink::get(const std::string& key, std::map<std::string, int>& map)
{
  return getParamImpl(key, map, false);
}
bool MasterLink::get(const std::string& key, std::map<std::string, bool>& map)
{
  return getParamImpl(key, map, false);
}

bool MasterLink::getCached(const std::string& key, std::map<std::string, std::string>& map)
{
  return getParamImpl(key, map, true);
}
bool MasterLink::getCached(const std::string& key, std::map<std::string, double>& map)
{
  return getParamImpl(key, map, true);
}
bool MasterLink::getCached(const std::string& key, std::map<std::string, float>& map)
{
  return getParamImpl(key, map, true);
}
bool MasterLink::getCached(const std::string& key, std::map<std::string, int>& map)
{
  return getParamImpl(key, map, true);
}
bool MasterLink::getCached(const std::string& key, std::map<std::string, bool>& map)
{
  return getParamImpl(key, map, true);
}

bool MasterLink::getParamNames(std::vector<std::string>& keys)
{
  RpcValue params, result, payload;
  params[0] = this_node::getName();
  if (!this->execute("getParamNames", params, result, payload, false)) {
    return false;
  }
  // Make sure it's an array type
  if (result.getType() != RpcValue::TypeArray) {
    return false;
  }
  // Make sure it returned 3 elements
  if (result.size() != 3) {
    return false;
  }
  // Get the actual parameter keys
  RpcValue parameters = result[2];
  // Resize the output
  keys.resize(parameters.size());

  // Fill the output vector with the answer
  for (int i = 0; i < parameters.size(); ++i) {
    if (parameters[i].getType() != RpcValue::TypeString) {
      return false;
    }
    keys[i] = std::string(parameters[i]);
  }
  return true;
}

bool MasterLink::search(const std::string& key, std::string& result_out)
{
  return search(this_node::getName(), key, result_out);
}

bool MasterLink::search(const std::string& ns, const std::string& key, std::string& result_out)
{
  RpcValue params, result, payload;
  params[0] = ns;

  // searchParam needs a separate form of remapping -- remapping on the unresolved name, rather than the
  // resolved one.

  std::string remapped = key;
  auto it = names::getUnresolvedRemappings().find(key);
  if (it != names::getUnresolvedRemappings().end()) {
    remapped = it->second;
  }

  params[1] = remapped;
  // We don't loop here, because validateXmlrpcResponse() returns false
  // both when we can't contact the master and when the master says, "I
  // don't have that param."
  if (!this->execute("searchParam", params, result, payload, false)) {
    return false;
  }

  result_out = (std::string)payload;

  return true;
}

void MasterLink::update(const std::string& key, const RpcValue& v)
{
  if (!internal_)
    return;
  std::string clean_key = names::clean(key);
  MINIROS_DEBUG_NAMED("cached_parameters", "Received parameter update for key [%s]", clean_key.c_str());

  std::scoped_lock<std::mutex> lock(internal_->params_mutex);

  if (internal_->subscribed_params.find(clean_key) != internal_->subscribed_params.end()) {
    internal_->params[clean_key] = v;
  }
  invalidateParentParams(clean_key);
}

void MasterLink::paramUpdateCallback(const RpcValue& params, RpcValue& result)
{
  result[0] = 1;
  result[1] = std::string("");
  result[2] = 0;

  this->update((std::string)params[1], params[2]);
}

Error MasterLink::initParam(const M_string& remappings)
{
  auto it = remappings.begin();
  auto end = remappings.end();
  for (; it != end; ++it) {
    const std::string& name = it->first;
    const std::string& param = it->second;

    if (name.size() < 2) {
      continue;
    }

    if (name[0] == '_' && name[1] != '_') {
      std::string local_name = "~" + name.substr(1);

      bool success = false;

      try {
        auto i = std::stoi(param);
        this->set(names::resolve(local_name), i);
        success = true;
      } catch (std::invalid_argument&) {
      } catch (std::out_of_range&) {
      }

      if (success) {
        continue;
      }

      try {
        double d = std::stod(param);
        this->set(names::resolve(local_name), d);
        success = true;
      } catch (std::invalid_argument&) {
      } catch (std::out_of_range&) {
      }

      if (success) {
        continue;
      }

      if (param == "true" || param == "True" || param == "TRUE") {
        this->set(names::resolve(local_name), true);
      } else if (param == "false" || param == "False" || param == "FALSE") {
        this->set(names::resolve(local_name), false);
      } else {
        this->set(names::resolve(local_name), param);
      }
    }
  }

  if (internal_->rpcManager) {
    internal_->rpcManager->bind("paramUpdate",
      [this](const RpcValue& params, RpcValue& result) {
        return paramUpdateCallback(params, result);
      });
  }
  return Error::Ok;
}

} // namespace miniros
