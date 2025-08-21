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

#ifndef MINIROS_MASTER_LINK_H
#define MINIROS_MASTER_LINK_H

#include "internal/forwards.h"

#include "miniros/macros.h"
#include "miniros/errors.h"

namespace XmlRpc {
class XmlRpcValue;
}

namespace miniros {

class RPCManager;

/**
 * \brief Contains information retrieved from the master about a topic
 */
struct MINIROS_DECL TopicInfo {
  TopicInfo() = default;

  TopicInfo(const std::string& _name, const std::string& _datatype /*, const std::string& _md5sum*/)
      : name(_name), datatype(_datatype)
  //, md5sum(_md5sum)
  {
  }

  std::string name;     ///< Name of the topic
  std::string datatype; ///< Datatype of the topic

  // not possible yet unfortunately (master does not have this information)
  // std::string md5sum;      ///< md5sum of the topic
};

/**
 * \brief MasterLink provides all interactions with rosmaster:
 *  - registration of subscribers and publishers
 *  - interaction with rosparam part.
 */
class MasterLink {
public:
  MINIROS_DECL MasterLink();

  MINIROS_DECL ~MasterLink();

  using RpcValue = XmlRpc::XmlRpcValue;

  /// Init connection to rosmaster.
  MINIROS_DECL Error initLink(const M_string& remappings, const std::shared_ptr<RPCManager>& rpcManager);

  /// Init rosparam part.
  MINIROS_DECL Error initParam(const M_string& remappings);

  /// Stop serving any request.
  MINIROS_DECL void disconnect();

  /** @brief Execute an XMLRPC call on the master
   *
   * @param method The RPC method to invoke
   * @param request The arguments to the RPC call
   * @param response [out] The response that was received.
   * @param payload [out] The payload that was received.
   * @param wait_for_master Whether or not this call should loop until it can contact the master
   *
   * @return true if call succeeds, false otherwise.
   */
  MINIROS_DECL Error execute(const std::string& method, const RpcValue& request,
    RpcValue& response, RpcValue& payload, bool wait_for_master) const;

  /** @brief Get the hostname where the master runs.
   *
   * @return The master's hostname, as a string
   */
  MINIROS_DECL std::string getHost() const;

  /** @brief Get the port where the master runs.
   *
   * @return The master's port.
   */
  MINIROS_DECL uint32_t getPort() const;

  /**
   * \brief Get the full URI to the master (eg. http://host:port/)
   */
  MINIROS_DECL std::string getURI() const;

  /** @brief Check whether the master is up
   *
   * This method tries to contact the master.  You can call it any time
   * after miniros::init has been called.  The intended usage is to check
   * whether the master is up before trying to make other requests
   * (subscriptions, advertisements, etc.).
   *
   * @returns true if the master is available, false otherwise.
   */
  MINIROS_DECL bool check() const;

  /** @brief Get the list of topics that are being published by all nodes.
   *
   * This method communicates with the master to retrieve the list of all
   * currently advertised topics.
   *
   * @param topics A place to store the resulting list.  Each item in the
   * list is a pair <string topic, string type>.  The type is represented
   * in the format "package_name/MessageName", and is also retrievable
   * through message.__getDataType() or MessageName::__s_getDataType().
   *
   * @return true on success, false otherwise (topics not filled in)
   */
  MINIROS_DECL bool getTopics(std::vector<TopicInfo>& topics) const;

  /**
   * \brief Retrieves the currently-known list of nodes from the master
   */
  MINIROS_DECL bool getNodes(std::vector<std::string>& nodes) const;

  /**
   * @brief Set the max time this node should spend looping trying to connect to the master
   * @param timeout - The timeout.  A negative value means infinite
   */
  MINIROS_DECL void setRetryTimeout(WallDuration timeout);

  /** \brief Set an arbitrary XML/RPC value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param v The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const RpcValue& v);

  /** \brief Set a string value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param s The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::string& s);

  /** \brief Set a string value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param s The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const char* s);

  /** \brief Set a double value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param d The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, double d);

  /** \brief Set an integer value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param i The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, int i);

  /** \brief Set a bool value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param b The value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, bool b);

  /** \brief Set a string vector value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param vec The vector value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::vector<std::string>& vec);

  /** \brief Set a double vector value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param vec The vector value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::vector<double>& vec);

  /** \brief Set a float vector value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param vec The vector value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::vector<float>& vec);

  /** \brief Set an integer  vector value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param vec The vector value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::vector<int>& vec);

  /** \brief Set a bool vector value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param vec The vector value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::vector<bool>& vec);

  /** \brief Set a string->string map value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param map The map value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::map<std::string, std::string>& map);

  /** \brief Set a string->double map value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param map The map value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::map<std::string, double>& map);

  /** \brief Set a string->float map value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param map The map value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::map<std::string, float>& map);

  /** \brief Set a string->int map value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param map The map value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::map<std::string, int>& map);

  /** \brief Set a string->bool map value on the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param map The map value to be inserted.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL Error set(const std::string& key, const std::map<std::string, bool>& map);

  /** \brief Get a string value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] s Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::string& s);

  /** \brief Get a double value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] d Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, double& d);

  /** \brief Get a float value from the parameter server (internally using the double value).
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] f Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, float& f);

  /** \brief Get an integer value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] i Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, int& i);

  /** \brief Get a boolean value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] b Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, bool& b);

  /** \brief Get an arbitrary XML/RPC value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] v Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, RpcValue& v);

  /** \brief Get a string value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] s Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::string& s);

  /** \brief Get a double value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] d Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, double& d);

  /** \brief Get a float value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] f Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, float& f);

  /** \brief Get an integer value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] i Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, int& i);

  /** \brief Get a boolean value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] b Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, bool& b);

  /** \brief Get an arbitrary XML/RPC value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] v Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, RpcValue& v);

  /** \brief Get a string vector value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::vector<std::string>& vec);

  /** \brief Get a double  vector value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::vector<double>& vec);

  /** \brief Get a float  vector value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::vector<float>& vec);

  /** \brief Get an int vector value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::vector<int>& vec);

  /** \brief Get a bool vector value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::vector<bool>& vec);

  /** \brief Get a string vector value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::vector<std::string>& vec);

  /** \brief Get a double vector value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::vector<double>& vec);

  /** \brief Get a float vector value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::vector<float>& vec);

  /** \brief Get an int vector value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::vector<int>& vec);

  /** \brief Get a bool vector value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] vec Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::vector<bool>& vec);

  /** \brief Get a string->string map value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::map<std::string, std::string>& map);

  /** \brief Get a string->double map value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::map<std::string, double>& map);

  /** \brief Get a string->float map value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::map<std::string, float>& map);

  /** \brief Get a string->int map value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::map<std::string, int>& map);

  /** \brief Get a string->bool map value from the parameter server.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool get(const std::string& key, std::map<std::string, bool>& map);

  /** \brief Get a string->string map value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::map<std::string, std::string>& map);

  /** \brief Get a string->double map value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::map<std::string, double>& map);

  /** \brief Get a string->float map value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::map<std::string, float>& map);

  /** \brief Get a string->int map value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::map<std::string, int>& map);

  /** \brief Get a string->bool map value from the parameter server, with local caching
   *
   * This function will cache parameters locally, and subscribe for updates from
   * the parameter server.  Once the parameter is retrieved for the first time
   * no subsequent getCached() calls with the same key will query the master --
   * they will instead look up in the local cache.
   *
   * \param key The key to be used in the parameter server's dictionary
   * \param[out] map Storage for the retrieved value.
   *
   * \return true if the parameter value was retrieved, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool getCached(const std::string& key, std::map<std::string, bool>& map);

  /** \brief Check whether a parameter exists on the parameter server.
   *
   * \param key The key to check.
   *
   * \return true if the parameter exists, false otherwise
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool has(const std::string& key);

  /** \brief Delete a parameter from the parameter server.
   *
   * \param key The key to delete.
   *
   * \return true if the deletion succeeded, false otherwise.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool del(const std::string& key);

  /** \brief Search up the tree for a parameter with a given key
   *
   * This function parameter server's searchParam feature to search up the tree for
   * a parameter.  For example, if the parameter server has a parameter [/a/b]
   * and you specify the namespace [/a/c/d], searching for the parameter "b" will
   * yield [/a/b].  If [/a/c/d/b] existed, that parameter would be returned instead.
   *
   * \param ns The namespace to begin the search in
   * \param key the parameter to search for
   * \param [out] result the found value (if any)
   *
   * \return true if the parameter was found, false otherwise.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool search(const std::string& ns, const std::string& key, std::string& result);

  /** \brief Search up the tree for a parameter with a given key.  This version defaults to starting in
   * the current node's name
   *
   * This function parameter server's searchParam feature to search up the tree for
   * a parameter.  For example, if the parameter server has a parameter [/a/b]
   * and you specify the namespace [/a/c/d], searching for the parameter "b" will
   * yield [/a/b].  If [/a/c/d/b] existed, that parameter would be returned instead.
   *
   * \param key the parameter to search for
   * \param [out] result the found value (if any)
   *
   * \return true if the parameter was found, false otherwise.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  MINIROS_DECL bool search(const std::string& key, std::string& result);

  /**
   * \brief Get the list of all the parameters in the server
   * \param keys The vector of all the keys
   * \return false if the process fails
   */
  MINIROS_DECL bool getParamNames(std::vector<std::string>& keys);

  /** \brief Assign value from parameter server, with default.
   *
   * This method tries to retrieve the indicated parameter value from the
   * parameter server, storing the result in param_val.  If the value
   * cannot be retrieved from the server, default_val is used instead.
   *
   * \param param_name The key to be searched on the parameter server.
   * \param[out] param_val Storage for the retrieved value.
   * \param default_val Value to use if the server doesn't contain this
   * parameter.
   * \return true if the parameter was retrieved from the server, false otherwise.
   * \throws InvalidNameException if the key is not a valid graph resource name
   */
  template <typename T> bool param(const std::string& param_name, T& param_val, const T& default_val)
  {
    if (has(param_name)) {
      if (get(param_name, param_val)) {
        return true;
      }
    }

    param_val = default_val;
    return false;
  }

  /**
   * \brief Return value from parameter server, or default if unavailable.
   *
   * This method tries to retrieve the indicated parameter value from the
   * parameter server. If the parameter cannot be retrieved, \c default_val
   * is returned instead.
   *
   * \param param_name The key to be searched on the parameter server.
   *
   * \param default_val Value to return if the server doesn't contain this
   * parameter.
   *
   * \return The parameter value retrieved from the parameter server, or
   * \c default_val if unavailable.
   *
   * \throws InvalidNameException If the key is not a valid graph resource name.
   */
  template <typename T> T param(const std::string& param_name, const T& default_val)
  {
    T param_val;
    param(param_name, param_val, default_val);
    return param_val;
  }

protected:
  bool getParamImpl(const std::string& key, RpcValue& v, bool use_cache);
  void invalidateParentParams(const std::string& key);
  void update(const std::string& key, const RpcValue& v);
  void paramUpdateCallback(const RpcValue& params, RpcValue& result);

  template <class T>
  Error setParamImpl(const std::string& key, const std::vector<T>& vec);

  template <class T>
  Error setParamImpl(const std::string& key, const std::map<std::string, T>& map);

  bool getParamImpl(const std::string& key, std::string& s, bool use_cache);
  bool getParamImpl(const std::string& key, double& d, bool use_cache);
  bool getParamImpl(const std::string& key, float& f, bool use_cache);
  bool getParamImpl(const std::string& key, int& i, bool use_cache);
  bool getParamImpl(const std::string& key, bool& b, bool use_cache);


  template <class T>
  bool getParamImpl(const std::string& key, std::vector<T>& vec, bool cached);

  template <class T>
  bool getParamImpl(const std::string& key, std::map<std::string, T>& map, bool cached);

  struct Internal;
  Internal* internal_;
};
} // namespace miniros

#endif
