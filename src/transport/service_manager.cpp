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

#include <cstdio>

#include "miniros/transport/service_manager.h"
#include "miniros/transport/xmlrpc_manager.h"
#include "miniros/transport/connection_manager.h"
#include "miniros/transport/poll_manager.h"
#include "miniros/transport/service_publication.h"
#include "miniros/transport/service_client_link.h"
#include "miniros/transport/service_server_link.h"
#include "miniros/this_node.h"
#include "miniros/transport/network.h"
#include "miniros/master.h"
#include "miniros/transport/transport_tcp.h"
#include "miniros/transport/transport_udp.h"
#include "miniros/init.h"
#include "miniros/transport/connection.h"
#include "miniros/transport/file_log.h"

#include "xmlrpcpp/XmlRpc.h"

#include <miniros/console.h>

using namespace XmlRpc; // A battle to be fought later
using namespace std; // sigh

namespace miniros
{

const ServiceManagerPtr& ServiceManager::instance()
{
  static ServiceManagerPtr service_manager = std::make_shared<ServiceManager>();
  return service_manager;
}

ServiceManager::ServiceManager()
: shutting_down_(false)
{
}

ServiceManager::~ServiceManager()
{
  shutdown();
}

void ServiceManager::start(PollManagerPtr pm, ConnectionManagerPtr cm, XMLRPCManagerPtr rpcm)
{
  shutting_down_ = false;

  poll_manager_ = pm;
  connection_manager_ = cm;
  xmlrpc_manager_ = rpcm;
}

void ServiceManager::shutdown()
{
  std::scoped_lock<std::recursive_mutex> shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return;
  }

  shutting_down_ = true;

  ROSCPP_LOG_DEBUG("ServiceManager::shutdown(): unregistering our advertised services");
  {
    std::scoped_lock<std::mutex> ss_lock(service_publications_mutex_);

    for (L_ServicePublication::iterator i = service_publications_.begin();
         i != service_publications_.end(); ++i)
    {
      unregisterService((*i)->getName());
      //ROSCPP_LOG_DEBUG( "shutting down service %s", (*i)->getName().c_str());
      (*i)->drop();
    }
    service_publications_.clear();
  }

  L_ServiceServerLink local_service_clients;
  {
    std::scoped_lock<std::mutex> lock(service_server_links_mutex_);
    local_service_clients.swap(service_server_links_);
  }

  {
    L_ServiceServerLink::iterator it = local_service_clients.begin();
    L_ServiceServerLink::iterator end = local_service_clients.end();
    for (; it != end; ++it)
    {
      (*it)->getConnection()->drop(Connection::Destructing);
    }

    local_service_clients.clear();
  }

}

bool ServiceManager::advertiseService(const AdvertiseServiceOptions& ops)
{
  std::scoped_lock<std::recursive_mutex> shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return false;
  }

  {
    std::scoped_lock<std::mutex> lock(service_publications_mutex_);

    if (isServiceAdvertised(ops.service))
    {
      MINIROS_ERROR("Tried to advertise a service that is already advertised in this node [%s]", ops.service.c_str());
      return false;
    }

    ServicePublicationPtr pub(std::make_shared<ServicePublication>(ops.service, ops.md5sum, ops.datatype, ops.req_datatype, ops.res_datatype, ops.helper, ops.callback_queue, ops.tracked_object));
    service_publications_.push_back(pub);
  }

  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = ops.service;
  char uri_buf[1024];
  std::snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           network::getHost().c_str(), connection_manager_->getTCPPort());
  args[2] = string(uri_buf);
  args[3] = xmlrpc_manager_->getServerURI();
  master::execute("registerService", args, result, payload, true);

  return true;
}

bool ServiceManager::unadvertiseService(const string &serv_name)
{
  std::scoped_lock<std::recursive_mutex> shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return false;
  }

  ServicePublicationPtr pub;
  {
    std::scoped_lock<std::mutex> lock(service_publications_mutex_);

    for (L_ServicePublication::iterator i = service_publications_.begin();
         i != service_publications_.end(); ++i)
    {
      if((*i)->getName() == serv_name && !(*i)->isDropped())
      {
        pub = *i;
        service_publications_.erase(i);
        break;
      }
    }
  }

  if (pub)
  {
    unregisterService(pub->getName());
    ROSCPP_LOG_DEBUG( "shutting down service [%s]", pub->getName().c_str());
    pub->drop();
    return true;
  }

  return false;
}

bool ServiceManager::unregisterService(const std::string& service)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = service;
  char uri_buf[1024];
  std::snprintf(uri_buf, sizeof(uri_buf), "rosrpc://%s:%d",
           network::getHost().c_str(), connection_manager_->getTCPPort());
  args[2] = string(uri_buf);

  return master::execute("unregisterService", args, result, payload, false);
}

bool ServiceManager::isServiceAdvertised(const string& serv_name)
{
  for (L_ServicePublication::iterator s = service_publications_.begin(); s != service_publications_.end(); ++s)
  {
    if (((*s)->getName() == serv_name) && !(*s)->isDropped())
    {
      return true;
    }
  }

  return false;
}

ServicePublicationPtr ServiceManager::lookupServicePublication(const std::string& service)
{
  std::scoped_lock<std::mutex> lock(service_publications_mutex_);

  for (L_ServicePublication::iterator t = service_publications_.begin();
       t != service_publications_.end(); ++t)
  {
    if ((*t)->getName() == service)
    {
      return *t;
    }
  }

  return ServicePublicationPtr();
}

ServiceServerLinkPtr ServiceManager::createServiceServerLink(const std::string& service, bool persistent,
                                             const std::string& request_md5sum, const std::string& response_md5sum,
                                             const M_string& header_values)
{

  std::scoped_lock<std::recursive_mutex> shutdown_lock(shutting_down_mutex_);
  if (shutting_down_)
  {
    return ServiceServerLinkPtr();
  }

  uint32_t serv_port;
  std::string serv_host;
  if (!lookupService(service, serv_host, serv_port))
  {
    return ServiceServerLinkPtr();
  }

  TransportTCPPtr transport(std::make_shared<TransportTCP>(&poll_manager_->getPollSet()));

  // Make sure to initialize the connection *before* transport->connect()
  // is called, otherwise we might miss a connect error (see #434).
  ConnectionPtr connection(std::make_shared<Connection>());
  connection_manager_->addConnection(connection);
  connection->initialize(transport, false, HeaderReceivedFunc());

  if (transport->connect(serv_host, serv_port))
  {
    ServiceServerLinkPtr client(std::make_shared<ServiceServerLink>(service, persistent, request_md5sum, response_md5sum, header_values));

    {
      std::scoped_lock<std::mutex> lock(service_server_links_mutex_);
      service_server_links_.push_back(client);
    }

    client->initialize(connection);

    return client;
  }

  ROSCPP_LOG_DEBUG("Failed to connect to service [%s] (mapped=[%s]) at [%s:%d]", service.c_str(), service.c_str(), serv_host.c_str(), serv_port);

  return ServiceServerLinkPtr();
}

void ServiceManager::removeServiceServerLink(const ServiceServerLinkPtr& client)
{
  // Guard against this getting called as a result of shutdown() dropping all connections (where shutting_down_mutex_ is already locked)
  if (shutting_down_)
  {
    return;
  }

  std::scoped_lock<std::recursive_mutex> shutdown_lock(shutting_down_mutex_);
  // Now check again, since the state may have changed between pre-lock/now
  if (shutting_down_)
  {
    return;
  }

  std::scoped_lock<std::mutex> lock(service_server_links_mutex_);

  L_ServiceServerLink::iterator it = std::find(service_server_links_.begin(), service_server_links_.end(), client);
  if (it != service_server_links_.end())
  {
    service_server_links_.erase(it);
  }
}

bool ServiceManager::lookupService(const string &name, string &serv_host, uint32_t &serv_port)
{
  XmlRpcValue args, result, payload;
  args[0] = this_node::getName();
  args[1] = name;
  if (!master::execute("lookupService", args, result, payload, false))
    return false;

  string serv_uri(payload);
  if (!serv_uri.length()) // shouldn't happen. but let's be sure.
  {
    MINIROS_ERROR("lookupService: Empty server URI returned from master");

    return false;
  }

  if (!network::splitURI(serv_uri, serv_host, serv_port))
  {
    MINIROS_ERROR("lookupService: Bad service uri [%s]", serv_uri.c_str());

    return false;
  }

  return true;
}

} // namespace miniros

