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

#ifndef MINIROS_ADVERTISE_SERVICE_OPTIONS_H
#define MINIROS_ADVERTISE_SERVICE_OPTIONS_H

#include "miniros/transport/service_callback_helper.h"
#include "miniros/traits/service_traits.h"
#include "miniros/traits/message_traits.h"
#include "miniros/common.h"

namespace miniros
{

/**
 * \brief Encapsulates all options available for creating a ServiceServer
 */
struct MINIROS_DECL AdvertiseServiceOptions
{
  AdvertiseServiceOptions()
  : callback_queue(0)
  {
  }

  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service request/response types
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template<class MReq, class MRes>
  void init(const std::string& _service, const std::function<bool(MReq&, MRes&)>& _callback)
  {
    namespace st = service_traits;
    namespace mt = message_traits;
    if (st::md5sum<MReq>() != st::md5sum<MRes>())
    {
      reportMismatchServiceMd5(_service.c_str());
    }

    service = _service;
    md5sum = st::md5sum<MReq>();
    datatype = st::datatype<MReq>();
    req_datatype = mt::datatype<MReq>();
    res_datatype = mt::datatype<MRes>();
    helper = std::make_shared<ServiceCallbackHelperT<ServiceSpec<MReq, MRes> > >(_callback);
  }

  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service type
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template<class Service>
  void init(const std::string& _service, const std::function<bool(typename Service::Request&, typename Service::Response&)>& _callback)
  {
    namespace st = service_traits;
    namespace mt = message_traits;
    typedef typename Service::Request Request;
    typedef typename Service::Response Response;
    service = _service;
    md5sum = st::md5sum<Service>();
    datatype = st::datatype<Service>();
    req_datatype = mt::datatype<Request>();
    res_datatype = mt::datatype<Response>();
    helper = std::make_shared<ServiceCallbackHelperT<ServiceSpec<Request, Response> > >(_callback);
  }

  /**
   * \brief Templated convenience method for filling out md5sum/etc. based on the service spec type
   * \param _service Service name to advertise on
   * \param _callback Callback to call when this service is called
   */
  template<class Spec>
  void initBySpecType(const std::string& _service, const typename Spec::CallbackType& _callback)
  {
    namespace st = service_traits;
    namespace mt = message_traits;
    typedef typename Spec::RequestType Request;
    typedef typename Spec::ResponseType Response;
    service = _service;
    md5sum = st::md5sum<Request>();
    datatype = st::datatype<Request>();
    req_datatype = mt::datatype<Request>();
    res_datatype = mt::datatype<Response>();
    helper = std::make_shared<ServiceCallbackHelperT<Spec> >(_callback);
  }

  std::string service;                                                ///< Service name
  std::string md5sum;                                                 ///< MD5 of the service
  std::string datatype;                                               ///< Datatype of the service
  std::string req_datatype;                                           ///< Request message datatype
  std::string res_datatype;                                           ///< Response message datatype

  ServiceCallbackHelperPtr helper;                                     ///< Helper object used for creating messages and calling callbacks

  CallbackQueueInterface* callback_queue;                             ///< Queue to add callbacks to.  If NULL, the global callback queue will be used

  /**
   * \brief An object whose destruction will prevent the callback associated with this service from being called
   *
   * A shared pointer to an object to track for these callbacks.  If set, the a weak_ptr will be created to this object,
   * and if the reference count goes to 0 the subscriber callbacks will not get called.
   *
   * \note Note that setting this will cause a new reference to be added to the object before the
   * callback, and for it to go out of scope (and potentially be deleted) in the code path (and therefore
   * thread) that the callback is invoked from.
   */
  VoidConstPtr tracked_object;

  /**
   * \brief Templated helper function for creating an AdvertiseServiceOptions with all of its options
   * \param service Service name to advertise on
   * \param callback The callback to invoke when the service is called
   * \param tracked_object The tracked object to use (see AdvertiseServiceOptions::tracked_object)
   * \param queue The callback queue to use (see AdvertiseServiceOptions::callback_queue)
   */
  template<class Service>
  static AdvertiseServiceOptions create(const std::string& service,
                                 const std::function<bool(typename Service::Request&, typename Service::Response&)>& callback,
                                 const VoidConstPtr& tracked_object,
                                 CallbackQueueInterface* queue)
  {
    AdvertiseServiceOptions ops;
    ops.init<typename Service::Request, typename Service::Response>(service, callback);
    ops.tracked_object = tracked_object;
    ops.callback_queue = queue;
    return ops;
  }

protected:
  static void reportMismatchServiceMd5(const char* service);
};

} // namespace miniros

#endif // MINIROS_ADVERTISE_SERVICE_OPTIONS_H
