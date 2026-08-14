/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
*********************************************************************/
/*
 * Cross platform macros.
 *
 */
#ifndef MINIROS_XMLRPCPP_DECL_H
#define MINIROS_XMLRPCPP_DECL_H

#include <miniros/macros.h>

// Stock/ROS xmlrpc++ uses the same header names under <xmlrpcpp/> and
// declares ::XmlRpc. Mixing that with MiniROS produces runtime errors
// such as undefined symbol XmlRpc::setVerbosity.
#if defined(_XMLRPC_H_) || defined(_XMLRPCUTIL_H_) || defined(_XMLRPCCLIENT_H_) || \
    defined(_XMLRPCVALUE_H_) || defined(_XMLRPCSERVER_H_) || defined(XMLRPCPP_DECL_H_INCLUDED)
#error "System/ROS xmlrpc++ was included before MiniROS. Use \"miniros/xmlrpcpp/...\" only; do not include <xmlrpcpp/...>."
#endif
#define XMLRPCPP_DECL_H_INCLUDED
#define _XMLRPC_H_
#define _XMLRPCUTIL_H_
#define _XMLRPCCLIENT_H_
#define _XMLRPCVALUE_H_
#define _XMLRPCSERVER_H_
#define _XMLRPCDISPATCH_H_
#define _XMLRPCEXCEPTION_H_
#define _XMLRPCSERVERCONNECTION_H_
#define _XMLRPCSERVERMETHOD_H_
#define _XMLRPCSOCKET_H_
#define _XMLRPCSOURCE_H_

#define XMLRPCPP_DECL MINIROS_HELPER_EXPORT

/*
#ifdef MINIROS_BUILD_SHARED_LIBS // ros is being built around shared libraries
  #ifdef xmlrpcpp_EXPORTS // we are building a shared lib/dll
    #define XMLRPCPP_DECL MINIROS_HELPER_EXPORT
  #else // we are using shared lib/dll
    #define XMLRPCPP_DECL MINIROS_HELPER_IMPORT
  #endif
#else // ros is being built around static libraries
  #define XMLRPCPP_DECL
#endif
*/

#endif /* MINIROS_XMLRPCPP_DECL_H */
