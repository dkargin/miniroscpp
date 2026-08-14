// this file modified by Morgan Quigley on 22 April 2008 to add 
// a std::exception-derived class
#ifndef MINIROS_XMLRPC_H_
#define MINIROS_XMLRPC_H_

// Bundled xmlrpc++ is namespaced as miniros::XmlRpc. Never include
// <xmlrpcpp/...> or "xmlrpcpp/..." — those resolve to ROS/system xmlrpc++
// (::XmlRpc) when it is installed, and produce runtime errors such as
// undefined symbol XmlRpc::setVerbosity.
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307
// 

#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <string>
#endif

#include "miniros/xmlrpcpp/XmlRpcClient.h"
#include "miniros/xmlrpcpp/XmlRpcException.h"
#include "miniros/xmlrpcpp/XmlRpcServer.h"
#include "miniros/xmlrpcpp/XmlRpcServerMethod.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"
#include <stdexcept>

#endif // MINIROS_XMLRPC_H_
