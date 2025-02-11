
#ifndef _XMLRPCSERVERMETHOD_H_
#define _XMLRPCSERVERMETHOD_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include "XmlRpcDecl.h"

#ifndef MAKEDEPEND
# include <string>
#endif

namespace XmlRpc {

  // Representation of a parameter or result value
  class XmlRpcValue;

  // The XmlRpcServer processes client requests to call RPCs
  class XmlRpcServer;

  class XmlRpcServerConnection;

  //! Abstract class representing a single RPC method
  class XMLRPCPP_DECL XmlRpcServerMethod {
  public:
    //! Constructor
    XmlRpcServerMethod(std::string const& name, XmlRpcServer* server = 0);

    //! Destructor
    virtual ~XmlRpcServerMethod();

    //! Returns the name of the method
    const std::string& name() { return _name; }

    //! Execute the method. Subclasses must provide a definition for this method.
    virtual void execute(const XmlRpcValue& params, XmlRpcValue& result, XmlRpcServerConnection* connection) = 0;

    //! Returns a help string for the method.
    //! Subclasses should define this method if introspection is being used.
    virtual std::string help() const { return {}; }

  protected:
    std::string _name;
    XmlRpcServer* _server;
  };
} // namespace XmlRpc

#endif // _XMLRPCSERVERMETHOD_H_
