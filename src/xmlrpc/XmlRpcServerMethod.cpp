
#include "miniros/xmlrpcpp/XmlRpcServerMethod.h"
#include "miniros/xmlrpcpp/XmlRpcServer.h"

namespace miniros {
namespace XmlRpc {

  XmlRpcServerMethod::XmlRpcServerMethod(std::string const& name, XmlRpcMethods* server)
  {
    _name = name;
    _server = server;
    if (_server) _server->addMethod(this);
  }

  XmlRpcServerMethod::~XmlRpcServerMethod()
  {
    if (_server) _server->removeMethod(this);
  }


} // namespace XmlRpc
} // namespace miniros
