#ifndef _XMLRPCSERVERCONNECTION_H_
#define _XMLRPCSERVERCONNECTION_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include <string>

#include "../network/net_address.h"
#include "miniros/http/http_tools.h"

#include "xmlrpcpp/XmlRpcDecl.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcValue.h"

namespace XmlRpc {

// The server waits for client connections and provides methods
class XmlRpcServer;
class XmlRpcServerMethod;

//! A class to handle XML RPC requests from a particular client
class XMLRPCPP_DECL XmlRpcServerConnection : public XmlRpcSource {
public:
  // Static data
  static const char SYSTEM_MULTICALL[];
  static const char METHODNAME[];
  static const char PARAMS[];

  static const char FAULTCODE[];
  static const char FAULTSTRING[];

  //! Constructor
  XmlRpcServerConnection(int fd, XmlRpcServer* server, bool deleteOnClose = false);

  //! Destructor
  virtual ~XmlRpcServerConnection() override;

  // XmlRpcSource interface implementation
  //! Handle IO on the client connection socket.
  //!   @param eventType Type of IO event that occurred. @see XmlRpcDispatch::EventType.
  unsigned handleEvent(unsigned eventType) override;

  /// Get address of client's endpoint.
  const miniros::network::NetAddress& getClientAddress() const;

  /// Get address of this server.
  const miniros::network::NetAddress& getHostAddress() const;

protected:

  bool readHeader();
  bool readRequest();
  bool writeResponse();

  // Parses the request, runs the method, generates the response xml.
  virtual void executeRequest();

  // Execute a named method with the specified params.
  bool executeMethod(const std::string& methodName, XmlRpcValue& params, XmlRpcValue& result);

  // Execute multiple calls and return the results in an array.
  bool executeMulticall(const std::string& methodName, XmlRpcValue& params, XmlRpcValue& result);

  // Construct a response from the result XML.
  void generateResponse(std::string const& resultXml);
  void generateFaultResponse(std::string const& msg, int errorCode = -1);
  std::string generateHeader(std::string const& body);


  // The XmlRpc server that accepted this connection
  XmlRpcServer* _server;

  // Possible IO states for the connection
  enum ServerConnectionState { READ_HEADER, READ_REQUEST, WRITE_RESPONSE };
  ServerConnectionState _connectionState;

  /// Address of endpoint.
  miniros::network::NetAddress _clientAddress;
  /// Address of a server.
  miniros::network::NetAddress _hostAddress;

  miniros::http::HttpParserFrame _httpFrame;

  // Response
  std::string _response;

  // Number of bytes of the response written so far
  int _bytesWritten;

  // Whether to keep the current client connection open for further requests
  bool _keepAlive;
};

} // namespace XmlRpc

#endif // _XMLRPCSERVERCONNECTION_H_
