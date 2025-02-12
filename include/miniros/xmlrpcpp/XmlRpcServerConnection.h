#ifndef _XMLRPCSERVERCONNECTION_H_
#define _XMLRPCSERVERCONNECTION_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#ifndef MAKEDEPEND
# include <string>
#endif

#include "xmlrpcpp/XmlRpcValue.h"
#include "xmlrpcpp/XmlRpcSource.h"
#include "xmlrpcpp/XmlRpcDecl.h"

namespace XmlRpc {

// The server waits for client connections and provides methods
class XmlRpcServer;
class XmlRpcServerMethod;


/// Intermediate storage for HTTP data.
struct HttpFrame {
  enum ParserState {
    ParseRequest, ParseFieldName, ParseFieldValue, ParseBody, ParseInvalid
  };

  /// Current state of a parser.
  ParserState state = ParseInvalid;

  /// Request headers.
  std::string header;

  /// Number of bytes expected in the request body (parsed from header).
  int contentLength = 0;

  /// Request body.
  std::string request;

  /// Raw request type: {GET, POST, PUT, ...}
  std::string_view requestType;

  /// Request URL
  std::string_view requestUrl;

  /// HTTP Version.
  std::string_view requestHttpVersion;

  /// HTTP Field.
  struct Field {
    std::string_view name;
    std::string_view value;
  };

  /// Parsed fields.
  std::vector<Field> fields;

  /// Reset all fields
  void reset();
};

//! A class to handle XML RPC requests from a particular client
class XMLRPCPP_DECL XmlRpcServerConnection : public XmlRpcSource {
public:
  // Static data
  static const char METHODNAME_TAG[];
  static const char PARAMS_TAG[];
  static const char PARAMS_ETAG[];
  static const char PARAM_TAG[];
  static const char PARAM_ETAG[];

  static const char SYSTEM_MULTICALL[];
  static const char METHODNAME[];
  static const char PARAMS[];

  static const char FAULTCODE[];
  static const char FAULTSTRING[];

  //! Constructor
  XmlRpcServerConnection(int fd, XmlRpcServer* server, bool deleteOnClose = false);
  //! Destructor
  virtual ~XmlRpcServerConnection();

  // XmlRpcSource interface implementation
  //! Handle IO on the client connection socket.
  //!   @param eventType Type of IO event that occurred. @see XmlRpcDispatch::EventType.
  virtual unsigned handleEvent(unsigned eventType);

protected:

  bool readHeader();
  bool readRequest();
  bool writeResponse();

  // Parses the request, runs the method, generates the response xml.
  virtual void executeRequest();

  // Parse the methodName and parameters from the request.
  std::string parseRequest(XmlRpcValue& params);

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

  HttpFrame _httpFrame;

  // Response
  std::string _response;

  // Number of bytes of the response written so far
  int _bytesWritten;

  // Whether to keep the current client connection open for further requests
  bool _keepAlive;
};

} // namespace XmlRpc

#endif // _XMLRPCSERVERCONNECTION_H_
