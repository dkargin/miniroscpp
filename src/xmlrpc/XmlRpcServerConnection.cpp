#include <cassert>
#include <cstring>
#include <cstdlib>

#include "miniros/network/net_address.h"
#include "miniros/internal/xml_tools.h"
#include "xmlrpcpp/XmlRpcServerConnection.h"

#include "xmlrpcpp/XmlRpc.h"
#include "xmlrpcpp/XmlRpcSocket.h"

#ifndef MAKEDEPEND
# include <stdio.h>
#ifndef _WINDOWS
# include <strings.h>
#endif
#endif

using namespace XmlRpc;

// Static data
const char XmlRpcServerConnection::SYSTEM_MULTICALL[] = "system.multicall";
const char XmlRpcServerConnection::METHODNAME[] = "methodName";
const char XmlRpcServerConnection::PARAMS[] = "params";
const char XmlRpcServerConnection::FAULTCODE[] = "faultCode";
const char XmlRpcServerConnection::FAULTSTRING[] = "faultString";

// The server delegates handling client requests to a serverConnection object.
XmlRpcServerConnection::XmlRpcServerConnection(int fd, XmlRpcServer* server, bool deleteOnClose /*= false*/) :
  XmlRpcSource(fd, deleteOnClose)
{
  _server = server;
  _connectionState = READ_HEADER;
  _bytesWritten = 0;
  _keepAlive = true;
  _httpFrame.finishRequest();

  if (fd) {
    miniros::network::readRemoteAddress(fd, _clientAddress);
    miniros::network::readLocalAddress(fd, _hostAddress);
  }

  if (_clientAddress.valid())
    XmlRpcUtil::log(2,"XmlRpcServerConnection: new socket %d from %s:%d", fd, _clientAddress.address.c_str(), _clientAddress.port());
  else
    XmlRpcUtil::log(2,"XmlRpcServerConnection: new socket %d from unknown endpoint", fd);
}

XmlRpcServerConnection::~XmlRpcServerConnection()
{
  XmlRpcUtil::log(2,"XmlRpcServerConnection(%d) dtor.", _fd);
  _server->removeConnection(this);
}

// Handle input on the server socket by accepting the connection
// and reading the rpc request. Return true to continue to monitor
// the socket for events, false to remove it from the dispatcher.
unsigned XmlRpcServerConnection::handleEvent(unsigned /*eventType*/)
{
  if (_connectionState == READ_HEADER) {
    if ( !readHeader())
      return 0;
  }

  if (_connectionState == READ_REQUEST) {
    if ( ! readRequest())
      return 0;
  }

  if (_connectionState == WRITE_RESPONSE) {
    if ( ! writeResponse())
      return 0;
  }

  return (_connectionState == WRITE_RESPONSE) ? XmlRpcDispatch::WritableEvent : XmlRpcDispatch::ReadableEvent;
}

bool XmlRpcServerConnection::readHeader()
{
  // Read available data
  bool eof;
  size_t oldSize = _httpFrame.data.size();
  if ( !XmlRpcSocket::nbRead(this->getfd(), _httpFrame.data, &eof)) {
    // Its only an error if we already have read some data
    if (_httpFrame.data.length() > 0)
      XmlRpcUtil::error("XmlRpcServerConnection(%d)::readHeader: error while reading header (%s).", _fd, XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  XmlRpcUtil::log(4, "XmlRpcServerConnection(%d)::readHeader: read %d->%d bytes.", _fd, oldSize, _httpFrame.data.length());

  int parsed = _httpFrame.incrementalParse();

  // If we haven't gotten the entire header yet, return (keep reading)
  if (!_httpFrame.hasHeader()  || !parsed) {
    // EOF in the middle of a request is an error, otherwise it is ok
    if (eof) {
      if (_httpFrame.data.length() > 0)
        XmlRpcUtil::error("XmlRpcServerConnection(%d)::readHeader: EOF while reading header", _fd);
      else
        XmlRpcUtil::log(4, "XmlRpcServerConnection(%d)::readHeader: EOF", _fd);
      return false;   // Either way we close the connection
    }
    return true;  // Keep reading
  }

  if (_httpFrame.contentLength() <= 0) {
    XmlRpcUtil::error("XmlRpcServerConnection(%d)::readHeader: Invalid Content-length specified (%d).", _fd, _httpFrame.contentLength());
    return false;
  }

  // Parse out any interesting bits from the header (HTTP version, connection)
  _keepAlive = _httpFrame.keepAlive();

  XmlRpcUtil::log(3, "XmlRpcServerConnection(%d)::readHeader: ContentLength=%d, KeepAlive=%d, parsed=%d", _fd, _httpFrame.contentLength(), _keepAlive, parsed);

  _connectionState = READ_REQUEST;
  return true;    // Continue monitoring this source
}

bool XmlRpcServerConnection::readRequest()
{
  // If we don't have the entire request yet, read available data
  if (_httpFrame.state() == miniros::http::HttpParserFrame::ParseBody) {
    bool eof;
    if ( ! XmlRpcSocket::nbRead(this->getfd(), _httpFrame.data, &eof)) {
      XmlRpcUtil::error("XmlRpcServerConnection(%d)::readRequest: read error (%s).", _fd, XmlRpcSocket::getErrorMsg().c_str());
      _httpFrame.finishRequest();
      return false;
    }
    _httpFrame.incrementalParse();

    // If we haven't gotten the entire request yet, return (keep reading)
    if (_httpFrame.state() == miniros::http::HttpParserFrame::ParseBody) {
      if (eof) {
        XmlRpcUtil::error("XmlRpcServerConnection(%d)::readRequest: EOF while reading request", _fd);
        _httpFrame.finishRequest();
        return false;   // Either way we close the connection
      }
      XmlRpcUtil::log(3, "XmlRpcServerConnection(%d)::readRequest got only %d/%d bytes.", _fd, _httpFrame.bodyLength(), _httpFrame.contentLength());
      return true;
    }
  }

  assert(_httpFrame.state() == miniros::http::HttpParserFrame::ParseComplete);
  auto body = _httpFrame.body();
  // Otherwise, parse and dispatch the request
  XmlRpcUtil::log(3, "XmlRpcServerConnection(%d)::readRequest read %d/%d bytes.", _fd, _httpFrame.bodyLength(), _httpFrame.contentLength());

  _connectionState = WRITE_RESPONSE;

  return true;    // Continue monitoring this source
}


bool XmlRpcServerConnection::writeResponse()
{
  if (_response.length() == 0) {
    executeRequest();
    _bytesWritten = 0;

    if (_response.length() == 0) {
      XmlRpcUtil::error("XmlRpcServerConnection(%d)::writeResponse: empty response.", _fd);
      return false;
    }
  }

  // Try to write the response
  if ( ! XmlRpcSocket::nbWrite(this->getfd(), _response, &_bytesWritten)) {
    XmlRpcUtil::error("XmlRpcServerConnection(%d)::writeResponse: write error (%s).", _fd, XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }
  XmlRpcUtil::log(3, "XmlRpcServerConnection(%d)::writeResponse: wrote %d of %d bytes.", _fd, _bytesWritten, _response.length());

  // Prepare to read the next request
  if (_bytesWritten == static_cast<int>(_response.length())) {
    _httpFrame.finishRequest();
    _response = "";
    _connectionState = READ_HEADER;
  }

  return _keepAlive;    // Continue monitoring this source if true
}

// Run the method, generate _response string
void XmlRpcServerConnection::executeRequest()
{
  XmlRpcValue params, resultValue;
  miniros::xml::XmlCodec codec;

  std::string_view methodView;
  codec.parseXmlRpcRequest(_httpFrame.body(), methodView, params);
  std::string methodName(methodView);
  XmlRpcUtil::log(2, "XmlRpcServerConnection(%d)::executeRequest: calling method '%s'", _fd,
                    methodName.c_str());

  try {

    if ( ! executeMethod(methodName, params, resultValue) &&
         ! executeMulticall(methodName, params, resultValue))
      generateFaultResponse(methodName + ": unknown method name");
    else
      generateResponse(resultValue.toXml());
  } catch (const XmlRpcException& fault) {
    XmlRpcUtil::log(2, "XmlRpcServerConnection(%d)::executeRequest: fault %s.", _fd, fault.getMessage().c_str());
    generateFaultResponse(fault.getMessage(), fault.getCode());
  }
  XmlRpcUtil::log(2, "XmlRpcServerConnection(%d)::executeRequest: finished calling method '%s'", _fd, methodName.c_str());
}

// Execute a named method with the specified params.
bool XmlRpcServerConnection::executeMethod(const std::string& methodName,
                                      XmlRpcValue& params, XmlRpcValue& result)
{
  XmlRpcServerMethod* method = _server->findMethod(methodName);

  if ( ! method) return false;

  miniros::network::ClientInfo clientInfo;
  clientInfo.fd = getfd();
  method->execute(params, result, clientInfo);

  // Ensure a valid result value
  if ( !result.valid())
      result = std::string();

  return true;
}

// Execute multiple calls and return the results in an array.
bool
XmlRpcServerConnection::executeMulticall(const std::string& methodName, 
                                         XmlRpcValue& params, XmlRpcValue& result)
{
  if (methodName != SYSTEM_MULTICALL) return false;

  // There ought to be 1 parameter, an array of structs
  if (params.size() != 1 || params[0].getType() != XmlRpcValue::TypeArray)
    throw XmlRpcException(std::string(SYSTEM_MULTICALL) + ": Invalid argument (expected an array)");

  int nc = params[0].size();
  result.setSize(nc);

  for (int i=0; i<nc; ++i) {

    if ( ! params[0][i].hasMember(METHODNAME) ||
         ! params[0][i].hasMember(PARAMS)) {
      result[i][FAULTCODE] = -1;
      result[i][FAULTSTRING] = std::string(SYSTEM_MULTICALL) +
              ": Invalid argument (expected a struct with members methodName and params)";
      continue;
    }

    const std::string& methodName = params[0][i][METHODNAME];
    XmlRpcValue& methodParams = params[0][i][PARAMS];

    XmlRpcValue resultValue;
    resultValue.setSize(1);
    try {
      if ( ! executeMethod(methodName, methodParams, resultValue[0]) &&
           ! executeMulticall(methodName, params, resultValue[0]))
      {
        result[i][FAULTCODE] = -1;
        result[i][FAULTSTRING] = methodName + ": unknown method name";
      }
      else
        result[i] = resultValue;

    } catch (const XmlRpcException& fault) {
        result[i][FAULTCODE] = fault.getCode();
        result[i][FAULTSTRING] = fault.getMessage();
    }
  }

  return true;
}


// Create a response from results xml
void
XmlRpcServerConnection::generateResponse(std::string const& resultXml)
{
  const char RESPONSE_1[] = 
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse><params><param>\r\n\t";
  const char RESPONSE_2[] =
    "\r\n</param></params></methodResponse>\r\n";

  std::string body = RESPONSE_1 + resultXml + RESPONSE_2;
  std::string header = generateHeader(body);

  _response = header + body;
  XmlRpcUtil::log(5, "XmlRpcServerConnection(%d)::generateResponse:\n%s\n", _fd, _response.c_str());
}

// Prepend http headers
std::string
XmlRpcServerConnection::generateHeader(std::string const& body)
{
  std::string header = 
    "HTTP/1.1 200 OK\r\n"
    "Server: ";
  header += XMLRPC_VERSION;
  header += "\r\n"
    "Content-Type: text/xml\r\n"
    "Content-length: ";

  char buffLen[40];
#ifdef _MSC_VER
  sprintf_s(buffLen,40,"%d\r\n\r\n", (int)body.size());
#else
  sprintf(buffLen,"%d\r\n\r\n", (int)body.size());
#endif

  return header + buffLen;
}


void
XmlRpcServerConnection::generateFaultResponse(std::string const& errorMsg, int errorCode)
{
  const char RESPONSE_1[] = 
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse><fault>\r\n\t";
  const char RESPONSE_2[] =
    "\r\n</fault></methodResponse>\r\n";

  XmlRpcValue faultStruct;
  faultStruct[FAULTCODE] = errorCode;
  faultStruct[FAULTSTRING] = errorMsg;
  std::string body = RESPONSE_1 + faultStruct.toXml() + RESPONSE_2;
  std::string header = generateHeader(body);

  _response = header + body;
}

const miniros::network::NetAddress& XmlRpcServerConnection::getClientAddress() const
{
  return _clientAddress;
}

const miniros::network::NetAddress& XmlRpcServerConnection::getHostAddress() const
{
  return _hostAddress;
}
