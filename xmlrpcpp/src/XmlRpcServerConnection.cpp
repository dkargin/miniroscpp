#include <cassert>
#include <cstring>
#include <cstdlib>

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
const char XmlRpcServerConnection::METHODNAME_TAG[] = "<methodName>";
const char XmlRpcServerConnection::PARAMS_TAG[] = "<params>";
const char XmlRpcServerConnection::PARAMS_ETAG[] = "</params>";
const char XmlRpcServerConnection::PARAM_TAG[] = "<param>";
const char XmlRpcServerConnection::PARAM_ETAG[] = "</param>";

const char XmlRpcServerConnection::SYSTEM_MULTICALL[] = "system.multicall";
const char XmlRpcServerConnection::METHODNAME[] = "methodName";
const char XmlRpcServerConnection::PARAMS[] = "params";

const char XmlRpcServerConnection::FAULTCODE[] = "faultCode";
const char XmlRpcServerConnection::FAULTSTRING[] = "faultString";


// The server delegates handling client requests to a serverConnection object.
XmlRpcServerConnection::XmlRpcServerConnection(int fd, XmlRpcServer* server, bool deleteOnClose /*= false*/) :
  XmlRpcSource(fd, deleteOnClose)
{
  XmlRpcUtil::log(2,"XmlRpcServerConnection: new socket %d.", fd);
  _server = server;
  _connectionState = READ_HEADER;
  _bytesWritten = 0;
  _keepAlive = true;
}


XmlRpcServerConnection::~XmlRpcServerConnection()
{
  XmlRpcUtil::log(4,"XmlRpcServerConnection dtor.");
  _server->removeConnection(this);
}


// Handle input on the server socket by accepting the connection
// and reading the rpc request. Return true to continue to monitor
// the socket for events, false to remove it from the dispatcher.
unsigned
XmlRpcServerConnection::handleEvent(unsigned /*eventType*/)
{
  if (_connectionState == READ_HEADER)
    if ( ! readHeader()) return 0;

  if (_connectionState == READ_REQUEST)
    if ( ! readRequest()) return 0;

  if (_connectionState == WRITE_RESPONSE)
    if ( ! writeResponse()) return 0;

  return (_connectionState == WRITE_RESPONSE) 
        ? XmlRpcDispatch::WritableEvent : XmlRpcDispatch::ReadableEvent;
}

void HttpFrame::reset()
{
  header = "";
  request = "";
  fields.clear();
  requestType = {};
  requestUrl = {};
  requestHttpVersion = {};
}

bool XmlRpcServerConnection::readHeader()
{
  // Read available data
  bool eof;
  if ( ! XmlRpcSocket::nbRead(this->getfd(), _httpFrame.header, &eof)) {
    // Its only an error if we already have read some data
    if (_httpFrame.header.length() > 0)
      XmlRpcUtil::error("XmlRpcServerConnection::readHeader: error while reading header (%s).",XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  XmlRpcUtil::log(4, "XmlRpcServerConnection::readHeader: read %d bytes.", _httpFrame.header.length());
  const char *hp = _httpFrame.header.c_str();  // Start of header
  const char *ep = hp + _httpFrame.header.length();   // End of string
  const char *bp = nullptr;                 // Start of body
  const char *lp = nullptr;                 // Start of content-length value
  const char *kp = nullptr;                 // Start of connection value

  HttpFrame::ParserState state = HttpFrame::ParseRequest;
  std::string_view fieldName;
  std::string_view fieldValue;

  const char* tokenStart = hp;

  for (const char *cp = hp; (bp == 0) && (cp < ep);) {
    if (state == HttpFrame::ParseRequest) {
      if (strncmp(cp, "\r\n", 2) == 0) {
        _httpFrame.requestHttpVersion = std::string_view(tokenStart, cp - tokenStart);
        cp += 2;
        tokenStart = cp;
        state = HttpFrame::ParseFieldName;
        continue;
      } if (_httpFrame.requestType.empty() && *cp == ' ') {
        _httpFrame.requestType = std::string_view(tokenStart, cp - tokenStart);
        tokenStart = cp + 1;
      } else if (_httpFrame.requestUrl.empty() && *cp == ' ') {
        _httpFrame.requestUrl = std::string_view(tokenStart, cp - tokenStart);
        tokenStart = cp + 1;
      } else {
        // Error?
      }
    }
    else if (state == HttpFrame::ParseFieldName) {
      assert(tokenStart != nullptr);
      if (tokenStart == cp) {
        if (strncasecmp(cp, "Content-length: ", 16) == 0) {
          lp = cp + 16;
        } else if (strncasecmp(cp, "Connection: ", 12) == 0) {
          kp = cp + 12;
        }

        if ((ep - cp > 4) && (strncmp(cp, "\r\n\r\n", 4) == 0))
          bp = cp + 4;
        else if ((ep - cp > 2) && (strncmp(cp, "\n\n", 2) == 0))
          bp = cp + 2;
      }

      if (*cp == ':') {
        state = HttpFrame::ParseFieldValue;
        fieldName = std::string_view(tokenStart, cp - tokenStart);
        cp++;
        tokenStart = cp;
        continue;
      }

      if (strncmp(cp, "\n\n", 2) == 0 || strncmp(cp, "\r\n", 2) == 0) {
        state = HttpFrame::ParseBody;
        cp += 2;
        tokenStart = cp;
        bp = cp;
        continue;
      }
    } else if (state == HttpFrame::ParseFieldValue) {
      assert(tokenStart != nullptr);
      if (strncmp(cp, "\r\n", 2) == 0) {
        fieldValue = std::string_view(tokenStart, cp - tokenStart);
        _httpFrame.fields.push_back({fieldName, fieldValue});
        cp += 2;
        tokenStart = cp;
        state = HttpFrame::ParseFieldName;
        continue;
      }
    } else
      break;
    cp++;
  }

  // If we haven't gotten the entire header yet, return (keep reading)
  if (!bp) {
    // EOF in the middle of a request is an error, otherwise it is ok
    if (eof) {
      XmlRpcUtil::log(4, "XmlRpcServerConnection::readHeader: EOF");
      if (_httpFrame.header.length() > 0)
        XmlRpcUtil::error("XmlRpcServerConnection::readHeader: EOF while reading header");
      return false;   // Either way we close the connection
    }
    
    return true;  // Keep reading
  }

  // Decode content length
  if (!lp) {
    XmlRpcUtil::error("XmlRpcServerConnection::readHeader: No Content-length specified");
    return false;   // We could try to figure it out by parsing as we read, but for now...
  }

  _httpFrame.contentLength = atoi(lp);
  if (_httpFrame.contentLength <= 0) {
    XmlRpcUtil::error("XmlRpcServerConnection::readHeader: Invalid Content-length specified (%d).", _httpFrame.contentLength);
    return false;
  }

  XmlRpcUtil::log(3, "XmlRpcServerConnection::readHeader: specified content length is %d.", _httpFrame.contentLength);

  // Otherwise copy non-header data to request buffer and set state to read request.
  _httpFrame.request = bp;

  // Parse out any interesting bits from the header (HTTP version, connection)
  _keepAlive = true;
  if (_httpFrame.header.find("HTTP/1.0") != std::string::npos) {
    if (kp == 0 || strncasecmp(kp, "keep-alive", 10) != 0)
      _keepAlive = false;           // Default for HTTP 1.0 is to close the connection
  } else {
    if (kp != 0 && strncasecmp(kp, "close", 5) == 0)
      _keepAlive = false;
  }
  XmlRpcUtil::log(3, "KeepAlive: %d", _keepAlive);

  _httpFrame.header = "";
  _connectionState = READ_REQUEST;
  return true;    // Continue monitoring this source
}

bool
XmlRpcServerConnection::readRequest()
{
  // If we dont have the entire request yet, read available data
  const int requestLength = static_cast<int>(_httpFrame.request.length());
  if (requestLength < _httpFrame.contentLength) {
    bool eof;
    if ( ! XmlRpcSocket::nbRead(this->getfd(), _httpFrame.request, &eof)) {
      XmlRpcUtil::error("XmlRpcServerConnection::readRequest: read error (%s).",XmlRpcSocket::getErrorMsg().c_str());
      return false;
    }

    // If we haven't gotten the entire request yet, return (keep reading)
    if (requestLength < _httpFrame.contentLength) {
      if (eof) {
        XmlRpcUtil::error("XmlRpcServerConnection::readRequest: EOF while reading request");
        return false;   // Either way we close the connection
      }
      return true;
    }
  }

  // Otherwise, parse and dispatch the request
  XmlRpcUtil::log(3, "XmlRpcServerConnection::readRequest read %d bytes.", _httpFrame.request.length());

  _connectionState = WRITE_RESPONSE;

  return true;    // Continue monitoring this source
}


bool
XmlRpcServerConnection::writeResponse()
{
  if (_response.length() == 0) {
    executeRequest();
    _bytesWritten = 0;
    if (_response.length() == 0) {
      XmlRpcUtil::error("XmlRpcServerConnection::writeResponse: empty response.");
      return false;
    }
  }

  // Try to write the response
  if ( ! XmlRpcSocket::nbWrite(this->getfd(), _response, &_bytesWritten)) {
    XmlRpcUtil::error("XmlRpcServerConnection::writeResponse: write error (%s).",XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }
  XmlRpcUtil::log(3, "XmlRpcServerConnection::writeResponse: wrote %d of %d bytes.", _bytesWritten, _response.length());

  // Prepare to read the next request
  if (_bytesWritten == static_cast<int>(_response.length())) {
    _httpFrame.reset();
    _response = "";
    _connectionState = READ_HEADER;
  }

  return _keepAlive;    // Continue monitoring this source if true
}

// Run the method, generate _response string
void
XmlRpcServerConnection::executeRequest()
{
  XmlRpcValue params, resultValue;
  std::string methodName = parseRequest(params);
  XmlRpcUtil::log(2, "XmlRpcServerConnection::executeRequest: server calling method '%s'", 
                    methodName.c_str());

  try {

    if ( ! executeMethod(methodName, params, resultValue) &&
         ! executeMulticall(methodName, params, resultValue))
      generateFaultResponse(methodName + ": unknown method name");
    else
      generateResponse(resultValue.toXml());

  } catch (const XmlRpcException& fault) {
    XmlRpcUtil::log(2, "XmlRpcServerConnection::executeRequest: fault %s.",
                    fault.getMessage().c_str()); 
    generateFaultResponse(fault.getMessage(), fault.getCode());
  }
}

// Parse the method name and the argument values from the request.
std::string
XmlRpcServerConnection::parseRequest(XmlRpcValue& params)
{
  int offset = 0;   // Number of chars parsed from the request

  const auto& request = _httpFrame.request;
  std::string methodName = XmlRpcUtil::parseTag(METHODNAME_TAG, request, &offset);

  if (methodName.size() > 0 && XmlRpcUtil::findTag(PARAMS_TAG, request, &offset))
  {
    int nArgs = 0;
    while (XmlRpcUtil::nextTagIs(PARAM_TAG, request, &offset)) {
      params[nArgs++] = XmlRpcValue(request, &offset);
      (void) XmlRpcUtil::nextTagIs(PARAM_ETAG, request, &offset);
    }

    (void) XmlRpcUtil::nextTagIs(PARAMS_ETAG, request, &offset);
  }

  return methodName;
}

// Execute a named method with the specified params.
bool
XmlRpcServerConnection::executeMethod(const std::string& methodName, 
                                      XmlRpcValue& params, XmlRpcValue& result)
{
  XmlRpcServerMethod* method = _server->findMethod(methodName);

  if ( ! method) return false;

  method->execute(params, result, this);

  // Ensure a valid result value
  if ( ! result.valid())
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
  XmlRpcUtil::log(5, "XmlRpcServerConnection::generateResponse:\n%s\n", _response.c_str()); 
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

