#include <cassert>
#include <sstream>
#include <stdio.h>
#include <string.h>

#include "xmlrpcpp/XmlRpcClient.h"
#include "xmlrpcpp/XmlRpcSocket.h"
#include "xmlrpcpp/XmlRpcUtil.h"
#include "xmlrpcpp/XmlRpcValue.h"

#include "miniros/internal/xml_tools.h"

using namespace XmlRpc;

// Static data
const char XmlRpcClient::REQUEST_BEGIN[] = 
  "<?xml version=\"1.0\"?>\r\n"
  "<methodCall><methodName>";
const char XmlRpcClient::REQUEST_END_METHODNAME[] = "</methodName>\r\n";
const char XmlRpcClient::PARAMS_TAG[] = "<params>";
const char XmlRpcClient::PARAMS_ETAG[] = "</params>";
const char XmlRpcClient::PARAM_TAG[] = "<param>";
const char XmlRpcClient::PARAM_ETAG[] =  "</param>";
const char XmlRpcClient::REQUEST_END[] = "</methodCall>\r\n";
const char XmlRpcClient::METHODRESPONSE_TAG[] = "<methodResponse>";
const char XmlRpcClient::FAULT_TAG[] = "<fault>";


const char * XmlRpcClient::connectionStateStr(ClientConnectionState state) {
  switch(state) {
    case NO_CONNECTION:
      return "NO_CONNECTION";
    case CONNECTING:
      return "CONNECTING";
    case WRITE_REQUEST:
      return "WRITE_REQUEST";
    case READ_RESPONSE:
      return "READ_RESPONSE";
    case IDLE:
      return "IDLE";
    default:
      return "UNKNOWN";
  }
}

XmlRpcClient::XmlRpcClient(const char* host, int port, const char* uri/*=0*/)
  : _connectionState(NO_CONNECTION),
  _host(host),
  _port(port),
  _sendAttempts(0),
  _bytesWritten(0),
  _executing(false),
  _eof(false),
  _isFault(false)
{
  XmlRpcUtil::log(1, "XmlRpcClient new client: host %s, port %d.", host, port);

  if (uri)
    _uri = uri;
  else
    _uri = "/RPC2";

  std::stringstream ss;

  updateName();
  // Default to keeping the connection open until an explicit close is done
  setKeepOpen();

  _httpFrame.finishResponse();
}


XmlRpcClient::~XmlRpcClient()
{
  this->close();
}

void XmlRpcClient::updateName()
{
  std::stringstream ss;
  if (_fd >= 0)
    ss << "[" << _fd << "] ";
  if (_uri.empty() || _uri[0] == '/') {
    ss << _host << ":" << _port;
  } else {
    ss << _uri;
  }
  _name = ss.str();
}


// Close the owned fd
void XmlRpcClient::close()
{
  XmlRpcUtil::log(4, "XmlRpcClient(%s)::close: fd %d.", name().c_str(), getfd());
  _connectionState = NO_CONNECTION;
  _disp.exit();
  _disp.removeSource(this);
  XmlRpcSource::close();
}


// Clear the referenced flag even if exceptions or errors occur.
struct ClearFlagOnExit {
  ClearFlagOnExit(bool& flag) : _flag(flag) {}
  ~ClearFlagOnExit() { _flag = false; }
  bool& _flag;
};

// Execute the named procedure on the remote server.
// Params should be an array of the arguments for the method.
// Returns true if the request was sent and a result received (although the result
// might be a fault).
bool
XmlRpcClient::execute(const char* method, XmlRpcValue const& params, XmlRpcValue& result)
{
  XmlRpcUtil::log(1, "XmlRpcClient(%s)::execute: method %s (_connectionState %s).", name().c_str(), method, connectionStateStr(_connectionState));

  result.clear();
  // This is not a thread-safe operation, if you want to do multithreading, use separate
  // clients for each thread. If you want to protect yourself from multiple threads
  // accessing the same client, replace this code with a real mutex.
  if (_executing)
    return false;

  _executing = true;
  ClearFlagOnExit cf(_executing);

  _sendAttempts = 0;
  _isFault = false;

  if ( ! setupConnection())
    return false;

  if ( ! generateRequest(method, params))
    return false;

  result.clear();
  double msTime = -1.0;   // Process until exit is called
  _disp.work(msTime);

  if (_connectionState != IDLE  || _httpFrame.state() != miniros::http::HttpParserFrame::ParseComplete)
    return false;

  {
    std::scoped_lock<std::mutex> lock(_dataGuard);
    const std::string_view body = _httpFrame.body();
    if (!parseResponse(body, result, _isFault))
      return false;

    XmlRpcUtil::log(1, "XmlRpcClient(%s)::execute: method %s completed.", name().c_str(), method);

    _httpFrame.finishResponse();
  }
  return true;
}

bool XmlRpcClient::executeNonBlock(const char* method, XmlRpcValue const& params)
{
  XmlRpcUtil::log(1, "XmlRpcClient(%s)::executeNonBlock: method %s (_connectionState %s).", name().c_str(),
    method, connectionStateStr(_connectionState));

  // This is not a thread-safe operation, if you want to do multithreading, use separate
  // clients for each thread. If you want to protect yourself from multiple threads
  // accessing the same client, replace this code with a real mutex.
  if (_executing)
    return false;

  _executing = true;
  ClearFlagOnExit cf(_executing);

  _sendAttempts = 0;
  _isFault = false;

  if ( ! setupConnection())
    return false;

  if ( ! generateRequest(method, params))
    return false;

  return true;
}

bool XmlRpcClient::executeCheckDone(XmlRpcValue& result)
{
  result.clear();
  // Are we done yet?
  // If we lost connection, the call failed.
  if (_connectionState == NO_CONNECTION) {
    return true;
  }

  // Otherwise, assume the call is still in progress.
  if (_connectionState != IDLE) {
    return false;
  }

  {
    std::scoped_lock<std::mutex> lock(_dataGuard);
    if (!parseResponse(_httpFrame.body(), result, _isFault))
    {
      // Hopefully the caller can determine that parsing failed.
    }
    //XmlRpcUtil::log(1, "XmlRpcClient::execute: method %s completed.", method);
    _httpFrame.finishResponse();
  }
  return true;
}

const std::string& XmlRpcClient::name() const
{
  return _name;
}

// XmlRpcSource interface implementation
// Handle server responses. Called by the event dispatcher during execute.
unsigned
XmlRpcClient::handleEvent(unsigned eventType)
{
  if (eventType == XmlRpcDispatch::Exception)
  {
    if (_connectionState == WRITE_REQUEST && _bytesWritten == 0)
      XmlRpcUtil::error("Error in XmlRpcClient::handleEvent: could not connect to server (%s).", 
                       XmlRpcSocket::getErrorMsg().c_str());
    else
      XmlRpcUtil::error("Error in XmlRpcClient::handleEvent (state %s): %s.", 
                        connectionStateStr(_connectionState),
                        XmlRpcSocket::getErrorMsg().c_str());
    return 0;
  }

  bool finishedSending = false;
  if (_connectionState == WRITE_REQUEST) {
    if ( ! writeRequest()) return 0;
    finishedSending = true;
  }

  if (_connectionState == READ_RESPONSE)
    if ( ! readResponse()) return 0;

  // This should probably always ask for Exception events too
  return (_connectionState == WRITE_REQUEST) 
        ? XmlRpcDispatch::WritableEvent : XmlRpcDispatch::ReadableEvent;
}


// Create the socket connection to the server if necessary
bool XmlRpcClient::setupConnection()
{
  _timeRequestStart = std::chrono::steady_clock::now();

  // If an error occurred last time through, or if the server closed the connection, close our end
  if ((_connectionState != NO_CONNECTION && _connectionState != IDLE) || _eof)
    close();

  _eof = false;
  if (_connectionState == NO_CONNECTION)
    if (! doConnect()) 
      return false;

  // Prepare to write the request
  _connectionState = WRITE_REQUEST;
  _bytesWritten = 0;

  // Notify the dispatcher to listen on this source (calls handleEvent when the socket is writable)
  _disp.removeSource(this);       // Make sure nothing is left over
  _disp.addSource(this, XmlRpcDispatch::WritableEvent | XmlRpcDispatch::Exception);

  return true;
}


// Connect to the xmlrpc server
bool
XmlRpcClient::doConnect()
{
  int fd = XmlRpcSocket::socket();
  if (fd < 0)
  {
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not create socket (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  XmlRpcUtil::log(3, "XmlRpcClient(%s)::doConnect: fd %d.", name().c_str(), fd);
  this->setfd(fd);

  // Don't block on connect/reads/writes
  if ( ! XmlRpcSocket::setNonBlocking(fd))
  {
    this->close();
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not set socket to non-blocking IO mode (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  if ( ! XmlRpcSocket::connect(fd, _host, _port))
  {
    this->close();
    XmlRpcUtil::error("Error in XmlRpcClient::doConnect: Could not connect to server (%s).", XmlRpcSocket::getErrorMsg().c_str());
    return false;
  }

  updateName();
  return true;
}

// Encode the request to call the specified method with the specified parameters into xml
bool XmlRpcClient::generateRequest(const char* methodName, XmlRpcValue const& params)
{
  std::string body = REQUEST_BEGIN;
  body += methodName;
  body += REQUEST_END_METHODNAME;

  // If params is an array, each element is a separate parameter
  if (params.valid()) {
    body += PARAMS_TAG;
    if (params.getType() == XmlRpcValue::TypeArray)
    {
      for (int i=0; i<params.size(); ++i) {
        body += PARAM_TAG;
        body += params[i].toXml();
        body += PARAM_ETAG;
      }
    }
    else
    {
      body += PARAM_TAG;
      body += params.toXml();
      body += PARAM_ETAG;
    }
      
    body += PARAMS_ETAG;
  }
  body += REQUEST_END;

  std::string header = generateHeader(body.length());
  XmlRpcUtil::log(4, "XmlRpcClient(%s)::generateRequest: header is %d bytes, content-length is %d.", name().c_str(),
                  header.length(), body.length());

  _request = header + body;
  return true;
}

// Prepend http headers
std::string XmlRpcClient::generateHeader(size_t length) const
{
  std::string header = 
    "POST " + _uri + " HTTP/1.1\r\n"
    "User-Agent: ";
  header += XMLRPC_VERSION;
  header += "\r\nHost: ";
  header += _host;

  char buff[40];
  std::snprintf(buff,40,":%d\r\n", _port);

  header += buff;
  header += "Content-Type: text/xml\r\nContent-length: ";

  std::snprintf(buff,40,"%zu\r\n\r\n", length);

  return header + buff;
}

bool XmlRpcClient::writeRequest()
{
  if (_bytesWritten == 0)
    XmlRpcUtil::log(5, "XmlRpcClient(%s)::writeRequest (attempt %d):\n%s\n", name().c_str(), _sendAttempts+1, _request.c_str());

  std::scoped_lock lock(_dataGuard);
  // Try to write the request
  if ( ! XmlRpcSocket::nbWrite(this->getfd(), _request, &_bytesWritten)) {
    XmlRpcUtil::error("XmlRpcClient(%s)::writeRequest: write error (%s).", name().c_str(), XmlRpcSocket::getErrorMsg().c_str());
    // If the write fails, we had an unrecoverable error. Close the socket.
    close();
    return false;
  }
    
  XmlRpcUtil::log(3, "XmlRpcClient(%s)::writeRequest: wrote %d of %d bytes.", name().c_str(), _bytesWritten, _request.length());

  // Wait for the result
  if (_bytesWritten == int(_request.length())) {
    _timeRequestSent = std::chrono::steady_clock::now();
    _connectionState = READ_RESPONSE;
  } else {
    // On partial write, remove the portion of the output that was written from
    // the request buffer.
    _request = _request.substr(_bytesWritten);
    _bytesWritten = 0;
  }
  return true;
}

bool XmlRpcClient::readResponse()
{
  // Read available data
  std::scoped_lock lock(_dataGuard);
  size_t oldSize = _httpFrame.data.size();
  bool readOk = XmlRpcSocket::nbRead(this->getfd(), _httpFrame.data, &_eof);
  assert(oldSize <= _httpFrame.data.size());
  size_t dataRead = _httpFrame.data.length() - oldSize;
  if ( !readOk || (_eof && dataRead == 0))
  {
    // If we haven't read any data yet and this is a keep-alive connection, the server may
    // have timed out, so we try one more time.
    if (getKeepOpen() && dataRead == 0 && _sendAttempts++ == 0) {
      XmlRpcUtil::log(4, "XmlRpcClient(%s)::readResponse: re-trying connection", name().c_str());
      XmlRpcSource::close();
      _connectionState = NO_CONNECTION;
      _eof = false;
      return setupConnection();
    }

    XmlRpcUtil::error("XmlRpcClient(%d)::readHeader: error while reading header (%s).", _fd, XmlRpcSocket::getErrorMsg().c_str());
    close();
    return false;
  }

  double timeRead = std::chrono::duration<double>(std::chrono::steady_clock::now() - _timeRequestStart).count() * 1000;
  XmlRpcUtil::log(4, "XmlRpcClient(%s)::readHeader: client has read %d bytes, t=%fms", name().c_str(), dataRead, timeRead);

  if (dataRead == 0) {
    return true;
  }

  _httpFrame.incrementalParse();

  XmlRpcUtil::log(4, "XmlRpcClient(%s)::readHeader client read content length: %d", name().c_str(), _httpFrame.contentLength());

  if (_httpFrame.state() == miniros::http::HttpParserFrame::ParseComplete) {
    if (_httpFrame.contentLength() <= 0) {
      XmlRpcUtil::error("Error in XmlRpcClient(%s)::readHeader: Invalid Content-length specified (%d).", name().c_str(), _httpFrame.contentLength());
      // Close the socket because we can't make further use of it.
      close();
      return false;
    }

    // Otherwise, parse and return the result
    XmlRpcUtil::log(3, "XmlRpcClient(%s)::readResponse (read %d bytes)", name().c_str(), _httpFrame.bodyLength());

    _timeResponseBody = std::chrono::steady_clock::now();
    _connectionState = IDLE;

    double timeSent = std::chrono::duration<double>(_timeRequestSent - _timeRequestStart).count() * 1000;
    double timeHeader = std::chrono::duration<double>(_timeResponseHeader - _timeRequestStart).count() * 1000;
    double timeBody = std::chrono::duration<double>(_timeResponseBody - _timeRequestStart).count() * 1000;

    XmlRpcUtil::log(5, "Response sent=%f recvH=%f recvB=%f", timeSent, timeHeader, timeBody);
    return false;
  }
  // Expect more data to come.
  return true;    // Stop monitoring this source (causes return from work)
}


// Convert the response xml into a result value
bool XmlRpcClient::parseResponse(const std::string_view& responseView, XmlRpcValue& result, bool& isFault) const
{
  std::string response(responseView);

  // Parse response xml into result
  int offset = 0;
  if ( ! XmlRpcUtil::findTag(METHODRESPONSE_TAG, response, &offset)) {
    XmlRpcUtil::error("Error in XmlRpcClient(%s)::parseResponse: Invalid response - no methodResponse. Response:\n%s", name().c_str(), response.c_str());
    return false;
  }

  // Expect either <params><param>... or <fault>...
  if ((XmlRpcUtil::nextTagIs(PARAMS_TAG, response, &offset) && XmlRpcUtil::nextTagIs(PARAM_TAG, response,&offset))
       || (XmlRpcUtil::nextTagIs(FAULT_TAG, response, &offset) && (isFault = true))) //< _isFault assignment is intended behaviour
  {
    miniros::xml::XmlCodec codec;
    size_t offsetCopy = offset;
    if ( !codec.parseXmlRpcValue(result, responseView, offsetCopy))
    {
      XmlRpcUtil::error("Error in XmlRpcClient(%s)::parseResponse: Invalid response value. Response:\n%s", name().c_str(), response.c_str());
      return false;
    }
    offset = offsetCopy;
  } else {
    XmlRpcUtil::error("Error in XmlRpcClient(%s)::parseResponse: Invalid response - no param or fault tag. Response:\n%s", name().c_str(), response.c_str());
    return false;
  }
      
  return result.valid();
}

