//
// Created by dkargin on 7/22/25.
//

#include "../../../include/miniros/http/endpoints/xmlrpc.h"

#include "miniros/xmlrpcpp/XmlRpcException.h"
#include "miniros/xmlrpcpp/XmlRpcServer.h"
#include "miniros/xmlrpcpp/XmlRpcServerMethod.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

const char METHODNAME_TAG[] = "<methodName>";
const char PARAMS_TAG[] = "<params>";
const char PARAMS_ETAG[] = "</params>";
const char PARAM_TAG[] = "<param>";
const char PARAM_ETAG[] = "</param>";

const char SYSTEM_MULTICALL[] = "system.multicall";
const char METHODNAME[] = "methodName";
const char PARAMS[] = "params";

const char FAULTCODE[] = "faultCode";
const char FAULTSTRING[] = "faultString";

namespace miniros {
namespace http {

using namespace XmlRpc;

// Create a response from results xml
void generateResponseBody(std::string const& resultXml, std::string& outBuf)
{
  const char RESPONSE_1[] =
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse><params><param>\r\n\t";
  const char RESPONSE_2[] =
    "\r\n</param></params></methodResponse>\r\n";


  outBuf += RESPONSE_1;
  outBuf += resultXml;
  outBuf += RESPONSE_2;
}

void generateFaultResponseBody(std::string const& errorMsg, int errorCode, std::string& outBuf)
{
  const char RESPONSE_1[] =
    "<?xml version=\"1.0\"?>\r\n"
    "<methodResponse><fault>\r\n\t";
  const char RESPONSE_2[] =
    "\r\n</fault></methodResponse>\r\n";

  XmlRpcValue faultStruct;
  faultStruct[FAULTCODE] = errorCode;
  faultStruct[FAULTSTRING] = errorMsg;
  outBuf += RESPONSE_1;
  outBuf += faultStruct.toXml();
  outBuf += RESPONSE_2;
}

// Parse the method name and the argument values from the request.
std::string parseRequest(const HttpParserFrame& httpFrame, XmlRpc::XmlRpcValue& params)
{
  int offset = 0;   // Number of chars parsed from the request

  std::string request{httpFrame.body()};
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

Error XmlRpcHandler::handle(const HttpParserFrame& frame, const ClientInfo& clientInfo,
  HttpResponseHeader& responseHeader, std::string& body)
{
  RpcValue params, resultValue;
  std::string methodName = parseRequest(frame, params);
  MINIROS_DEBUG("XmlRpcHandler(%d)::executeRequest: calling method '%s'", clientInfo.fd, methodName.c_str());

  try {
    if ( ! executeMethod(clientInfo, methodName, params, resultValue) &&
         ! executeMulticall(clientInfo, methodName, params, resultValue)) {
      generateFaultResponseBody(methodName + ": unknown method name", 200, body);
    }
    else {
      generateResponseBody(resultValue.toXml(), body);
    }
  } catch (const XmlRpcException& fault) {
    MINIROS_ERROR("XmlRpcHandler(%d)::executeRequest: fault %s.", clientInfo.fd, fault.getMessage().c_str());
    generateFaultResponseBody(fault.getMessage(), fault.getCode(), body);
  }
  MINIROS_DEBUG("XmlRpcHandler(%d)::executeRequest: finished calling method '%s'", clientInfo.fd, methodName.c_str());
  return Error::Ok;
}

// Execute a named method with the specified params.
bool XmlRpcHandler::executeMethod(const ClientInfo& clientInfo, const std::string& methodName,
  RpcValue& params, RpcValue& result)
{
  XmlRpcServerMethod* method = server_->findMethod(methodName);

  if ( ! method)
    return false;

  method->execute(params, result, clientInfo);

  // Ensure a valid result value
  if ( !result.valid())
      result = std::string();

  return true;
}

// Execute multiple calls and return the results in an array.
bool XmlRpcHandler::executeMulticall(const ClientInfo& clientInfo, const std::string& methodName,
  RpcValue& params, RpcValue& result)
{
  if (methodName != SYSTEM_MULTICALL) return false;

  // There ought to be 1 parameter, an array of structs
  if (params.size() != 1 || params[0].getType() != XmlRpcValue::TypeArray) {
    throw XmlRpcException(std::string(SYSTEM_MULTICALL) + ": Invalid argument (expected an array)");
  }

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
      if ( ! executeMethod(clientInfo, methodName, methodParams, resultValue[0]) &&
           ! executeMulticall(clientInfo, methodName, params, resultValue[0]))
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

}
}