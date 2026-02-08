//
// Created by dkargin on 7/22/25.
//

#include "miniros/console.h"
#include "miniros/http/endpoints/xmlrpc.h"

#include "http/http_request.h"
#include "miniros/xmlrpcpp/XmlRpcException.h"
#include "miniros/xmlrpcpp/XmlRpcServer.h"
#include "miniros/xmlrpcpp/XmlRpcServerMethod.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

#include "miniros/internal/xml_tools.h"

#define MINIROS_PACKAGE_NAME "http"

namespace {
constexpr char SYSTEM_MULTICALL[] = "system.multicall";
constexpr char METHODNAME[] = "methodName";
constexpr char PARAMS[] = "params";
constexpr char FAULTCODE[] = "faultCode";
constexpr char FAULTSTRING[] = "faultString";
}

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

Error XmlRpcHandler::handle(const network::ClientInfo& clientInfo, std::shared_ptr<HttpRequest> request)
{
  RpcValue params, resultValue;

  std::string_view methodView;
  std::string body;
  if (!xml::XmlCodec::parseXmlRpcRequest(request->requestBody(), methodView, params) || methodView.empty()) {
    const char* msg = "Failed to parse XMLRPC request";
    MINIROS_ERROR("XmlRpcHandler(%d)::executeRequest: fault %s.", clientInfo.fd, msg);
    generateFaultResponseBody(msg, 0, body);
    request->setResponseStatus(200, "OK");
    request->setResponseBody(body, "text/xml");
    return Error::Ok;
  }

  std::string methodName(methodView);

  MINIROS_DEBUG("XmlRpcHandler(%d)::executeRequest method '%s'(%s)", clientInfo.fd, methodName.c_str(), params.toJsonStr().c_str());

  try {
    if ( ! executeMethod(clientInfo, methodName, params, resultValue) &&
         ! executeMulticall(clientInfo, methodName, params, resultValue)) {
      generateFaultResponseBody(methodName + ": unknown method name", 200, body);
      request->setResponseStatus(200, "OK");
      request->setResponseBody(body, "text/xml");
    }
    else {
      generateResponseBody(resultValue.toXml(), body);
      request->setResponseStatus(200, "OK");
      request->setResponseBody(body, "text/xml");
    }
  } catch (const XmlRpcException& fault) {
    MINIROS_ERROR("XmlRpcHandler(%d)::executeRequest: fault %s.", clientInfo.fd, fault.getMessage().c_str());
    generateFaultResponseBody(fault.getMessage(), fault.getCode(), body);
    request->setResponseStatus(200, "OK");
    request->setResponseBody(body, "text/xml");
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