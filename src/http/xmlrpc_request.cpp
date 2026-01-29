//
// Created by dkargin on 12/XX/25.
//

#include "miniros/http/xmlrpc_request.h"
#include "miniros/internal/xml_tools.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"

#include <sstream>
#include <cstdio>

namespace miniros {
namespace http {

// Static constants matching XmlRpcClient
static const char REQUEST_BEGIN[] = 
  "<?xml version=\"1.0\"?>\r\n"
  "<methodCall><methodName>";
static const char REQUEST_END_METHODNAME[] = "</methodName>\r\n";
static const char PARAMS_TAG[] = "<params>";
static const char PARAMS_ETAG[] = "</params>";
static const char PARAM_TAG[] = "<param>";
static const char PARAM_ETAG[] =  "</param>";
static const char REQUEST_END[] = "</methodCall>\r\n";
static const char METHODRESPONSE_TAG[] = "<methodResponse>";
static const char FAULT_TAG[] = "<fault>";

XmlRpcRequest::XmlRpcRequest()
  : HttpRequest(HttpMethod::Post, "/RPC2")
{
}

XmlRpcRequest::XmlRpcRequest(const char* methodName, const char* uri)
  : HttpRequest(HttpMethod::Post, uri ? uri : "/RPC2"),
    method_name_(methodName ? methodName : "")
{
  // Set Content-Type header for XML-RPC
  setHeader("Content-Type", "text/xml");
  // Set User-Agent header
  setHeader("User-Agent", XmlRpc::XMLRPC_VERSION);
}

void XmlRpcRequest::setMethodName(const char* methodName)
{
  method_name_ = methodName ? methodName : "";
}

const std::string& XmlRpcRequest::methodName() const
{
  return method_name_;
}

void XmlRpcRequest::setParamArray(const XmlRpc::XmlRpcValue& params)
{
  params_ = params;
  generateRequestBody();
}

void XmlRpcRequest::setParams(const XmlRpc::XmlRpcValue& param0)
{
  if (params_.getType() != RpcValue::TypeArray) {
    params_ = RpcValue::Array(1);
  } else if (params_.size() != 1) {
    params_.setSize(1);
  }
  params_[0] = param0;
  generateRequestBody();
}

void XmlRpcRequest::setParams(const RpcValue& param0, const RpcValue& param1)
{
  if (params_.getType() != RpcValue::TypeArray) {
    params_ = RpcValue::Array(2);
  } else if (params_.size() != 2) {
    params_.setSize(2);
  }
  params_[0] = param0;
  params_[1] = param1;
  generateRequestBody();
}

void XmlRpcRequest::setParams(const RpcValue& param0, const RpcValue& param1, const RpcValue& param2)
{
  if (params_.getType() != RpcValue::TypeArray) {
    params_ = RpcValue::Array(3);
  } else if (params_.size() != 3) {
    params_.setSize(3);
  }
  params_[0] = param0;
  params_[1] = param1;
  params_[2] = param2;
  generateRequestBody();
}

const XmlRpc::XmlRpcValue& XmlRpcRequest::params() const
{
  return params_;
}

void XmlRpcRequest::generateRequestBody()
{
  // Generate the XML-RPC request body
  std::string body = generateRequestXml();
  
  // Set it as the HTTP request body
  setRequestBody(body);
  
  // Ensure Content-Type is set
  setHeader("Content-Type", "text/xml");
  
  // Ensure User-Agent is set
  if (getHeader("User-Agent").empty()) {
    setHeader("User-Agent", XmlRpc::XMLRPC_VERSION);
  }
}

std::string XmlRpcRequest::generateRequestXml() const
{
  std::string body = REQUEST_BEGIN;
  body += method_name_;
  body += REQUEST_END_METHODNAME;

  // If params is an array, each element is a separate parameter
  body += PARAMS_TAG;
  if (params_.valid()) {
    if (params_.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i=0; i<params_.size(); ++i) {
        body += PARAM_TAG;
        body += params_[i].toXml();
        body += PARAM_ETAG;
      }
    }
    else
    {
      body += PARAM_TAG;
      body += params_.toXml();
      body += PARAM_ETAG;
    }
  }
  body += PARAMS_ETAG;
  body += REQUEST_END;
  
  return body;
}

Error XmlRpcRequest::processResponse()
{
  if (onComplete) {
    auto [code, msg,data] = parseResponse();
    onComplete(code, msg, data);
  }
  return Error::Ok;
}

std::tuple<int, std::string, XmlRpc::XmlRpcValue> XmlRpcRequest::parseResponse() const
{
  XmlRpc::XmlRpcValue value;
  using Type = XmlRpc::XmlRpcValue::Type;
  bool ok = false;

  std::string_view responseView = response_body_;
  if (!parseResponseImpl(responseView, value, ok)) {
    return {0, {}, "failed to parse response"};
  }

  if (value.getType() != Type::TypeArray) {
    return {0, {}, "invalid response: not an array"};
  }

  if (value.size() < 2 && value[0].getType() != Type::TypeInt) {
    return {0, {}, "invalid response: unexpected value"};
  }

  int res = 0;
  XmlRpc::XmlRpcValue data;
  std::string msg;

  if (value.size() > 0) {
    res = value[0].as<int>();
  }

  if (value.size() > 1 && value[1].getType() == Type::TypeString) {
    msg = value[1].as<std::string>();
  }

  if (value.size() > 2) {
    data = value[2];
  }

  return {res, msg, data};
}

bool XmlRpcRequest::parseResponseImpl(const std::string_view& responseView, XmlRpc::XmlRpcValue& result, bool& isFault)
{
  std::string response(responseView);

  // Parse response xml into result
  int offset = 0;
  if ( ! XmlRpc::XmlRpcUtil::findTag(METHODRESPONSE_TAG, response, &offset)) {
    XmlRpc::XmlRpcUtil::error("Error in XmlRpcRequest::parseResponse: Invalid response - no methodResponse. Response:\n%s", response.c_str());
    return false;
  }

  // Expect either <params><param>... or <fault>...
  if ((XmlRpc::XmlRpcUtil::nextTagIs(PARAMS_TAG, response, &offset) && XmlRpc::XmlRpcUtil::nextTagIs(PARAM_TAG, response,&offset))
       || (XmlRpc::XmlRpcUtil::nextTagIs(FAULT_TAG, response, &offset) && (isFault = true))) //< isFault assignment is intended behaviour
  {
    xml::XmlCodec codec;
    size_t offsetCopy = offset;
    if ( !codec.parseXmlRpcValue(result, responseView, offsetCopy))
    {
      XmlRpc::XmlRpcUtil::error("Error in XmlRpcRequest::parseResponse: Invalid response value. Response:\n%s", response.c_str());
      return false;
    }
    offset = offsetCopy;
  } else {
    XmlRpc::XmlRpcUtil::error("Error in XmlRpcRequest::parseResponse: Invalid response - no param or fault tag. Response:\n%s", response.c_str());
    return false;
  }
      
  return result.valid();
}

void XmlRpcRequest::reset()
{
  // Call base class reset
  HttpRequest::reset();
  
  // Reset XML-RPC specific fields
  method_name_.clear();
  params_.clear();
  
  // Reset to POST method and default URI
  setMethod(HttpMethod::Post);
  setPath("/RPC2");
  
  // Set default headers
  setHeader("Content-Type", "text/xml");
  setHeader("User-Agent", XmlRpc::XMLRPC_VERSION);
}

} // namespace http
} // namespace miniros
