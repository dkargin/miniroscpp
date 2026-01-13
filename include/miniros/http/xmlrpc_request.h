//
// Created by dkargin on 12/XX/25.
//

#ifndef MINIROS_HTTP_XMLRPC_REQUEST_H
#define MINIROS_HTTP_XMLRPC_REQUEST_H

#include "miniros/http/http_request.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

#include <string>
#include <string_view>

namespace miniros {
namespace http {

  //! A class to represent XML-RPC requests that can be used with HttpClient.
  //! This class extends HttpRequest to automatically handle XML-RPC encoding
  //! and decoding following the XML-RPC protocol specification.
  class MINIROS_DECL XmlRpcRequest : public HttpRequest {
  public:
    using RpcValue = XmlRpc::XmlRpcValue;

    //! Construct an empty XML-RPC request
    XmlRpcRequest();

    //! Construct an XML-RPC request with method name and URI
    //! @param methodName The name of the remote procedure to call
    //! @param uri The URI path for the XML-RPC endpoint (default: "/RPC2")
    XmlRpcRequest(const char* methodName, const char* uri = "/RPC2");

    //! Set the XML-RPC method name
    void setMethodName(const char* methodName);

    //! Get the XML-RPC method name
    const std::string& methodName() const;

    //! Set the parameters for the XML-RPC call
    //! @param params The parameters as an XmlRpcValue (can be an array or a single value)
    void setParamArray(const RpcValue& params);

    void setParams(const RpcValue& param0, const RpcValue& param1, const RpcValue& param2);

    void setParams(const RpcValue& param0, const RpcValue& param1);

    void setParams(const RpcValue& param0);

    //! Get the parameters
    const RpcValue& params() const;

    //! Generate the XML-RPC request body and set it as the HTTP request body.
    //! This should be called before sending the request via HttpClient.
    //! The method automatically sets the Content-Type header to "text/xml".
    void generateRequestBody();

    /// Parse ROS-specific XMLRPC response.
    std::tuple<int, XmlRpc::XmlRpcValue, std::string> parseResponse() const;

    //! Reset the request for reuse
    void reset() override;

    Error processResponse() override;

    std::function<void (int code, const XmlRpc::XmlRpcValue& data, const std::string msg)> onComplete;

  private:
    //! Generate the XML-RPC request body following the encoding scheme from XmlRpcClient
    std::string generateRequestXml() const;

    //! Internal implementation of parseResponse that takes a string_view
    static bool parseResponseImpl(const std::string_view& responseView, XmlRpc::XmlRpcValue& result, bool& isFault);

    std::string method_name_;
    RpcValue params_ = RpcValue::Array(0);
  };

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_XMLRPC_REQUEST_H
