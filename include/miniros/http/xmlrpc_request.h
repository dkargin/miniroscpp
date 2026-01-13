//
// Created by dkargin on 12/XX/25.
//

#ifndef MINIROS_HTTP_XMLRPC_REQUEST_H
#define MINIROS_HTTP_XMLRPC_REQUEST_H

#include "miniros/http/http_request.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"
#include "miniros/xmlrpcpp/XmlRpcUtil.h"
#include "miniros/xmlrpcpp/XmlRpcDecl.h"

#include <string>
#include <string_view>

namespace miniros {
namespace http {

  //! A class to represent XML-RPC requests that can be used with HttpClient.
  //! This class extends HttpRequest to automatically handle XML-RPC encoding
  //! and decoding following the XML-RPC protocol specification.
  class MINIROS_DECL XmlRpcRequest : public HttpRequest {
  public:
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
    void setParams(const XmlRpc::XmlRpcValue& params);

    //! Get the parameters
    const XmlRpc::XmlRpcValue& params() const;

    //! Generate the XML-RPC request body and set it as the HTTP request body.
    //! This should be called before sending the request via HttpClient.
    //! The method automatically sets the Content-Type header to "text/xml".
    void generateRequestBody();

    //! Parse the XML-RPC response from the HTTP response body.
    //! @param result The result value to be populated
    //! @param isFault Output parameter indicating if the response is a fault
    //! @return true if parsing was successful, false otherwise
    bool parseResponse(XmlRpc::XmlRpcValue& result, bool& isFault) const;

    //! Parse the XML-RPC response from the HTTP response body.
    //! This version uses the response body stored in the HttpRequest.
    //! @param result The result value to be populated
    //! @return true if parsing was successful, false otherwise
    bool parseResponse(XmlRpc::XmlRpcValue& result) const;

    //! Reset the request for reuse
    void reset() override;

  private:
    //! Generate the XML-RPC request body following the encoding scheme from XmlRpcClient
    std::string generateRequestXml() const;

    //! Internal implementation of parseResponse that takes a string_view
    bool parseResponseImpl(const std::string_view& responseView, XmlRpc::XmlRpcValue& result, bool& isFault) const;

    std::string method_name_;
    XmlRpc::XmlRpcValue params_;
  };

} // namespace http
} // namespace miniros

#endif // MINIROS_HTTP_XMLRPC_REQUEST_H
