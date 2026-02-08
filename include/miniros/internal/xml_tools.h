//
// Created by dkargin on 11/23/25.
//

#ifndef MINIROS_INTERNAL_XML_H
#define MINIROS_INTERNAL_XML_H

#include <string>

#include "miniros/macros.h"

namespace XmlRpc {
class XmlRpcValue;
}

namespace miniros {
namespace xml {

struct MINIROS_DECL XmlCodec {
  using Value = XmlRpc::XmlRpcValue;

  /// Replace XML escape sequences by original characters.
  static std::string decode(const std::string_view& encoded);

  /// Replace characters by XML escape sequences.
  static std::string encode(const std::string_view& raw);

  /// Parses XMLRPC request.
  MINIROS_NODISCARD static bool parseXmlRpcRequest(const std::string_view& data, std::string_view& method, Value& params);

  /// XMLRPC decoding
  /// @param value - reference to value container.
  /// @param data - raw XMLRPC data.
  /// @param offset - offset in XMLRPC data.
  MINIROS_NODISCARD static bool parseXmlRpcValue(Value& value, const std::string_view& data, size_t& offset);

  /** @brief Validate an XML/RPC response
   *
   * @param method The RPC method that was invoked.
   * @param response The resonse that was received.
   * @param payload The payload that was received.
   *
   * @return true if validation succeeds, false otherwise.
   */
  MINIROS_NODISCARD static bool validateXmlrpcResponse(const std::string& method, const Value &response, Value &payload);

  static bool parseXmlRpcBool(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcInt(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcDouble(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcString(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcTime(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcBinary(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcArray(Value& value, const std::string_view& data, size_t& offset);
  static bool parseXmlRpcStruct(Value& value, const std::string_view& data, size_t& offset);
};

} // namespace xml
} // namespace miniros

#endif // MINIROS_INTERNAL_XML_H
