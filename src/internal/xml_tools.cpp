//
// Created by dkargin on 11/23/25.
//
#include <cassert>
#include <cstring>

#include "miniros/errors.h"
#include "miniros/internal/xml_tools.h"
#include "miniros/b64/decode.h"
#include "miniros/xmlrpcpp/XmlRpcValue.h"

namespace miniros {
namespace xml {
// Returns contents between <tag> and </tag>, updates offset to char after </tag>
std::string_view parseXmlTag(const char* tag, std::string_view const& data, size_t& offset)
{
  if (offset >= data.length())
    return {};
  size_t istart = data.find(tag, offset);
  if (istart == std::string::npos)
    return {};
  istart += strlen(tag);

  /// TODO: Make it allocation-free.
  std::string etag = "</";
  etag += tag + 1;
  size_t iend = data.find(etag, istart);
  if (iend == std::string::npos)
    return {};

  offset = iend + etag.length();
  return data.substr(istart, iend-istart);
}

// Returns true if the tag is found and updates offset to the char after the tag
bool findXmlTag(const char* tag, std::string_view const& data, size_t& offset)
{
  if (offset >= data.size())
    return false;
  size_t istart = data.find(tag, offset);
  if (istart == std::string::npos)
    return false;

  offset = istart + strlen(tag);
  return true;
}

// Returns true if the tag is found at the specified offset (modulo any whitespace)
// and updates offset to the char after the tag
bool nextXmlTagIs(const char* tag, std::string_view const& data, size_t& offset)
{
  if (offset >= data.size())
    return false;
  const char* cp = &data[offset];
  const char* end = data.data() + data.size();

  // Drop all whitespaces.
  size_t nc = 0;
  while (cp < end && *cp && isspace(*cp)) {
    ++cp;
    ++nc;
  }

  size_t len = strlen(tag);
  if  (*cp && (strncmp(cp, tag, len) == 0)) {
    offset += nc + len;
    return true;
  }
  return false;
}

// Returns the next tag and updates offset to the char after the tag, or empty string
// if the next non-whitespace character is not '<'
std::string_view getNextXmlTag(const std::string_view& xml, size_t& offset)
{
  if (offset >= xml.size())
    return {};

  size_t pos = offset;
  const char* cp = xml.data() + pos;
  const char* end = xml.data() + xml.size();

  while (cp < end && *cp && isspace(*cp)) {
    ++cp;
    ++pos;
  }

  if (*cp != '<')
    return {};

  size_t tagStart = pos;
  do {
    ++pos;
  } while (cp < end && *cp++ != '>' && *cp != 0);

  offset = pos;
  return std::string_view(&xml[tagStart], pos - tagStart);
}

namespace {
// xml encodings (xml-encoded entities are preceded with '&')
constexpr char  AMP = '&';
constexpr char  rawEntity[] = { '<',   '>',   '&',    '\'',    '\"',    0 };
constexpr char* const xmlEntity[] = { "lt;", "gt;", "amp;", "apos;", "quot;", 0 };
constexpr int   xmlEntLen[] = { 3,     3,     4,      5,       5 };
}

std::string XmlCodec::decode(const std::string_view& encoded)
{
  std::string::size_type iAmp = encoded.find(AMP);
  if (iAmp == std::string::npos)
    return std::string(encoded);

  std::string decoded(encoded, 0, iAmp);
  std::string::size_type iSize = encoded.size();
  decoded.reserve(iSize);

  const char* ens = encoded.data();
  while (iAmp != iSize) {
    if (encoded[iAmp] == AMP && iAmp+1 < iSize) {
      int iEntity;
      for (iEntity=0; xmlEntity[iEntity] != 0; ++iEntity) {
        if (strncmp(ens+iAmp+1, xmlEntity[iEntity], xmlEntLen[iEntity]) == 0)
        {
          decoded += rawEntity[iEntity];
          iAmp += xmlEntLen[iEntity]+1;
          break;
        }
      }
      if (xmlEntity[iEntity] == 0)    // unrecognized sequence
        decoded += encoded[iAmp++];
    } else {
      decoded += encoded[iAmp++];
    }
  }

  return decoded;
}

std::string XmlCodec::encode(const std::string_view& raw)
{
  std::string::size_type iRep = raw.find_first_of(rawEntity);
  if (iRep == std::string::npos)
    return std::string(raw);

  std::string encoded(raw, 0, iRep);
  std::string::size_type iSize = raw.size();

  while (iRep != iSize) {
    int iEntity = 0;
    for (; rawEntity[iEntity] != 0; ++iEntity) {
      if (raw[iRep] == rawEntity[iEntity])
      {
        encoded += AMP;
        encoded += xmlEntity[iEntity];
        break;
      }
    }
    if (rawEntity[iEntity] == 0) {
      encoded += raw[iRep];
    }
    ++iRep;
  }
  return encoded;
}

namespace {
constexpr char METHODRESPONSE_TAG[] = "<methodResponse>";
constexpr char METHODNAME_TAG[] = "<methodName>";
constexpr char PARAMS_ETAG[] = "</params>";
constexpr char PARAM_ETAG[] = "</param>";
constexpr char PARAMS_TAG[] = "<params>";
constexpr char PARAM_TAG[] = "<param>";
constexpr char PARAMS[] = "params";
constexpr char FAULT_TAG[] = "<fault>";

constexpr char VALUE_TAG[]     = "<value>";
constexpr char VALUE_ETAG[]    = "</value>";

constexpr char BOOLEAN_TAG[]   = "<boolean>";
constexpr char BOOLEAN_ETAG[]  = "</boolean>";
constexpr char DOUBLE_TAG[]    = "<double>";
constexpr char DOUBLE_ETAG[]   = "</double>";
constexpr char INT_TAG[]       = "<int>";
constexpr char I4_TAG[]        = "<i4>";
constexpr char I4_ETAG[]       = "</i4>";
constexpr char STRING_TAG[]    = "<string>";
constexpr char DATETIME_TAG[]  = "<dateTime.iso8601>";
constexpr char DATETIME_ETAG[] = "</dateTime.iso8601>";
constexpr char BASE64_TAG[]    = "<base64>";
constexpr char BASE64_ETAG[]   = "</base64>";

constexpr char ARRAY_TAG[]     = "<array>";
constexpr char DATA_TAG[]      = "<data>";
constexpr char DATA_ETAG[]     = "</data>";
constexpr char ARRAY_ETAG[]    = "</array>";

constexpr char STRUCT_TAG[]    = "<struct>";
constexpr char MEMBER_TAG[]    = "<member>";
constexpr char NAME_TAG[]      = "<name>";
constexpr char NAME_ETAG[]     = "</name>";
constexpr char MEMBER_ETAG[]   = "</member>";
constexpr char STRUCT_ETAG[]   = "</struct>";

}

// Parse the method name and the argument values from the request.
bool XmlCodec::parseXmlRpcRequest(const std::string_view& request, std::string_view& method, Value& params)
{
  size_t offset = 0;   // Number of chars parsed from the request

  method = parseXmlTag(METHODNAME_TAG, request, offset);
  if (method.empty())
    return false;

  if (!findXmlTag(PARAMS_TAG, request, offset))
    return false;

  Value::ValueArray array;
  while (nextXmlTagIs(PARAM_TAG, request, offset)) {
    Value val;
    if (!parseXmlRpcValue(val, request, offset)) {
      return false;
    }
    array.push_back(val);
    (void) nextXmlTagIs(PARAM_ETAG, request, offset);
  }

  (void) nextXmlTagIs(PARAMS_ETAG, request, offset);
  params.setArray(std::move(array));
  return true;
}

// Set the value from xml. The chars at *offset into valueXml
// should be the start of a <value> tag. Destroys any existing value.
bool XmlCodec::parseXmlRpcValue(XmlRpc::XmlRpcValue& value, std::string_view const& valueXml, size_t& offset)
{
  size_t savedOffset = offset;

  value.clear();

  if ( !nextXmlTagIs(VALUE_TAG, valueXml, offset))
    return false;       // Not a value, offset not updated

  size_t afterValueOffset = offset;
  std::string_view typeTag = getNextXmlTag(valueXml, offset);

  bool result = false;
  if (typeTag == BOOLEAN_TAG)
    result = parseXmlRpcBool(value, valueXml, offset);
  else if (typeTag == I4_TAG || typeTag == INT_TAG)
    result = parseXmlRpcInt(value, valueXml, offset);
  else if (typeTag == DOUBLE_TAG)
    result = parseXmlRpcDouble(value, valueXml, offset);
  else if (typeTag.empty() || typeTag == STRING_TAG)
    result = parseXmlRpcString(value, valueXml, offset);
  else if (typeTag == DATETIME_TAG)
    result = parseXmlRpcTime(value, valueXml, offset);
  else if (typeTag == BASE64_TAG)
    result = parseXmlRpcBinary(value, valueXml, offset);
  else if (typeTag == ARRAY_TAG)
    result = parseXmlRpcArray(value, valueXml, offset);
  else if (typeTag == STRUCT_TAG)
    result = parseXmlRpcStruct(value, valueXml, offset);
  // Watch for empty/blank strings with no <string>tag
  else if (typeTag == VALUE_ETAG)
  {
    offset = afterValueOffset;   // back up & try again
    result = parseXmlRpcString(value, valueXml, offset);
  }

  if (result) {  // Skip over the </value> tag
    findXmlTag(VALUE_ETAG, valueXml, offset);
  }
  else        // Unrecognized tag after <value>
    offset = savedOffset;

  return result;
}

// Convert the response xml into a result value
Error parseXmlRpcResponse(const std::string_view& responseView, XmlRpc::XmlRpcValue& result, bool& isFault)
{
  std::string response(responseView);

  // Parse response xml into result
  size_t offset = 0;
  if ( !findXmlTag(METHODRESPONSE_TAG, response, offset)) {
    return Error::InvalidResponse;
  }

  XmlCodec codec;
  // Expect either <params><param>... or <fault>...
  if ((nextXmlTagIs(PARAMS_TAG, response,offset) && nextXmlTagIs(PARAM_TAG, response, offset))
       || (nextXmlTagIs(FAULT_TAG, response,offset) && (isFault = true))) //< _isFault assignment is intended behaviour
  {
    if ( !codec.parseXmlRpcValue(result, response, offset)) {
      //XmlRpcUtil::error("Error in XmlRpcClient(%s)::parseResponse: Invalid response value. Response:\n%s", name().c_str(), response.c_str());
      return Error::InvalidResponse;
    }
  } else {
    //XmlRpcUtil::error("Error in XmlRpcClient(%s)::parseResponse: Invalid response - no param or fault tag. Response:\n%s", name().c_str(), response.c_str());
    return Error::InvalidResponse;
  }

  if (!result.valid())
    return Error::InvalidValue;
  return Error::Ok;
}


bool XmlCodec::parseXmlRpcBool(Value& value, const std::string_view& valueXml, size_t& offset)
{
  const char* valueStart = &valueXml[offset];
  char* valueEnd;
  long ivalue = strtol(valueStart, &valueEnd, 10);
  if (valueEnd == valueStart || (ivalue != 0 && ivalue != 1))
    return false;

  value.setBool(ivalue == 1);
  offset += valueEnd - valueStart;
  return true;
}

bool XmlCodec::parseXmlRpcInt(Value& value, const std::string_view& valueXml, size_t& offset)
{
  const char* valueStart = &valueXml[offset];
  char* valueEnd;
  long ivalue = strtol(valueStart, &valueEnd, 10);
  if (valueEnd == valueStart)
    return false;

  value.setInt(ivalue);
  offset += valueEnd - valueStart;
  return true;
}

bool XmlCodec::parseXmlRpcDouble(Value& value, const std::string_view& valueXml, size_t& offset)
{
  const char* valueStart = &valueXml[offset];
  char* valueEnd;

  // ticket #2438
  // push/pop the locale here. Value 123.45 can get read by strtod
  // as '123', if the locale expects a comma instead of dot.
  // if there are locale problems, silently continue.
  std::string tmplocale;
  char* locale_cstr = setlocale(LC_NUMERIC, 0);
  if (locale_cstr)
  {
    tmplocale = locale_cstr;
    setlocale(LC_NUMERIC, "POSIX");
  }

  double dvalue = strtod(valueStart, &valueEnd);

  if (tmplocale.size() > 0) {
    setlocale(LC_NUMERIC, tmplocale.c_str());
  }

  if (valueEnd == valueStart)
    return false;

  value.setDouble(dvalue);
  offset += valueEnd - valueStart;
  return true;
}

bool XmlCodec::parseXmlRpcString(Value& value, const std::string_view& valueXml, size_t& offset)
{
  size_t valueEnd = valueXml.find('<', offset);
  if (valueEnd == std::string::npos)
    return false;     // No end tag;

  std::string decoded = decode(valueXml.substr(offset, valueEnd-offset));
  offset += decoded.size();
  value.setString(std::move(decoded));
  return true;
}

bool XmlCodec::parseXmlRpcTime(Value& value, const std::string_view& valueXml, size_t& offset)
{
  size_t valueEnd = valueXml.find('<', offset);
  if (valueEnd == std::string::npos)
    return false;     // No end tag;

  std::string_view stime = valueXml.substr(offset, valueEnd-offset);

  struct tm t;
#ifdef _MSC_VER
  if (sscanf_s(stime.data(),"%4d%2d%2dT%2d:%2d:%2d",&t.tm_year,&t.tm_mon,&t.tm_mday,&t.tm_hour,&t.tm_min,&t.tm_sec) != 6)
#else
  if (sscanf(stime.data(),"%4d%2d%2dT%2d:%2d:%2d",&t.tm_year,&t.tm_mon,&t.tm_mday,&t.tm_hour,&t.tm_min,&t.tm_sec) != 6)
#endif
    return false;

  t.tm_isdst = -1;
  value.setDateTime(t);
  offset += stime.length();
  return true;
}

namespace {
std::size_t base64EncodedSize(std::size_t raw_size)
{
  // encoder will still write to output buffer for empty input.
  if (raw_size == 0) return 1;

  // 4 encoded character per 3 input bytes, rounded up,
  // plus a newline character per 72 output characters, rounded up.
  std::size_t encoded = (raw_size + 2) / 3 * 4;
  encoded += (encoded + 71) / 72;
  return encoded;
}

std::size_t base64DecodedSize(std::size_t encoded_size)
{
  // decoded will still write to output buffer for empty input.
  if (encoded_size == 0) return 1;

  // 3 decoded bytes per 4 encoded characters, rounded up just to be sure.
  return (encoded_size + 3) / 4 * 3;
}

}

// Base64
bool XmlCodec::parseXmlRpcBinary(Value& value, const std::string_view& valueXml, size_t& offset)
{
  size_t valueEnd = valueXml.find('<', offset);
  if (valueEnd == std::string::npos)
    return false;     // No end tag;

  std::size_t encodedSize = valueEnd - offset;

  size_t decodedSize = base64DecodedSize(encodedSize);
  Value::BinaryData binary(decodedSize, '\0');

  base64::Decoder b64decoder;

  std::size_t size = b64decoder.decode(&valueXml[offset], encodedSize, binary.data());
  binary.resize(size);

  value.setBinary(std::move(binary));
  offset += encodedSize;
  return true;
}

// Array
bool XmlCodec::parseXmlRpcArray(Value& value, const std::string_view& valueXml, size_t& offset)
{
  if ( ! nextXmlTagIs(DATA_TAG, valueXml, offset))
    return false;

  std::vector<Value> array;
  {
    Value v;
    while (parseXmlRpcValue(v, valueXml, offset))
      array.push_back(std::move(v));       // copy...
  }

  value.setArray(std::move(array));

  // Skip the trailing </data>
  (void) nextXmlTagIs(DATA_ETAG, valueXml, offset);
  return true;
}

bool XmlCodec::parseXmlRpcStruct(Value& value, const std::string_view& valueXml, size_t& offset)
{
  Value::ValueStruct map;

  while (nextXmlTagIs(MEMBER_TAG, valueXml, offset)) {
    // name
    const std::string_view name = parseXmlTag(NAME_TAG, valueXml, offset);
    // value
    Value val;
    if (!parseXmlRpcValue(val, valueXml, offset))
      return false;
    if (!val.valid())
      return false;
    const std::pair<const std::string, Value> p(name, val);
    map.insert(p);

    (void) nextXmlTagIs(MEMBER_ETAG, valueXml, offset);
  }

  value.setStruct(std::move(map));
  return true;
}
} // namespace xml
} // namespace miniros