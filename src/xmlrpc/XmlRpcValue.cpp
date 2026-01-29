
#include "xmlrpcpp/XmlRpcValue.h"
#include "xmlrpcpp/XmlRpcException.h"
#include "xmlrpcpp/XmlRpcUtil.h"

#include "b64/encode.h"
#include "b64/decode.h"

#ifndef MAKEDEPEND
# include <ostream>
# include <stdio.h>
#endif

#include <sstream>

namespace XmlRpc {

  static const char VALUE_TAG[]     = "<value>";
  static const char VALUE_ETAG[]    = "</value>";

  static const char BOOLEAN_TAG[]   = "<boolean>";
  static const char BOOLEAN_ETAG[]  = "</boolean>";
  static const char DOUBLE_TAG[]    = "<double>";
  static const char DOUBLE_ETAG[]   = "</double>";
  static const char INT_TAG[]       = "<int>";
  static const char I4_TAG[]        = "<i4>";
  static const char I4_ETAG[]       = "</i4>";
  static const char STRING_TAG[]    = "<string>";
  static const char DATETIME_TAG[]  = "<dateTime.iso8601>";
  static const char DATETIME_ETAG[] = "</dateTime.iso8601>";
  static const char BASE64_TAG[]    = "<base64>";
  static const char BASE64_ETAG[]   = "</base64>";

  static const char ARRAY_TAG[]     = "<array>";
  static const char DATA_TAG[]      = "<data>";
  static const char DATA_ETAG[]     = "</data>";
  static const char ARRAY_ETAG[]    = "</array>";

  static const char STRUCT_TAG[]    = "<struct>";
  static const char MEMBER_TAG[]    = "<member>";
  static const char NAME_TAG[]      = "<name>";
  static const char NAME_ETAG[]     = "</name>";
  static const char MEMBER_ETAG[]   = "</member>";
  static const char STRUCT_ETAG[]   = "</struct>";

  XmlRpcValue XmlRpcValue::Array(size_t size)
  {
    XmlRpcValue value;
    value.setSize(size);
    return value;
  }

  XmlRpcValue XmlRpcValue::Dict()
  {
    XmlRpcValue value;

    value.assertStruct();
    return value;
  }

  // Clean up
  void XmlRpcValue::invalidate()
  {
    switch (_type) {
      case TypeString:    delete _value.asString; break;
      case TypeDateTime:  delete _value.asTime;   break;
      case TypeBase64:    delete _value.asBinary; break;
      case TypeArray:     delete _value.asArray;  break;
      case TypeStruct:    delete _value.asStruct; break;
      default: break;
    }
    _type = TypeInvalid;
    _value.asBinary = 0;
  }

  
  // Type checking
  void XmlRpcValue::assertTypeOrInvalid(Type t)
  {
    if (_type == TypeInvalid)
    {
      _type = t;
      switch (_type) {    // Ensure there is a valid value for the type
        case TypeString:   _value.asString = new std::string(); break;
        case TypeDateTime: _value.asTime = new struct tm();     break;
        case TypeBase64:   _value.asBinary = new BinaryData();  break;
        case TypeArray:    _value.asArray = new ValueArray();   break;
        case TypeStruct:   _value.asStruct = new ValueStruct(); break;
        default:           _value.asBinary = 0; break;
      }
    }
    else if (_type != t)
      throw XmlRpcException("type error");
  }

  void XmlRpcValue::assertType(Type t) const
  {
    if (_type != t)
      throw XmlRpcException("type error");
  }

  void XmlRpcValue::assertArrayConst(size_t size) const
  {
    if (_type != TypeArray)
      throw XmlRpcException("type error: expected an array");
    else if (_value.asArray->size() < size)
      throw XmlRpcException("range error: array index too large");
  }


  void XmlRpcValue::assertArray(size_t size)
  {
    if (_type == TypeInvalid) {
      _type = TypeArray;
      _value.asArray = new ValueArray(size);
    } else if (_type == TypeArray) {
      if (_value.asArray->size() < size)
        _value.asArray->resize(size);
    } else
      throw XmlRpcException("type error: expected an array");
  }

  void XmlRpcValue::assertStructConst() const
  {
    if (_type != TypeStruct)
      throw XmlRpcException("type error: expected a struct");
  }

  void XmlRpcValue::assertStruct()
  {
    if (_type == TypeInvalid) {
      _type = TypeStruct;
      _value.asStruct = new ValueStruct();
    } else if (_type != TypeStruct)
      throw XmlRpcException("type error: expected a struct");
  }


  // Operators
  XmlRpcValue& XmlRpcValue::operator=(XmlRpcValue const& rhs)
  {
    if (this != &rhs)
    {
      invalidate();
      _type = rhs._type;
      switch (_type) {
        case TypeBoolean:  _value.asBool = rhs._value.asBool; break;
        case TypeInt:      _value.asInt = rhs._value.asInt; break;
        case TypeDouble:   _value.asDouble = rhs._value.asDouble; break;
        case TypeDateTime: _value.asTime = new struct tm(*rhs._value.asTime); break;
        case TypeString:   _value.asString = new std::string(*rhs._value.asString); break;
        case TypeBase64:   _value.asBinary = new BinaryData(*rhs._value.asBinary); break;
        case TypeArray:    _value.asArray = new ValueArray(*rhs._value.asArray); break;
        case TypeStruct:   _value.asStruct = new ValueStruct(*rhs._value.asStruct); break;
        default:           _value.asBinary = 0; break;
      }
    }
    return *this;
  }


  // Predicate for tm equality
  static bool tmEq(struct tm const& t1, struct tm const& t2) {
    return t1.tm_sec == t2.tm_sec && t1.tm_min == t2.tm_min &&
            t1.tm_hour == t2.tm_hour && t1.tm_mday == t2.tm_mday &&
            t1.tm_mon == t2.tm_mon && t1.tm_year == t2.tm_year;
  }

  bool XmlRpcValue::operator==(XmlRpcValue const& other) const
  {
    if (_type != other._type)
      return false;

    switch (_type) {
      case TypeBoolean:  return ( !_value.asBool && !other._value.asBool) ||
                                ( _value.asBool && other._value.asBool);
      case TypeInt:      return _value.asInt == other._value.asInt;
      case TypeDouble:   return _value.asDouble == other._value.asDouble;
      case TypeDateTime: return tmEq(*_value.asTime, *other._value.asTime);
      case TypeString:   return *_value.asString == *other._value.asString;
      case TypeBase64:   return *_value.asBinary == *other._value.asBinary;
      case TypeArray:    return *_value.asArray == *other._value.asArray;

      // The map<>::operator== requires the definition of value< for kcc
      case TypeStruct:   //return *_value.asStruct == *other._value.asStruct;
        {
          if (_value.asStruct->size() != other._value.asStruct->size())
            return false;
          
          ValueStruct::const_iterator it1=_value.asStruct->begin();
          ValueStruct::const_iterator it2=other._value.asStruct->begin();
          while (it1 != _value.asStruct->end()) {
            const XmlRpcValue& v1 = it1->second;
            const XmlRpcValue& v2 = it2->second;
            if ( ! (v1 == v2))
              return false;
            it1++;
            it2++;
          }
          return true;
        }
      default: break;
    }
    return true;    // Both invalid values ...
  }

  bool XmlRpcValue::operator!=(XmlRpcValue const& other) const
  {
    return !(*this == other);
  }

  bool XmlRpcValue::isPrimitive() const
  {
    return _type != TypeStruct && _type != TypeArray && _type != TypeInvalid;
  }

  // Works for strings, binary data, arrays, and structs.
  int XmlRpcValue::size() const
  {
    switch (_type) {
      case TypeString: return int(_value.asString->size());
      case TypeBase64: return int(_value.asBinary->size());
      case TypeArray:  return int(_value.asArray->size());
      case TypeStruct: return int(_value.asStruct->size());
      default: break;
    }

    throw XmlRpcException("type error");
  }

  // Checks for existence of struct member
  bool XmlRpcValue::hasMember(const std::string& name) const
  {
    return _type == TypeStruct && _value.asStruct->find(name) != _value.asStruct->end();
  }

  bool XmlRpcValue::eraseMember(const std::string& key)
  {
    if (_type != TypeStruct)
      return false;

    if (_value.asStruct->erase(key))
      return true;
    return false;
  }

  // Encode the Value in xml
  std::string XmlRpcValue::toXml() const
  {
    switch (_type) {
      case TypeBoolean:  return boolToXml();
      case TypeInt:      return intToXml();
      case TypeDouble:   return doubleToXml();
      case TypeString:   return stringToXml();
      case TypeDateTime: return timeToXml();
      case TypeBase64:   return binaryToXml();
      case TypeArray:    return arrayToXml();
      case TypeStruct:   return structToXml();
      default: break;
    }
    return std::string();   // Invalid value
  }

  std::string XmlRpcValue::boolToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += BOOLEAN_TAG;
    xml += (_value.asBool ? "1" : "0");
    xml += BOOLEAN_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  std::string XmlRpcValue::intToXml() const
  {
    char buf[256];
    std::snprintf(buf, sizeof(buf)-1, "%d", _value.asInt);
    buf[sizeof(buf)-1] = 0;
    std::string xml = VALUE_TAG;
    xml += I4_TAG;
    xml += buf;
    xml += I4_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  std::string XmlRpcValue::doubleToXml() const
  {
    // ticket #2438
    std::stringstream ss;
    ss.imbue(std::locale::classic()); // ensure we're using "C" locale for formatting floating-point (1.4 vs. 1,4, etc.)
    ss.precision(17);
    ss << _value.asDouble;

    std::string xml = VALUE_TAG;
    xml += DOUBLE_TAG;
    xml += ss.str();
    xml += DOUBLE_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  std::string XmlRpcValue::stringToXml() const
  {
    std::string xml = VALUE_TAG;
    //xml += STRING_TAG; optional
    xml += XmlRpcUtil::xmlEncode(*_value.asString);
    //xml += STRING_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  std::string XmlRpcValue::timeToXml() const
  {
    struct tm* t = _value.asTime;
    char buf[20];
    std::snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d", 
      t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
    buf[sizeof(buf)-1] = 0;

    std::string xml = VALUE_TAG;
    xml += DATETIME_TAG;
    xml += buf;
    xml += DATETIME_ETAG;
    xml += VALUE_ETAG;
    return xml;
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

  std::string XmlRpcValue::binaryToXml() const
  {
    // Wrap with xml
    std::string xml = VALUE_TAG;
    xml += BASE64_TAG;

    std::size_t offset = xml.size();
    // might reserve too much, we'll shrink later
    xml.resize(xml.size() + base64EncodedSize(_value.asBinary->size()));

    base64::Encoder encoder;
    offset += encoder.encode(_value.asBinary->data(), _value.asBinary->size(), &xml[offset]);
    offset += encoder.encode_end(&xml[offset]);
    xml.resize(offset);

    xml += BASE64_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }


  // In general, its preferable to generate the xml of each element of the
  // array as it is needed rather than glomming up one big string.
  std::string XmlRpcValue::arrayToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += ARRAY_TAG;
    xml += DATA_TAG;

    int s = int(_value.asArray->size());
    for (int i=0; i<s; ++i)
       xml += _value.asArray->at(i).toXml();

    xml += DATA_ETAG;
    xml += ARRAY_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }

  // In general, its preferable to generate the xml of each element
  // as it is needed rather than glomming up one big string.
  std::string XmlRpcValue::structToXml() const
  {
    std::string xml = VALUE_TAG;
    xml += STRUCT_TAG;

    ValueStruct::const_iterator it;
    for (it=_value.asStruct->begin(); it!=_value.asStruct->end(); ++it) {
      xml += MEMBER_TAG;
      xml += NAME_TAG;
      xml += XmlRpcUtil::xmlEncode(it->first);
      xml += NAME_ETAG;
      xml += it->second.toXml();
      xml += MEMBER_ETAG;
    }

    xml += STRUCT_ETAG;
    xml += VALUE_ETAG;
    return xml;
  }



  // Write the value without xml encoding it
  std::ostream& XmlRpcValue::write(std::ostream& os) const {
    switch (_type) {
      default:           break;
      case TypeBoolean:  os << _value.asBool; break;
      case TypeInt:      os << _value.asInt; break;
      case TypeDouble:   os << _value.asDouble; break;
      case TypeString:   os << *_value.asString; break;
      case TypeDateTime:
        {
          struct tm* t = _value.asTime;
          char buf[20];
          std::snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d", 
            t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
          buf[sizeof(buf)-1] = 0;
          os << buf;
          break;
        }
      case TypeBase64:
        {
          std::stringstream buffer;
          buffer.write(_value.asBinary->data(), _value.asBinary->size());
          base64::Encoder encoder;
          encoder.encode(buffer, os);
          break;
        }
      case TypeArray:
        {
          int s = int(_value.asArray->size());
          os << '{';
          for (int i=0; i<s; ++i)
          {
            if (i > 0) os << ',';
            _value.asArray->at(i).write(os);
          }
          os << '}';
          break;
        }
      case TypeStruct:
        {
          os << '[';
          ValueStruct::const_iterator it;
          for (it=_value.asStruct->begin(); it!=_value.asStruct->end(); ++it)
          {
            if (it!=_value.asStruct->begin()) os << ',';
            os << it->first << ':';
            it->second.write(os);
          }
          os << ']';
          break;
        }
    }
    
    return os;
  }

  std::ostream& writePad(std::ostream& os, int num, char padding=' ')
  {
    for (int i=0; i<num; ++i) {
      os << padding;
    }
    return os;
  }

  std::ostream& XmlRpcValue::writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings) const
  {
    /*
    *{
      "first_name": "John",
      "last_name": "Smith",
      "is_alive": true,
      "age": 27,
      "address": {
        "street_address": "21 2nd Street",
        "city": "New York",
        "state": "NY",
        "postal_code": "10021-3100"
      },
     */
    switch (_type) {
      default:           break;
      case TypeInvalid:
        os << "null";
        break;
      case TypeStruct:
      {
        bool smallDict = (this->size() < 2);
        if (!state.sameline)
          writePad(os, state.offset);
        os << '{';
        if (!smallDict)
          os << std::endl;
        state.offset += settings.tabs;
        ValueStruct::const_iterator it;
        /*
         * "var" : { "var2" : 1 },
         */
        for (it=_value.asStruct->begin(); it!=_value.asStruct->end(); ++it)
        {
          // Adding comma from previous line.
          if (it != _value.asStruct->begin())
            os << ',' << std::endl;
          if (!smallDict)
            writePad(os, state.offset);
          os << "\"" << it->first << "\": ";
          bool same = state.sameline;
          state.sameline = true;
          it->second.writeJson(os, state, settings);
          state.sameline = same;
        }
        if (!smallDict)
          os << std::endl;
        state.offset -= settings.tabs;
        if (!smallDict)
          writePad(os, state.offset);
        os << '}';
        break;
      }
      case TypeArray:
      {
        size_t s = _value.asArray->size();
        bool smallArray = (s < 2);
        os << '[';
        if (!smallArray) {
          os << std::endl;
          state.offset += settings.tabs;
        }
        for (size_t i=0; i<s; ++i)
        {
          if (i > 0)
            os << ", " << std::endl;
          if (!smallArray) {
            writePad(os, state.offset);
          }
          _value.asArray->at(i).writeJson(os, state, settings);
        }
        if (!smallArray) {
          os << std::endl;
          state.offset -= settings.tabs;
        }
        if (!smallArray)
          writePad(os, state.offset);
        os << ']';
        break;
      }
      case TypeBoolean:  os << (_value.asBool ? "true" : "false"); break;
      case TypeInt:      os << _value.asInt; break;
      case TypeDouble:   os << _value.asDouble; break;
      case TypeString:   os << "\"" << *_value.asString << "\""; break;
      case TypeDateTime:
      {
        struct tm* t = _value.asTime;
        char buf[20];
        std::snprintf(buf, sizeof(buf)-1, "%4d%02d%02dT%02d:%02d:%02d",
          t->tm_year,t->tm_mon,t->tm_mday,t->tm_hour,t->tm_min,t->tm_sec);
        buf[sizeof(buf)-1] = 0;
        os << buf;
        break;
      }
      case TypeBase64:
      {
        std::stringstream buffer;
        buffer.write(_value.asBinary->data(), _value.asBinary->size());
        base64::Encoder encoder;
        encoder.encode(buffer, os);
        break;
      }
    } //switch
    return os;
  }

  std::string XmlRpcValue::toJsonStr(const miniros::JsonSettings& settings) const
  {
    miniros::JsonState state;
    std::stringstream ss;
    writeJson(ss, state, settings);
    return ss.str();
  }



  void XmlRpcValue::setBool(bool value)
  {
    if (_type != TypeBoolean) {
      invalidate();
    }
    _type = TypeBoolean;
    _value.asBool = value;
  }

  void XmlRpcValue::setInt(int value)
  {
    if (_type != TypeInt) {
      invalidate();
    }
    _type = TypeInt;
    _value.asInt = value;
  }

  void XmlRpcValue::setDouble(double value)
  {
    if (_type != TypeDouble) {
      invalidate();
    }
    _type = TypeDouble;
    _value.asDouble = value;
  }

  void XmlRpcValue::setString(std::string&& value)
  {
    if (_type != TypeString) {
      invalidate();
      _type = TypeString;
      _value.asString = new std::string(std::move(value));
    } else {
      *_value.asString = std::move(value);
    }
  }

  void XmlRpcValue::setDateTime(const struct tm& value)
  {
    if (_type != TypeDateTime) {
      invalidate();
      _type = TypeDateTime;
      _value.asTime = new tm(value);
    } else {
      *_value.asTime = value;
    }
  }

  void XmlRpcValue::setArray(const ValueArray& value)
  {
    if (_type != TypeArray) {
      invalidate();
      _type = TypeArray;
      _value.asArray = new ValueArray(value);
    } else {
      *_value.asArray = value;
    }
  }

  void XmlRpcValue::setArray(ValueArray&& value)
  {
    if (_type != TypeArray) {
      invalidate();
      _type = TypeArray;
      _value.asArray = new ValueArray(std::move(value));
    } else {
      *_value.asArray = std::move(value);
    }
  }

  void XmlRpcValue::setStruct(const ValueStruct& value)
  {
    if (_type != TypeStruct) {
      invalidate();
      _type = TypeStruct;
      _value.asStruct = new ValueStruct(value);
    } else {
      *_value.asStruct = value;
    }
  }

  void XmlRpcValue::setStruct(ValueStruct&& value)
  {
    if (_type != TypeStruct) {
      invalidate();
      _type = TypeStruct;
      _value.asStruct = new ValueStruct(std::move(value));
    } else {
      *_value.asStruct = std::move(value);
    }
  }

  void XmlRpcValue::setBinary(const BinaryData& value)
  {
    if (_type != TypeBase64) {
      invalidate();
      _type = TypeBase64;
      _value.asBinary = new BinaryData(value);
    } else {
      *_value.asBinary = value;
    }
  }

  void XmlRpcValue::setBinary(BinaryData&& value)
  {
    if (_type != TypeBase64) {
      invalidate();
      _type = TypeBase64;
      _value.asBinary = new BinaryData(std::move(value));
    } else {
      *_value.asBinary = std::move(value);
    }
  }
} // namespace XmlRpc


// ostream
std::ostream& operator<<(std::ostream& os, const XmlRpc::XmlRpcValue& v)
{
  // If you want to output in xml format:
  //return os << v.toXml();
  return v.write(os);
}

