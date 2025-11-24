
#ifndef _XMLRPCVALUE_H_
#define _XMLRPCVALUE_H_
//
// XmlRpc++ Copyright (c) 2002-2003 by Chris Morley
//
#if defined(_MSC_VER)
# pragma warning(disable:4786)    // identifier was truncated in debug info
#endif

#include "XmlRpcDecl.h"

#include "miniros/internal/json_tools.h"

#include <cstring>

#ifndef MAKEDEPEND
# include <map>
# include <string>
# include <vector>
# include <time.h>
#endif

namespace XmlRpc {

  //! RPC method arguments and results are represented by Values
  //   should probably refcount them...
  class XMLRPCPP_DECL XmlRpcValue {
  public:

    enum Type {
      TypeInvalid,
      TypeBoolean,
      TypeInt,
      TypeDouble,
      TypeString,
      TypeDateTime,
      TypeBase64,
      TypeArray,
      TypeStruct
    };

    // Non-primitive types
    typedef std::vector<char> BinaryData;
    typedef std::vector<XmlRpcValue> ValueArray;
    typedef std::map<std::string, XmlRpcValue> ValueStruct;
    typedef ValueStruct::iterator iterator;
    typedef ValueStruct::const_iterator const_iterator;

    //! Constructors
    XmlRpcValue() : _type(TypeInvalid) { _value.asBinary = 0; }
    XmlRpcValue(bool value) : _type(TypeBoolean) { _value.asBool = value; }
    XmlRpcValue(int value)  : _type(TypeInt) { _value.asInt = value; }
    XmlRpcValue(double value)  : _type(TypeDouble) { _value.asDouble = value; }

    XmlRpcValue(std::string const& value) : _type(TypeString) 
    { _value.asString = new std::string(value); }

    XmlRpcValue(const char* value)  : _type(TypeString)
    { _value.asString = new std::string(value); }

    XmlRpcValue(struct tm* value)  : _type(TypeDateTime) 
    { _value.asTime = new struct tm(*value); }


    XmlRpcValue(void* value, int nBytes)  : _type(TypeBase64)
    {
      _value.asBinary = new BinaryData((char*)value, ((char*)value)+nBytes);
    }

    //! Copy
    XmlRpcValue(XmlRpcValue const& rhs) : _type(TypeInvalid) { *this = rhs; }

    //! Transfer
    XmlRpcValue(XmlRpcValue && rhs) noexcept
    {
      _type = rhs._type;
      _value = rhs._value;
      // No need to zero rhs._value, TypeInvalid ensures that destructor of rhs does nothing.
      rhs._type = TypeInvalid;
    }

    //! Destructor (make virtual if you want to subclass)
    /*virtual*/ ~XmlRpcValue() { invalidate(); }

    /// Creates array of specified size.
    NODISCARD static XmlRpcValue Array(size_t size);

    /// Creates a dictionary object.
    NODISCARD static XmlRpcValue Dict();

    //! Erase the current value
    void clear() { invalidate(); }

    // Assign new value/type.
    /// Assign boolean value.
    void setBool(bool value);
    /// Assign integer value.
    void setInt(int value);
    void setDouble(double value);
    void setString(std::string&& value);
    void setDateTime(const struct tm& value);
    void setArray(const ValueArray& value);
    void setArray(ValueArray&& value);
    void setStruct(const ValueStruct& value);
    void setStruct(ValueStruct&& value);
    void setBinary(const BinaryData& data);
    void setBinary(BinaryData&& data);

    // Operators
    XmlRpcValue& operator=(XmlRpcValue const& rhs);
    XmlRpcValue& operator=(bool const& rhs) { return operator=(XmlRpcValue(rhs)); }
    XmlRpcValue& operator=(int const& rhs) { return operator=(XmlRpcValue(rhs)); }
    XmlRpcValue& operator=(double const& rhs) { return operator=(XmlRpcValue(rhs)); }
    XmlRpcValue& operator=(const char* rhs) { return operator=(XmlRpcValue(std::string(rhs))); }

    bool operator==(XmlRpcValue const& other) const;
    bool operator!=(XmlRpcValue const& other) const;

    operator bool&()          { assertTypeOrInvalid(TypeBoolean); return _value.asBool; }
    operator int&()           { assertTypeOrInvalid(TypeInt); return _value.asInt; }
    operator double&()        { assertTypeOrInvalid(TypeDouble); return _value.asDouble; }
    operator std::string&()   { assertTypeOrInvalid(TypeString); return *_value.asString; }
    operator BinaryData&()    { assertTypeOrInvalid(TypeBase64); return *_value.asBinary; }
    operator struct tm&()     { assertTypeOrInvalid(TypeDateTime); return *_value.asTime; }

    operator const bool&() const         { assertType(TypeBoolean); return _value.asBool; }
    operator const int&() const          { assertType(TypeInt); return _value.asInt; }
    operator const double&() const       { assertType(TypeDouble); return _value.asDouble; }
    operator const std::string&() const  { assertType(TypeString); return *_value.asString; }
    operator const BinaryData&() const   { assertType(TypeBase64); return *_value.asBinary; }
    operator const struct tm&() const    { assertType(TypeDateTime); return *_value.asTime; }

    template <class T> T as() const
    {
      return static_cast<T>(*this);
    }

    XmlRpcValue const& operator[](int i) const { assertArrayConst(i+1); return _value.asArray->at(i); }
    XmlRpcValue& operator[](int i)             { assertArray(i+1); return _value.asArray->at(i); }
    XmlRpcValue& operator[](std::string const& k) const { assertStructConst(); return (*_value.asStruct)[k]; }
    XmlRpcValue& operator[](std::string const& k) { assertStruct(); return (*_value.asStruct)[k]; }
    XmlRpcValue& operator[](const char* k) const { assertStructConst(); std::string s(k); return (*_value.asStruct)[s]; }
    XmlRpcValue& operator[](const char* k) { assertStruct(); std::string s(k); return (*_value.asStruct)[s]; }

    iterator begin() {assertStruct(); return (*_value.asStruct).begin(); }
    iterator end() {assertStruct(); return (*_value.asStruct).end(); }

    const_iterator begin() const {assertStructConst(); return (*_value.asStruct).begin(); }
    const_iterator end() const {assertStructConst(); return (*_value.asStruct).end(); }

    // Accessors
    //! Return true if the value has been set to something.
    bool valid() const { return _type != TypeInvalid; }

    //! Return the type of the value stored. \see Type.
    Type const &getType() const { return _type; }

    /// Check if value is a primitive type.
    bool isPrimitive() const;

    //! Return the size for string, base64, array, and struct values.
    int size() const;

    //! Specify the size for array values. Array values will grow beyond this size if needed.
    void setSize(size_t size)    { assertArray(size); }

    //! Check for the existence of a struct member by name.
    bool hasMember(const std::string& name) const;

    //! Erase member of a struct.
    bool eraseMember(const std::string& key);

    //! Encode the Value in xml
    std::string toXml() const;

    //! Write the value (no xml encoding)
    std::ostream& write(std::ostream& os) const;

    std::ostream& writeJson(std::ostream& os, miniros::JsonState& state, const miniros::JsonSettings& settings) const;

  protected:
    // Clean up
    void invalidate();

    // Type checking
    void assertTypeOrInvalid(Type t);
    void assertType(Type t) const;
    void assertArrayConst(size_t size) const;
    void assertArray(size_t size);
    void assertStructConst() const;
    void assertStruct();

    // XML encoding
    std::string boolToXml() const;
    std::string intToXml() const;
    std::string doubleToXml() const;
    std::string stringToXml() const;
    std::string timeToXml() const;
    std::string binaryToXml() const;
    std::string arrayToXml() const;
    std::string structToXml() const;

    // Type tag and values
    Type _type;

    // At some point I will split off Arrays and Structs into
    // separate ref-counted objects for more efficient copying.
    union {
      bool          asBool;
      int           asInt;
      double        asDouble;
      struct tm*    asTime;
      std::string*  asString;
      BinaryData*   asBinary;
      ValueArray*   asArray;
      ValueStruct*  asStruct;
    } _value;
    
  };
} // namespace XmlRpc


XMLRPCPP_DECL std::ostream& operator<<(std::ostream& os, const XmlRpc::XmlRpcValue& v);


#endif // _XMLRPCVALUE_H_
