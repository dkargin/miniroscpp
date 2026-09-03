/*
 * Structured JSON encoding for ROS1 / MiniROS messages.
 *
 * Default layout matches ROS1 rosbridge / roslibjs: Time/Duration use secs/nsecs,
 * uint8[] and char[] are base64. JsonSettings::binary_as_base64=false writes those
 * fields as JSON number arrays for ROS-less consumers.
 */

#pragma once

#include "miniros/serialization.h"
#include "miniros/internal/json_tools.h"
#include "miniros/rostime.h"
#include "miniros/b64/encode.h"

#include <cmath>
#include <cstdio>
#include <ostream>
#include <string>
#include <type_traits>
#include <vector>
#include <array>
#include <charconv>
#include <limits>
#include <cstdint>
#include <iterator>

namespace miniros
{
namespace serialization
{

inline constexpr JsonSettings jsonCompact{0};
inline constexpr JsonSettings jsonStrict{0, false};

namespace json_detail
{

template<typename T>
using remove_cvref_t = std::remove_cv_t<std::remove_reference_t<T>>;

template<typename T>
struct is_std_vector : std::false_type {};
template<typename T, class A>
struct is_std_vector<std::vector<T, A>> : std::true_type {};

template<typename T>
struct is_std_array : std::false_type {};
template<typename T, size_t N>
struct is_std_array<std::array<T, N>> : std::true_type {};

template<typename T>
struct is_std_string : std::false_type {};
template<class A>
struct is_std_string<std::basic_string<char, std::char_traits<char>, A>> : std::true_type {};

template<typename T>
inline constexpr bool is_json_container_v =
    is_std_vector<T>::value || is_std_array<T>::value || is_std_string<T>::value;

} // namespace json_detail

/**
 * \brief Storage object for KVSerializer that writes compact or pretty JSON.
 *
 * Nested messages become objects. Time and Duration are written as
 * {"secs":...,"nsecs":...} (ROS1 rosbridge / rospy names).
 */
class JsonWriter
{
public:
  explicit JsonWriter(std::string& out, const JsonSettings& settings = jsonCompact)
    : out_(out)
    , settings_(settings)
  {
    const size_t min_cap = out_.size() + 64;
    if (out_.capacity() < min_cap)
      out_.reserve(min_cap);
  }

  template<typename T>
  void element(const char* name, const T& value)
  {
    writeSeparator();
    writeKey(name);
    writeValue(value);
  }

  template<typename T>
  void boolean(const char* name, const T& value)
  {
    writeSeparator();
    writeKey(name);
    out_.append(static_cast<bool>(value) ? "true" : "false");
  }

  template<typename Range>
  void boolArray(const char* name, const Range& value)
  {
    writeSeparator();
    writeKey(name);
    beginArray();
    for (const auto& item : value)
    {
      writeSeparator();
      out_.append(static_cast<bool>(item) ? "true" : "false");
    }
    endArray();
  }

  template<typename Range>
  void binary(const char* name, const Range& value)
  {
    writeSeparator();
    writeKey(name);
    writeBinary(value);
  }

  /// Write a complete JSON value (object, array, or primitive).
  template<typename T>
  void write(const T& value)
  {
    writeValue(value);
  }

private:
  std::string& out_;
  JsonSettings settings_;
  /// Bit i is set while nesting frame i has not emitted its first item.
  uint64_t first_mask_ = 0;
  int depth_ = 0;
  int indent_ = 0;

  bool pretty() const { return settings_.tabs > 0; }

  void pushScope()
  {
    if (depth_ < 64)
      first_mask_ |= (uint64_t{1} << depth_);
    ++depth_;
  }

  void popScope()
  {
    if (depth_ > 0)
      --depth_;
  }

  bool scopeEmpty() const
  {
    if (depth_ <= 0 || depth_ > 64)
      return true;
    return (first_mask_ & (uint64_t{1} << (depth_ - 1))) != 0;
  }

  void appendIndent()
  {
    if (indent_ > 0)
      out_.append(static_cast<size_t>(indent_), ' ');
  }

  void writeSeparator()
  {
    if (depth_ <= 0)
      return;
    if (!scopeEmpty())
      out_.push_back(',');
    if (depth_ <= 64)
      first_mask_ &= ~(uint64_t{1} << (depth_ - 1));
    if (pretty())
    {
      out_.push_back('\n');
      appendIndent();
    }
  }

  void writeKey(const char* name)
  {
    out_.push_back('"');
    out_.append(name);
    out_.append(pretty() ? "\": " : "\":");
  }

  void beginObject()
  {
    out_.push_back('{');
    pushScope();
    indent_ += settings_.tabs;
  }

  void endObject()
  {
    const bool empty = scopeEmpty();
    popScope();
    indent_ -= settings_.tabs;
    if (!empty && pretty())
    {
      out_.push_back('\n');
      appendIndent();
    }
    out_.push_back('}');
  }

  void beginArray()
  {
    out_.push_back('[');
    pushScope();
    indent_ += settings_.tabs;
  }

  void endArray()
  {
    const bool empty = scopeEmpty();
    popScope();
    indent_ -= settings_.tabs;
    if (!empty && pretty())
    {
      out_.push_back('\n');
      appendIndent();
    }
    out_.push_back(']');
  }

  template<typename T>
  void appendInteger(T v)
  {
    char buf[32];
    const std::to_chars_result r = std::to_chars(buf, buf + sizeof(buf), v);
    if (r.ec != std::errc())
    {
      out_.push_back('0');
      return;
    }
    out_.append(buf, static_cast<size_t>(r.ptr - buf));
  }

  void appendFloat(double v, int precision)
  {
    if (!std::isfinite(v))
    {
      out_.append("null");
      return;
    }
    char buf[64];
    const int n = std::snprintf(buf, sizeof(buf), "%.*g", precision, v);
    if (n <= 0)
    {
      out_.append("null");
      return;
    }
    out_.append(buf, static_cast<size_t>(n));
  }

  void appendEscaped(const char* s, size_t n)
  {
    out_.reserve(out_.size() + n + 2);
    out_.push_back('"');
    for (size_t i = 0; i < n; ++i)
    {
      const unsigned char c = static_cast<unsigned char>(s[i]);
      switch (c)
      {
        case '"':  out_.append("\\\""); break;
        case '\\': out_.append("\\\\"); break;
        case '\b': out_.append("\\b"); break;
        case '\f': out_.append("\\f"); break;
        case '\n': out_.append("\\n"); break;
        case '\r': out_.append("\\r"); break;
        case '\t': out_.append("\\t"); break;
        default:
          if (c < 0x20)
          {
            char buf[8];
            std::snprintf(buf, sizeof(buf), "\\u%04x", c);
            out_.append(buf, 6);
          }
          else
          {
            out_.push_back(static_cast<char>(c));
          }
          break;
      }
    }
    out_.push_back('"');
  }

  void appendBase64(const uint8_t* data, size_t n)
  {
    const size_t encoded = ((n + 2) / 3) * 4;
    const size_t start = out_.size();
    out_.resize(start + encoded + 2);
    char* p = &out_[start];
    *p = '"';
    base64::base64_encodestate state;
    base64::base64_init_encodestate(&state);
    int written = 0;
    if (n > 0)
    {
      written = base64::base64_encode_block(
          reinterpret_cast<const char*>(data), static_cast<int>(n), p + 1, &state, base64::BASE64_NO_WRAP);
    }
    written += base64::base64_encode_blockend(p + 1 + written, &state);
    p[1 + written] = '"';
    out_.resize(start + static_cast<size_t>(written) + 2);
  }

  template<typename Range>
  void writeBinary(const Range& value)
  {
    const size_t n = static_cast<size_t>(std::size(value));
    if (settings_.binary_as_base64)
    {
      const uint8_t* data = n ? reinterpret_cast<const uint8_t*>(std::data(value)) : nullptr;
      appendBase64(data, n);
      return;
    }
    beginArray();
    for (const auto& item : value)
    {
      writeSeparator();
      appendInteger(static_cast<unsigned>(item));
    }
    endArray();
  }

  template<typename Sec, typename Nsec>
  void writeTimeLike(Sec sec, Nsec nsec)
  {
    beginObject();
    element("secs", sec);
    element("nsecs", nsec);
    endObject();
  }

  void writeValue(bool v)
  {
    out_.append(v ? "true" : "false");
  }

  template<typename T, typename std::enable_if_t<
      std::is_integral<T>::value && !std::is_same<T, bool>::value, int> = 0>
  void writeValue(T v)
  {
    appendInteger(v);
  }

  void writeValue(float v)
  {
    appendFloat(static_cast<double>(v), std::numeric_limits<float>::max_digits10);
  }

  void writeValue(double v)
  {
    appendFloat(v, std::numeric_limits<double>::max_digits10);
  }

  void writeValue(const char* v)
  {
    if (!v)
    {
      out_.append("null");
      return;
    }
    appendEscaped(v, std::char_traits<char>::length(v));
  }

  template<class Alloc>
  void writeValue(const std::basic_string<char, std::char_traits<char>, Alloc>& v)
  {
    appendEscaped(v.data(), v.size());
  }

  void writeValue(const miniros::Time& v)
  {
    writeTimeLike(v.sec, v.nsec);
  }

  void writeValue(const miniros::Duration& v)
  {
    writeTimeLike(v.sec, v.nsec);
  }

  template<class Alloc>
  void writeValue(const std::vector<uint8_t, Alloc>& v)
  {
    writeBinary(v);
  }

  template<size_t N>
  void writeValue(const std::array<uint8_t, N>& v)
  {
    writeBinary(v);
  }

  template<typename T, class Alloc>
  void writeValue(const std::vector<T, Alloc>& v)
  {
    beginArray();
    for (const auto& item : v)
    {
      writeSeparator();
      writeValue(item);
    }
    endArray();
  }

  template<typename T, size_t N>
  void writeValue(const std::array<T, N>& v)
  {
    beginArray();
    for (const auto& item : v)
    {
      writeSeparator();
      writeValue(item);
    }
    endArray();
  }

  template<typename T, typename std::enable_if_t<
      !std::is_arithmetic<json_detail::remove_cvref_t<T>>::value &&
      !json_detail::is_json_container_v<json_detail::remove_cvref_t<T>> &&
      !std::is_same<json_detail::remove_cvref_t<T>, miniros::Time>::value &&
      !std::is_same<json_detail::remove_cvref_t<T>, miniros::Duration>::value, int> = 0>
  void writeValue(const T& v)
  {
    beginObject();
    KVSerializer<json_detail::remove_cvref_t<T>>::allInOne(*this, v);
    endObject();
  }
};

template<typename M>
void serializeJson(std::string& out, const M& message, const JsonSettings& settings = jsonCompact)
{
  JsonWriter writer(out, settings);
  writer.write(message);
}

template<typename M>
std::string serializeJson(const M& message, const JsonSettings& settings = jsonCompact)
{
  std::string out;
  serializeJson(out, message, settings);
  return out;
}

template<typename M>
void serializeJson(std::ostream& os, const M& message, const JsonSettings& settings = jsonCompact)
{
  os << serializeJson(message, settings);
}

} // namespace serialization
} // namespace miniros
