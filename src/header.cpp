/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "miniros/header.h"

#include "miniros/console.h"

#include <sstream>
#include <cstring>
#include <cerrno>
#include <cassert>
#include <string_view>

#define SMINIROS_SERIALIZE_PRIMITIVE(ptr, data) { memcpy(ptr, &data, sizeof(data)); ptr += sizeof(data); }
#define SMINIROS_SERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(ptr, data, data_size); ptr += data_size; } }
#define SMINIROS_DESERIALIZE_PRIMITIVE(ptr, data) { memcpy(&data, ptr, sizeof(data)); ptr += sizeof(data); }
#define SMINIROS_DESERIALIZE_BUFFER(ptr, data, data_size) { if (data_size > 0) { memcpy(data, ptr, data_size); ptr += data_size; } }

// Remove this when no longer supporting platforms with libconsole-bridge-dev < 0.3.0,
// in particular Debian Jessie: https://packages.debian.org/jessie/libconsole-bridge-dev
#ifndef MINIROS_CONSOLE_BRIDGE_logError
# define MINIROS_CONSOLE_BRIDGE_logError(fmt, ...)  \
  miniros_console_bridge::log(__FILE__, __LINE__, miniros_console_bridge::CONSOLE_BRIDGE_LOG_ERROR, fmt, ##__VA_ARGS__)
#endif

using namespace std;

namespace miniros
{

Header::Header()
: read_map_(new M_string())
{

}

Header::~Header()
{

}

bool Header::parse(const shared_array_uint8_t& buffer, uint32_t size, std::string& error_msg)
{
  return parse(buffer.get(), size, error_msg);
}

bool Header::parse(uint8_t* buffer, uint32_t size, std::string& error_msg)
{
  std::string key_;
  uint8_t* buf = buffer;
  while (buf < buffer + size)
  {
    uint32_t len;
    SMINIROS_DESERIALIZE_PRIMITIVE(buf, len);

    if (len > 1000000)
    {
      error_msg = "Received an invalid TCPROS header.  Each element must be prepended by a 4-byte length.";
      MINIROS_CONSOLE_BRIDGE_logError("%s", error_msg.c_str());

      return false;
    }
    // TODO: It can be done using plain old C.
    std::string_view line((char*)buf, len);

    buf += len;

    //printf(":%s:\n", line.c_str());
    size_t eqpos = line.find_first_of("=");
    if (eqpos == string::npos)
    {
      error_msg = "Received an invalid TCPROS header.  Each line must have an equals sign.";
      MINIROS_CONSOLE_BRIDGE_logError("%s", error_msg.c_str());

      return false;
    }
    std::string_view key_ref = line.substr(0, eqpos);
    std::string_view value_ref = line.substr(eqpos+1);

    key_.assign(key_ref.data(), key_ref.length());

    (*read_map_)[key_].assign(value_ref.data(), value_ref.length());
  }

  return true;
}

bool Header::getValue(const std::string& key, std::string& value) const
{
  M_string::const_iterator it = read_map_->find(key);
  if (it == read_map_->end())
  {
    return false;
  }

  value = it->second;

  return true;
}

void Header::write(const M_string& key_vals, shared_array_uint8_t& buffer, uint32_t& size)
{
  // Calculate the necessary size
  size = 0;
  {
    for (const auto& r: key_vals)
    {
      const std::string& key = r.first;
      const std::string& value = r.second;

      size += (uint32_t)key.length();
      size += (uint32_t)value.length();
      size += 1; // = sign
      size += 4; // 4-byte length
    }
  }

  if (size == 0)
  {
    return;
  }

  buffer.reset(new uint8_t[size]);
  char* ptr = (char*)buffer.get();

  // Write the data
  {
    for (const auto& r: key_vals)
    {
      const std::string& key = r.first;
      const std::string& value = r.second;

      uint32_t len = static_cast<uint32_t>(key.length() + value.length() + 1);
      SMINIROS_SERIALIZE_PRIMITIVE(ptr, len);
      SMINIROS_SERIALIZE_BUFFER(ptr, key.data(), key.length());
      constexpr char equals = '=';
      SMINIROS_SERIALIZE_PRIMITIVE(ptr, equals);
      SMINIROS_SERIALIZE_BUFFER(ptr, value.data(), value.length());
    }
  }

  assert(ptr == (char*)buffer.get() + size);
}

}
