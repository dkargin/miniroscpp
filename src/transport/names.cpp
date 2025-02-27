/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cassert>
#include <cstring>
#include <sstream>

#include "miniros/names.h"
#include "miniros/this_node.h"

namespace miniros
{

namespace names
{

M_string g_remappings;
M_string g_unresolved_remappings;

const M_string& getRemappings()
{
  return g_remappings;
}

const M_string& getUnresolvedRemappings()
{
  return g_unresolved_remappings;
}

bool isValidCharInName(char c)
{
  if (isalnum(c) || c == '/' || c == '_')
  {
    return true;
  }

  return false;
}

bool isPrivate(const std::string& key)
{
  return !key.empty() && key[0] == '~';
}

bool isGlobal(const std::string& key)
{
  return !key.empty() && key[0] == '/';
}

bool validate(const std::string& name, std::string& error)
{
  if (name.empty())
  {
    return true;
  }

  // First element is special, can be only ~ / or alpha
  char c = name[0];
  if (!isalpha(c) && c != '/' && c != '~')
  {
    std::stringstream ss;
    ss << "Character [" << c << "] is not valid as the first character in Graph Resource Name [" << name << "].  Valid characters are a-z, A-Z, / and in some cases ~.";
    error = ss.str();
    return false;
  }

  for (size_t i = 1; i < name.size(); ++i)
  {
    c = name[i];
    if (!isValidCharInName(c))
    {
      std::stringstream ss;
      ss << "Character [" << c << "] at element [" << i << "] is not valid in Graph Resource Name [" << name <<"].  Valid characters are a-z, A-Z, 0-9, / and _.";
      error = ss.str();

      return false;
    }
  }

  return true;
}

std::string clean(const std::string& name)
{
  std::string clean = name;

  size_t pos = clean.find("//");
  while (pos != std::string::npos)
  {
    clean.erase(pos, 1);
    pos = clean.find("//", pos);
  }

  if (!name.empty() && *clean.rbegin() == '/')
  {
    clean.erase(clean.size() - 1, 1);
  }

  return clean;
}

std::string append(const std::string& left, const std::string& right)
{
  return clean(left + "/" + right);
}

std::string remap(const std::string& name)
{
  std::string resolved = resolve(name, false);

  auto it = g_remappings.find(resolved);
  if (it != g_remappings.end())
  {
    return it->second;
  }

  return name;
}

std::string resolve(const std::string& name, bool _remap)
{
  std::string s = resolve(this_node::getNamespace(), name, _remap);
  return s;
}

std::string resolve(const std::string& ns, const std::string& name, bool _remap)
{
  std::string error;
  if (!validate(name, error))
  {
    throw InvalidNameException(error);
  }

  if (name.empty())
  {
    if (ns.empty())
    {
      return "/";
    }

    if (ns[0] == '/')
    {
      return ns;
    }

    return append("/", ns);
  }

  std::string copy = name;

  if (copy[0] == '~')
  {
    copy = append(this_node::getName(), copy.substr(1));
  }

  if (copy[0] != '/')
  {
    copy = append("/", append(ns, copy));
  }

  copy = clean(copy);

  if (_remap)
  {
    copy = remap(copy);
  }

  return copy;
}

void init(const M_string& remappings)
{
  for (const auto& [left, right] : remappings)
  {
    if (!left.empty() && left[0] != '_' && left != this_node::getName())
    {
      std::string resolved_left = resolve(left, false);
      std::string resolved_right = resolve(right, false);
      g_remappings[resolved_left] = resolved_right;
      g_unresolved_remappings[left] = right;
    }
  }
}

std::string parentNamespace(const std::string& name)
{
  std::string error;
  if (!validate(name, error))
  {
  	throw InvalidNameException(error);
  }

  if (!name.compare(""))  return "";
  if (!name.compare("/")) return "/"; 

  std::string stripped_name;

  // rstrip trailing slash
  if (name.find_last_of('/') == name.size()-1)
    stripped_name = name.substr(0, name.size() -2);
  else
    stripped_name = name;

  //pull everything up to the last /
  size_t last_pos = stripped_name.find_last_of('/');
  if (last_pos == std::string::npos)
  {
    return "";
  }
  else if (last_pos == 0)
    return "/";
  return stripped_name.substr(0, last_pos);
}

Path::Path() = default;

Path::Path(const Path& other)
{
  m_fullName = other.m_fullName;
  m_ns.reserve(other.m_ns.size());
  for (const auto& element: other.m_ns) {
    int index = element.data() - other.m_fullName.data();
    assert(index >= 0);
    if (index < 0)
      continue;
    m_ns.emplace_back(&m_fullName[index], element.size());
  }

  if (!other.m_lastName.empty()) {
    int index = m_lastName.data() - other.m_fullName.data();
    m_lastName = std::string_view(m_fullName.data() + index, other.m_lastName.size());
  }
}

Path::~Path() = default;

void Path::clear()
{
  m_fullName.clear();
  m_ns.clear();
  m_lastName = {};
}

std::string Path::name() const
{
  return std::string(m_lastName.data(), m_lastName.size());
}

Error Path::fromString(const std::string& path)
{
  if (path.empty())
    return Error::Ok;

  clear();

  m_fullName = path;
  // Index of first symbol of current token.
  size_t tokenStart = 0;

  if (path[0] == '/')
    m_absolute = true;
  else if (path[0] == '~')
    m_private = true;

  size_t i = 0;
  for (;i < path.size(); i++) {
    auto c = path[i];
    if (!isValidCharInName(c))
      return Error::InvalidValue;
    if (c == '/') {
      if (i > tokenStart) {
        m_ns.push_back(std::string_view(&m_fullName[tokenStart], i - tokenStart));
      }
      tokenStart = i + 1;
    }
  }

  if (i > tokenStart) {
    m_lastName = std::string_view(&m_fullName[tokenStart], i - tokenStart);
  }

  return Error::Ok;
}

size_t Path::size() const
{
  size_t s = m_ns.size();
  if (!m_lastName.empty())
    s++;
  return s;
}

/// Return string element.
std::string Path::str(int i) const
{
  auto v = this->view(i);
  return std::string(v.data(), v.size());
}

std::string Path::right(int i) const
{
  if (i > size())
    return "";
  if (i == 0)
    return "";

  int pos = size() - i;
  auto v = view(pos);

  const auto* end = &m_fullName[0] + m_fullName.size();
  size_t leftLen = end - v.data();
  std::string s = std::string(v.data(), leftLen);
  if (pos == 0 && isAbsolute())
     return std::string("/") + s;
  return s;
}

std::string Path::left(int i) const
{
  if (i >= size())
    return m_fullName;

  auto v = view(i);

  const auto* end = v.data();
  size_t len = end - m_fullName.data();
  return std::string(m_fullName.data(), len);
}

bool Path::isAbsolute() const
{
  return m_absolute;
}


/// Return string view element.
std::string_view Path::view(int i) const
{
  if (i == m_ns.size())
    return m_lastName;
  if (i < m_ns.size())
    return m_ns[i];
  return {};
}

bool operator == (const Path& a, const Path& b)
{
  if (a.m_ns.size() != b.m_ns.size())
    return false;
  for (size_t i = 0; i < a.m_ns.size(); i++) {
    if (a.m_ns[i] != b.m_ns[i])
      return false;
  }
  return a.m_lastName == b.m_lastName;
}

bool operator != (const Path& a, const Path& b)
{
  if (a.m_ns.size() != b.m_ns.size())
    return true;

  for (size_t i = 0; i < a.m_ns.size(); i++) {
    if (a.m_ns[i] != b.m_ns[i])
      return true;
  }
  return a.m_lastName != b.m_lastName;
}

bool operator < (const Path& a, const Path& b)
{
  return a.m_fullName < b.m_fullName;
}

} // namespace names

} // namespace miniros
