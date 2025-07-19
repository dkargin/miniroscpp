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
 *   * Neither the names of Willow Garage, Inc. nor the names of its
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

#ifndef MINIROSCPP_NAMES_H
#define MINIROSCPP_NAMES_H

#include <string_view>

#include "miniros/internal/forwards.h"
#include "miniros/common.h"
#include "miniros/errors.h"

namespace miniros
{

/**
 * \brief Contains functions which allow you to manipulate ROS names
 */
namespace names
{

/**
 * \brief Cleans a graph resource name: removes double slashes, trailing slash
 */
MINIROS_DECL std::string clean(const std::string& name);

/**
 * \brief Resolve a graph resource name into a fully qualified graph resource name
 *
 * See http://wiki.ros.org/Names for more details
 *
 * \param name Name to resolve
 * \param remap Whether or not to apply remappings to the name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
MINIROS_DECL std::string resolve(const std::string& name, bool remap = true);

/**
 * \brief Resolve a graph resource name into a fully qualified graph resource name
 *
 * See http://wiki.ros.org/Names for more details
 *
 * \param ns Namespace to use in resolution
 * \param name Name to resolve
 * \param remap Whether or not to apply remappings to the name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
MINIROS_DECL std::string resolve(const std::string& ns, const std::string& name, bool remap = true);
/**
 * \brief Append one name to another
 */
MINIROS_DECL std::string append(const std::string& left, const std::string& right);
/**
 * \brief Apply remappings to a name
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
MINIROS_DECL std::string remap(const std::string& name);
/**
 * \brief Validate a name against the name spec
 */
MINIROS_DECL bool validate(const std::string& name, std::string& error);

/** Check if name is a private namespace */
MINIROS_DECL bool isPrivate(const std::string& key);

MINIROS_DECL bool isGlobal(const std::string& key);

MINIROS_DECL const M_string& getRemappings();
MINIROS_DECL const M_string& getUnresolvedRemappings();

/**
 * \brief Get the parent namespace of a name
 * \param name The namespace of which to get the parent namespace.  
 * \throws InvalidNameException if the name passed is not a valid graph resource name
 */
MINIROS_DECL std::string parentNamespace(const std::string& name);

/// Fully annotated name
struct MINIROS_DECL Path {
  Path();
  Path(const Path& other);

  ~Path();

  void clear();

  /// Get number of elements in a path.
  /// It will take into account all path elements and a final name.
  size_t size() const;

  /// Return string element.
  std::string str(int i) const;

  /// Return string view element.
  std::string_view view(int i) const;

  /// Get final name.
  std::string name() const;

  /// Get a name from position [i, end]
  std::string right(int i) const;

  /// Get a path [0, i)
  std::string left(int i) const;

  /// Returns a string with a full original path.
  const std::string& fullPath() const;

  Error fromString(const std::string& path);

  bool isAbsolute() const;

  /// Check if this path starts with 'other' path.
  bool startsWith(const Path& other) const;

  friend MINIROS_DECL bool operator == (const Path& a, const Path& b);
  friend MINIROS_DECL bool operator != (const Path& a, const Path& b);
  friend MINIROS_DECL bool operator < (const Path& a, const Path& b);

protected:
  /// Name of parameter without a namespace.
  std::string_view m_lastName;

  /// This is an absolute path.
  bool m_absolute = false;
  bool m_private = false;

  /// Full path.
  std::string m_fullPath;

  /// A chain of namespaces.
  std::vector<std::string_view> m_ns;
};

/// Read list of topics from file.
/// It is used by `minibag record` command.
/// @param file - path to a file with a list of topics.
/// @param topics - output array with topics.
MINIROS_DECL bool readTopicList(const std::string& file, std::vector<std::string>& topics);

/// Read topic list from a string.
/// @param data - a buffer with topic list
/// @param topics - output array with topics.
MINIROS_DECL bool readTopicListStr(const std::string& data, std::vector<std::string>& topics);


} // namespace names

} // namespace miniros

#endif // MINIROSCPP_NAMES_H
