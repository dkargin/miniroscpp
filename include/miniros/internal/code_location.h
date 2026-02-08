//
// Created by dkargin on 2/4/26.
//

#ifndef MINIROS_CODE_LOCATION_H
#define MINIROS_CODE_LOCATION_H

#include <cstring>
#include <string>

namespace miniros {
namespace internal {

struct CodeLocation {
  CodeLocation() = default;
  CodeLocation(const char *file, const char *method, int line) : m_file(file), m_method(method), m_line(line) {}
  CodeLocation(const CodeLocation& c) : m_file(c.m_file), m_line(c.m_line) {}

  CodeLocation& operator = (const CodeLocation& other)
  {
    m_file = other.m_file;
    m_line = other.m_line;
    m_method = other.m_method;
    return *this;
  }

  static CodeLocation make(const char* file, int line)
  {
    return CodeLocation(file, "", line);
  }

  std::string str() const
  {
    std::string result;
    result.reserve(255);
    if (m_file) {
      result += std::string(m_file) + ":" + std::to_string(m_line);
    }
    if (m_method) {
      result += std::string(m_method);
    }
    return result;
  }

  bool valid() const
  {
    return m_file != nullptr;
  }

  friend bool operator == (const CodeLocation& c1, const CodeLocation& c2)
  {
    return c1.m_line == c2.m_line && strcmp(c1.m_file, c2.m_file) == 0 && strcmp(c1.m_method, c2.m_method) == 0;
  }

  friend bool operator != (const CodeLocation& c1, const CodeLocation& c2)
  {
    return c1.m_line != c2.m_line || strcmp(c1.m_file, c2.m_file) != 0 || strcmp(c1.m_method, c2.m_method) != 0;
  }

  friend bool operator < (const CodeLocation& a, const CodeLocation& b)
  {
    int f = strcmp(a.m_file, b.m_file);
    if (f == 0) {
      int m = strcmp(a.m_method, b.m_method);
      if (m == 0) {
        return a.m_line < b.m_line;
      }
      return m < 0;
    }
    return f < 0;
  }

protected:
  const char* m_file = nullptr;
  const char* m_method = nullptr;
  int m_line = 0;
};

}

#define THIS_LOCATION internal::CodeLocation::make(__FILE__, __LINE__)

}
#endif // MINIROS_CODE_LOCATION_H
