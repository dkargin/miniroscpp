//
// Created by dkargin on 5/17/25.
//

#include <cstdlib>
#include <cstring>

#include "static_write_buf.h"

#include <charconv>
#include <cstdio>

namespace miniros {
StaticWriteBuf::StaticWriteBuf(char* const data, size_t size) : m_data(data), m_size(size)
{
  if (m_data && m_size) {
    m_ptr = data;
    m_ptr[m_size-1] = '\0';
  }
}

void StaticWriteBuf::reset() {
  m_ptr = m_data;
  if (m_data && m_size) {
    m_ptr[m_size-1] = '\0';
    *m_ptr = '\0';
  }
}

int StaticWriteBuf::left() const {
  return static_cast<int>(m_data + m_size - m_ptr - 1);
}

int StaticWriteBuf::puts(const char* str) {
  size_t len = str ? strlen(str) : 0;
  size_t available = static_cast<size_t>(left());

  if (len > available) {
    len = available;
  }
  if (len > 0) {
    strncpy(m_ptr, str, len);
    m_ptr += len;
    *m_ptr = '\0';
  }
  return static_cast<int>(len);
}

int StaticWriteBuf::putsq(const char* str) {
  size_t len = str ? strlen(str) : 0;
  size_t available = static_cast<size_t>(left());

  if (len + 2> available) {
    return 0;
  }

  putc('\"');
  if (len > 0) {
    memcpy(m_ptr, str, len);
    m_ptr += len;
    *m_ptr = '\0';
  }
  putc('\"');
  return static_cast<int>(len + 2);
}

int StaticWriteBuf::puti(int val)
{
  std::to_chars_result r = std::to_chars(m_ptr, m_ptr + left(), val);
  if (r.ec ==  std::errc::value_too_large) {
    *m_ptr = '\0';
    return 0;
  }
  const int len = static_cast<int>(r.ptr - m_ptr);
  m_ptr = r.ptr;
  *m_ptr = '\0';
  return len;
}

int StaticWriteBuf::putui64(uint64_t val)
{
  std::to_chars_result r = std::to_chars(m_ptr, m_ptr + left(), val);
  if (r.ec ==  std::errc::value_too_large) {
    *m_ptr = '\0';
    return 0;
  }
  const int len = static_cast<int>(r.ptr - m_ptr);
  m_ptr = r.ptr;
  *m_ptr = '\0';
  return len;
}


int StaticWriteBuf::putc(char val) {
  size_t available = static_cast<size_t>(left());
  if (available > 0) {
    *m_ptr = val;
    m_ptr++;
    *m_ptr = '\0';
    return 1;
  }
  return 0;
}

int StaticWriteBuf::putd(double val)
{
  if (val == 0.0) {
    return this->putc('0');
  }
  std::to_chars_result r = std::to_chars(m_ptr, m_ptr + left(), val, std::chars_format::fixed, 6);

  if (r.ec == std::errc::value_too_large) {
    *m_ptr = '\0';
    return 0;
  }

  const int len = static_cast<int>(r.ptr - m_ptr);
  m_ptr = r.ptr;
  *m_ptr = '\0';
  return len;
}

}