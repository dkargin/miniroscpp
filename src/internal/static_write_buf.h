//
// Created by dkargin on 5/17/25.
//

#ifndef NAVIGATO_STATIC_WRITE_BUF_H
#define NAVIGATO_STATIC_WRITE_BUF_H

#include <cstdint>

namespace miniros {
class StaticWriteBuf {
public:
  StaticWriteBuf(char* const data, size_t size);

  /// Return number of bytes left in a buffer.
  int left() const;

  /// Put string to a buffer.
  /// @returns number of written characters.
  int puts(const char* str);

  /// Put quoted string to a buffer.
  /// @returns number of written characters
  int putsq(const char* str);

  /// Put a single character to a buffer.
  /// @returns number of written characters.
  int putc(char val);

  /// Put an integer to a buffer.
  /// @returns number of written characters.
  int puti(int val);

  int putui64(uint64_t val);


  /// Put an double to a buffer.
  /// @returns number of written characters.
  int putd(double val);

  /// Reset buffer to an original state.
  void reset();

  /// Get access to internal data.
  const char* data() const
  {
    return m_data;
  }

  /// Maximum size of a buffer
  size_t capacity() const
  {
    return m_size;
  }

  /// Return current size of a string.
  size_t size() const
  {
    return m_data ? m_ptr - m_data : 0;
  }

protected:
  char* const m_data = nullptr;
  /// Current size of a buffer.
  const size_t m_size = 0;

  /// Current pointer.
  char* m_ptr = nullptr;
};
}

#endif //NAVIGATO_STATIC_WRITE_BUF_H
