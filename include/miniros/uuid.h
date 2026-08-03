#ifndef MINIROS_UUID_H
#define MINIROS_UUID_H

#include <array>
#include <cstdint>
#include <cstring>
#include <string>

#include "miniros/macros.h"

namespace miniros {

/// UUID has the following hex structure:
/// 8-4-4-4-12
///  0 1 2 3  4 5  6 7  8 9 101112131415
/// xxxxxxxx-xxxx-Mxxx-Nxxx-xxxxxxxxxxxx
/// M and N encode type of UUID. They correspond to character 6 and 8.
struct MINIROS_DECL UUID {
  enum { Dim = 16 };
  uint8_t bytes[Dim] = {};

  void generate();

  /// Reset all values to zero.
  void reset();

  /// Check if UUID is valid.
  bool valid() const;

  /// Canonical string form: 8-4-4-4-12 lowercase hex.
  std::string toString() const;

  /// Parse canonical UUID string (dashes optional). Returns false on failure.
  bool fromString(const std::string& str);

  static UUID fromBytes(const std::array<uint8_t, Dim>& in)
  {
    UUID u;
    std::memcpy(u.bytes, in.data(), Dim);
    return u;
  }

  std::array<uint8_t, Dim> toBytes() const
  {
    std::array<uint8_t, Dim> out{};
    std::memcpy(out.data(), bytes, Dim);
    return out;
  }
};

MINIROS_DECL bool operator==(const UUID& a, const UUID& b);

} // namespace miniros

#endif
