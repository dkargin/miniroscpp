//
// Created for SHA1 hash implementation.
//

#ifndef MINIROS_INTERNAL_SHA1_H
#define MINIROS_INTERNAL_SHA1_H

#include <cstdint>
#include <string>

namespace miniros {
namespace internal {

/// Simple SHA1 implementation for WebSocket handshake and other uses.
/// This is a minimal implementation to avoid external dependencies.
class SHA1 {
public:
  SHA1();
  
  /// Reset the hash state.
  void reset();

  /// Update hash with new data.
  /// @param data - pointer to data to hash
  /// @param len - length of data in bytes
  void update(const uint8_t* data, size_t len);

  /// Finalize hash computation and write result.
  /// @param hash - output buffer (must be at least 20 bytes)
  void finalize(uint8_t* hash);

  /// Compute SHA1 hash of a string and return as binary string (20 bytes).
  static std::string compute(const std::string& input);

private:
  uint32_t h[5];
  uint64_t length;
  uint8_t buffer[64];
  size_t bufferLength;

  uint32_t leftRotate(uint32_t value, int amount);
  void processBlock();
};

} // namespace internal
} // namespace miniros

#endif // MINIROS_INTERNAL_SHA1_H
