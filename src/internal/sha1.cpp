//
// Created for SHA1 hash implementation.
//

#include <cstring>

#include "../../include/miniros/internal/sha1.h"

namespace miniros {
namespace internal {

SHA1::SHA1() {
  reset();
}

void SHA1::reset() {
  h[0] = 0x67452301;
  h[1] = 0xEFCDAB89;
  h[2] = 0x98BADCFE;
  h[3] = 0x10325476;
  h[4] = 0xC3D2E1F0;
  length = 0;
  bufferLength = 0;
}

void SHA1::update(const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    buffer[bufferLength++] = data[i];
    length += 8;
    if (bufferLength == 64) {
      processBlock();
      bufferLength = 0;
    }
  }
}

void SHA1::finalize(uint8_t* hash) {
  // Pad the message
  buffer[bufferLength++] = 0x80;
  if (bufferLength > 56) {
    while (bufferLength < 64) buffer[bufferLength++] = 0;
    processBlock();
    bufferLength = 0;
  }
  while (bufferLength < 56) buffer[bufferLength++] = 0;

  // Append length
  uint64_t bitLength = length;
  for (int i = 7; i >= 0; i--) {
    buffer[56 + i] = bitLength & 0xFF;
    bitLength >>= 8;
  }
  processBlock();

  // Output hash
  for (int i = 0; i < 5; i++) {
    hash[i * 4 + 0] = (h[i] >> 24) & 0xFF;
    hash[i * 4 + 1] = (h[i] >> 16) & 0xFF;
    hash[i * 4 + 2] = (h[i] >> 8) & 0xFF;
    hash[i * 4 + 3] = h[i] & 0xFF;
  }
}

std::string SHA1::compute(const std::string& input) {
  SHA1 sha;
  sha.update(reinterpret_cast<const uint8_t*>(input.data()), input.size());
  uint8_t hash[20];
  sha.finalize(hash);
  return std::string(reinterpret_cast<const char*>(hash), 20);
}

uint32_t SHA1::leftRotate(uint32_t value, int amount) {
  return (value << amount) | (value >> (32 - amount));
}

void SHA1::processBlock() {
  uint32_t w[80];
  for (int i = 0; i < 16; i++) {
    w[i] = (buffer[i * 4] << 24) | (buffer[i * 4 + 1] << 16) |
           (buffer[i * 4 + 2] << 8) | buffer[i * 4 + 3];
  }
  for (int i = 16; i < 80; i++) {
    w[i] = leftRotate(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
  }

  uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];

  for (int i = 0; i < 80; i++) {
    uint32_t f, k;
    if (i < 20) {
      f = (b & c) | ((~b) & d);
      k = 0x5A827999;
    } else if (i < 40) {
      f = b ^ c ^ d;
      k = 0x6ED9EBA1;
    } else if (i < 60) {
      f = (b & c) | (b & d) | (c & d);
      k = 0x8F1BBCDC;
    } else {
      f = b ^ c ^ d;
      k = 0xCA62C1D6;
    }

    uint32_t temp = leftRotate(a, 5) + f + e + k + w[i];
    e = d;
    d = c;
    c = leftRotate(b, 30);
    b = a;
    a = temp;
  }

  h[0] += a;
  h[1] += b;
  h[2] += c;
  h[3] += d;
  h[4] += e;
}

} // namespace internal
} // namespace miniros
