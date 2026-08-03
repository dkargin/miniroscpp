//
// Multimaster UDP framing encode/decode via ROS message serialization.
//

#include "multimaster_protocol.h"

#include "miniros/internal/sha1.h"

#include <algorithm>
#include <cassert>

namespace miniros {
namespace master {
namespace mm {

void tokenHash(const std::string& token, std::array<uint8_t, kTokenHashSize>& out)
{
  out.fill(0);
  if (token.empty())
    return;
  std::string dig = internal::SHA1::compute(token);
  assert(dig.size() >= kTokenHashSize);
  std::memcpy(out.data(), dig.data(), kTokenHashSize);
}

bool tokenHashEqual(const std::array<uint8_t, kTokenHashSize>& a, const std::array<uint8_t, kTokenHashSize>& b)
{
  return a == b;
}

bool tokenHashEmpty(const std::array<uint8_t, kTokenHashSize>& h)
{
  return std::all_of(h.begin(), h.end(), [](uint8_t b) { return b == 0; });
}

uint32_t crc32(const uint8_t* data, size_t size)
{
  uint32_t crc = 0xffffffffu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int b = 0; b < 8; ++b) {
      const uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xedb88320u & mask);
    }
  }
  return ~crc;
}

Error buildPacket(Header h, const std::vector<uint8_t>& payload, std::vector<uint8_t>& out)
{
  h.magic = kMagic;
  h.version = kVersion;
  h.checksum = payload.empty() ? 0 : crc32(payload.data(), payload.size());
  if (Error e = serializePayload(h, out); !e)
    return e;
  out.insert(out.end(), payload.begin(), payload.end());
  return Error::Ok;
}

Error parsePacket(const uint8_t* data, size_t size, Header& header, const uint8_t*& payload, size_t& payload_len)
{
  if (!data || size < 4)
    return Error::InvalidValue;
  const uint32_t magic = static_cast<uint32_t>(data[0]) | (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) | (static_cast<uint32_t>(data[3]) << 24);
  if (magic != kMagic)
    return Error::InvalidValue;
  serialization::IStream stream(const_cast<uint8_t*>(data), static_cast<uint32_t>(size));
  try {
    serialization::deserialize(stream, header);
  } catch (...) {
    return Error::InvalidValue;
  }
  if (header.magic != kMagic)
    return Error::InvalidValue;
  payload = stream.getData();
  payload_len = stream.getLength();
  const uint32_t expect = payload_len == 0 ? 0 : crc32(payload, payload_len);
  if (header.checksum != expect)
    return Error::InvalidValue;
  return Error::Ok;
}

} // namespace mm
} // namespace master
} // namespace miniros
