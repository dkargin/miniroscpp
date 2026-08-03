//
// Multimaster UDP framing: ROS-serialized MasterUdpHeader, then payload bytes.
//
// Wire (no TCPROS 4-byte length prefix):
//   MasterUdpHeader (fixed-size; magic first)
//   payload...  (ROS-serialized body selected by header.op, or empty)
//
// checksum is CRC32 of payload bytes only (0 if empty).
//

#ifndef MINIROS_MULTIMASTER_PROTOCOL_H
#define MINIROS_MULTIMASTER_PROTOCOL_H

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <tuple>
#include <vector>

#include "miniros/errors.h"
#include "miniros/uuid.h"
#include "miniros/macros.h"
#include "miniros/serialization.h"

#include "miniros_msgs/MasterUdpHeader.hxx"

namespace miniros {
namespace master {
namespace mm {

using Header = miniros_msgs::MasterUdpHeader;

constexpr uint32_t kMagic = 0x314D524Du; // 'MRM1' little-endian
constexpr uint8_t kVersion = Header::VERSION;
constexpr size_t kTokenHashSize = 8;
constexpr size_t kMaxPayload = 1200;

static_assert(kTokenHashSize == std::tuple_size<Header::_token_hash_type>::value, "token hash size");
static_assert(UUID::Dim == std::tuple_size<Header::_uuid_type>::value, "uuid size");

/// Compute 8-byte token fingerprint (first 8 bytes of SHA-1).
MINIROS_DECL void tokenHash(const std::string& token, std::array<uint8_t, kTokenHashSize>& out);
MINIROS_DECL bool tokenHashEqual(const std::array<uint8_t, kTokenHashSize>& a,
  const std::array<uint8_t, kTokenHashSize>& b);
MINIROS_DECL bool tokenHashEmpty(const std::array<uint8_t, kTokenHashSize>& h);

/// CRC32 of payload (ISO-HDLC / Ethernet polynomial).
MINIROS_DECL uint32_t crc32(const uint8_t* data, size_t size);

/// Serialize a ROS message into a raw payload buffer (no 4-byte length prefix).
template <typename M>
Error serializePayload(const M& msg, std::vector<uint8_t>& out)
{
  const uint32_t len = serialization::serializationLength(msg);
  out.resize(len);
  if (len == 0)
    return Error::Ok;
  serialization::OStream stream(out.data(), len);
  try {
    serialization::serialize(stream, msg);
  } catch (...) {
    return Error::InvalidValue;
  }
  return Error::Ok;
}

/// Deserialize a ROS message from a raw payload buffer.
template <typename M>
Error deserializePayload(const uint8_t* data, size_t size, M& msg)
{
  if (size == 0) {
    msg = M();
    return Error::Ok;
  }
  if (!data)
    return Error::InvalidValue;
  serialization::IStream stream(const_cast<uint8_t*>(data), static_cast<uint32_t>(size));
  try {
    serialization::deserialize(stream, msg);
  } catch (...) {
    return Error::InvalidValue;
  }
  return Error::Ok;
}

template <typename M>
Error deserializePayload(const std::vector<uint8_t>& data, M& msg)
{
  return deserializePayload(data.data(), data.size(), msg);
}

/// Build datagram: serialized Header followed by payload; fills checksum from payload.
MINIROS_DECL Error buildPacket(Header h, const std::vector<uint8_t>& payload, std::vector<uint8_t>& out);

/// Parse datagram; verifies magic/checksum. Payload points into @p data (not copied).
MINIROS_DECL Error parsePacket(const uint8_t* data, size_t size, Header& header, const uint8_t*& payload,
  size_t& payload_len);

} // namespace mm
} // namespace master
} // namespace miniros

#endif // MINIROS_MULTIMASTER_PROTOCOL_H
