#include "miniros/uuid.h"

#include <cstring>
#include <random>

namespace miniros {

namespace {

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_int_distribution<> dis(0, 255);

constexpr unsigned int UUID_Version = 0x40;
constexpr unsigned int UUID_Variant = 0x80;

} // namespace

void UUID::generate()
{
  for (int i = 0; i < Dim; i++) {
    bytes[i] = dis(gen);
  }
  constexpr unsigned int mask4 = 0xf;
  constexpr unsigned int mask3 = 0x3f;
  bytes[6] = (bytes[6] & mask4) | UUID_Version;
  bytes[8] = (bytes[8] & mask3) | UUID_Variant;
}

void UUID::reset()
{
  for (int i = 0; i < Dim; i++) {
    bytes[i] = 0;
  }
}

bool UUID::valid() const
{
  if (!(bytes[6] & UUID_Version))
    return false;
  if (!(bytes[8] & UUID_Variant))
    return false;

  for (int i = 0; i < Dim; i++) {
    if (bytes[i] != 0)
      return true;
  }
  return false;
}

std::string UUID::toString() const
{
  static const char* hex = "0123456789abcdef";
  std::string out;
  out.reserve(36);
  for (int i = 0; i < Dim; ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10)
      out.push_back('-');
    out.push_back(hex[bytes[i] >> 4]);
    out.push_back(hex[bytes[i] & 0xf]);
  }
  return out;
}

bool UUID::fromString(const std::string& str)
{
  auto hexNibble = [](char c) -> int {
    if (c >= '0' && c <= '9')
      return c - '0';
    if (c >= 'a' && c <= 'f')
      return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
      return c - 'A' + 10;
    return -1;
  };

  uint8_t parsed[Dim] = {};
  size_t byteIndex = 0;
  for (size_t i = 0; i < str.size() && byteIndex < Dim; ) {
    if (str[i] == '-') {
      ++i;
      continue;
    }
    if (i + 1 >= str.size())
      return false;
    const int hi = hexNibble(str[i]);
    const int lo = hexNibble(str[i + 1]);
    if (hi < 0 || lo < 0)
      return false;
    parsed[byteIndex++] = static_cast<uint8_t>((hi << 4) | lo);
    i += 2;
  }
  if (byteIndex != Dim)
    return false;

  std::memcpy(bytes, parsed, Dim);
  return valid();
}

bool operator==(const UUID& a, const UUID& b)
{
  return std::memcmp(a.bytes, b.bytes, UUID::Dim) == 0;
}

} // namespace miniros
