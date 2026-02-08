//
// Created by dkargin on 3/16/25.
//

#ifndef MINIROS_NET_ADDRESS_H
#define MINIROS_NET_ADDRESS_H

#include <string>

#include "miniros/macros.h"
#include "miniros/errors.h"

namespace miniros {
namespace network {

struct URL;

/// Network address.
/// Main goal of this structure is to hide system-dependent details from user code.
class MINIROS_DECL NetAddress {
public:
  enum Type {
    AddressInvalid,
    AddressIPv4,
    AddressIPv6,
    /// Unspecified address type. Used for creating NetAddress address from arbitrary hostname.
    AddressUnspecified,
  };

  /// String representation of network address.
  /// It does not contain port. It can contain some DNS name.
  std::string address;

  /// Address is unspecified.
  bool unspecified = false;

  NetAddress();
  NetAddress(const NetAddress& other);
  NetAddress(NetAddress&& other) noexcept;
  ~NetAddress();

  NetAddress& operator=(const NetAddress& other);

  /// Reset internal address.
  void reset();

  /// Check if an address is loopback.
  bool isLoopback() const;

  bool isUnspecified() const;

  /// Get network port.
  int port() const;

  /// Check if address is valid.
  bool valid() const { return type_ != AddressInvalid; }

  Type type() const { return type_; }

  /// Generate string representation of an address.
  std::string str() const;

  /// Check type of provided address in a string form.
  static Type checkAddressType(const std::string& address);

  friend bool operator < (const NetAddress& a, const NetAddress& b);
  friend bool operator == (const NetAddress& a, const NetAddress& b);
  friend bool operator != (const NetAddress& a, const NetAddress& b);

  /// Get pointer to a raw address.
  /// Typically, it is sockaddr_t object.
  const void* rawAddress() const
  {
    return rawAddress_;
  }

  /// Get size of network address.
  size_t rawAddressSize() const
  {
    return rawAddressSize_;
  }

  /// Update port for existing address.
  /// Port must be nonzero value.
  Error setPort(int port);

  /// Assign new raw address value.
  void assignRawAddress(Type type, const void* addr, size_t size);

  /// Create network address from some IP address or hostname and port.
  /// @param type - address family.
  /// @param address - hostname or IP (or similar) address.
  /// @param port - numeric port.
  MINIROS_NODISCARD static NetAddress fromString(Type type, const std::string& address, int port);

  /// Create network address from string representation of IP4 address.
  /// This function will not resolve DNS names.
  MINIROS_NODISCARD static NetAddress fromIp4String(const std::string& address, int port)
  {
    return fromString(Type::AddressIPv4, address, port);
  }

  /// Create network address from string representation of IPv6 address.
  /// This function will not resolve DNS names.
  MINIROS_NODISCARD static NetAddress fromIp6String(const std::string& address, int port)
  {
    return fromString(Type::AddressIPv6, address, port);
  }

  /// Create network address from URL string.
  /// @param address - URL string to parse (e.g., "http://example.com:8080/path")
  /// @param port - default port to use if URL doesn't specify one
  MINIROS_NODISCARD static NetAddress fromURL(const std::string& address, int port);

  /// Create network address from URL object.
  /// @param url - URL object containing host and port information
  MINIROS_NODISCARD static NetAddress fromURL(const URL& url);

protected:
  Type type_ = AddressInvalid;

  /// Pointer to actual address implementation.
  /// It points to sockaddr_t or one of its variants.
  void* rawAddress_ = nullptr;
  size_t rawAddressSize_ = 0;
};

/// Address comparator which checks for IP address only, without comparing the port.
struct AddressCompareNoPort {
  bool operator()(const NetAddress& a, const NetAddress& b) const;
};

/// Fills in local address from socket.
MINIROS_DECL Error readLocalAddress(int sockfd, NetAddress& address);

/// Fills in remote address from socket.
MINIROS_DECL Error readRemoteAddress(int sockfd, NetAddress& address);

/// Fill in broadcast address with specific port.
MINIROS_DECL Error fillBroadcastAddress(NetAddress& address, int port, NetAddress::Type type);

/// Information about connection to client.
struct MINIROS_DECL ClientInfo {
  NetAddress remoteAddress;
  NetAddress localAddress;
  int fd = -1;
  bool sameProcess = false;
};

} // namespace network
} // namespace miniros

#endif //MINIROS_NET_ADDRESS_H
