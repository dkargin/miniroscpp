//
// Created by dkargin on 3/16/25.
//

#ifndef MINIROS_NET_ADDRESS_H
#define MINIROS_NET_ADDRESS_H

#include <cstdint>
#include <string>

#include "miniros/macros.h"

namespace miniros {
namespace network {

/// Network address.
struct MINIROS_DECL NetAddress {
  enum Type {
    AddressInvalid,
    AddressIPv4,
    AddressIPv6,
  };

  Type type = AddressInvalid;

  /// String representation of network address.
  std::string address;

  /// Network port.
  int port = 0;

  /// Pointer to actual address implementation.
  /// It points to sockaddr_t or one of its variants.
  void* rawAddress = nullptr;

  NetAddress();
  NetAddress(const NetAddress& other);
  NetAddress(NetAddress&& other) noexcept;
  ~NetAddress();

  NetAddress& operator=(const NetAddress& other);

  /// Reset internal address.
  void reset();

  /// Check if an address is local one.
  bool isLocal() const;

  /// Check if address is valid.
  bool valid() const { return type != AddressInvalid; }

  /// Generate string representation of an address.
  std::string str() const;

  /// Check type of provided address in a string form.
  static Type checkAddressType(const std::string& address);

  friend bool operator < (const NetAddress& a, const NetAddress& b);
  friend bool operator == (const NetAddress& a, const NetAddress& b);
  friend bool operator != (const NetAddress& a, const NetAddress& b);
};

/// Address comparator which checks for IP address only, without comparing the port.
struct AddressCompare {
  bool operator()(const NetAddress& a, const NetAddress& b) const;
};

/// Fills in local address from socket.
MINIROS_DECL bool readLocalAddress(int sockfd, NetAddress& address);

/// Fills in remote address from socket.
MINIROS_DECL bool readRemoteAddress(int sockfd, NetAddress& address);

} // namespace network
} // namespace miniros

#endif //MINIROS_NET_ADDRESS_H
