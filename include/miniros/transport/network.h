/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MINIROS_NETWORK_H
#define MINIROS_NETWORK_H

#include "miniros/internal/forwards.h"

namespace miniros
{

/**
 * \brief internal
 */
namespace network
{

MINIROS_DECL bool splitURI(const std::string& uri, std::string& host, uint32_t& port);
MINIROS_DECL const std::string& getHost();
MINIROS_DECL uint16_t getTCPROSPort();


/// Network address.
struct MINIROS_DECL NetAddress {
  enum Type {
    AddressInvalid,
    AddressIPv4,
    //AddressIPv6,
  };

  Type type;

  /// String representation of network address.
  std::string address;

  /// Network port.
  int port;

  /// Pointer to actual address implementation.
  void* rawAddress;

  NetAddress();
  NetAddress(const NetAddress& other);
  NetAddress(NetAddress&& other);

  ~NetAddress();

  /// Reset internal address.
  void reset();

  /// Check if address is valid.
  bool valid() const { return type != AddressInvalid; }
};

/// Fills in local address from socket.
MINIROS_DECL bool readLocalAddress_v4(int sockfd, NetAddress& address);

/// Fills in remote address from socket.
MINIROS_DECL bool readRemoteAddress_v4(int sockfd, NetAddress& address);

} // namespace network

} // namespace miniros

#endif
