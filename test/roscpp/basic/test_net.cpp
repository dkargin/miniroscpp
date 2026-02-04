//
// Created by dkargin on 7/21/25.
//

#include <gtest/gtest.h>

#include "miniros/network/net_adapter.h"
#include "miniros/network/net_address.h"
#include "miniros/network/socket.h"
#include "miniros/io/io.h"

using Error = miniros::Error;

using namespace miniros::network;

TEST(Address, ip4)
{
  NetAddress address1 = NetAddress::fromIp4String("192.168.1.10", 10);
  ASSERT_TRUE(address1.valid());
  EXPECT_GT(address1.rawAddressSize(), 0);
  EXPECT_EQ(address1.port(), 10);

  const sockaddr* addr = static_cast<const sockaddr*>(address1.rawAddress());
  NetAddress address2;
  ASSERT_EQ(fillAddress(addr, address2), Error::Ok);

  EXPECT_EQ(address2.port(), 10);
  EXPECT_EQ(address1, address2);

  NetAddress address3 = NetAddress::fromIp4String("very wrong address", 10);
  ASSERT_FALSE(address3.valid());
  EXPECT_EQ(address3.type(), NetAddress::AddressInvalid);
}

TEST(Address, ip6)
{
  NetAddress address1 = NetAddress::fromIp6String("fe80::1ff:fe23:4567:890a", 10);
  ASSERT_TRUE(address1.valid());
  EXPECT_GT(address1.rawAddressSize(), 0);
  EXPECT_EQ(address1.port(), 10);

  const sockaddr* addr = static_cast<const sockaddr*>(address1.rawAddress());
  NetAddress address2;
  ASSERT_EQ(fillAddress(addr, address2), Error::Ok);

  EXPECT_EQ(address2.port(), 10);
  EXPECT_EQ(address1, address2);


  NetAddress address3 = NetAddress::fromIp6String("very wrong address", 10);
  ASSERT_FALSE(address3.valid());
  EXPECT_EQ(address3.type(), NetAddress::AddressInvalid);
}

TEST(Adapters, SameNet)
{
  NetAdapter adapter;

  adapter.address = NetAddress::fromIp4String("192.168.1.10", 0);
  adapter.mask = NetAddress::fromIp4String("255.255.255.0", 0);

  NetAddress address1 = NetAddress::fromIp4String("192.168.1.11", 13);
  EXPECT_TRUE(adapter.matchNetAddress(address1));

  NetAddress address2 = NetAddress::fromIp4String("192.168.2.31", 13);
  EXPECT_FALSE(adapter.matchNetAddress(address2));
}

TEST(SocketUDP, SimplestBroadcast)
{
  // Just sending packet to some broadcast address.
  NetSocket socket;
  constexpr int port = 11811;

  ASSERT_TRUE(socket.initUDP(false));
  ASSERT_TRUE(socket.setBroadcast(true));
  ASSERT_TRUE(socket.bind(port));

  const char msg[] = "hello";
  const char HOSTNAME[] = "127.0.0.1";

  // Sending to broadcast address from sockaddr_in
  sockaddr_in servaddr;
  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = INADDR_BROADCAST; //inet_addr(HOSTNAME);
  servaddr.sin_port = htons(port);

  NetAddress brAddress;
  brAddress.assignRawAddress(NetAddress::AddressIPv4, &servaddr, sizeof(servaddr));

  auto [sent1, err1] = socket.send(msg, strlen(msg) + 1, &brAddress);
  ASSERT_EQ(err1, Error::Ok);
  ASSERT_GT(sent1, 0);

  const char msg2[] = "world";
  NetAddress address = NetAddress::fromIp4String("255.255.255.255", port);
  auto [sent2, err2] = socket.send(msg2, sizeof(msg2), &address);
  ASSERT_EQ(err2, Error::Ok);
  ASSERT_GT(sent2, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
