//
// Created by dkargin on 7/21/25.
//

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "miniros/network/net_adapter.h"
#include "miniros/network/net_address.h"
#include "miniros/network/socket.h"
#include "miniros/io/io.h"

#ifndef WIN32
#include <sys/socket.h>
#endif

using Error = miniros::Error;

using namespace miniros::network;

namespace {

std::string makePatternedBuffer(size_t size, char seed)
{
  std::string out(size, '\0');
  for (size_t i = 0; i < size; ++i) {
    out[i] = static_cast<char>(seed + static_cast<char>(i % 67));
  }
  return out;
}

/// Drain receiver until expectedSize bytes arrive or timeout.
std::string recvExact(NetSocket& sock, size_t expectedSize, std::chrono::milliseconds timeout)
{
  std::string received;
  received.reserve(expectedSize);
  const auto deadline = std::chrono::steady_clock::now() + timeout;

  while (received.size() < expectedSize) {
    if (std::chrono::steady_clock::now() > deadline) {
      break;
    }
    auto [n, err] = sock.recv(received, nullptr);
    if (err == Error::EndOfFile) {
      break;
    }
    if (err == Error::WouldBlock || n == 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (err != Error::Ok && err != Error::WouldBlock) {
      break;
    }
  }
  return received;
}

/// Keep calling write2 until all data is sent or a hard error occurs.
std::pair<size_t, Error> write2Fully(
  NetSocket& sock,
  const std::string& header,
  const std::string& body,
  size_t alreadyWritten = 0)
{
  const size_t total = header.size() + body.size();
  size_t written = alreadyWritten;
  size_t attempts = 0;
  constexpr size_t kMaxAttempts = 100000;

  while (written < total && attempts++ < kMaxAttempts) {
    auto [n, err] = sock.write2(
      header.data(), header.size(),
      body.data(), body.size(),
      written);
    written += n;
    if (err == Error::WouldBlock || (err == Error::Ok && written < total)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    if (err != Error::Ok) {
      return {written - alreadyWritten, err};
    }
    break;
  }
  return {written - alreadyWritten, written >= total ? Error::Ok : Error::Timeout};
}

bool shrinkSendBuffer(NetSocket& sock, int bytes)
{
#ifndef WIN32
  return setsockopt(sock.fd(), SOL_SOCKET, SO_SNDBUF, &bytes, sizeof(bytes)) == 0;
#else
  (void)sock;
  (void)bytes;
  return true;
#endif
}

struct TcpPair {
  std::unique_ptr<NetSocket> listener;
  std::unique_ptr<NetSocket> client;
  std::shared_ptr<NetSocket> server;
};

TcpPair makeConnectedPair()
{
  TcpPair pair;
  pair.listener = std::make_unique<NetSocket>();
  pair.client = std::make_unique<NetSocket>();

  EXPECT_EQ(pair.listener->tcpListen(0, NetAddress::AddressIPv4), Error::Ok);
  const int port = pair.listener->port();
  EXPECT_GT(port, 0);

  NetAddress addr = NetAddress::fromIp4String("127.0.0.1", port);
  EXPECT_EQ(pair.client->tcpConnect(addr, false), Error::Ok);

  auto [accepted, acceptErr] = pair.listener->accept();
  EXPECT_EQ(acceptErr, Error::Ok);
  EXPECT_TRUE(accepted);
  pair.server = accepted;

  // NetSocket::recv drains until WouldBlock, so both ends must be non-blocking.
  EXPECT_EQ(pair.client->setNonBlock(), Error::Ok);
  EXPECT_EQ(pair.server->setNonBlock(), Error::Ok);
  return pair;
}

}  // namespace

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

TEST(Address, invalidAddress)
{
  NetAddress address;
  // Check completely invalid address.
  Error err = addressFromString(NetAddress::AddressIPv4, "1235132525233421234_235^232147", 123456, address);
  EXPECT_EQ(err, Error::InvalidValue);
  ASSERT_FALSE(address.valid());

  // Check some valid hostname with just an unknown IP.
  Error err2 = addressFromString(NetAddress::AddressIPv4, "nice_but_not_known", 1234, address);
  EXPECT_EQ(err2, Error::AddressIsUnknown);
  EXPECT_FALSE(address.valid());
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
  ASSERT_EQ(adapter.address.type(), NetAddress::AddressIPv4);
  adapter.mask = NetAddress::fromIp4String("255.255.255.0", 0);
  ASSERT_EQ(adapter.mask.type(), NetAddress::AddressIPv4);

  NetAddress address1 = NetAddress::fromIp4String("192.168.1.11", 13);
  ASSERT_EQ(address1.type(), NetAddress::AddressIPv4);

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
  ASSERT_EQ(sent1, 6);

  const char msg2[] = "world";
  NetAddress address = NetAddress::fromIp4String("255.255.255.255", port);
  auto [sent2, err2] = socket.send(msg2, sizeof(msg2), &address);
  ASSERT_EQ(err2, Error::Ok);
  ASSERT_EQ(sent2, 6);
}

TEST(SocketTCP, Write2SmallPayload)
{
  auto pair = makeConnectedPair();
  ASSERT_TRUE(pair.server);
  ASSERT_TRUE(pair.client);

  const std::string header = "HDR:";
  const std::string body = "payload-body";
  const std::string expected = header + body;

  auto [sent, sendErr] = write2Fully(*pair.client, header, body);
  ASSERT_EQ(sendErr, Error::Ok);
  EXPECT_EQ(sent, expected.size());

  std::string received = recvExact(*pair.server, expected.size(), std::chrono::seconds(2));
  EXPECT_EQ(received, expected);
}

TEST(SocketTCP, Write2ResumeAfterHeaderFullySent)
{
  // Reproduces the fillIoVec bug: written == headerSize must still send the body.
  auto pair = makeConnectedPair();
  ASSERT_TRUE(pair.server);
  ASSERT_TRUE(pair.client);

  const std::string header = "HTTP/1.1 200 OK\r\nContent-Length: 12\r\n\r\n";
  const std::string body = "hello-world!";
  const std::string expected = header + body;

  // First call pretends the header was already acknowledged.
  auto [sent, sendErr] = write2Fully(*pair.client, header, body, header.size());
  ASSERT_EQ(sendErr, Error::Ok);
  EXPECT_EQ(sent, body.size());

  std::string received = recvExact(*pair.server, body.size(), std::chrono::seconds(2));
  EXPECT_EQ(received, body);
}

TEST(SocketTCP, Write2ResumeMidHeaderAndMidBody)
{
  auto pair = makeConnectedPair();
  ASSERT_TRUE(pair.server);
  ASSERT_TRUE(pair.client);

  const std::string header = makePatternedBuffer(128, 'A');
  const std::string body = makePatternedBuffer(256, 'a');
  const std::string expected = header + body;

  // Send first half of the header with a plain send(), then resume via write2.
  {
    auto [n, err] = pair.client->send(header.data(), header.size() / 2, nullptr);
    ASSERT_EQ(err, Error::Ok);
    ASSERT_EQ(n, header.size() / 2);
  }

  auto [sent, sendErr] = write2Fully(*pair.client, header, body, header.size() / 2);
  ASSERT_EQ(sendErr, Error::Ok);
  EXPECT_EQ(sent, expected.size() - header.size() / 2);

  std::string received = recvExact(*pair.server, expected.size(), std::chrono::seconds(2));
  EXPECT_EQ(received, expected);
}

TEST(SocketTCP, Write2LargeHttpResponseChunked)
{
  // Mirrors HttpServerConnection::handleWriteResponse for large file/API payloads:
  // small HTTP header + multi-megabyte body, resumed via cumulative `data_sent_`.
  auto pair = makeConnectedPair();
  ASSERT_TRUE(pair.server);
  ASSERT_TRUE(pair.client);

  // Shrink send buffer so the OS cannot accept 5–10MB in one write2 call.
  shrinkSendBuffer(*pair.client, 4 * 1024);

  constexpr size_t kBodySize = 8 * 1024 * 1024;  // 8MB, in the observed 5–10MB range
  const std::string body = makePatternedBuffer(kBodySize, 'B');
  const std::string header =
    "HTTP/1.1 200 OK\r\n"
    "Content-Type: application/octet-stream\r\n"
    "Content-Length: " + std::to_string(body.size()) + "\r\n"
    "Connection: keep-alive\r\n"
    "\r\n";
  ASSERT_LT(header.size(), 256u);
  const std::string expected = header + body;
  const size_t total = expected.size();

  std::string received;
  received.reserve(total);
  std::thread receiver([&]() {
    received = recvExact(*pair.server, total, std::chrono::seconds(60));
  });

  // Same retry shape as handleWriteResponse: pass data_sent_ into write2 each time.
  size_t data_sent = 0;
  size_t wouldBlockCount = 0;
  size_t iterations = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);

  while (data_sent < total) {
    ASSERT_LT(std::chrono::steady_clock::now(), deadline)
      << "timed out after " << data_sent << "/" << total
      << " (WouldBlock=" << wouldBlockCount << ", iterations=" << iterations << ")";

    auto [n, err] = pair.client->write2(
      header.data(), header.size(),
      body.data(), body.size(),
      data_sent);
    ++iterations;
    if (n > 0) {
      data_sent += n;
    }

    if (err == Error::WouldBlock) {
      ++wouldBlockCount;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    ASSERT_EQ(err, Error::Ok) << "write2 failed at " << data_sent << "/" << total;
    if (data_sent < total) {
      // Defensive: Ok with incomplete send must not drop the remainder.
      ++wouldBlockCount;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    break;
  }

  EXPECT_EQ(data_sent, total);
  EXPECT_GT(wouldBlockCount, 0u)
    << "expected at least one WouldBlock for 8MB non-blocking write2 (iterations="
    << iterations << ")";
  // Crossing the header→body boundary must keep working after partial sends.
  EXPECT_GT(data_sent, header.size());

  receiver.join();
  ASSERT_EQ(received.size(), total);
  EXPECT_EQ(received.compare(0, header.size(), header), 0);
  EXPECT_EQ(received.compare(header.size(), body.size(), body), 0);
}

TEST(SocketTCP, Write2HeaderOnlyAndBodyOnlyEdges)
{
  auto pair = makeConnectedPair();
  ASSERT_TRUE(pair.server);
  ASSERT_TRUE(pair.client);

  const std::string header = "only-header";
  {
    auto [sent, err] = write2Fully(*pair.client, header, {});
    ASSERT_EQ(err, Error::Ok);
    EXPECT_EQ(sent, header.size());
    EXPECT_EQ(recvExact(*pair.server, header.size(), std::chrono::seconds(2)), header);
  }

  // Already-complete write reports Ok / 0 bytes.
  {
    auto [n, err] = pair.client->write2(header.data(), header.size(), nullptr, 0, header.size());
    EXPECT_EQ(err, Error::Ok);
    EXPECT_EQ(n, 0u);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
