#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "miniros/io/io.h"
#include "miniros/master_discovery.h"
#include "miniros/network/net_address.h"
#include "miniros/network/socket.h"
#include "miniros/uuid.h"

#include "multimaster_protocol.h"
#include "miniros_msgs/MasterOffer.hxx"

using miniros::DiscoveredMaster;
using miniros::DiscoveryReport;
using miniros::Error;
using miniros::MasterDiscovery;
using miniros::UUID;
using miniros::WallDuration;

namespace {

int freeUdpPort()
{
  miniros::network::NetSocket probe;
  EXPECT_TRUE(probe.initUDP(false));
  EXPECT_TRUE(probe.bind(0));
  const int port = probe.port();
  probe.close();
  return port;
}

void sendAdvert(int port, uint8_t op, const UUID& id, const std::string& host, uint16_t masterPort, bool withToken)
{
  miniros_msgs::MasterOffer offer;
  offer.master_port = masterPort;
  offer.host = host;
  std::vector<uint8_t> payload;
  ASSERT_TRUE(miniros::master::mm::serializePayload(offer, payload));

  miniros::master::mm::Header header;
  header.op = op;
  header.seq = 1;
  header.uuid = id.toBytes();
  if (withToken)
    header.token_hash[0] = 0xab;

  std::vector<uint8_t> packet;
  ASSERT_TRUE(miniros::master::mm::buildPacket(header, payload, packet));

  miniros::network::NetSocket sender;
  ASSERT_TRUE(sender.initUDP(false));
  // Unicast to the bound discovery port. Multicast delivery needs a joined group;
  // the receive and report path is the same for both.
  const miniros::network::NetAddress dest =
    miniros::network::NetAddress::fromIp4String("127.0.0.1", port);
  auto [n, err] = sender.send(packet.data(), packet.size(), &dest);
  ASSERT_TRUE(err) << err.toString();
  ASSERT_EQ(n, packet.size());
}

DiscoveryReport waitFor(MasterDiscovery& discovery, std::chrono::milliseconds budget)
{
  const auto deadline = std::chrono::steady_clock::now() + budget;
  DiscoveryReport all;
  while (std::chrono::steady_clock::now() < deadline) {
    DiscoveryReport step = discovery.update();
    all.added.insert(all.added.end(), step.added.begin(), step.added.end());
    all.updated.insert(all.updated.end(), step.updated.begin(), step.updated.end());
    all.removed.insert(all.removed.end(), step.removed.begin(), step.removed.end());
    if (!all.added.empty() || !all.updated.empty() || !all.removed.empty())
      return all;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return all;
}

const DiscoveredMaster* findMaster(const std::vector<DiscoveredMaster>& list, const UUID& id)
{
  for (const auto& master : list) {
    if (master.uuid == id)
      return &master;
  }
  return nullptr;
}

} // namespace

TEST(MasterDiscovery, defaultsMatchMiniroscoreGroup)
{
  EXPECT_STREQ(MasterDiscovery::kDefaultGroup, "239.255.42.42");
  EXPECT_EQ(MasterDiscovery::kDefaultPort, 11312);
}

TEST(MasterDiscovery, updateReportsAdvertisedMaster)
{
  const int port = freeUdpPort();
  ASSERT_GT(port, 0);

  MasterDiscovery discovery;
  ASSERT_TRUE(discovery.setGroup(MasterDiscovery::kDefaultGroup, port));
  ASSERT_TRUE(discovery.start());
  EXPECT_TRUE(discovery.started());
  EXPECT_TRUE(discovery.update().empty());

  UUID id;
  id.generate();
  sendAdvert(port, miniros_msgs::MasterUdpHeader::OP_DISCOVER, id, "10.1.2.3|field-robot", 11311, false);

  const DiscoveryReport found = waitFor(discovery, std::chrono::milliseconds(1000));
  ASSERT_EQ(found.added.size(), 1u);
  EXPECT_TRUE(found.updated.empty());
  EXPECT_TRUE(found.removed.empty());

  const DiscoveredMaster& master = found.added.front();
  EXPECT_EQ(master.uuid, id);
  EXPECT_EQ(master.hostname, "field-robot");
  EXPECT_EQ(master.address, "10.1.2.3");
  EXPECT_EQ(master.masterPort, 11311);
  EXPECT_EQ(master.uri, "http://10.1.2.3:11311/");
  EXPECT_FALSE(master.hasToken);

  EXPECT_TRUE(discovery.update().empty());
  ASSERT_EQ(discovery.masters().size(), 1u);

  sendAdvert(port, miniros_msgs::MasterUdpHeader::OP_OFFER, id, "10.9.8.7|field-robot", 11411, true);
  const DiscoveryReport changed = waitFor(discovery, std::chrono::milliseconds(1000));
  ASSERT_EQ(changed.updated.size(), 1u);
  EXPECT_TRUE(changed.added.empty());
  EXPECT_EQ(changed.updated.front().address, "10.9.8.7");
  EXPECT_EQ(changed.updated.front().masterPort, 11411);
  EXPECT_EQ(changed.updated.front().uri, "http://10.9.8.7:11411/");
  EXPECT_TRUE(changed.updated.front().hasToken);
  EXPECT_EQ(discovery.masters().size(), 1u);

  discovery.setStaleTimeout(WallDuration(0.05));
  std::this_thread::sleep_for(std::chrono::milliseconds(80));
  const DiscoveryReport gone = discovery.update();
  ASSERT_EQ(gone.removed.size(), 1u);
  EXPECT_EQ(gone.removed.front().uuid, id);
  EXPECT_TRUE(discovery.masters().empty());
}

TEST(MasterDiscovery, ignoresUnrelatedOpcodes)
{
  const int port = freeUdpPort();
  ASSERT_GT(port, 0);

  MasterDiscovery discovery;
  ASSERT_TRUE(discovery.setGroup(MasterDiscovery::kDefaultGroup, port));
  ASSERT_TRUE(discovery.start());

  UUID id;
  id.generate();
  sendAdvert(port, miniros_msgs::MasterUdpHeader::OP_HEARTBEAT, id, "10.1.2.3|field-robot", 11311, false);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_TRUE(discovery.update().empty());
  EXPECT_TRUE(discovery.masters().empty());
  EXPECT_EQ(findMaster(discovery.masters(), id), nullptr);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  miniros::ensureNetworkInitialized();
  return RUN_ALL_TESTS();
}
