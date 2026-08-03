//
// Integration tests for multimaster collectives (2-master + 3-master mesh).
//

#include <chrono>
#include <cstdlib>
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "miniros/build_config.h"
#include "miniros/launcher.h"
#include "miniros/network/socket.h"

namespace {

constexpr int kMasterPortA = 11411;
constexpr int kMasterPortB = 11511;
constexpr int kMasterPortC = 11611;
constexpr const char* kToken = "mm_test_token";

std::filesystem::path binDir()
{
#ifdef MINIROS_TEST_BIN_DIR
  return std::filesystem::path(MINIROS_TEST_BIN_DIR);
#else
  return std::filesystem::current_path();
#endif
}

std::filesystem::path executable(const char* name)
{
  return binDir() / name;
}

std::filesystem::path multimasterLogDir()
{
  return binDir() / "multimaster-logs";
}

std::filesystem::path multimasterLogPath(const std::string& leafName)
{
  const auto logDir = multimasterLogDir();
  std::error_code ec;
  std::filesystem::create_directories(logDir, ec);
  return logDir / leafName;
}

/// Print multimaster console/crash excerpts to stderr (shows up in ctest / CI job logs).
void dumpMultimasterLogs(const char* reason)
{
  const auto logDir = multimasterLogDir();
  std::cerr << "=== MULTIMASTER DIAGNOSTICS (" << reason << ") dir=" << logDir << " ===\n";
  std::error_code ec;
  if (!std::filesystem::is_directory(logDir, ec)) {
    std::cerr << "(log dir missing)\n";
    return;
  }

  const auto printFile = [](const std::filesystem::path& path) {
    std::error_code fec;
    if (!std::filesystem::is_regular_file(path, fec))
      return;
    const auto sz = std::filesystem::file_size(path, fec);
    if (fec) {
      std::cerr << "=== " << path.filename().string() << " (stat failed) ===\n";
      return;
    }
    std::cerr << "=== " << path.filename().string() << " (" << sz << " bytes) ===\n";
    if (sz == 0) {
      std::cerr << "(empty)\n";
      return;
    }
    std::ifstream in(path);
    if (!in) {
      std::cerr << "(failed to open)\n";
      return;
    }
    constexpr std::uintmax_t kMax = 64 * 1024;
    if (sz <= kMax) {
      std::cerr << in.rdbuf();
    } else {
      in.seekg(static_cast<std::streamoff>(sz - kMax));
      std::string line;
      std::getline(in, line); // discard partial first line
      std::cerr << in.rdbuf();
    }
    std::cerr << "\n=== end " << path.filename().string() << " ===\n";
  };

  for (const auto& entry : std::filesystem::directory_iterator(logDir, ec)) {
    if (!entry.is_regular_file(ec))
      continue;
    const auto name = entry.path().filename().string();
    if (name.find(".console.log") != std::string::npos || name.find(".crash") != std::string::npos)
      printFile(entry.path());
  }
  std::cerr << "=== END MULTIMASTER DIAGNOSTICS ===\n";
}

std::string httpGet(const std::string& url)
{
  const std::string cmd = "curl -s --max-time 2 \"" + url + "\"";
  FILE* pipe = popen(cmd.c_str(), "r");
  if (!pipe)
    return {};
  std::string out;
  char buf[512];
  while (fgets(buf, sizeof(buf), pipe))
    out += buf;
  pclose(pipe);
  return out;
}

void debugShutdown(int port)
{
  (void)httpGet("http://127.0.0.1:" + std::to_string(port) + "/debugAPI/shutdown");
}

void stopProcess(miniros::Launcher& proc)
{
  if (!proc.valid())
    return;
  // SIGINT then SIGKILL — never hang the suite if a helper ignores signals.
  (void)proc.stopAndWait(miniros::WallDuration(2.0));
}

void stopMaster(miniros::Launcher& proc, int port)
{
  if (proc.valid())
    debugShutdown(port);
  stopProcess(proc);
}

/// CI sets TSAN_OPTIONS=halt_on_error=1 for the suite. Multimaster helpers that
/// inherit that and hit a race abort before advertising topics, which fails the
/// test as a mysterious empty topic list. Keep reporting races, do not kill kids.
void softenSanitizerForChild(miniros::Launcher& launcher)
{
  if (std::getenv("TSAN_OPTIONS") || std::getenv("ASAN_OPTIONS")) {
    launcher.env("TSAN_OPTIONS",
      "halt_on_error=0:abort_on_error=0:second_deadlock_stack=1:history_size=7");
    launcher.env("ASAN_OPTIONS", "halt_on_error=0:abort_on_error=0");
  }
}

void prepareHelper(miniros::Launcher& launcher, const char* logLeaf)
{
  softenSanitizerForChild(launcher);
  launcher.redirectOutput(multimasterLogPath(logLeaf));
}

miniros::Error startMaster(int port, const std::vector<std::string>& peers, miniros::Launcher& launcher,
  const char* logLabel)
{
  // Multicast off: tests use --peer unicast probes (works without a multicast NIC).
  // Disable master cache so successive tests do not restore stale peer UUIDs.
  softenSanitizerForChild(launcher);
  launcher.redirectOutput(multimasterLogPath(std::string("master-") + logLabel + ".console.log"));
  const auto crashLog = multimasterLogPath(std::string("master-") + logLabel + ".crash").string();
  std::ofstream(crashLog, std::ios::trunc);
  launcher.env("MINIROS_CRASH_LOG", crashLog.c_str())
          .env("MINIROS_MASTER_LOG_DIR", multimasterLogDir().string().c_str());

  std::vector<std::string> args = {
    "-p", std::to_string(port),
    "--token", kToken,
    "--multicast", "off",
    "--rosout", "false",
    "--node_check_period", "0",
    "--no-cache",
    "--debugAPI",
  };
  for (const std::string& peer : peers) {
    args.push_back("--peer");
    args.push_back(peer);
  }
  const int flags = miniros::Launcher::FLAG_DETACHED | miniros::Launcher::FLAG_NOTIFY;
  if (miniros::Error err = launcher.start(executable("miniroscore"), args, flags); !err)
    return err;
  miniros::ChildReady ready;
  if (miniros::Error err = launcher.waitReady(&ready, miniros::WallDuration(8.0)); !err)
    return err;
  if (ready.rpcPort != 0 && ready.rpcPort != port)
    return miniros::Error::InvalidResponse;
  return miniros::Error::Ok;
}

int pairedCountFromStatus(const std::string& json)
{
  // Prefer explicit paired_count field from /api2/multimaster/status.
  const std::string key = "\"paired_count\":";
  const size_t pos = json.find(key);
  if (pos == std::string::npos)
    return -1;
  return std::atoi(json.c_str() + pos + key.size());
}

std::string multimasterStatus(int masterPort)
{
  return httpGet("http://127.0.0.1:" + std::to_string(masterPort) + "/api2/multimaster/status");
}

bool waitPairedCount(int masterPort, int expected, std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pairedCountFromStatus(multimasterStatus(masterPort)) >= expected)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return false;
}

std::string publishedTopicsJson(int masterPort)
{
  return httpGet("http://127.0.0.1:" + std::to_string(masterPort) + "/api2/published_topics");
}

/// Wait until `topic` appears in /api2/published_topics on the given master
/// (used to confirm multimaster sync of a foreign publisher under TSan).
bool waitPublishedTopic(int masterPort, const std::string& topic, std::chrono::seconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (publishedTopicsJson(masterPort).find(topic) != std::string::npos)
      return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
  return false;
}

std::chrono::seconds tsanAwareSeconds(int base)
{
  // TSan is imprinted into libroscxx; query the linked binary rather than env/compiler macros.
  if (miniros::BuildConfig::useTSan())
    return std::chrono::seconds(base * 3);
  return std::chrono::seconds(base);
}

std::string timeoutArg(int baseSeconds)
{
  return "_timeout:=" + std::to_string(tsanAwareSeconds(baseSeconds).count());
}

std::vector<std::string> peerUuidsFromStatus(const std::string& json)
{
  std::vector<std::string> out;
  const std::string key = "\"uuid\":\"";
  for (size_t pos = 0; (pos = json.find(key, pos)) != std::string::npos; ) {
    pos += key.size();
    const size_t end = json.find('"', pos);
    if (end == std::string::npos)
      break;
    out.push_back(json.substr(pos, end - pos));
    pos = end + 1;
  }
  return out;
}

void manualPair(int fromPort, const std::string& peerUuid)
{
  const std::string url = "http://127.0.0.1:" + std::to_string(fromPort) +
    "/api2/multimaster/connect?uuid=" + peerUuid;
  (void)httpGet(url);
}

/// True if nothing is listening on TCP `port` (try exclusive bind).
bool portFree(int port)
{
  miniros::network::NetSocket sock;
  if (miniros::Error err = sock.tcpSocket(miniros::network::NetAddress::AddressIPv4); !err)
    return false;
  // Do not set reuse — we want bind to fail if the port is taken.
  return static_cast<bool>(sock.bind(port));
}

void ensurePortsFree()
{
  for (int port : {kMasterPortA, kMasterPortB, kMasterPortC}) {
    if (portFree(port))
      continue;
    debugShutdown(port);
  }
  for (int attempt = 0; attempt < 50; ++attempt) {
    bool allFree = true;
    for (int port : {kMasterPortA, kMasterPortB, kMasterPortC}) {
      if (!portFree(port)) {
        allFree = false;
        debugShutdown(port);
      }
    }
    if (allFree)
      return;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace

class MultimasterIntegrationTest : public ::testing::Test {
protected:
  miniros::Launcher masterA_;
  miniros::Launcher masterB_;
  miniros::Launcher responder_;
  miniros::Launcher requester_;

  void SetUp() override
  {
    ASSERT_TRUE(std::filesystem::exists(executable("miniroscore")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_requester")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_responder")));
    ensurePortsFree();

    const std::string peerB = "127.0.0.1:" + std::to_string(kMasterPortB);
    const std::string peerA = "127.0.0.1:" + std::to_string(kMasterPortA);

    ASSERT_EQ(startMaster(kMasterPortA, {peerB}, masterA_, "A"), miniros::Error::Ok);
    ASSERT_EQ(startMaster(kMasterPortB, {peerA}, masterB_, "B"), miniros::Error::Ok);
    ASSERT_TRUE(waitPairedCount(kMasterPortA, 1, tsanAwareSeconds(8)))
      << "A failed to pair with B\n" << multimasterStatus(kMasterPortA);
  }

  void TearDown() override
  {
    if (HasFailure()) {
      std::cerr << "MultimasterIntegrationTest TearDown: A_running=" << masterA_.running()
                << " B_running=" << masterB_.running()
                << " responder_running=" << responder_.running()
                << " requester_running=" << requester_.running() << std::endl;
      dumpMultimasterLogs("integration-test-failed");
    }
    stopProcess(requester_);
    stopProcess(responder_);
    stopMaster(masterA_, kMasterPortA);
    stopMaster(masterB_, kMasterPortB);
  }
};

TEST_F(MultimasterIntegrationTest, RequesterOnMasterAReachesResponderOnMasterB)
{
  const std::string masterAUri = "http://127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterBUri = "http://127.0.0.1:" + std::to_string(kMasterPortB);
  const std::string nonce = "integration-test-nonce";

  prepareHelper(responder_, "responder.console.log");
  ASSERT_EQ(responder_.env("ROS_MASTER_URI", masterBUri.c_str())
              .start(executable("mm_responder"), {timeoutArg(40)}, miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);

  // Confirm local advertise on B first — empty B topics means the helper never registered.
  ASSERT_TRUE(waitPublishedTopic(kMasterPortB, "/mm/response", tsanAwareSeconds(15)))
      << "Responder never advertised /mm/response on B\n"
      << "responder_running=" << responder_.running() << "\n"
      << "B status: " << multimasterStatus(kMasterPortB) << "\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);
  // Wait until the foreign /mm/response publisher has been synced to A before
  // starting the requester (under TSan a fixed 500ms sleep is not enough).
  ASSERT_TRUE(waitPublishedTopic(kMasterPortA, "/mm/response", tsanAwareSeconds(15)))
      << "Master A never learned foreign publisher /mm/response\n"
      << "responder_running=" << responder_.running() << "\n"
      << "A status: " << multimasterStatus(kMasterPortA) << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);

  prepareHelper(requester_, "requester.console.log");
  ASSERT_EQ(requester_.env("ROS_MASTER_URI", masterAUri.c_str())
              .start(executable("mm_requester"),
                {"_nonce:=" + nonce, timeoutArg(30)},
                0),
    miniros::Error::Ok);

  EXPECT_EQ(requester_.waitExit(), 0) << "mm_requester failed to get cross-master response";
  stopProcess(responder_);
}

/// Master A + requester first; peer master B and responder arrive later.
class MultimasterLatePeerTest : public ::testing::Test {
protected:
  miniros::Launcher masterA_;
  miniros::Launcher masterB_;
  miniros::Launcher responder_;
  miniros::Launcher requester_;

  void SetUp() override
  {
    ASSERT_TRUE(std::filesystem::exists(executable("miniroscore")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_requester")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_responder")));
    ensurePortsFree();
  }

  void TearDown() override
  {
    if (HasFailure()) {
      std::cerr << "MultimasterLatePeerTest TearDown: A_running=" << masterA_.running()
                << " B_running=" << masterB_.running()
                << " responder_running=" << responder_.running()
                << " requester_running=" << requester_.running() << std::endl;
      dumpMultimasterLogs("late-peer-test-failed");
    }
    stopProcess(requester_);
    stopProcess(responder_);
    stopMaster(masterA_, kMasterPortA);
    stopMaster(masterB_, kMasterPortB);
  }
};

TEST_F(MultimasterLatePeerTest, MasterAThenRequesterThenMasterBThenResponder)
{
  const std::string peerB = "127.0.0.1:" + std::to_string(kMasterPortB);
  const std::string peerA = "127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterAUri = "http://127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterBUri = "http://127.0.0.1:" + std::to_string(kMasterPortB);
  const std::string nonce = "late-peer-b-nonce";

  // 1. Master A
  ASSERT_EQ(startMaster(kMasterPortA, {peerB}, masterA_, "A"), miniros::Error::Ok);

  // 2. Requester on A (before B exists — republishes until a response arrives)
  const auto exchangeTimeout = tsanAwareSeconds(45);
  prepareHelper(requester_, "requester.console.log");
  ASSERT_EQ(requester_.env("ROS_MASTER_URI", masterAUri.c_str())
              .start(executable("mm_requester"),
                {"_nonce:=" + nonce, timeoutArg(45)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);
  ASSERT_TRUE(waitPublishedTopic(kMasterPortA, "/mm/request", tsanAwareSeconds(10)))
      << "Requester never advertised /mm/request on A\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA);

  // 3. Master B
  ASSERT_EQ(startMaster(kMasterPortB, {peerA}, masterB_, "B"), miniros::Error::Ok);
  ASSERT_TRUE(waitPairedCount(kMasterPortA, 1, tsanAwareSeconds(15)))
      << "A failed to pair with late B\n" << multimasterStatus(kMasterPortA);

  // 4. Responder on B. Requester is already publishing; success is the exchange.
  prepareHelper(responder_, "responder.console.log");
  ASSERT_EQ(responder_.env("ROS_MASTER_URI", masterBUri.c_str())
              .start(executable("mm_responder"), {timeoutArg(40)}, miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);

  ASSERT_TRUE(waitPublishedTopic(kMasterPortB, "/mm/response", tsanAwareSeconds(15)))
      << "Responder never advertised /mm/response on B\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);
  ASSERT_TRUE(waitPublishedTopic(kMasterPortA, "/mm/response", tsanAwareSeconds(15)))
      << "Foreign /mm/response never synced to A\n"
      << "A status: " << multimasterStatus(kMasterPortA) << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);

  int requesterRc = -1;
  ASSERT_EQ(requester_.waitExit(miniros::WallDuration(static_cast<double>(exchangeTimeout.count())),
              &requesterRc),
    miniros::Error::Ok)
      << "requester did not exit within " << exchangeTimeout.count() << "s\n"
      << "A status: " << multimasterStatus(kMasterPortA) << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);
  EXPECT_EQ(requesterRc, 0)
      << "requester never got ack after late peer B\n"
      << "A status: " << multimasterStatus(kMasterPortA) << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "B topics: " << publishedTopicsJson(kMasterPortB);
  stopProcess(responder_);
}

class MultimasterThreeMasterTest : public ::testing::Test {
protected:
  miniros::Launcher masterA_;
  miniros::Launcher masterB_;
  miniros::Launcher masterC_;
  miniros::Launcher publisher_;
  miniros::Launcher subscriber_;

  void SetUp() override
  {
    ASSERT_TRUE(std::filesystem::exists(executable("miniroscore")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_publisher")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_subscriber")));
    ensurePortsFree();

    const std::string peerA = "127.0.0.1:" + std::to_string(kMasterPortA);
    const std::string peerB = "127.0.0.1:" + std::to_string(kMasterPortB);
    const std::string peerC = "127.0.0.1:" + std::to_string(kMasterPortC);

    ASSERT_EQ(startMaster(kMasterPortA, {peerB, peerC}, masterA_, "A"), miniros::Error::Ok);
    ASSERT_EQ(startMaster(kMasterPortB, {peerA, peerC}, masterB_, "B"), miniros::Error::Ok);
    ASSERT_EQ(startMaster(kMasterPortC, {peerA, peerB}, masterC_, "C"), miniros::Error::Ok);

    std::this_thread::sleep_for(std::chrono::seconds(2));
  }

  void TearDown() override
  {
    if (HasFailure()) {
      std::cerr << "MultimasterThreeMasterTest TearDown: A=" << masterA_.running()
                << " B=" << masterB_.running() << " C=" << masterC_.running()
                << " pub=" << publisher_.running() << " sub=" << subscriber_.running() << std::endl;
      dumpMultimasterLogs("three-master-test-failed");
    }
    stopProcess(subscriber_);
    stopProcess(publisher_);
    stopMaster(masterA_, kMasterPortA);
    stopMaster(masterB_, kMasterPortB);
    stopMaster(masterC_, kMasterPortC);
  }
};

TEST_F(MultimasterThreeMasterTest, SubscriberOnAHearsPublisherOnC)
{
  // Manual pair B→A and B→C (collective token already set).
  std::string uuidA;
  std::string uuidC;
  for (int i = 0; i < 20 && (uuidA.empty() || uuidC.empty()); ++i) {
    const auto uuids = peerUuidsFromStatus(multimasterStatus(kMasterPortB));
    if (uuids.size() >= 2) {
      uuidA = uuids[0];
      uuidC = uuids[1];
      break;
    }
    if (pairedCountFromStatus(multimasterStatus(kMasterPortB)) >= 2)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }

  if (!uuidA.empty())
    manualPair(kMasterPortB, uuidA);
  if (!uuidC.empty())
    manualPair(kMasterPortB, uuidC);

  ASSERT_TRUE(waitPairedCount(kMasterPortA, 2, std::chrono::seconds(10)))
    << "Master A should be paired with B and C (full mesh)\n"
    << multimasterStatus(kMasterPortA);
  ASSERT_TRUE(waitPairedCount(kMasterPortC, 2, std::chrono::seconds(5)))
    << "Master C should be paired with A and B\n"
    << multimasterStatus(kMasterPortC);

  const std::string masterAUri = "http://127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterCUri = "http://127.0.0.1:" + std::to_string(kMasterPortC);
  const std::string payload = "abc-mesh-payload";

  prepareHelper(publisher_, "publisher.console.log");
  ASSERT_EQ(publisher_.env("ROS_MASTER_URI", masterCUri.c_str())
              .start(executable("mm_publisher"),
                {"_payload:=" + payload, timeoutArg(25)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);

  ASSERT_TRUE(waitPublishedTopic(kMasterPortA, "/mm/chatter", tsanAwareSeconds(15)))
      << "Master A never learned foreign /mm/chatter\n"
      << "publisher_running=" << publisher_.running() << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "C topics: " << publishedTopicsJson(kMasterPortC);

  prepareHelper(subscriber_, "subscriber.console.log");
  ASSERT_EQ(subscriber_.env("ROS_MASTER_URI", masterAUri.c_str())
              .start(executable("mm_subscriber"),
                {"_payload:=" + payload, timeoutArg(15)},
                0),
    miniros::Error::Ok);

  EXPECT_EQ(subscriber_.waitExit(), 0) << "mm_subscriber on A failed to hear publisher on C";
  stopProcess(publisher_);
}

/// A then subscriber, then late C then publisher (B already in the mesh).
class MultimasterLateEdgeMasterTest : public ::testing::Test {
protected:
  miniros::Launcher masterA_;
  miniros::Launcher masterB_;
  miniros::Launcher masterC_;
  miniros::Launcher publisher_;
  miniros::Launcher subscriber_;

  void SetUp() override
  {
    ASSERT_TRUE(std::filesystem::exists(executable("miniroscore")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_publisher")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_subscriber")));
    ensurePortsFree();
  }

  void TearDown() override
  {
    if (HasFailure()) {
      std::cerr << "MultimasterLateEdgeMasterTest TearDown: A=" << masterA_.running()
                << " B=" << masterB_.running() << " C=" << masterC_.running()
                << " pub=" << publisher_.running() << " sub=" << subscriber_.running() << std::endl;
      dumpMultimasterLogs("late-edge-test-failed");
    }
    stopProcess(subscriber_);
    stopProcess(publisher_);
    stopMaster(masterA_, kMasterPortA);
    stopMaster(masterB_, kMasterPortB);
    stopMaster(masterC_, kMasterPortC);
  }
};

TEST_F(MultimasterLateEdgeMasterTest, MasterAThenSubscriberThenMasterCThenPublisher)
{
  const std::string peerA = "127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string peerB = "127.0.0.1:" + std::to_string(kMasterPortB);
  const std::string peerC = "127.0.0.1:" + std::to_string(kMasterPortC);
  const std::string masterAUri = "http://127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterCUri = "http://127.0.0.1:" + std::to_string(kMasterPortC);
  const std::string payload = "late-edge-c-payload";

  // 1. Masters A + B (mesh backbone); C arrives later.
  ASSERT_EQ(startMaster(kMasterPortA, {peerB, peerC}, masterA_, "A"), miniros::Error::Ok);
  ASSERT_EQ(startMaster(kMasterPortB, {peerA, peerC}, masterB_, "B"), miniros::Error::Ok);
  ASSERT_TRUE(waitPairedCount(kMasterPortA, 1, tsanAwareSeconds(10)))
      << "A should pair with B\n" << multimasterStatus(kMasterPortA);

  // 2. Subscriber on A (before C / publisher exist)
  prepareHelper(subscriber_, "subscriber.console.log");
  ASSERT_EQ(subscriber_.env("ROS_MASTER_URI", masterAUri.c_str())
              .start(executable("mm_subscriber"),
                {"_payload:=" + payload, timeoutArg(45)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  ASSERT_TRUE(subscriber_.valid());

  // 3. Master C
  ASSERT_EQ(startMaster(kMasterPortC, {peerA, peerB}, masterC_, "C"), miniros::Error::Ok);
  ASSERT_TRUE(waitPairedCount(kMasterPortA, 2, tsanAwareSeconds(15)))
      << "A should pair with late C\n" << multimasterStatus(kMasterPortA);
  ASSERT_TRUE(waitPairedCount(kMasterPortC, 2, tsanAwareSeconds(10)))
      << "C should join full mesh\n" << multimasterStatus(kMasterPortC);

  // 4. Publisher on C
  prepareHelper(publisher_, "publisher.console.log");
  ASSERT_EQ(publisher_.env("ROS_MASTER_URI", masterCUri.c_str())
              .start(executable("mm_publisher"),
                {"_payload:=" + payload, timeoutArg(40)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);

  EXPECT_EQ(subscriber_.waitExit(), 0)
      << "subscriber never heard late publisher on C\n"
      << "A status: " << multimasterStatus(kMasterPortA) << "\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA) << "\n"
      << "C topics: " << publishedTopicsJson(kMasterPortC);
  stopProcess(publisher_);
}

/// Edge masters A and C first; middle master B arrives late and joins the mesh.
class MultimasterLateMiddleMasterTest : public ::testing::Test {
protected:
  miniros::Launcher masterA_;
  miniros::Launcher masterB_;
  miniros::Launcher masterC_;
  miniros::Launcher publisher_;
  miniros::Launcher subscriber_;

  void SetUp() override
  {
    ASSERT_TRUE(std::filesystem::exists(executable("miniroscore")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_publisher")));
    ASSERT_TRUE(std::filesystem::exists(executable("mm_subscriber")));
    ensurePortsFree();
  }

  void TearDown() override
  {
    if (HasFailure()) {
      std::cerr << "MultimasterLateMiddleMasterTest TearDown: A=" << masterA_.running()
                << " B=" << masterB_.running() << " C=" << masterC_.running()
                << " pub=" << publisher_.running() << " sub=" << subscriber_.running() << std::endl;
      dumpMultimasterLogs("late-middle-test-failed");
    }
    stopProcess(subscriber_);
    stopProcess(publisher_);
    stopMaster(masterA_, kMasterPortA);
    stopMaster(masterB_, kMasterPortB);
    stopMaster(masterC_, kMasterPortC);
  }
};

TEST_F(MultimasterLateMiddleMasterTest, MastersACThenNodesThenLateMasterB)
{
  const std::string peerA = "127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string peerB = "127.0.0.1:" + std::to_string(kMasterPortB);
  const std::string peerC = "127.0.0.1:" + std::to_string(kMasterPortC);
  const std::string masterAUri = "http://127.0.0.1:" + std::to_string(kMasterPortA);
  const std::string masterCUri = "http://127.0.0.1:" + std::to_string(kMasterPortC);
  const std::string payload = "late-middle-b-payload";

  // 1. Edge masters A and C (probe each other + future middle B).
  ASSERT_EQ(startMaster(kMasterPortA, {peerB, peerC}, masterA_, "A"), miniros::Error::Ok);
  ASSERT_EQ(startMaster(kMasterPortC, {peerA, peerB}, masterC_, "C"), miniros::Error::Ok);
  ASSERT_TRUE(waitPairedCount(kMasterPortA, 1, tsanAwareSeconds(15)))
      << "A should pair with C before middle B arrives\n" << multimasterStatus(kMasterPortA);

  // 2. Nodes on the edges
  prepareHelper(publisher_, "publisher.console.log");
  ASSERT_EQ(publisher_.env("ROS_MASTER_URI", masterCUri.c_str())
              .start(executable("mm_publisher"),
                {"_payload:=" + payload, timeoutArg(45)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);
  ASSERT_TRUE(waitPublishedTopic(kMasterPortA, "/mm/chatter", tsanAwareSeconds(15)))
      << "A never learned /mm/chatter from C before B\n"
      << "A topics: " << publishedTopicsJson(kMasterPortA);

  prepareHelper(subscriber_, "subscriber.console.log");
  ASSERT_EQ(subscriber_.env("ROS_MASTER_URI", masterAUri.c_str())
              .start(executable("mm_subscriber"),
                {"_payload:=" + payload, timeoutArg(45)},
                miniros::Launcher::FLAG_DETACHED),
    miniros::Error::Ok);

  // 3. Middle master B arrives late and joins the collective.
  ASSERT_EQ(startMaster(kMasterPortB, {peerA, peerC}, masterB_, "B"), miniros::Error::Ok);
  ASSERT_TRUE(waitPairedCount(kMasterPortB, 2, tsanAwareSeconds(15)))
      << "Late middle B should pair with A and C\n" << multimasterStatus(kMasterPortB);
  ASSERT_TRUE(waitPairedCount(kMasterPortA, 2, tsanAwareSeconds(10)))
      << "A should also see B after it arrives\n" << multimasterStatus(kMasterPortA);

  // 4. Edge traffic still completes (A↔C), and B is part of the mesh.
  EXPECT_EQ(subscriber_.waitExit(), 0) << "subscriber failed after late middle master B joined";
  EXPECT_GE(pairedCountFromStatus(multimasterStatus(kMasterPortC)), 2)
      << multimasterStatus(kMasterPortC);
  stopProcess(publisher_);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
