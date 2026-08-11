//
// Master cache restore: publisher survives a master restart; a later subscriber
// still discovers it via the restored graph.
//
// 1. Start miniroscore with --dir (isolated cache.<port>).
// 2. Start a latching publisher; wait until cache is on disk.
// 3. Stop master (SIGINT → flush); publisher keeps running.
// 4. Restart master from the same cache dir/port.
// 5. Subscribe and receive the latched message from the surviving publisher.
//

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "miniros/launcher.h"
#include "miniros/platform.h"
#include "miniros/ros.h"
#include "std_msgs/String.hxx"

namespace {

constexpr int kMasterPort = 11721;
constexpr const char* kTopic = "cache_test_chatter";
constexpr const char* kPayload = "hello-from-cached-publisher";
constexpr const char* kPubNode = "cache_test_pub";

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

std::filesystem::path cacheDir()
{
  return binDir() / "master-cache-test";
}

std::filesystem::path cacheFile(int port)
{
  return cacheDir() / ("cache." + std::to_string(port));
}

void resetCacheDir()
{
  const auto dir = cacheDir();
  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
}

bool waitForCacheWithNode(const std::filesystem::path& path, const std::string& nodeName,
                          std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::ifstream in(path);
    if (in) {
      std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
      if (contents.find(nodeName) != std::string::npos)
        return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return false;
}

miniros::Error startMaster(miniros::Launcher& launcher, int port)
{
  // Same sanitizer policy as manage-master: keep reporting races, do not kill
  // the private master on the first TSan warning.
  if (std::getenv("TSAN_OPTIONS") || std::getenv("ASAN_OPTIONS")) {
    launcher.env("TSAN_OPTIONS",
                 "halt_on_error=0:abort_on_error=0:second_deadlock_stack=1:history_size=7");
    launcher.env("ASAN_OPTIONS", "halt_on_error=0:abort_on_error=0");
  }

  const auto logDir = cacheDir() / "logs";
  std::error_code ec;
  std::filesystem::create_directories(logDir, ec);
  const auto crashLog = (logDir / "miniroscore.crash").string();
  const auto consoleLog = (logDir / "miniroscore.console.log").string();
  // Stable advertised host so cache URI matches post-restart re-registrations.
  launcher.env("ROS_HOSTNAME", "127.0.0.1")
          .env("MINIROS_MASTER_LOG_DIR", logDir.string().c_str())
          .env("MINIROS_CRASH_LOG", crashLog.c_str())
          .env("MINIROS_MASTER_CONSOLE_LOG", consoleLog.c_str());

  const std::vector<std::string> args = {
    "-p", std::to_string(port),
    "--dir", cacheDir().string(),
    "--rosout", "false",
    "--node_check_period", "0",
  };
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

} // namespace

class MasterCacheTest : public ::testing::Test {
protected:
  void TearDown() override
  {
    if (miniros::isInitialized()) {
      miniros::shutdown();
    }
    pub_.stopAndWait(miniros::WallDuration(5.0));
    master2_.stopAndWait(miniros::WallDuration(5.0));
    master_.stopAndWait(miniros::WallDuration(5.0));
  }

  miniros::Launcher master_;
  miniros::Launcher master2_;
  miniros::Launcher pub_;
};

TEST_F(MasterCacheTest, PublisherSurvivesMasterRestartFromCache)
{
  resetCacheDir();
  const std::string masterUri = "http://127.0.0.1:" + std::to_string(kMasterPort);

  ASSERT_EQ(startMaster(master_, kMasterPort), miniros::Error::Ok);

  ASSERT_EQ(pub_.env("ROS_MASTER_URI", masterUri.c_str())
              .env("ROS_HOSTNAME", "127.0.0.1")
              .start(executable("basic-test_master_cache_pub"), {},
                     miniros::Launcher::FLAG_NOTIFY),
            miniros::Error::Ok);

  miniros::ChildReady pubReady;
  ASSERT_EQ(pub_.waitReady(&pubReady, miniros::WallDuration(10.0)), miniros::Error::Ok)
      << "publisher should finish advertise/notify while the first master is up";
  ASSERT_TRUE(pub_.running());

  // Wait until the publisher is actually present in the on-disk snapshot.
  ASSERT_TRUE(waitForCacheWithNode(cacheFile(kMasterPort), kPubNode, std::chrono::seconds(5)))
      << "expected " << kPubNode << " in " << cacheFile(kMasterPort);

  // Clean stop so MasterCache::flush writes the latest snapshot.
  master_.stopAndWait(miniros::WallDuration(5.0));
  ASSERT_FALSE(master_.running());
  ASSERT_TRUE(pub_.running()) << "publisher must survive master restart";
  ASSERT_TRUE(waitForCacheWithNode(cacheFile(kMasterPort), kPubNode, std::chrono::seconds(1)))
      << "cache must still contain publisher after clean master stop";

  // Restart master from the same cache dir/port (new Launcher — not assignable).
  ASSERT_EQ(startMaster(master2_, kMasterPort), miniros::Error::Ok);
  ASSERT_TRUE(pub_.running()) << "publisher must still be alive after master restart";

  // Give MasterCache time to finish getPid/getPublications before attaching a
  // subscriber (restore is asynchronous on the master update loop).
  miniros::WallDuration(0.5).sleep();

  // Subscriber side in this process: discover the restored publisher and take
  // the latched message.
  ASSERT_TRUE(miniros::set_environment_variable("ROS_MASTER_URI", masterUri.c_str()));
  ASSERT_TRUE(miniros::set_environment_variable("ROS_HOSTNAME", "127.0.0.1"));
  int argc = 1;
  char arg0[] = "basic-test_master_cache";
  char* argv[] = {arg0, nullptr};
  miniros::init(argc, argv, "cache_test_sub",
                miniros::init_options::AnonymousName | miniros::init_options::NoRosout |
                    miniros::init_options::NoSimTime);

  // Wait until restore re-learns the publisher (getPublications → register).
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    bool sawTopic = false;
    while (std::chrono::steady_clock::now() < deadline) {
      std::vector<miniros::TopicInfo> topics;
      if (auto link = miniros::getMasterLink(); link && link->getTopics(topics)) {
        for (const auto& t : topics) {
          if (t.name == std::string("/") + kTopic || t.name == kTopic) {
            sawTopic = true;
            break;
          }
        }
      }
      if (sawTopic)
        break;
      miniros::WallDuration(0.1).sleep();
    }
    ASSERT_TRUE(sawTopic) << "restarted master should advertise topic " << kTopic
                          << " after cache restore of surviving publisher";
  }

  std::atomic<int> got{0};
  std::string last;
  miniros::NodeHandle nh;
  miniros::Subscriber sub = nh.subscribe<std_msgs::String>(
      kTopic, 1, [&](const std_msgs::String::ConstPtr& msg) {
        last = msg->data;
        got.fetch_add(1);
      });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
  while (got.load() == 0 && std::chrono::steady_clock::now() < deadline) {
    miniros::spinOnce();
    miniros::WallDuration(0.05).sleep();
  }

  ASSERT_GE(got.load(), 1) << "subscriber should receive latched message from restored publisher";
  EXPECT_EQ(last, kPayload);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
