//
// Start a node before the master, then bring the master up and verify the node
// connects (notify) instead of hanging forever.
//

#include <chrono>
#include <filesystem>
#include <string>

#include <gtest/gtest.h>

#include "miniros/launcher.h"

namespace {

constexpr int kMasterPort = 11711;

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

miniros::Error startMaster(int port, miniros::Launcher& launcher)
{
  const std::vector<std::string> args = {
    "-p", std::to_string(port),
    "--rosout", "false",
    "--node_check_period", "0",
  };
  // Master can be detached; we only need NOTIFY for readiness.
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

class LateMasterTest : public ::testing::Test {
protected:
  void TearDown() override
  {
    node_.stopAndWait(miniros::WallDuration(5.0));
    master_.stopAndWait(miniros::WallDuration(5.0));
  }

  miniros::Launcher master_;
  miniros::Launcher node_;
};

TEST_F(LateMasterTest, NodeStartsBeforeMasterThenConnects)
{
  const std::string masterUri = "http://127.0.0.1:" + std::to_string(kMasterPort);

  // Keep stdout/stderr (no FLAG_DETACHED) so child logs show up under gdb/CI.
  ASSERT_EQ(node_.env("ROS_MASTER_URI", masterUri.c_str())
              .start(executable("basic-test_late_master_node"), {},
                     miniros::Launcher::FLAG_NOTIFY),
            miniros::Error::Ok);

  // NodeHandle/start blocks in registerService(wait_for_master) until master exists.
  // It must not notify yet — there is no master.
  EXPECT_EQ(node_.waitReady(nullptr, miniros::WallDuration(2.0)), miniros::Error::Timeout);
  ASSERT_TRUE(node_.running()) << "basic-test_late_master_node exited before master started";

  ASSERT_EQ(startMaster(kMasterPort, master_), miniros::Error::Ok);

  miniros::ChildReady ready;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
  miniros::Error waitErr = miniros::Error::Timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    waitErr = node_.waitReady(&ready, miniros::WallDuration(1.0));
    if (waitErr == miniros::Error::Ok)
      break;
    if (!node_.running()) {
      FAIL() << "basic-test_late_master_node exited without notifying (still waiting for master connect)";
    }
  }
  ASSERT_EQ(waitErr, miniros::Error::Ok)
      << "Node should finish init/advertise once the master appears on " << masterUri;
  EXPECT_TRUE(ready.ready);
  EXPECT_GT(ready.pid, 0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
