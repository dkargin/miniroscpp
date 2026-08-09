//
// Start a node before the master, then bring the master up and verify the node
// connects (notify) instead of hanging forever.
//

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
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

std::filesystem::path lateMasterLogDir()
{
  return binDir() / "late-master-logs";
}

/// Print crash/console excerpts to stderr so they appear in the CI job log.
void dumpLateMasterLogs(const char* reason)
{
  const auto logDir = lateMasterLogDir();
  std::cerr << "=== LATE MASTER DIAGNOSTICS (" << reason << ") dir=" << logDir << " ===\n";

  const auto printFile = [](const std::filesystem::path& path, bool whole_if_small) {
    std::error_code ec;
    if (!std::filesystem::is_regular_file(path, ec)) {
      std::cerr << "=== missing " << path << " ===\n";
      return;
    }
    const auto sz = std::filesystem::file_size(path, ec);
    if (ec || sz == 0) {
      std::cerr << "=== empty " << path << " ===\n";
      return;
    }
    std::cerr << "=== " << path.filename().string() << " (" << sz << " bytes) ===\n";
    std::ifstream in(path);
    if (!in) {
      std::cerr << "(failed to open)\n";
      return;
    }
    // Crash files are short; console may include TSan stacks — print up to ~64KiB from the end.
    constexpr std::uintmax_t kMax = 64 * 1024;
    if (whole_if_small || sz <= kMax) {
      std::cerr << in.rdbuf();
    } else {
      in.seekg(static_cast<std::streamoff>(sz - kMax));
      std::string line;
      std::getline(in, line); // discard partial first line
      std::cerr << in.rdbuf();
    }
    std::cerr << "\n=== end " << path.filename().string() << " ===\n";
  };

  printFile(logDir / "miniroscore.crash", true);
  printFile(logDir / "miniroscore.console.log", false);
}

miniros::Error startMaster(int port, miniros::Launcher& launcher)
{
  // Point crash/console dumps at the test bin dir so CI artifacts / job logs can
  // show stacks if this private master dies (not the shared advanced-suite master).
  const auto logDir = lateMasterLogDir();
  std::error_code ec;
  std::filesystem::create_directories(logDir, ec);
  const auto crashLog = (logDir / "miniroscore.crash").string();
  const auto consoleLog = (logDir / "miniroscore.console.log").string();
  // Truncate previous run so failure dumps are not stale.
  std::ofstream(crashLog, std::ios::trunc);
  std::ofstream(consoleLog, std::ios::trunc);
  launcher.env("MINIROS_MASTER_LOG_DIR", logDir.string().c_str())
          .env("MINIROS_CRASH_LOG", crashLog.c_str())
          .env("MINIROS_MASTER_CONSOLE_LOG", consoleLog.c_str());

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
    if (HasFailure()) {
      const bool masterAlive = master_.running();
      std::cerr << "late_master TearDown: master_running=" << masterAlive
                << " node_running=" << node_.running() << std::endl;
      dumpLateMasterLogs(masterAlive ? "test-failed-master-still-up" : "test-failed-master-dead");
    }
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
    if (!master_.running()) {
      dumpLateMasterLogs("master-exited-during-wait");
      FAIL() << "miniroscore exited while waiting for node to connect on " << masterUri;
    }
    waitErr = node_.waitReady(&ready, miniros::WallDuration(1.0));
    if (waitErr == miniros::Error::Ok)
      break;
    if (!node_.running()) {
      dumpLateMasterLogs("node-exited-during-wait");
      FAIL() << "basic-test_late_master_node exited without notifying (still waiting for master connect)";
    }
  }
  if (waitErr != miniros::Error::Ok) {
    dumpLateMasterLogs("node-never-connected");
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
