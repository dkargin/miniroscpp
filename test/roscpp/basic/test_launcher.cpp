//
// Tests for Launcher notify / waitReady, start failures, crash, and kill behaviour.
//

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "miniros/common.h"
#include "miniros/launcher.h"

#if !defined(WIN32)
#include <unistd.h>
#endif

namespace {

std::filesystem::path notifyChildPath()
{
#ifdef MINIROS_NOTIFY_CHILD
  return std::filesystem::path(MINIROS_NOTIFY_CHILD);
#elif defined(MINIROS_TEST_BIN_DIR)
  std::filesystem::path p = std::filesystem::path(MINIROS_TEST_BIN_DIR) / "basic-notify_child";
#  if defined(WIN32)
  p += ".exe";
#  endif
  return p;
#else
  std::filesystem::path p = std::filesystem::current_path() / "basic-notify_child";
#  if defined(WIN32)
  p += ".exe";
#  endif
  return p;
#endif
}

void clearNotifyEnv()
{
#if defined(WIN32)
  _putenv("NOTIFY_SOCKET=");
  _putenv("MINIROS_NOTIFY_HANDLE=");
  _putenv("MINIROS_NOTIFY_FD=");
#else
  unsetenv("NOTIFY_SOCKET");
  unsetenv("MINIROS_NOTIFY_FD");
  unsetenv("MINIROS_NOTIFY_HANDLE");
#endif
}

} // namespace

TEST(Notify, SilentNoopWithoutChannel)
{
  clearNotifyEnv();
  miniros::NodeNotifyInfo info;
  info.rpcPort = 1234;
  info.uri = "http://x:1234";
  EXPECT_EQ(miniros::notifyNodeStarted(info), miniros::Error::Ok);
  EXPECT_EQ(miniros::notifyNodeExiting(), miniros::Error::Ok);
}

#if !defined(WIN32)
TEST(Notify, PipeDeliversPidAndPort)
{
  int fds[2] = {-1, -1};
  ASSERT_EQ(pipe(fds), 0);
  ASSERT_EQ(setenv("MINIROS_NOTIFY_FD", std::to_string(fds[1]).c_str(), 1), 0);
  unsetenv("NOTIFY_SOCKET");

  miniros::NodeNotifyInfo info;
  info.rpcPort = 4242;
  info.uri = "http://127.0.0.1:4242";
  ASSERT_EQ(miniros::notifyNodeStarted(info), miniros::Error::Ok);

  char buf[512] = {};
  const ssize_t n = read(fds[0], buf, sizeof(buf) - 1);
  close(fds[0]);
  close(fds[1]);
  unsetenv("MINIROS_NOTIFY_FD");

  ASSERT_GT(n, 0);
  const std::string msg(buf, static_cast<size_t>(n));
  EXPECT_NE(msg.find("READY=1"), std::string::npos);
  EXPECT_NE(msg.find("X_MINIROS_RPC_PORT=4242"), std::string::npos);
  EXPECT_NE(msg.find("MAINPID=" + std::to_string(miniros::Launcher::myPid())), std::string::npos);
}
#endif

TEST(Launcher, WaitReadyReceivesNotifyPayload)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath())) << notifyChildPath();

  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"ready", "5555", "http://127.0.0.1:5555"},
              miniros::Launcher::FLAG_NOTIFY),
    miniros::Error::Ok);

  miniros::ChildReady ready;
  ASSERT_EQ(launcher.waitReady(&ready, miniros::WallDuration(5.0)), miniros::Error::Ok);
  EXPECT_TRUE(ready.ready);
  EXPECT_EQ(ready.rpcPort, 5555);
  EXPECT_EQ(ready.uri, "http://127.0.0.1:5555");
  EXPECT_EQ(ready.pid, launcher.pid());

  EXPECT_EQ(launcher.waitExit(), 0);
}

TEST(Launcher, WaitReadyTimesOutWithoutNotify)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath()));
  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"silent"}, miniros::Launcher::FLAG_NOTIFY), miniros::Error::Ok);
  EXPECT_EQ(launcher.waitReady(nullptr, miniros::WallDuration(0.3)), miniros::Error::Timeout);
  (void)launcher.waitExit();
}

TEST(Launcher, StartFailsWhenFileMissing)
{
  miniros::Launcher launcher;
  EXPECT_EQ(launcher.start(notifyChildPath().parent_path() / "no_such_launcher_binary_xyz", {}, 0),
    miniros::Error::FileNotFound);
  EXPECT_FALSE(launcher.valid());
}

TEST(Launcher, StartFailsWithoutExecutePermission)
{
  const auto path = notifyChildPath().parent_path() / "launcher_noexec_probe";
  {
    std::ofstream out(path);
    ASSERT_TRUE(out) << path;
    out << "#!/bin/sh\necho should-not-run\n";
  }
  std::error_code ec;
  std::filesystem::permissions(path,
    std::filesystem::perms::owner_read | std::filesystem::perms::owner_write,
    std::filesystem::perm_options::replace, ec);
  ASSERT_FALSE(ec) << ec.message();

  miniros::Launcher launcher;
  const miniros::Error err = launcher.start(path, {}, 0);
  EXPECT_TRUE(err == miniros::Error::PermissionDenied || err == miniros::Error::SystemError ||
    err == miniros::Error::InvalidValue)
    << err.toString();
  EXPECT_FALSE(launcher.valid());
  std::filesystem::remove(path, ec);
}

TEST(Launcher, ChildSegfaultIsObserved)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath()));
  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"crash"}, 0), miniros::Error::Ok);
  EXPECT_NE(launcher.waitExit(), 0);
  EXPECT_FALSE(launcher.valid());
}

TEST(Launcher, ChildUncaughtExceptionIsObserved)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath()));
  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"throw"}, 0), miniros::Error::Ok);
  EXPECT_NE(launcher.waitExit(), 0);
  EXPECT_FALSE(launcher.valid());
}

TEST(Launcher, StopEndsCooperativeLoop)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath()));
  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"loop"}, miniros::Launcher::FLAG_NOTIFY), miniros::Error::Ok);
  ASSERT_EQ(launcher.waitReady(nullptr, miniros::WallDuration(5.0)), miniros::Error::Ok);

  const miniros::Error err = launcher.stop();
  if (err == miniros::Error::PermissionDenied) {
    GTEST_SKIP() << "process signal not permitted in this environment";
  }
  ASSERT_EQ(err, miniros::Error::Ok) << err.toString();
  EXPECT_EQ(launcher.waitExit(), 0);
  EXPECT_FALSE(launcher.valid());
}

TEST(Launcher, TerminateKillsSignalIgnoringLoop)
{
  ASSERT_TRUE(std::filesystem::exists(notifyChildPath()));
  miniros::Launcher launcher;
  ASSERT_EQ(launcher.start(notifyChildPath(), {"ignore"}, miniros::Launcher::FLAG_NOTIFY), miniros::Error::Ok);
  ASSERT_EQ(launcher.waitReady(nullptr, miniros::WallDuration(5.0)), miniros::Error::Ok);

  // Cooperative stop must not end a child that ignores soft interrupts.
  const miniros::Error stopErr = launcher.stop();
  if (stopErr == miniros::Error::PermissionDenied) {
    GTEST_SKIP() << "process signal not permitted in this environment";
  }
  EXPECT_EQ(stopErr, miniros::Error::Ok);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_TRUE(launcher.valid());

  const miniros::Error termErr = launcher.terminate();
  if (termErr == miniros::Error::PermissionDenied) {
    GTEST_SKIP() << "process terminate not permitted in this environment";
  }
  ASSERT_EQ(termErr, miniros::Error::Ok) << termErr.toString();
  EXPECT_NE(launcher.waitExit(), 0);
  EXPECT_FALSE(launcher.valid());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
