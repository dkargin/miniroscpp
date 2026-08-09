//
// Created by dkargin on 8/31/25.
//
// Commands:
//   start  - launch detached miniroscore; wait until READY via notify
//   stop   - SIGINT the pid from miniroscore.pid
//
// Exit codes:
//   0  - success
//   1  - generic failure
//   90 - master did not become ready (MINIROS_EXIT_MASTER_UNAVAILABLE)
//
// Environment:
//   MINIROS_MASTER_LOG_DIR / ROS_LOG_DIR - log directory for rosout.log
//   MINIROS_MASTER_DEBUG=1              - --xmlrpc_log=4 + console log file
//   TSAN_OPTIONS / ASAN_OPTIONS         - if set in the parent, overridden for
//                                         the master child with halt_on_error=0
//                                         so a sanitizer hit does not kill the
//                                         shared process for the whole suite.
//

#include <csignal>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "miniros/launcher.h"
#include "miniros/duration.h"

using namespace miniros;

namespace {

constexpr int EXIT_MASTER_UNAVAILABLE = 90;
constexpr char pidFileName[] = "miniroscore.pid";
constexpr char roscoreFile[] = "miniroscore";

bool envTruthy(const char* name)
{
  const char* v = std::getenv(name);
  if (!v || !*v)
    return false;
  return std::strcmp(v, "1") == 0 || std::strcmp(v, "true") == 0 || std::strcmp(v, "TRUE") == 0 ||
         std::strcmp(v, "yes") == 0 || std::strcmp(v, "on") == 0;
}

std::filesystem::path resolveLogDir(const std::filesystem::path& binDir)
{
  if (const char* d = std::getenv("MINIROS_MASTER_LOG_DIR"); d && *d)
    return std::filesystem::absolute(d);
  if (const char* d = std::getenv("ROS_LOG_DIR"); d && *d)
    return std::filesystem::absolute(d);
  return std::filesystem::absolute(binDir / "master-logs");
}

} // namespace

int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: manage-master <start|stop>" << std::endl;
    return EXIT_FAILURE;
  }

  std::filesystem::path myPath(argv[0]);
  std::filesystem::path binDir = myPath.parent_path();
  std::filesystem::current_path(binDir);

  const std::string command = argv[1];
  const std::filesystem::path logDir = resolveLogDir(binDir);
  std::error_code ec;
  std::filesystem::create_directories(logDir, ec);

  if (command == "start") {
    Launcher launcher;
    const std::filesystem::path pidFilePath = std::filesystem::absolute(pidFileName);
    const std::string logDirStr = logDir.string();

    launcher.env("ROS_LOG_DIR", logDirStr.c_str());
    launcher.env("MINIROS_MASTER_LOG_DIR", logDirStr.c_str());

    // miniroscore is built with the same sanitizer flags as the tests. CI sets
    // TSAN_OPTIONS=halt_on_error=1 for the suite; if the shared master inherits
    // that and hits a race, TSan aborts it and every later test hangs on
    // wait_for_master. Keep reporting races, but do not kill the master.
    if (std::getenv("TSAN_OPTIONS") || std::getenv("ASAN_OPTIONS")) {
      launcher.env("TSAN_OPTIONS",
                   "halt_on_error=0:abort_on_error=0:second_deadlock_stack=1:history_size=7");
      launcher.env("ASAN_OPTIONS", "halt_on_error=0:abort_on_error=0");
      std::cout << "Sanitizer options for master: halt_on_error=0 (shared process)"
                << std::endl;
    }

    std::vector<std::string> args = {
      std::string("--pidfile=") + pidFilePath.string(),
      std::string("--rosout=true"),
    };

    // Always keep a console capture under the log dir so a crash still leaves
    // stderr (and handleCrashes() output) in the CI artifact tree.
    const std::string consoleLog = (logDir / "miniroscore.console.log").string();
    const std::string crashLog = (logDir / "miniroscore.crash").string();
    launcher.env("MINIROS_MASTER_CONSOLE_LOG", consoleLog.c_str());
    launcher.env("MINIROS_CRASH_LOG", crashLog.c_str());
    std::cout << "miniroscore console -> " << consoleLog << std::endl;
    std::cout << "miniroscore crash log -> " << crashLog << std::endl;

    if (envTruthy("MINIROS_MASTER_DEBUG")) {
      args.emplace_back("--xmlrpc_log=4");
      std::cout << "MINIROS_MASTER_DEBUG: xmlrpc_log=4" << std::endl;
    }

    // DETACHED: master survives after this process exits.
    // NOTIFY: wait until miniroscore calls notifyNodeStarted() (listening).
    Error err = launcher.start(roscoreFile, args, Launcher::FLAG_DETACHED | Launcher::FLAG_NOTIFY);
    if (err != Error::Ok) {
      std::cerr << "Failed to start miniroscore: " << err.toString() << std::endl;
      return EXIT_FAILURE;
    }

    ChildReady ready;
    err = launcher.waitReady(&ready, WallDuration(2.0));
    if (err != Error::Ok) {
      std::cerr << "MASTER_UNAVAILABLE: miniroscore did not become ready: " << err.toString()
                << " (exit " << EXIT_MASTER_UNAVAILABLE << ")" << std::endl;
      launcher.signal(SIGINT);
      launcher.waitExit();
      return EXIT_MASTER_UNAVAILABLE;
    }

    std::cout << "Started roscore pid=" << launcher.pid()
              << " uri=" << ready.uri
              << " logs=" << logDirStr << std::endl;
    return EXIT_SUCCESS;
  }

  if (command == "stop") {
    int pid = 0;
    {
      std::ifstream file(pidFileName);
      if (!file.is_open()) {
        std::cerr << "Failed to open PID file \"" << pidFileName << "\"" << std::endl;
        return EXIT_FAILURE;
      }
      file >> pid;
    }
    if (pid <= 0) {
      std::cerr << "No valid pid in " << pidFileName << std::endl;
      return EXIT_FAILURE;
    }
    Launcher launcher(pid);
    launcher.signal(SIGINT);
    launcher.waitExit();
    std::ofstream file(pidFileName);
    file << std::endl;
    std::cout << "Done" << std::endl;
    return EXIT_SUCCESS;
  }

  std::cerr << "Unknown command \"" << command << "\"" << std::endl;
  return EXIT_FAILURE;
}
