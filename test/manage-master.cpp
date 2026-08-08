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

    std::vector<std::string> args = {
      std::string("--pidfile=") + pidFilePath.string(),
      std::string("--rosout=true"),
    };

    if (envTruthy("MINIROS_MASTER_DEBUG")) {
      args.emplace_back("--xmlrpc_log=4");
      const std::string consoleLog = (logDir / "miniroscore.console.log").string();
      launcher.env("MINIROS_MASTER_CONSOLE_LOG", consoleLog.c_str());
      std::cout << "MINIROS_MASTER_DEBUG: console -> " << consoleLog << std::endl;
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
