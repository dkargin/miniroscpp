//
// Created by dkargin on 8/31/25.
//

#include <csignal>
#include <iostream>
#include <filesystem>
#include <fstream>


#include "miniros/launcher.h"


using namespace miniros;
int main(int argc, char** argv)
{
  if (argc < 2) {
    std::cerr << "Usage: [cmd]" << std::endl;
    return EXIT_FAILURE;
  }

  constexpr char pidFile[] = "miniroscore.pid";
  constexpr char roscoreFile[] = "miniroscore";

  // Expecting that all executables are stored either in build_dir/bin or $CMAKE_INSTALL_PREFIX/bin.
  std::filesystem::path myPath(argv[0]);

  std::filesystem::path binDir = myPath.parent_path();

  std::filesystem::current_path(binDir);

  std::string command = argv[1];
  if (command == "start") {
    Launcher launcher;

    std::filesystem::path pidFilePath = std::filesystem::absolute(pidFile);

    std::vector<std::string> args = {
      std::string("--pidfile=") + pidFilePath.string()
    };

    Error err = launcher.start(roscoreFile, args, Launcher::FLAG_DETACHED);
    if (err != Error::Ok) {
      std::cerr << "Failed to start miniroscore: " << err.toString() << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "Started roscore pid=" << launcher.pid() << std::endl;
    return EXIT_SUCCESS;
  }
  if (command == "stop") {
    int pid = 0;
    {
      std::ifstream file(pidFile);
      if (!file.is_open()) {
        std::cerr << "Failed to open PID file \"" << pidFile << "\"" << std::endl;
        return EXIT_FAILURE;
      }
      file >> pid;
    }
    Launcher launcher(pid);
    launcher.signal(SIGINT);
    launcher.waitExit();
    std::ofstream file(pidFile);
    file << std::endl;
    std::cout << "Done" << std::endl;
    return EXIT_SUCCESS;
  }
  std::cerr << "Unknown command \"" << command << "\"" << std::endl;
  return EXIT_FAILURE;
}
