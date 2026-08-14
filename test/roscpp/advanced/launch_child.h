//
// Spawn a sibling test binary via miniros::Launcher (C++ stand-in for a rostest XML).
//

#ifndef MINIROS_TEST_LAUNCH_CHILD_H
#define MINIROS_TEST_LAUNCH_CHILD_H

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "miniros/ros.h"
#include "miniros/launcher.h"

#include "../../require_master.h"

namespace miniros {
namespace test {

inline std::filesystem::path launchChildBinDir()
{
#ifdef MINIROS_TEST_BIN_DIR
  return std::filesystem::path(MINIROS_TEST_BIN_DIR);
#else
  return std::filesystem::current_path();
#endif
}

inline std::filesystem::path launchChildBinary(const char* name)
{
  auto p = launchChildBinDir() / name;
#if defined(WIN32)
  p += ".exe";
#endif
  return p;
}

inline int launchTestChild(const char* launcher_name,
                           const char* child_name,
                           const std::vector<std::string>& child_args,
                           bool require_master = true)
{
  miniros::init(miniros::M_string(), launcher_name);
  if (require_master) {
    requireMasterOrExit(launcher_name);
  }

  const std::filesystem::path child = launchChildBinary(child_name);
  if (!std::filesystem::exists(child)) {
    std::cerr << launcher_name << ": missing " << child << "\n";
    return 1;
  }

  Launcher launcher;
  if (const char* uri = std::getenv("ROS_MASTER_URI"); uri && *uri) {
    launcher.env("ROS_MASTER_URI", uri);
  }

  if (Error err = launcher.start(child, child_args); !err) {
    std::cerr << launcher_name << ": failed to start " << child << ": " << err.toString() << "\n";
    return 1;
  }

  const int code = launcher.waitExit();
  miniros::shutdown();
  return code;
}

} // namespace test
} // namespace miniros

#endif // MINIROS_TEST_LAUNCH_CHILD_H
