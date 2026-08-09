//
// Shared helper for C++ launchers that replay former rostest XML scenarios:
// set master params, then spawn advanced-test_remapping with the right args.
//

#ifndef MINIROS_TEST_LAUNCH_REMAPPING_H
#define MINIROS_TEST_LAUNCH_REMAPPING_H

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "miniros/ros.h"
#include "miniros/launcher.h"
#include "miniros/master_link.h"

#include "../../require_master.h"

namespace miniros {
namespace test {

inline std::filesystem::path testBinDir()
{
#ifdef MINIROS_TEST_BIN_DIR
  return std::filesystem::path(MINIROS_TEST_BIN_DIR);
#else
  return std::filesystem::current_path();
#endif
}

inline std::filesystem::path testRemappingBinary()
{
  auto p = testBinDir() / "advanced-test_remapping";
#if defined(WIN32)
  p += ".exe";
#endif
  return p;
}

/**
 * \brief Replay a former rostest remapping XML: set params, spawn the gtest child.
 *
 * \param launcher_name   Node name for this launcher (also used in requireMasterOrExit).
 * \param use_local_remap Value of /use_local_remap (as in the XML).
 * \param child_args      Argv for advanced-test_remapping: optional `from:=to` remaps,
 *                        then the two expected namespace strings.
 * \param remap_from      Optional /remap_from (local_remappings.xml).
 * \param remap_to        Optional /remap_to (local_remappings.xml).
 */
inline int launchTestRemapping(const char* launcher_name,
                               bool use_local_remap,
                               const std::vector<std::string>& child_args,
                               const char* remap_from = nullptr,
                               const char* remap_to = nullptr)
{
  miniros::init(miniros::M_string(), launcher_name);
  requireMasterOrExit(launcher_name);

  MasterLinkPtr master = getMasterLink();
  if (!master) {
    std::cerr << launcher_name << ": no MasterLink\n";
    return 1;
  }
  if (Error err = master->set("/use_local_remap", use_local_remap); !err) {
    std::cerr << launcher_name << ": failed to set /use_local_remap: " << err.toString() << "\n";
    return 1;
  }
  if (remap_from) {
    if (Error err = master->set("/remap_from", remap_from); !err) {
      std::cerr << launcher_name << ": failed to set /remap_from: " << err.toString() << "\n";
      return 1;
    }
  }
  if (remap_to) {
    if (Error err = master->set("/remap_to", remap_to); !err) {
      std::cerr << launcher_name << ": failed to set /remap_to: " << err.toString() << "\n";
      return 1;
    }
  }

  const std::filesystem::path child = testRemappingBinary();
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

#endif // MINIROS_TEST_LAUNCH_REMAPPING_H
