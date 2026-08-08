//
// Shared helper for tests that need a live external master.
// Call after miniros::init(). Prefer MasterLink::check() (XML-RPC getPid) —
// it is fast, does not wait_for_master, and is easy to spot next to the
// registerPublisher("/rosout") / registerSubscriber("/rosout") lines that
// appear when the first NodeHandle is created.
//

#ifndef MINIROS_TEST_REQUIRE_MASTER_H
#define MINIROS_TEST_REQUIRE_MASTER_H

#include <cstdlib>

#include <gtest/gtest.h>

#include "miniros/console.h"
#include "miniros/init.h"
#include "miniros/master_link.h"

namespace miniros {
namespace test {

/// Exit code for "external master is down / unreachable".
/// launch_test.sh treats this specially (dumps master stacks + rosout).
constexpr int EXIT_MASTER_UNAVAILABLE = 90;

/**
 * \brief Abort with EXIT_MASTER_UNAVAILABLE if the master does not answer getPid.
 *
 * Intended for use in main() right after miniros::init():
 * \code
 *   miniros::init(argc, argv, "my_test");
 *   miniros::test::requireMasterOrExit("my_test");
 *   return RUN_ALL_TESTS();
 * \endcode
 *
 * Creating a NodeHandle afterwards will also register /rosout with the master
 * (visible in miniroscore logs) if the link is healthy.
 */
inline void requireMasterOrExit(const char* context = nullptr)
{
  MasterLinkPtr link = getMasterLink();
  if (link && link->check()) {
    if (context && context[0]) {
      MINIROS_INFO("MASTER_CHECK_OK [%s]: getPid succeeded", context);
    } else {
      MINIROS_INFO("MASTER_CHECK_OK: getPid succeeded");
    }
    return;
  }

  if (context && context[0]) {
    MINIROS_FATAL("MASTER_UNAVAILABLE [%s]: MasterLink::check()/getPid failed "
                  "(is miniroscore running? ROS_MASTER_URI=%s)",
      context,
      std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "(default)");
  } else {
    MINIROS_FATAL("MASTER_UNAVAILABLE: MasterLink::check()/getPid failed "
                  "(is miniroscore running? ROS_MASTER_URI=%s)",
      std::getenv("ROS_MASTER_URI") ? std::getenv("ROS_MASTER_URI") : "(default)");
  }
  std::_Exit(EXIT_MASTER_UNAVAILABLE);
}

/**
 * \brief GTest assertion form — use in TEST / SetUp when failure should be a
 * failed assertion rather than process exit.
 */
inline ::testing::AssertionResult assertMasterAlive(const char* context = nullptr)
{
  MasterLinkPtr link = getMasterLink();
  if (link && link->check()) {
    if (context && context[0]) {
      MINIROS_INFO("MASTER_CHECK_OK [%s]: getPid succeeded", context);
    }
    return ::testing::AssertionSuccess();
  }
  std::string msg = "MASTER_UNAVAILABLE: MasterLink::check()/getPid failed";
  if (context && context[0]) {
    msg += " [";
    msg += context;
    msg += "]";
  }
  return ::testing::AssertionFailure() << msg;
}

} // namespace test
} // namespace miniros

#endif // MINIROS_TEST_REQUIRE_MASTER_H
