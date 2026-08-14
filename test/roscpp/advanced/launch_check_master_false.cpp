//
// C++ stand-in for former launch/check_master_false.xml:
//   <test ... type="advanced-check_master" args="no"/>
//
// The child points ROS_MASTER_URI at an invalid host; the launcher still
// probes the real master so CI fail-fast still applies to this wrapper.
//

#include "launch_child.h"

int main()
{
  return miniros::test::launchTestChild(
      "launch_check_master_false",
      "advanced-check_master",
      {"no"});
}
